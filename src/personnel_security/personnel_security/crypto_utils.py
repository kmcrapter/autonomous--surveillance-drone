"""Cryptographic Utilities for Secure Personnel Identification, Replay Protection, and Duplicate-Tag Detection."""

import hmac
import hashlib
import time
import secrets
from typing import Dict, Any, Tuple, Optional

# Simulated hardware security keys (pre-shared cryptographic seeds for authorized tags)
SIMULATED_TAG_KEYS: Dict[str, bytes] = {
    "TAG-UWB-101": b"k_alpha_cmd_vance_sec_89f7a21c",
    "TAG-NFC-204": b"k_logistics_mitchell_sec_410b9d3e",
    "TAG-BLE-309": b"k_sentry_kowalski_sec_77a4128f",
    "TAG-WEAR-412": b"k_border_rostova_sec_33c8291a",
    "TAG-BLE-515": b"k_sentry_miller_sec_66b5910d",
    # Cloned tag key shares the same master key to test cryptographic replay / duplicate detection
    "TAG-UWB-101-CLONE": b"k_alpha_cmd_vance_sec_89f7a21c"
}

def generate_tag_signature(tag_id: str, nonce: str, timestamp: float, coords: Tuple[float, float]) -> str:
    """Generates an HMAC-SHA256 signature for the tag payload using pre-shared hardware seed."""
    key = SIMULATED_TAG_KEYS.get(tag_id)
    if not key:
        return ""
    # Standard cryptographic challenge: HMAC(key, tag_id || nonce || timestamp || x || y)
    msg = f"{tag_id}:{nonce}:{timestamp:.2f}:{coords[0]:.1f}:{coords[1]:.1f}".encode('utf-8')
    return hmac.new(key, msg, hashlib.sha256).hexdigest()

def verify_tag_signature(tag_id: str, nonce: str, timestamp: float, coords: Tuple[float, float], signature: str) -> bool:
    """Verifies HMAC-SHA256 signature in constant time against timing attacks."""
    expected = generate_tag_signature(tag_id, nonce, timestamp, coords)
    if not expected or not signature:
        return False
    return hmac.compare_digest(expected, signature)

class NonceReplayCache:
    """Implements sliding-window replay protection and nonce deduplication."""

    def __init__(self, window_seconds: float = 6.0):
        self.window_seconds = window_seconds
        self.seen_nonces: Dict[str, float] = {}

    def cleanup(self, current_time: float):
        cutoff = current_time - self.window_seconds
        expired = [nonce for nonce, ts in self.seen_nonces.items() if ts < cutoff]
        for nonce in expired:
            del self.seen_nonces[nonce]

    def validate_and_record(self, nonce: str, timestamp: float, current_time: float) -> Tuple[bool, str]:
        self.cleanup(current_time)
        if abs(current_time - timestamp) > self.window_seconds:
            return False, f"EXPIRED_TIMESTAMP (delta={abs(current_time - timestamp):.2f}s > {self.window_seconds}s)"
        if nonce in self.seen_nonces:
            return False, f"REPLAY_DETECTED (nonce {nonce} already consumed)"
        self.seen_nonces[nonce] = timestamp
        return True, "VALID"

class DuplicateTagDetector:
    """Detects physically impossible movement or simultaneous multi-point emissions (Token Cloning)."""

    def __init__(self, max_human_speed_ms: float = 12.0):
        self.max_human_speed_ms = max_human_speed_ms
        self.last_observations: Dict[str, Tuple[float, float, float, str]] = {}

    def check_observation(self, tag_id: str, x: float, y: float, timestamp: float, device_uuid: str) -> Tuple[bool, str]:
        if tag_id not in self.last_observations:
            self.last_observations[tag_id] = (x, y, timestamp, device_uuid)
            return True, "CLEAN"

        prev_x, prev_y, prev_ts, prev_uuid = self.last_observations[tag_id]
        dt = max(0.001, timestamp - prev_ts)
        dx = x - prev_x
        dy = y - prev_y
        dist = (dx * dx + dy * dy) ** 0.5

        if prev_uuid != device_uuid and dist > 3.0:
            return False, f"DUPLICATE_HARDWARE_ID (UUIDs '{prev_uuid}' and '{device_uuid}' claim same Tag '{tag_id}' at distance {dist:.1f}m)"

        implied_speed = dist / dt
        if dist > 15.0 and implied_speed > self.max_human_speed_ms:
            return False, f"PHYSICAL_SPEED_VIOLATION (Moved {dist:.1f}m in {dt:.2f}s -> {implied_speed:.1f} m/s; suspected cloned emitter)"

        self.last_observations[tag_id] = (x, y, timestamp, device_uuid)
        return True, "CLEAN"
