"""ROS 2 Node for Cryptographic Credential Authentication, Anti-Replay, and Clone Detection."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json
import time
from typing import Dict, Any, List

from personnel_security.crypto_utils import (
    NonceReplayCache,
    DuplicateTagDetector,
    verify_tag_signature
)
from personnel_security.personnel_registry import PersonnelRegistryManager

class CredentialAuthenticationNode(Node):
    def __init__(self):
        super().__init__('credential_authentication_node')

        self.replay_cache = NonceReplayCache(window_seconds=6.0)
        self.duplicate_detector = DuplicateTagDetector(max_human_speed_ms=12.0)

        # Raw Tag Subscription
        self.raw_sub = self.create_subscription(
            String,
            '/personnel/raw_tags',
            self.raw_tag_callback,
            10
        )

        # Authenticated Tag Publisher
        self.auth_pub = self.create_publisher(
            String,
            '/personnel/authenticated_tags',
            10
        )

        self.get_logger().info("CredentialAuthenticationNode active (Cryptographic Verification & Anti-Replay Engine).")

    def raw_tag_callback(self, msg: String):
        now = time.time()
        try:
            payload = json.loads(msg.data)
            tags = payload.get("tags", [])
        except Exception as e:
            self.get_logger().warn(f"Failed to parse raw tag payload: {e}")
            return

        verified_records = []

        for tag in tags:
            tag_id = tag.get("tag_id", "UNKNOWN")
            nonce = tag.get("nonce", "")
            ts = tag.get("timestamp", now)
            sig = tag.get("signature", "")
            hw_uuid = tag.get("hardware_uuid", "UNKNOWN")
            coords = (tag.get("simulated_coords", {}).get("x", 0.0), tag.get("simulated_coords", {}).get("y", 0.0))

            # 1. Look up profile in authorized registry
            profile = PersonnelRegistryManager.find_by_tag_id(tag_id)
            if not profile:
                verified_records.append({
                    "tag_id": tag_id,
                    "personnel_id": "UNKNOWN",
                    "full_name": "Unregistered Holder",
                    "assigned_unit": "UNASSIGNED",
                    "clearance_level": "NONE",
                    "tag_type": tag.get("tag_type", "UNKNOWN"),
                    "hardware_uuid": hw_uuid,
                    "auth_status": "UNKNOWN_TAG",
                    "confidence": 0.15,
                    "reason": f"Tag ID '{tag_id}' is not registered in the personnel authorization database.",
                    "rssi_dbm": tag.get("rssi_dbm", -90.0),
                    "estimated_distance_m": tag.get("estimated_distance_m", 50.0),
                    "coords": coords,
                    "timestamp": now
                })
                continue

            # 2. Replay Protection: Check Nonce Freshness
            replay_ok, replay_msg = self.replay_cache.validate_and_record(nonce, ts, now)
            if not replay_ok:
                verified_records.append({
                    "tag_id": tag_id,
                    "personnel_id": profile["personnel_id"],
                    "full_name": profile["full_name"],
                    "assigned_unit": profile["assigned_unit"],
                    "clearance_level": profile["clearance_level"],
                    "tag_type": tag.get("tag_type", profile["tag_type"]),
                    "hardware_uuid": hw_uuid,
                    "auth_status": "CREDENTIAL_CONFLICT",
                    "confidence": 0.20,
                    "reason": f"Cryptographic Replay Attack Detected: {replay_msg}",
                    "rssi_dbm": tag.get("rssi_dbm", -90.0),
                    "estimated_distance_m": tag.get("estimated_distance_m", 50.0),
                    "coords": coords,
                    "timestamp": now
                })
                continue

            # 3. Duplicate-Tag & Cloned Hardware Emitter Detection
            clone_clean, clone_msg = self.duplicate_detector.check_observation(
                tag_id, coords[0], coords[1], ts, hw_uuid
            )
            if not clone_clean:
                verified_records.append({
                    "tag_id": tag_id,
                    "personnel_id": profile["personnel_id"],
                    "full_name": profile["full_name"],
                    "assigned_unit": profile["assigned_unit"],
                    "clearance_level": profile["clearance_level"],
                    "tag_type": tag.get("tag_type", profile["tag_type"]),
                    "hardware_uuid": hw_uuid,
                    "auth_status": "CREDENTIAL_CONFLICT",
                    "confidence": 0.25,
                    "reason": f"Duplicate/Cloned Token Detected: {clone_msg}",
                    "rssi_dbm": tag.get("rssi_dbm", -90.0),
                    "estimated_distance_m": tag.get("estimated_distance_m", 50.0),
                    "coords": coords,
                    "timestamp": now
                })
                continue

            # 4. Cryptographic HMAC Signature Verification
            sig_valid = verify_tag_signature(tag_id, nonce, ts, coords, sig)
            if not sig_valid:
                verified_records.append({
                    "tag_id": tag_id,
                    "personnel_id": profile["personnel_id"],
                    "full_name": profile["full_name"],
                    "assigned_unit": profile["assigned_unit"],
                    "clearance_level": profile["clearance_level"],
                    "tag_type": tag.get("tag_type", profile["tag_type"]),
                    "hardware_uuid": hw_uuid,
                    "auth_status": "CREDENTIAL_CONFLICT",
                    "confidence": 0.30,
                    "reason": "Cryptographic signature validation failure. Key mismatch or packet corruption.",
                    "rssi_dbm": tag.get("rssi_dbm", -90.0),
                    "estimated_distance_m": tag.get("estimated_distance_m", 50.0),
                    "coords": coords,
                    "timestamp": now
                })
                continue

            # Authentication Succeeded
            verified_records.append({
                "tag_id": tag_id,
                "personnel_id": profile["personnel_id"],
                "full_name": profile["full_name"],
                "rank": profile.get("rank", "Officer"),
                "assigned_unit": profile["assigned_unit"],
                "role": profile["role"],
                "clearance_level": profile["clearance_level"],
                "tag_type": tag.get("tag_type", profile["tag_type"]),
                "hardware_uuid": hw_uuid,
                "auth_status": "AUTHENTICATED",
                "confidence": 0.99,
                "reason": "Cryptographically authenticated and anti-replay verified.",
                "rssi_dbm": tag.get("rssi_dbm", -55.0),
                "estimated_distance_m": tag.get("estimated_distance_m", 12.0),
                "coords": coords,
                "timestamp": now
            })

        # Publish verified authentication records
        out_msg = String()
        out_msg.data = json.dumps({
            "timestamp": now,
            "authenticated_count": len(verified_records),
            "authenticated_tags": verified_records
        })
        self.auth_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CredentialAuthenticationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
