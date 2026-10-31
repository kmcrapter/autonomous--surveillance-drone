"""Consented Synthetic Personnel Registry and Biometric Template Database for Simulation."""

import numpy as np
from typing import Dict, Any, List, Optional, Tuple

def _generate_synthetic_embedding(seed_val: int) -> List[float]:
    """Generates deterministic unit-normalized 128-D synthetic feature embedding vector."""
    rng = np.random.RandomState(seed_val)
    vec = rng.randn(128)
    norm = np.linalg.norm(vec)
    return (vec / max(1e-6, norm)).tolist()

# Consented Synthetic Personnel Profiles (No real military personnel biometrics)
SYNTHETIC_PERSONNEL_REGISTRY: Dict[str, Dict[str, Any]] = {
    "PERS-101": {
        "personnel_id": "PERS-101",
        "full_name": "Col. Marcus Vance",
        "rank": "Colonel",
        "assigned_unit": "1st Recon Command / Alpha Outpost",
        "role": "Sector Commander",
        "clearance_level": "LEVEL-4_COMMAND",
        "tag_id": "TAG-UWB-101",
        "tag_type": "UWB_BEACON",
        "device_model": "AeroGuard UWB Ultra-Wideband Tactical Beacon v2.4",
        "authorized_zones": ["zone_alpha", "zone_bravo", "zone_charlie", "zone_delta"],
        "restricted_infrastructure_access": True,
        "face_template_id": "BIO-VANCE-001",
        "biometric_embedding": _generate_synthetic_embedding(101),
        "visual_profile": {
            "gender": "male",
            "clothing_color": "high_vis_orange",
            "role_marker": "guard_cap"
        },
        "default_world_coords": (6.0, 20.0),
        "tag_active": True
    },
    "PERS-102": {
        "personnel_id": "PERS-102",
        "full_name": "Capt. Sarah Mitchell",
        "rank": "Captain",
        "assigned_unit": "4th Combat Logistics Battalion",
        "role": "Supply Operations Officer",
        "clearance_level": "LEVEL-2_LOGISTICS",
        "tag_id": "TAG-NFC-204",
        "tag_type": "ENCRYPTED_RFID",
        "device_model": "Mil-Spec Dual-Frequency Crypto RFID Smartcard",
        "authorized_zones": ["zone_alpha"],
        "restricted_infrastructure_access": False,
        "face_template_id": "BIO-MITCHELL-002",
        "biometric_embedding": _generate_synthetic_embedding(102),
        "visual_profile": {
            "gender": "female",
            "clothing_color": "logistics_tan",
            "role_marker": "badge_clip"
        },
        "default_world_coords": (-10.0, -25.0),
        "tag_active": True
    },
    "PERS-103": {
        "personnel_id": "PERS-103",
        "full_name": "Sgt. J. Kowalski",
        "rank": "Sergeant",
        "assigned_unit": "Eastern Watchtower Sentry Detachment",
        "role": "Perimeter Guard",
        "clearance_level": "LEVEL-3_SECURITY",
        "tag_id": "TAG-BLE-309",
        "tag_type": "BLE_BEACON",
        "device_model": "Tactical BLE-5.3 Encrypted Personnel Tag",
        "authorized_zones": ["zone_alpha", "zone_delta"],
        "restricted_infrastructure_access": True,
        "face_template_id": "BIO-KOWALSKI-003",
        "biometric_embedding": _generate_synthetic_embedding(103),
        "visual_profile": {
            "gender": "male",
            "clothing_color": "field_green",
            "role_marker": "helmet"
        },
        "default_world_coords": (65.0, -15.0),
        "tag_active": True
    },
    "PERS-104": {
        "personnel_id": "PERS-104",
        "full_name": "Lt. Elena Rostova",
        "rank": "Lieutenant",
        "assigned_unit": "Sovereign Border Demarcation Unit",
        "role": "Border Patrol Officer",
        "clearance_level": "LEVEL-3_SECURITY",
        "tag_id": "TAG-WEAR-412",
        "tag_type": "WEARABLE_SECURITY_MODULE",
        "device_model": "BioWrist Secure Multi-Modal Wearable v1.9",
        "authorized_zones": ["zone_alpha", "zone_charlie"],
        "restricted_infrastructure_access": True,
        "face_template_id": "BIO-ROSTOVA-004",
        "biometric_embedding": _generate_synthetic_embedding(104),
        "visual_profile": {
            "gender": "female",
            "clothing_color": "patrol_navy",
            "role_marker": "tactical_vest"
        },
        "default_world_coords": (-15.0, 28.0),
        "tag_active": True
    },
    "PERS-105": {
        "personnel_id": "PERS-105",
        "full_name": "Specialist D. Miller",
        "rank": "Specialist",
        "assigned_unit": "River Basin Recon / Sector Charlie",
        "role": "River Crossing Sentry",
        "clearance_level": "LEVEL-2_LOGISTICS",
        "tag_id": "TAG-BLE-515",
        "tag_type": "BLE_BEACON",
        "device_model": "Tactical BLE-5.3 Encrypted Personnel Tag",
        "authorized_zones": ["zone_charlie"],
        "restricted_infrastructure_access": False,
        "face_template_id": "BIO-MILLER-005",
        "biometric_embedding": _generate_synthetic_embedding(105),
        "visual_profile": {
            "gender": "male",
            "clothing_color": "woodland_camo",
            "role_marker": "boonie_hat"
        },
        "default_world_coords": (-8.0, 66.0),
        "tag_active": True
    }
}

class PersonnelRegistryManager:
    """Provides role-based access control and biometric data minimization."""

    @classmethod
    def get_all_profiles(cls, minimize_biometrics: bool = True) -> List[Dict[str, Any]]:
        results = []
        for pid, data in SYNTHETIC_PERSONNEL_REGISTRY.items():
            entry = dict(data)
            if minimize_biometrics and "biometric_embedding" in entry:
                del entry["biometric_embedding"]
            results.append(entry)
        return results

    @classmethod
    def find_by_personnel_id(cls, pid: str) -> Optional[Dict[str, Any]]:
        return SYNTHETIC_PERSONNEL_REGISTRY.get(pid, None)

    @classmethod
    def find_by_tag_id(cls, tag_id: str) -> Optional[Dict[str, Any]]:
        for pid, data in SYNTHETIC_PERSONNEL_REGISTRY.items():
            if data.get("tag_id") == tag_id:
                return data
        return None

    @classmethod
    def match_biometric_embedding(cls, probe_embedding: List[float], threshold: float = 0.75) -> Tuple[Optional[Dict[str, Any]], float]:
        """Calculates cosine similarity against consented synthetic database templates."""
        probe = np.array(probe_embedding)
        norm_probe = np.linalg.norm(probe)
        if norm_probe < 1e-6:
            return None, 0.0
        probe_unit = probe / norm_probe

        best_match = None
        highest_score = 0.0

        for pid, profile in SYNTHETIC_PERSONNEL_REGISTRY.items():
            template = np.array(profile["biometric_embedding"])
            score = float(np.dot(probe_unit, template))
            if score > highest_score:
                highest_score = score
                if score >= threshold:
                    best_match = profile

        return best_match, round(highest_score, 3)
