"""Zone definition and geofence management for surveillance sectors."""

import math
from typing import Dict, List, Tuple, Any

class ZoneManager:
    """Manages predefined surveillance zones, geofences, and priority ordering."""

    BORDER_LINE_Y = 32.0
    SOVEREIGN_MAX_Y = 28.5 # Strict boundary: Drone remains 100% inside Country A

    ZONES = {
        "zone_alpha": {
            "id": "zone_alpha",
            "name": "Perimeter Alpha (Base Fence - Country A)",
            "priority": 1,
            "bounds": {"min_x": -45.0, "max_x": 45.0, "min_y": -55.0, "max_y": 25.0},
            "polygon": [
                (-45.0, -55.0),
                (45.0, -55.0),
                (45.0, 25.0),
                (-45.0, 25.0)
            ],
            "recommended_altitude": 14.0,
            "speed": 3.5,
            "side_overlap": 0.55,
            "description": "Base perimeter security fence, vehicle parking, and gates inside Country A."
        },
        "zone_bravo": {
            "id": "zone_bravo",
            "name": "Sovereign Forest Sector Bravo (West Border Woodline)",
            "priority": 2,
            "bounds": {"min_x": -85.0, "max_x": -25.0, "min_y": -20.0, "max_y": 28.0},
            "polygon": [
                (-85.0, -20.0),
                (-25.0, -20.0),
                (-25.0, 28.0),
                (-85.0, 28.0)
            ],
            "recommended_altitude": 18.0,
            "speed": 4.0,
            "side_overlap": 0.60,
            "description": "Sovereign western pine forest edge; patrols Country A side looking North across border."
        },
        "zone_charlie": {
            "id": "zone_charlie",
            "name": "Border Demarcation Standoff Corridor Charlie",
            "priority": 3,
            "bounds": {"min_x": -85.0, "max_x": 85.0, "min_y": 16.0, "max_y": 28.0},
            "polygon": [
                (-85.0, 16.0),
                (85.0, 16.0),
                (85.0, 28.0),
                (-85.0, 28.0)
            ],
            "recommended_altitude": 16.0,
            "speed": 4.5,
            "side_overlap": 0.50,
            "description": "Direct sovereign border corridor along Country A fence line surveying border posts and foreign territory without crossing."
        },
        "zone_delta": {
            "id": "zone_delta",
            "name": "Outpost Ridge Delta (Eastern Sovereign Bluff)",
            "priority": 4,
            "bounds": {"min_x": 35.0, "max_x": 85.0, "min_y": -35.0, "max_y": 28.0},
            "polygon": [
                (35.0, -35.0),
                (85.0, -35.0),
                (85.0, 28.0),
                (35.0, 28.0)
            ],
            "recommended_altitude": 20.0,
            "speed": 4.0,
            "side_overlap": 0.50,
            "description": "Elevated bluff, perimeter watchtower, and eastern boundary inside Country A."
        }
    }

    # Global Safe Geofence Limits (meters) - STRICTLY RESTRICTED TO COUNTRY A
    GEOFENCE = {
        "min_x": -100.0,
        "max_x": 100.0,
        "min_y": -65.0,
        "max_y": 28.5, # HARD SOVEREIGN GEOFENCE: NEVER PASS THE BORDER LINE INTO COUNTRY B
        "min_z": 1.0,
        "max_z": 45.0
    }

    HOME_PAD = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.49, # Landed altitude
        "takeoff_alt": 15.0
    }

    @classmethod
    def get_zone(cls, zone_id: str) -> Dict[str, Any]:
        return cls.ZONES.get(zone_id, None)

    @classmethod
    def get_all_zones(cls) -> Dict[str, Dict[str, Any]]:
        return cls.ZONES

    @classmethod
    def get_ordered_zones(cls, selected_ids: List[str] = None) -> List[Dict[str, Any]]:
        if not selected_ids:
            selected_ids = list(cls.ZONES.keys())
        zones = [cls.ZONES[zid] for zid in selected_ids if zid in cls.ZONES]
        zones.sort(key=lambda z: z["priority"])
        return zones

    @classmethod
    def is_within_geofence(cls, x: float, y: float, z: float) -> bool:
        g = cls.GEOFENCE
        return (g["min_x"] <= x <= g["max_x"] and
                g["min_y"] <= y <= g["max_y"] and
                g["min_z"] <= z <= g["max_z"])

    @classmethod
    def which_zone(cls, x: float, y: float) -> str:
        for zid, zinfo in cls.ZONES.items():
            b = zinfo["bounds"]
            if b["min_x"] <= x <= b["max_x"] and b["min_y"] <= y <= b["max_y"]:
                return zid
        return "transit"
