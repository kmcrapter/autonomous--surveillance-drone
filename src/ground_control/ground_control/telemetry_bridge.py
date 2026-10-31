"""Telemetry Bridge connecting ROS 2 nodes, SQLite database, and the Streamlit Dashboard."""

import os
import json
import time
import math
from typing import Dict, Any, List, Optional
import numpy as np

from event_manager.database import SurveillanceDatabase
from coverage_planner.zone_manager import ZoneManager
from coverage_planner.boustrophedon import BoustrophedonPlanner

TELEMETRY_STATE_FILE = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/telemetry_state.json"
COMMAND_QUEUE_FILE = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/command_queue.json"

class TelemetryBridge:
    """Provides unified access to drone telemetry, database events, and command dispatches."""

    def __init__(self):
        self.db = SurveillanceDatabase()
        self.planner = BoustrophedonPlanner()
        os.makedirs(os.path.dirname(TELEMETRY_STATE_FILE), exist_ok=True)

    def send_mission_command(self, action: str, params: Optional[Dict[str, Any]] = None):
        """Sends an operator command to the mission manager node via command queue."""
        payload = {"action": action.upper(), "timestamp": time.time(), "params": params or {}}
        with open(COMMAND_QUEUE_FILE, "w") as f:
            json.dump(payload, f)

    def write_telemetry_state(self, state_dict: Dict[str, Any]):
        """Called by ROS 2 bridge node to publish real-time telemetry into dashboard cache."""
        temp_file = TELEMETRY_STATE_FILE + ".tmp"
        with open(temp_file, "w") as f:
            json.dump(state_dict, f)
        os.replace(temp_file, TELEMETRY_STATE_FILE)

    def get_latest_telemetry(self) -> Dict[str, Any]:
        """Reads real-time simulation telemetry or returns initialized fallback state."""
        if os.path.exists(TELEMETRY_STATE_FILE):
            try:
                with open(TELEMETRY_STATE_FILE, "r") as f:
                    return json.load(f)
            except Exception:
                pass

        # Default / Standby Telemetry
        return {
            "mission_id": f"MSN-{int(time.time())}",
            "state": "IDLE",
            "position": {"x": 0.0, "y": 0.0, "z": 0.38},
            "velocity": {"vx": 0.0, "vy": 0.0, "vz": 0.0, "speed_ms": 0.0},
            "yaw_deg": 0.0,
            "current_zone": "zone_alpha",
            "current_wp_index": 0,
            "total_waypoints": 65,
            "progress_pct": 0.0,
            "distance_flown_m": 0.0,
            "battery": {
                "percentage": 100.0,
                "voltage": 25.2,
                "current_a": 0.0,
                "power_w": 18.0,
                "remaining_wh": 555.0,
                "consumed_wh": 0.0,
                "remaining_time_min": 52.0,
                "is_charging": False
            },
            "rth": {
                "dist_to_home_m": 0.0,
                "rth_time_sec": 0.0,
                "rth_energy_wh": 0.0,
                "rth_energy_pct": 0.0,
                "rth_trigger_pct": 20.0,
                "should_return": False
            },
            "sensor_health": {
                "forward_rgb": True,
                "downward_rgb": True,
                "thermal_lwir": True,
                "night_vision": True,
                "imu": True,
                "gnss": True
            },
            "detections": []
        }

    def get_events(self, mission_id: Optional[str] = None) -> List[Dict[str, Any]]:
        return self.db.get_all_events(mission_id)

    def update_event_review(self, event_id: str, status: str, notes: Optional[str] = None):
        self.db.update_event_review(event_id, status, notes)
