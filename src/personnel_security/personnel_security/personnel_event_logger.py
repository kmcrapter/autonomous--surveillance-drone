"""ROS 2 Node for Personnel Identification Audit Logging and SQLite Data Persistence."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json
import time
import os
from typing import Dict, Any, List, Tuple

from event_manager.database import SurveillanceDatabase

PERSONNEL_STATE_FILE = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/personnel_state.json"

class PersonnelEventLoggerNode(Node):
    def __init__(self):
        super().__init__('personnel_event_logger')

        self.db = SurveillanceDatabase()
        self.last_logged_states: Dict[str, Tuple[str, float]] = {}
        os.makedirs(os.path.dirname(PERSONNEL_STATE_FILE), exist_ok=True)

        self.identities_sub = self.create_subscription(
            String,
            '/personnel/identities',
            self.identities_callback,
            10
        )

        self.security_events_sub = self.create_subscription(
            String,
            '/personnel/security_events',
            self.security_events_callback,
            10
        )

        self.alerts_sub = self.create_subscription(
            String,
            '/personnel/verification_alerts',
            self.alerts_callback,
            10
        )

        self.get_logger().info("PersonnelEventLoggerNode active (Audit Logging & SQLite Bridge).")

    def identities_callback(self, msg: String):
        now = time.time()
        try:
            payload = json.loads(msg.data)
            entities = payload.get("identities", [])
        except Exception:
            return

        try:
            temp_file = PERSONNEL_STATE_FILE + ".tmp"
            with open(temp_file, "w") as f:
                json.dump(payload, f)
            os.replace(temp_file, PERSONNEL_STATE_FILE)
        except Exception:
            pass

        for e in entities:
            eid = e.get("entity_id", "")
            state = e.get("identity_state", "UNVERIFIED")

            should_log = False
            if eid not in self.last_logged_states:
                should_log = True
            else:
                last_state, last_time = self.last_logged_states[eid]
                if last_state != state or (now - last_time >= 10.0):
                    should_log = True

            if should_log:
                self.last_logged_states[eid] = (state, now)
                wpos = e.get("world_pos", {})
                self.db.insert_personnel_audit_log({
                    "timestamp": now,
                    "entity_id": eid,
                    "personnel_id": e.get("personnel_id", "UNKNOWN"),
                    "full_name": e.get("full_name", "Unidentified Person"),
                    "assigned_unit": e.get("assigned_unit", "UNASSIGNED"),
                    "tag_id": e.get("tag_id", "NONE"),
                    "tag_type": e.get("tag_type", "NONE"),
                    "identity_state": state,
                    "face_status": e.get("face_verification_status", "UNVERIFIED"),
                    "confidence": e.get("confidence", 0.0),
                    "world_x": wpos.get("x", 0.0),
                    "world_y": wpos.get("y", 0.0),
                    "details": e.get("details", ""),
                    "review_status": e.get("operator_review_status", "PENDING_REVIEW")
                })

    def security_events_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            events = payload.get("security_events", [])
            for ev in events:
                self.db.insert_security_policy_event({
                    "id": ev["event_id"],
                    "timestamp": ev["timestamp"],
                    "policy_rule_id": ev.get("policy_rule_id", "RULE-GENERAL"),
                    "event_category": ev.get("event_category", "POLICY_EVENT"),
                    "zone_id": ev.get("zone_id", "ZONE-N/A"),
                    "zone_name": ev.get("zone_name", "General"),
                    "target_entity_id": ev.get("target_entity_id", "UNKNOWN"),
                    "target_name": ev.get("target_name", "Subject"),
                    "target_identity_state": ev.get("target_identity_state", "UNVERIFIED"),
                    "severity": ev.get("severity", "MEDIUM"),
                    "description": ev.get("description", ""),
                    "review_status": "PENDING_REVIEW",
                    "operator_notes": ""
                })
        except Exception as e:
            self.get_logger().warn(f"Failed to log security policy event: {e}")

    def alerts_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            alert_type = payload.get("type")
            rec = payload.get("record", {})
            if alert_type == "VOICE_ANNOUNCEMENT_DISPATCHED":
                self.db.insert_personnel_verification({
                    "id": rec.get("announcement_id", f"VER-{int(time.time()*1000)}"),
                    "timestamp": rec.get("timestamp", time.time()),
                    "entity_id": rec.get("target_id", "BROADCAST"),
                    "action": "VOICE_SAFETY_ANNOUNCEMENT",
                    "status": "TRANSMITTED",
                    "notes": rec.get("text", "")
                })
            elif alert_type == "NO_ID_CARD_ALERT":
                al = payload.get("alert", {})
                self.db.insert_personnel_verification({
                    "id": al.get("alert_id", f"ALERT-{int(time.time()*1000)}"),
                    "timestamp": al.get("timestamp", time.time()),
                    "entity_id": al.get("target_id", "UNIDENTIFIED"),
                    "action": "NO_ID_CARD_ALERT",
                    "status": "ALERT_TRIGGERED",
                    "notes": al.get("message", "Person detected with no electronic ID card.")
                })
        except Exception as e:
            self.get_logger().warn(f"Failed to record verification interaction: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PersonnelEventLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
