"""ROS 2 Node for AI-Assisted Suspicious Activity & Policy-Defined Security Assessment.

Strictly decouples:
  1. Object Detections (Physical presence of entity)
  2. Identity Verification Failures (Tag/Biometric outcome)
  3. Policy-Defined Security Events (Spatial, infrastructure, or behavioral rule alerts)
Never automatically classifies any person as hostile.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json
import time
import math
from typing import Dict, Any, List, Tuple

RESTRICTED_ZONES = [
    {
        "id": "SEC-ZONE-01",
        "name": "Ammunition & Ordnance Depot",
        "center": (25.0, -10.0),
        "radius": 12.0,
        "required_clearance": "LEVEL-4_COMMAND",
        "severity": "HIGH",
        "policy_description": "Restricted zone containing sensitive munitions. Mandatory Level-4 authorization."
    },
    {
        "id": "SEC-ZONE-02",
        "name": "Operations HQ Command Bunker",
        "center": (0.0, -15.0),
        "radius": 15.0,
        "required_clearance": "LEVEL-3_SECURITY",
        "severity": "MEDIUM",
        "policy_description": "Tactical Operations Center. Clearance Level 3 or higher required."
    },
    {
        "id": "SEC-ZONE-03",
        "name": "Drone Flight Line & Hangar Apron",
        "center": (0.0, 0.0),
        "radius": 8.0,
        "required_clearance": "LEVEL-2_LOGISTICS",
        "severity": "LOW",
        "policy_description": "Active UAV launch and recovery apron. Flight line clearance required."
    }
]

BORDER_LINE_Y = 32.0

class ActivityAssessmentNode(Node):
    def __init__(self):
        super().__init__('activity_assessment_node')

        self.latest_identities: Dict[str, Dict[str, Any]] = {}
        self.dwell_tracker: Dict[int, List[Tuple[float, float, float]]] = {}

        self.identities_sub = self.create_subscription(
            String,
            '/personnel/identities',
            self.identities_callback,
            10
        )

        self.detections_sub = self.create_subscription(
            String,
            '/drone/detections',
            self.detections_callback,
            10
        )

        self.events_pub = self.create_publisher(
            String,
            '/personnel/security_events',
            10
        )

        self.timer = self.create_timer(0.5, self.evaluate_policies)
        self.get_logger().info("ActivityAssessmentNode active (Policy Event Engine & Infrastructure Geofencing).")

    def identities_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            ids = payload.get("identities", [])
            for item in ids:
                self.latest_identities[item["entity_id"]] = item
        except Exception:
            pass

    def detections_callback(self, msg: String):
        now = time.time()
        try:
            dets = json.loads(msg.data)
        except Exception:
            return

        for d in dets:
            tid = d.get("track_id", 0)
            wx, wy = d.get("world_pos", [0.0, 0.0])
            if tid not in self.dwell_tracker:
                self.dwell_tracker[tid] = []
            self.dwell_tracker[tid].append((now, wx, wy))
            self.dwell_tracker[tid] = [pt for pt in self.dwell_tracker[tid] if now - pt[0] <= 30.0]

    def evaluate_policies(self):
        now = time.time()
        generated_events = []

        # 1. Evaluate Restricted Infrastructure Geofence Rules
        for eid, entity in self.latest_identities.items():
            ex = entity["world_pos"]["x"]
            ey = entity["world_pos"]["y"]
            clearance = entity.get("clearance_level", "NONE")
            ident_state = entity.get("identity_state", "UNVERIFIED")

            for zone in RESTRICTED_ZONES:
                zx, zy = zone["center"]
                dist = math.sqrt((ex - zx)**2 + (ey - zy)**2)
                if dist <= zone["radius"]:
                    has_clearance = False
                    if clearance == "LEVEL-4_COMMAND":
                        has_clearance = True
                    elif clearance == "LEVEL-3_SECURITY" and zone["required_clearance"] in ["LEVEL-3_SECURITY", "LEVEL-2_LOGISTICS"]:
                        has_clearance = True
                    elif clearance == "LEVEL-2_LOGISTICS" and zone["required_clearance"] == "LEVEL-2_LOGISTICS":
                        has_clearance = True

                    if not has_clearance:
                        generated_events.append({
                            "event_id": f"SEC-EVT-ZONE-{int(now*1000)}",
                            "timestamp": now,
                            "policy_rule_id": "RULE-INFRASTRUCTURE-01",
                            "event_category": "RESTRICTED_ZONE_UNAUTHORIZED_ENTRY",
                            "zone_id": zone["id"],
                            "zone_name": zone["name"],
                            "target_entity_id": eid,
                            "target_name": entity.get("full_name", "Unverified Person"),
                            "target_identity_state": ident_state,
                            "world_pos": {"x": round(ex, 1), "y": round(ey, 1)},
                            "severity": zone["severity"],
                            "distinction_breakdown": {
                                "is_object_detection": True,
                                "is_identity_verification_failure": ident_state in ["UNVERIFIED", "UNKNOWN_TAG"],
                                "is_policy_defined_security_event": True,
                                "hostile_classification": False
                            },
                            "description": f"Observed presence in {zone['name']} without required {zone['required_clearance']}. Policy rule breach flagged for human review.",
                            "recommended_action": "Request operator review; initiate simulated verification workflow; voice announcement advisory."
                        })

        # 2. Evaluate Border Buffer Loitering
        for tid, history in list(self.dwell_tracker.items()):
            if len(history) < 10:
                continue
            first_time, first_x, first_y = history[0]
            last_time, last_x, last_y = history[-1]
            dwell_duration = last_time - first_time

            in_border_buffer = all(27.0 <= pt[2] <= 37.0 for pt in history[-6:])
            net_displacement = math.sqrt((last_x - first_x)**2 + (last_y - first_y)**2)

            if in_border_buffer and dwell_duration > 12.0 and net_displacement < 6.0:
                generated_events.append({
                    "event_id": f"SEC-EVT-LOITER-{int(now*1000)}",
                    "timestamp": now,
                    "policy_rule_id": "RULE-BORDER-BUFFER-02",
                    "event_category": "BORDER_BUFFER_LOITERING",
                    "zone_id": "BORDER-BUFFER",
                    "zone_name": "International Border Buffer Strip",
                    "target_entity_id": f"PERS-TRK-{tid}",
                    "target_name": "Border Proximity Subject",
                    "target_identity_state": "UNVERIFIED",
                    "world_pos": {"x": round(last_x, 1), "y": round(last_y, 1)},
                    "severity": "MEDIUM",
                    "distinction_breakdown": {
                        "is_object_detection": True,
                        "is_identity_verification_failure": True,
                        "is_policy_defined_security_event": True,
                        "hostile_classification": False
                    },
                    "description": f"Entity observed lingering within border buffer strip for {dwell_duration:.1f}s (displacement: {net_displacement:.1f}m).",
                    "recommended_action": "Issue safety boundary voice announcement. Request operator review."
                })

        if generated_events:
            msg = String()
            msg.data = json.dumps({
                "timestamp": now,
                "event_count": len(generated_events),
                "security_events": generated_events
            })
            self.events_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ActivityAssessmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
