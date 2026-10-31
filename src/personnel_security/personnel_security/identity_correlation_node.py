"""ROS 2 Node for Multi-Source Identity Correlation, Identity State Machine & Verification Interaction Workflow."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import json
import time
import math
import os
from typing import Dict, Any, List, Optional, Tuple

VOICE_ANNOUNCEMENT_TEXT = (
    "Identity verification required. Please remain in the designated safe area "
    "and contact authorized personnel."
)

OPERATOR_ACTION_QUEUE_FILE = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/operator_action_queue.json"

class IdentityCorrelationNode(Node):
    def __init__(self):
        super().__init__('identity_correlation_node')

        self.latest_auth_tags: Dict[str, Dict[str, Any]] = {}
        self.latest_visual_verifications: Dict[int, Dict[str, Any]] = {}
        self.active_identities: Dict[str, Dict[str, Any]] = {}
        self.announcements_history: List[Dict[str, Any]] = []
        self.emitted_alerts: List[Dict[str, Any]] = []

        # Subscriptions
        self.auth_sub = self.create_subscription(
            String,
            '/personnel/authenticated_tags',
            self.auth_tags_callback,
            10
        )

        self.visual_sub = self.create_subscription(
            String,
            '/personnel/visual_verification',
            self.visual_callback,
            10
        )

        self.operator_action_sub = self.create_subscription(
            String,
            '/personnel/operator_action',
            self.operator_action_callback,
            10
        )

        # Publishers
        self.identities_pub = self.create_publisher(
            String,
            '/personnel/identities',
            10
        )

        self.alerts_pub = self.create_publisher(
            String,
            '/personnel/verification_alerts',
            10
        )

        self.events_pub = self.create_publisher(
            String,
            '/events/observations',
            10
        )

        # 5 Hz correlation loop
        self.timer = self.create_timer(0.2, self.correlation_loop)
        self.get_logger().info("IdentityCorrelationNode active (Multi-Source Fusion & State Machine).")

    def auth_tags_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            tags = payload.get("authenticated_tags", [])
            for t in tags:
                self.latest_auth_tags[t["tag_id"]] = t
        except Exception as e:
            self.get_logger().warn(f"Failed to parse auth tags: {e}")

    def visual_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            results = payload.get("results", [])
            for r in results:
                tid = r.get("track_id", 0)
                self.latest_visual_verifications[tid] = r
        except Exception as e:
            self.get_logger().warn(f"Failed to parse visual verifications: {e}")

    def operator_action_callback(self, msg: String):
        now = time.time()
        try:
            action_data = json.loads(msg.data)
            action_type = action_data.get("action", "").upper()
            target_id = action_data.get("target_id", "")
            notes = action_data.get("notes", "")

            if action_type == "TRIGGER_VOICE_ANNOUNCEMENT":
                announcement_record = {
                    "announcement_id": f"VOICE-{int(now * 1000)}",
                    "timestamp": now,
                    "target_id": target_id,
                    "text": VOICE_ANNOUNCEMENT_TEXT,
                    "status": "BROADCAST_TRANSMITTED",
                    "operator_notes": notes
                }
                self.announcements_history.append(announcement_record)
                self.get_logger().info(f"Broadcast simulated voice announcement to target {target_id}: '{VOICE_ANNOUNCEMENT_TEXT}'")

                al_msg = String()
                al_msg.data = json.dumps({
                    "type": "VOICE_ANNOUNCEMENT_DISPATCHED",
                    "record": announcement_record
                })
                self.alerts_pub.publish(al_msg)

            elif action_type in ["INITIATE_VERIFICATION_WORKFLOW", "APPROVE_REVIEW", "DISMISS_REVIEW", "ACKNOWLEDGED_CLEARED"]:
                if target_id in self.active_identities:
                    entity = self.active_identities[target_id]
                    entity["operator_review_status"] = action_type
                    entity["operator_notes"] = notes
                    self.get_logger().info(f"Updated entity {target_id} review status to {action_type}")

        except Exception as e:
            self.get_logger().warn(f"Failed to process operator action: {e}")

    def correlation_loop(self):
        now = time.time()

        # Check operator action queue file from GCS backend
        if os.path.exists(OPERATOR_ACTION_QUEUE_FILE):
            try:
                with open(OPERATOR_ACTION_QUEUE_FILE, "r") as f:
                    action_payload = json.load(f)
                os.remove(OPERATOR_ACTION_QUEUE_FILE)
                msg = String()
                msg.data = json.dumps(action_payload)
                self.operator_action_callback(msg)
            except Exception:
                pass

        correlated_entities = []
        matched_tags = set()

        for tid, vis in list(self.latest_visual_verifications.items()):
            if now - vis.get("timestamp", now) > 4.0:
                continue

            vx = vis["world_pos"]["x"]
            vy = vis["world_pos"]["y"]

            # Search for closest electronic tag within 18m
            best_tag = None
            best_dist = 999.0
            for tag_id, tag in self.latest_auth_tags.items():
                if now - tag.get("timestamp", now) > 4.0:
                    continue
                tx, ty = tag["coords"]
                dist = math.sqrt((vx - tx)**2 + (vy - ty)**2)
                if dist < 18.0 and dist < best_dist:
                    best_dist = dist
                    best_tag = tag

            entity_id = f"PERS-TRK-{tid}"

            # Multi-Source Evidence Fusion
            if best_tag is not None:
                matched_tags.add(best_tag["tag_id"])
                auth_status = best_tag["auth_status"]

                if auth_status == "AUTHENTICATED":
                    identity_state = "AUTHENTICATED"
                    confidence = 0.98
                    details = f"Electronic credential verified via {best_tag['tag_type']} ({best_tag['hardware_uuid']})."
                    if vis.get("visual_identity_status") == "VISUAL_VERIFIED":
                        details += " Visual facial match corroborated."
                elif auth_status == "UNKNOWN_TAG":
                    identity_state = "UNKNOWN_TAG"
                    confidence = 0.20
                    details = f"Unregistered electronic credential detected ({best_tag['tag_id']}). Requires human review."
                elif auth_status == "CREDENTIAL_CONFLICT":
                    identity_state = "CREDENTIAL_CONFLICT"
                    confidence = 0.15
                    details = f"Conflicting credential signals detected ({best_tag.get('reason', 'Duplicate Token')}). Non-hostile review required."
                else:
                    identity_state = "UNVERIFIED"
                    confidence = 0.35
                    details = "Electronic credential validation inconclusive."

                entity_record = {
                    "entity_id": entity_id,
                    "track_id": tid,
                    "personnel_id": best_tag.get("personnel_id", "UNKNOWN"),
                    "full_name": best_tag.get("full_name", "Unidentified Person"),
                    "rank": best_tag.get("rank", "N/A"),
                    "assigned_unit": best_tag.get("assigned_unit", "UNASSIGNED"),
                    "role": best_tag.get("role", "Monitored Individual"),
                    "tag_id": best_tag.get("tag_id", "NO_TAG"),
                    "tag_type": best_tag.get("tag_type", "NONE"),
                    "has_id_card": True,
                    "identity_state": identity_state,
                    "confidence": confidence,
                    "world_pos": {"x": vx, "y": vy},
                    "face_verification_status": vis.get("visual_identity_status", "UNVERIFIED"),
                    "face_match_outcome": vis.get("match_outcome", "INSUFFICIENT_QUALITY"),
                    "face_confidence": vis.get("confidence_score", 0.0),
                    "image_quality": vis.get("image_quality", {}),
                    "rssi_dbm": best_tag.get("rssi_dbm", -90.0),
                    "details": details,
                    "operator_review_status": self.active_identities.get(entity_id, {}).get("operator_review_status", "PENDING_REVIEW"),
                    "operator_notes": self.active_identities.get(entity_id, {}).get("operator_notes", ""),
                    "voice_announcement_issued": self.active_identities.get(entity_id, {}).get("voice_announcement_issued", False),
                    "last_updated": now
                }
            else:
                # =========================================================================
                # NO ID CARD DETECTED FOR THIS PERSON
                # =========================================================================
                has_matched_face = (vis.get("visual_identity_status") == "VISUAL_VERIFIED" and vis.get("matched_profile"))

                if has_matched_face:
                    p = vis["matched_profile"]
                    identity_state = "VISUAL_VERIFIED"
                    confidence = vis.get("confidence_score", 0.85)
                    details = f"Visual facial match ({p['full_name']}) without active electronic ID card."
                    pid = p["personnel_id"]
                    pname = p["full_name"]
                    prank = p.get("rank", "Officer")
                    punit = p["assigned_unit"]
                    has_id_card = False
                else:
                    # Individual has NO ID CARD and face is not in database -> ALERT TRIGGER
                    identity_state = "UNVERIFIED"
                    confidence = 0.25
                    details = "NO ID CARD DETECTED: Individual observed with zero electronic credentials and unverified face. Alert dispatched."
                    pid = "UNVERIFIED"
                    pname = "Subject (No ID Card)"
                    prank = "N/A"
                    punit = "UNASSIGNED"
                    has_id_card = False

                    # Trigger explicit NO ID CARD ALERT
                    if entity_id not in self.active_identities or not self.active_identities[entity_id].get("no_id_alert_emitted"):
                        alert_payload = {
                            "alert_id": f"ALERT-NO-ID-{int(now * 1000)}",
                            "timestamp": now,
                            "type": "NO_ID_CARD_ALERT",
                            "severity": "WARNING",
                            "target_id": entity_id,
                            "world_pos": {"x": round(vx, 1), "y": round(vy, 1)},
                            "title": "SECURITY ADVISORY: UNIDENTIFIED PERSON - NO ID CARD DETECTED",
                            "message": f"Individual detected at coordinates ({vx:.1f}, {vy:.1f}) carrying NO electronic identification card. Initiating safety prompt.",
                            "recommended_action": "Verify credentials, issue neutral voice advisory, or request operator visual review.",
                            "hostile_classification": False
                        }
                        self.emitted_alerts.append(alert_payload)
                        self.get_logger().warn(f"ALERT: Unidentified individual {entity_id} detected with NO ID CARD at ({vx:.1f}, {vy:.1f})!")

                        # Publish alert to GCS and Event Center
                        al_msg = String()
                        al_msg.data = json.dumps({"type": "NO_ID_CARD_ALERT", "alert": alert_payload})
                        self.alerts_pub.publish(al_msg)
                        self.events_pub.publish(al_msg)

                entity_record = {
                    "entity_id": entity_id,
                    "track_id": tid,
                    "personnel_id": pid,
                    "full_name": pname,
                    "rank": prank,
                    "assigned_unit": punit,
                    "role": "Unverified Individual (No ID Card)" if not has_id_card else "Authorized Personnel (Card Missing)",
                    "tag_id": "NONE",
                    "tag_type": "NO_ID_CARD_DETECTED",
                    "has_id_card": has_id_card,
                    "identity_state": identity_state,
                    "confidence": confidence,
                    "world_pos": {"x": vx, "y": vy},
                    "face_verification_status": vis.get("visual_identity_status", "UNVERIFIED"),
                    "face_match_outcome": vis.get("match_outcome", "INSUFFICIENT_QUALITY"),
                    "face_confidence": vis.get("confidence_score", 0.0),
                    "image_quality": vis.get("image_quality", {}),
                    "rssi_dbm": -110.0,
                    "details": details,
                    "no_id_alert_emitted": True,
                    "operator_review_status": self.active_identities.get(entity_id, {}).get("operator_review_status", "PENDING_REVIEW"),
                    "operator_notes": self.active_identities.get(entity_id, {}).get("operator_notes", ""),
                    "voice_announcement_issued": self.active_identities.get(entity_id, {}).get("voice_announcement_issued", False),
                    "last_updated": now
                }

            # Automated Non-Coercive Safety Prompt
            if entity_record["identity_state"] in ["UNVERIFIED", "UNKNOWN_TAG", "CREDENTIAL_CONFLICT"]:
                if not entity_record["voice_announcement_issued"]:
                    entity_record["voice_announcement_issued"] = True
                    self.announcements_history.append({
                        "announcement_id": f"VOICE-{int(now * 1000)}",
                        "timestamp": now,
                        "target_id": entity_id,
                        "text": VOICE_ANNOUNCEMENT_TEXT,
                        "status": "AUTOMATED_SAFETY_PROMPT",
                        "operator_notes": f"Prompted by {entity_record['identity_state']} observation (No ID card present)."
                    })
                    self.get_logger().info(f"Safety announcement queued for {entity_id}: '{VOICE_ANNOUNCEMENT_TEXT}'")

            self.active_identities[entity_id] = entity_record
            correlated_entities.append(entity_record)

        # 2. Add standalone electronic tags detected without visual camera detection
        for tag_id, tag in self.latest_auth_tags.items():
            if tag_id in matched_tags or now - tag.get("timestamp", now) > 4.0:
                continue

            entity_id = f"TAG-ONLY-{tag_id}"
            auth_status = tag.get("auth_status", "UNKNOWN_TAG")
            entity_record = {
                "entity_id": entity_id,
                "track_id": 0,
                "personnel_id": tag.get("personnel_id", "UNKNOWN"),
                "full_name": tag.get("full_name", "Electronic Tag Holder"),
                "rank": tag.get("rank", "N/A"),
                "assigned_unit": tag.get("assigned_unit", "UNASSIGNED"),
                "role": tag.get("role", "Personnel"),
                "tag_id": tag_id,
                "tag_type": tag.get("tag_type", "UWB_BEACON"),
                "has_id_card": True,
                "identity_state": auth_status,
                "confidence": tag.get("confidence", 0.95),
                "world_pos": {"x": tag["coords"][0], "y": tag["coords"][1]},
                "face_verification_status": "NOT_IN_OPTICAL_FOV",
                "face_match_outcome": "OUT_OF_VIEW",
                "face_confidence": 0.0,
                "image_quality": {},
                "rssi_dbm": tag.get("rssi_dbm", -60.0),
                "details": f"RF electronic identification active ({tag.get('reason', 'Authenticated')}). Out of optical camera field of view.",
                "operator_review_status": self.active_identities.get(entity_id, {}).get("operator_review_status", "PENDING_REVIEW"),
                "operator_notes": self.active_identities.get(entity_id, {}).get("operator_notes", ""),
                "voice_announcement_issued": False,
                "last_updated": now
            }
            self.active_identities[entity_id] = entity_record
            correlated_entities.append(entity_record)

        # Publish unified correlated identities
        out_msg = String()
        out_msg.data = json.dumps({
            "timestamp": now,
            "entity_count": len(correlated_entities),
            "identities": correlated_entities,
            "announcements": self.announcements_history[-10:],
            "active_alerts": self.emitted_alerts[-10:]
        })
        self.identities_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IdentityCorrelationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
