"""ROS 2 Node for Event Logging and SQLite Data Layer."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

from event_manager.database import SurveillanceDatabase

class EventManagerNode(Node):
    def __init__(self):
        super().__init__('event_manager_node')

        self.db = SurveillanceDatabase()
        self.current_mission_id = "MSN-DEFAULT"
        self.last_telem_log_time = 0.0

        # Subscriptions
        self.obs_sub = self.create_subscription(String, '/events/observations', self.observation_callback, 20)
        self.telem_sub = self.create_subscription(String, '/drone/mission_telemetry', self.telemetry_callback, 10)
        self.review_sub = self.create_subscription(String, '/drone/operator_review', self.operator_review_callback, 10)

        self.get_logger().info("EventManagerNode active (Connected to SQLite surveillance.db).")

    def observation_callback(self, msg: String):
        try:
            ev = json.loads(msg.data)
            self.db.insert_event({
                "id": ev["event_id"],
                "mission_id": self.current_mission_id,
                "timestamp": ev["timestamp"],
                "category": ev["category"],
                "label": ev["label"],
                "track_id": ev["track_id"],
                "confidence": ev["confidence"],
                "world_x": ev["world_pos"]["x"],
                "world_y": ev["world_pos"]["y"],
                "camera_source": ev["camera_source"],
                "snapshot_path": ev.get("snapshot_path", ""),
                "review_status": ev.get("review_status", "PENDING_REVIEW"),
                "operator_notes": ev.get("operator_notes", "")
            })
            self.get_logger().info(f"Persisted event {ev['event_id']} ({ev['label']}) to database.")
        except Exception as e:
            self.get_logger().error(f"Failed to persist observation event: {e}")

    def telemetry_callback(self, msg: String):
        now = time.time()
        try:
            data = json.loads(msg.data)
            self.current_mission_id = data.get("mission_id", self.current_mission_id)

            # Sample telemetry every 1.5 seconds for history
            if now - self.last_telem_log_time >= 1.5:
                self.last_telem_log_time = now
                pos = data.get("position", {})
                vel = data.get("velocity", {})
                self.db.insert_telemetry_sample({
                    "mission_id": self.current_mission_id,
                    "timestamp": now,
                    "state": data.get("state", "UNKNOWN"),
                    "x": pos.get("x", 0.0),
                    "y": pos.get("y", 0.0),
                    "z": pos.get("z", 0.0),
                    "speed": vel.get("speed_ms", 0.0),
                    "battery_pct": data.get("battery_pct", 100.0),
                    "current_zone": data.get("current_zone", "transit")
                })

                # Upsert mission record
                self.db.upsert_mission({
                    "id": self.current_mission_id,
                    "scenario": "Autonomous 3D Border & Perimeter Surveillance",
                    "start_time": now - data.get("mission_duration_sec", 0.0),
                    "end_time": now if data.get("state") in ["COMPLETE", "ABORTED"] else None,
                    "status": data.get("state", "RUNNING"),
                    "assigned_zones": "Alpha, Bravo, Charlie, Delta",
                    "waypoints_total": data.get("total_waypoints", 0),
                    "distance_m": data.get("distance_flown_m", 0.0),
                    "battery_consumed_pct": 100.0 - data.get("battery_pct", 100.0),
                    "rth_count": 1 if data.get("state") in ["RETURNING", "CHARGING"] else 0,
                    "summary": f"Autonomous surveillance progress: {data.get('progress_pct', 0)}%, zone: {data.get('current_zone', 'N/A')}"
                })
        except Exception as e:
            self.get_logger().error(f"Error handling telemetry in EventManager: {e}")

    def operator_review_callback(self, msg: String):
        try:
            req = json.loads(msg.data)
            ev_id = req.get("event_id")
            status = req.get("status", "ACKNOWLEDGED")
            notes = req.get("notes")
            if ev_id:
                self.db.update_event_review(ev_id, status, notes)
                self.get_logger().info(f"Updated event {ev_id} review status to {status}")
        except Exception as e:
            self.get_logger().error(f"Error updating operator review: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EventManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
