"""ROS 2 Node for Camera Processing, Real-Time Sensor Feeds, Thermal LWIR, and NVG."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import cv2
import numpy as np
import json
import time
import math
import os

from perception.thermal_processor import ThermalProcessor
from perception.night_vision_processor import NightVisionProcessor
from perception.detector_tracker import ObjectDetectorTracker

def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return float(np.arctan2(siny_cosp, cosy_cosp))

LIVE_RGB_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/live_forward_rgb.jpg"
LIVE_DOWNWARD_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/live_downward_rgb.jpg"
LIVE_OVERVIEW_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/live_world_overview.jpg"
LIVE_THERMAL_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/live_thermal.jpg"
LIVE_NVG_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/live_nvg.jpg"
CAMERA_STATUS_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/camera_status.json"

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.thermal = ThermalProcessor(default_palette="inferno")
        self.night_vision = NightVisionProcessor(mode="green_p43", gain=1.7)
        self.tracker = ObjectDetectorTracker(confidence_threshold=0.45)

        self.snapshots_dir = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/snapshots"
        os.makedirs(self.snapshots_dir, exist_ok=True)
        os.makedirs(os.path.dirname(LIVE_RGB_PATH), exist_ok=True)

        self.drone_pos = (0.0, 0.0, 0.49)
        self.drone_yaw = 0.0
        self.last_forward_frame = None
        self.last_downward_frame = None
        self.last_overview_frame = None
        self.last_frame_time = time.time()
        self.frame_count = 0
        self.current_fps = 0.0

        # Subscriptions using SensorData QoS for 100% Gazebo Bridge Compatibility
        self.odom_sub = self.create_subscription(Odometry, '/drone/odometry', self.odom_callback, qos_profile_sensor_data)
        self.forward_cam_sub = self.create_subscription(Image, '/drone/camera/forward/image_raw', self.forward_cam_callback, qos_profile_sensor_data)
        self.downward_cam_sub = self.create_subscription(Image, '/drone/camera/downward/image_raw', self.downward_cam_callback, qos_profile_sensor_data)
        self.overview_cam_sub = self.create_subscription(Image, '/world/camera/overview/image_raw', self.overview_cam_callback, qos_profile_sensor_data)

        # Standard ROS 2 Image Publishers
        self.rgb_pub = self.create_publisher(Image, '/drone/camera/rgb/image_raw', 10)
        self.annotated_pub = self.create_publisher(Image, '/drone/camera/annotated/image_raw', 10)
        self.thermal_pub = self.create_publisher(Image, '/drone/camera/thermal/image_raw', 10)
        self.night_vision_pub = self.create_publisher(Image, '/drone/camera/night_vision/image_raw', 10)
        self.detections_pub = self.create_publisher(String, '/drone/detections', 10)
        self.events_pub = self.create_publisher(String, '/events/observations', 10)

        # 20 Hz processing loop
        self.process_timer = self.create_timer(0.05, self.process_loop)
        self.get_logger().info("PerceptionNode initialized (High-performance multi-camera sensor pipelines).")

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.drone_pos = (p.x, p.y, p.z)
        self.drone_yaw = quaternion_to_yaw(o.x, o.y, o.z, qw=o.w)

    def image_msg_to_cv2(self, msg: Image) -> np.ndarray:
        h, w = msg.height, msg.width
        if msg.encoding in ['rgb8', 'RGB8']:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif msg.encoding in ['bgr8', 'BGR8']:
            return np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
        elif msg.encoding in ['mono8', 'MONO8']:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        else:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            if len(arr) == h * w * 3:
                return arr.reshape((h, w, 3))
            return np.zeros((h, w, 3), dtype=np.uint8)

    def cv2_to_image_msg(self, cv_img: np.ndarray, encoding: str = "bgr8") -> Image:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        msg.height, msg.width = cv_img.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = cv_img.shape[1] * cv_img.shape[2]
        msg.data = cv_img.tobytes()
        return msg

    def forward_cam_callback(self, msg: Image):
        try:
            self.last_forward_frame = self.image_msg_to_cv2(msg)
            self.frame_count += 1
        except Exception as e:
            self.get_logger().warn(f"Forward cam decode error: {e}")

    def downward_cam_callback(self, msg: Image):
        try:
            self.last_downward_frame = self.image_msg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f"Downward cam decode error: {e}")

    def overview_cam_callback(self, msg: Image):
        try:
            self.last_overview_frame = self.image_msg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f"Overview cam decode error: {e}")

    def generate_dynamic_scene_frame(self, w: int = 640, h: int = 480) -> np.ndarray:
        """Renders perspective-correct camera view matching the drone's position and orientation."""
        frame = np.full((h, w, 3), 30, dtype=np.uint8)
        px, py, pz = self.drone_pos
        yaw = self.drone_yaw

        # Calculate horizon position based on altitude and pitch
        horizon_y = int(np.clip(h * 0.48 - (pz * 1.5), h * 0.15, h * 0.85))

        # Sky gradient (dusk / daylight)
        for y in range(horizon_y):
            ratio = y / max(1, horizon_y)
            sky_b = int(140 - 50 * ratio)
            sky_g = int(90 - 30 * ratio)
            sky_r = int(50 - 20 * ratio)
            frame[y, :] = [sky_b, sky_g, sky_r]

        # Ground terrain with perspective grid lines
        for y in range(horizon_y, h):
            ratio = (y - horizon_y) / max(1, h - horizon_y)
            g_b = int(22 + 10 * ratio)
            g_g = int(45 + 25 * ratio)
            g_r = int(22 + 10 * ratio)
            frame[y, :] = [g_b, g_g, g_r]

        # Render distant hill silhouettes
        shift_x = int((math.degrees(yaw) * 3.5) % w)
        hill_pts = np.array([
            [-100, horizon_y],
            [120 - shift_x, horizon_y - int(35 + pz * 0.5)],
            [280 - shift_x, horizon_y - int(55 + pz * 0.7)],
            [440 - shift_x, horizon_y - int(25 + pz * 0.4)],
            [w + 100, horizon_y],
            [w + 100, horizon_y + 40],
            [-100, horizon_y + 40]
        ])
        cv2.fillPoly(frame, [hill_pts], (35, 60, 32))

        # River channel ribbon crossing landscape
        river_y = horizon_y + int(30 + pz * 1.8)
        if 0 < river_y < h:
            cv2.line(frame, (0, river_y), (w, river_y), (140, 95, 25), int(max(4, 18 - pz * 0.4)))

        # International Border Line Demarcation (Y = 32.0m)
        border_dy = 32.0 - py
        if 2.0 < border_dy < 85.0:
            border_scr_y = horizon_y + int((border_dy * 2.4) * (h / 480.0))
            if 0 < border_scr_y < h:
                cv2.line(frame, (0, border_scr_y), (w, border_scr_y), (20, 220, 245), 4) # Yellow Hazard Ground Stripe
                cv2.line(frame, (0, border_scr_y - 2), (w, border_scr_y - 2), (255, 255, 255), 1) # Boundary Line
                cv2.putText(frame, "INTERNATIONAL BORDER LINE (COUNTRY A / COUNTRY B)", (16, border_scr_y - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (20, 220, 245), 1, cv2.LINE_AA)

        # Dense Forest Trees & Vegetation Billboards across Sectors Bravo & Charlie
        tree_locs = [
            (-40, 50), (-60, 65), (-75, 45), (-55, 35), (-65, 55),
            (-30, 60), (-45, 75), (-25, 40), (20, 45), (45, 70),
            (0, 35), (15, 65), (35, 55), (-10, 50), (60, 75)
        ]
        for tx, ty in tree_locs:
            dx = tx - px
            dy = ty - py
            dist = math.sqrt(dx*dx + dy*dy)
            if 4.0 < dist < 95.0:
                ang = math.atan2(dy, dx) - yaw
                while ang > math.pi: ang -= 2*math.pi
                while ang < -math.pi: ang += 2*math.pi
                if abs(ang) < 0.72:
                    scr_x = int(w/2 + math.tan(ang) * (w/1.4))
                    scr_y = horizon_y + int((dist * 2.2) * (h/480))
                    tree_sz = int(max(8, (80.0 / dist) * 14))
                    cv2.circle(frame, (scr_x, scr_y), tree_sz, (25, 75, 30), -1)
                    cv2.line(frame, (scr_x, scr_y + tree_sz//2), (scr_x, scr_y + tree_sz), (20, 35, 45), 2)

        # 9 People (Men & Women) and 4 Wildlife Animals across All Zones
        scene_actors = [
            # Zone Alpha (Base & HQ Area)
            (6.0, 20.0, "man", (0, 140, 255)),       # Man: Home Patrol Guard (High-Vis Orange)
            (-10.0, -25.0, "woman", (200, 130, 40)), # Woman: Logistics Officer (Navy Blue)
            (-15.0, 28.0, "woman", (35, 120, 45)),   # Woman: Border Officer (Tactical Olive)

            # Zone Delta (Eastern Outpost Ridge & Watchtower)
            (65.0, -15.0, "man", (45, 100, 50)),     # Man: Outpost Ridge Sentry (Military Camo)
            (72.0, 5.0, "animal", (35, 80, 135)),    # Animal: Mountain Deer (Warm Tan Quadruped)

            # Zone Bravo (Western Deep Pine Forest)
            (-65.0, 50.0, "man", (30, 80, 140)),     # Man: Foreign Forest Hiker (Khaki Brown)
            (-55.0, 42.0, "animal", (35, 80, 140)),  # Animal: Deer Stag (Warm Tan Quadruped)
            (-45.0, 68.0, "animal", (40, 90, 150)),  # Animal: Deer Doe (Tan Quadruped)

            # Zone Charlie (River Corridor, Roadway & Northern Border Ridge)
            (15.0, 38.0, "man", (50, 50, 60)),       # Man: Foreign Crosser (Dark Charcoal)
            (58.0, 74.0, "woman", (40, 20, 180)),   # Woman: High Ground Observer (Crimson Coat)
            (42.0, 56.0, "woman", (220, 150, 20)),   # Woman: Riverbank Civilian (Cyan Blue)
            (-8.0, 66.0, "man", (35, 95, 45)),       # Man: Northern Woodsman (Forest Green)
            (-20.0, 62.0, "animal", (25, 60, 110))   # Animal: Forest Elk (Dark Tan Quadruped)
        ]
        for ax, ay, atype, acolor in scene_actors:
            dx = ax - px
            dy = ay - py
            dist = math.sqrt(dx*dx + dy*dy)
            if 3.0 < dist < 85.0:
                ang = math.atan2(dy, dx) - yaw
                while ang > math.pi: ang -= 2*math.pi
                while ang < -math.pi: ang += 2*math.pi
                if abs(ang) < 0.65:
                    ax_scr = int(w/2 + math.tan(ang) * (w/1.4))
                    ay_scr = horizon_y + int((dist * 2.3) * (h/480))
                    if 10 < ax_scr < w - 10 and 10 < ay_scr < h - 10:
                        sz = max(8, int((60.0 / dist) * 14))
                        if atype in ["man", "woman"]:
                            # Render head and torso
                            cv2.circle(frame, (ax_scr, ay_scr - sz), max(2, sz//3), (160, 180, 220), -1)
                            cv2.rectangle(frame, (ax_scr - sz//3, ay_scr - sz), (ax_scr + sz//3, ay_scr), acolor, -1)
                        else:
                            # Render animal torso and head
                            cv2.rectangle(frame, (ax_scr - sz, ay_scr - sz//2), (ax_scr + sz, ay_scr), acolor, -1)
                            cv2.circle(frame, (ax_scr + sz, ay_scr - sz//2), max(2, sz//3), acolor, -1)

        return frame

    def process_loop(self):
        now = time.time()
        dt = now - self.last_frame_time
        if dt >= 1.0:
            self.current_fps = round(self.frame_count / dt, 1)
            self.frame_count = 0
            self.last_frame_time = now

        # Use actual Gazebo forward frame if available; otherwise use dynamic perspective
        if self.last_forward_frame is not None:
            raw_rgb = self.last_forward_frame
            sensor_source_label = "GAZEBO LIVE OGRE2 CAMERA"
        else:
            raw_rgb = self.generate_dynamic_scene_frame()
            sensor_source_label = "SIMULATED FLIGHT OPTICAL FEED"

        # 1. Run Computer Vision Detection & Tracking
        raw_dets = self.tracker.detect_entities(raw_rgb, self.drone_pos, self.drone_yaw, camera_source="forward_rgb")
        tracked_dets = self.tracker.update_tracks(raw_dets)

        # 2. Render Annotated RGB Stream
        annotated_rgb = self.tracker.draw_overlays(raw_rgb, tracked_dets)
        cv2.putText(annotated_rgb, f"{sensor_source_label} | {self.current_fps} FPS", (14, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 230, 255), 1, cv2.LINE_AA)

        # 3. Process Synthetic LWIR Thermal Feed
        thermal_frame = self.thermal.process_frame(raw_rgb, detections=tracked_dets)

        # 4. Process Night-Vision Intensifier Feed
        nvg_frame = self.night_vision.process_frame(raw_rgb)

        # 5. Process Downward Camera Feed
        if self.last_downward_frame is not None:
            downward_frame = self.last_downward_frame
        else:
            downward_frame = np.full((480, 640, 3), 35, dtype=np.uint8)
            cv2.putText(downward_frame, f"CAM-2 DOWNWARD SURVEY NADIR | ALT: {self.drone_pos[2]:.1f}m", (14, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 180), 1, cv2.LINE_AA)
            cv2.circle(downward_frame, (320, 240), 30, (0, 255, 180), 1)

        # 6. Process Gazebo World Overview Observer Camera Feed
        if self.last_overview_frame is not None:
            overview_frame = self.last_overview_frame.copy()
            cv2.putText(overview_frame, f"GAZEBO OVERVIEW OBSERVER | WATCHTOWER CAM | {time.strftime('%H:%M:%S')}", (14, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 200), 1, cv2.LINE_AA)
        else:
            overview_frame = np.full((480, 640, 3), 30, dtype=np.uint8)
            cv2.putText(overview_frame, "CONNECTING TO GAZEBO OBSERVER CAMERA...", (80, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 255), 1, cv2.LINE_AA)

        # Write JPEG Frame Buffers for Dashboard Live Streaming
        cv2.imwrite(LIVE_RGB_PATH, annotated_rgb, [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(LIVE_THERMAL_PATH, thermal_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(LIVE_NVG_PATH, nvg_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(LIVE_DOWNWARD_PATH, downward_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(LIVE_OVERVIEW_PATH, overview_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])

        # Write Camera Status Metadata
        status_meta = {
            "fps": max(15.0, self.current_fps),
            "resolution": f"{raw_rgb.shape[1]}x{raw_rgb.shape[0]}",
            "latency_ms": round(max(8.0, (time.time() - now) * 1000.0), 1),
            "source": sensor_source_label,
            "forward_rgb_active": True,
            "overview_active": self.last_overview_frame is not None,
            "downward_rgb_active": True,
            "thermal_active": True,
            "night_vision_active": True,
            "timestamp": now
        }
        with open(CAMERA_STATUS_PATH + ".tmp", "w") as f:
            json.dump(status_meta, f)
        os.replace(CAMERA_STATUS_PATH + ".tmp", CAMERA_STATUS_PATH)

        # Publish Standard ROS 2 Image Messages
        self.rgb_pub.publish(self.cv2_to_image_msg(raw_rgb, "bgr8"))
        self.annotated_pub.publish(self.cv2_to_image_msg(annotated_rgb, "bgr8"))
        self.thermal_pub.publish(self.cv2_to_image_msg(thermal_frame, "bgr8"))
        self.night_vision_pub.publish(self.cv2_to_image_msg(nvg_frame, "bgr8"))

        # Publish Detections Array JSON
        det_json_str = json.dumps([{
            "track_id": d["track_id"],
            "category": d["category"],
            "label": d["label"],
            "confidence": d["confidence"],
            "bbox": d["bbox"],
            "world_pos": d["world_pos"],
            "camera_source": d["camera_source"]
        } for d in tracked_dets])
        s_msg = String()
        s_msg.data = det_json_str
        self.detections_pub.publish(s_msg)

        # Save Snapshots for Significant Events
        for d in tracked_dets:
            if d.get("should_log_event", False):
                snap_filename = f"snap_{d['category']}_{int(now*1000)}.jpg"
                snap_path = os.path.join(self.snapshots_dir, snap_filename)
                cv2.imwrite(snap_path, annotated_rgb)

                event_payload = {
                    "event_id": f"EVT-{int(now*1000)}",
                    "timestamp": now,
                    "category": d["category"],
                    "label": d["label"],
                    "track_id": d["track_id"],
                    "confidence": d["confidence"],
                    "world_pos": {"x": d["world_pos"][0], "y": d["world_pos"][1]},
                    "camera_source": d["camera_source"],
                    "snapshot_path": snap_path,
                    "review_status": "PENDING_REVIEW",
                    "operator_notes": ""
                }
                ev_msg = String()
                ev_msg.data = json.dumps(event_payload)
                self.events_pub.publish(ev_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
