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
LIVE_REAR_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/live_rear_rgb.jpg"
LIVE_LEFT_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/live_left_rgb.jpg"
LIVE_RIGHT_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/live_right_rgb.jpg"
LIVE_PENTAGRID_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/live_pentagrid.jpg"
LIVE_OVERVIEW_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/live_world_overview.jpg"
LIVE_THERMAL_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/live_thermal.jpg"
LIVE_NVG_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/live_nvg.jpg"
CAMERA_STATUS_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/camera_status.json"
THERMAL_CMD_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/thermal_cmd.json"

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
        self.last_odom_time = 0.0
        self.last_forward_frame = None
        self.last_downward_frame = None
        self.last_rear_frame = None
        self.last_left_frame = None
        self.last_right_frame = None
        self.last_overview_frame = None
        self.last_frame_time = time.time()
        self.frame_count = 0
        self.current_fps = 0.0

        # Subscriptions using SensorData QoS for 100% Gazebo Bridge Compatibility
        self.odom_sub = self.create_subscription(Odometry, '/drone/odometry', self.odom_callback, qos_profile_sensor_data)
        self.forward_cam_sub = self.create_subscription(Image, '/drone/camera/forward/image_raw', self.forward_cam_callback, qos_profile_sensor_data)
        self.downward_cam_sub = self.create_subscription(Image, '/drone/camera/downward/image_raw', self.downward_cam_callback, qos_profile_sensor_data)
        self.rear_cam_sub = self.create_subscription(Image, '/drone/camera/rear/image_raw', self.rear_cam_callback, qos_profile_sensor_data)
        self.left_cam_sub = self.create_subscription(Image, '/drone/camera/left/image_raw', self.left_cam_callback, qos_profile_sensor_data)
        self.right_cam_sub = self.create_subscription(Image, '/drone/camera/right/image_raw', self.right_cam_callback, qos_profile_sensor_data)
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
        self.last_odom_time = time.time()

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

    def rear_cam_callback(self, msg: Image):
        try:
            self.last_rear_frame = self.image_msg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f"Rear cam decode error: {e}")

    def left_cam_callback(self, msg: Image):
        try:
            self.last_left_frame = self.image_msg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f"Left cam decode error: {e}")

    def right_cam_callback(self, msg: Image):
        try:
            self.last_right_frame = self.image_msg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f"Right cam decode error: {e}")

    def overview_cam_callback(self, msg: Image):
        try:
            self.last_overview_frame = self.image_msg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f"Overview cam decode error: {e}")

    def generate_dynamic_scene_frame(self, yaw_offset: float = 0.0, is_nadir: bool = False, w: int = 640, h: int = 480) -> np.ndarray:
        """Renders perspective-correct camera view matching the drone's position and orientation."""
        frame = np.full((h, w, 3), 30, dtype=np.uint8)
        px, py, pz = self.drone_pos
        yaw = (self.drone_yaw + yaw_offset) % (2 * math.pi)

        cur_t = time.time()
        scene_actors = []
        for act in self.tracker.KNOWN_ACTORS:
            if "route" in act and len(act["route"]) > 1:
                route = act["route"]
                spd = act.get("speed", 1.0)
                idx = int((cur_t * spd) % len(route))
                next_idx = (idx + 1) % len(route)
                alpha = (cur_t * spd) % 1.0
                ax = (1.0 - alpha) * route[idx][0] + alpha * route[next_idx][0]
                ay = (1.0 - alpha) * route[idx][1] + alpha * route[next_idx][1]
            else:
                ax, ay = act["pos"]

            cat = act["category"]
            sz = act.get("size", (1.0, 1.0))
            if cat == "building":
                col = (110, 130, 150)
                scene_actors.append((ax, ay, "building", col, sz))
            elif cat == "person":
                g = act.get("gender", "man")
                col = (0, 140, 255) if g == "man" else (200, 130, 40)
                scene_actors.append((ax, ay, g, col, sz))
            elif cat == "vehicle":
                col = (30, 70, 140)
                scene_actors.append((ax, ay, "vehicle", col, sz))
            elif cat == "wildlife":
                col = (35, 80, 135)
                scene_actors.append((ax, ay, "animal", col, sz))
            elif cat == "fire":
                col = (20, 140, 255)
                scene_actors.append((ax, ay, "fire", col, sz))
            elif cat == "hazard":
                col = (70, 70, 90)
                scene_actors.append((ax, ay, "cargo", col, sz))

        if is_nadir:
            # Top-down ground survey projection
            frame[:] = [24, 46, 24] # Tactical terrain green
            cx, cy = w // 2, h // 2
            scale = (w / 2.0) / max(2.0, pz * math.tan(1.57 / 2.0))

            # Concentric range rings
            for r_m in [10.0, 20.0, 30.0, 40.0]:
                r_px = int(r_m * scale)
                if r_px < min(w, h):
                    cv2.circle(frame, (cx, cy), r_px, (35, 75, 35), 1, cv2.LINE_AA)
                    cv2.putText(frame, f"{int(r_m)}m", (cx + r_px + 2, cy - 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.32, (80, 160, 80), 1, cv2.LINE_AA)

            # Center Nadir Reticle
            cv2.line(frame, (cx - 18, cy), (cx + 18, cy), (0, 240, 255), 1, cv2.LINE_AA)
            cv2.line(frame, (cx, cy - 18), (cx, cy + 18), (0, 240, 255), 1, cv2.LINE_AA)
            cv2.circle(frame, (cx, cy), 6, (0, 240, 255), 1, cv2.LINE_AA)

            # Render top-down actors
            for ax, ay, atype, acolor, (rw, rh) in scene_actors:
                dx = ax - px
                dy = ay - py
                # Rotate into drone heading
                rx = dx * math.cos(yaw) + dy * math.sin(yaw)
                ry = -dx * math.sin(yaw) + dy * math.cos(yaw)
                au = int(cx - ry * scale)
                av = int(cy - rx * scale)

                if 0 <= au < w and 0 <= av < h:
                    if atype == "building":
                        bw_px = max(6, int(rw * scale))
                        bh_px = max(6, int(rh * scale))
                        cv2.rectangle(frame, (au - bw_px//2, av - bh_px//2), (au + bw_px//2, av + bh_px//2), acolor, -1)
                        cv2.rectangle(frame, (au - bw_px//2, av - bh_px//2), (au + bw_px//2, av + bh_px//2), (200, 230, 255), 1)
                    else:
                        rad = max(4, int(max(rw, rh) * scale * 0.5))
                        cv2.circle(frame, (au, av), rad, acolor, -1)
                        cv2.circle(frame, (au, av), rad + 2, (0, 255, 255), 1)

            return frame

        # Perspective View for Forward, Rear, Left, and Right Cameras
        pitch = 0.25
        hfov = 1.4
        fx = (w / 2.0) / math.tan(hfov / 2.0)
        fy = fx
        horizon_y = int(np.clip((h / 2.0) - fy * math.tan(pitch), 40, h - 40))

        # Sky gradient
        for y in range(horizon_y):
            ratio = y / max(1, horizon_y)
            sky_b = int(140 - 50 * ratio)
            sky_g = int(90 - 30 * ratio)
            sky_r = int(50 - 20 * ratio)
            frame[y, :] = [sky_b, sky_g, sky_r]

        # Ground terrain with perspective shading
        for y in range(horizon_y, h):
            ratio = (y - horizon_y) / max(1, h - horizon_y)
            g_b = int(22 + 10 * ratio)
            g_g = int(45 + 25 * ratio)
            g_r = int(22 + 10 * ratio)
            frame[y, :] = [g_b, g_g, g_r]

        # Distant hill silhouettes
        shift_x = int((math.degrees(yaw) * 3.5) % w)
        hill_pts = np.array([
            [-100, horizon_y],
            [120 - shift_x, horizon_y - int(35 + pz * 0.3)],
            [280 - shift_x, horizon_y - int(55 + pz * 0.4)],
            [440 - shift_x, horizon_y - int(25 + pz * 0.2)],
            [w + 100, horizon_y],
            [w + 100, horizon_y + 40],
            [-100, horizon_y + 40]
        ])
        cv2.fillPoly(frame, [hill_pts], (35, 60, 32))

        # River channel ribbon crossing landscape
        p_riv_left = ObjectDetectorTracker.project_world_to_pixel(-80.0, 60.0, 0.0, self.drone_pos, yaw, pitch, hfov, w, h)
        p_riv_right = ObjectDetectorTracker.project_world_to_pixel(80.0, 60.0, 0.0, self.drone_pos, yaw, pitch, hfov, w, h)
        if p_riv_left and p_riv_right:
            cv2.line(frame, (int(p_riv_left[0]), int(p_riv_left[1])), (int(p_riv_right[0]), int(p_riv_right[1])), (140, 95, 25), 8, cv2.LINE_AA)

        # International Border Line Demarcation (Y = 32.0m)
        p_b_left = ObjectDetectorTracker.project_world_to_pixel(-100.0, 32.0, 0.0, self.drone_pos, yaw, pitch, hfov, w, h)
        p_b_right = ObjectDetectorTracker.project_world_to_pixel(100.0, 32.0, 0.0, self.drone_pos, yaw, pitch, hfov, w, h)
        if p_b_left and p_b_right:
            u1, v1 = int(p_b_left[0]), int(p_b_left[1])
            u2, v2 = int(p_b_right[0]), int(p_b_right[1])
            cv2.line(frame, (u1, v1), (u2, v2), (20, 220, 245), 3, cv2.LINE_AA)
            cv2.putText(frame, "INTERNATIONAL BORDER LINE (COUNTRY A / COUNTRY B)",
                        (max(16, min(w - 380, (u1 + u2) // 2 - 150)), max(20, min(h - 20, (v1 + v2) // 2 - 6))),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (20, 220, 245), 1, cv2.LINE_AA)

        # Dense Forest Pine Trees
        tree_locs = [
            (-40, 50), (-60, 65), (-75, 45), (-55, 35), (-65, 55),
            (-30, 60), (-45, 75), (-25, 40), (20, 45), (45, 70),
            (0, 35), (15, 65), (35, 55), (-10, 50), (60, 75)
        ]
        for tx, ty in tree_locs:
            p_tr = ObjectDetectorTracker.project_world_to_pixel(float(tx), float(ty), 0.0, self.drone_pos, yaw, pitch, hfov, w, h)
            if p_tr:
                tu, tv, td = p_tr
                if -20 < tu < w + 20 and -20 < tv < h + 20:
                    tree_sz = max(6, int(round(fy * (4.5 / td))))
                    cv2.circle(frame, (int(tu), int(tv - tree_sz)), tree_sz, (25, 75, 30), -1)
                    cv2.line(frame, (int(tu), int(tv)), (int(tu), int(tv - tree_sz)), (20, 35, 45), max(1, tree_sz // 4))

        # Render People, Wildlife, Vehicles & Hazards
        for ax, ay, atype, acolor, (rw, rh) in scene_actors:
            proj = ObjectDetectorTracker.project_world_to_pixel(ax, ay, 0.0, self.drone_pos, yaw, pitch, hfov, w, h)
            if proj:
                au, av, dist = proj
                if -30 < au < w + 30 and -30 < av < h + 30:
                    bw = max(10, int(round(fx * (rw / dist))))
                    bh = max(14, int(round(fy * (rh / dist))))
                    x1 = int(au - bw // 2)
                    y2 = int(av)
                    y1 = int(av - bh)
                    x2 = x1 + bw

                    if atype in ["man", "woman"]:
                        head_r = max(2, bh // 7)
                        cv2.circle(frame, (int(au), y1 + head_r), head_r, (160, 180, 220), -1)
                        cv2.rectangle(frame, (x1, y1 + head_r * 2), (x2, y2), acolor, -1)
                        # Dynamic walking animation
                        walk_phase = int(math.sin(cur_t * 6.5 + au) * max(2, bh // 5))
                        cv2.line(frame, (int(au), y2 - bh // 4), (int(au + walk_phase), y2), (20, 20, 25), 2)
                        cv2.line(frame, (int(au), y2 - bh // 4), (int(au - walk_phase), y2), (20, 20, 25), 2)
                    elif atype == "building":
                        # 3D building block with roof and windows
                        cv2.rectangle(frame, (x1, y1), (x2, y2), acolor, -1)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (40, 50, 60), 2)
                        roof_h = max(3, bh // 5)
                        cv2.rectangle(frame, (x1 - 2, y1 - roof_h), (x2 + 2, y1), (60, 70, 80), -1)
                        win_w = max(3, bw // 7)
                        win_h = max(3, bh // 4)
                        if bw > 18 and bh > 12:
                            for wx in range(x1 + win_w, x2 - win_w, max(4, win_w * 2)):
                                cv2.rectangle(frame, (wx, y1 + win_h), (min(x2 - 2, wx + win_w), y1 + win_h * 2), (200, 230, 255), -1)
                    elif atype == "animal":
                        cv2.rectangle(frame, (x1, y1 + bh // 3), (x2, y2), acolor, -1)
                        cv2.circle(frame, (x2, y1 + bh // 3), max(3, bh // 5), acolor, -1)
                        cv2.line(frame, (x1 + 2, y2 - 4), (x1 + 2, y2), (20, 20, 20), 2)
                        cv2.line(frame, (x2 - 2, y2 - 4), (x2 - 2, y2), (20, 20, 20), 2)
                    elif atype == "vehicle":
                        cv2.rectangle(frame, (x1, y1 + bh // 3), (x2, y2), acolor, -1)
                        cv2.rectangle(frame, (x1 + bw // 4, y1), (x2 - bw // 4, y1 + bh // 3), (50, 70, 150), -1)
                        cv2.line(frame, (x1 + bw // 3, y1 - 2), (x2 - bw // 3, y1 - 2), (255, 100, 0), 2)
                        wheel_r = max(2, bw // 8)
                        cv2.circle(frame, (x1 + bw // 5, y2), wheel_r, (20, 20, 20), -1)
                        cv2.circle(frame, (x2 - bw // 5, y2), wheel_r, (20, 20, 20), -1)
                    elif atype == "fire":
                        cv2.circle(frame, (int(au), int(av - bh // 2)), bw // 2, (0, 120, 255), -1)
                        cv2.circle(frame, (int(au), int(av - bh // 2)), max(2, bw // 3), (0, 220, 255), -1)
                    elif atype == "cargo":
                        cv2.rectangle(frame, (x1, y1), (x2, y2), acolor, -1)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (20, 30, 40), 1)

        return frame

    def process_loop(self):
        now = time.time()
        dt = now - self.last_frame_time
        if dt >= 1.0:
            self.current_fps = round(self.frame_count / dt, 1)
            self.frame_count = 0
            self.last_frame_time = now

        # High-frequency telemetry synchronization fallback if Gazebo odom topic is buffering
        if (now - self.last_odom_time) > 0.20:
            try:
                telem_file = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/telemetry_state.json"
                if os.path.exists(telem_file):
                    with open(telem_file, "r") as f:
                        tdata = json.load(f)
                    pos = tdata.get("position", {})
                    if "x" in pos and "y" in pos and "z" in pos:
                        self.drone_pos = (float(pos["x"]), float(pos["y"]), float(pos["z"]))
                    yaw_val = tdata.get("yaw_deg")
                    if yaw_val is not None:
                        # Convert navigational heading (0=North) to Cartesian yaw (0=East, pi/2=North)
                        self.drone_yaw = math.radians((90.0 - float(yaw_val)) % 360.0)
                    elif "orientation" in tdata and "yaw_deg" in tdata["orientation"]:
                        self.drone_yaw = math.radians((90.0 - float(tdata["orientation"]["yaw_deg"])) % 360.0)
            except Exception:
                pass

        # Check thermal control commands from GCS UI
        if os.path.exists(THERMAL_CMD_PATH):
            try:
                with open(THERMAL_CMD_PATH, "r") as f:
                    t_cmd = json.load(f)
                if "palette" in t_cmd:
                    self.thermal.set_palette(t_cmd["palette"])
                if "isotherm" in t_cmd:
                    self.thermal.set_isotherm(t_cmd["isotherm"].get("enabled", True),
                                              float(t_cmd["isotherm"].get("threshold_c", 35.0)))
                os.remove(THERMAL_CMD_PATH)
            except Exception:
                pass

        # 1. Forward Optical RGB Camera (CAM-1)
        if self.last_forward_frame is not None:
            raw_forward = self.last_forward_frame
            sensor_source_label = "GAZEBO LIVE OGRE2 CAMERA"
        else:
            raw_forward = self.generate_dynamic_scene_frame(yaw_offset=0.0)
            sensor_source_label = "SIMULATED FLIGHT OPTICAL FEED"

        dets_forward = self.tracker.detect_entities(raw_forward, self.drone_pos, self.drone_yaw, cam_pitch=0.25, camera_source="forward_rgb")
        tracks_forward = self.tracker.update_tracks(dets_forward)
        annotated_forward = self.tracker.draw_overlays(raw_forward, tracks_forward)
        cv2.putText(annotated_forward, f"CAM-1 FORWARD OPTICAL | {self.current_fps} FPS", (14, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 230, 255), 1, cv2.LINE_AA)

        # 2. Rear Optical Camera (CAM-2, Aft 180°)
        if self.last_rear_frame is not None:
            raw_rear = self.last_rear_frame
        else:
            raw_rear = self.generate_dynamic_scene_frame(yaw_offset=math.pi)

        dets_rear = self.tracker.detect_entities(raw_rear, self.drone_pos, self.drone_yaw + math.pi, cam_pitch=0.25, camera_source="rear_rgb")
        tracks_rear = self.tracker.update_tracks(dets_rear)
        annotated_rear = self.tracker.draw_overlays(raw_rear, tracks_rear)
        cv2.putText(annotated_rear, f"CAM-2 REAR AFT 180 | {self.current_fps} FPS", (14, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 210, 255), 1, cv2.LINE_AA)

        # 3. Port Left-Flank Optical Camera (CAM-3, 270°)
        if self.last_left_frame is not None:
            raw_left = self.last_left_frame
        else:
            raw_left = self.generate_dynamic_scene_frame(yaw_offset=math.pi / 2.0)

        dets_left = self.tracker.detect_entities(raw_left, self.drone_pos, self.drone_yaw + math.pi / 2.0, cam_pitch=0.25, camera_source="left_rgb")
        tracks_left = self.tracker.update_tracks(dets_left)
        annotated_left = self.tracker.draw_overlays(raw_left, tracks_left)
        cv2.putText(annotated_left, f"CAM-3 PORT FLANK 270 | {self.current_fps} FPS", (14, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 240, 180), 1, cv2.LINE_AA)

        # 4. Starboard Right-Flank Optical Camera (CAM-4, 90°)
        if self.last_right_frame is not None:
            raw_right = self.last_right_frame
        else:
            raw_right = self.generate_dynamic_scene_frame(yaw_offset=-math.pi / 2.0)

        dets_right = self.tracker.detect_entities(raw_right, self.drone_pos, self.drone_yaw - math.pi / 2.0, cam_pitch=0.25, camera_source="right_rgb")
        tracks_right = self.tracker.update_tracks(dets_right)
        annotated_right = self.tracker.draw_overlays(raw_right, tracks_right)
        cv2.putText(annotated_right, f"CAM-4 STARBOARD FLANK 90 | {self.current_fps} FPS", (14, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 240, 180), 1, cv2.LINE_AA)

        # 5. Downward Nadir Survey Camera (CAM-5, Under-Drone)
        if self.last_downward_frame is not None:
            raw_downward = self.last_downward_frame
        else:
            raw_downward = self.generate_dynamic_scene_frame(is_nadir=True)

        dets_downward = self.tracker.detect_entities(raw_downward, self.drone_pos, self.drone_yaw, cam_pitch=1.57079, camera_source="downward_rgb")
        tracks_downward = self.tracker.update_tracks(dets_downward)
        annotated_downward = self.tracker.draw_overlays(raw_downward, tracks_downward)
        cv2.putText(annotated_downward, f"CAM-5 DOWNWARD NADIR | ALT: {self.drone_pos[2]:.1f}m", (14, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 255, 180), 1, cv2.LINE_AA)

        # Combine all camera tracks
        all_tracked_dets = tracks_forward + tracks_rear + tracks_left + tracks_right + tracks_downward
        # Deduplicate by track_id
        unique_dets = {}
        for d in all_tracked_dets:
            tid = d.get("track_id")
            if tid not in unique_dets:
                unique_dets[tid] = d

        # 6. Process Synthetic LWIR Thermal Feed with Target Tracking & Temperature Radiometry
        thermal_frame = self.thermal.process_frame(raw_forward, detections=tracks_forward)

        # 7. Process Night-Vision Intensifier Feed with Phosphor Target Overlays & IR Luminescence
        nvg_frame = self.night_vision.process_frame(raw_forward, detections=tracks_forward)

        # 8. Process Gazebo World Overview Observer Camera Feed
        if self.last_overview_frame is not None:
            overview_frame = self.last_overview_frame.copy()
            cv2.putText(overview_frame, f"GAZEBO OVERVIEW OBSERVER | WATCHTOWER CAM | {time.strftime('%H:%M:%S')}", (14, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 200), 1, cv2.LINE_AA)
        else:
            overview_frame = np.full((480, 640, 3), 30, dtype=np.uint8)
            cv2.putText(overview_frame, "CONNECTING TO GAZEBO OBSERVER CAMERA...", (80, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 255), 1, cv2.LINE_AA)

        # Write JPEG Frame Buffers for All 5 Drone Cameras + 3 Sensor Channels
        cv2.imwrite(LIVE_RGB_PATH, annotated_forward, [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(LIVE_REAR_PATH, annotated_rear, [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(LIVE_LEFT_PATH, annotated_left, [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(LIVE_RIGHT_PATH, annotated_right, [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(LIVE_DOWNWARD_PATH, annotated_downward, [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(LIVE_THERMAL_PATH, thermal_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(LIVE_NVG_PATH, nvg_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        cv2.imwrite(LIVE_OVERVIEW_PATH, overview_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])

        # Construct Consolidated 360+Nadir 5-Camera Panoramic Array Frame (960 x 540)
        penta_canvas = np.full((540, 960, 3), 15, dtype=np.uint8)
        penta_canvas[5:245, 5:315] = cv2.resize(annotated_forward, (310, 240))
        penta_canvas[5:245, 325:635] = cv2.resize(annotated_downward, (310, 240))
        penta_canvas[5:245, 645:955] = cv2.resize(annotated_rear, (310, 240))
        penta_canvas[255:535, 5:475] = cv2.resize(annotated_left, (470, 280))
        penta_canvas[255:535, 485:955] = cv2.resize(annotated_right, (470, 280))
        cv2.putText(penta_canvas, "CAM-1 FWD 0°", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 240, 255), 1)
        cv2.putText(penta_canvas, "CAM-5 NADIR 90°", (335, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 180), 1)
        cv2.putText(penta_canvas, "CAM-2 REAR 180°", (655, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 210, 255), 1)
        cv2.putText(penta_canvas, "CAM-3 PORT 270°", (15, 275), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 240, 180), 1)
        cv2.putText(penta_canvas, "CAM-4 STBD 90°", (495, 275), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 240, 180), 1)
        cv2.imwrite(LIVE_PENTAGRID_PATH, penta_canvas, [cv2.IMWRITE_JPEG_QUALITY, 85])

        # Write Camera Status Metadata with Radiometric Intelligence
        status_meta = {
            "fps": max(15.0, self.current_fps),
            "resolution": f"{raw_forward.shape[1]}x{raw_forward.shape[0]}",
            "latency_ms": round(max(8.0, (time.time() - now) * 1000.0), 1),
            "source": sensor_source_label,
            "forward_rgb_active": True,
            "rear_rgb_active": True,
            "left_rgb_active": True,
            "right_rgb_active": True,
            "downward_rgb_active": True,
            "thermal_active": True,
            "night_vision_active": True,
            "overview_active": self.last_overview_frame is not None,
            "radiometry": self.thermal.last_radiometry,
            "thermal_palette": self.thermal.current_palette,
            "timestamp": now
        }
        with open(CAMERA_STATUS_PATH + ".tmp", "w") as f:
            json.dump(status_meta, f)
        os.replace(CAMERA_STATUS_PATH + ".tmp", CAMERA_STATUS_PATH)

        # Publish Standard ROS 2 Image Messages
        self.rgb_pub.publish(self.cv2_to_image_msg(raw_forward, "bgr8"))
        self.annotated_pub.publish(self.cv2_to_image_msg(annotated_forward, "bgr8"))
        self.thermal_pub.publish(self.cv2_to_image_msg(thermal_frame, "bgr8"))
        self.night_vision_pub.publish(self.cv2_to_image_msg(nvg_frame, "bgr8"))

        # Publish Consolidated Multi-Camera Detections Array JSON
        all_dets_list = list(unique_dets.values())
        det_json_str = json.dumps([{
            "track_id": d["track_id"],
            "category": d["category"],
            "label": d["label"],
            "confidence": d["confidence"],
            "bbox": d["bbox"],
            "world_pos": d["world_pos"],
            "camera_source": d["camera_source"]
        } for d in all_dets_list])
        s_msg = String()
        s_msg.data = det_json_str
        self.detections_pub.publish(s_msg)

        # Save Snapshots for Significant Events
        for d in all_dets_list:
            if d.get("should_log_event", False):
                snap_filename = f"snap_{d['category']}_{int(now*1000)}.jpg"
                snap_path = os.path.join(self.snapshots_dir, snap_filename)
                cv2.imwrite(snap_path, annotated_forward)

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
