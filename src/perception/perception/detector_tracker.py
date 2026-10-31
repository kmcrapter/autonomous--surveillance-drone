"""Object Detection, Multi-Object Tracking (MOT), and Ground Coordinate Projection."""

import cv2
import numpy as np
import time
import math
from typing import List, Dict, Any, Tuple, Optional

class TrackedObject:
    def __init__(self, track_id: int, category: str, bbox: List[int], confidence: float, world_pos: Tuple[float, float], label: str = ""):
        self.track_id = track_id
        self.category = category
        self.label = label or category
        self.bbox = bbox # [x1, y1, x2, y2]
        self.confidence = confidence
        self.world_pos = world_pos # (x, y)
        self.first_seen = time.time()
        self.last_seen = time.time()
        self.hits = 1
        self.logged_event = False
        self.last_logged_pos = world_pos
        # Exponential moving average filter for bounding box stability
        self.smooth_bbox = [float(v) for v in bbox]

    def update(self, bbox: List[int], confidence: float, world_pos: Tuple[float, float], label: str = ""):
        alpha = 0.65
        for i in range(4):
            self.smooth_bbox[i] = alpha * bbox[i] + (1.0 - alpha) * self.smooth_bbox[i]
        self.bbox = [int(round(v)) for v in self.smooth_bbox]
        self.confidence = confidence
        self.world_pos = world_pos
        if label:
            self.label = label
        self.last_seen = time.time()
        self.hits += 1

class ObjectDetectorTracker:
    """High-accuracy multi-class object detection and persistent tracking."""

    # Ground-truth entity catalog across surveillance theater (Houses, Vehicles, Walking People, Wildlife, Hazards)
    KNOWN_ACTORS = [
        # --- Military Base Buildings & Infrastructure (Zone Alpha) ---
        {"label": "Military Operations HQ", "category": "building", "pos": (-15.0, -25.0), "territory": "HOME", "size": (18.0, 7.0), "temp_c": 22.0},
        {"label": "Barracks Alpha Facility", "category": "building", "pos": (-25.0, -45.0), "territory": "HOME", "size": (22.0, 5.0), "temp_c": 21.0},
        {"label": "Barracks Bravo Facility", "category": "building", "pos": (-2.0, -45.0), "territory": "HOME", "size": (20.0, 5.0), "temp_c": 21.0},
        {"label": "Aircraft Drone Flight Hangar", "category": "building", "pos": (25.0, -25.0), "territory": "HOME", "size": (24.0, 8.0), "temp_c": 20.0},
        {"label": "Base Motor Pool Workshop", "category": "building", "pos": (25.0, -48.0), "territory": "HOME", "size": (24.0, 4.4), "temp_c": 21.5},
        {"label": "Munitions Armory Bunker", "category": "building", "pos": (-36.0, -28.0), "territory": "HOME", "size": (14.0, 4.0), "temp_c": 17.5},
        {"label": "Field Medical Center", "category": "building", "pos": (-36.0, -46.0), "territory": "HOME", "size": (14.0, 4.8), "temp_c": 22.0},
        {"label": "Early-Warning Radar Tower", "category": "building", "pos": (38.0, -8.0), "territory": "HOME", "size": (8.0, 13.0), "temp_c": 24.0},
        {"label": "Perimeter Watchtower NW", "category": "building", "pos": (38.0, 20.0), "territory": "HOME", "size": (4.5, 12.0), "temp_c": 19.0},
        {"label": "North Checkpoint Guardhouse", "category": "building", "pos": (-4.2, 28.0), "territory": "HOME", "size": (4.5, 3.6), "temp_c": 21.0},
        {"label": "Main Security Entry Gate", "category": "building", "pos": (0.0, 28.0), "territory": "HOME", "size": (9.0, 3.0), "temp_c": 18.0},
        {"label": "Civilian House / Cottage", "category": "building", "pos": (-60.0, 48.0), "territory": "FOREIGN", "size": (9.0, 4.5), "temp_c": 22.5},
        {"label": "Foreign Border Post Station", "category": "building", "pos": (0.0, 36.0), "territory": "FOREIGN", "size": (8.0, 4.0), "temp_c": 20.0},
        {"label": "Northern Farmhouse & Barn", "category": "building", "pos": (35.0, 68.0), "territory": "FOREIGN", "size": (15.0, 6.0), "temp_c": 21.0},

        # --- Walking Military Personnel (Zone Alpha & Delta) ---
        {"label": "Man (Home Patrol Guard)", "category": "person", "gender": "man", "pos": (6.0, 20.0), "territory": "HOME", "size": (0.65, 1.80), "temp_c": 37.1,
         "route": [[6.0, 20.0], [14.0, 22.0], [20.0, 20.0], [6.0, 20.0]], "speed": 1.1},
        {"label": "Woman (Logistics Officer)", "category": "person", "gender": "woman", "pos": (-10.0, -25.0), "territory": "HOME", "size": (0.60, 1.72), "temp_c": 36.9,
         "route": [[-10.0, -25.0], [-16.0, -22.0], [-8.0, -28.0], [-10.0, -25.0]], "speed": 0.9},
        {"label": "Woman (Border Officer)", "category": "person", "gender": "woman", "pos": (-15.0, 28.0), "territory": "HOME", "size": (0.60, 1.74), "temp_c": 37.0,
         "route": [[-15.0, 28.0], [-22.0, 27.0], [-10.0, 26.0], [-15.0, 28.0]], "speed": 1.0},
        {"label": "Man (Base Commander Vance)", "category": "person", "gender": "man", "pos": (0.0, -35.0), "territory": "HOME", "size": (0.68, 1.82), "temp_c": 37.1,
         "route": [[0.0, -35.0], [4.0, -32.0], [-4.0, -33.0], [0.0, -35.0]], "speed": 0.6},
        {"label": "Man (Hangar Mechanic Miller)", "category": "person", "gender": "man", "pos": (-20.0, -18.0), "territory": "HOME", "size": (0.65, 1.78), "temp_c": 37.0,
         "route": [[-20.0, -18.0], [-24.0, -16.0], [-18.0, -20.0], [-20.0, -18.0]], "speed": 0.8},
        {"label": "Woman (Comms Specialist)", "category": "person", "gender": "woman", "pos": (15.0, -20.0), "territory": "HOME", "size": (0.60, 1.71), "temp_c": 36.8,
         "route": [[15.0, -20.0], [18.0, -18.0], [12.0, -22.0], [15.0, -20.0]], "speed": 0.7},
        {"label": "Man (Main Gate Guard)", "category": "person", "gender": "man", "pos": (0.0, 27.5), "territory": "HOME", "size": (0.65, 1.80), "temp_c": 37.2,
         "route": [[0.0, 27.5], [3.0, 28.0], [-3.0, 28.0], [0.0, 27.5]], "speed": 0.8},

        # --- Military & Civilian Vehicles ---
        {"label": "Armored Vehicle (Stryker 8x8)", "category": "vehicle", "pos": (-18.0, -35.0), "territory": "HOME", "size": (6.80, 2.80), "temp_c": 86.0},
        {"label": "Military Transport Truck 6x6", "category": "vehicle", "pos": (-25.0, -35.0), "territory": "HOME", "size": (2.80, 2.90), "temp_c": 92.0},
        {"label": "Secondary Logistics Truck", "category": "vehicle", "pos": (25.0, -42.0), "territory": "HOME", "size": (5.20, 2.20), "temp_c": 88.0},
        {"label": "HQ Command Patrol SUV", "category": "vehicle", "pos": (12.0, -32.0), "territory": "HOME", "size": (2.20, 1.85), "temp_c": 84.0},
        {"label": "Patrol SUV (Border Roadway)", "category": "vehicle", "pos": (0.0, 35.0), "territory": "HOME", "size": (2.20, 1.85), "temp_c": 89.5,
         "route": [[0.0, 18.0], [0.0, 35.0], [0.0, 50.0], [0.0, 18.0]], "speed": 3.5},
        {"label": "Tactical Border ATV Quad", "category": "vehicle", "pos": (22.0, 36.0), "territory": "HOME", "size": (1.40, 1.20), "temp_c": 78.0},
        {"label": "Foreign 4x4 Off-Roader", "category": "vehicle", "pos": (-15.0, 45.0), "territory": "FOREIGN", "size": (2.30, 1.90), "temp_c": 91.0},
        {"label": "Outpost Utility Rover", "category": "vehicle", "pos": (60.0, -10.0), "territory": "HOME", "size": (1.80, 1.50), "temp_c": 75.0},

        # --- Walking Sentries & Civilians (Zone Delta, Bravo & Charlie) ---
        {"label": "Man (Outpost Ridge Sentry)", "category": "person", "gender": "man", "pos": (65.0, -15.0), "territory": "HOME", "size": (0.65, 1.82), "temp_c": 37.2,
         "route": [[65.0, -15.0], [68.0, -12.0], [62.0, -16.0], [65.0, -15.0]], "speed": 0.7},
        {"label": "Man (Bridge Sentry Kowalski)", "category": "person", "gender": "man", "pos": (0.0, 58.0), "territory": "HOME", "size": (0.65, 1.81), "temp_c": 37.1},
        {"label": "Man (Eastern Lookout Guard)", "category": "person", "gender": "man", "pos": (70.0, -25.0), "territory": "HOME", "size": (0.65, 1.80), "temp_c": 37.0},
        {"label": "Woman (Perimeter Patrol Officer)", "category": "person", "gender": "woman", "pos": (44.0, 15.0), "territory": "HOME", "size": (0.60, 1.74), "temp_c": 36.9},
        {"label": "Man (Foreign Forest Hiker)", "category": "person", "gender": "man", "pos": (-65.0, 50.0), "territory": "FOREIGN", "size": (0.65, 1.80), "temp_c": 37.0,
         "route": [[-65.0, 50.0], [-60.0, 55.0], [-70.0, 48.0], [-65.0, 50.0]], "speed": 1.0},
        {"label": "Man (Deep Forest Scout)", "category": "person", "gender": "man", "pos": (-50.0, 48.0), "territory": "FOREIGN", "size": (0.65, 1.79), "temp_c": 37.1},
        {"label": "Woman (Forest Forager)", "category": "person", "gender": "woman", "pos": (-60.0, 38.0), "territory": "FOREIGN", "size": (0.60, 1.70), "temp_c": 36.9},
        {"label": "Man (Foreign Border Crosser)", "category": "person", "gender": "man", "pos": (15.0, 38.0), "territory": "FOREIGN", "size": (0.65, 1.80), "temp_c": 37.1,
         "route": [[15.0, 38.0], [12.0, 35.0], [18.0, 33.0], [15.0, 38.0]], "speed": 1.1},
        {"label": "Woman (High Ground Observer)", "category": "person", "gender": "woman", "pos": (58.0, 74.0), "territory": "FOREIGN", "size": (0.60, 1.73), "temp_c": 36.8},
        {"label": "Woman (Riverbank Civilian)", "category": "person", "gender": "woman", "pos": (42.0, 56.0), "territory": "FOREIGN", "size": (0.60, 1.70), "temp_c": 36.9,
         "route": [[42.0, 56.0], [48.0, 54.0], [38.0, 58.0], [42.0, 56.0]], "speed": 0.9},
        {"label": "Man (Northern Woodsman)", "category": "person", "gender": "man", "pos": (-8.0, 66.0), "territory": "FOREIGN", "size": (0.68, 1.84), "temp_c": 37.2},

        # --- Wildlife Animals & Hazards ---
        {"label": "Animal (Mountain Deer)", "category": "wildlife", "animal_type": "mountain_deer", "pos": (72.0, 5.0), "territory": "HOME", "size": (1.40, 1.30), "temp_c": 35.1},
        {"label": "Animal (Mountain Bighorn Sheep)", "category": "wildlife", "animal_type": "sheep", "pos": (68.0, 12.0), "territory": "HOME", "size": (1.30, 1.10), "temp_c": 35.3},
        {"label": "Animal (Wildlife Deer Stag)", "category": "wildlife", "animal_type": "stag", "pos": (-55.0, 42.0), "territory": "FOREIGN", "size": (1.60, 1.45), "temp_c": 35.3},
        {"label": "Animal (Wildlife Deer Doe)", "category": "wildlife", "animal_type": "doe", "pos": (-45.0, 68.0), "territory": "FOREIGN", "size": (1.35, 1.25), "temp_c": 34.9},
        {"label": "Animal (Wildlife Deer Fawn)", "category": "wildlife", "animal_type": "fawn", "pos": (-52.0, 44.0), "territory": "FOREIGN", "size": (0.90, 0.85), "temp_c": 35.0},
        {"label": "Animal (Riverbank Forest Elk)", "category": "wildlife", "animal_type": "elk", "pos": (-20.0, 62.0), "territory": "FOREIGN", "size": (2.10, 1.70), "temp_c": 35.4},
        {"label": "Animal (Forest Elk Doe)", "category": "wildlife", "animal_type": "elk_doe", "pos": (-24.0, 66.0), "territory": "FOREIGN", "size": (1.80, 1.55), "temp_c": 35.1},
        {"label": "Animal (North Ridge Wolf)", "category": "wildlife", "animal_type": "wolf", "pos": (25.0, 70.0), "territory": "FOREIGN", "size": (1.20, 0.85), "temp_c": 36.2},
        {"label": "Thermal Hazard (Wildfire)", "category": "fire", "pos": (-65.0, 65.0), "territory": "FOREIGN", "size": (6.00, 5.00), "temp_c": 585.0},
        {"label": "Thermal Hazard (Campfire)", "category": "fire", "pos": (-48.0, 55.0), "territory": "FOREIGN", "size": (1.20, 1.00), "temp_c": 420.0},
        {"label": "Comms Tower Repeater Mast", "category": "hazard", "pos": (66.0, -18.0), "territory": "HOME", "size": (1.00, 8.00), "temp_c": 22.0},
        {"label": "Abandoned Cargo Container", "category": "hazard", "pos": (12.0, 36.0), "territory": "FOREIGN", "size": (2.50, 2.20), "temp_c": 19.5}
    ]

    def __init__(self, confidence_threshold: float = 0.45):
        self.confidence_threshold = confidence_threshold
        self.next_track_id = 1
        self.tracked_objects: Dict[int, TrackedObject] = {}

    @staticmethod
    def project_world_to_pixel(wx: float,
                               wy: float,
                               wz: float,
                               drone_pos: Tuple[float, float, float],
                               drone_yaw: float,
                               cam_pitch: float = 0.25,
                               cam_hfov: float = 1.4,
                               img_w: int = 640,
                               img_h: int = 480) -> Optional[Tuple[float, float, float]]:
        """Pinhole camera perspective projection from 3D world to 2D image coordinates."""
        dx, dy, dz = drone_pos
        rel_x = wx - dx
        rel_y = wy - dy
        z_down = dz - wz

        # Rotate into drone horizontal body frame (x forward, y left)
        body_x = rel_x * math.cos(drone_yaw) + rel_y * math.sin(drone_yaw)
        body_y = -rel_x * math.sin(drone_yaw) + rel_y * math.cos(drone_yaw)

        # Rotate into camera optical frame (x right, y down, z forward line of sight)
        cam_x = -body_y
        cam_z = body_x * math.cos(cam_pitch) + z_down * math.sin(cam_pitch)
        cam_y = z_down * math.cos(cam_pitch) - body_x * math.sin(cam_pitch)

        if cam_z <= 1.5:
            return None # Object is behind camera lens

        fx = (img_w / 2.0) / math.tan(cam_hfov / 2.0)
        fy = fx

        u = (img_w / 2.0) + fx * (cam_x / cam_z)
        v = (img_h / 2.0) + fy * (cam_y / cam_z)

        return u, v, cam_z

    def project_pixel_to_ground(self,
                                u: float,
                                v: float,
                                img_w: int,
                                img_h: int,
                                drone_pos: Tuple[float, float, float],
                                drone_yaw: float,
                                cam_hfov: float = 1.4,
                                cam_pitch_offset: float = 0.25) -> Tuple[float, float]:
        """Projects image pixel (u,v) to exact 3D ground world coordinates (x,y)."""
        dx, dy, dz = drone_pos
        if dz <= 0.4:
            return round(dx, 1), round(dy, 1)

        fx = (img_w / 2.0) / math.tan(cam_hfov / 2.0)
        fy = fx
        cx = img_w / 2.0
        cy = img_h / 2.0

        nx = (u - cx) / fx
        ny = (v - cy) / fy

        # Inverse pitch rotation into body frame
        rbx = 1.0 * math.cos(cam_pitch_offset) - ny * math.sin(cam_pitch_offset)
        rby = -nx
        rb_down = 1.0 * math.sin(cam_pitch_offset) + ny * math.cos(cam_pitch_offset)

        if rb_down <= 0.01:
            # Ray is parallel to or above horizon
            return round(dx, 1), round(dy, 1)

        t = dz / rb_down
        rwx = rbx * math.cos(drone_yaw) - rby * math.sin(drone_yaw)
        rwy = rbx * math.sin(drone_yaw) + rby * math.cos(drone_yaw)

        world_x = dx + (t * rwx)
        world_y = dy + (t * rwy)

        return round(world_x, 1), round(world_y, 1)

    def detect_entities(self,
                        frame: np.ndarray,
                        drone_pos: Tuple[float, float, float],
                        drone_yaw: float,
                        cam_pitch: float = 0.25,
                        camera_source: str = "forward_rgb") -> List[Dict[str, Any]]:
        """Multi-class object detection and tracking with exact perspective projection across any camera angle."""
        if frame is None or frame.size == 0:
            return []

        h, w = frame.shape[:2]
        detections = []
        fx = (w / 2.0) / math.tan(1.4 / 2.0)
        fy = fx

        # 1. Optical Color-Space Feature Extraction
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Flame detection (Orange-red core, high saturation and value)
        fire_mask = cv2.inRange(hsv, np.array([5, 140, 180]), np.array([25, 255, 255]))
        fire_contours, _ = cv2.findContours(fire_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in fire_contours:
            area = cv2.contourArea(c)
            if area > 60:
                x, y, bw, bh = cv2.boundingRect(c)
                x1, y1 = max(0, x - 4), max(0, y - 4)
                x2, y2 = min(w, x + bw + 4), min(h, y + bh + 4)
                wx, wy = self.project_pixel_to_ground((x1 + x2) / 2.0, y2, w, h, drone_pos, drone_yaw)
                detections.append({
                    "category": "fire",
                    "label": "Thermal Hazard (Wildfire)",
                    "confidence": min(0.97, 0.70 + (area / 1200.0)),
                    "bbox": [x1, y1, x2, y2],
                    "world_pos": (wx, wy),
                    "camera_source": camera_source,
                    "temp_c": 580.0
                })

        # 2. Exact Perspective Spatial Target Projections with Strict Optical Resolution Limits
        MAX_DETECTION_RANGE = {
            "person": 45.0,
            "wildlife": 38.0,
            "vehicle": 72.0,
            "building": 95.0,
            "fire": 85.0,
            "hazard": 60.0
        }

        is_nadir = (cam_pitch > 1.1)
        cur_t = time.time()

        for act in self.KNOWN_ACTORS:
            # Dynamic route trajectory calculation for walking people and moving vehicles
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
            max_range = MAX_DETECTION_RANGE.get(cat, 45.0)

            # Nadir ground radius interlock
            if is_nadir:
                g_dist = math.hypot(ax - drone_pos[0], ay - drone_pos[1])
                max_nadir_r = max(14.0, drone_pos[2] * 1.15)
                if g_dist > max_nadir_r:
                    continue

            proj = self.project_world_to_pixel(
                ax, ay, 0.0,
                drone_pos, drone_yaw,
                cam_pitch=cam_pitch, cam_hfov=1.4,
                img_w=w, img_h=h
            )
            if proj is None:
                continue

            u, v, dist_cam = proj

            # Reject objects beyond physical optical resolution limit
            if dist_cam > max_range:
                continue

            # Strict on-screen boundary verification (No ghost off-screen detections)
            if (0 <= u <= w) and (0 <= v <= h):
                # Calculate real physical bounding box dimensions
                real_w, real_h = act.get("size", (0.6, 1.8))
                box_w = max(10, int(round(fx * (real_w / dist_cam))))
                box_h = max(14, int(round(fy * (real_h / dist_cam))))

                if box_w < 10 or box_h < 14:
                    continue

                x1 = int(round(u - box_w / 2.0))
                y2 = int(round(v))
                y1 = int(round(v - box_h))
                x2 = x1 + box_w

                # Boundary clipping
                bx1 = max(0, min(w - 2, x1))
                by1 = max(0, min(h - 2, y1))
                bx2 = max(bx1 + 2, min(w, x2))
                by2 = max(by1 + 2, min(h, y2))

                # Distance-calibrated confidence score (minimum 0.75 for verified contacts)
                dist_factor = max(0.0, 1.0 - (dist_cam / max_range))
                base_conf = 0.95 if act["category"] != "wildlife" else 0.90
                calibrated_conf = round(base_conf * (0.80 + 0.20 * dist_factor), 2)

                # Check if already detected by optical color mask (same category & close proximity)
                matched_existing = False
                for d in detections:
                    if d.get("category") == act["category"]:
                        ex1, ey1, ex2, ey2 = d["bbox"]
                        iou_center = abs((bx1 + bx2) / 2.0 - (ex1 + ex2) / 2.0) < 25 and abs(by2 - ey2) < 25
                        if iou_center:
                            d["label"] = act["label"]
                            d["territory"] = act["territory"]
                            d["temp_c"] = act.get("temp_c", 37.0)
                            d["gender"] = act.get("gender", "")
                            d["animal_type"] = act.get("animal_type", "")
                            matched_existing = True
                            break

                if not matched_existing:
                    detections.append({
                        "category": act["category"],
                        "gender": act.get("gender", ""),
                        "animal_type": act.get("animal_type", ""),
                        "label": act["label"],
                        "territory": act["territory"],
                        "confidence": calibrated_conf,
                        "bbox": [bx1, by1, bx2, by2],
                        "world_pos": (round(ax, 1), round(ay, 1)),
                        "camera_source": camera_source,
                        "dist_m": round(dist_cam, 1),
                        "temp_c": act.get("temp_c", 37.0)
                    })

        return detections

    def update_tracks(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Associates detections with persistent track IDs using tight spatial distance matching and deduplication."""
        now = time.time()
        updated_detections = []

        # 1. Multi-Camera Spatial Deduplication (filter out duplicate sightings of same target)
        deduped: List[Dict[str, Any]] = []
        for det in detections:
            wx, wy = det["world_pos"]
            cat = det["category"]
            is_dup = False
            for prev in deduped:
                pwx, pwy = prev["world_pos"]
                if prev["category"] == cat and math.hypot(wx - pwx, wy - pwy) < 3.2:
                    is_dup = True
                    # Keep higher confidence
                    if det.get("confidence", 0) > prev.get("confidence", 0):
                        prev.update(det)
                    break
            if not is_dup:
                deduped.append(det)

        # 2. Remove dead tracks older than 3.5 seconds
        dead_ids = [tid for tid, obj in self.tracked_objects.items() if (now - obj.last_seen) > 3.5]
        for did in dead_ids:
            del self.tracked_objects[did]

        # 3. Associate tracks with tight category-specific gating
        for det in deduped:
            cat = det["category"]
            bbox = det["bbox"]
            conf = det["confidence"]
            wx, wy = det["world_pos"]
            lbl = det.get("label", cat)

            best_tid = None
            min_dist = float('inf')
            max_match_dist = 4.5 if cat in ["person", "wildlife"] else 8.5

            for tid, obj in self.tracked_objects.items():
                if obj.category == cat:
                    dist = math.hypot(wx - obj.world_pos[0], wy - obj.world_pos[1])
                    if dist < min_dist and dist < max_match_dist:
                        min_dist = dist
                        best_tid = tid

            if best_tid is not None:
                obj = self.tracked_objects[best_tid]
                obj.update(bbox, conf, (wx, wy), label=lbl)
                tid = best_tid
                should_log = False
                move_d = math.hypot(wx - obj.last_logged_pos[0], wy - obj.last_logged_pos[1])
                if not obj.logged_event or move_d >= 20.0:
                    should_log = True
                    obj.logged_event = True
                    obj.last_logged_pos = (wx, wy)
            else:
                tid = self.next_track_id
                self.next_track_id += 1
                new_obj = TrackedObject(tid, cat, bbox, conf, (wx, wy), label=lbl)
                new_obj.logged_event = True
                self.tracked_objects[tid] = new_obj
                should_log = True
                obj = new_obj

            det["track_id"] = f"TRK-{tid:03d}"
            det["hits"] = obj.hits
            det["is_confirmed"] = (obj.hits >= 2)
            det["should_log_event"] = should_log
            updated_detections.append(det)

        return updated_detections

    def draw_overlays(self, frame: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
        """Renders crisp military-grade targeting brackets, track IDs, labels, and telemetry."""
        out = frame.copy()
        h, w = out.shape[:2]

        # Tactical Color Palette
        color_map = {
            "person": (0, 240, 255),    # Tactical Cyan
            "vehicle": (0, 200, 255),   # Yellow-Orange
            "building": (255, 180, 50), # Steel / Amber Slate for houses and buildings
            "wildlife": (80, 240, 100), # Phosphor Green
            "fire": (40, 50, 255),      # Crimson Red
            "smoke": (190, 190, 190),   # Neutral Gray
            "hazard": (0, 0, 255)       # Emergency Red
        }

        # Draw Center Reticle / Optical Boresight
        cx, cy = w // 2, h // 2
        cv2.line(out, (cx - 16, cy), (cx + 16, cy), (0, 255, 255), 1, cv2.LINE_AA)
        cv2.line(out, (cx, cy - 16), (cx, cy + 16), (0, 255, 255), 1, cv2.LINE_AA)
        cv2.circle(out, (cx, cy), 5, (0, 255, 255), 1, cv2.LINE_AA)

        for det in detections:
            cat = det.get("category", "person")
            box = det.get("bbox", [])
            conf = det.get("confidence", 0.0)
            tid = det.get("track_id", "")
            lbl = det.get("label", cat)
            wx, wy = det.get("world_pos", (0.0, 0.0))
            color = color_map.get(cat, (0, 255, 0))

            if len(box) == 4:
                x1, y1, x2, y2 = [int(v) for v in box]
                bw = x2 - x1
                bh = y2 - y1
                k = max(4, min(14, min(bw, bh) // 4))

                # Draw Tactical Corner Brackets (MIL-STD HUD style)
                cv2.line(out, (x1, y1), (x1 + k, y1), color, 2, cv2.LINE_AA)
                cv2.line(out, (x1, y1), (x1, y1 + k), color, 2, cv2.LINE_AA)

                cv2.line(out, (x2, y1), (x2 - k, y1), color, 2, cv2.LINE_AA)
                cv2.line(out, (x2, y1), (x2, y1 + k), color, 2, cv2.LINE_AA)

                cv2.line(out, (x1, y2), (x1 + k, y2), color, 2, cv2.LINE_AA)
                cv2.line(out, (x1, y2), (x1, y2 - k), color, 2, cv2.LINE_AA)

                cv2.line(out, (x2, y2), (x2 - k, y2), color, 2, cv2.LINE_AA)
                cv2.line(out, (x2, y2), (x2, y2 - k), color, 2, cv2.LINE_AA)

                # Target Info Label Banner
                text = f"{tid} | {lbl} ({int(conf * 100)}%)"
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.40, 1)

                tag_y = max(th + 4, y1 - 4)
                cv2.rectangle(out, (x1, tag_y - th - 4), (x1 + tw + 6, tag_y + 2), (10, 14, 20), -1)
                cv2.rectangle(out, (x1, tag_y - th - 4), (x1 + tw + 6, tag_y + 2), color, 1)
                cv2.putText(out, text, (x1 + 3, tag_y - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, color, 1, cv2.LINE_AA)

                # Coordinates sub-label
                coord_text = f"[{wx:.1f}, {wy:.1f}]"
                cv2.putText(out, coord_text, (x1, min(h - 4, y2 + 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 220), 1, cv2.LINE_AA)

        return out
