"""ROS 2 Node for Visual Identity Verification & Face Quality Assessment (Layer 2).

Provides Image Quality Assessment (IQA), environmental obscuration checks (fog/night/distance),
synthetic facial biometric template matching, and non-hostile confidence scoring.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
import numpy as np
import json
import time
from typing import Dict, Any, List, Optional, Tuple

from personnel_security.personnel_registry import (
    PersonnelRegistryManager,
    _generate_synthetic_embedding
)

class FaceVerificationNode(Node):
    def __init__(self):
        super().__init__('face_verification_node')

        self.last_rgb_frame: Optional[np.ndarray] = None
        self.pending_operator_requests: List[Dict[str, Any]] = []

        # Subscriptions
        self.cam_sub = self.create_subscription(
            Image,
            '/drone/camera/rgb/image_raw',
            self.cam_callback,
            qos_profile_sensor_data
        )

        self.detections_sub = self.create_subscription(
            String,
            '/drone/detections',
            self.detections_callback,
            10
        )

        self.verify_req_sub = self.create_subscription(
            String,
            '/personnel/verify_request',
            self.verify_request_callback,
            10
        )

        # Publisher for Visual Verification Results
        self.visual_pub = self.create_publisher(
            String,
            '/personnel/visual_verification',
            10
        )

        self.get_logger().info("FaceVerificationNode active (Layer 2 Biometric & Image Quality Engine).")

    def cam_callback(self, msg: Image):
        try:
            h, w = msg.height, msg.width
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding in ['bgr8', 'BGR8']:
                self.last_rgb_frame = arr.reshape((h, w, 3))
            elif msg.encoding in ['rgb8', 'RGB8']:
                bgr = cv2.cvtColor(arr.reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
                self.last_rgb_frame = bgr
        except Exception:
            pass

    def verify_request_callback(self, msg: String):
        try:
            req = json.loads(msg.data)
            self.pending_operator_requests.append(req)
            self.get_logger().info(f"Received operator-initiated visual verification request: {req}")
        except Exception as e:
            self.get_logger().warn(f"Failed to parse verify request: {e}")

    def assess_image_quality(self, crop_roi: Optional[np.ndarray], world_pos: Tuple[float, float]) -> Dict[str, Any]:
        """Calculates sharpness, lighting, contrast, and environmental obscuration."""
        wx, wy = world_pos
        is_dense_forest_fog = (wx < -30.0 and wy > 35.0) or (wy > 70.0)

        if crop_roi is None or crop_roi.size == 0:
            return {
                "quality_passed": False,
                "sharpness_score": 45.0 if not is_dense_forest_fog else 15.0,
                "lighting_level": 50.0 if not is_dense_forest_fog else 20.0,
                "contrast": 35.0 if not is_dense_forest_fog else 12.0,
                "face_visible": not is_dense_forest_fog,
                "reason": "DENSE_FOREST_CANOPY_AND_FOG_OBSCURATION" if is_dense_forest_fog else "SIMULATED_DISTANCE_CROP"
            }

        h, w = crop_roi.shape[:2]
        gray = cv2.cvtColor(crop_roi, cv2.COLOR_BGR2GRAY)

        lap_var = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        sharpness_score = round(min(100.0, lap_var), 1)
        mean_lum = float(np.mean(gray))
        contrast = float(np.std(gray))

        face_visible = True
        reasons = []

        if min(w, h) < 18:
            face_visible = False
            reasons.append("RESOLUTION_TOO_LOW_LONG_DISTANCE")

        if mean_lum < 28.0:
            face_visible = False
            reasons.append("LOW_LIGHTING_NIGHT_CONDITIONS")

        if lap_var < 20.0:
            face_visible = False
            reasons.append("MOTION_BLUR_OR_OUT_OF_FOCUS")

        if is_dense_forest_fog:
            face_visible = False
            reasons.append("DENSE_FOREST_CANOPY_AND_FOG_OBSCURATION")

        quality_passed = face_visible and (lap_var >= 35.0) and (mean_lum >= 35.0)

        return {
            "quality_passed": quality_passed,
            "sharpness_score": sharpness_score,
            "lighting_level": round(mean_lum, 1),
            "contrast": round(contrast, 1),
            "face_visible": face_visible,
            "reason": "; ".join(reasons) if reasons else "OPTIMAL_FACIAL_VISIBILITY"
        }

    def detections_callback(self, msg: String):
        now = time.time()
        try:
            dets = json.loads(msg.data)
        except Exception:
            return

        person_dets = [d for d in dets if d.get("category") == "person"]
        if not person_dets:
            return

        verification_results = []

        for d in person_dets:
            track_id = d.get("track_id", 0)
            bbox = d.get("bbox", [0, 0, 10, 10])
            wpos = (d.get("world_pos", [0.0, 0.0])[0], d.get("world_pos", [0.0, 0.0])[1])

            # Extract crop if frame is available
            crop = None
            if self.last_rgb_frame is not None:
                x1, y1, x2, y2 = bbox
                ih, iw = self.last_rgb_frame.shape[:2]
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(iw, x2), min(ih, y2)
                if x2 > x1 + 5 and y2 > y1 + 5:
                    crop = self.last_rgb_frame[y1:y2, x1:x2]

            iqa = self.assess_image_quality(crop, wpos)

            if not iqa["face_visible"]:
                verification_results.append({
                    "track_id": track_id,
                    "world_pos": {"x": wpos[0], "y": wpos[1]},
                    "face_visible": False,
                    "image_quality": iqa,
                    "match_outcome": "INSUFFICIENT_QUALITY",
                    "visual_identity_status": "UNVERIFIED",
                    "confidence_score": 0.25,
                    "matched_personnel_id": None,
                    "matched_profile": None,
                    "notes": f"Facial recognition inconclusive: {iqa['reason']}. Defaulting safely to UNVERIFIED.",
                    "timestamp": now
                })
            else:
                sim_seed = 999
                dist_vance = ((wpos[0] - 6.0)**2 + (wpos[1] - 20.0)**2)**0.5
                dist_mitchell = ((wpos[0] + 10.0)**2 + (wpos[1] + 25.0)**2)**0.5
                dist_kowalski = ((wpos[0] - 65.0)**2 + (wpos[1] + 15.0)**2)**0.5
                dist_rostova = ((wpos[0] + 15.0)**2 + (wpos[1] - 28.0)**2)**0.5
                dist_miller = ((wpos[0] + 8.0)**2 + (wpos[1] - 66.0)**2)**0.5

                if dist_vance < 14.0: sim_seed = 101
                elif dist_mitchell < 14.0: sim_seed = 102
                elif dist_kowalski < 14.0: sim_seed = 103
                elif dist_rostova < 14.0: sim_seed = 104
                elif dist_miller < 14.0: sim_seed = 105

                base_vec = np.array(_generate_synthetic_embedding(sim_seed))
                noise = np.random.RandomState(int(now * 10) % 1000).randn(128) * 0.08
                test_vec = (base_vec + noise).tolist()

                matched_profile, score = PersonnelRegistryManager.match_biometric_embedding(test_vec, threshold=0.75)

                if matched_profile:
                    verification_results.append({
                        "track_id": track_id,
                        "world_pos": {"x": wpos[0], "y": wpos[1]},
                        "face_visible": True,
                        "image_quality": iqa,
                        "match_outcome": "MATCH",
                        "visual_identity_status": "VISUAL_VERIFIED",
                        "confidence_score": score,
                        "matched_personnel_id": matched_profile["personnel_id"],
                        "matched_profile": matched_profile,
                        "notes": f"Visual identity confirmed: matched {matched_profile['full_name']} ({matched_profile['assigned_unit']}) with {score*100:.1f}% confidence.",
                        "timestamp": now
                    })
                else:
                    verification_results.append({
                        "track_id": track_id,
                        "world_pos": {"x": wpos[0], "y": wpos[1]},
                        "face_visible": True,
                        "image_quality": iqa,
                        "match_outcome": "NO_MATCH",
                        "visual_identity_status": "UNVERIFIED",
                        "confidence_score": score,
                        "matched_personnel_id": None,
                        "matched_profile": None,
                        "notes": f"Face visible but no match in consented personnel database (score: {score:.2f}). Marked as UNVERIFIED.",
                        "timestamp": now
                    })

        res_msg = String()
        res_msg.data = json.dumps({"timestamp": now, "results": verification_results})
        self.visual_pub.publish(res_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FaceVerificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
