"""Night-Vision Image Intensifier (NVG I²) Processing Pipeline with Target Tracking."""

import cv2
import numpy as np
from typing import List, Dict, Any, Optional

class NightVisionProcessor:
    """Simulates realistic Generation-3 Image Intensifier night-vision optics with tactical targeting."""

    def __init__(self, mode: str = "green_p43", gain: float = 1.7):
        self.mode = mode # 'green_p43' or 'white_p45'
        self.gain = gain
        self._vignette_mask = None

    def _get_vignette(self, shape):
        h, w = shape[:2]
        if self._vignette_mask is not None and self._vignette_mask.shape == (h, w):
            return self._vignette_mask

        # Radial falloff mask
        kernel_x = cv2.getGaussianKernel(w, w * 0.45)
        kernel_y = cv2.getGaussianKernel(h, h * 0.45)
        kernel = kernel_y * kernel_x.T
        mask = kernel / kernel.max()
        self._vignette_mask = np.clip(mask * 1.15, 0.2, 1.0)
        return self._vignette_mask

    def process_frame(self, rgb_frame: np.ndarray, detections: Optional[List[Dict[str, Any]]] = None) -> np.ndarray:
        """Applies night-vision intensification, phosphor response, noise, vignette, and tactical target brackets."""
        if rgb_frame is None or rgb_frame.size == 0:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        h, w = rgb_frame.shape[:2]
        gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY).astype(np.float32)

        # 1. Amplification Gain and Gamma correction
        intensified = gray * self.gain
        # Nonlinear toe compression (analog tube saturation curve)
        intensified = 255.0 * (1.0 - np.exp(-intensified / 128.0))

        # 2. Simulate Active IR Illuminator Reflection on Targets
        if detections:
            for det in detections:
                box = det.get("bbox", [])
                if len(box) == 4:
                    x1, y1, x2, y2 = [int(v) for v in box]
                    x1, y1 = max(0, min(w - 2, x1)), max(0, min(h - 2, y1))
                    x2, y2 = max(x1 + 2, min(w, x2)), max(y1 + 2, min(h, y2))
                    # Biological skin/clothing retro-reflection under 850nm IR illuminator
                    intensified[y1:y2, x1:x2] = np.clip(intensified[y1:y2, x1:x2] * 1.35 + 30.0, 0, 255)

        # 3. Add realistic Microchannel Plate (MCP) scintillation grain
        noise = np.random.normal(0, 12.0, (h, w)).astype(np.float32)
        intensified = np.clip(intensified + noise, 0, 255)

        # 4. Optical vignette radial falloff
        vignette = self._get_vignette((h, w))
        intensified = intensified * vignette

        intensified_uint8 = np.clip(intensified, 0, 255).astype(np.uint8)

        # 5. Phosphor Spectral Mapping
        if self.mode == "white_p45":
            # P45 White Phosphor (cool blue-tinted monochrome)
            nvg_frame = np.zeros((h, w, 3), dtype=np.uint8)
            nvg_frame[:, :, 0] = np.clip(intensified_uint8 * 1.05, 0, 255).astype(np.uint8)
            nvg_frame[:, :, 1] = intensified_uint8
            nvg_frame[:, :, 2] = np.clip(intensified_uint8 * 0.90, 0, 255).astype(np.uint8)
            hud_color = (180, 240, 255)
        else:
            # P43 Green Phosphor (classic 545nm green peak)
            nvg_frame = np.zeros((h, w, 3), dtype=np.uint8)
            nvg_frame[:, :, 0] = (intensified_uint8 * 0.15).astype(np.uint8) # Slight blue
            nvg_frame[:, :, 1] = intensified_uint8                           # Intense green
            nvg_frame[:, :, 2] = (intensified_uint8 * 0.10).astype(np.uint8) # Low red
            hud_color = (100, 255, 100)

        # 6. Night Vision HUD Overlay & Reticle
        cv2.putText(nvg_frame, f"NVG GEN-3 I-SQUARE | {self.mode.upper()} | GAIN: {self.gain:.1f}x",
                    (16, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.50, hud_color, 1, cv2.LINE_AA)
        cx, cy = w // 2, h // 2
        cv2.circle(nvg_frame, (cx, cy), 14, hud_color, 1, cv2.LINE_AA)
        cv2.line(nvg_frame, (cx - 6, cy), (cx + 6, cy), hud_color, 1, cv2.LINE_AA)
        cv2.line(nvg_frame, (cx, cy - 6), (cx, cy + 6), hud_color, 1, cv2.LINE_AA)

        # 7. Render Tactical Luminous Phosphor Brackets for All Detections
        if detections:
            for det in detections:
                box = det.get("bbox", [])
                if len(box) == 4:
                    x1, y1, x2, y2 = [int(v) for v in box]
                    bw = x2 - x1
                    bh = y2 - y1
                    k = max(4, min(12, min(bw, bh) // 4))
                    tid = det.get("track_id", "TRK")
                    lbl = det.get("label", "")
                    conf = det.get("confidence", 0.0)
                    dist = det.get("dist_m", 0.0)

                    # Luminous phosphor green brackets
                    cv2.line(nvg_frame, (x1, y1), (x1 + k, y1), hud_color, 2, cv2.LINE_AA)
                    cv2.line(nvg_frame, (x1, y1), (x1, y1 + k), hud_color, 2, cv2.LINE_AA)
                    cv2.line(nvg_frame, (x2, y1), (x2 - k, y1), hud_color, 2, cv2.LINE_AA)
                    cv2.line(nvg_frame, (x2, y1), (x2, y1 + k), hud_color, 2, cv2.LINE_AA)
                    cv2.line(nvg_frame, (x1, y2), (x1 + k, y2), hud_color, 2, cv2.LINE_AA)
                    cv2.line(nvg_frame, (x1, y2), (x1, y2 - k), hud_color, 2, cv2.LINE_AA)
                    cv2.line(nvg_frame, (x2, y2), (x2 - k, y2), hud_color, 2, cv2.LINE_AA)
                    cv2.line(nvg_frame, (x2, y2), (x2, y2 - k), hud_color, 2, cv2.LINE_AA)

                    tag_text = f"[{tid} | {lbl} ({int(conf*100)}%)]"
                    cv2.putText(nvg_frame, tag_text, (x1, max(12, y1 - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.38, hud_color, 1, cv2.LINE_AA)
                    if dist > 0:
                        cv2.putText(nvg_frame, f"RNG: {dist:.1f}m", (x1, min(h - 4, y2 + 12)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, hud_color, 1, cv2.LINE_AA)

        cv2.putText(nvg_frame, "LOW-LIGHT PHOTON AMPLIFICATION ACTIVE",
                    (16, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, hud_color, 1, cv2.LINE_AA)

        return nvg_frame
