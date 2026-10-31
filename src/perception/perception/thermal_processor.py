"""Synthetic Thermal / Infrared Camera Processing Pipeline with Tactical Targeting."""

import cv2
import numpy as np
import time
from typing import List, Dict, Any, Optional

class ThermalProcessor:
    """Simulates calibrated Long-Wave Infrared (LWIR) thermography with tactical targeting overlays."""

    PALETTES = {
        "inferno": cv2.COLORMAP_INFERNO,
        "ironbow": cv2.COLORMAP_JET,
        "white_hot": None,
        "black_hot": None,
        "jet": cv2.COLORMAP_JET,
        "plasma": cv2.COLORMAP_PLASMA,
        "bone": cv2.COLORMAP_BONE
    }

    def __init__(self, default_palette: str = "inferno"):
        self.current_palette = default_palette if default_palette in self.PALETTES else "inferno"

    def set_palette(self, palette_name: str):
        if palette_name in self.PALETTES:
            self.current_palette = palette_name

    def process_frame(self, rgb_frame: np.ndarray, detections: Optional[List[Dict[str, Any]]] = None) -> np.ndarray:
        """Converts an RGB image into calibrated synthetic thermal imagery with tactical target overlays."""
        if rgb_frame is None or rgb_frame.size == 0:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        h, w = rgb_frame.shape[:2]
        gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)

        # Baseline ambient temperature map (approx 18°C to 24°C mapped to 45..90 intensity)
        thermal_raw = cv2.normalize(gray, None, 48, 92, cv2.NORM_MINMAX).astype(np.float32)

        # Segment water features (high blue/cyan ratio -> cold thermal sink 14°C)
        b, g, r = cv2.split(rgb_frame)
        water_mask = (b.astype(np.float32) > (r.astype(np.float32) + 20)) & (g.astype(np.float32) > (r.astype(np.float32) + 10))
        thermal_raw[water_mask] = np.clip(thermal_raw[water_mask] * 0.52, 20.0, 42.0)

        # Inject calibrated radiant thermal signatures for detected entities
        if detections:
            for det in detections:
                box = det.get("bbox", [])
                cat = det.get("category", "")
                if len(box) == 4:
                    x1, y1, x2, y2 = [int(v) for v in box]
                    x1, y1 = max(0, min(w - 2, x1)), max(0, min(h - 2, y1))
                    x2, y2 = max(x1 + 2, min(w, x2)), max(y1 + 2, min(h, y2))
                    bw = x2 - x1
                    bh = y2 - y1
                    sub = thermal_raw[y1:y2, x1:x2]

                    if cat in ["person", "human", "pedestrian"]:
                        # 36.8°C - 37.4°C core metabolic signature
                        head_y = y1 + int(bh * 0.25)
                        head_x = (x1 + x2) // 2
                        cv2.circle(thermal_raw, (head_x, head_y), max(4, bw // 4), 215.0, -1)
                        # Torso heat
                        thermal_raw[y1:y2, x1:x2] = np.clip(sub + 95.0, 115.0, 218.0)
                    elif cat in ["wildlife", "animal"]:
                        # 34.5°C - 35.5°C coat insulated heat
                        thermal_raw[y1:y2, x1:x2] = np.clip(sub + 78.0, 95.0, 192.0)
                    elif cat in ["vehicle", "truck", "car"]:
                        # 85.0°C - 95.0°C engine block and tires
                        thermal_raw[y1:y2, x1:x2] = np.clip(sub + 125.0, 140.0, 238.0)
                    elif cat in ["fire", "hazard"]:
                        # Extreme thermal flare 550°C+
                        cv2.circle(thermal_raw, ((x1 + x2) // 2, (y1 + y2) // 2), max(16, bw // 2), 255.0, -1)
                        thermal_raw[y1:y2, x1:x2] = 255.0

        # Subtle Gaussian thermal blur for atmospheric diffusion
        thermal_blurred = cv2.GaussianBlur(thermal_raw, (5, 5), 1.2)
        thermal_uint8 = np.clip(thermal_blurred, 0, 255).astype(np.uint8)

        # Apply Color Palette
        if self.current_palette == "white_hot":
            thermal_color = cv2.cvtColor(thermal_uint8, cv2.COLOR_GRAY2BGR)
        elif self.current_palette == "black_hot":
            inv = 255 - thermal_uint8
            thermal_color = cv2.cvtColor(inv, cv2.COLOR_GRAY2BGR)
        else:
            colormap_id = self.PALETTES.get(self.current_palette, cv2.COLORMAP_INFERNO)
            thermal_color = cv2.applyColorMap(thermal_uint8, colormap_id)

        # Render Tactical Overlays & Target Brackets
        cx, cy = w // 2, h // 2
        # Center spot radiometer crosshair
        cv2.line(thermal_color, (cx - 14, cy), (cx + 14, cy), (0, 255, 255), 1, cv2.LINE_AA)
        cv2.line(thermal_color, (cx, cy - 14), (cx, cy + 14), (0, 255, 255), 1, cv2.LINE_AA)
        center_val = thermal_uint8[cy, cx]
        synth_temp_c = round(12.0 + (center_val / 255.0) * 85.0, 1)

        # OSD Status Header
        cv2.putText(thermal_color, f"LWIR THERMAL | PALETTE: {self.current_palette.upper()}", (16, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(thermal_color, f"T_CTR: {synth_temp_c} C", (cx + 16, cy - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)

        # Render Tactical Thermal Target Brackets for Detections
        if detections:
            for det in detections:
                box = det.get("bbox", [])
                if len(box) == 4:
                    x1, y1, x2, y2 = [int(v) for v in box]
                    bw = x2 - x1
                    bh = y2 - y1
                    k = max(4, min(12, min(bw, bh) // 4))
                    tid = det.get("track_id", "TRK")
                    cat = det.get("category", "").upper()
                    temp = det.get("temp_c", 37.0)

                    # Thermal bracket color: High contrast bright yellow/white
                    tb_color = (0, 255, 255) if cat != "FIRE" else (0, 0, 255)

                    cv2.line(thermal_color, (x1, y1), (x1 + k, y1), tb_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x1, y1), (x1, y1 + k), tb_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x2, y1), (x2 - k, y1), tb_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x2, y1), (x2, y1 + k), tb_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x1, y2), (x1 + k, y2), tb_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x1, y2), (x1, y2 - k), tb_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x2, y2), (x2 - k, y2), tb_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x2, y2), (x2, y2 - k), tb_color, 2, cv2.LINE_AA)

                    tag_text = f"[{tid} | {temp:.1f} C | {cat}]"
                    cv2.putText(thermal_color, tag_text, (x1, max(12, y1 - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.38, tb_color, 1, cv2.LINE_AA)

        # Temperature Scale Color Bar on Right
        bar_x = w - 24
        bar_h = 160
        bar_y = (h - bar_h) // 2
        for i in range(bar_h):
            val = int(255 * (1.0 - (i / bar_h)))
            if self.current_palette in ["white_hot", "black_hot"]:
                c_val = val if self.current_palette == "white_hot" else (255 - val)
                col = (c_val, c_val, c_val)
            else:
                colormap_id = self.PALETTES.get(self.current_palette, cv2.COLORMAP_INFERNO)
                col = cv2.applyColorMap(np.array([[val]], dtype=np.uint8), colormap_id)[0, 0].tolist()
            cv2.line(thermal_color, (bar_x, bar_y + i), (bar_x + 12, bar_y + i), col, 1)

        cv2.rectangle(thermal_color, (bar_x, bar_y), (bar_x + 12, bar_y + bar_h), (255, 255, 255), 1)
        cv2.putText(thermal_color, "95 C", (bar_x - 30, bar_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (255, 255, 255), 1)
        cv2.putText(thermal_color, "14 C", (bar_x - 30, bar_y + bar_h), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (255, 255, 255), 1)

        return thermal_color
