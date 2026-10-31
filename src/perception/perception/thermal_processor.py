"""Synthetic Thermal / Infrared Camera Processing Pipeline."""

import cv2
import numpy as np
import time

class ThermalProcessor:
    """Simulates realistic Long-Wave Infrared (LWIR) thermography with selectable color palettes."""

    PALETTES = {
        "inferno": cv2.COLORMAP_INFERNO,
        "ironbow": cv2.COLORMAP_JET, # Standard ironbow approximation
        "white_hot": None, # Monochrome white-hot
        "black_hot": None, # Monochrome inverted
        "jet": cv2.COLORMAP_JET,
        "plasma": cv2.COLORMAP_PLASMA,
        "bone": cv2.COLORMAP_BONE
    }

    def __init__(self, default_palette: str = "inferno"):
        self.current_palette = default_palette if default_palette in self.PALETTES else "inferno"

    def set_palette(self, palette_name: str):
        if palette_name in self.PALETTES:
            self.current_palette = palette_name

    def process_frame(self, rgb_frame: np.ndarray, detections=None) -> np.ndarray:
        """Converts an RGB image into calibrated synthetic thermal imagery."""
        if rgb_frame is None or rgb_frame.size == 0:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        h, w = rgb_frame.shape[:2]
        gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)

        # Baseline ambient temperature map (approx 18°C to 26°C mapped to 40..85 intensity)
        thermal_raw = cv2.normalize(gray, None, 45, 95, cv2.NORM_MINMAX).astype(np.float32)

        # Segment water features (high blue/cyan ratio -> cold signature 14°C)
        b, g, r = cv2.split(rgb_frame)
        water_mask = (b.astype(np.float32) > (r.astype(np.float32) + 20)) & (g.astype(np.float32) > (r.astype(np.float32) + 10))
        thermal_raw[water_mask] = np.clip(thermal_raw[water_mask] * 0.55, 20, 45)

        # Inject heat signatures for detected entities
        if detections:
            for det in detections:
                box = det.get("bbox", [])
                cat = det.get("category", "")
                if len(box) == 4:
                    x1, y1, x2, y2 = [int(v) for v in box]
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(w, x2), min(h, y2)
                    if x2 > x1 and y2 > y1:
                        sub = thermal_raw[y1:y2, x1:x2]
                        if cat in ["person", "human", "pedestrian"]:
                            # 35°C - 37°C core metabolic signature
                            cv2.circle(thermal_raw, ((x1+x2)//2, (y1+y2)//2), max(6, (x2-x1)//3), 195, -1)
                            thermal_raw[y1:y2, x1:x2] = np.clip(sub + 90.0, 110, 215)
                        elif cat in ["wildlife", "animal", "deer"]:
                            # 34°C - 36°C coat insulated heat
                            thermal_raw[y1:y2, x1:x2] = np.clip(sub + 75.0, 95, 190)
                        elif cat in ["vehicle", "truck", "car"]:
                            # 75°C - 95°C engine block and tires
                            thermal_raw[y1:y2, x1:x2] = np.clip(sub + 120.0, 140, 235)
                        elif cat in ["fire", "hazard"]:
                            # Extreme thermal flare 500°C+
                            cv2.circle(thermal_raw, ((x1+x2)//2, (y1+y2)//2), max(15, (x2-x1)//2), 255, -1)
                            thermal_raw[y1:y2, x1:x2] = 255.0

        # Subtle Gaussian thermal diffusion
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

        # Render Thermal OSD Overlays (Legend, crosshair, min/max synthetic temp)
        # Center reticle
        cx, cy = w // 2, h // 2
        cv2.line(thermal_color, (cx - 15, cy), (cx + 15, cy), (0, 255, 255), 1)
        cv2.line(thermal_color, (cx, cy - 15), (cx, cy + 15), (0, 255, 255), 1)

        # Calculate synthetic temperature at reticle
        center_val = thermal_uint8[cy, cx]
        synth_temp_c = round(12.0 + (center_val / 255.0) * 85.0, 1)

        # OSD Status Badges
        cv2.putText(thermal_color, f"LWIR THERMAL | {self.current_palette.upper()}", (16, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(thermal_color, f"T_CTR: {synth_temp_c} C", (cx + 18, cy - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        # Temperature Scale Color Bar on Right
        bar_x = w - 30
        bar_h = 160
        bar_y1 = 50
        gradient = np.linspace(255, 0, bar_h, dtype=np.uint8).reshape((bar_h, 1))
        if self.current_palette in ["white_hot", "black_hot"]:
            bar_rgb = cv2.cvtColor(gradient, cv2.COLOR_GRAY2BGR)
        else:
            bar_rgb = cv2.applyColorMap(gradient, self.PALETTES.get(self.current_palette, cv2.COLORMAP_INFERNO))
        bar_rgb = cv2.resize(bar_rgb, (14, bar_h), interpolation=cv2.INTER_NEAREST)
        thermal_color[bar_y1:bar_y1+bar_h, bar_x:bar_x+14] = bar_rgb
        cv2.rectangle(thermal_color, (bar_x, bar_y1), (bar_x+14, bar_y1+bar_h), (255, 255, 255), 1)
        cv2.putText(thermal_color, "97C", (bar_x - 32, bar_y1 + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
        cv2.putText(thermal_color, "12C", (bar_x - 32, bar_y1 + bar_h), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)

        # Notice tag: Synthetic Thermal
        cv2.putText(thermal_color, "SYNTHETIC THERMAL MODEL - RELATIVE RADIANCE", (16, h - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA)

        return thermal_color
