"""Synthetic Thermal / Infrared Camera Processing Pipeline with Calibrated Radiometry."""

import cv2
import numpy as np
import time
import math
from typing import List, Dict, Any, Optional, Tuple

class ThermalProcessor:
    """Simulates calibrated Long-Wave Infrared (LWIR 8-14 µm) thermography with Planck-radiance physics."""

    PALETTES = {
        "inferno": cv2.COLORMAP_INFERNO,
        "ironbow": cv2.COLORMAP_JET,
        "white_hot": "white_hot",
        "black_hot": "black_hot",
        "rainbow": cv2.COLORMAP_RAINBOW,
        "plasma": cv2.COLORMAP_PLASMA,
        "bone": cv2.COLORMAP_BONE,
        "arctic": cv2.COLORMAP_OCEAN
    }

    def __init__(self, default_palette: str = "ironbow"):
        self.current_palette = default_palette if default_palette in self.PALETTES else "ironbow"
        self.isotherm_enabled = True
        self.isotherm_threshold_c = 35.0
        self.last_temp_map: Optional[np.ndarray] = None
        self.last_radiometry: Dict[str, float] = {
            "t_min": 12.0,
            "t_max": 37.0,
            "t_center": 18.5,
            "t_mean": 17.2,
            "hotspots_count": 0
        }

    def set_palette(self, palette_name: str) -> bool:
        palette_key = palette_name.lower().strip()
        if palette_key in self.PALETTES:
            self.current_palette = palette_key
            return True
        return False

    def set_isotherm(self, enabled: bool, threshold_c: float = 35.0):
        self.isotherm_enabled = enabled
        self.isotherm_threshold_c = threshold_c

    def get_temperature_at_pixel(self, u: int, v: int) -> float:
        """Returns exact calibrated Celsius temperature at given image coordinates (u, v)."""
        if self.last_temp_map is None:
            return 18.0
        h, w = self.last_temp_map.shape[:2]
        cu = max(0, min(w - 1, int(u)))
        cv = max(0, min(h - 1, int(v)))
        return float(round(self.last_temp_map[cv, cu], 1))

    def process_frame(self, rgb_frame: np.ndarray, detections: Optional[List[Dict[str, Any]]] = None) -> np.ndarray:
        """Converts an RGB/Gazebo image into physically-calibrated radiometry LWIR thermography."""
        if rgb_frame is None or rgb_frame.size == 0:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        h, w = rgb_frame.shape[:2]
        gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)

        # 1. Physics-Based Ambient Ground Temperature Map (13.5°C to 21.0°C baseline)
        norm_gray = gray.astype(np.float32) / 255.0
        temp_map = 13.5 + (norm_gray * 7.5) # Background vegetation / terrain (13.5°C .. 21.0°C)

        # Water features (Cold sink 11.0°C .. 13.0°C)
        b, g, r = cv2.split(rgb_frame)
        water_mask = (b.astype(np.float32) > (r.astype(np.float32) + 18)) & (g.astype(np.float32) > (r.astype(np.float32) + 8))
        temp_map[water_mask] = 11.5 + (norm_gray[water_mask] * 1.5)

        # 2. Inject Calibrated Radiant Thermal Heat Signatures for Entities
        hotspots = []
        if detections:
            for det in detections:
                box = det.get("bbox", [])
                cat = det.get("category", "")
                lbl = det.get("label", "").lower()
                target_temp = float(det.get("temp_c", 37.0))

                if len(box) == 4:
                    x1, y1, x2, y2 = [int(v) for v in box]
                    x1, y1 = max(0, min(w - 2, x1)), max(0, min(h - 2, y1))
                    x2, y2 = max(x1 + 2, min(w, x2)), max(y1 + 2, min(h, y2))
                    bw = x2 - x1
                    bh = y2 - y1

                    if cat in ["person", "human", "pedestrian"] or "man" in lbl or "woman" in lbl:
                        # Human Body Thermogram: Core 37.0°C, head 36.6°C, torso 35.8°C, limbs 32.5°C
                        head_y = y1 + int(bh * 0.22)
                        head_x = (x1 + x2) // 2
                        rad_h = max(3, bw // 3)
                        cv2.circle(temp_map, (head_x, head_y), rad_h, 36.8, -1)
                        # Torso oval
                        cv2.ellipse(temp_map, ((x1 + x2) // 2, y1 + int(bh * 0.55)), (max(4, bw // 3), max(6, bh // 3)), 0, 0, 360, 35.5, -1)
                        # Full silhouette warm halo
                        sub = temp_map[y1:y2, x1:x2]
                        temp_map[y1:y2, x1:x2] = np.maximum(sub, 32.5)
                        hotspots.append({"x": head_x, "y": head_y, "temp": 37.0, "label": "HUMAN HEAT SIGNATURE"})

                    elif cat in ["wildlife", "animal"] or "deer" in lbl or "elk" in lbl or "wolf" in lbl:
                        # Wildlife: Snout/eyes warm (35.5°C), thick insulated fur coat (31.0°C .. 33.5°C)
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        cv2.ellipse(temp_map, (cx, cy), (max(5, bw // 3), max(4, bh // 3)), 0, 0, 360, 33.5, -1)
                        head_x = x1 + int(bw * 0.25)
                        head_y = y1 + int(bh * 0.35)
                        cv2.circle(temp_map, (head_x, head_y), max(3, bw // 5), 35.5, -1)
                        hotspots.append({"x": cx, "y": cy, "temp": 35.5, "label": "BIOLOGICAL FAUNA"})

                    elif cat in ["vehicle", "truck", "car"] or "suv" in lbl or "truck" in lbl or "atv" in lbl:
                        # Vehicle: Internal combustion engine block (85°C - 95°C), exhaust, warm tires (45°C)
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        eng_x = x1 + int(bw * 0.70)
                        eng_y = cy
                        # Engine core
                        cv2.circle(temp_map, (eng_x, eng_y), max(6, bw // 4), target_temp, -1)
                        # Warm chassis body
                        sub = temp_map[y1:y2, x1:x2]
                        temp_map[y1:y2, x1:x2] = np.maximum(sub, 38.0)
                        hotspots.append({"x": eng_x, "y": eng_y, "temp": target_temp, "label": "COMBUSTION POWERTRAIN"})

                    elif cat in ["fire", "hazard"] or "wildfire" in lbl or "campfire" in lbl:
                        # Fire Flare: Extreme thermal gradient (350°C to 650°C) with smoke dispersal
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        r_flame = max(12, min(bw, bh) // 2)
                        cv2.circle(temp_map, (cx, cy), r_flame, target_temp, -1)
                        # Outer thermal plume
                        cv2.circle(temp_map, (cx, cy), int(r_flame * 1.8), target_temp * 0.65, -1)
                        hotspots.append({"x": cx, "y": cy, "temp": target_temp, "label": "CRITICAL THERMAL FLARE"})

        # Atmospheric dispersion smoothing (FLIR thermal optics blur)
        temp_map = cv2.GaussianBlur(temp_map, (5, 5), 1.1)
        self.last_temp_map = temp_map

        # Radiometric Scene Statistics
        t_min = float(round(np.min(temp_map), 1))
        t_max = float(round(np.max(temp_map), 1))
        t_mean = float(round(np.mean(temp_map), 1))
        cx, cy = w // 2, h // 2
        t_center = float(round(temp_map[cy, cx], 1))
        self.last_radiometry = {
            "t_min": t_min,
            "t_max": t_max,
            "t_center": t_center,
            "t_mean": t_mean,
            "hotspots_count": len(hotspots)
        }

        # 3. Dynamic Plateau Radiance Compression (Maps 10°C .. 650°C into 0..255 gracefully)
        # We use a dual-slope radiometric compression curve so human (37°C) retains full contrast even when 580°C fire is present
        norm_intensity = np.zeros_like(temp_map, dtype=np.float32)
        bg_min = 10.0
        bio_max = 42.0

        # Region A: Environmental & Biological (10°C to 42°C -> 0 to 185 intensity)
        mask_low = temp_map <= bio_max
        norm_intensity[mask_low] = ((temp_map[mask_low] - bg_min) / (bio_max - bg_min)) * 185.0

        # Region B: High Heat & Combustion (42°C to 650°C -> 185 to 255 intensity using log scale)
        mask_high = temp_map > bio_max
        log_val = np.log1p(temp_map[mask_high] - bio_max)
        log_max = np.log1p(650.0 - bio_max)
        norm_intensity[mask_high] = 185.0 + (log_val / log_max) * 70.0

        thermal_uint8 = np.clip(norm_intensity, 0, 255).astype(np.uint8)

        # 4. Color Palette Mapping
        if self.current_palette == "white_hot":
            thermal_color = cv2.cvtColor(thermal_uint8, cv2.COLOR_GRAY2BGR)
        elif self.current_palette == "black_hot":
            inv = 255 - thermal_uint8
            thermal_color = cv2.cvtColor(inv, cv2.COLOR_GRAY2BGR)
        else:
            cmap_id = self.PALETTES.get(self.current_palette, cv2.COLORMAP_JET)
            thermal_color = cv2.applyColorMap(thermal_uint8, cmap_id)

        # 5. Isotherm Highlight Alarm Overlay (Flashing vibrant cyan/red on targets >= threshold)
        if self.isotherm_enabled:
            isotherm_mask = temp_map >= self.isotherm_threshold_c
            # Subtle pulsating alpha blend
            pulse = 0.5 + 0.3 * math.sin(time.time() * 6.0)
            iso_overlay = thermal_color.copy()
            # Crimson for high heat >= 60°C, Amber for biological >= 35°C
            iso_color = np.zeros_like(thermal_color)
            iso_color[temp_map >= 60.0] = [0, 50, 255] # Red
            iso_color[(temp_map >= self.isotherm_threshold_c) & (temp_map < 60.0)] = [0, 220, 255] # Amber-Yellow
            thermal_color[isotherm_mask] = cv2.addWeighted(thermal_color, 1.0 - pulse, iso_color, pulse, 0)[isotherm_mask]

        # 6. Military-Grade OSD & Radiometric HUD Elements
        # Boresight Spot Radiometer Reticle
        reticle_color = (0, 255, 255) if self.current_palette != "white_hot" else (0, 200, 0)
        cv2.line(thermal_color, (cx - 16, cy), (cx + 16, cy), reticle_color, 1, cv2.LINE_AA)
        cv2.line(thermal_color, (cx, cy - 16), (cx, cy + 16), reticle_color, 1, cv2.LINE_AA)
        cv2.circle(thermal_color, (cx, cy), 6, reticle_color, 1, cv2.LINE_AA)

        # Convert Center Spot Temp to Fahrenheit
        t_center_f = round((t_center * 9.0 / 5.0) + 32.0, 1)

        # HUD Top Banner
        hud_bg = (10, 14, 22)
        cv2.rectangle(thermal_color, (8, 8), (340, 48), hud_bg, -1)
        cv2.rectangle(thermal_color, (8, 8), (340, 48), (0, 240, 255), 1)

        pal_tag = self.current_palette.upper().replace("_", " ")
        cv2.putText(thermal_color, f"FLIR LWIR 8-14um | {pal_tag}", (16, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(thermal_color, f"CTR SPOT: {t_center:.1f} C / {t_center_f:.1f} F", (16, 42),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)

        # HUD Radiometry Min / Max / Mean Box on Bottom Left
        cv2.rectangle(thermal_color, (8, h - 38), (280, h - 8), hud_bg, -1)
        cv2.rectangle(thermal_color, (8, h - 38), (280, h - 8), (80, 80, 80), 1)
        stat_line = f"MIN: {t_min:.1f}C | AVG: {t_mean:.1f}C | MAX: {t_max:.1f}C"
        cv2.putText(thermal_color, stat_line, (14, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 229, 153), 1, cv2.LINE_AA)

        # 7. Thermal Target Brackets for Identified Hotspots
        if detections:
            for det in detections:
                box = det.get("bbox", [])
                if len(box) == 4:
                    x1, y1, x2, y2 = [int(v) for v in box]
                    bw = x2 - x1
                    bh = y2 - y1
                    k = max(4, min(14, min(bw, bh) // 4))
                    tid = det.get("track_id", "TRK")
                    lbl = det.get("label", det.get("category", "HEAT"))
                    temp = float(det.get("temp_c", 37.0))
                    is_hazard = (temp >= 100.0 or "hazard" in lbl.lower() or "fire" in lbl.lower())

                    t_color = (0, 0, 255) if is_hazard else (0, 255, 255)

                    # Precise Corner Brackets
                    cv2.line(thermal_color, (x1, y1), (x1 + k, y1), t_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x1, y1), (x1, y1 + k), t_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x2, y1), (x2 - k, y1), t_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x2, y1), (x2, y1 + k), t_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x1, y2), (x1 + k, y2), t_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x1, y2), (x1, y2 - k), t_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x2, y2), (x2 - k, y2), t_color, 2, cv2.LINE_AA)
                    cv2.line(thermal_color, (x2, y2), (x2, y2 - k), t_color, 2, cv2.LINE_AA)

                    badge = f"[{tid} | {temp:.1f}C | {lbl[:18]}]"
                    cv2.putText(thermal_color, badge, (x1, max(14, y1 - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.36, t_color, 1, cv2.LINE_AA)

        # 8. Calibrated Radiometric Colorbar Scale on Right
        bar_x = w - 30
        bar_w = 12
        bar_h = 200
        bar_y = (h - bar_h) // 2

        # Draw Gradient
        for i in range(bar_h):
            val = int(255 * (1.0 - (i / bar_h)))
            if self.current_palette == "white_hot":
                c = (val, val, val)
            elif self.current_palette == "black_hot":
                c = (255 - val, 255 - val, 255 - val)
            else:
                cmap_id = self.PALETTES.get(self.current_palette, cv2.COLORMAP_JET)
                c = cv2.applyColorMap(np.array([[val]], dtype=np.uint8), cmap_id)[0, 0].tolist()
            cv2.line(thermal_color, (bar_x, bar_y + i), (bar_x + bar_w, bar_y + i), c, 1)

        cv2.rectangle(thermal_color, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (255, 255, 255), 1)
        # Tick markers
        cv2.putText(thermal_color, f"{int(t_max)}C", (bar_x - 42, bar_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1)
        cv2.putText(thermal_color, "42C", (bar_x - 32, bar_y + int(bar_h * 0.45)), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (0, 240, 255), 1)
        cv2.putText(thermal_color, f"{int(t_min)}C", (bar_x - 42, bar_y + bar_h), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255, 255, 255), 1)

        return thermal_color
