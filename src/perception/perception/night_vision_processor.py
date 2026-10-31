"""Night-Vision Image Intensifier (NVG I²) Processing Pipeline."""

import cv2
import numpy as np

class NightVisionProcessor:
    """Simulates realistic Generation-3 Image Intensifier night-vision optics."""

    def __init__(self, mode: str = "green_p43", gain: float = 1.6):
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

    def process_frame(self, rgb_frame: np.ndarray) -> np.ndarray:
        """Applies night-vision intensification, phosphor response, noise, and vignette."""
        if rgb_frame is None or rgb_frame.size == 0:
            return np.zeros((480, 640, 3), dtype=np.uint8)

        h, w = rgb_frame.shape[:2]
        gray = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY).astype(np.float32)

        # 1. Amplification Gain and Gamma correction
        intensified = gray * self.gain
        # Nonlinear toe compression (analog tube saturation curve)
        intensified = 255.0 * (1.0 - np.exp(-intensified / 128.0))

        # 2. Add realistic Microchannel Plate (MCP) scintillation grain
        noise = np.random.normal(0, 14.0, (h, w)).astype(np.float32)
        intensified = np.clip(intensified + noise, 0, 255)

        # 3. Optical vignette radial falloff
        vignette = self._get_vignette((h, w))
        intensified = intensified * vignette

        intensified_uint8 = np.clip(intensified, 0, 255).astype(np.uint8)

        # 4. Phosphor Spectral Mapping
        if self.mode == "white_p45":
            # P45 White Phosphor (cool blue-tinted monochrome)
            nvg_frame = np.zeros((h, w, 3), dtype=np.uint8)
            nvg_frame[:, :, 0] = np.clip(intensified_uint8 * 1.05, 0, 255).astype(np.uint8)
            nvg_frame[:, :, 1] = intensified_uint8
            nvg_frame[:, :, 2] = np.clip(intensified_uint8 * 0.90, 0, 255).astype(np.uint8)
        else:
            # P43 Green Phosphor (classic 545nm green peak)
            nvg_frame = np.zeros((h, w, 3), dtype=np.uint8)
            nvg_frame[:, :, 0] = (intensified_uint8 * 0.15).astype(np.uint8) # Slight blue
            nvg_frame[:, :, 1] = intensified_uint8 # Intense green
            nvg_frame[:, :, 2] = (intensified_uint8 * 0.10).astype(np.uint8) # Low red

        # 5. Night Vision HUD Overlay
        cv2.putText(nvg_frame, f"NVG I-SQUARE | {self.mode.upper()} | GAIN: {self.gain:.1f}x",
                    (16, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 255, 120), 2, cv2.LINE_AA)
        cv2.circle(nvg_frame, (w // 2, h // 2), 12, (80, 240, 80), 1)

        cv2.putText(nvg_frame, "LOW-LIGHT PHOTON AMPLIFICATION MODEL",
                    (16, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 220, 100), 1, cv2.LINE_AA)

        return nvg_frame
