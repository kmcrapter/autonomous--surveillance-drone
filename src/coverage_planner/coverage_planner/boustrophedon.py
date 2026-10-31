"""Boustrophedon (lawnmower) path generation with camera footprint & overlap calculations."""

import math
from typing import List, Dict, Any, Tuple

class BoustrophedonPlanner:
    """Generates optimal continuous lawnmower survey trajectories."""

    def __init__(self,
                 camera_hfov_deg: float = 80.0,
                 camera_vfov_deg: float = 60.0,
                 default_side_overlap: float = 0.50,
                 default_front_overlap: float = 0.60):
        self.camera_hfov = math.radians(camera_hfov_deg)
        self.camera_vfov = math.radians(camera_vfov_deg)
        self.default_side_overlap = default_side_overlap
        self.default_front_overlap = default_front_overlap

    def calculate_camera_footprint(self, altitude: float) -> Tuple[float, float]:
        """Calculates ground footprint width (cross-track) and length (along-track) in meters."""
        ground_w = 2.0 * altitude * math.tan(self.camera_hfov / 2.0)
        ground_h = 2.0 * altitude * math.tan(self.camera_vfov / 2.0)
        return ground_w, ground_h

    def generate_zone_waypoints(self,
                                zone_info: Dict[str, Any],
                                custom_altitude: float = None,
                                custom_speed: float = None) -> List[Dict[str, Any]]:
        """Generates continuous boustrophedon 3D waypoints for a given zone."""
        bounds = zone_info["bounds"]
        altitude = custom_altitude or zone_info.get("recommended_altitude", 15.0)
        speed = custom_speed or zone_info.get("speed", 4.0)
        side_overlap = zone_info.get("side_overlap", self.default_side_overlap)

        ground_w, ground_h = self.calculate_camera_footprint(altitude)
        # Track spacing in Y direction
        track_spacing = max(6.0, ground_h * (1.0 - side_overlap))
        # Waypoint spacing along X track
        step_x = max(8.0, ground_w * (1.0 - self.default_front_overlap))

        min_x = bounds["min_x"] + (ground_w * 0.2)
        max_x = bounds["max_x"] - (ground_w * 0.2)
        min_y = bounds["min_y"] + (ground_h * 0.2)
        # Constrain strictly to Country A sovereign territory (standoff before border line at Y=32.0)
        max_y = min(28.0, bounds["max_y"] - (ground_h * 0.2))

        if max_x <= min_x:
            min_x, max_x = bounds["min_x"], bounds["max_x"]
        if max_y <= min_y:
            min_y, max_y = bounds["min_y"], bounds["max_y"]

        waypoints = []
        curr_y = min_y
        left_to_right = True
        idx = 0

        while curr_y <= max_y + 0.1:
            if left_to_right:
                x_pts = []
                x = min_x
                while x <= max_x:
                    x_pts.append(x)
                    x += step_x
                if not x_pts or x_pts[-1] < max_x:
                    x_pts.append(max_x)
            else:
                x_pts = []
                x = max_x
                while x >= min_x:
                    x_pts.append(x)
                    x -= step_x
                if not x_pts or x_pts[-1] > min_x:
                    x_pts.append(min_x)

            for x_coord in x_pts:
                waypoints.append({
                    "id": f"{zone_info['id']}_wp_{idx}",
                    "zone_id": zone_info["id"],
                    "x": round(x_coord, 2),
                    "y": round(curr_y, 2),
                    "z": round(altitude, 2),
                    "speed": speed,
                    "footprint_w": round(ground_w, 2),
                    "footprint_h": round(ground_h, 2)
                })
                idx += 1

            left_to_right = not left_to_right
            curr_y += track_spacing

        return waypoints

    def generate_full_mission_plan(self,
                                   ordered_zones: List[Dict[str, Any]],
                                   transit_altitude: float = 16.0) -> List[Dict[str, Any]]:
        """Generates complete multi-zone mission waypoints including transit points."""
        plan = []
        for zone in ordered_zones:
            zone_wps = self.generate_zone_waypoints(zone)
            if zone_wps:
                # Add safe transit lead-in waypoint at transit altitude
                first_wp = zone_wps[0]
                plan.append({
                    "id": f"transit_to_{zone['id']}",
                    "zone_id": "transit",
                    "x": first_wp["x"],
                    "y": first_wp["y"],
                    "z": transit_altitude,
                    "speed": 5.0,
                    "is_transit": True
                })
                plan.extend(zone_wps)
        return plan

    def generate_perimeter_plan(self,
                                altitude: float = 15.0,
                                speed: float = 4.5,
                                standoff_y: float = 28.0) -> List[Dict[str, Any]]:
        """Generates sovereign border perimeter defense patrol sweep along Country A boundary."""
        plan = [
            {"id": "perimeter_climb", "zone_id": "transit", "x": 0.0, "y": 0.0, "z": altitude, "speed": 4.0},
            {"id": "perimeter_west_entry", "zone_id": "zone_bravo", "x": -80.0, "y": 20.0, "z": altitude, "speed": speed},
            {"id": "perimeter_border_w1", "zone_id": "zone_bravo", "x": -80.0, "y": standoff_y, "z": altitude, "speed": speed},
            {"id": "perimeter_border_w2", "zone_id": "zone_alpha", "x": -40.0, "y": standoff_y, "z": altitude, "speed": speed},
            {"id": "perimeter_border_gate", "zone_id": "zone_alpha", "x": 0.0, "y": standoff_y, "z": altitude, "speed": speed},
            {"id": "perimeter_border_e1", "zone_id": "zone_charlie", "x": 40.0, "y": standoff_y, "z": altitude, "speed": speed},
            {"id": "perimeter_border_e2", "zone_id": "zone_delta", "x": 80.0, "y": standoff_y, "z": altitude, "speed": speed},
            {"id": "perimeter_outpost_ridge", "zone_id": "zone_delta", "x": 75.0, "y": 0.0, "z": altitude, "speed": speed},
            {"id": "perimeter_south_east", "zone_id": "zone_delta", "x": 50.0, "y": -30.0, "z": altitude, "speed": speed},
            {"id": "perimeter_south_base", "zone_id": "zone_alpha", "x": 0.0, "y": -45.0, "z": altitude, "speed": speed},
            {"id": "perimeter_south_west", "zone_id": "zone_bravo", "x": -50.0, "y": -30.0, "z": altitude, "speed": speed},
            {"id": "perimeter_border_return", "zone_id": "zone_bravo", "x": -80.0, "y": standoff_y, "z": altitude, "speed": speed},
            {"id": "perimeter_border_sweep_e", "zone_id": "zone_delta", "x": 80.0, "y": standoff_y, "z": altitude, "speed": speed}
        ]
        return plan

    def generate_spiral_plan(self,
                             center_x: float = 0.0,
                             center_y: float = 0.0,
                             max_radius: float = 65.0,
                             altitude: float = 16.0,
                             speed: float = 4.2) -> List[Dict[str, Any]]:
        """Generates expanding rectangular spiral search pattern constrained within Country A."""
        wps = [
            {"id": "spiral_climb", "zone_id": "transit", "x": center_x, "y": center_y, "z": altitude, "speed": 4.0}
        ]
        step = 16.0 # Camera footprint width
        x, y = center_x, center_y
        dist = step
        idx = 1
        sovereign_max_y = 28.0

        while dist <= max_radius:
            # 1. Right (+X)
            x += dist
            wps.append({"id": f"spiral_wp_{idx}", "zone_id": "spiral", "x": round(x, 2), "y": round(min(sovereign_max_y, y), 2), "z": altitude, "speed": speed})
            idx += 1
            # 2. Up (+Y, capped by sovereign border)
            y += dist
            wps.append({"id": f"spiral_wp_{idx}", "zone_id": "spiral", "x": round(x, 2), "y": round(min(sovereign_max_y, y), 2), "z": altitude, "speed": speed})
            idx += 1
            dist += step

            # 3. Left (-X)
            x -= dist
            wps.append({"id": f"spiral_wp_{idx}", "zone_id": "spiral", "x": round(x, 2), "y": round(min(sovereign_max_y, y), 2), "z": altitude, "speed": speed})
            idx += 1
            # 4. Down (-Y)
            y -= dist
            wps.append({"id": f"spiral_wp_{idx}", "zone_id": "spiral", "x": round(x, 2), "y": round(min(sovereign_max_y, y), 2), "z": altitude, "speed": speed})
            idx += 1
            dist += step

        return wps
