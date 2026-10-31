"""3D Trajectory Tracking and Velocity Guidance Controller."""

import math
from typing import Tuple, Dict, Any

class FlightController:
    """Computes smooth 3D guidance velocity vectors toward active waypoints."""

    def __init__(self,
                 kp_xy: float = 0.85,
                 kp_z: float = 1.0,
                 kp_yaw: float = 1.2,
                 max_xy_speed: float = 4.5,
                 max_z_speed: float = 2.0,
                 max_yaw_rate: float = 1.0,
                 waypoint_tolerance_m: float = 1.6):
        self.kp_xy = kp_xy
        self.kp_z = kp_z
        self.kp_yaw = kp_yaw
        self.max_xy_speed = max_xy_speed
        self.max_z_speed = max_z_speed
        self.max_yaw_rate = max_yaw_rate
        self.waypoint_tolerance_m = waypoint_tolerance_m

    def compute_cmd_vel(self,
                        curr_pos: Tuple[float, float, float],
                        curr_yaw: float,
                        target_pos: Tuple[float, float, float],
                        target_speed: float = None) -> Tuple[float, float, float, float, bool]:
        """Returns (cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate, is_waypoint_reached)."""
        cx, cy, cz = curr_pos
        tx, ty, tz = target_pos

        dx = tx - cx
        dy = ty - cy
        dz = tz - cz

        dist_xy = math.sqrt((dx * dx) + (dy * dy))
        dist_3d = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))

        is_reached = (dist_3d <= self.waypoint_tolerance_m)

        max_speed = target_speed if target_speed else self.max_xy_speed

        # Desired horizontal velocity in world frame
        if dist_xy > 0.05:
            des_speed = min(max_speed, self.kp_xy * dist_xy)
            vx_world = des_speed * (dx / dist_xy)
            vy_world = des_speed * (dy / dist_xy)
        else:
            vx_world = 0.0
            vy_world = 0.0

        # Desired vertical velocity
        vz_world = max(-self.max_z_speed, min(self.max_z_speed, self.kp_z * dz))

        # Desired yaw heading (face movement direction if moving > 0.5 m/s)
        if dist_xy > 1.0:
            target_yaw = math.atan2(dy, dx)
        else:
            target_yaw = curr_yaw

        # Yaw error wrapped to [-pi, pi]
        yaw_err = target_yaw - curr_yaw
        while yaw_err > math.pi:
            yaw_err -= 2.0 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2.0 * math.pi

        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.kp_yaw * yaw_err))

        return vx_world, vy_world, vz_world, yaw_rate, is_reached
