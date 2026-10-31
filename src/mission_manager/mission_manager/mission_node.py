"""ROS 2 Autonomous Mission Manager Node with Full State Machine and Coverage Controller."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import BatteryState

import json
import time
import math
from typing import List, Dict, Any, Tuple

from mission_manager.state_machine import MissionState, MissionStateMachine
from mission_manager.flight_controller import FlightController
from coverage_planner.boustrophedon import BoustrophedonPlanner
from coverage_planner.zone_manager import ZoneManager

def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Computes Euler yaw from quaternion components."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        self.fsm = MissionStateMachine(initial_state=MissionState.IDLE)
        self.controller = FlightController()
        self.planner = BoustrophedonPlanner()

        # Mission state variables
        self.mission_id = f"MSN-{int(time.time())}"
        self.mission_start_time = None
        self.mission_end_time = None
        self.active_zones = ["zone_alpha", "zone_bravo", "zone_charlie", "zone_delta"]
        self.mission_waypoints: List[Dict[str, Any]] = []
        self.curr_wp_index = 0
        self.saved_resume_wp_index = 0
        self.total_distance_m = 0.0
        self.prev_pos = None

        # Current drone state from odometry
        self.drone_pos = (0.0, 0.0, 0.42)
        self.drone_yaw = 0.0
        self.drone_vel = (0.0, 0.0, 0.0)
        self.battery_pct = 100.0
        self.battery_low_rth = False
        self.sensor_health = {"rgb_forward": True, "rgb_downward": True, "imu": True}

        # Subscriptions
        self.odom_sub = self.create_subscription(Odometry, '/drone/odometry', self.odom_callback, 10)
        self.batt_sub = self.create_subscription(BatteryState, '/drone/battery_state', self.battery_callback, 10)
        self.rth_alert_sub = self.create_subscription(Bool, '/drone/battery_low_rth_alert', self.rth_alert_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/drone/mission_command', self.command_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/drone/mission_state', 10)
        self.telemetry_pub = self.create_publisher(String, '/drone/mission_telemetry', 10)
        self.coverage_pub = self.create_publisher(String, '/drone/coverage_status', 10)

        # Generate initial default plan
        self.generate_waypoints_plan()

        # 10 Hz Control Loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("MissionManagerNode active (State: IDLE, Plan generated).")

    def generate_waypoints_plan(self):
        ordered = ZoneManager.get_ordered_zones(self.active_zones)
        self.mission_waypoints = self.planner.generate_full_mission_plan(ordered)
        self.curr_wp_index = 0
        self.get_logger().info(f"Generated {len(self.mission_waypoints)} waypoints for {len(ordered)} zones.")

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        v = msg.twist.twist.linear

        new_pos = (p.x, p.y, p.z)
        if self.prev_pos is not None and self.fsm.current_state in [MissionState.TAKEOFF, MissionState.SURVEYING, MissionState.RETURNING, MissionState.LANDING]:
            d = math.sqrt((new_pos[0] - self.prev_pos[0])**2 + (new_pos[1] - self.prev_pos[1])**2 + (new_pos[2] - self.prev_pos[2])**2)
            if d < 10.0: # Filter any simulation reset jumps
                self.total_distance_m += d
        self.prev_pos = new_pos

        self.drone_pos = new_pos
        self.drone_yaw = quaternion_to_yaw(o.x, o.y, o.z, qw=o.w)
        self.drone_vel = (v.x, v.y, v.z)

    def battery_callback(self, msg: BatteryState):
        self.battery_pct = msg.percentage * 100.0

    def rth_alert_callback(self, msg: Bool):
        self.battery_low_rth = msg.data
        if self.battery_low_rth and self.fsm.current_state == MissionState.SURVEYING:
            self.get_logger().warn("LOW BATTERY RTH TRIGGERED! Transitioning to RETURNING state.")
            self.saved_resume_wp_index = self.curr_wp_index
            self.fsm.transition_to(MissionState.RETURNING, time.time())

    def command_callback(self, msg: String):
        raw_cmd = msg.data.strip()
        self.get_logger().info(f"Received Mission Command: {raw_cmd}")
        now = time.time()

        try:
            cmd_data = json.loads(raw_cmd)
            action = cmd_data.get("action", "").upper()
        except Exception:
            action = raw_cmd.upper()

        if action == "START":
            if self.fsm.current_state in [MissionState.IDLE, MissionState.COMPLETE]:
                self.mission_id = f"MSN-{int(now)}"
                self.mission_start_time = now
                self.mission_end_time = None
                self.total_distance_m = 0.0
                self.generate_waypoints_plan()
                self.fsm.transition_to(MissionState.PRECHECK, now)
        elif action == "PAUSE":
            if self.fsm.current_state == MissionState.SURVEYING:
                self.fsm.transition_to(MissionState.PAUSED, now)
        elif action == "RESUME":
            if self.fsm.current_state == MissionState.PAUSED:
                self.fsm.transition_to(MissionState.SURVEYING, now)
        elif action == "RTH":
            if self.fsm.current_state in [MissionState.SURVEYING, MissionState.PAUSED, MissionState.TAKEOFF]:
                self.saved_resume_wp_index = self.curr_wp_index
                self.fsm.transition_to(MissionState.RETURNING, now)
        elif action == "ABORT":
            self.fsm.transition_to(MissionState.ABORTED, now)
        elif action == "RESET":
            self.fsm.transition_to(MissionState.IDLE, now)
            self.curr_wp_index = 0
            self.total_distance_m = 0.0

    def control_loop(self):
        now = time.time()
        curr_state = self.fsm.current_state

        cmd_vx = 0.0
        cmd_vy = 0.0
        cmd_vz = 0.0
        cmd_yaw_rate = 0.0

        if curr_state == MissionState.IDLE:
            cmd_vx = cmd_vy = cmd_vz = cmd_yaw_rate = 0.0

        elif curr_state == MissionState.PRECHECK:
            # Automatic pre-flight self tests: check battery > 30%, sensor health
            if self.battery_pct > 30.0 and self.sensor_health["rgb_forward"]:
                self.fsm.transition_to(MissionState.TAKEOFF, now)
            else:
                self.get_logger().error("Precheck failed: battery too low or sensors offline.")
                self.fsm.transition_to(MissionState.IDLE, now)

        elif curr_state == MissionState.TAKEOFF:
            # Vertical climb to mission altitude (15m)
            target_pos = (0.0, 0.0, 15.0)
            cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate, reached = self.controller.compute_cmd_vel(
                self.drone_pos, self.drone_yaw, target_pos, target_speed=3.0
            )
            if self.drone_pos[2] >= 14.0 or reached:
                self.get_logger().info("Takeoff altitude reached. Transitioning to SURVEYING.")
                self.fsm.transition_to(MissionState.SURVEYING, now)

        elif curr_state == MissionState.SURVEYING:
            if self.curr_wp_index < len(self.mission_waypoints):
                target_wp = self.mission_waypoints[self.curr_wp_index]
                target_pos = (target_wp["x"], target_wp["y"], target_wp["z"])
                target_speed = target_wp.get("speed", 4.0)

                cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate, reached = self.controller.compute_cmd_vel(
                    self.drone_pos, self.drone_yaw, target_pos, target_speed
                )
                if reached:
                    self.curr_wp_index += 1
            else:
                # All waypoints surveyed!
                self.get_logger().info("All surveillance waypoints complete! Transitioning to RETURNING.")
                self.fsm.transition_to(MissionState.RETURNING, now)

        elif curr_state == MissionState.PAUSED:
            # Hover in place
            cmd_vx = cmd_vy = cmd_vz = cmd_yaw_rate = 0.0

        elif curr_state == MissionState.RETURNING:
            # Transit directly back to Home Station at safe transit altitude (16m)
            target_pos = (0.0, 0.0, 16.0)
            cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate, reached = self.controller.compute_cmd_vel(
                self.drone_pos, self.drone_yaw, target_pos, target_speed=4.5
            )
            dist_to_pad_xy = math.sqrt(self.drone_pos[0]**2 + self.drone_pos[1]**2)
            if dist_to_pad_xy <= 1.8:
                self.get_logger().info("Aligned above landing pad. Transitioning to LANDING.")
                self.fsm.transition_to(MissionState.LANDING, now)

        elif curr_state == MissionState.LANDING:
            # Descend vertically onto Landing Pad (0, 0, 0.42)
            target_pos = (0.0, 0.0, 0.38)
            cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate, reached = self.controller.compute_cmd_vel(
                self.drone_pos, self.drone_yaw, target_pos, target_speed=1.5
            )
            # Throttle descent rate
            cmd_vz = max(-1.2, min(-0.4, cmd_vz))
            if self.drone_pos[2] <= 0.46:
                cmd_vx = cmd_vy = cmd_vz = cmd_yaw_rate = 0.0
                if self.curr_wp_index < len(self.mission_waypoints) and self.battery_low_rth:
                    self.get_logger().info("Touchdown on inductive charger! Starting CHARGING cycle.")
                    self.fsm.transition_to(MissionState.CHARGING, now)
                else:
                    self.get_logger().info("Mission COMPLETE! Drone safely secured on landing pad.")
                    self.mission_end_time = now
                    self.fsm.transition_to(MissionState.COMPLETE, now)

        elif curr_state == MissionState.CHARGING:
            cmd_vx = cmd_vy = cmd_vz = cmd_yaw_rate = 0.0
            if self.battery_pct >= 95.0:
                self.get_logger().info("Battery charged to 95%+. Resuming mission.")
                self.fsm.transition_to(MissionState.RESUMING, now)

        elif curr_state == MissionState.RESUMING:
            # Takeoff to transit altitude then continue from saved waypoint
            target_pos = (0.0, 0.0, 15.0)
            cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate, reached = self.controller.compute_cmd_vel(
                self.drone_pos, self.drone_yaw, target_pos, target_speed=2.5
            )
            if self.drone_pos[2] >= 14.0 or reached:
                self.curr_wp_index = self.saved_resume_wp_index
                self.fsm.transition_to(MissionState.SURVEYING, now)

        elif curr_state == MissionState.ABORTED:
            # Emergency descent
            target_pos = (self.drone_pos[0], self.drone_pos[1], 0.38)
            cmd_vx, cmd_vy, cmd_vz, cmd_yaw_rate, reached = self.controller.compute_cmd_vel(
                self.drone_pos, self.drone_yaw, target_pos, target_speed=1.5
            )
            if self.drone_pos[2] <= 0.46:
                self.fsm.transition_to(MissionState.IDLE, now)

        # Publish Twist command
        t_msg = Twist()
        t_msg.linear.x = float(cmd_vx)
        t_msg.linear.y = float(cmd_vy)
        t_msg.linear.z = float(cmd_vz)
        t_msg.angular.z = float(cmd_yaw_rate)
        self.cmd_vel_pub.publish(t_msg)

        # Publish Mission State String
        s_msg = String()
        s_msg.data = curr_state.value
        self.state_pub.publish(s_msg)

        # Publish Full Telemetry JSON
        total_wps = len(self.mission_waypoints)
        progress_pct = round((self.curr_wp_index / max(1, total_wps)) * 100.0, 1) if total_wps > 0 else 0.0
        current_zone_id = ZoneManager.which_zone(self.drone_pos[0], self.drone_pos[1])

        telemetry_data = {
            "mission_id": self.mission_id,
            "state": curr_state.value,
            "position": {
                "x": round(self.drone_pos[0], 2),
                "y": round(self.drone_pos[1], 2),
                "z": round(self.drone_pos[2], 2)
            },
            "velocity": {
                "vx": round(self.drone_vel[0], 2),
                "vy": round(self.drone_vel[1], 2),
                "vz": round(self.drone_vel[2], 2),
                "speed_ms": round(math.sqrt(self.drone_vel[0]**2 + self.drone_vel[1]**2 + self.drone_vel[2]**2), 2)
            },
            "yaw_deg": round(math.degrees(self.drone_yaw), 1),
            "current_zone": current_zone_id,
            "current_wp_index": self.curr_wp_index,
            "total_waypoints": total_wps,
            "progress_pct": progress_pct,
            "distance_flown_m": round(self.total_distance_m, 1),
            "battery_pct": round(self.battery_pct, 1),
            "mission_duration_sec": round((now - self.mission_start_time) if self.mission_start_time else 0.0, 1),
            "sensor_health": self.sensor_health
        }
        tel_msg = String()
        tel_msg.data = json.dumps(telemetry_data)
        self.telemetry_pub.publish(tel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
