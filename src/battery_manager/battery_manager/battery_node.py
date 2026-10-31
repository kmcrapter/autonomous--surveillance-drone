"""ROS 2 Node for Battery Physics and RTH Threshold Computation."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import json
import time

from battery_manager.battery_model import BatteryModel

class BatteryManagerNode(Node):
    def __init__(self):
        super().__init__('battery_manager_node')

        self.declare_parameter('capacity_wh', 555.0)
        self.declare_parameter('safety_reserve_pct', 20.0)
        self.declare_parameter('charge_rate_pct_sec', 2.5)

        cap_wh = self.get_parameter('capacity_wh').value
        reserve = self.get_parameter('safety_reserve_pct').value
        charge_rate = self.get_parameter('charge_rate_pct_sec').value

        self.model = BatteryModel(
            capacity_wh=cap_wh,
            safety_reserve_pct=reserve,
            charge_rate_pct_sec=charge_rate
        )

        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_z = 0.38
        self.curr_vx = 0.0
        self.curr_vy = 0.0
        self.curr_vz = 0.0
        self.flight_state = "IDLE"
        self.last_tick = time.time()

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odometry',
            self.odom_callback,
            10
        )
        self.state_sub = self.create_subscription(
            String,
            '/drone/mission_state',
            self.mission_state_callback,
            10
        )

        # Publishers
        self.batt_pub = self.create_publisher(BatteryState, '/drone/battery_state', 10)
        self.telemetry_pub = self.create_publisher(String, '/drone/battery_telemetry', 10)
        self.rth_alert_pub = self.create_publisher(Bool, '/drone/battery_low_rth_alert', 10)

        # 5 Hz timer loop
        self.timer = self.create_timer(0.2, self.update_loop)
        self.get_logger().info("BatteryManagerNode initialized (6S 555Wh model, dynamic RTH monitoring active).")

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        self.curr_x = pos.x
        self.curr_y = pos.y
        self.curr_z = pos.z
        self.curr_vx = vel.x
        self.curr_vy = vel.y
        self.curr_vz = vel.z

    def mission_state_callback(self, msg: String):
        self.flight_state = msg.data.strip()

    def update_loop(self):
        now = time.time()
        dt = now - self.last_tick
        self.last_tick = now

        is_flying = (self.curr_z > 0.8) or (self.flight_state in ["TAKEOFF", "SURVEYING", "RETURNING", "LANDING"])
        is_charging = (self.flight_state == "CHARGING")

        status = self.model.update(
            dt=dt,
            vx=self.curr_vx,
            vy=self.curr_vy,
            vz=self.curr_vz,
            altitude=self.curr_z,
            is_flying=is_flying,
            charging_override=is_charging
        )

        rth_req = self.model.calculate_rth_requirements(self.curr_x, self.curr_y, self.curr_z)

        # Publish ROS 2 BatteryState
        b_msg = BatteryState()
        b_msg.header.stamp = self.get_clock().now().to_msg()
        b_msg.voltage = float(status["voltage"])
        b_msg.current = float(status["current_a"])
        b_msg.charge = float(status["remaining_wh"] / status["voltage"])
        b_msg.capacity = float(status["capacity_wh"] / status["voltage"])
        b_msg.percentage = float(status["percentage"] / 100.0)
        b_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if is_charging else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.batt_pub.publish(b_msg)

        # Combined telemetry JSON
        telemetry_payload = {
            "timestamp": now,
            "battery": status,
            "rth": rth_req,
            "flight_state": self.flight_state
        }
        str_msg = String()
        str_msg.data = json.dumps(telemetry_payload)
        self.telemetry_pub.publish(str_msg)

        # Low battery RTH alert
        rth_alert = Bool()
        rth_alert.data = bool(rth_req["should_return"] and is_flying)
        self.rth_alert_pub.publish(rth_alert)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
