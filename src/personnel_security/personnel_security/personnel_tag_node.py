"""ROS 2 Node for Simulated Electronic Personnel Identification Tags (Layer 1).

Simulates UWB Beacons, Encrypted RFID/NFC, BLE Tags, and Wearable Security Modules,
including RF propagation, RSSI path-loss, cryptographic packet formatting, and scenario injection.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import json
import time
import math
import secrets
from typing import Dict, Any, List, Tuple

from personnel_security.crypto_utils import generate_tag_signature
from personnel_security.personnel_registry import SYNTHETIC_PERSONNEL_REGISTRY

SCENARIO_STATE_FILE = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/personnel_scenario.json"

class PersonnelTagNode(Node):
    def __init__(self):
        super().__init__('personnel_tag_node')

        self.drone_pos = (0.0, 0.0, 0.49)
        self.active_scenario = "DEFAULT"

        # Drone Odometry Subscription
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odometry',
            self.odom_callback,
            qos_profile_sensor_data
        )

        # Scenario Control Subscription
        self.scenario_sub = self.create_subscription(
            String,
            '/personnel/scenario_control',
            self.scenario_callback,
            10
        )

        # Raw Tag Broadcast Publisher
        self.raw_tag_pub = self.create_publisher(
            String,
            '/personnel/raw_tags',
            10
        )

        # 5 Hz RF transmission cycle
        self.timer = self.create_timer(0.2, self.broadcast_tags)
        self.get_logger().info("PersonnelTagNode initialized (UWB / BLE / RFID RF Simulation).")

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.drone_pos = (p.x, p.y, p.z)

    def scenario_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.active_scenario = data.get("scenario", self.active_scenario).upper()
            self.get_logger().info(f"Switched personnel simulation scenario to: {self.active_scenario}")
        except Exception as e:
            self.get_logger().warn(f"Failed to parse scenario control payload: {e}")

    def compute_rf_link(self, person_pos: Tuple[float, float], max_range_m: float) -> Tuple[bool, float, float]:
        """Calculates distance, RF reception, and RSSI (dBm) with log-distance path loss."""
        dx = person_pos[0] - self.drone_pos[0]
        dy = person_pos[1] - self.drone_pos[1]
        dz = 0.9 - self.drone_pos[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist > max_range_m:
            return False, dist, -110.0

        # Log-distance path loss model: RSSI(d) = RSSI_0 - 10*n*log10(d)
        rssi_0 = -42.0 # dBm at 1m
        path_loss_exp = 2.2 # slightly lossy terrain
        rssi = rssi_0 - (10.0 * path_loss_exp * math.log10(max(1.0, dist)))
        return True, round(dist, 1), round(rssi, 1)

    def broadcast_tags(self):
        now = time.time()
        emitted_packets = []

        # Read scenario file if present
        try:
            with open(SCENARIO_STATE_FILE, "r") as f:
                sc_data = json.load(f)
                self.active_scenario = sc_data.get("active_scenario", self.active_scenario)
        except Exception:
            pass

        # 1. Base Authorized Personnel Tags
        for pid, pdata in SYNTHETIC_PERSONNEL_REGISTRY.items():
            tag_id = pdata["tag_id"]
            tag_type = pdata["tag_type"]
            coords = pdata["default_world_coords"]

            # Scenario B: Capt. Sarah Mitchell's tag is temporarily unpowered/depleted
            if pid == "PERS-102" and self.active_scenario in ["SCENARIO_B", "OFFICER_TAG_DEPLETED"]:
                continue

            # Determine maximum detection range based on device technology
            max_range = 85.0 if tag_type == "UWB_BEACON" else (35.0 if tag_type == "ENCRYPTED_RFID" else 65.0)

            in_range, dist, rssi = self.compute_rf_link(coords, max_range)
            if in_range:
                nonce = secrets.token_hex(8)
                signature = generate_tag_signature(tag_id, nonce, now, coords)
                packet = {
                    "tag_id": tag_id,
                    "personnel_id": pid,
                    "hardware_uuid": f"HW-UUID-{pid}-{tag_type[:3]}",
                    "tag_type": tag_type,
                    "device_model": pdata["device_model"],
                    "timestamp": now,
                    "nonce": nonce,
                    "signature": signature,
                    "rssi_dbm": rssi,
                    "estimated_distance_m": dist,
                    "simulated_coords": {"x": coords[0], "y": coords[1]},
                    "battery_level_pct": 94.0,
                    "assigned_unit": pdata["assigned_unit"],
                    "clearance_level": pdata["clearance_level"]
                }
                emitted_packets.append(packet)

        # 2. Scenario C: Cloned / Conflicting Tag Token
        if self.active_scenario in ["SCENARIO_C", "CREDENTIAL_CONFLICT_TEST"]:
            clone_coords = (55.0, 30.0)
            in_range, dist, rssi = self.compute_rf_link(clone_coords, 85.0)
            if in_range:
                clone_nonce = secrets.token_hex(8)
                clone_sig = generate_tag_signature("TAG-UWB-101", clone_nonce, now, clone_coords)
                clone_packet = {
                    "tag_id": "TAG-UWB-101",
                    "personnel_id": "PERS-101",
                    "hardware_uuid": "HW-CLONE-UNAUTH-77X",
                    "tag_type": "UWB_BEACON",
                    "device_model": "Unauthorized Rogue Transceiver",
                    "timestamp": now,
                    "nonce": clone_nonce,
                    "signature": clone_sig,
                    "rssi_dbm": rssi,
                    "estimated_distance_m": dist,
                    "simulated_coords": {"x": clone_coords[0], "y": clone_coords[1]},
                    "battery_level_pct": 99.0,
                    "assigned_unit": "1st Recon Command / Alpha Outpost",
                    "clearance_level": "LEVEL-4_COMMAND"
                }
                emitted_packets.append(clone_packet)

        # 3. Scenario Unknown Tag (Uncatalogued device)
        if self.active_scenario in ["SCENARIO_UNKNOWN_TAG", "SCENARIO_C"]:
            unk_coords = (15.0, 38.0)
            in_range, dist, rssi = self.compute_rf_link(unk_coords, 45.0)
            if in_range:
                unk_nonce = secrets.token_hex(8)
                unk_packet = {
                    "tag_id": "TAG-UNK-994",
                    "personnel_id": "UNKNOWN",
                    "hardware_uuid": "HW-UNK-VENDOR-9941A",
                    "tag_type": "BLE_BEACON",
                    "device_model": "Generic Commercial BLE Tag",
                    "timestamp": now,
                    "nonce": unk_nonce,
                    "signature": "invalid_unregistered_sig_994",
                    "rssi_dbm": rssi,
                    "estimated_distance_m": dist,
                    "simulated_coords": {"x": unk_coords[0], "y": unk_coords[1]},
                    "battery_level_pct": 82.0,
                    "assigned_unit": "UNASSIGNED",
                    "clearance_level": "NONE"
                }
                emitted_packets.append(unk_packet)

        # Publish raw tag packets
        msg = String()
        msg.data = json.dumps({"timestamp": now, "count": len(emitted_packets), "tags": emitted_packets})
        self.raw_tag_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonnelTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
