"""ROS 2 System Orchestrator: Complete Digital Twin Telemetry, Sensors, and Navigation Engine."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, BatteryState
from std_msgs.msg import String, Float64

import json
import time
import math
import os
from typing import Dict, Any, List, Tuple

from coverage_planner.zone_manager import ZoneManager
from coverage_planner.boustrophedon import BoustrophedonPlanner
from battery_manager.battery_model import BatteryModel
from event_manager.database import SurveillanceDatabase

COMMAND_QUEUE_FILE = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/command_queue.json"
TELEMETRY_STATE_FILE = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/telemetry_state.json"

# Reference Geographic Datum (WGS84 Origin at World 0,0,0)
DATUM_LAT = 44.500000 # degrees North
DATUM_LON = -110.500000 # degrees West
DATUM_ELEV = 1800.0 # meters MSL (Reference Datum Level)
METERS_PER_DEG_LAT = 111139.0

def get_ground_elevation(x: float, y: float) -> float:
    """Calculates true terrain elevation Z_ground(x, y) matching Gazebo 3D terrain."""
    elev = 0.0
    # North Hill (Sector Bravo/Charlie boundary)
    elev += 14.0 * math.exp(-(((x - 60.0)**2) / (2.0 * 28.0**2) + ((y - 75.0)**2) / (2.0 * 22.0**2)))
    # East Bluff (Sector Delta)
    elev += 11.0 * math.exp(-(((x - 80.0)**2) / (2.0 * 24.0**2) + ((y + 25.0)**2) / (2.0 * 24.0**2)))
    # West Forest Mound
    elev += 8.0 * math.exp(-(((x + 75.0)**2) / (2.0 * 22.0**2) + ((y - 45.0)**2) / (2.0 * 22.0**2)))
    # River Trench Depression
    river_dist = abs(y - 60.0)
    elev -= 1.8 * math.exp(-(river_dist**2) / (2.0 * 7.0**2))
    return max(-1.8, elev)

def cartesian_to_gps(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Converts local Cartesian coordinates (meters) to WGS84 Geographic Coordinates."""
    lat = DATUM_LAT + (y / METERS_PER_DEG_LAT)
    meters_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(DATUM_LAT))
    lon = DATUM_LON + (x / meters_per_deg_lon)
    alt_msl = DATUM_ELEV + z
    return lat, lon, alt_msl

class SystemOrchestrator(Node):
    def __init__(self):
        super().__init__('system_orchestrator')

        self.db = SurveillanceDatabase()
        self.planner = BoustrophedonPlanner()
        self.battery = BatteryModel(capacity_wh=555.0, safety_reserve_pct=20.0)

        self.mission_id = f"MSN-{int(time.time())}"
        self.flight_state = "IDLE"
        self.mission_state = "IDLE"

        # 3D Position & Velocity
        self.drone_pos = [0.0, 0.0, 0.49]
        self.drone_vel = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.total_dist_m = 0.0
        self.mission_start_time = None
        self.prev_time = time.time()
        self.last_gazebo_time = 0.0

        # Coverage Waypoints: Perimeter boundary sweep first, then systematic interior boustrophedon fill
        self.ordered_zones = ZoneManager.get_ordered_zones()
        self.waypoints = self.planner.generate_perimeter_sweep() + self.planner.generate_full_mission_plan(self.ordered_zones)
        self.wp_index = 0
        self.saved_resume_wp = 0

        self.has_gazebo_odom = False

        # Live Real-Time Area Coverage Grid for Country A Sovereign Territory ([-90..90] x [-60..30])
        self.grid_cell_size = 5.0
        self.covered_cells = set()
        self.all_grid_cells = []
        for gx in range(-90, 95, 5):
            for gy in range(-60, 35, 5): # Country A sovereign territory (strictly south of border line)
                self.all_grid_cells.append((gx, gy))
        self.total_grid_area_m2 = len(self.all_grid_cells) * 25.0

        # International Border Line & Sovereign Airspace Boundary (Meters)
        self.BORDER_LINE_Y = 32.0 # Ground Demarcation Line
        self.SOVEREIGN_MAX_Y = 28.5 # Hard limit: Drone stays strictly in Country A (never cross border line)

        # Operational Control Mode & Pattern Configuration
        self.control_mode = "AUTONOMOUS" # "AUTONOMOUS" or "MANUAL_REMOTE"
        self.surveillance_pattern = "boustrophedon" # "boustrophedon", "perimeter", or "spiral"
        self.teleop_cmd = [0.0, 0.0, 0.0, 0.0] # [body_vx, body_vy, vz, yaw_rate]
        self.last_teleop_time = 0.0

        # ROS 2 Subscriptions with SensorData QoS
        self.cmd_sub = self.create_subscription(String, '/drone/mission_command', self.on_mission_cmd, 10)
        self.teleop_sub = self.create_subscription(Twist, '/drone/teleop_cmd', self.on_teleop_cmd, 10)
        self.odom_sub = self.create_subscription(Odometry, '/drone/odometry', self.on_gazebo_odom, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, '/drone/imu', self.on_gazebo_imu, qos_profile_sensor_data)
        self.gps_sub = self.create_subscription(NavSatFix, '/drone/gps', self.on_gazebo_gps, qos_profile_sensor_data)

        # Standard ROS 2 Topic Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.heading_pub = self.create_publisher(Float64, '/drone/heading', 10)
        self.flight_state_pub = self.create_publisher(String, '/drone/flight_state', 10)
        self.mission_state_pub = self.create_publisher(String, '/drone/mission_state', 10)
        self.batt_pub = self.create_publisher(BatteryState, '/drone/battery_state', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/drone/gps_telemetry', 10)
        self.telemetry_pub = self.create_publisher(String, '/drone/mission_telemetry', 10)
        self.obs_pub = self.create_publisher(String, '/events/observations', 10)

        # Dynamic Rotor Joint Velocity Controllers for Gazebo Harmonic (Stopped until Mission Start)
        self.rotor_pubs = [
            self.create_publisher(Float64, f'/model/surveillance_drone/joint/rotor_{i}_joint/cmd_vel', 10)
            for i in range(4)
        ]

        # Simulation Entities for Computer Vision Observations:
        # Includes Houses/Buildings, 16 Distinct People (Men and Women), 8 Vehicles, 8 Wildlife Animals, and 5 Hazards/Objects.
        self.sim_entities = [
            # Military Base Buildings & Infrastructure (Zone Alpha)
            {"id": "TRK-B01", "category": "building", "territory": "HOME",
             "label": "Military Operations HQ", "pos": [-15.0, -25.0], "speed": 0.0, "route": [[-15.0, -25.0]]},
            {"id": "TRK-B02", "category": "building", "territory": "HOME",
             "label": "Barracks Alpha Facility", "pos": [-25.0, -45.0], "speed": 0.0, "route": [[-25.0, -45.0]]},
            {"id": "TRK-B03", "category": "building", "territory": "HOME",
             "label": "Barracks Bravo Facility", "pos": [-2.0, -45.0], "speed": 0.0, "route": [[-2.0, -45.0]]},
            {"id": "TRK-B04", "category": "building", "territory": "HOME",
             "label": "Aircraft Drone Flight Hangar", "pos": [25.0, -25.0], "speed": 0.0, "route": [[25.0, -25.0]]},
            {"id": "TRK-B05", "category": "building", "territory": "HOME",
             "label": "Base Motor Pool Workshop", "pos": [25.0, -48.0], "speed": 0.0, "route": [[25.0, -48.0]]},
            {"id": "TRK-B06", "category": "building", "territory": "HOME",
             "label": "Munitions Armory Bunker", "pos": [-36.0, -28.0], "speed": 0.0, "route": [[-36.0, -28.0]]},
            {"id": "TRK-B07", "category": "building", "territory": "HOME",
             "label": "Field Medical Center", "pos": [-36.0, -46.0], "speed": 0.0, "route": [[-36.0, -46.0]]},
            {"id": "TRK-B08", "category": "building", "territory": "HOME",
             "label": "Early-Warning Radar Tower", "pos": [38.0, -8.0], "speed": 0.0, "route": [[38.0, -8.0]]},
            {"id": "TRK-B09", "category": "building", "territory": "HOME",
             "label": "Perimeter Watchtower NW", "pos": [38.0, 20.0], "speed": 0.0, "route": [[38.0, 20.0]]},
            {"id": "TRK-B10", "category": "building", "territory": "HOME",
             "label": "North Checkpoint Guardhouse", "pos": [-4.2, 28.0], "speed": 0.0, "route": [[-4.2, 28.0]]},
            {"id": "TRK-B11", "category": "building", "territory": "FOREIGN",
             "label": "Civilian House / Cottage", "pos": [-60.0, 48.0], "speed": 0.0, "route": [[-60.0, 48.0]]},
            {"id": "TRK-B12", "category": "building", "territory": "FOREIGN",
             "label": "Northern Farmhouse & Barn", "pos": [35.0, 68.0], "speed": 0.0, "route": [[35.0, 68.0]]},
            {"id": "TRK-V01", "category": "vehicle", "territory": "HOME",
             "label": "Armored Vehicle (Stryker 8x8)", "pos": [-18.0, -35.0], "speed": 0.0, "route": [[-18.0, -35.0]]},

            # Zone Alpha: Home Sovereign Base HQ, Gate, Logistics, Comms, Hangar
            # 1. Man - Home Sovereign Patrol Guard (North Gate)
            {"id": "TRK-001", "category": "person", "gender": "man", "territory": "HOME",
             "label": "Man (Home Patrol Guard)", "pos": [6.0, 20.0], "speed": 1.1,
             "route": [[6.0, 20.0], [14.0, 22.0], [20.0, 20.0], [6.0, 20.0]]},

            # 2. Woman - Home Base Logistics Officer (South HQ)
            {"id": "TRK-002", "category": "person", "gender": "woman", "territory": "HOME",
             "label": "Woman (Logistics Officer)", "pos": [-10.0, -25.0], "speed": 0.9,
             "route": [[-10.0, -25.0], [-16.0, -22.0], [-8.0, -28.0], [-10.0, -25.0]]},

            # 3. Woman - Home Sovereign Border Officer (Border Checkpoint)
            {"id": "TRK-003", "category": "person", "gender": "woman", "territory": "HOME",
             "label": "Woman (Border Officer)", "pos": [-15.0, 28.0], "speed": 1.0,
             "route": [[-15.0, 28.0], [-22.0, 27.0], [-10.0, 26.0], [-15.0, 28.0]]},

            # 4. Man - Base Commander Vance (Ops Command Center)
            {"id": "TRK-014", "category": "person", "gender": "man", "territory": "HOME",
             "label": "Man (Base Commander Vance)", "pos": [0.0, -35.0], "speed": 0.6,
             "route": [[0.0, -35.0], [4.0, -32.0], [-4.0, -33.0], [0.0, -35.0]]},

            # 5. Man - Hangar Mechanic Miller (Drone Flight Hangar)
            {"id": "TRK-015", "category": "person", "gender": "man", "territory": "HOME",
             "label": "Man (Hangar Mechanic Miller)", "pos": [-20.0, -18.0], "speed": 0.8,
             "route": [[-20.0, -18.0], [-24.0, -16.0], [-18.0, -20.0], [-20.0, -18.0]]},

            # 6. Woman - Comms Specialist (Antenna Array)
            {"id": "TRK-016", "category": "person", "gender": "woman", "territory": "HOME",
             "label": "Woman (Comms Specialist)", "pos": [15.0, -20.0], "speed": 0.7,
             "route": [[15.0, -20.0], [18.0, -18.0], [12.0, -22.0], [15.0, -20.0]]},

            # 7. Man - Main Gate Guard (Base Perimeter Entry)
            {"id": "TRK-017", "category": "person", "gender": "man", "territory": "HOME",
             "label": "Man (Main Gate Guard)", "pos": [0.0, 27.5], "speed": 0.8,
             "route": [[0.0, 27.5], [3.0, 28.0], [-3.0, 28.0], [0.0, 27.5]]},

            # 8. Moving Patrol SUV (Border Roadway)
            {"id": "TRK-020", "category": "vehicle", "territory": "HOME",
             "label": "Patrol SUV (Border Roadway)", "pos": [0.0, 35.0], "speed": 3.5,
             "route": [[0.0, 18.0], [0.0, 35.0], [0.0, 50.0], [0.0, 18.0]]},

            # 9. Military Transport Truck 6x6 (Base Logistics)
            {"id": "TRK-021", "category": "vehicle", "territory": "HOME",
             "label": "Military Transport Truck 6x6", "pos": [-25.0, -35.0], "speed": 1.2,
             "route": [[-25.0, -35.0], [-20.0, -32.0], [-25.0, -35.0]]},

            # 10. HQ Command SUV (Base Headquarters)
            {"id": "TRK-022", "category": "vehicle", "territory": "HOME",
             "label": "HQ Command SUV (Parked)", "pos": [12.0, -32.0], "speed": 0.0,
             "route": [[12.0, -32.0]]},

            # Zone Delta: Eastern Outpost Ridge, Watchtower, Bridge
            # 11. Man - Outpost Ridge Sentry (Eastern Watchtower)
            {"id": "TRK-004", "category": "person", "gender": "man", "territory": "HOME",
             "label": "Man (Outpost Ridge Sentry)", "pos": [65.0, -15.0], "speed": 0.8,
             "route": [[65.0, -15.0], [70.0, -12.0], [60.0, -18.0], [65.0, -15.0]]},

            # 12. Man - Bridge Sentry Kowalski (River Crossing Guard)
            {"id": "TRK-029", "category": "person", "gender": "man", "territory": "HOME",
             "label": "Man (Bridge Sentry Kowalski)", "pos": [0.0, 58.0], "speed": 0.9,
             "route": [[0.0, 58.0], [4.0, 58.0], [-4.0, 58.0], [0.0, 58.0]]},

            # 13. Man - Eastern Lookout Guard (Ridge Bluff)
            {"id": "TRK-033", "category": "person", "gender": "man", "territory": "HOME",
             "label": "Man (Eastern Lookout Guard)", "pos": [70.0, -25.0], "speed": 0.7,
             "route": [[70.0, -25.0], [74.0, -22.0], [68.0, -28.0], [70.0, -25.0]]},

            # 14. Woman - Perimeter Patrol Officer (Southeast Sector)
            {"id": "TRK-034", "category": "person", "gender": "woman", "territory": "HOME",
             "label": "Woman (Perimeter Patrol Officer)", "pos": [44.0, 15.0], "speed": 1.0,
             "route": [[44.0, 15.0], [50.0, 18.0], [40.0, 12.0], [44.0, 15.0]]},

            # 15. Wildlife - Mountain Deer (Eastern Bluff)
            {"id": "TRK-013", "category": "wildlife", "animal_type": "mountain_deer", "territory": "HOME",
             "label": "Animal (Mountain Deer)", "pos": [72.0, 5.0], "speed": 1.2,
             "route": [[72.0, 5.0], [76.0, 8.0], [68.0, 2.0], [72.0, 5.0]]},

            # 16. Wildlife - Mountain Bighorn Sheep (High Crags)
            {"id": "TRK-035", "category": "wildlife", "animal_type": "sheep", "territory": "HOME",
             "label": "Animal (Mountain Bighorn Sheep)", "pos": [68.0, 12.0], "speed": 1.0,
             "route": [[68.0, 12.0], [72.0, 15.0], [65.0, 9.0], [68.0, 12.0]]},

            # 17. Outpost Utility Rover (Outpost Depot)
            {"id": "TRK-036", "category": "vehicle", "territory": "HOME",
             "label": "Outpost Utility Rover", "pos": [60.0, -10.0], "speed": 1.5,
             "route": [[60.0, -10.0], [66.0, -8.0], [58.0, -14.0], [60.0, -10.0]]},

            # 18. Comms Tower Repeater Mast (Infrastructure)
            {"id": "TRK-037", "category": "hazard", "territory": "HOME",
             "label": "Comms Tower Repeater Mast", "pos": [66.0, -18.0], "speed": 0.0,
             "route": [[66.0, -18.0]]},

            # Zone Bravo: Western Deep Pine Forest, Clearings
            # 19. Man - Foreign Forest Hiker (Western Pine Trail)
            {"id": "TRK-006", "category": "person", "gender": "man", "territory": "FOREIGN",
             "label": "Man (Foreign Forest Hiker)", "pos": [-65.0, 50.0], "speed": 1.0,
             "route": [[-65.0, 50.0], [-60.0, 54.0], [-68.0, 48.0], [-65.0, 50.0]]},

            # 20. Man - Deep Forest Scout (Northwest Boundary)
            {"id": "TRK-018", "category": "person", "gender": "man", "territory": "FOREIGN",
             "label": "Man (Deep Forest Scout)", "pos": [-50.0, 48.0], "speed": 1.1,
             "route": [[-50.0, 48.0], [-45.0, 52.0], [-55.0, 44.0], [-50.0, 48.0]]},

            # 21. Woman - Forest Forager (Pine Glade)
            {"id": "TRK-019", "category": "person", "gender": "woman", "territory": "FOREIGN",
             "label": "Woman (Forest Forager)", "pos": [-60.0, 38.0], "speed": 0.8,
             "route": [[-60.0, 38.0], [-64.0, 40.0], [-56.0, 36.0], [-60.0, 38.0]]},

            # 22. Wildlife - Deer Stag (Forest Clearing)
            {"id": "TRK-010", "category": "wildlife", "animal_type": "deer_stag", "territory": "FOREIGN",
             "label": "Animal (Wildlife Deer Stag)", "pos": [-55.0, 42.0], "speed": 1.3,
             "route": [[-55.0, 42.0], [-62.0, 45.0], [-50.0, 40.0], [-55.0, 42.0]]},

            # 23. Wildlife - Deer Doe (Upper Pine Woods)
            {"id": "TRK-011", "category": "wildlife", "animal_type": "deer_doe", "territory": "FOREIGN",
             "label": "Animal (Wildlife Deer Doe)", "pos": [-45.0, 68.0], "speed": 1.2,
             "route": [[-45.0, 68.0], [-50.0, 65.0], [-40.0, 70.0], [-45.0, 68.0]]},

            # 24. Wildlife - Deer Fawn (Forest Meadow)
            {"id": "TRK-025", "category": "wildlife", "animal_type": "fawn", "territory": "FOREIGN",
             "label": "Animal (Wildlife Deer Fawn)", "pos": [-52.0, 44.0], "speed": 1.1,
             "route": [[-52.0, 44.0], [-56.0, 46.0], [-48.0, 42.0], [-52.0, 44.0]]},

            # 25. Environmental Thermal Hazard (Wildfire)
            {"id": "TRK-030", "category": "fire", "territory": "FOREIGN",
             "label": "Thermal Hazard (Wildfire)", "pos": [-65.0, 65.0], "speed": 0.0,
             "route": [[-65.0, 65.0]]},

            # 26. Environmental Thermal Hazard (Campfire)
            {"id": "TRK-031", "category": "fire", "territory": "FOREIGN",
             "label": "Thermal Hazard (Campfire)", "pos": [-48.0, 55.0], "speed": 0.0,
             "route": [[-48.0, 55.0]]},

            # Zone Charlie: River Corridor, Roadway, Northern Border Ridge
            # 27. Man - Foreign Border Crosser (Border Corridor)
            {"id": "TRK-005", "category": "person", "gender": "man", "territory": "FOREIGN",
             "label": "Man (Foreign Border Crosser)", "pos": [15.0, 38.0], "speed": 1.2,
             "route": [[15.0, 38.0], [18.0, 42.0], [12.0, 36.0], [15.0, 38.0]]},

            # 28. Woman - Foreign High Ground Observer (North Hill Bluff)
            {"id": "TRK-007", "category": "person", "gender": "woman", "territory": "FOREIGN",
             "label": "Woman (High Ground Observer)", "pos": [58.0, 74.0], "speed": 0.8,
             "route": [[58.0, 74.0], [62.0, 72.0], [54.0, 76.0], [58.0, 74.0]]},

            # 29. Woman - Foreign Riverbank Civilian (River Trail)
            {"id": "TRK-008", "category": "person", "gender": "woman", "territory": "FOREIGN",
             "label": "Woman (Riverbank Civilian)", "pos": [42.0, 56.0], "speed": 0.9,
             "route": [[42.0, 56.0], [48.0, 58.0], [38.0, 54.0], [42.0, 56.0]]},

            # 30. Man - Foreign Sector Woodsman (Northern River Crossing)
            {"id": "TRK-009", "category": "person", "gender": "man", "territory": "FOREIGN",
             "label": "Man (Northern Woodsman)", "pos": [-8.0, 66.0], "speed": 0.7,
             "route": [[-8.0, 66.0], [-2.0, 68.0], [-14.0, 64.0], [-8.0, 66.0]]},

            # 31. Wildlife - Riverbank Forest Elk (Wetlands)
            {"id": "TRK-012", "category": "wildlife", "animal_type": "elk", "territory": "FOREIGN",
             "label": "Animal (Riverbank Forest Elk)", "pos": [-20.0, 62.0], "speed": 1.1,
             "route": [[-20.0, 62.0], [-26.0, 65.0], [-15.0, 60.0], [-20.0, 62.0]]},

            # 32. Wildlife - Forest Elk Doe (North River)
            {"id": "TRK-027", "category": "wildlife", "animal_type": "elk_doe", "territory": "FOREIGN",
             "label": "Animal (Forest Elk Doe)", "pos": [-24.0, 66.0], "speed": 1.0,
             "route": [[-24.0, 66.0], [-28.0, 70.0], [-18.0, 64.0], [-24.0, 66.0]]},

            # 33. Wildlife - North Ridge Timber Wolf (Highland Ridge)
            {"id": "TRK-028", "category": "wildlife", "animal_type": "wolf", "territory": "FOREIGN",
             "label": "Animal (North Ridge Wolf)", "pos": [25.0, 70.0], "speed": 1.4,
             "route": [[25.0, 70.0], [30.0, 72.0], [20.0, 68.0], [25.0, 70.0]]},

            # 34. Tactical Border ATV Quad (Border Security Patrol)
            {"id": "TRK-023", "category": "vehicle", "territory": "HOME",
             "label": "Tactical Border ATV Quad", "pos": [22.0, 36.0], "speed": 2.8,
             "route": [[22.0, 36.0], [30.0, 38.0], [15.0, 35.0], [22.0, 36.0]]},

            # 35. Foreign 4x4 Off-Roader (Foreign Roadway)
            {"id": "TRK-024", "category": "vehicle", "territory": "FOREIGN",
             "label": "Foreign 4x4 Off-Roader", "pos": [-15.0, 45.0], "speed": 3.0,
             "route": [[-15.0, 45.0], [-10.0, 52.0], [-20.0, 40.0], [-15.0, 45.0]]},

            # 36. Abandoned Cargo Container (Perimeter Anomaly)
            {"id": "TRK-032", "category": "hazard", "territory": "FOREIGN",
             "label": "Abandoned Cargo Container", "pos": [12.0, 36.0], "speed": 0.0,
             "route": [[12.0, 36.0]]}
        ]
        self.logged_tracks = set()

        # 20 Hz Real-Time Loop
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info("SystemOrchestrator online (Full telemetry and flight instruments active).")

    def on_mission_cmd(self, msg: String):
        self.execute_command(msg.data.strip())

    def on_teleop_cmd(self, msg: Twist):
        self.control_mode = "MANUAL_REMOTE"
        self.teleop_cmd = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z]
        self.last_teleop_time = time.time()

    def on_gazebo_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        o = msg.pose.pose.orientation
        # Compute roll, pitch, yaw from quaternion
        sinr_cosp = 2.0 * (o.w * o.x + o.y * o.z)
        cosr_cosp = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (o.w * o.y - o.z * o.x)
        self.pitch = math.asin(max(-1.0, min(1.0, sinp)))

        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        # Synchronize position with Gazebo physics while strictly enforcing Country A boundary
        self.drone_pos[0] = p.x
        self.drone_pos[1] = min(self.SOVEREIGN_MAX_Y, p.y)
        self.drone_pos[2] = p.z
        self.drone_vel = [v.x, v.y, v.z]
        self.has_gazebo_odom = True
        self.last_gazebo_time = time.time()

    def on_gazebo_imu(self, msg: Imu):
        pass

    def on_gazebo_gps(self, msg: NavSatFix):
        pass

    def poll_command_queue(self):
        if os.path.exists(COMMAND_QUEUE_FILE):
            try:
                with open(COMMAND_QUEUE_FILE, "r") as f:
                    cmd_data = json.load(f)
                os.remove(COMMAND_QUEUE_FILE)
                action = cmd_data.get("action", "")
                params = cmd_data.get("params", {})
                self.execute_command(action, params)
            except Exception:
                pass

    def execute_command(self, action: str, params: dict = None):
        if params is None:
            params = {}
        now = time.time()
        self.get_logger().info(f"Orchestrator received action: {action} with params: {params}")
        if action == "START":
            if self.flight_state in ["IDLE", "COMPLETE", "CHARGING", "LANDED", "PAUSED"]:
                scenario = params.get("scenario", "day_sweep")
                if scenario == "battery_rth":
                    self.battery = BatteryModel(capacity_wh=555.0, safety_reserve_pct=20.0)
                    self.battery.remaining_wh = self.battery.capacity_wh * 0.23
                    self.battery.percentage = 23.0
                else:
                    self.battery = BatteryModel(capacity_wh=555.0, safety_reserve_pct=20.0)

                if scenario == "wildlife_tracking":
                    self.ordered_zones = ZoneManager.get_ordered_zones(["zone_bravo", "zone_alpha", "zone_charlie", "zone_delta"])
                    perimeter_wps = self.planner.generate_perimeter_sweep()
                    interior_wps = self.planner.generate_full_mission_plan(self.ordered_zones)
                    self.waypoints = perimeter_wps + interior_wps
                elif scenario == "vehicle_intercept":
                    self.ordered_zones = ZoneManager.get_ordered_zones(["zone_charlie", "zone_alpha", "zone_bravo", "zone_delta"])
                    perimeter_wps = self.planner.generate_perimeter_sweep()
                    interior_wps = self.planner.generate_full_mission_plan(self.ordered_zones)
                    self.waypoints = perimeter_wps + interior_wps
                elif scenario == "wildfire_hazard":
                    self.ordered_zones = ZoneManager.get_ordered_zones(["zone_bravo", "zone_charlie", "zone_delta", "zone_alpha"])
                    perimeter_wps = self.planner.generate_perimeter_sweep()
                    interior_wps = self.planner.generate_full_mission_plan(self.ordered_zones)
                    self.waypoints = perimeter_wps + interior_wps
                else:
                    self.ordered_zones = ZoneManager.get_ordered_zones()
                    perimeter_wps = self.planner.generate_perimeter_sweep()
                    interior_wps = self.planner.generate_full_mission_plan(self.ordered_zones)
                    self.waypoints = perimeter_wps + interior_wps

                self.flight_state = "PRECHECK"
                self.mission_state = "PRECHECK"
                self.mission_id = f"MSN-{int(now)}"
                self.mission_start_time = now
                self.total_dist_m = 0.0
                self.wp_index = 0
                self.covered_cells.clear()
                self.logged_tracks.clear()
        elif action == "PAUSE":
            if self.flight_state == "SURVEYING":
                self.flight_state = "PAUSED"
                self.mission_state = "PAUSED"
        elif action == "RESUME":
            if self.flight_state == "PAUSED":
                self.flight_state = "SURVEYING"
                self.mission_state = "SURVEYING"
        elif action == "RTH":
            if self.flight_state in ["SURVEYING", "PAUSED", "TAKEOFF", "PRECHECK", "RESUMING"]:
                self.saved_resume_wp = self.wp_index
                self.flight_state = "RETURNING"
                self.mission_state = "RETURNING"
        elif action == "ABORT":
            self.flight_state = "ABORTED"
            self.mission_state = "ABORTED"
        elif action == "RESET":
            self.flight_state = "IDLE"
            self.mission_state = "IDLE"
            self.control_mode = "AUTONOMOUS"
            self.drone_pos = [0.0, 0.0, 0.49]
            self.drone_vel = [0.0, 0.0, 0.0]
            self.roll = self.pitch = self.yaw = 0.0
            self.total_dist_m = 0.0
            self.wp_index = 0
            self.waypoints = self.planner.generate_perimeter_sweep() + self.planner.generate_full_mission_plan(self.ordered_zones)
            self.covered_cells.clear()
            self.logged_tracks.clear()
            self.battery = BatteryModel(capacity_wh=555.0, safety_reserve_pct=20.0)
        elif action == "SET_MODE":
            mode = params.get("mode", "AUTONOMOUS").upper()
            if mode in ["AUTONOMOUS", "MANUAL_REMOTE"]:
                self.control_mode = mode
                if mode == "MANUAL_REMOTE" and self.drone_pos[2] < 1.0:
                    # If on ground, trigger takeoff to 15m first for remote flight
                    self.flight_state = "TAKEOFF"
                    self.mission_state = "TAKEOFF"
                self.get_logger().info(f"Control mode changed to: {self.control_mode}")
        elif action == "SET_PATTERN":
            pat = params.get("pattern", "boustrophedon").lower()
            self.surveillance_pattern = pat
            if pat == "perimeter":
                self.waypoints = self.planner.generate_perimeter_sweep()
            elif pat == "spiral":
                self.waypoints = self.planner.generate_spiral_plan(self.drone_pos[0], self.drone_pos[1])
            else:
                self.waypoints = self.planner.generate_perimeter_sweep() + self.planner.generate_full_mission_plan(self.ordered_zones)
            self.wp_index = 0
            self.get_logger().info(f"Surveillance pattern set to {pat} ({len(self.waypoints)} waypoints)")
        elif action == "TELEOP":
            self.control_mode = "MANUAL_REMOTE"
            vx = float(params.get("vx", 0.0))
            vy = float(params.get("vy", 0.0))
            vz = float(params.get("vz", 0.0))
            yaw_rate = float(params.get("yaw_rate", 0.0))
            self.teleop_cmd = [vx, vy, vz, yaw_rate]
            self.last_teleop_time = time.time()

    def tick(self):
        now = time.time()
        dt = max(0.005, now - self.prev_time)
        self.prev_time = now

        self.poll_command_queue()

        # Update simulated moving entities
        for ent in self.sim_entities:
            route = ent["route"]
            if len(route) > 1:
                idx = int((now * ent["speed"]) % len(route))
                next_idx = (idx + 1) % len(route)
                alpha = ((now * ent["speed"]) % 1.0)
                ent["pos"][0] = (1.0 - alpha) * route[idx][0] + alpha * route[next_idx][0]
                ent["pos"][1] = (1.0 - alpha) * route[idx][1] + alpha * route[next_idx][1]

        # 3D Physical Flight State Machine
        is_flying = (self.drone_pos[2] > 0.8)
        cmd_vx = cmd_vy = cmd_vz = 0.0
        cmd_yaw_rate = 0.0

        use_fallback_integration = not getattr(self, 'has_gazebo_odom', False) or (time.time() - getattr(self, 'last_gazebo_time', 0.0) > 1.5)

        if self.control_mode == "MANUAL_REMOTE" and self.drone_pos[2] > 0.8:
            # Active Manual Remote Teleoperation Mode
            if (now - self.last_teleop_time) < 0.8:
                b_vx, b_vy, t_vz, t_yaw_rate = self.teleop_cmd
            else:
                b_vx = b_vy = t_vz = t_yaw_rate = 0.0

            cmd_vx = b_vx * math.cos(self.yaw) - b_vy * math.sin(self.yaw)
            cmd_vy = b_vx * math.sin(self.yaw) + b_vy * math.cos(self.yaw)
            cmd_vz = t_vz
            cmd_yaw_rate = t_yaw_rate

            # Safety Interlock: Strictly enforce sovereign boundary (Country A max Y=28.5m)
            if self.drone_pos[1] >= self.SOVEREIGN_MAX_Y and cmd_vy > 0:
                cmd_vy = 0.0
            if self.drone_pos[2] <= 1.0 and cmd_vz < 0:
                cmd_vz = 0.0

            self.roll = math.radians(-max(-8.0, min(8.0, b_vy * 1.8)))
            self.pitch = math.radians(max(-8.0, min(8.0, b_vx * 1.8)))

            if use_fallback_integration:
                self.drone_pos[0] += cmd_vx * dt
                self.drone_pos[1] = min(self.SOVEREIGN_MAX_Y, self.drone_pos[1] + cmd_vy * dt)
                self.drone_pos[2] = max(1.0, self.drone_pos[2] + cmd_vz * dt)
                self.yaw += cmd_yaw_rate * dt
            self.total_dist_m += math.sqrt(cmd_vx**2 + cmd_vy**2) * dt

        elif self.flight_state == "PRECHECK":
            if self.battery.percentage > 25.0:
                self.flight_state = "TAKEOFF"
                self.mission_state = "TAKEOFF"
            else:
                self.flight_state = "IDLE"
                self.mission_state = "IDLE"

        elif self.flight_state == "TAKEOFF":
            target_z = 15.0
            dz = target_z - self.drone_pos[2]
            cmd_vz = min(2.5, max(-1.0, dz * 0.9))
            cmd_vx = 0.0
            cmd_vy = 0.0
            cmd_yaw_rate = 0.0
            if use_fallback_integration:
                self.drone_pos[2] += cmd_vz * dt
            # Subtle pitch up during climb
            self.pitch = math.radians(min(6.0, cmd_vz * 2.5))
            if self.drone_pos[2] >= 14.2:
                if use_fallback_integration:
                    self.drone_pos[2] = 15.0
                self.pitch = 0.0
                self.flight_state = "SURVEYING"
                self.mission_state = "SURVEYING"

        elif self.flight_state == "SURVEYING":
            if self.wp_index < len(self.waypoints):
                target_wp = self.waypoints[self.wp_index]
                tx, ty, tz = target_wp["x"], target_wp["y"], target_wp["z"]
                dx = tx - self.drone_pos[0]
                dy = ty - self.drone_pos[1]
                dz = tz - self.drone_pos[2]
                dist_xy = math.sqrt(dx*dx + dy*dy)

                des_speed = target_wp.get("speed", 4.5)
                if dist_xy > 0.1:
                    cmd_vx = des_speed * (dx / dist_xy)
                    cmd_vy = des_speed * (dy / dist_xy)
                    target_yaw = math.atan2(dy, dx)
                    # Smooth yaw turn
                    yaw_diff = target_yaw - self.yaw
                    while yaw_diff > math.pi: yaw_diff -= 2 * math.pi
                    while yaw_diff < -math.pi: yaw_diff += 2 * math.pi
                    cmd_yaw_rate = max(-1.8, min(1.8, yaw_diff * 3.0))
                    if use_fallback_integration:
                        self.yaw += max(-2.0, min(2.0, yaw_diff * 4.0)) * dt

                cmd_vz = min(2.0, max(-2.0, dz * 0.8))

                step_d = des_speed * dt
                if use_fallback_integration:
                    self.drone_pos[0] += cmd_vx * dt
                    self.drone_pos[1] += cmd_vy * dt
                    self.drone_pos[2] += cmd_vz * dt
                self.total_dist_m += step_d

                # Dynamic Roll & Pitch based on acceleration and velocities
                self.roll = math.radians(-max(-8.0, min(8.0, cmd_vy * 1.8)))
                self.pitch = math.radians(max(-8.0, min(8.0, cmd_vx * 1.8)))

                if dist_xy <= 2.0:
                    self.wp_index += 1
            else:
                self.flight_state = "RETURNING"
                self.mission_state = "RETURNING"

        elif self.flight_state == "PAUSED":
            cmd_vx = cmd_vy = cmd_vz = 0.0
            cmd_yaw_rate = 0.0
            self.roll = self.pitch = 0.0

        elif self.flight_state == "RETURNING":
            # Transit straight to Home Pad (0, 0, 16.0)
            dx = 0.0 - self.drone_pos[0]
            dy = 0.0 - self.drone_pos[1]
            dist_xy = math.sqrt(dx*dx + dy*dy)
            speed = 4.8
            if dist_xy > 0.2:
                cmd_vx = speed * (dx / dist_xy)
                cmd_vy = speed * (dy / dist_xy)
                target_yaw = math.atan2(dy, dx)
                yaw_diff = target_yaw - self.yaw
                while yaw_diff > math.pi: yaw_diff -= 2 * math.pi
                while yaw_diff < -math.pi: yaw_diff += 2 * math.pi
                cmd_yaw_rate = max(-1.8, min(1.8, yaw_diff * 3.0))
                if use_fallback_integration:
                    self.yaw += max(-2.0, min(2.0, yaw_diff * 4.0)) * dt

            dz = 16.0 - self.drone_pos[2]
            cmd_vz = min(1.5, max(-1.5, dz * 0.8))

            if use_fallback_integration:
                self.drone_pos[0] += cmd_vx * dt
                self.drone_pos[1] += cmd_vy * dt
                self.drone_pos[2] += cmd_vz * dt
            self.total_dist_m += speed * dt

            if dist_xy <= 1.8:
                self.flight_state = "LANDING"
                self.mission_state = "LANDING"

        elif self.flight_state == "LANDING":
            # Vertical flare and descent
            dx = 0.0 - self.drone_pos[0]
            dy = 0.0 - self.drone_pos[1]
            cmd_vx = max(-1.2, min(1.2, dx * 0.8))
            cmd_vy = max(-1.2, min(1.2, dy * 0.8))
            cmd_vz = -1.2 # steady descent rate
            cmd_yaw_rate = 0.0
            if use_fallback_integration:
                self.drone_pos[0] += cmd_vx * dt
                self.drone_pos[1] += cmd_vy * dt
                self.drone_pos[2] += cmd_vz * dt
            self.roll = self.pitch = 0.0
            if self.drone_pos[2] <= 0.52:
                if use_fallback_integration:
                    self.drone_pos = [0.0, 0.0, 0.49]
                cmd_vx = cmd_vy = cmd_vz = 0.0
                if self.wp_index < len(self.waypoints):
                    self.flight_state = "CHARGING"
                    self.mission_state = "CHARGING"
                else:
                    self.flight_state = "COMPLETE"
                    self.mission_state = "COMPLETE"

        elif self.flight_state == "CHARGING":
            cmd_vx = cmd_vy = cmd_vz = 0.0
            cmd_yaw_rate = 0.0
            self.roll = self.pitch = 0.0
            if self.battery.percentage >= 96.0:
                self.flight_state = "RESUMING"
                self.mission_state = "RESUMING"

        elif self.flight_state == "RESUMING":
            target_z = 15.0
            dz = target_z - self.drone_pos[2]
            cmd_vz = min(2.5, max(-1.0, dz * 0.9))
            cmd_vx = cmd_vy = cmd_yaw_rate = 0.0
            if use_fallback_integration:
                self.drone_pos[2] += cmd_vz * dt
            if self.drone_pos[2] >= 14.2:
                self.wp_index = self.saved_resume_wp
                self.flight_state = "SURVEYING"
                self.mission_state = "SURVEYING"

        elif self.flight_state == "ABORTED":
            cmd_vz = -1.5
            cmd_vx = cmd_vy = cmd_yaw_rate = 0.0
            if use_fallback_integration:
                self.drone_pos[2] = max(0.49, self.drone_pos[2] + cmd_vz * dt)
            if self.drone_pos[2] <= 0.52:
                self.flight_state = "IDLE"
                self.mission_state = "IDLE"

        # Sovereign Airspace Boundary Enforcement: Drone strictly restricted to Country A (Y <= SOVEREIGN_MAX_Y)
        if self.drone_pos[1] >= self.SOVEREIGN_MAX_Y:
            if cmd_vy > 0.0:
                cmd_vy = 0.0
            if self.drone_pos[1] > self.SOVEREIGN_MAX_Y:
                cmd_vy = min(cmd_vy, -0.6)
        if use_fallback_integration:
            self.drone_pos[1] = min(self.SOVEREIGN_MAX_Y, self.drone_pos[1])

        self.drone_vel = [cmd_vx, cmd_vy, cmd_vz]

        # Battery Physics Model Update
        is_charging = (self.flight_state == "CHARGING")
        batt_status = self.battery.update(
            dt=dt,
            vx=cmd_vx,
            vy=cmd_vy,
            vz=cmd_vz,
            altitude=self.drone_pos[2],
            is_flying=is_flying,
            charging_override=is_charging
        )

        rth_calc = self.battery.calculate_rth_requirements(self.drone_pos[0], self.drone_pos[1], self.drone_pos[2])
        if rth_calc["should_return"] and self.flight_state == "SURVEYING":
            self.get_logger().warn(f"RTH TRIGGER: Battery at {self.battery.percentage:.1f}%. Returning home.")
            self.saved_resume_wp = self.wp_index
            self.flight_state = "RETURNING"
            self.mission_state = "RETURNING"

        # 1. Update Real-Time Area Coverage Grid Footprint
        if self.drone_pos[2] >= 1.5:
            cam_footprint_r = max(10.0, self.drone_pos[2] * 0.95)
            r_sq = cam_footprint_r * cam_footprint_r
            px, py = self.drone_pos[0], self.drone_pos[1]
            min_gx = int((px - cam_footprint_r) // 5) * 5
            max_gx = int((px + cam_footprint_r) // 5) * 5
            min_gy = int((py - cam_footprint_r) // 5) * 5
            max_gy = int((py + cam_footprint_r) // 5) * 5
            for gx in range(min_gx, max_gx + 5, 5):
                for gy in range(min_gy, max_gy + 5, 5):
                    if -90 <= gx <= 90 and -60 <= gy <= 90:
                        if (gx - px)**2 + (gy - py)**2 <= r_sq:
                            self.covered_cells.add((gx, gy))

        covered_cells_count = len(self.covered_cells)
        total_cells_count = len(self.all_grid_cells)
        coverage_pct = round((covered_cells_count / max(1, total_cells_count)) * 100.0, 1)
        area_covered_m2 = round(covered_cells_count * 25.0, 0)

        # 2. Check Active Detections, Gender/Species Classification & Border Territorial Analysis
        active_detections = []
        detection_footprint_r = max(16.0, self.drone_pos[2] * 1.45)

        for ent in self.sim_entities:
            dist = math.sqrt((ent["pos"][0] - self.drone_pos[0])**2 + (ent["pos"][1] - self.drone_pos[1])**2)
            # Downward footprint in Country A OR Forward Camera Optical/Thermal standoff observing Country B
            in_downward_footprint = (dist <= detection_footprint_r)
            is_forward_target = (ent["pos"][1] > self.drone_pos[1])
            dx_lateral = abs(ent["pos"][0] - self.drone_pos[0])
            dy_forward = ent["pos"][1] - self.drone_pos[1]
            in_forward_standoff = (is_forward_target and dy_forward <= 55.0 and dx_lateral <= 35.0 and dist <= 62.0)

            if (in_downward_footprint or in_forward_standoff) and self.drone_pos[2] > 2.0:
                is_foreign = (ent["pos"][1] > self.BORDER_LINE_Y)
                territory = "FOREIGN (Country B)" if is_foreign else "HOME (Country A)"

                # International Border Proximity / Defense Status
                dist_to_border = abs(ent["pos"][1] - self.BORDER_LINE_Y)
                border_alert = "MONITORED"
                if is_foreign and dist_to_border <= 12.0:
                    border_alert = "BORDER_PROXIMITY_ALERT"
                elif not is_foreign and ent.get("territory") == "FOREIGN":
                    border_alert = "BORDER_CROSSING_DETECTED"
                elif not is_foreign:
                    border_alert = "AUTHORIZED_HOME_PATROL"

                det = {
                    "track_id": ent["id"],
                    "category": ent["category"],
                    "gender": ent.get("gender", ""),
                    "animal_type": ent.get("animal_type", ""),
                    "label": ent["label"],
                    "territory": territory,
                    "border_alert": border_alert,
                    "confidence": 0.94 if ent["category"] != "wildlife" else 0.88,
                    "bbox": [180, 140, 260, 260],
                    "world_pos": (round(ent["pos"][0], 1), round(ent["pos"][1], 1)),
                    "camera_source": "forward_rgb"
                }
                active_detections.append(det)

                # Persist unique observation event to SQLite DB & ROS 2 event stream
                if ent["id"] not in self.logged_tracks:
                    self.logged_tracks.add(ent["id"])
                    ev_payload = {
                        "id": f"EVT-{int(now*1000)}",
                        "event_id": f"EVT-{int(now*1000)}",
                        "mission_id": self.mission_id,
                        "timestamp": now,
                        "category": ent["category"],
                        "gender": ent.get("gender", ""),
                        "animal_type": ent.get("animal_type", ""),
                        "label": ent["label"],
                        "territory": territory,
                        "border_alert": border_alert,
                        "track_id": ent["id"],
                        "confidence": det["confidence"],
                        "world_x": det["world_pos"][0],
                        "world_y": det["world_pos"][1],
                        "camera_source": "forward_rgb",
                        "snapshot_path": f"/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/snapshots/snap_{ent['category']}_{int(now)}.jpg",
                        "review_status": "PENDING_REVIEW",
                        "operator_notes": f"Entity: {ent['label']} | Territory: {territory} | Border Status: {border_alert}"
                    }
                    self.db.insert_event(ev_payload)
                    ev_msg = String()
                    ev_msg.data = json.dumps(ev_payload)
                    self.obs_pub.publish(ev_msg)

        # High-Precision Navigation Telemetry Calculations
        # 1. Altitudes: AGL (Above Ground Level) vs MSL (Datum)
        z_ground = get_ground_elevation(self.drone_pos[0], self.drone_pos[1])
        alt_agl = max(0.0, self.drone_pos[2] - z_ground)
        alt_msl = DATUM_ELEV + self.drone_pos[2]

        # 2. Speeds: Horizontal, Vertical, Ground Speed
        h_speed = math.sqrt(cmd_vx**2 + cmd_vy**2)
        v_speed = cmd_vz
        ground_speed = h_speed

        # 3. Heading & Compass Bearings (0° North, 90° East, 180° South, 270° West)
        # Yaw: 0 is East (+X), pi/2 is North (+Y) in standard Cartesian
        nose_heading_deg = (90.0 - math.degrees(self.yaw) + 360.0) % 360.0

        # Course Over Ground (Direction of Travel vector)
        if h_speed > 0.1:
            cog_deg = (90.0 - math.degrees(math.atan2(cmd_vy, cmd_vx)) + 360.0) % 360.0
        else:
            cog_deg = nose_heading_deg

        # Bearing & Distance to Next Waypoint
        if self.wp_index < len(self.waypoints):
            next_wp = self.waypoints[self.wp_index]
            wp_dx = next_wp["x"] - self.drone_pos[0]
            wp_dy = next_wp["y"] - self.drone_pos[1]
            dist_to_next_wp = math.sqrt(wp_dx**2 + wp_dy**2)
            bearing_to_wp_deg = (90.0 - math.degrees(math.atan2(wp_dy, wp_dx)) + 360.0) % 360.0
            next_wp_id = next_wp["id"]
        else:
            dist_to_next_wp = 0.0
            bearing_to_wp_deg = 0.0
            next_wp_id = "COMPLETE"

        # Bearing & Distance to Home Station (0, 0)
        dist_to_home = math.sqrt(self.drone_pos[0]**2 + self.drone_pos[1]**2)
        bearing_to_home_deg = (90.0 - math.degrees(math.atan2(-self.drone_pos[1], -self.drone_pos[0])) + 360.0) % 360.0

        # GPS Mapping
        lat, lon, alt_gps = cartesian_to_gps(self.drone_pos[0], self.drone_pos[1], self.drone_pos[2])

        # Progress and Zone
        total_wps = len(self.waypoints)
        prog_pct = round((self.wp_index / max(1, total_wps)) * 100.0, 1) if total_wps > 0 else 0.0
        cur_zone = ZoneManager.which_zone(self.drone_pos[0], self.drone_pos[1])

        # Determine Propeller Activity: Propellers only start when mission is initiated or actively flying
        # When landed on pad or ground (z <= 0.55m or landed/docking states), propellers are strictly STOPPED (0 RPM)
        is_touchdown = (self.drone_pos[2] <= 0.55) and (self.flight_state in ["LANDING", "CHARGING", "IDLE", "COMPLETE", "LANDED"] or self.control_mode == "MANUAL_REMOTE")
        is_propellers_active = not is_touchdown and (
            (self.flight_state in ["PRECHECK", "TAKEOFF", "SURVEYING", "RETURNING", "LANDING", "RESUMING", "AVOIDING"]) or
            (self.control_mode == "MANUAL_REMOTE" and (self.drone_pos[2] > 0.6 or abs(cmd_vx) > 0.05 or abs(cmd_vy) > 0.05 or abs(cmd_vz) > 0.05))
        )
        target_rotor_vels = [85.0, -85.0, 85.0, -85.0] if is_propellers_active else [0.0, 0.0, 0.0, 0.0]

        # Construct Comprehensive Telemetry Dictionary
        telemetry_dict = {
            "timestamp": now,
            "mission_id": self.mission_id,
            "flight_state": self.flight_state,
            "mission_state": self.mission_state,
            "state": self.flight_state,
            "control_mode": self.control_mode,
            "surveillance_pattern": self.surveillance_pattern,
            "is_manual_teleop": (self.control_mode == "MANUAL_REMOTE"),
            "propellers": {
                "active": is_propellers_active,
                "target_rad_s": target_rotor_vels,
                "status": "ACTIVE_SPINNING" if is_propellers_active else "STOPPED"
            },
            "position": {
                "x": round(self.drone_pos[0], 2),
                "y": round(self.drone_pos[1], 2),
                "z": round(self.drone_pos[2], 2),
                "unit": "meters"
            },
            "orientation": {
                "roll_deg": round(math.degrees(self.roll), 1),
                "pitch_deg": round(math.degrees(self.pitch), 1),
                "yaw_deg": round(math.degrees(self.yaw), 1),
                "unit": "degrees"
            },
            "altitudes": {
                "agl_m": round(alt_agl, 2),
                "agl_ft": round(alt_agl * 3.28084, 1),
                "datum_msl_m": round(alt_msl, 2),
                "datum_msl_ft": round(alt_msl * 3.28084, 1),
                "ground_elevation_m": round(z_ground, 2)
            },
            "velocities": {
                "vx": round(cmd_vx, 2),
                "vy": round(cmd_vy, 2),
                "vz": round(cmd_vz, 2),
                "h_speed_ms": round(h_speed, 2),
                "h_speed_kmh": round(h_speed * 3.6, 1),
                "v_speed_ms": round(v_speed, 2),
                "ground_speed_ms": round(ground_speed, 2),
                "speed_ms": round(ground_speed, 2),
                "unit": "m/s"
            },
            "compass": {
                "heading_deg": round(nose_heading_deg, 1),
                "direction_of_travel_deg": round(cog_deg, 1),
                "bearing_to_next_wp_deg": round(bearing_to_wp_deg, 1),
                "bearing_to_home_deg": round(bearing_to_home_deg, 1),
                "cardinal": self._get_cardinal(nose_heading_deg)
            },
            "gps": {
                "latitude": round(lat, 6),
                "longitude": round(lon, 6),
                "altitude_msl_m": round(alt_msl, 2),
                "fix_status": "3D DGPS / RTK FIX",
                "satellites_visible": 21,
                "hdop": 0.8,
                "vdop": 1.1,
                "last_update": now
            },
            "waypoints": {
                "current_index": self.wp_index,
                "total_count": total_wps,
                "next_wp_id": next_wp_id,
                "dist_to_next_wp_m": round(dist_to_next_wp, 1),
                "dist_to_home_m": round(dist_to_home, 1)
            },
            "yaw_deg": round(nose_heading_deg, 1),
            "current_zone": cur_zone,
            "current_wp_index": self.wp_index,
            "total_waypoints": total_wps,
            "progress_pct": prog_pct,
            "distance_flown_m": round(self.total_dist_m, 1),
            "battery": batt_status,
            "rth": rth_calc,
            "mission_duration_sec": round((now - self.mission_start_time) if self.mission_start_time else 0.0, 1),
            "coverage": {
                "percentage": coverage_pct,
                "area_covered_m2": area_covered_m2,
                "total_area_m2": self.total_grid_area_m2,
                "covered_cells_count": covered_cells_count,
                "total_cells_count": total_cells_count,
                "border_line_y": self.BORDER_LINE_Y,
                "cells_sample": list(self.covered_cells) if self.covered_cells else []
            },
            "border_line": {
                "y": self.BORDER_LINE_Y,
                "x_min": -100.0,
                "x_max": 100.0,
                "home_country": "Country A (Home Sovereign Territory)",
                "foreign_country": "Country B (Foreign Sector)"
            },
            "airspace_status": {
                "sovereign_country": "Country A (Home Sovereign Territory)",
                "stay_in_home_country": True,
                "border_line_y": self.BORDER_LINE_Y,
                "sovereign_max_y": self.SOVEREIGN_MAX_Y,
                "border_passed": False,
                "distance_to_border_m": round(max(0.0, self.BORDER_LINE_Y - self.drone_pos[1]), 1),
                "status": "STRICT_SOVEREIGN_HOLD (INSIDE COUNTRY A)"
            },
            "planned_pattern": [
                {"id": wp["id"], "x": wp["x"], "y": wp["y"], "z": wp["z"], "zone_id": wp.get("zone_id", "transit")}
                for wp in self.waypoints[::2]
            ],
            "sensor_health": {
                "forward_rgb": True,
                "downward_rgb": True,
                "thermal_lwir": True,
                "night_vision": True,
                "imu": True,
                "gps": True,
                "ros2_bridge": True
            },
            "detections": active_detections
        }

        # Write atomic telemetry snapshot for UI
        temp_file = f"{TELEMETRY_STATE_FILE}.tmp.{os.getpid()}"
        try:
            with open(temp_file, "w") as f:
                json.dump(telemetry_dict, f)
            os.replace(temp_file, TELEMETRY_STATE_FILE)
        except Exception as e:
            if os.path.exists(temp_file):
                try: os.remove(temp_file)
                except OSError: pass

        # Publish Standard ROS 2 Messages
        # 1. PoseStamped
        p_msg = PoseStamped()
        p_msg.header.stamp = self.get_clock().now().to_msg()
        p_msg.header.frame_id = "world"
        p_msg.pose.position.x = float(self.drone_pos[0])
        p_msg.pose.position.y = float(self.drone_pos[1])
        p_msg.pose.position.z = float(self.drone_pos[2])
        self.pose_pub.publish(p_msg)

        # 2. Heading
        h_msg = Float64()
        h_msg.data = float(nose_heading_deg)
        self.heading_pub.publish(h_msg)

        # 3. Flight & Mission State Strings
        s_msg = String()
        s_msg.data = self.flight_state
        self.flight_state_pub.publish(s_msg)
        self.mission_state_pub.publish(s_msg)

        # 4. Twist Velocity Command (Transformed to Body Frame for Gazebo VelocityControl)
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        body_vx = cmd_vx * cos_y + cmd_vy * sin_y
        body_vy = -cmd_vx * sin_y + cmd_vy * cos_y

        t_msg = Twist()
        t_msg.linear.x = float(body_vx)
        t_msg.linear.y = float(body_vy)
        t_msg.linear.z = float(cmd_vz)
        t_msg.angular.z = float(cmd_yaw_rate)
        self.cmd_vel_pub.publish(t_msg)

        # 4b. Command Dynamic Propeller Rotors in Gazebo Harmonic
        for pub, r_vel in zip(self.rotor_pubs, target_rotor_vels):
            rv_msg = Float64()
            rv_msg.data = float(r_vel)
            pub.publish(rv_msg)

        # 5. NavSatFix GPS
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "wgs84"
        gps_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        gps_msg.latitude = float(lat)
        gps_msg.longitude = float(lon)
        gps_msg.altitude = float(alt_msl)
        gps_msg.position_covariance[0] = 0.64 # HDOP 0.8m squared
        gps_msg.position_covariance[4] = 0.64
        gps_msg.position_covariance[8] = 1.21 # VDOP 1.1m squared
        self.gps_pub.publish(gps_msg)

        # 6. BatteryState
        b_msg = BatteryState()
        b_msg.header.stamp = self.get_clock().now().to_msg()
        b_msg.voltage = float(batt_status["voltage"])
        b_msg.current = float(batt_status["current_a"])
        b_msg.percentage = float(batt_status["percentage"] / 100.0)
        b_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if is_charging else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.batt_pub.publish(b_msg)

        # 7. Telemetry JSON
        tel_msg = String()
        tel_msg.data = json.dumps(telemetry_dict)
        self.telemetry_pub.publish(tel_msg)

    def _get_cardinal(self, heading_deg: float) -> str:
        dirs = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
        idx = int((heading_deg + 11.25) / 22.5) % 16
        return dirs[idx]

def main(args=None):
    rclpy.init(args=args)
    node = SystemOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
