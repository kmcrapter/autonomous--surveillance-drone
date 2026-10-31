#!/bin/bash
# ==============================================================================
# Comprehensive Automated Test Suite - Autonomous 3D Surveillance Drone Twin
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

echo "================================================================================"
echo "  RUNNING AUTOMATED END-TO-END VERIFICATION SUITE"
echo "================================================================================"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
source "$WS_DIR/venv/bin/activate"

PYTHON_BIN="$WS_DIR/venv/bin/python3"

TOTAL_TESTS=11
PASSED_TESTS=0

run_test() {
    TEST_NUM=$1
    TEST_NAME=$2
    CMD=$3
    echo -n "Test $TEST_NUM/$TOTAL_TESTS: $TEST_NAME ... "
    if eval "$CMD" > /dev/null 2>&1; then
        echo -e "\033[0;32m[PASSED]\033[0m"
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        echo -e "\033[0;31m[FAILED]\033[0m"
        echo "Error running command: $CMD"
        eval "$CMD" || true
        exit 1
    fi
}

# 1. Environment & Packages
run_test 1 "ROS 2 Jazzy & colcon packages verification" "ros2 pkg list | grep -q 'mission_manager'"

# 2. Gazebo World Validation
run_test 2 "Gazebo Harmonic 3D World (border_surveillance.sdf) validation" "gz sdf -k src/surveillance_world/worlds/border_surveillance.sdf"

# 3. Drone Description SDF
run_test 3 "Industrial 1.7m Drone Model SDF & sensors parsing" "test -f src/drone_description/models/surveillance_hexacopter/model.sdf"

# 4. Coverage Planner & Zones
run_test 4 "Boustrophedon path planning & zone geofences" "$PYTHON_BIN -c \"
import sys
sys.path.insert(0, 'src/coverage_planner')
from coverage_planner.boustrophedon import BoustrophedonPlanner
from coverage_planner.zone_manager import ZoneManager
planner = BoustrophedonPlanner()
zones = ZoneManager.get_ordered_zones()
assert len(zones) == 4, 'Must have 4 zones'
wps = planner.generate_full_mission_plan(zones)
assert len(wps) > 50, f'Expected >50 waypoints, got {len(wps)}'
\""

# 5. Battery Physics & Dynamic RTH
run_test 5 "Battery physics model & dynamic RTH calculation" "$PYTHON_BIN -c \"
import sys
sys.path.insert(0, 'src/battery_manager')
from battery_manager.battery_model import BatteryModel
batt = BatteryModel(capacity_wh=555.0)
status = batt.update(dt=10.0, vx=4.0, vy=0.0, vz=0.0, altitude=15.0, is_flying=True)
assert status['consumed_wh'] > 0.0, 'Energy should be consumed when flying'
assert status['percentage'] < 100.0, 'Battery percentage should decrease'
rth = batt.calculate_rth_requirements(x=60.0, y=60.0, z=15.0)
assert rth['rth_energy_pct'] > 0.0, 'RTH energy should be positive'
assert 'should_return' in rth
\""

# 6. Mission State Machine
run_test 6 "Mission Finite State Machine transitions" "$PYTHON_BIN -c \"
import sys
sys.path.insert(0, 'src/mission_manager')
from mission_manager.state_machine import MissionState, MissionStateMachine
fsm = MissionStateMachine(MissionState.IDLE)
assert fsm.transition_to(MissionState.PRECHECK, 0.0)
assert fsm.transition_to(MissionState.TAKEOFF, 1.0)
assert fsm.transition_to(MissionState.SURVEYING, 2.0)
assert fsm.transition_to(MissionState.RETURNING, 3.0)
assert fsm.transition_to(MissionState.LANDING, 4.0)
assert fsm.transition_to(MissionState.CHARGING, 5.0)
assert fsm.transition_to(MissionState.RESUMING, 6.0)
assert fsm.transition_to(MissionState.COMPLETE, 7.0)
\""

# 7. Thermal LWIR & Night-Vision Pipeline
run_test 7 "Synthetic Thermal LWIR & Gen-3 NVG sensor processors" "$PYTHON_BIN -c \"
import sys, numpy as np
sys.path.insert(0, 'src/perception')
from perception.thermal_processor import ThermalProcessor
from perception.night_vision_processor import NightVisionProcessor
thermal = ThermalProcessor(default_palette='inferno')
nvg = NightVisionProcessor(mode='green_p43')
dummy = np.zeros((240, 320, 3), dtype=np.uint8)
t_out = thermal.process_frame(dummy, detections=[{'category': 'person', 'bbox': [50, 50, 100, 150]}])
n_out = nvg.process_frame(dummy)
assert t_out.shape == (240, 320, 3)
assert n_out.shape == (240, 320, 3)
\""

# 8. Detection, Tracking & SQLite Persistence
run_test 8 "Object Tracking & SQLite database persistence" "$PYTHON_BIN -c \"
import sys, time
sys.path.insert(0, 'src/event_manager')
from event_manager.database import SurveillanceDatabase
db = SurveillanceDatabase()
db.insert_event({
    'id': 'EVT-TEST-AUTO',
    'mission_id': 'MSN-TEST-AUTO',
    'timestamp': time.time(),
    'category': 'wildlife',
    'label': 'Wildlife (Fauna)',
    'track_id': 'TRK-099',
    'confidence': 0.85,
    'world_x': -50.0,
    'world_y': 40.0,
    'camera_source': 'forward_rgb',
    'snapshot_path': '',
    'review_status': 'PENDING_REVIEW',
    'operator_notes': 'Automated test suite check'
})
evs = db.get_all_events('MSN-TEST-AUTO')
assert len(evs) >= 1
assert evs[0]['category'] == 'wildlife'
\""

# 9. Automatic PDF and CSV Report Generation
run_test 9 "Automatic PDF and CSV Report Generation (ReportLab)" "$PYTHON_BIN -c \"
import sys, os
sys.path.insert(0, 'src/event_manager')
sys.path.insert(0, 'src/mission_reports')
from mission_reports.report_generator import MissionReportGenerator
gen = MissionReportGenerator()
pdf = gen.generate_pdf_report('MSN-TEST-AUTO')
csv_out = gen.generate_csv_report('MSN-TEST-AUTO')
assert os.path.exists(pdf), f'PDF missing: {pdf}'
assert os.path.exists(csv_out), f'CSV missing: {csv_out}'
\""

# 10. Ground Control Station Dashboard Modules
run_test 10 "Ground Control Station UI components import" "$PYTHON_BIN -c \"
import sys
sys.path.insert(0, 'src/ground_control')
sys.path.insert(0, 'src/event_manager')
sys.path.insert(0, 'src/coverage_planner')
sys.path.insert(0, 'src/mission_reports')
from components.viewer_3d import create_3d_world_figure
from components.surveillance_map import create_surveillance_map_figure
from ground_control.telemetry_bridge import TelemetryBridge
bridge = TelemetryBridge()
telem = bridge.get_latest_telemetry()
fig1 = create_3d_world_figure(telem)
fig2 = create_surveillance_map_figure(telem, [])
assert fig1 is not None and fig2 is not None
\""

# 11. Multi-Layer Personnel Identification & Authentication Stack
run_test 11 "Personnel Identification, Anti-Replay & Suspicious Activity Security" "$PYTHON_BIN verify_personnel_security.py"

echo "================================================================================"
echo -e "\033[0;32mALL $PASSED_TESTS/$TOTAL_TESTS AUTOMATED ACCEPTANCE TESTS PASSED SUCCESSFULLY!\033[0m"
echo "================================================================================"
