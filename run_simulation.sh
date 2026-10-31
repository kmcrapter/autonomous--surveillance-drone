#!/bin/bash
# ==============================================================================
# Autonomous 3D Surveillance Drone Digital Twin - Master Launch Script
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

echo "================================================================================"
echo "  AUTONOMOUS 3D SURVEILLANCE DRONE DIGITAL TWIN"
echo "  Workspace: $WS_DIR"
echo "================================================================================"

# 1. Clean up any lingering background Gazebo / ROS processes
killall -9 gz-sim-server parameter_bridge ruby gz 2>/dev/null || true

# 2. Source ROS 2 Jazzy & Workspace
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[OK] Sourced ROS 2 Jazzy environment."
else
    echo "[WARN] /opt/ros/jazzy/setup.bash not found."
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo "[OK] Sourced workspace install space."
fi

# 3. Activate Python Virtual Environment
if [ -f "$WS_DIR/venv/bin/activate" ]; then
    source "$WS_DIR/venv/bin/activate"
    echo "[OK] Activated Python Virtual Environment."
fi

# 4. Environment Variables for Gazebo & Python
export PYTHONPATH="$WS_DIR/src:$WS_DIR/src/ground_control:$WS_DIR/src/event_manager:$WS_DIR/src/coverage_planner:$WS_DIR/src/mission_manager:$WS_DIR/src/perception:$WS_DIR/src/battery_manager:$WS_DIR/src/mission_reports:$WS_DIR/src/system_bringup:$WS_DIR/src/ground_control/components:$PYTHONPATH"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/drone_description/models:$WS_DIR/src/surveillance_world/models:$GZ_SIM_RESOURCE_PATH"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export PYTHONUNBUFFERED=1

MODE="${1:---full}"

cleanup() {
    echo ""
    echo "[INFO] Shutting down simulation background services..."
    kill $(jobs -p) 2>/dev/null || true
    killall -9 gz-sim-server gz-sim-gui parameter_bridge orchestrator perception_node uvicorn 2>/dev/null || true
    echo "[OK] Cleaned up."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# Kill any lingering stale processes from prior runs
killall -9 gz-sim-server gz-sim-gui parameter_bridge orchestrator perception_node uvicorn 2>/dev/null || true
rm -f "$WS_DIR/data/db/command_queue.json"

case "$MODE" in
    --dashboard-only)
        echo "[INFO] Launching Modern Ground Control Station Dashboard on http://localhost:8080 ..."
        python3 -m uvicorn ground_control.server:app --host 0.0.0.0 --port 8080
        ;;

    --streamlit)
        echo "[INFO] Launching Legacy Streamlit Dashboard on http://localhost:8501 ..."
        streamlit run src/ground_control/app.py --server.port 8501 --server.headless true
        ;;

    --test)
        echo "[INFO] Executing comprehensive end-to-end test suite..."
        ./test_all.sh
        ;;

    --verify-s19)
        echo "[INFO] Executing Section 19 12-Step E2E Verification Suite..."
        ./venv/bin/python3 verify_section_19_e2e.py
        ;;

    --headless)
        echo "[INFO] Starting Headless Gazebo Harmonic Simulation..."
        gz sim -s -r "$WS_DIR/src/surveillance_world/worlds/border_surveillance.sdf" &
        GZ_PID=$!
        sleep 3

        echo "[INFO] Starting Ros-Gz Parameter Bridge..."
        ros2 run ros_gz_bridge parameter_bridge \
            /drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
            /drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
            /drone/camera/forward/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
            /drone/camera/downward/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
            /world/camera/overview/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
            /drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
            /drone/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
            /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
        BRIDGE_PID=$!
        sleep 2

        echo "[INFO] Starting System Orchestrator & Navigation Engine..."
        "$WS_DIR/venv/bin/python3" -m system_bringup.system_orchestrator &
        ORCH_PID=$!
        sleep 1

        echo "[INFO] Starting Perception & Computer Vision Node..."
        "$WS_DIR/venv/bin/python3" -m perception.perception_node &
        PERC_PID=$!
        sleep 1

        echo "[INFO] Launching Modern Ground Control Station on http://localhost:8080 ..."
        "$WS_DIR/venv/bin/python3" -m uvicorn ground_control.server:app --host 0.0.0.0 --port 8080
        ;;

    --full|*)
        echo "[INFO] Launching Graphical Gazebo Harmonic 3D World (Visible GUI Window)..."
        gz sim -r "$WS_DIR/src/surveillance_world/worlds/border_surveillance.sdf" &
        GZ_PID=$!
        sleep 4

        echo "[INFO] Starting Ros-Gz Parameter Bridge..."
        ros2 run ros_gz_bridge parameter_bridge \
            /drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
            /drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
            /drone/camera/forward/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
            /drone/camera/downward/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
            /world/camera/overview/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
            /drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
            /drone/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
            /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
        BRIDGE_PID=$!
        sleep 2

        echo "[INFO] Starting System Orchestrator & Navigation Engine..."
        "$WS_DIR/venv/bin/python3" -m system_bringup.system_orchestrator &
        ORCH_PID=$!
        sleep 1

        echo "[INFO] Starting Perception & Computer Vision Node..."
        "$WS_DIR/venv/bin/python3" -m perception.perception_node &
        PERC_PID=$!
        sleep 1

        echo "[INFO] Launching Modern Ground Control Station on http://localhost:8080 ..."
        "$WS_DIR/venv/bin/python3" -m uvicorn ground_control.server:app --host 0.0.0.0 --port 8080
        ;;
esac
