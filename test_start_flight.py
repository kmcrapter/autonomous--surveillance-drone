import os
import sys
import time
import json
import subprocess

WS_ROOT = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws"
venv_python = os.path.join(WS_ROOT, "venv/bin/python3")

print("1. Starting Gazebo Harmonic headless...")
gz_proc = subprocess.Popen([
    "gz", "sim", "-s", "-r",
    os.path.join(WS_ROOT, "src/surveillance_world/worlds/border_surveillance.sdf")
], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

time.sleep(3)

print("2. Starting ros_gz_bridge parameter_bridge...")
bridge_proc = subprocess.Popen([
    "ros2", "run", "ros_gz_bridge", "parameter_bridge",
    "/drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
    "/drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

time.sleep(2)

print("3. Starting SystemOrchestrator...")
env = os.environ.copy()
env["PYTHONPATH"] = f"{WS_ROOT}/src/system_bringup:{WS_ROOT}/src/coverage_planner:{WS_ROOT}/src/battery_manager:{WS_ROOT}/src/event_manager:" + env.get("PYTHONPATH", "")
orch_proc = subprocess.Popen([
    venv_python, "-m", "system_bringup.system_orchestrator"
], env=env)

time.sleep(2)

print("4. Writing START command to command_queue.json...")
cmd_file = os.path.join(WS_ROOT, "data/db/command_queue.json")
with open(cmd_file, "w") as f:
    json.dump({"action": "START", "timestamp": time.time(), "params": {}}, f)

print("5. Monitoring flight state and position for 8 seconds...")
state_file = os.path.join(WS_ROOT, "data/db/telemetry_state.json")
for i in range(16):
    time.sleep(0.5)
    if os.path.exists(state_file):
        try:
            with open(state_file, "r") as f:
                data = json.load(f)
            pos = data.get("position", {})
            vel = data.get("velocities", {})
            print(f"[{i*0.5:.1f}s] State: {data.get('flight_state')} | Pos: x={pos.get('x')}, y={pos.get('y')}, z={pos.get('z')} | Vz={vel.get('vz')}")
        except Exception as e:
            pass

print("6. Cleaning up...")
orch_proc.terminate()
bridge_proc.terminate()
gz_proc.terminate()
time.sleep(1)
os.system("killall -9 gz-sim-server parameter_bridge 2>/dev/null || true")
print("Done.")
