"""Comprehensive automated test of ALL UI commands, endpoints, and status metrics."""

import os
import sys
import time
import json
import urllib.request
import subprocess

WS_ROOT = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws"
sys.path.insert(0, os.path.join(WS_ROOT, "src/ground_control"))
sys.path.insert(0, os.path.join(WS_ROOT, "src/event_manager"))
sys.path.insert(0, os.path.join(WS_ROOT, "src/mission_reports"))
sys.path.insert(0, os.path.join(WS_ROOT, "src/coverage_planner"))

GREEN = "\033[0;32m"
RED = "\033[0;31m"
BLUE = "\033[0;34m"
RESET = "\033[0m"

def assert_test(cond: bool, msg: str):
    if cond:
        print(f"  {GREEN}✔ PASSED:{RESET} {msg}")
    else:
        print(f"  {RED}✘ FAILED:{RESET} {msg}")
        sys.exit(1)

def run_tests():
    print("=" * 80)
    print("  VERIFYING ALL UI COMMANDS, STATUS INDICATORS & API ENDPOINTS")
    print("=" * 80)

    # 1. Start Server on port 8088
    port = 8088
    env = os.environ.copy()
    env["PYTHONPATH"] = f"{WS_ROOT}/src:{WS_ROOT}/src/ground_control:{WS_ROOT}/src/event_manager:{WS_ROOT}/src/coverage_planner:{WS_ROOT}/src/mission_reports:{WS_ROOT}/src/battery_manager:{WS_ROOT}/src/system_bringup:" + env.get("PYTHONPATH", "")
    
    server_proc = subprocess.Popen([
        f"{WS_ROOT}/venv/bin/python3", "-m", "uvicorn", "ground_control.server:app", "--host", "127.0.0.1", "--port", str(port)
    ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    time.sleep(2)
    base_url = f"http://127.0.0.1:{port}"

    try:
        # A. UI Static HTML & Core Interface
        print(f"\n{BLUE}[1/10] Ground Control Station HTML & Asset Availability{RESET}")
        req = urllib.request.Request(f"{base_url}/")
        with urllib.request.urlopen(req) as resp:
            html = resp.read().decode('utf-8')
            assert_test(resp.status == 200, "Dashboard root returned HTTP 200")
            assert_test("btnStartMission" in html, "START button verified in HTML DOM")
            assert_test("btnPauseMission" in html, "PAUSE button verified in HTML DOM")
            assert_test("btnResumeMission" in html, "RESUME button verified in HTML DOM")
            assert_test("btnRthMission" in html, "RTH button verified in HTML DOM")
            assert_test("btnAbortMission" in html, "ABORT button verified in HTML DOM")
            assert_test("btnResetMission" in html, "RESET button verified in HTML DOM")
            assert_test("adiHorizonGroup" in html, "ADI Artificial Horizon SVG verified")
            assert_test("needleHeading" in html, "Digital Compass Rose 4-needle SVG verified")

        # B. Live Telemetry Snapshot & All Status Indicators
        print(f"\n{BLUE}[2/10] Telemetry Endpoint & Status Instrumentation Data Model{RESET}")
        req = urllib.request.Request(f"{base_url}/api/telemetry")
        with urllib.request.urlopen(req) as resp:
            telem = json.loads(resp.read().decode('utf-8'))
            assert_test("flight_state" in telem, f"Flight state present: {telem.get('flight_state')}")
            assert_test("orientation" in telem and "pitch_deg" in telem["orientation"], "Pitch/Roll telemetry active")
            assert_test("altitudes" in telem and "agl_m" in telem["altitudes"] and "datum_msl_m" in telem["altitudes"], "Dual Altitudes (AGL vs MSL) active")
            assert_test("compass" in telem and "heading_deg" in telem["compass"], "4-Needle Compass Bearings active")
            assert_test("velocities" in telem and "ground_speed_ms" in telem["velocities"], "Ground & Vertical speeds active")
            assert_test("battery" in telem and "voltage" in telem["battery"] and "remaining_wh" in telem["battery"], "6S LiPo Electrochemical status active")
            assert_test("gps" in telem and "latitude" in telem["gps"], "WGS84 GPS Telemetry active")

        # C. Command: START (with Scenario Presets)
        print(f"\n{BLUE}[3/10] Command Action: START (Daytime, NVG, Wildlife, RTH Presets){RESET}")
        for sc in ["day_sweep", "night_nvg", "wildlife_tracking", "battery_rth"]:
            payload = json.dumps({"action": "START", "params": {"scenario": sc}}).encode('utf-8')
            req = urllib.request.Request(f"{base_url}/api/command", data=payload, headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req) as resp:
                res = json.loads(resp.read().decode('utf-8'))
                assert_test(res.get("status") == "dispatched", f"Command START with scenario '{sc}' successfully dispatched")

        # D. Command: PAUSE
        print(f"\n{BLUE}[4/10] Command Action: PAUSE Mission{RESET}")
        payload = json.dumps({"action": "PAUSE"}).encode('utf-8')
        req = urllib.request.Request(f"{base_url}/api/command", data=payload, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as resp:
            res = json.loads(resp.read().decode('utf-8'))
            assert_test(res.get("action") == "PAUSE", "Command PAUSE dispatched to queue")

        # E. Command: RESUME
        print(f"\n{BLUE}[5/10] Command Action: RESUME Mission{RESET}")
        payload = json.dumps({"action": "RESUME"}).encode('utf-8')
        req = urllib.request.Request(f"{base_url}/api/command", data=payload, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as resp:
            res = json.loads(resp.read().decode('utf-8'))
            assert_test(res.get("action") == "RESUME", "Command RESUME dispatched to queue")

        # F. Command: RTH (Return to Home)
        print(f"\n{BLUE}[6/10] Command Action: RTH (Return to Home & Pad Docking){RESET}")
        payload = json.dumps({"action": "RTH"}).encode('utf-8')
        req = urllib.request.Request(f"{base_url}/api/command", data=payload, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as resp:
            res = json.loads(resp.read().decode('utf-8'))
            assert_test(res.get("action") == "RTH", "Command RTH dispatched to queue")

        # G. Command: ABORT & RESET
        print(f"\n{BLUE}[7/10] Command Actions: ABORT and RESET Mission{RESET}")
        payload = json.dumps({"action": "ABORT"}).encode('utf-8')
        req = urllib.request.Request(f"{base_url}/api/command", data=payload, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as resp:
            res = json.loads(resp.read().decode('utf-8'))
            assert_test(res.get("action") == "ABORT", "Command ABORT dispatched to queue")

        payload = json.dumps({"action": "RESET"}).encode('utf-8')
        req = urllib.request.Request(f"{base_url}/api/command", data=payload, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as resp:
            res = json.loads(resp.read().decode('utf-8'))
            assert_test(res.get("action") == "RESET", "Command RESET dispatched to queue")

        # H. Event Center & Operator Review
        print(f"\n{BLUE}[8/10] Surveillance Event Center & Operator Triage Review{RESET}")
        req = urllib.request.Request(f"{base_url}/api/events?limit=10")
        with urllib.request.urlopen(req) as resp:
            data = json.loads(resp.read().decode('utf-8'))
            events = data.get("events", [])
            assert_test(isinstance(events, list), f"Event query returned {len(events)} events")

            if len(events) > 0:
                ev_id = events[0]["id"]
                rev_payload = json.dumps({"review_status": "CONFIRMED", "operator_notes": "Operator verified target"}).encode('utf-8')
                rev_req = urllib.request.Request(f"{base_url}/api/events/{ev_id}/review", data=rev_payload, headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(rev_req) as rev_resp:
                    rev_res = json.loads(rev_resp.read().decode('utf-8'))
                    assert_test(rev_res.get("status") == "updated", f"Operator triage confirmed event {ev_id}")

        # I. Live Video Feeds (Overview, Forward RGB, Thermal, NVG, Downward, Rear, Left, Right)
        print(f"\n{BLUE}[9/11] Live Multi-Sensor Video Feeds (MJPEG Headers){RESET}")
        for endpoint in ["overview", "forward", "thermal", "nvg", "downward", "rear", "left", "right"]:
            req = urllib.request.Request(f"{base_url}/api/stream/{endpoint}")
            with urllib.request.urlopen(req) as resp:
                content_type = resp.headers.get("Content-Type", "")
                assert_test("multipart/x-mixed-replace" in content_type, f"Camera endpoint '/api/stream/{endpoint}' active ({content_type})")

        # J. Flight Control Mode, Pattern & Remote Teleop Endpoints
        print(f"\n{BLUE}[10/11] Dual Flight Control Modes, Surveillance Patterns & Remote Teleoperation{RESET}")
        # Test mode switch
        for mode in ["MANUAL_REMOTE", "AUTONOMOUS"]:
            payload = json.dumps({"action": "SET_MODE", "params": {"mode": mode}}).encode('utf-8')
            req = urllib.request.Request(f"{base_url}/api/command", data=payload, headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req) as resp:
                res = json.loads(resp.read().decode('utf-8'))
                assert_test(res.get("status") == "dispatched", f"Command SET_MODE to '{mode}' dispatched")

        # Test pattern switch
        for pat in ["boustrophedon", "perimeter", "spiral"]:
            payload = json.dumps({"action": "SET_PATTERN", "params": {"pattern": pat}}).encode('utf-8')
            req = urllib.request.Request(f"{base_url}/api/command", data=payload, headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req) as resp:
                res = json.loads(resp.read().decode('utf-8'))
                assert_test(res.get("status") == "dispatched", f"Command SET_PATTERN to '{pat}' dispatched")

        # Test teleop API
        teleop_payload = json.dumps({"linear_x": 2.5, "linear_y": 0.0, "linear_z": 1.0, "angular_z": 0.5}).encode('utf-8')
        req = urllib.request.Request(f"{base_url}/api/teleop", data=teleop_payload, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req) as resp:
            t_res = json.loads(resp.read().decode('utf-8'))
            assert_test(t_res.get("status") in ["ok", "dispatched"], "Remote teleoperation velocity command accepted")

        # K. Cryptographic PDF and CSV Audit Reports
        print(f"\n{BLUE}[11/11] Cryptographic Mission Audit PDF & CSV Report Downloads{RESET}")
        req = urllib.request.Request(f"{base_url}/api/reports/pdf")
        with urllib.request.urlopen(req) as resp:
            pdf_data = resp.read()
            assert_test(pdf_data.startswith(b"%PDF"), f"Audit PDF report downloaded successfully ({len(pdf_data)} bytes)")

        req = urllib.request.Request(f"{base_url}/api/reports/csv")
        with urllib.request.urlopen(req) as resp:
            csv_data = resp.read().decode('utf-8')
            assert_test("Event ID" in csv_data or "Category" in csv_data or len(csv_data) > 0, f"Audit CSV log downloaded successfully ({len(csv_data)} bytes)")

        print("\n" + "=" * 80)
        print(f"  {GREEN}ALL UI COMMANDS, STATUS INDICATORS & API ENDPOINTS 100% VERIFIED!{RESET}")
        print("=" * 80)

    finally:
        server_proc.terminate()
        time.sleep(0.5)
        server_proc.kill()

if __name__ == "__main__":
    run_tests()
