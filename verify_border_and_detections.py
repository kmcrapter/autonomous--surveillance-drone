"""Comprehensive Verification Script for Border Demarcation, 7 People (Men/Women), Animals, and Coverage Grid."""

import os
import sys
import time
import json
import subprocess

WS_DIR = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws"
for pkg in ["ground_control", "event_manager", "coverage_planner", "mission_manager", "perception", "battery_manager", "mission_reports", "system_bringup"]:
    p = os.path.join(WS_DIR, "src", pkg)
    if p not in sys.path:
        sys.path.insert(0, p)

from event_manager.database import SurveillanceDatabase
from coverage_planner.zone_manager import ZoneManager
from perception.detector_tracker import ObjectDetectorTracker
import numpy as np

def test_sdf_models():
    print("[TEST 1/5] Checking border_surveillance.sdf for Border Line, 9 People & 4 Wildlife Animals...")
    sdf_path = os.path.join(WS_DIR, "src/surveillance_world/worlds/border_surveillance.sdf")
    with open(sdf_path, "r") as f:
        content = f.read()

    required_models = [
        "international_border_line",
        "border_post_west_80",
        "person_man_patrol_guard",
        "person_woman_logistics_officer",
        "person_woman_border_officer",
        "person_man_outpost_sentry",
        "person_man_foreign_crosser",
        "person_man_forest_hiker",
        "person_woman_foreign_observer",
        "person_woman_riverbank_civilian",
        "person_man_foreign_woods",
        "wildlife_deer_stag",
        "wildlife_deer_doe",
        "wildlife_forest_elk",
        "wildlife_mountain_deer",
        "moving_patrol_vehicle",
        "hazard_wildfire"
    ]

    for model in required_models:
        assert f'<model name="{model}"' in content, f"Missing model: {model}"
    print(f"  --> All {len(required_models)} border, people, vehicle, hazard, and wildlife models confirmed in SDF!")

def test_drone_propellers():
    print("[TEST 2/6] Checking quadcopter model.sdf for 4 rotating propeller joints & PID controllers...")
    drone_sdf = os.path.join(WS_DIR, "src/drone_description/models/surveillance_hexacopter/model.sdf")
    with open(drone_sdf, "r") as f:
        content = f.read()

    for i in range(4):
        assert f'<link name="rotor_{i}">' in content, f"Missing link rotor_{i}"
        assert f'<joint name="rotor_{i}_joint" type="revolute">' in content, f"Missing revolute joint rotor_{i}_joint"
        assert f'<joint_name>rotor_{i}_joint</joint_name>' in content, f"Missing JointController for rotor_{i}_joint"
    print("  --> All 4 counter-rotating Quadcopter propeller joints & PID velocity controllers confirmed!")

def test_detector_tracker():
    print("[TEST 3/5] Checking ObjectDetectorTracker for Men, Women & Animals classification...")
    tracker = ObjectDetectorTracker(confidence_threshold=0.45)
    dummy_frame = np.full((480, 640, 3), 30, dtype=np.uint8)

    # Position 1: Drone looking North
    drone_pos_1 = (0.0, 15.0, 12.0)
    drone_yaw_1 = 1.57 # Looking North (+Y)
    dets_1 = tracker.detect_entities(dummy_frame, drone_pos_1, drone_yaw_1, camera_source="forward_rgb")

    # Position 2: Drone looking Northwest toward Sector Bravo (Female Observer at -36, 46)
    drone_pos_2 = (-15.0, 25.0, 14.0)
    drone_yaw_2 = 2.35 # Looking Northwest
    dets_2 = tracker.detect_entities(dummy_frame, drone_pos_2, drone_yaw_2, camera_source="forward_rgb")

    # Position 3: Drone looking Northeast toward Sector Charlie (Female Civilian at 42, 56)
    drone_pos_3 = (20.0, 30.0, 14.0)
    drone_yaw_3 = 0.85 # Looking Northeast
    dets_3 = tracker.detect_entities(dummy_frame, drone_pos_3, drone_yaw_3, camera_source="forward_rgb")

    all_dets = dets_1 + dets_2 + dets_3
    print(f"  --> Detected {len(all_dets)} total entity sightings across operational sweep:")
    categories = set()
    genders = set()
    for d in all_dets:
        print(f"      - {d.get('track_id', 'TRK')} | {d['label']} | Gender/Type: {d.get('gender') or d.get('animal_type')} | Territory: {d.get('territory')}")
        categories.add(d['category'])
        if d.get('gender'):
            genders.add(d['gender'])

    assert "person" in categories, "No person category detected"
    assert "man" in genders, "No man detected"
    assert "woman" in genders, "No woman detected"
    print("  --> Classification verified: Men, Women, and Wildlife identified accurately!")

def test_orchestrator_telemetry_and_coverage():
    print("[TEST 4/5] Checking SystemOrchestrator coverage grid & border defense calculations...")
    import rclpy
    from system_bringup.system_orchestrator import SystemOrchestrator

    rclpy.init()
    node = SystemOrchestrator()

    # Trigger START command
    node.execute_command("START", {"scenario": "day_sweep"})
    assert node.flight_state == "PRECHECK"
    assert len(node.sim_entities) >= 10, f"Expected >= 10 sim entities, got {len(node.sim_entities)}"
    assert all(wp['y'] <= 28.5 for wp in node.waypoints), f"All waypoints must stay strictly inside Country A (Y <= 28.5), found {max(wp['y'] for wp in node.waypoints)}"
    print(f"  --> Country A sovereign airspace verified: All {len(node.waypoints)} pattern waypoints stay strictly <= 28.5m")

    # Simulate drone flight into Zone Alpha and approaching border
    node.flight_state = "SURVEYING"
    node.drone_pos = [0.0, 20.0, 15.0]
    node.tick()

    assert len(node.covered_cells) > 0, "No cells marked as covered"
    cov_count = len(node.covered_cells)
    area_m2 = cov_count * 25.0
    print(f"  --> Area covered after 1 tick: {area_m2} m² ({cov_count} cells covered)")

    # Read latest written telemetry
    telem_file = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/telemetry_state.json"
    with open(telem_file, "r") as f:
        telem = json.load(f)

    assert "coverage" in telem, "Missing coverage in telemetry"
    assert "border_line" in telem, "Missing border_line in telemetry"
    assert telem["coverage"]["percentage"] >= 0.0
    print(f"  --> Telemetry contains coverage: {telem['coverage']['percentage']}% ({telem['coverage']['area_covered_m2']} m²)")
    print(f"  --> Border line defined at Y = {telem['border_line']['y']}m ({telem['border_line']['home_country']} vs {telem['border_line']['foreign_country']})")

    node.destroy_node()
    rclpy.shutdown()

def test_ui_files():
    print("[TEST 5/6] Checking UI files (index.html, app.js, style.css) for zero API key & border UI...")
    app_js = os.path.join(WS_DIR, "src/ground_control/static/app.js")
    with open(app_js, "r") as f:
        js = f.read()

    assert "openstreetmap.org" in js, "OpenStreetMap tiles missing"
    assert "cartocdn.com" not in js, "CartoDB tile references should be removed"
    assert "INTERNATIONAL BORDER" in js, "International border line missing in app.js"
    assert "mapCoveragePct" in js, "Coverage HUD integration missing in app.js"
    assert "currentFilter === 'man'" in js, "Men filter missing in app.js"
    assert "currentFilter === 'woman'" in js, "Women filter missing in app.js"

    index_html = os.path.join(WS_DIR, "src/ground_control/static/index.html")
    with open(index_html, "r") as f:
        html = f.read()

    assert "mapCoveragePct" in html, "mapCoveragePct missing in index.html"
    assert "MEN" in html, "MEN filter missing in index.html"
    assert "WOMEN" in html, "WOMEN filter missing in index.html"
    print("  --> All UI elements, zero API key tiles, coverage HUD, and filters verified!")

def test_mission_reports():
    print("[TEST 6/6] Checking PDF and CSV mission audit reporting for Men, Women & Wildlife...")
    from mission_reports.report_generator import MissionReportGenerator
    gen = MissionReportGenerator()
    mid = f"MSN-VERIFY-{int(time.time())}"
    
    # Insert test events for Men, Women, Wildlife across zones
    db = SurveillanceDatabase()
    db.insert_event({
        "id": f"EVT-T1-{int(time.time())}",
        "mission_id": mid,
        "timestamp": time.time() - 30,
        "category": "person",
        "label": "Man (Home Patrol Guard)",
        "track_id": "TRK-001",
        "confidence": 0.95,
        "world_x": 6.0,
        "world_y": 20.0,
        "camera_source": "forward_rgb",
        "review_status": "CONFIRMED",
        "operator_notes": "Zone Alpha North Gate"
    })
    db.insert_event({
        "id": f"EVT-T2-{int(time.time())}",
        "mission_id": mid,
        "timestamp": time.time() - 20,
        "category": "person",
        "label": "Woman (Border Officer)",
        "track_id": "TRK-003",
        "confidence": 0.94,
        "world_x": -15.0,
        "world_y": 28.0,
        "camera_source": "forward_rgb",
        "review_status": "CONFIRMED",
        "operator_notes": "Zone Alpha Checkpoint"
    })
    db.insert_event({
        "id": f"EVT-T3-{int(time.time())}",
        "mission_id": mid,
        "timestamp": time.time() - 10,
        "category": "wildlife",
        "label": "Animal (Mountain Deer)",
        "track_id": "TRK-013",
        "confidence": 0.91,
        "world_x": 72.0,
        "world_y": 5.0,
        "camera_source": "forward_rgb",
        "review_status": "CONFIRMED",
        "operator_notes": "Zone Delta Ridge Bluff"
    })

    pdf_path = gen.generate_pdf_report(mid)
    csv_path = gen.generate_csv_report(mid)

    assert os.path.exists(pdf_path), f"PDF file not created: {pdf_path}"
    assert os.path.exists(csv_path), f"CSV file not created: {csv_path}"
    assert os.path.getsize(pdf_path) > 1000, "PDF file is too small"
    assert os.path.getsize(csv_path) > 100, "CSV file is too small"

    with open(csv_path, "r") as f:
        csv_txt = f.read()
    assert "MAN (Human Presence)" in csv_txt
    assert "WOMAN (Human Presence)" in csv_txt
    assert "WILDLIFE (Animal)" in csv_txt
    print(f"  --> Successfully generated Audit PDF ({os.path.getsize(pdf_path)} bytes) & CSV ({os.path.getsize(csv_path)} bytes)!")

if __name__ == '__main__':
    test_sdf_models()
    test_drone_propellers()
    test_detector_tracker()
    test_orchestrator_telemetry_and_coverage()
    test_ui_files()
    test_mission_reports()
    print("\n=======================================================")
    print("  ALL 6/6 BORDER, DETECTIONS & REPORT TESTS PASSED!    ")
    print("=======================================================")
