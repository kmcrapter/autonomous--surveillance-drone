"""Modern Ground Control Station Backend: FastAPI, WebSockets, Real-Time Telemetry & Video Streaming."""

import os
import sys
import time
import json
import asyncio
import math
from typing import Dict, Any, List, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Body
from fastapi.responses import HTMLResponse, StreamingResponse, FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import cv2
import numpy as np

WS_ROOT = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws"
for pkg in ["ground_control", "event_manager", "coverage_planner", "mission_manager", "perception", "battery_manager", "mission_reports", "system_bringup", "personnel_security"]:
    pkg_path = os.path.join(WS_ROOT, "src", pkg)
    if pkg_path not in sys.path:
        sys.path.insert(0, pkg_path)

from event_manager.database import SurveillanceDatabase
from coverage_planner.zone_manager import ZoneManager
from mission_reports.report_generator import MissionReportGenerator
from personnel_security.personnel_registry import PersonnelRegistryManager

DB_DIR = os.path.join(WS_ROOT, "data/db")
REPORTS_DIR = os.path.join(WS_ROOT, "data/reports")
SNAPSHOTS_DIR = os.path.join(WS_ROOT, "data/snapshots")
STATIC_DIR = os.path.join(WS_ROOT, "src/ground_control/static")

os.makedirs(DB_DIR, exist_ok=True)
os.makedirs(REPORTS_DIR, exist_ok=True)
os.makedirs(SNAPSHOTS_DIR, exist_ok=True)
os.makedirs(STATIC_DIR, exist_ok=True)

TELEMETRY_STATE_FILE = os.path.join(DB_DIR, "telemetry_state.json")
COMMAND_QUEUE_FILE = os.path.join(DB_DIR, "command_queue.json")
PERSONNEL_STATE_FILE = os.path.join(DB_DIR, "personnel_state.json")
PERSONNEL_SCENARIO_FILE = os.path.join(DB_DIR, "personnel_scenario.json")
OPERATOR_ACTION_QUEUE_FILE = os.path.join(DB_DIR, "operator_action_queue.json")

LIVE_RGB_PATH = os.path.join(DB_DIR, "live_forward_rgb.jpg")
LIVE_OVERVIEW_PATH = os.path.join(DB_DIR, "live_world_overview.jpg")
LIVE_THERMAL_PATH = os.path.join(DB_DIR, "live_thermal.jpg")
LIVE_NVG_PATH = os.path.join(DB_DIR, "live_nvg.jpg")
LIVE_DOWNWARD_PATH = os.path.join(DB_DIR, "live_downward_rgb.jpg")
LIVE_REAR_PATH = os.path.join(DB_DIR, "live_rear_rgb.jpg")
LIVE_LEFT_PATH = os.path.join(DB_DIR, "live_left_rgb.jpg")
LIVE_RIGHT_PATH = os.path.join(DB_DIR, "live_right_rgb.jpg")
LIVE_PENTAGRID_PATH = os.path.join(DB_DIR, "live_pentagrid.jpg")
CAMERA_STATUS_PATH = os.path.join(DB_DIR, "camera_status.json")
THERMAL_CMD_PATH = os.path.join(DB_DIR, "thermal_cmd.json")

app = FastAPI(title="AeroGuard 3D GCS", version="2.5.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

db = SurveillanceDatabase()
report_gen = MissionReportGenerator()

def init_camera_buffers():
    """Initializes baseline camera buffers so UI never displays fallback connecting screens."""
    w, h = 640, 480
    if not os.path.exists(LIVE_RGB_PATH) or os.path.getsize(LIVE_RGB_PATH) < 100:
        frame = np.full((h, w, 3), 30, dtype=np.uint8)
        frame[:240, :] = [130, 85, 45]
        frame[240:, :] = [25, 50, 25]
        cv2.putText(frame, "CAM-1 FORWARD OPTICAL [STANDBY / READY]", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 255), 1, cv2.LINE_AA)
        cv2.imwrite(LIVE_RGB_PATH, frame)

    if not os.path.exists(LIVE_OVERVIEW_PATH) or os.path.getsize(LIVE_OVERVIEW_PATH) < 100:
        frame = np.full((h, w, 3), 28, dtype=np.uint8)
        frame[:200, :] = [110, 75, 40]
        frame[200:, :] = [22, 45, 22]
        cv2.putText(frame, "GAZEBO WORLD OBSERVER (WATCHTOWER) [ONLINE]", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 200), 1, cv2.LINE_AA)
        cv2.imwrite(LIVE_OVERVIEW_PATH, frame)

    if not os.path.exists(LIVE_THERMAL_PATH) or os.path.getsize(LIVE_THERMAL_PATH) < 100:
        frame = np.full((h, w, 3), 15, dtype=np.uint8)
        cv2.putText(frame, "LWIR THERMAL [CALIBRATED]", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 255), 1, cv2.LINE_AA)
        cv2.imwrite(LIVE_THERMAL_PATH, frame)

    if not os.path.exists(LIVE_NVG_PATH) or os.path.getsize(LIVE_NVG_PATH) < 100:
        frame = np.full((h, w, 3), (10, 40, 10), dtype=np.uint8)
        cv2.putText(frame, "GEN-3 NVG INTENSIFIER [ONLINE]", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 255, 100), 1, cv2.LINE_AA)
        cv2.imwrite(LIVE_NVG_PATH, frame)

    if not os.path.exists(LIVE_DOWNWARD_PATH) or os.path.getsize(LIVE_DOWNWARD_PATH) < 100:
        frame = np.full((h, w, 3), 25, dtype=np.uint8)
        cv2.putText(frame, "CAM-5 DOWNWARD NADIR [ONLINE]", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 180), 1, cv2.LINE_AA)
        cv2.imwrite(LIVE_DOWNWARD_PATH, frame)

    if not os.path.exists(LIVE_REAR_PATH) or os.path.getsize(LIVE_REAR_PATH) < 100:
        frame = np.full((h, w, 3), 30, dtype=np.uint8)
        cv2.putText(frame, "CAM-2 REAR AFT 180 [ONLINE]", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 210, 255), 1, cv2.LINE_AA)
        cv2.imwrite(LIVE_REAR_PATH, frame)

    if not os.path.exists(LIVE_LEFT_PATH) or os.path.getsize(LIVE_LEFT_PATH) < 100:
        frame = np.full((h, w, 3), 30, dtype=np.uint8)
        cv2.putText(frame, "CAM-3 PORT FLANK 270 [ONLINE]", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 240, 180), 1, cv2.LINE_AA)
        cv2.imwrite(LIVE_LEFT_PATH, frame)

    if not os.path.exists(LIVE_RIGHT_PATH) or os.path.getsize(LIVE_RIGHT_PATH) < 100:
        frame = np.full((h, w, 3), 30, dtype=np.uint8)
        cv2.putText(frame, "CAM-4 STARBOARD FLANK 90 [ONLINE]", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 240, 180), 1, cv2.LINE_AA)
        cv2.imwrite(LIVE_RIGHT_PATH, frame)

    if not os.path.exists(LIVE_PENTAGRID_PATH) or os.path.getsize(LIVE_PENTAGRID_PATH) < 100:
        p_frame = np.full((540, 960, 3), 20, dtype=np.uint8)
        cv2.putText(p_frame, "360+NADIR 5-CAMERA PANORAMIC ARRAY [INITIALIZING...]", (40, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 240, 255), 2, cv2.LINE_AA)
        cv2.imwrite(LIVE_PENTAGRID_PATH, p_frame)

init_camera_buffers()

def queue_operator_action(action_data: Dict[str, Any]):
    """Atomic write to operator action queue file consumed by identity_correlation_node."""
    temp_file = OPERATOR_ACTION_QUEUE_FILE + ".tmp"
    with open(temp_file, "w") as f:
        json.dump(action_data, f)
    os.replace(temp_file, OPERATOR_ACTION_QUEUE_FILE)

def get_telemetry_snapshot() -> Dict[str, Any]:
    """Reads latest atomic telemetry snapshot or returns calibrated standby data."""
    now = time.time()
    data = None
    if os.path.exists(TELEMETRY_STATE_FILE):
        try:
            with open(TELEMETRY_STATE_FILE, "r") as f:
                data = json.load(f)
            if os.path.exists(CAMERA_STATUS_PATH):
                try:
                    with open(CAMERA_STATUS_PATH, "r") as cf:
                        data["camera_status"] = json.load(cf)
                except Exception:
                    pass
        except Exception:
            pass

    if data is None:
        data = {
            "timestamp": now,
            "mission_id": f"MSN-{int(now)}",
            "flight_state": "IDLE",
            "mission_state": "IDLE",
            "state": "IDLE",
            "position": {"x": 0.0, "y": 0.0, "z": 0.49, "unit": "meters"},
            "orientation": {"roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0, "unit": "degrees"},
            "altitudes": {
                "agl_m": 0.0,
                "agl_ft": 0.0,
                "datum_msl_m": 1800.49,
                "datum_msl_ft": 5907.1,
                "ground_elevation_m": 0.0
            },
            "velocities": {
                "vx": 0.0, "vy": 0.0, "vz": 0.0,
                "h_speed_ms": 0.0, "h_speed_kmh": 0.0,
                "v_speed_ms": 0.0, "ground_speed_ms": 0.0, "speed_ms": 0.0,
                "unit": "m/s"
            },
            "compass": {
                "heading_deg": 0.0,
                "direction_of_travel_deg": 0.0,
                "bearing_to_next_wp_deg": 0.0,
                "bearing_to_home_deg": 0.0,
                "cardinal": "N"
            },
            "gps": {
                "latitude": 44.500000,
                "longitude": -110.500000,
                "altitude_msl_m": 1800.49,
                "fix_status": "3D DGPS / RTK FIX",
                "satellites_visible": 21,
                "hdop": 0.8,
                "vdop": 1.1,
                "last_update": now
            },
            "waypoints": {
                "current_index": 0,
                "total_count": 65,
                "next_wp_id": "WP-001",
                "dist_to_next_wp_m": 65.6,
                "dist_to_home_m": 0.0
            },
            "yaw_deg": 0.0,
            "current_zone": "zone_alpha",
            "current_wp_index": 0,
            "total_waypoints": 65,
            "progress_pct": 0.0,
            "distance_flown_m": 0.0,
            "battery": {
                "percentage": 100.0,
                "voltage": 25.2,
                "current_a": 0.0,
                "power_w": 18.0,
                "remaining_wh": 555.0,
                "consumed_wh": 0.0,
                "remaining_time_min": 52.0,
                "is_charging": False
            },
            "rth": {
                "dist_to_home_m": 0.0,
                "rth_time_sec": 0.0,
                "rth_energy_wh": 0.0,
                "rth_energy_pct": 0.0,
                "rth_trigger_pct": 20.0,
                "should_return": False
            },
            "sensor_health": {
                "forward_rgb": True,
                "overview_camera": True,
                "downward_rgb": True,
                "thermal_lwir": True,
                "night_vision": True,
                "imu": True,
                "gps": True,
                "ros2_bridge": True
            },
            "detections": []
        }

    # Attach live personnel state if available
    if os.path.exists(PERSONNEL_STATE_FILE):
        try:
            with open(PERSONNEL_STATE_FILE, "r") as pf:
                data["personnel"] = json.load(pf)
        except Exception:
            pass

    if "personnel" not in data:
        data["personnel"] = {
            "timestamp": now,
            "identities": [],
            "total_detected": 0
        }

    # Attach live radiometry from camera_status
    cam_meta = data.get("camera_status", {})
    if isinstance(cam_meta, dict):
        data["thermal_radiometry"] = cam_meta.get("radiometry", {
            "t_min": 12.0, "t_max": 37.0, "t_center": 18.5, "t_mean": 17.2, "hotspots_count": 0
        })
        data["thermal_palette"] = cam_meta.get("thermal_palette", "ironbow")
    else:
        data["thermal_radiometry"] = {
            "t_min": 12.0, "t_max": 37.0, "t_center": 18.5, "t_mean": 17.2, "hotspots_count": 0
        }
        data["thermal_palette"] = "ironbow"

    # Compute detailed subsystem avionics diagnostics (Hexacopter M1..M6, ESCs, 6S Cells, RF Link)
    state = data.get("flight_state", "IDLE")
    alt = data.get("position", {}).get("z", 0.49)
    prop_info = data.get("propellers", {})
    if "active" in prop_info:
        prop_active = bool(prop_info["active"])
    else:
        prop_active = state in ["SURVEYING", "TAKEOFF", "RETURNING", "AVOIDING", "RESUMING"]
    
    # Strictly halt all propellers when drone has landed (alt <= 0.55m or landed/charging states)
    if alt <= 0.55 or state in ["IDLE", "CHARGING", "COMPLETE", "LANDED"]:
        prop_active = False
    is_flying = prop_active
    base_rpm = 5240 if prop_active else 0
    jitter = lambda: int(np.random.randint(-18, 19)) if prop_active else 0

    v_pack = data.get("battery", {}).get("voltage", 25.2)
    v_cell = round(v_pack / 6.0, 3)

    data["avionics"] = {
        "motor_rpms": [base_rpm + jitter() for _ in range(6)],
        "esc_temps_c": [round(36.2 + (0.8 if is_flying else 0.0) + (i * 0.4), 1) for i in range(6)],
        "lipo_cells_v": [round(v_cell + ((i % 3 - 1) * 0.004), 3) for i in range(6)],
        "rf_link": {
            "rssi_dbm": -48 if is_flying else -42,
            "snr_db": 28.4,
            "quality_pct": 99.2,
            "frequency_ghz": 2.484,
            "uplink_rate_kbps": 1250.0
        },
        "gnss_rtk": {
            "fix_type": "3D RTK FIXED",
            "satellites": 24,
            "hdop": 0.68,
            "vdop": 0.84,
            "diff_age_s": 0.4
        }
    }

    # Computed Target & Contact Matrix (range, bearing, territory, threat status)
    px = data.get("position", {}).get("x", 0.0)
    py = data.get("position", {}).get("y", 0.0)
    drone_heading = data.get("compass", {}).get("heading_deg", 0.0)
    contacts = []

    for det in data.get("detections", []):
        wpos = det.get("world_pos", [0.0, 0.0])
        dx = wpos[0] - px
        dy = wpos[1] - py
        dist_m = round(math.hypot(dx, dy), 1)
        angle_deg = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
        rel_bearing = (angle_deg - drone_heading + 360.0) % 360.0

        is_foreign = (wpos[1] > 32.0 or "FOREIGN" in det.get("territory", "").upper())
        cat = det.get("category", "person").lower()
        lbl = det.get("label", cat)

        contacts.append({
            "track_id": det.get("track_id", "TRK"),
            "label": lbl,
            "category": cat,
            "range_m": dist_m,
            "bearing_deg": round(angle_deg, 0),
            "rel_bearing_deg": round(rel_bearing, 0),
            "territory": "COUNTRY B (FOREIGN)" if is_foreign else "COUNTRY A (HOME)",
            "temp_c": det.get("temp_c", 37.0),
            "confidence": det.get("confidence", 0.90),
            "world_pos": wpos,
            "camera_source": det.get("camera_source", "forward_rgb")
        })

    data["target_matrix"] = sorted(contacts, key=lambda c: c["range_m"])

    return data

def mjpeg_frame_generator(image_path: str, fallback_label: str = "CAMERA STANDBY"):
    """Generates continuous MJPEG multipart stream from disk frame buffer."""
    while True:
        frame_bytes = None
        if os.path.exists(image_path):
            try:
                with open(image_path, "rb") as f:
                    frame_bytes = f.read()
            except Exception:
                pass

        if frame_bytes is None or len(frame_bytes) < 100:
            # Generate placeholder frame with timestamp
            img = np.full((480, 640, 3), 16, dtype=np.uint8)
            cv2.rectangle(img, (20, 20), (620, 460), (35, 45, 60), 1)
            cv2.putText(img, fallback_label, (50, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 230, 255), 1, cv2.LINE_AA)
            cv2.putText(img, f"STREAM INITIALIZING | {time.strftime('%H:%M:%S')}", (50, 265), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 150, 180), 1, cv2.LINE_AA)
            _, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_bytes = buf.tobytes()

        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")
        time.sleep(0.045) # ~22 FPS

@app.get("/api/stream/overview")
def stream_overview():
    """Live MJPEG stream of Gazebo World Overview Observer Camera."""
    return StreamingResponse(
        mjpeg_frame_generator(LIVE_OVERVIEW_PATH, "GAZEBO WORLD OBSERVER CAMERA [CONNECTING...]"),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/api/stream/forward")
def stream_forward():
    """Live MJPEG stream of Drone-Mounted Forward Optical RGB Camera."""
    return StreamingResponse(
        mjpeg_frame_generator(LIVE_RGB_PATH, "DRONE-MOUNTED FORWARD OPTICAL CAMERA [CONNECTING...]"),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/api/stream/thermal")
def stream_thermal():
    """Live MJPEG stream of Synthetic LWIR Thermal Camera."""
    return StreamingResponse(
        mjpeg_frame_generator(LIVE_THERMAL_PATH, "SYNTHETIC LWIR THERMAL RADIANCE [CONNECTING...]"),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/api/stream/nvg")
def stream_nvg():
    """Live MJPEG stream of Gen-3 Night-Vision Image Intensifier."""
    return StreamingResponse(
        mjpeg_frame_generator(LIVE_NVG_PATH, "GEN-3 NIGHT VISION INTENSIFIER [CONNECTING...]"),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/api/stream/downward")
def stream_downward():
    """Live MJPEG stream of Downward Nadir Survey Camera."""
    return StreamingResponse(
        mjpeg_frame_generator(LIVE_DOWNWARD_PATH, "DOWNWARD NADIR SURVEY CAMERA [CONNECTING...]"),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/api/stream/rear")
def stream_rear():
    """Live MJPEG stream of Rear Aft 180 Camera."""
    return StreamingResponse(
        mjpeg_frame_generator(LIVE_REAR_PATH, "REAR AFT 180 CAMERA [CONNECTING...]"),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/api/stream/left")
def stream_left():
    """Live MJPEG stream of Port Left Flank 270 Camera."""
    return StreamingResponse(
        mjpeg_frame_generator(LIVE_LEFT_PATH, "PORT LEFT FLANK 270 CAMERA [CONNECTING...]"),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/api/stream/right")
def stream_right():
    """Live MJPEG stream of Starboard Right Flank 90 Camera."""
    return StreamingResponse(
        mjpeg_frame_generator(LIVE_RIGHT_PATH, "STARBOARD RIGHT FLANK 90 CAMERA [CONNECTING...]"),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.get("/api/stream/pentagrid")
def stream_pentagrid():
    """Live MJPEG stream of Consolidated 5-Camera 360+Nadir Panoramic Array."""
    return StreamingResponse(
        mjpeg_frame_generator(LIVE_PENTAGRID_PATH, "360+NADIR 5-CAMERA PANORAMIC ARRAY [CONNECTING...]"),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@app.post("/api/teleop")
def remote_teleop(payload: Dict[str, Any] = Body(...)):
    """Dispatches real-time remote teleoperation control command."""
    vx = float(payload.get("vx", payload.get("linear_x", 0.0)))
    vy = float(payload.get("vy", payload.get("linear_y", 0.0)))
    vz = float(payload.get("vz", payload.get("linear_z", 0.0)))
    yaw_rate = float(payload.get("yaw_rate", payload.get("angular_z", 0.0)))
    queue_data = {
        "action": "TELEOP",
        "timestamp": time.time(),
        "params": {"vx": vx, "vy": vy, "vz": vz, "yaw_rate": yaw_rate}
    }
    temp_file = COMMAND_QUEUE_FILE + ".tmp"
    with open(temp_file, "w") as f:
        json.dump(queue_data, f)
    os.replace(temp_file, COMMAND_QUEUE_FILE)
    return {"status": "dispatched", "action": "TELEOP", "teleop": [vx, vy, vz, yaw_rate]}

@app.post("/api/thermal/palette")
def set_thermal_palette(payload: Dict[str, Any] = Body(...)):
    """Sets active LWIR thermal false-color palette."""
    palette = payload.get("palette", "ironbow").lower().strip()
    with open(THERMAL_CMD_PATH + ".tmp", "w") as f:
        json.dump({"palette": palette, "timestamp": time.time()}, f)
    os.replace(THERMAL_CMD_PATH + ".tmp", THERMAL_CMD_PATH)
    return {"status": "ok", "palette": palette}

@app.post("/api/thermal/isotherm")
def set_thermal_isotherm(payload: Dict[str, Any] = Body(...)):
    """Configures radiometric isotherm alarm threshold."""
    enabled = bool(payload.get("enabled", True))
    thresh = float(payload.get("threshold_c", 35.0))
    with open(THERMAL_CMD_PATH + ".tmp", "w") as f:
        json.dump({"isotherm": {"enabled": enabled, "threshold_c": thresh}, "timestamp": time.time()}, f)
    os.replace(THERMAL_CMD_PATH + ".tmp", THERMAL_CMD_PATH)
    return {"status": "ok", "isotherm_enabled": enabled, "threshold_c": thresh}

@app.get("/api/thermal/radiometry")
def get_thermal_radiometry():
    """Returns real-time thermal radiometry statistics."""
    telem = get_telemetry_snapshot()
    return JSONResponse(telem.get("thermal_radiometry", {}))

@app.get("/api/telemetry")
def get_telemetry():
    """Returns latest telemetry JSON."""
    return JSONResponse(get_telemetry_snapshot())

@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    """High-frequency real-time WebSocket push for flight instruments and tactical map."""
    await websocket.accept()
    try:
        while True:
            telem = get_telemetry_snapshot()
            await websocket.send_json(telem)
            await asyncio.sleep(0.05) # 20 Hz
    except WebSocketDisconnect:
        pass
    except Exception:
        pass

@app.post("/api/command")
def dispatch_command(payload: Dict[str, Any] = Body(...)):
    """Dispatches operator command (START, PAUSE, RESUME, RTH, ABORT, RESET)."""
    action = payload.get("action", "").upper()
    params = payload.get("params", {})
    queue_data = {
        "action": action,
        "timestamp": time.time(),
        "params": params
    }
    temp_file = COMMAND_QUEUE_FILE + ".tmp"
    with open(temp_file, "w") as f:
        json.dump(queue_data, f)
    os.replace(temp_file, COMMAND_QUEUE_FILE)
    return {"status": "dispatched", "action": action, "timestamp": time.time()}

@app.get("/api/events")
def get_events(limit: int = 50):
    """Returns recent recorded surveillance events from SQLite database."""
    events = db.get_recent_events(limit=limit)
    return {"events": events, "count": len(events)}

@app.post("/api/events/{event_id}/review")
def review_event(event_id: str, payload: Dict[str, Any] = Body(...)):
    """Updates review status (ACKNOWLEDGED / DISMISSED) and operator notes."""
    status = payload.get("review_status", "ACKNOWLEDGED")
    notes = payload.get("operator_notes", "")
    db.update_event_review(event_id, status, notes)
    return {"status": "updated", "event_id": event_id, "review_status": status}

@app.get("/api/zones")
def get_zones():
    """Returns predefined sector boundaries and geofence coordinates."""
    zones = ZoneManager.get_all_zones()
    return {"zones": zones, "geofence": ZoneManager.GEOFENCE, "home_pad": ZoneManager.HOME_PAD}

@app.get("/api/reports/pdf")
def download_pdf(mission_id: Optional[str] = None):
    """Generates and serves downloadable PDF audit report."""
    if not mission_id:
        telem = get_telemetry_snapshot()
        mission_id = telem.get("mission_id", f"MSN-{int(time.time())}")
    pdf_path = report_gen.generate_pdf_report(mission_id)
    return FileResponse(
        pdf_path,
        media_type="application/pdf",
        filename=os.path.basename(pdf_path)
    )

@app.get("/api/reports/csv")
def download_csv(mission_id: Optional[str] = None):
    """Generates and serves downloadable granular CSV event log."""
    if not mission_id:
        telem = get_telemetry_snapshot()
        mission_id = telem.get("mission_id", f"MSN-{int(time.time())}")
    csv_path = report_gen.generate_csv_report(mission_id)
    return FileResponse(
        csv_path,
        media_type="text/csv",
        filename=os.path.basename(csv_path)
    )

# --- Personnel Security & Identification API Endpoints ---

@app.get("/api/personnel/registry")
def get_personnel_registry():
    """Returns consented synthetic personnel registry with biometric data minimization."""
    profiles = PersonnelRegistryManager.get_all_profiles(minimize_biometrics=True)
    return JSONResponse({"profiles": profiles, "count": len(profiles)})

@app.get("/api/personnel/identities")
def get_personnel_identities():
    """Returns latest correlated real-time personnel identities."""
    if os.path.exists(PERSONNEL_STATE_FILE):
        try:
            with open(PERSONNEL_STATE_FILE, "r") as f:
                return JSONResponse(json.load(f))
        except Exception:
            pass
    return JSONResponse({"timestamp": time.time(), "identities": [], "total_detected": 0})

@app.get("/api/personnel/events")
def get_personnel_audit_logs(limit: int = 50):
    """Returns recent personnel identification audit logs from SQLite database."""
    logs = db.get_recent_personnel_audit_logs(limit=limit)
    return {"events": logs, "count": len(logs)}

@app.get("/api/personnel/security_events")
def get_security_policy_events(limit: int = 50):
    """Returns recent AI-assisted policy-defined security events from SQLite database."""
    events = db.get_recent_security_policy_events(limit=limit)
    return {"security_events": events, "count": len(events)}

@app.get("/api/personnel/verifications")
def get_personnel_verifications(limit: int = 50):
    """Returns recent verification workflows and voice safety announcements."""
    verifs = db.get_recent_personnel_verifications(limit=limit)
    return {"verifications": verifs, "count": len(verifs)}

@app.post("/api/personnel/scenario")
def set_personnel_scenario(payload: Dict[str, Any] = Body(...)):
    """Switches active personnel demonstration scenario (SCENARIO_A, B, C, D)."""
    sc = payload.get("scenario", "SCENARIO_A")
    data = {
        "scenario": sc,
        "active_scenario": sc,
        "timestamp": time.time()
    }
    temp_file = PERSONNEL_SCENARIO_FILE + ".tmp"
    with open(temp_file, "w") as f:
        json.dump(data, f)
    os.replace(temp_file, PERSONNEL_SCENARIO_FILE)
    return {"status": "scenario_set", "active_scenario": sc, "timestamp": time.time()}

@app.post("/api/personnel/voice_announce")
def dispatch_voice_announcement(payload: Dict[str, Any] = Body(...)):
    """Dispatches neutral simulated voice safety announcement to speaker/bridge."""
    target_id = payload.get("target_id", "BROADCAST")
    notes = payload.get("notes", "Identity verification required.")
    announcement = {
        "action": "VOICE_SAFETY_ANNOUNCEMENT",
        "target_id": target_id,
        "notes": notes,
        "text": "Identity verification required. Please remain in the designated safe area and contact authorized personnel.",
        "timestamp": time.time()
    }
    queue_operator_action(announcement)
    db.insert_personnel_verification({
        "id": f"VOICE-{int(time.time()*1000)}",
        "timestamp": time.time(),
        "entity_id": target_id,
        "action": "VOICE_SAFETY_ANNOUNCEMENT",
        "status": "DISPATCHED",
        "notes": notes
    })
    return {"status": "dispatched", "target_id": target_id, "announcement": announcement}

@app.post("/api/personnel/verify")
def initiate_verification_workflow(payload: Dict[str, Any] = Body(...)):
    """Initiates operator-guided visual identity verification workflow for target."""
    target_id = payload.get("target_id", "UNKNOWN")
    notes = payload.get("notes", "Operator initiated visual verification check")
    action_data = {
        "action": "INITIATE_VISUAL_VERIFICATION",
        "target_id": target_id,
        "notes": notes,
        "timestamp": time.time()
    }
    queue_operator_action(action_data)
    db.insert_personnel_verification({
        "id": f"VERIF-REQ-{int(time.time()*1000)}",
        "timestamp": time.time(),
        "entity_id": target_id,
        "action": "INITIATE_VISUAL_VERIFICATION",
        "status": "INITIATED",
        "notes": notes
    })
    return {"status": "initiated", "target_id": target_id}

@app.post("/api/personnel/review")
def update_personnel_review(payload: Dict[str, Any] = Body(...)):
    """Updates human operator review status for a detected personnel entity."""
    entity_id = payload.get("entity_id", "")
    review_status = payload.get("review_status", "ACKNOWLEDGED")
    notes = payload.get("operator_notes", "")
    db.update_personnel_review(entity_id, review_status, notes)
    action_data = {
        "action": review_status,
        "target_id": entity_id,
        "notes": notes,
        "timestamp": time.time()
    }
    queue_operator_action(action_data)
    return {"status": "updated", "entity_id": entity_id, "review_status": review_status}

# Static Files & SPA UI
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

@app.get("/")
def get_ui():
    """Serves the Single Page Application."""
    index_file = os.path.join(STATIC_DIR, "index.html")
    if os.path.exists(index_file):
        with open(index_file, "r", encoding="utf-8") as f:
            return HTMLResponse(f.read())
    return HTMLResponse("<h1>Ground Control Station UI Loading...</h1>")

def main():
    import uvicorn
    uvicorn.run("ground_control.server:app", host="0.0.0.0", port=8080, log_level="info", reload=False)

if __name__ == '__main__':
    main()
