"""Autonomous 3D Drone Surveillance Digital Twin - Ground Control Station Dashboard."""

import streamlit as st
import time
import os
import sys

# Ensure ROS 2 package modules can be imported
ws_root = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws"
for pkg in ["ground_control", "event_manager", "coverage_planner", "mission_manager", "perception", "battery_manager", "mission_reports"]:
    pkg_path = os.path.join(ws_root, "src", pkg)
    if pkg_path not in sys.path:
        sys.path.insert(0, pkg_path)

from ground_control.telemetry_bridge import TelemetryBridge
from components.telemetry_bar import render_telemetry_bar
from components.viewer_3d import create_3d_world_figure
from components.surveillance_map import create_surveillance_map_figure
from components.camera_panel import render_camera_panel
from components.flight_instruments import render_flight_instruments_panel
from components.event_center import render_event_center
from components.mission_controls import render_mission_controls
from components.analytics_panel import render_analytics_panel

from perception.thermal_processor import ThermalProcessor
from perception.night_vision_processor import NightVisionProcessor
from coverage_planner.boustrophedon import BoustrophedonPlanner
from coverage_planner.zone_manager import ZoneManager

# 1. Page Configuration
st.set_page_config(
    page_title="Autonomous 3D Drone Surveillance Twin | GCS",
    page_icon="🛸",
    layout="wide",
    initial_sidebar_state="expanded"
)

# 2. Tactical Dark Theme CSS
st.markdown("""
<style>
    .stApp {
        background-color: #080c10;
        color: #e6edf3;
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    }
    .stTabs [data-baseweb="tab-list"] {
        gap: 6px;
        background-color: #0e141b;
        padding: 6px 10px;
        border-radius: 8px;
    }
    .stTabs [data-baseweb="tab"] {
        font-size: 13px;
        font-weight: 600;
        color: #9ca3af;
        border-radius: 6px;
        padding: 8px 14px;
    }
    .stTabs [aria-selected="true"] {
        background-color: #1f2937 !important;
        color: #00e5ff !important;
        border-bottom: 2px solid #00e5ff !important;
    }
    div[data-testid="stMetricValue"] {
        font-size: 20px;
        font-weight: 700;
        color: #e0f2fe;
    }
</style>
""", unsafe_allow_html=True)

# 3. State Singletons
if "bridge" not in st.session_state:
    st.session_state.bridge = TelemetryBridge()
if "thermal" not in st.session_state:
    st.session_state.thermal = ThermalProcessor(default_palette="inferno")
if "nvg" not in st.session_state:
    st.session_state.nvg = NightVisionProcessor(mode="green_p43", gain=1.7)
if "history" not in st.session_state:
    st.session_state.history = []

bridge = st.session_state.bridge
telemetry = bridge.get_latest_telemetry()
events = bridge.get_events()

# Record trajectory point in session
cur_pos = telemetry.get("position", {"x": 0.0, "y": 0.0, "z": 0.42})
st.session_state.history.append({"x": cur_pos["x"], "y": cur_pos["y"], "z": cur_pos["z"]})
if len(st.session_state.history) > 300:
    st.session_state.history = st.session_state.history[-300:]

# 4. Sidebar Controls
with st.sidebar:
    st.markdown("### 🛸 Digital Twin GCS")
    st.caption("Live Gazebo & ROS 2 Telemetry Bridge")
    st.markdown("---")

    st.markdown("##### 🛰️ System Specifications")
    st.markdown("""
    - **Airframe:** 1.70m Industrial Hexacopter
    - **All-Up Weight:** 12.5 kg
    - **Battery:** 6S 25,000 mAh LiPo (555 Wh)
    - **Cruising Speed:** 4.5 m/s (16.2 km/h)
    - **Cruising Alt:** 15.0 m AGL
    - **Sensors:** Dual RGB, LWIR Thermal, NVG Gen-3, IMU, GNSS
    """)

    st.markdown("---")
    st.markdown("##### 🛡️ Geofence & Safety Limits")
    st.markdown("""
    - **X-Bound (East/West):** ±110.0 m
    - **Y-Bound (North/South):** -75.0 m to +105.0 m
    - **Flight Ceiling:** 45.0 m MSL
    - **RTH Safety Reserve:** 20.0%
    """)

    st.markdown("---")
    auto_refresh = st.checkbox("Auto-Refresh Telemetry", value=True)
    if auto_refresh:
        time.sleep(0.4)
        st.rerun()

# 5. Top Telemetry Status Bar
render_telemetry_bar(telemetry)
st.markdown("<div style='margin-bottom: 12px;'></div>", unsafe_allow_html=True)

# 6. Tabbed Navigation Panels
tab1, tab2, tab3, tab4, tab5, tab6 = st.tabs([
    "🌐 Live 3D World & Flight Instruments",
    "🗺️ Tactical Surveillance Map & Controls",
    "📷 Multi-Sensor Camera Intelligence",
    "🔋 Live Battery & Power Management",
    "🚨 Observation Event Center",
    "📊 Analytics & Mission Reports"
])

# Tab 1: Live 3D World & Flight Instruments
with tab1:
    v_col1, v_col2 = st.columns([1.8, 1.2])
    with v_col1:
        c_mode = st.radio("3D Viewer Camera Mode", ["Drone-Follow", "Orbit / Free View", "Top-Down Tactical"], horizontal=True)
        fig_3d = create_3d_world_figure(telemetry, st.session_state.history, camera_mode=c_mode)
        st.plotly_chart(fig_3d, use_container_width=True, config={"displayModeBar": True})

    with v_col2:
        render_flight_instruments_panel(telemetry)

# Tab 2: Tactical Surveillance Map & Controls
with tab2:
    map_col, ctrl_col = st.columns([1.6, 1.4])
    with map_col:
        st.markdown("##### 🗺️ 2D Tactical Surveillance Map & Coverage Grid")
        planner = BoustrophedonPlanner()
        ordered_zones = ZoneManager.get_ordered_zones()
        planned_wps = planner.generate_full_mission_plan(ordered_zones)
        fig_map = create_surveillance_map_figure(telemetry, events, st.session_state.history, planned_wps)
        st.plotly_chart(fig_map, use_container_width=True, config={"displayModeBar": True})

    with ctrl_col:
        render_mission_controls(telemetry, bridge)

# Tab 3: Multi-Sensor Camera Intelligence
with tab3:
    render_camera_panel(telemetry, st.session_state.thermal, st.session_state.nvg)

# Tab 4: Live Battery & Power Management
with tab4:
    st.markdown("### 🔋 Live Battery & Power Monitoring Interface")
    batt = telemetry.get("battery", {})
    rth = telemetry.get("rth", {})

    b_c1, b_c2, b_c3, b_c4 = st.columns(4)
    b_c1.metric("State of Charge (SOC)", f"{batt.get('percentage', 100.0):.1f}%")
    b_c2.metric("Pack Terminal Voltage", f"{batt.get('voltage', 25.2):.2f} V")
    b_c3.metric("Current Draw", f"{batt.get('current_a', 0.0):.2f} A")
    b_c4.metric("Instantaneous Power", f"{batt.get('power_w', 0.0):.1f} W")

    st.markdown("<div style='margin-bottom: 12px;'></div>", unsafe_allow_html=True)
    bp1, bp2 = st.columns(2)
    with bp1:
        st.markdown("##### ⚡ Energy Consumption & Remaining Endurance")
        st.markdown(f"""
        - **Total Capacity:** `{batt.get('capacity_wh', 555.0):.0f} Wh (6S 25 Ah LiPo)`
        - **Remaining Energy:** `{batt.get('remaining_wh', 555.0):.1f} Wh`
        - **Energy Consumed:** `{batt.get('consumed_wh', 0.0):.1f} Wh`
        - **Estimated Remaining Flight Time:** `~{batt.get('remaining_time_min', 50.0):.1f} minutes`
        - **Inductive Charging Status:** `{'⚡ FAST CHARGING ACTIVE' if batt.get('is_charging') else 'DISCHARGING IN FLIGHT'}`
        """)

    with bp2:
        st.markdown("##### 🏠 Return-to-Home (RTH) Power Reserve Calculation")
        st.markdown(f"""
        - **Distance to Home Pad:** `{rth.get('dist_to_home_m', 0.0):.1f} meters`
        - **Transit Time to Return & Land:** `{rth.get('rth_time_sec', 0.0):.1f} seconds`
        - **Energy Required to Return:** `{rth.get('rth_energy_wh', 0.0):.2f} Wh ({rth.get('rth_energy_pct', 0.0):.1f}%)`
        - **Configured Safety Reserve:** `20.0%`
        - **Autonomous RTH Trigger Threshold:** `{rth.get('rth_trigger_pct', 20.0):.1f}%`
        - **RTH Critical Alert State:** `{'🚨 CRITICAL - RTH ACTIVE' if rth.get('should_return') else '🟢 NOMINAL'}`
        """)

# Tab 5: Event Center & Operator Audit
with tab5:
    render_event_center(events, bridge)

# Tab 6: Analytics & Reports
with tab6:
    render_analytics_panel(telemetry, events, bridge)
