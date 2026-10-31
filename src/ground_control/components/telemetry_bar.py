"""Top Status and Telemetry HUD Bar with Freshness & Warning Indicators."""

import streamlit as st
import time
from typing import Dict, Any

def render_telemetry_bar(telemetry: Dict[str, Any]):
    """Renders high-visibility dark tactical telemetry metrics with stale-data checks."""
    now = time.time()
    packet_time = telemetry.get("timestamp", now)
    telemetry_age = now - packet_time

    # Stale Data / Loss of Telemetry Detection
    if telemetry_age > 3.0:
        st.warning(f"⚠️ TELEMETRY STALE: Last update received {telemetry_age:.1f}s ago. Simulator or bridge communication degraded.")
    elif telemetry_age > 1.5:
        st.info(f"ℹ️ Telemetry latency elevated: {telemetry_age:.1f}s delay.")

    pos = telemetry.get("position", {"x": 0.0, "y": 0.0, "z": 0.38})
    vels = telemetry.get("velocities", {"ground_speed_ms": 0.0, "v_speed_ms": 0.0})
    batt = telemetry.get("battery", {"percentage": 100.0, "voltage": 25.2, "remaining_time_min": 52.0})
    state = telemetry.get("flight_state", telemetry.get("state", "IDLE"))
    gps = telemetry.get("gps", {"fix_status": "3D RTK FIX", "satellites_visible": 21})

    # State colors
    state_colors = {
        "IDLE": "#78909c",
        "PRECHECK": "#ab47bc",
        "TAKEOFF": "#29b6f6",
        "SURVEYING": "#00e676",
        "PAUSED": "#ffb300",
        "RETURNING": "#ff9100",
        "LANDING": "#00b0ff",
        "CHARGING": "#00e5ff",
        "RESUMING": "#69f0ae",
        "COMPLETE": "#00c853",
        "ABORTED": "#d50000"
    }
    badge_color = state_colors.get(state, "#00e676")

    c1, c2, c3, c4, c5, c6 = st.columns([1.8, 1.2, 1.2, 1.2, 1.5, 1.2])

    with c1:
        st.markdown(f"""
        <div style="background-color: #111827; padding: 10px 14px; border-radius: 8px; border-left: 4px solid {badge_color};">
            <span style="font-size: 11px; color: #9ca3af; text-transform: uppercase;">FLIGHT MODE / STATE</span><br>
            <span style="font-size: 18px; font-weight: bold; color: {badge_color};">{state}</span>
            <span style="font-size: 11px; color: #9ca3af; margin-left: 6px;">{telemetry.get('mission_id', 'MSN-001')}</span>
        </div>
        """, unsafe_allow_html=True)

    with c2:
        b_pct = batt.get("percentage", 100.0)
        b_col = "#00e676" if b_pct > 35 else ("#ffb300" if b_pct > 20 else "#ff1744")
        st.markdown(f"""
        <div style="background-color: #111827; padding: 10px 14px; border-radius: 8px; border-left: 4px solid {b_col};">
            <span style="font-size: 11px; color: #9ca3af; text-transform: uppercase;">BATTERY</span><br>
            <span style="font-size: 18px; font-weight: bold; color: {b_col};">{b_pct:.1f}%</span>
            <span style="font-size: 11px; color: #9ca3af; margin-left: 6px;">{batt.get('remaining_time_min', 0):.0f}m</span>
        </div>
        """, unsafe_allow_html=True)

    with c3:
        alts = telemetry.get("altitudes", {"agl_m": pos["z"]})
        st.markdown(f"""
        <div style="background-color: #111827; padding: 10px 14px; border-radius: 8px; border-left: 4px solid #00e5ff;">
            <span style="font-size: 11px; color: #9ca3af; text-transform: uppercase;">ALTITUDE (AGL)</span><br>
            <span style="font-size: 18px; font-weight: bold; color: #e0f2fe;">{alts.get('agl_m', pos['z']):.1f} m</span>
            <span style="font-size: 11px; color: #9ca3af; margin-left: 4px;">({alts.get('agl_ft', pos['z']*3.28):.0f}ft)</span>
        </div>
        """, unsafe_allow_html=True)

    with c4:
        st.markdown(f"""
        <div style="background-color: #111827; padding: 10px 14px; border-radius: 8px; border-left: 4px solid #34d399;">
            <span style="font-size: 11px; color: #9ca3af; text-transform: uppercase;">GROUND SPEED</span><br>
            <span style="font-size: 18px; font-weight: bold; color: #e0f7fa;">{vels.get('ground_speed_ms', 0.0):.1f} m/s</span>
            <span style="font-size: 11px; color: #9ca3af; margin-left: 4px;">({vels.get('ground_speed_ms', 0.0)*3.6:.0f}km/h)</span>
        </div>
        """, unsafe_allow_html=True)

    with c5:
        st.markdown(f"""
        <div style="background-color: #111827; padding: 10px 14px; border-radius: 8px; border-left: 4px solid #38bdf8;">
            <span style="font-size: 11px; color: #9ca3af; text-transform: uppercase;">GNSS FIX / SATELLITES</span><br>
            <span style="font-size: 15px; font-weight: bold; color: #00e676;">🟢 {gps.get('fix_status', '3D FIX')}</span>
            <span style="font-size: 11px; color: #93c5fd; margin-left: 6px;">({gps.get('satellites_visible', 21)} SVs)</span>
        </div>
        """, unsafe_allow_html=True)

    with c6:
        prog = telemetry.get("progress_pct", 0.0)
        st.markdown(f"""
        <div style="background-color: #111827; padding: 10px 14px; border-radius: 8px; border-left: 4px solid #a855f7;">
            <span style="font-size: 11px; color: #9ca3af; text-transform: uppercase;">COVERAGE</span><br>
            <span style="font-size: 18px; font-weight: bold; color: #ede7f6;">{prog:.1f}%</span>
            <span style="font-size: 11px; color: #d8b4fe; margin-left: 4px;">{telemetry.get('current_wp_index', 0)}/{telemetry.get('total_waypoints', 0)}</span>
        </div>
        """, unsafe_allow_html=True)
