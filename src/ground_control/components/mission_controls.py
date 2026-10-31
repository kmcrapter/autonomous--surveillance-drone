"""Mission Control Console Component with Consequential Command Confirmations."""

import streamlit as st
from typing import Dict, Any

def render_mission_controls(telemetry: Dict[str, Any], bridge=None):
    """Renders functional mission commands, scenario selector, and flight parameters."""
    st.markdown("### 🎮 Mission Control & Scenario Execution")

    curr_state = telemetry.get("state", "IDLE")

    # Command Execution Row
    c1, c2, c3, c4, c5 = st.columns(5)

    with c1:
        start_disabled = (curr_state not in ["IDLE", "COMPLETE"])
        if st.button("🚀 START MISSION", disabled=start_disabled, use_container_width=True, type="primary"):
            if bridge:
                bridge.send_mission_command("START")
                st.toast("Mission Start command dispatched!")
                st.rerun()

    with c2:
        pause_disabled = (curr_state != "SURVEYING")
        if st.button("⏸ PAUSE", disabled=pause_disabled, use_container_width=True):
            if bridge:
                bridge.send_mission_command("PAUSE")
                st.toast("Mission Paused.")
                st.rerun()

    with c3:
        resume_disabled = (curr_state != "PAUSED")
        if st.button("▶ RESUME", disabled=resume_disabled, use_container_width=True):
            if bridge:
                bridge.send_mission_command("RESUME")
                st.toast("Mission Resumed.")
                st.rerun()

    with c4:
        rth_disabled = (curr_state in ["IDLE", "LANDING", "CHARGING", "COMPLETE"])
        if st.button("🏠 RETURN TO HOME", disabled=rth_disabled, use_container_width=True):
            if bridge:
                bridge.send_mission_command("RTH")
                st.toast("Return to Home initiated.")
                st.rerun()

    with c5:
        abort_disabled = (curr_state in ["IDLE", "COMPLETE"])
        if st.button("⛔ EMERGENCY ABORT", disabled=abort_disabled, use_container_width=True):
            if bridge:
                bridge.send_mission_command("ABORT")
                st.warning("Emergency Abort command dispatched!")
                st.rerun()

    st.markdown("---")

    # Scenario Selection & Configuration Sliders
    sc_col, param_col = st.columns([1.8, 2.2])

    with sc_col:
        st.markdown("##### 🎯 Scenario Selector")
        scenarios = [
            "Daytime Full Perimeter Surveillance",
            "Nighttime Low-Light Surveillance",
            "Monitored Perimeter Pedestrian Movement",
            "Sector Bravo Wildlife Tracking",
            "Northern Roadway Vehicle Intercept",
            "Environmental Hazard: Wildfire & Smoke",
            "Low-Battery Auto-RTH & Inductive Charging",
            "Sensor Failure & Degraded Fallback"
        ]
        selected_scenario = st.selectbox("Select Active Simulation Scenario", scenarios, index=0)

        if st.button("Load & Apply Scenario", use_container_width=True):
            if bridge:
                bridge.send_mission_command("SCENARIO", {"scenario_name": selected_scenario})
                st.success(f"Scenario applied: {selected_scenario}")

    with param_col:
        st.markdown("##### ⚙️ Mission Flight Parameters")
        p_c1, p_c2 = st.columns(2)
        with p_c1:
            alt = st.slider("Survey Altitude (m)", min_value=10.0, max_value=35.0, value=16.0, step=1.0)
            speed = st.slider("Cruise Velocity (m/s)", min_value=2.0, max_value=8.0, value=4.5, step=0.5)
        with p_c2:
            reserve = st.slider("Battery Safety Reserve (%)", min_value=15.0, max_value=35.0, value=20.0, step=1.0)
            conf_thresh = st.slider("Detection Confidence Threshold", min_value=0.30, max_value=0.90, value=0.50, step=0.05)

    # Consequential Reset Dialog
    with st.expander("⚠️ Danger Zone: Simulation Reset"):
        st.write("Resetting the simulation stops ongoing flights, clears volatile odometry, and returns the drone to the home charging pad.")
        confirm = st.checkbox("Confirm simulation reset action", key="reset_confirm")
        if st.button("Reset Simulation State", disabled=(not confirm), type="secondary"):
            if bridge:
                bridge.send_mission_command("RESET")
                st.toast("Simulation reset command executed.")
                st.rerun()
