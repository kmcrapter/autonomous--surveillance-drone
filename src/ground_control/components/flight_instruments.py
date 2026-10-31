"""Dedicated Flight Instruments & Navigation Panel: Compass, Horizon, Altimeter, and GPS."""

import streamlit as st
import plotly.graph_objects as go
import numpy as np
import math
from typing import Dict, Any

def render_compass_figure(heading_deg: float,
                          cog_deg: float,
                          bearing_wp_deg: float,
                          bearing_home_deg: float) -> go.Figure:
    """Renders 360-degree digital compass rose with nose heading, course over ground, and waypoints."""
    fig = go.Figure()

    # Outer Compass Rose Ring
    fig.add_trace(go.Scatterpolar(
        r=[1.0] * 361,
        theta=list(range(361)),
        mode='lines',
        line=dict(color='#374151', width=2),
        showlegend=False,
        hoverinfo='none'
    ))

    # Cardinal Directions (N, E, S, W)
    cardinals = [(0, 'N', '#ff1744'), (90, 'E', '#e0f2fe'), (180, 'S', '#e0f2fe'), (270, 'W', '#e0f2fe')]
    for ang, lbl, col in cardinals:
        fig.add_trace(go.Scatterpolar(
            r=[1.08],
            theta=[ang],
            mode='text',
            text=[f"<b>{lbl}</b>"],
            textfont=dict(color=col, size=13),
            showlegend=False,
            hoverinfo='none'
        ))

    # 1. Drone Nose Heading Vector (Red Arrow)
    fig.add_trace(go.Scatterpolar(
        r=[0.0, 0.95],
        theta=[heading_deg, heading_deg],
        mode='lines+markers',
        line=dict(color='#ff1744', width=4),
        marker=dict(symbol='arrow-up', size=12, color='#ff1744'),
        name=f"Nose Heading ({heading_deg:.0f}°)"
    ))

    # 2. Direction of Travel (Course Over Ground / COG) (Cyan Vector)
    fig.add_trace(go.Scatterpolar(
        r=[0.0, 0.85],
        theta=[cog_deg, cog_deg],
        mode='lines+markers',
        line=dict(color='#00e5ff', width=3, dash='dot'),
        marker=dict(symbol='circle', size=8, color='#00e5ff'),
        name=f"Travel COG ({cog_deg:.0f}°)"
    ))

    # 3. Bearing to Next Waypoint (Amber Pointer)
    fig.add_trace(go.Scatterpolar(
        r=[0.0, 0.78],
        theta=[bearing_wp_deg, bearing_wp_deg],
        mode='lines+markers',
        line=dict(color='#ffd600', width=2.5),
        marker=dict(symbol='diamond', size=8, color='#ffd600'),
        name=f"Target WP ({bearing_wp_deg:.0f}°)"
    ))

    # 4. Bearing to Home Station (Green Pointer)
    fig.add_trace(go.Scatterpolar(
        r=[0.0, 0.70],
        theta=[bearing_home_deg, bearing_home_deg],
        mode='lines+markers',
        line=dict(color='#00e676', width=2),
        marker=dict(symbol='square', size=7, color='#00e676'),
        name=f"Home Pad ({bearing_home_deg:.0f}°)"
    ))

    fig.update_layout(
        template='plotly_dark',
        paper_bgcolor='#0b0f14',
        polar=dict(
            radialaxis=dict(visible=False, range=[0, 1.18]),
            angularaxis=dict(
                direction="clockwise",
                rotation=90, # 0 deg is North
                tickvals=[0, 45, 90, 135, 180, 225, 270, 315],
                ticktext=['0° N', '45°', '90° E', '135°', '180° S', '225°', '270° W', '315°'],
                tickfont=dict(size=9, color='#9ca3af'),
                gridcolor='#1e293b'
            ),
            bgcolor='#080b0f'
        ),
        margin=dict(l=20, r=20, t=25, b=25),
        height=260,
        legend=dict(orientation="h", yanchor="bottom", y=-0.35, xanchor="center", x=0.5, font=dict(size=9))
    )
    return fig

def render_artificial_horizon(roll_deg: float, pitch_deg: float) -> go.Figure:
    """Renders primary flight display artificial horizon / attitude director indicator."""
    fig = go.Figure()

    # Sky / Ground division
    pitch_offset = np.clip(pitch_deg * 2.0, -40, 40)
    fig.add_shape(type="rect", x0=-50, y0=pitch_offset, x1=50, y1=60, fillcolor="#1e3a8a", line_width=0)
    fig.add_shape(type="rect", x0=-50, y0=-60, x1=50, y1=pitch_offset, fillcolor="#451a03", line_width=0)

    # Horizon Line
    fig.add_shape(type="line", x0=-50, y0=pitch_offset, x1=50, y1=pitch_offset, line=dict(color="#ffffff", width=2))

    # Pitch Ladder rungs
    for r in [-20, -10, 10, 20]:
        ry = pitch_offset + (r * 2.0)
        if -50 < ry < 50:
            fig.add_shape(type="line", x0=-12, y0=ry, x1=12, y1=ry, line=dict(color="#ffffff", width=1, dash="dot"))

    # Fixed Aircraft Crosshair Symbol
    fig.add_trace(go.Scatter(
        x=[-18, -6, 0, 6, 18],
        y=[0, 0, -3, 0, 0],
        mode='lines',
        line=dict(color='#ffd600', width=3),
        showlegend=False,
        hoverinfo='none'
    ))

    fig.update_layout(
        template='plotly_dark',
        paper_bgcolor='#0b0f14',
        plot_bgcolor='#080b0f',
        xaxis=dict(visible=False, range=[-50, 50]),
        yaxis=dict(visible=False, range=[-50, 50]),
        margin=dict(l=10, r=10, t=10, b=10),
        height=190
    )
    return fig

def render_flight_instruments_panel(telemetry: Dict[str, Any]):
    """Renders comprehensive, live flight telemetry and instrument panel."""
    st.markdown("### 🎛️ Primary Flight Display & Navigation Instruments")

    alts = telemetry.get("altitudes", {"agl_m": 0.0, "agl_ft": 0.0, "datum_msl_m": 1800.0, "datum_msl_ft": 5905.5})
    vels = telemetry.get("velocities", {"h_speed_ms": 0.0, "v_speed_ms": 0.0, "ground_speed_ms": 0.0, "h_speed_kmh": 0.0})
    orient = telemetry.get("orientation", {"roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0})
    compass = telemetry.get("compass", {"heading_deg": 0.0, "direction_of_travel_deg": 0.0, "bearing_to_next_wp_deg": 0.0, "bearing_to_home_deg": 0.0})
    gps = telemetry.get("gps", {"latitude": 44.500000, "longitude": -110.500000, "altitude_msl_m": 1800.0, "fix_status": "3D RTK FIX", "satellites_visible": 21, "hdop": 0.8, "vdop": 1.1})
    pos = telemetry.get("position", {"x": 0.0, "y": 0.0, "z": 0.38})
    wps = telemetry.get("waypoints", {"current_index": 0, "total_count": 0, "dist_to_next_wp_m": 0.0, "dist_to_home_m": 0.0})

    # Row 1: Key Dual-Readout Flight Instruments (Altimeter & Airspeed)
    m1, m2, m3, m4 = st.columns(4)

    with m1:
        st.markdown(f"""
        <div style="background-color: #111827; padding: 12px 14px; border-radius: 8px; border-left: 4px solid #00e5ff;">
            <span style="font-size: 11px; color: #9ca3af; text-transform: uppercase;">ALTITUDE AGL (Above Ground)</span><br>
            <span style="font-size: 22px; font-weight: bold; color: #00e5ff;">{alts['agl_m']:.2f} m</span>
            <span style="font-size: 13px; color: #93c5fd; margin-left: 8px;">({alts['agl_ft']:.1f} ft)</span><br>
            <span style="font-size: 11px; color: #6b7280;">Terrain Surface: {alts.get('ground_elevation_m', 0.0):.1f}m</span>
        </div>
        """, unsafe_allow_html=True)

    with m2:
        st.markdown(f"""
        <div style="background-color: #111827; padding: 12px 14px; border-radius: 8px; border-left: 4px solid #60a5fa;">
            <span style="font-size: 11px; color: #9ca3af; text-transform: uppercase;">ALTITUDE MSL (Simulation Datum)</span><br>
            <span style="font-size: 22px; font-weight: bold; color: #60a5fa;">{alts['datum_msl_m']:.1f} m</span>
            <span style="font-size: 13px; color: #bfdbfe; margin-left: 8px;">({alts['datum_msl_ft']:.0f} ft)</span><br>
            <span style="font-size: 11px; color: #6b7280;">WGS84 Reference Origin: 1800m</span>
        </div>
        """, unsafe_allow_html=True)

    with m3:
        v_col = "#00e676" if abs(vels['v_speed_ms']) < 0.1 else ("#38bdf8" if vels['v_speed_ms'] > 0 else "#f87171")
        st.markdown(f"""
        <div style="background-color: #111827; padding: 12px 14px; border-radius: 8px; border-left: 4px solid {v_col};">
            <span style="font-size: 11px; color: #9ca3af; text-transform: uppercase;">VERTICAL SPEED (V-Speed)</span><br>
            <span style="font-size: 22px; font-weight: bold; color: {v_col};">{vels['v_speed_ms']:+.2f} m/s</span>
            <span style="font-size: 13px; color: #9ca3af; margin-left: 8px;">({vels['v_speed_ms']*196.85:+.0f} fpm)</span><br>
            <span style="font-size: 11px; color: #6b7280;">Z-Velocity Rate</span>
        </div>
        """, unsafe_allow_html=True)

    with m4:
        st.markdown(f"""
        <div style="background-color: #111827; padding: 12px 14px; border-radius: 8px; border-left: 4px solid #34d399;">
            <span style="font-size: 11px; color: #9ca3af; text-transform: uppercase;">GROUND SPEED (Horizontal)</span><br>
            <span style="font-size: 22px; font-weight: bold; color: #34d399;">{vels['ground_speed_ms']:.2f} m/s</span>
            <span style="font-size: 13px; color: #a7f3d0; margin-left: 8px;">({vels['h_speed_kmh']:.1f} km/h)</span><br>
            <span style="font-size: 11px; color: #6b7280;">Course Velocity</span>
        </div>
        """, unsafe_allow_html=True)

    st.markdown("<div style='margin-bottom: 14px;'></div>", unsafe_allow_html=True)

    # Row 2: Digital Compass, Artificial Horizon, and GNSS Status
    col_comp, col_att, col_gps = st.columns([1.5, 1.2, 1.5])

    with col_comp:
        st.markdown("##### 🧭 Digital Compass & Course Bearings")
        fig_c = render_compass_figure(
            heading_deg=compass["heading_deg"],
            cog_deg=compass["direction_of_travel_deg"],
            bearing_wp_deg=compass["bearing_to_next_wp_deg"],
            bearing_home_deg=compass["bearing_to_home_deg"]
        )
        st.plotly_chart(fig_c, use_container_width=True, config={"displayModeBar": False})

    with col_att:
        st.markdown("##### ✈️ Attitude Indicator (ADI)")
        fig_h = render_artificial_horizon(orient["roll_deg"], orient["pitch_deg"])
        st.plotly_chart(fig_h, use_container_width=True, config={"displayModeBar": False})
        st.markdown(f"""
        <div style="font-size: 11px; text-align: center; color: #9ca3af;">
            Roll: <b>{orient['roll_deg']:+.1f}°</b> &nbsp;|&nbsp;
            Pitch: <b>{orient['pitch_deg']:+.1f}°</b> &nbsp;|&nbsp;
            Yaw: <b>{orient['yaw_deg']:.1f}°</b>
        </div>
        """, unsafe_allow_html=True)

    with col_gps:
        st.markdown("##### 🛰️ GNSS Positioning & Navigation Status")
        st.markdown(f"""
        <div style="background-color: #0d131a; padding: 14px; border-radius: 8px; border: 1px solid #1f2937;">
            <div style="display: flex; justify-content: space-between; margin-bottom: 6px;">
                <span style="color: #9ca3af; font-size: 12px;">GPS FIX STATUS:</span>
                <span style="color: #00e676; font-weight: bold; font-size: 12px;">🟢 {gps.get('fix_status', '3D FIX')}</span>
            </div>
            <div style="display: flex; justify-content: space-between; margin-bottom: 4px;">
                <span style="color: #9ca3af; font-size: 12px;">LATITUDE:</span>
                <span style="color: #e0f2fe; font-family: monospace; font-size: 13px;">{gps['latitude']:.6f}° N</span>
            </div>
            <div style="display: flex; justify-content: space-between; margin-bottom: 4px;">
                <span style="color: #9ca3af; font-size: 12px;">LONGITUDE:</span>
                <span style="color: #e0f2fe; font-family: monospace; font-size: 13px;">{gps['longitude']:.6f}° W</span>
            </div>
            <div style="display: flex; justify-content: space-between; margin-bottom: 4px;">
                <span style="color: #9ca3af; font-size: 12px;">LOCAL CARTESIAN (X,Y,Z):</span>
                <span style="color: #ffd600; font-family: monospace; font-size: 12px;">({pos['x']:.1f}m, {pos['y']:.1f}m, {pos['z']:.1f}m)</span>
            </div>
            <div style="display: flex; justify-content: space-between; margin-bottom: 4px;">
                <span style="color: #9ca3af; font-size: 12px;">SATELLITES / ACCURACY:</span>
                <span style="color: #93c5fd; font-size: 12px;">{gps.get('satellites_visible', 21)} SVs (HDOP: {gps.get('hdop', 0.8)}m)</span>
            </div>
            <div style="display: flex; justify-content: space-between; margin-top: 8px; border-top: 1px solid #1f2937; padding-top: 6px;">
                <span style="color: #9ca3af; font-size: 12px;">DISTANCE TO HOME:</span>
                <span style="color: #00e676; font-weight: bold; font-size: 12px;">{wps.get('dist_to_home_m', 0.0):.1f} m</span>
            </div>
            <div style="display: flex; justify-content: space-between; margin-top: 2px;">
                <span style="color: #9ca3af; font-size: 12px;">DISTANCE TO NEXT WP:</span>
                <span style="color: #ffd600; font-weight: bold; font-size: 12px;">{wps.get('dist_to_next_wp_m', 0.0):.1f} m</span>
            </div>
        </div>
        """, unsafe_allow_html=True)
