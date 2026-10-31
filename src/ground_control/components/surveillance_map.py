"""2D Tactical Surveillance Map with Coverage Footprints, Waypoints, and Event Pins."""

import plotly.graph_objects as go
import numpy as np
import math
from typing import Dict, Any, List

def create_surveillance_map_figure(telemetry: Dict[str, Any],
                                    events: List[Dict[str, Any]],
                                    history_points: List[Dict[str, float]] = None,
                                    planned_waypoints: List[Dict[str, Any]] = None) -> go.Figure:
    """Renders 2D tactical top-down surveillance map."""
    fig = go.Figure()

    # 1. Zone Polygons (Strictly inside Country A - Sovereign Airspace)
    zones = [
        {
            "name": "Alpha: Perimeter Camp",
            "x": [-45, 45, 45, -45, -45],
            "y": [-55, -55, 25, 25, -55],
            "fill": "rgba(0, 230, 118, 0.12)",
            "line": "#00e676"
        },
        {
            "name": "Bravo: Sovereign Forest Woodline",
            "x": [-85, -25, -25, -85, -85],
            "y": [-20, -20, 28, 28, -20],
            "fill": "rgba(0, 176, 255, 0.12)",
            "line": "#00b0ff"
        },
        {
            "name": "Charlie: Border Standoff Corridor",
            "x": [-85, 85, 85, -85, -85],
            "y": [16, 16, 28, 28, 16],
            "fill": "rgba(255, 214, 0, 0.12)",
            "line": "#ffd600"
        },
        {
            "name": "Delta: Outpost Ridge",
            "x": [35, 85, 85, 35, 35],
            "y": [-35, -35, 28, 28, -35],
            "fill": "rgba(255, 61, 0, 0.12)",
            "line": "#ff3d00"
        }
    ]

    for z in zones:
        fig.add_trace(go.Scatter(
            x=z["x"], y=z["y"],
            fill="toself",
            fillcolor=z["fill"],
            line=dict(color=z["line"], width=2, dash='dash'),
            name=z["name"],
            hoverinfo='name'
        ))

    # Sovereign Airspace Boundary & Border Demarcation Line
    fig.add_trace(go.Scatter(
        x=[-110, 110], y=[28.5, 28.5],
        mode='lines',
        line=dict(color='#00f0ff', width=2, dash='dash'),
        name='Sovereign Airspace Limit (Country A)',
        hoverinfo='name'
    ))
    fig.add_trace(go.Scatter(
        x=[-110, 110], y=[32.0, 32.0],
        mode='lines',
        line=dict(color='#ffd000', width=3),
        name='International Border Line (Y=32m)',
        hoverinfo='name'
    ))

    # 2. Water Feature (River Ribbon)
    fig.add_trace(go.Scatter(
        x=[-110, 110, 110, -110, -110],
        y=[52, 52, 68, 68, 52],
        fill="toself",
        fillcolor="rgba(0, 136, 204, 0.25)",
        line=dict(color="#0088cc", width=1.5),
        name="Waterway (River)",
        hoverinfo='name'
    ))

    # 3. Planned Waypoints Flight Plan
    if planned_waypoints and len(planned_waypoints) > 1:
        wx = [p["x"] for p in planned_waypoints]
        wy = [p["y"] for p in planned_waypoints]
        fig.add_trace(go.Scatter(
            x=wx, y=wy,
            mode='lines+markers',
            line=dict(color='rgba(255, 255, 255, 0.25)', width=1.5, dash='dot'),
            marker=dict(size=4, color='rgba(255, 255, 255, 0.4)'),
            name='Planned Waypoints',
            hoverinfo='text',
            text=[f"WP {p.get('id', i)}" for i, p in enumerate(planned_waypoints)]
        ))

    # 4. Actual Flown Trajectory
    if history_points and len(history_points) > 1:
        hx = [p["x"] for p in history_points]
        hy = [p["y"] for p in history_points]
        fig.add_trace(go.Scatter(
            x=hx, y=hy,
            mode='lines',
            line=dict(color='#00e5ff', width=3),
            name='Flown Path'
        ))

    # 5. Recorded Observations & Event Pins
    if events:
        cat_symbols = {
            "person": "circle",
            "vehicle": "square",
            "wildlife": "diamond",
            "fire": "triangle-up",
            "smoke": "cross"
        }
        cat_colors = {
            "person": "#00e5ff",
            "vehicle": "#ff9100",
            "wildlife": "#76ff03",
            "fire": "#ff1744",
            "smoke": "#e0e0e0"
        }

        for ev in events:
            c = ev.get("category", "hazard")
            fig.add_trace(go.Scatter(
                x=[ev["world_x"]], y=[ev["world_y"]],
                mode='markers+text',
                marker=dict(
                    size=12,
                    symbol=cat_symbols.get(c, "circle"),
                    color=cat_colors.get(c, "#ffffff"),
                    line=dict(color='#000000', width=1.5)
                ),
                text=[ev.get("track_id", "")],
                textposition="top right",
                name=f"{ev['label']} ({ev.get('track_id', '')})",
                hovertext=f"<b>{ev['label']}</b><br>Track: {ev['track_id']}<br>Conf: {ev['confidence']:.0%}<br>Coords: ({ev['world_x']:.1f}, {ev['world_y']:.1f})<br>Status: {ev['review_status']}",
                hoverinfo='text'
            ))

    # 6. Home Station & Landing Pad
    fig.add_trace(go.Scatter(
        x=[0.0], y=[0.0],
        mode='markers+text',
        marker=dict(size=16, symbol='circle-cross', color='#00e676', line=dict(color='#ffffff', width=2)),
        text=['HOME PAD [H]'],
        textposition='bottom center',
        name='Home Pad'
    ))

    # 7. Current Drone Position & Camera Footprint Cone
    pos = telemetry.get("position", {"x": 0.0, "y": 0.0, "z": 0.42})
    yaw = math.radians(telemetry.get("yaw_deg", 0.0))
    dx, dy, dz = pos["x"], pos["y"], pos["z"]

    # Current camera ground footprint rectangle
    cam_w = 2.0 * max(2.0, dz) * math.tan(math.radians(80.0) / 2.0)
    cam_h = 2.0 * max(2.0, dz) * math.tan(math.radians(60.0) / 2.0)
    # Footprint box vertices centered at drone ground position
    fx = [dx - cam_w/2, dx + cam_w/2, dx + cam_w/2, dx - cam_w/2, dx - cam_w/2]
    fy = [dy - cam_h/2, dy - cam_h/2, dy + cam_h/2, dy + cam_h/2, dy - cam_h/2]

    fig.add_trace(go.Scatter(
        x=fx, y=fy,
        fill="toself",
        fillcolor="rgba(255, 23, 68, 0.15)",
        line=dict(color='#ff1744', width=1, dash='dot'),
        name='Camera FOV Footprint',
        hoverinfo='name'
    ))

    # Drone Marker & Heading vector
    arrow_len = 6.0
    hx = dx + (arrow_len * math.cos(yaw))
    hy = dy + (arrow_len * math.sin(yaw))

    fig.add_trace(go.Scatter(
        x=[dx, hx], y=[dy, hy],
        mode='lines',
        line=dict(color='#ff1744', width=3.5),
        showlegend=False
    ))

    fig.add_trace(go.Scatter(
        x=[dx], y=[dy],
        mode='markers+text',
        marker=dict(size=14, symbol='diamond', color='#ff1744', line=dict(color='#ffffff', width=2)),
        text=['DRONE'],
        textposition='top center',
        name='Drone Position'
    ))

    # Layout styling
    fig.update_layout(
        template='plotly_dark',
        paper_bgcolor='#0b0f14',
        plot_bgcolor='#0e141b',
        xaxis=dict(
            title='X (meters) [West <-> East]',
            range=[-115, 115],
            gridcolor='#1e2630',
            zerolinecolor='#2d3846'
        ),
        yaxis=dict(
            title='Y (meters) [South <-> North]',
            range=[-80, 110],
            gridcolor='#1e2630',
            zerolinecolor='#2d3846',
            scaleanchor="x",
            scaleratio=1
        ),
        margin=dict(l=40, r=20, t=20, b=40),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1, font=dict(size=10))
    )

    return fig
