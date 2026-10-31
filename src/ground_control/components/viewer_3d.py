"""Interactive 3D World Viewer with Drone-Follow and Multiple Camera Perspective Controls."""

import plotly.graph_objects as go
import numpy as np
import math
from typing import Dict, Any, List

def create_3d_world_figure(telemetry: Dict[str, Any],
                           history_points: List[Dict[str, float]] = None,
                           camera_mode: str = "Drone-Follow") -> go.Figure:
    """Renders the synchronized 3D Gazebo digital twin world with selectable camera perspectives."""
    fig = go.Figure()

    # 1. 3D Elevation Ground Terrain Matching Gazebo
    grid_size = 40
    gx = np.linspace(-110, 110, grid_size)
    gy = np.linspace(-75, 105, grid_size)
    GX, GY = np.meshgrid(gx, gy)
    GZ = np.zeros_like(GX)

    # North Hill elevation (Sector Bravo/Charlie)
    GZ += 14.0 * np.exp(-(((GX - 60.0)**2) / (2.0 * 28.0**2) + ((GY - 75.0)**2) / (2.0 * 22.0**2)))
    # East Bluff elevation (Sector Delta)
    GZ += 11.0 * np.exp(-(((GX - 80.0)**2) / (2.0 * 24.0**2) + ((GY + 25.0)**2) / (2.0 * 24.0**2)))
    # West Forest Mound
    GZ += 8.0 * np.exp(-(((GX + 75.0)**2) / (2.0 * 22.0**2) + ((GY - 45.0)**2) / (2.0 * 22.0**2)))
    # River Trench Depression
    river_dist = np.abs(GY - 60.0)
    GZ -= 1.8 * np.exp(-(river_dist**2) / (2.0 * 7.0**2))

    fig.add_trace(go.Surface(
        x=GX, y=GY, z=GZ,
        colorscale=[
            [0.0, '#1c341e'], # Riverbank moss
            [0.25, '#284425'], # Forest soil
            [0.6, '#385530'], # Grassland
            [0.85, '#5a6150'], # Rocky slopes
            [1.0, '#747b68'] # Hill summit
        ],
        showscale=False,
        opacity=0.92,
        name='Terrain'
    ))

    # 2. 3D Water Channel Ribbon
    rx = np.linspace(-110, 110, 25)
    ry_n = np.full_like(rx, 68.0)
    ry_s = np.full_like(rx, 52.0)
    rz = np.full_like(rx, 0.1)

    fig.add_trace(go.Mesh3d(
        x=np.concatenate([rx, rx]),
        y=np.concatenate([ry_n, ry_s]),
        z=np.concatenate([rz, rz]),
        i=list(range(len(rx)-1)),
        j=list(range(1, len(rx))),
        k=list(range(len(rx), 2*len(rx)-1)),
        color='#0088cc',
        opacity=0.8,
        name='River Channel'
    ))

    # 3. Camp Facilities & Infrastructure
    structures = [
        {"name": "Operations HQ", "x": -15, "y": -25, "z": 3.5, "color": '#546e7a'},
        {"name": "Barracks", "x": -15, "y": -45, "z": 2.5, "color": '#455a64'},
        {"name": "Drone Hangar", "x": 25, "y": -25, "z": 3.5, "color": '#607d8b'},
        {"name": "Watchtower (12m)", "x": 38, "y": 20, "z": 6.0, "color": '#b0bec5'},
        {"name": "Vehicular Bridge", "x": 0, "y": 60, "z": 0.4, "color": '#78909c'}
    ]
    for s in structures:
        fig.add_trace(go.Scatter3d(
            x=[s["x"]], y=[s["y"]], z=[s["z"]],
            mode='markers+text',
            marker=dict(size=9, symbol='square', color=s["color"], line=dict(color='#ffffff', width=1)),
            text=[s["name"]],
            textposition='top center',
            name=s["name"],
            showlegend=False
        ))

    # 4. Landing Pad & Home Station
    fig.add_trace(go.Scatter3d(
        x=[0.0], y=[0.0], z=[0.4],
        mode='markers+text',
        marker=dict(size=14, symbol='circle', color='#00e676', line=dict(color='#ffffff', width=2)),
        text=['HOME PAD [H]'],
        textposition='bottom center',
        name='Home Landing Pad'
    ))

    # 5. Surveillance Zone Boundary Frames
    zone_boxes = [
        {"name": "Alpha", "poly": [(-45, -55), (45, -55), (45, 28), (-45, 28), (-45, -55)], "color": '#00e676', "alt": 14.0},
        {"name": "Bravo", "poly": [(-85, 25), (-25, 25), (-25, 85), (-85, 85), (-85, 25)], "color": '#00b0ff', "alt": 18.0},
        {"name": "Charlie", "poly": [(-90, 50), (90, 50), (90, 75), (-90, 75), (-90, 50)], "color": '#ffd600', "alt": 16.0},
        {"name": "Delta", "poly": [(35, -35), (85, -35), (85, 30), (35, 30), (35, -35)], "color": '#ff3d00', "alt": 20.0}
    ]
    for zb in zone_boxes:
        px = [p[0] for p in zb["poly"]]
        py = [p[1] for p in zb["poly"]]
        pz = [zb["alt"]] * len(px)
        fig.add_trace(go.Scatter3d(
            x=px, y=py, z=pz,
            mode='lines',
            line=dict(color=zb["color"], width=3, dash='dash'),
            name=f"Zone {zb['name']}"
        ))

    # 6. Actual 3D Flown Flight Trajectory
    if history_points and len(history_points) > 1:
        hx = [p["x"] for p in history_points]
        hy = [p["y"] for p in history_points]
        hz = [p["z"] for p in history_points]
        fig.add_trace(go.Scatter3d(
            x=hx, y=hy, z=hz,
            mode='lines',
            line=dict(color='#00e5ff', width=4),
            name='Flown Path'
        ))

    # 7. Drone Current 3D Position & Nose Heading Vector
    pos = telemetry.get("position", {"x": 0.0, "y": 0.0, "z": 0.42})
    dx, dy, dz = pos["x"], pos["y"], pos["z"]
    orient = telemetry.get("orientation", {"yaw_deg": 0.0})
    yaw_rad = math.radians(orient["yaw_deg"])
    state = telemetry.get("state", "IDLE")

    # Heading pointer
    hx = dx + (5.0 * math.cos(yaw_rad))
    hy = dy + (5.0 * math.sin(yaw_rad))
    hz = dz

    # Drone 3D Marker
    fig.add_trace(go.Scatter3d(
        x=[dx], y=[dy], z=[dz],
        mode='markers+text',
        marker=dict(size=12, symbol='diamond', color='#ff1744', line=dict(color='#ffffff', width=2)),
        text=[f"DRONE [{state}] ({dz:.1f}m)"],
        textposition='top center',
        name='Surveillance Drone'
    ))

    # Nose Heading Line
    fig.add_trace(go.Scatter3d(
        x=[dx, hx], y=[dy, hy], z=[dz, hz],
        mode='lines',
        line=dict(color='#ff1744', width=4),
        name='Heading Vector',
        showlegend=False
    ))

    # Dynamic Camera Eye Configuration
    if camera_mode == "Drone-Follow":
        # Camera sits 12m behind and 6m above the drone looking ahead
        cam_eye_x = (dx - 12.0 * math.cos(yaw_rad)) / 110.0
        cam_eye_y = (dy - 12.0 * math.sin(yaw_rad)) / 90.0
        cam_eye_z = (dz + 6.0) / 35.0
        cam_center_x = dx / 110.0
        cam_center_y = dy / 90.0
        cam_center_z = dz / 35.0
        scene_camera = dict(
            eye=dict(x=cam_eye_x, y=cam_eye_y, z=max(0.4, cam_eye_z)),
            center=dict(x=cam_center_x, y=cam_center_y, z=cam_center_z)
        )
    elif camera_mode == "Top-Down Tactical":
        scene_camera = dict(
            eye=dict(x=0.0, y=0.0, z=2.2),
            center=dict(x=0.0, y=0.0, z=0.0),
            up=dict(x=0.0, y=1.0, z=0.0)
        )
    else: # Orbit / Isometric
        scene_camera = dict(
            eye=dict(x=1.3, y=-1.5, z=1.1),
            center=dict(x=0.0, y=0.0, z=0.0)
        )

    fig.update_layout(
        template='plotly_dark',
        paper_bgcolor='#0b0f14',
        scene=dict(
            xaxis=dict(title='X (m) [East]', range=[-115, 115], backgroundcolor='#080b0f', gridcolor='#1e2630'),
            yaxis=dict(title='Y (m) [North]', range=[-80, 110], backgroundcolor='#080b0f', gridcolor='#1e2630'),
            zaxis=dict(title='Z (m) [Altitude]', range=[0, 45], backgroundcolor='#080b0f', gridcolor='#1e2630'),
            aspectratio=dict(x=1.8, y=1.6, z=0.55),
            camera=scene_camera
        ),
        margin=dict(l=0, r=0, t=10, b=0),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1, font=dict(size=10))
    )

    return fig
