"""Mission Analytics, Plotly Charts, and Automatic Report Export Component."""

import streamlit as st
import plotly.graph_objects as go
import plotly.express as px
import pandas as pd
import time
import os
from typing import Dict, Any, List

from mission_reports.report_generator import MissionReportGenerator

def render_analytics_panel(telemetry: Dict[str, Any], events: List[Dict[str, Any]], bridge=None):
    """Renders interactive analytics charts and report export triggers."""
    st.markdown("### 📊 Mission Analytics & Automatic Audit Reports")

    mission_id = telemetry.get("mission_id", "MSN-DEFAULT")
    dist_m = telemetry.get("distance_flown_m", 0.0)
    dur_sec = telemetry.get("mission_duration_sec", 0.0)
    batt_used = 100.0 - telemetry.get("battery", {}).get("percentage", 100.0)
    prog_pct = telemetry.get("progress_pct", 0.0)

    # Key Metrics KPI Row
    k1, k2, k3, k4, k5 = st.columns(5)
    k1.metric("Total Flight Distance", f"{dist_m:.1f} m")
    k2.metric("Elapsed Mission Time", f"{dur_sec/60:.1f} min")
    k3.metric("Energy Consumed", f"{batt_used:.1f}%")
    k4.metric("Verified Area Coverage", f"{prog_pct:.1f}%")
    k5.metric("Security Events Logged", f"{len(events)}")

    st.markdown("---")

    # Analytics Charts
    c1, c2 = st.columns(2)

    with c1:
        st.markdown("##### 🔋 Battery Discharge & Reserve Model")
        # Generate simulated battery discharge curve
        times = [t for t in range(0, max(10, int(dur_sec)), max(1, int(dur_sec)//15 or 1))]
        if not times:
            times = [0, 60, 120, 180, 240, 300]
        cur_pct = telemetry.get("battery", {}).get("percentage", 100.0)
        pct_curve = [100.0 - (100.0 - cur_pct) * (t / max(1, dur_sec)) for t in times]

        fig_batt = go.Figure()
        fig_batt.add_trace(go.Scatter(
            x=[t/60 for t in times], y=pct_curve,
            mode='lines+markers',
            line=dict(color='#00e676', width=2.5),
            name='Battery %'
        ))
        # Add RTH Reserve Threshold Line
        rth_thresh = telemetry.get("rth", {}).get("rth_trigger_pct", 20.0)
        fig_batt.add_hline(y=rth_thresh, line_dash="dash", line_color="#ff1744",
                          annotation_text=f"RTH Reserve ({rth_thresh:.0f}%)", annotation_position="bottom right")

        fig_batt.update_layout(
            template='plotly_dark',
            paper_bgcolor='#0b0f14',
            plot_bgcolor='#0e141b',
            xaxis_title='Flight Time (Minutes)',
            yaxis_title='Capacity (%)',
            yaxis=dict(range=[0, 105]),
            margin=dict(l=30, r=20, t=20, b=30),
            height=260
        )
        st.plotly_chart(fig_batt, use_container_width=True)

    with c2:
        st.markdown("##### 🎯 Detection Distribution by Category")
        cat_counts = {}
        for ev in events:
            cat = ev.get("category", "other").upper()
            cat_counts[cat] = cat_counts.get(cat, 0) + 1

        if not cat_counts:
            cat_counts = {"PERSON": 2, "VEHICLE": 1, "WILDLIFE": 2, "FIRE": 1}

        fig_pie = go.Figure(data=[go.Pie(
            labels=list(cat_counts.keys()),
            values=list(cat_counts.values()),
            hole=0.45,
            marker=dict(colors=['#00e5ff', '#ff9100', '#76ff03', '#ff1744', '#b0bec5'])
        )])
        fig_pie.update_layout(
            template='plotly_dark',
            paper_bgcolor='#0b0f14',
            margin=dict(l=20, r=20, t=20, b=20),
            height=260
        )
        st.plotly_chart(fig_pie, use_container_width=True)

    st.markdown("---")

    # Automated PDF & CSV Report Generator
    st.markdown("##### 📄 Export Mission Report & Audit Trail")
    r_col1, r_col2 = st.columns(2)

    rep_gen = MissionReportGenerator()
    pdf_path = rep_gen.generate_pdf_report(mission_id)
    csv_path = rep_gen.generate_csv_report(mission_id)

    with r_col1:
        if os.path.exists(pdf_path):
            with open(pdf_path, "rb") as f:
                st.download_button(
                    label="📥 Download Mission PDF Audit Report",
                    data=f.read(),
                    file_name=f"mission_{mission_id}_report.pdf",
                    mime="application/pdf",
                    use_container_width=True
                )

    with r_col2:
        if os.path.exists(csv_path):
            with open(csv_path, "rb") as f:
                st.download_button(
                    label="📥 Download Granular CSV Event Records",
                    data=f.read(),
                    file_name=f"mission_{mission_id}_events.csv",
                    mime="text/csv",
                    use_container_width=True
                )
