"""Event Center & Chronological Audit Feed Component."""

import streamlit as st
import pandas as pd
from datetime import datetime
from typing import List, Dict, Any

def render_event_center(events: List[Dict[str, Any]], bridge=None):
    """Renders interactive event feed with filtering, status updates, and operator annotations."""
    st.markdown("### 🚨 Observation Event Center & Operator Audit")

    if not events:
        st.info("No security or environmental observations recorded yet. Active monitoring in progress.")
        return

    # Filter Controls
    fcol1, fcol2, fcol3 = st.columns([2, 2, 2])
    with fcol1:
        categories = ["ALL"] + sorted(list(set(e.get("category", "") for e in events)))
        selected_cat = st.selectbox("Filter Category", categories, index=0)
    with fcol2:
        statuses = ["ALL", "PENDING_REVIEW", "ACKNOWLEDGED", "DISMISSED"]
        selected_status = st.selectbox("Filter Review Status", statuses, index=0)
    with fcol3:
        search_query = st.text_input("Search (Track ID / Label)", "")

    # Apply filters
    filtered = events
    if selected_cat != "ALL":
        filtered = [e for e in filtered if e.get("category") == selected_cat]
    if selected_status != "ALL":
        filtered = [e for e in filtered if e.get("review_status") == selected_status]
    if search_query:
        q = search_query.lower()
        filtered = [e for e in filtered if q in e.get("label", "").lower() or q in e.get("track_id", "").lower()]

    st.markdown(f"**Showing {len(filtered)} observation records**")

    # Interactive Event Cards / Table
    for i, ev in enumerate(filtered[:15]): # Display top 15 matching
        ev_id = ev["id"]
        ts_str = datetime.fromtimestamp(ev["timestamp"]).strftime("%H:%M:%S")
        status = ev.get("review_status", "PENDING_REVIEW")
        status_color = "#ffb300" if status == "PENDING_REVIEW" else ("#00e676" if status == "ACKNOWLEDGED" else "#78909c")

        with st.expander(f"[{ts_str}] {ev.get('track_id', '')} - {ev.get('label', '')} ({ev.get('confidence', 0):.0%}) | Status: {status}", expanded=(i==0)):
            c1, c2, c3 = st.columns([1.5, 2.5, 2])
            with c1:
                snap = ev.get("snapshot_path")
                if snap and st.session_state.get(f"show_img_{ev_id}", True):
                    try:
                        st.image(snap, caption=f"Event {ev_id}", use_container_width=True)
                    except Exception:
                        st.caption("Snapshot unreadable")
                else:
                    st.caption("📷 Snapshot Stored")

            with c2:
                st.markdown(f"""
                - **Category:** `{ev.get('category', '').upper()}`
                - **Location (X, Y):** `({ev.get('world_x', 0):.1f}, {ev.get('world_y', 0):.1f})`
                - **Confidence:** `{ev.get('confidence', 0):.1%}`
                - **Sensor Source:** `{ev.get('camera_source', 'forward_rgb')}`
                - **Operator Notes:** *{ev.get('operator_notes') or 'None logged'}*
                """)

            with c3:
                st.markdown("**Operator Review Controls**")
                btn_col1, btn_col2 = st.columns(2)
                with btn_col1:
                    if st.button("Acknowledge", key=f"ack_{ev_id}"):
                        if bridge:
                            bridge.update_event_review(ev_id, "ACKNOWLEDGED")
                            st.success(f"{ev_id} Acknowledged")
                            st.rerun()
                with btn_col2:
                    if st.button("Dismiss", key=f"dism_{ev_id}"):
                        if bridge:
                            bridge.update_event_review(ev_id, "DISMISSED")
                            st.info(f"{ev_id} Dismissed")
                            st.rerun()

                new_notes = st.text_input("Add Operator Note", key=f"notes_in_{ev_id}", placeholder="Enter audit verification comment...")
                if st.button("Save Note", key=f"save_notes_{ev_id}"):
                    if bridge and new_notes:
                        bridge.update_event_review(ev_id, status, new_notes)
                        st.success("Note saved.")
                        st.rerun()
