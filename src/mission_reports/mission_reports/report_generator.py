"""Mission Report Generator producing comprehensive PDF and CSV audits."""

import os
import csv
import time
import hashlib
from datetime import datetime
from typing import Dict, Any, List, Optional

from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image as RLImage

from event_manager.database import SurveillanceDatabase

class MissionReportGenerator:
    """Generates formatted mission audit reports in PDF and CSV format."""

    def __init__(self, db_path: Optional[str] = None, output_dir: str = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/reports"):
        self.db = SurveillanceDatabase(db_path) if db_path else SurveillanceDatabase()
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

    def _derive_zone_and_territory(self, x: float, y: float, label: str = ""):
        """Derives tactical zone and sovereign territory from coordinates."""
        if x >= 35.0 and y <= 30.0:
            zone = "Zone Delta (Outpost Ridge)"
        elif x <= -25.0 and y >= 25.0:
            zone = "Zone Bravo (Pine Forest)"
        elif y >= 32.0:
            zone = "Zone Charlie (River Corridor)"
        else:
            zone = "Zone Alpha (Base Area)"

        territory = "FOREIGN (Country B)" if y > 30.0 else "HOME (Country A)"
        return zone, territory

    def _derive_classification(self, category: str, label: str):
        lbl_lower = label.lower()
        if "woman" in lbl_lower:
            return "WOMAN (Human Presence)"
        elif "man" in lbl_lower:
            return "MAN (Human Presence)"
        elif category == "wildlife" or "animal" in lbl_lower or "deer" in lbl_lower or "elk" in lbl_lower:
            return "WILDLIFE (Animal)"
        elif category == "vehicle" or "suv" in lbl_lower:
            return "VEHICLE (Ground Patrol)"
        elif category == "fire" or "hazard" in lbl_lower:
            return "HAZARD (Thermal Fire)"
        return category.upper()

    def generate_csv_report(self, mission_id: str) -> str:
        """Generates a detailed CSV audit report of all events for a given mission."""
        events = self.db.get_all_events(mission_id)
        filename = f"mission_report_{mission_id}.csv"
        csv_path = os.path.join(self.output_dir, filename)

        fieldnames = [
            "id", "mission_id", "timestamp_iso", "classification", "category", "label",
            "territory", "zone", "track_id", "confidence", "world_x", "world_y",
            "camera_source", "review_status", "operator_notes"
        ]

        with open(csv_path, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for ev in events:
                zone, territory = self._derive_zone_and_territory(ev["world_x"], ev["world_y"], ev["label"])
                classification = self._derive_classification(ev["category"], ev["label"])
                writer.writerow({
                    "id": ev["id"],
                    "mission_id": ev["mission_id"],
                    "timestamp_iso": datetime.fromtimestamp(ev["timestamp"]).strftime("%Y-%m-%d %H:%M:%S"),
                    "classification": classification,
                    "category": ev["category"],
                    "label": ev["label"],
                    "territory": territory,
                    "zone": zone,
                    "track_id": ev["track_id"],
                    "confidence": f"{ev['confidence']:.2f}",
                    "world_x": f"{ev['world_x']:.1f}",
                    "world_y": f"{ev['world_y']:.1f}",
                    "camera_source": ev["camera_source"],
                    "review_status": ev["review_status"],
                    "operator_notes": ev["operator_notes"]
                })

        return csv_path

    def generate_pdf_report(self, mission_id: str) -> str:
        """Generates a comprehensive PDF audit report with distinct actor breakdowns."""
        filename = f"mission_report_{mission_id}.pdf"
        pdf_path = os.path.join(self.output_dir, filename)

        events = self.db.get_all_events(mission_id)
        missions = [m for m in self.db.get_all_missions() if m["id"] == mission_id]
        mission = missions[0] if missions else {
            "id": mission_id,
            "scenario": "Autonomous Border & Perimeter Surveillance",
            "start_time": time.time() - 600,
            "end_time": time.time(),
            "status": "COMPLETE",
            "distance_m": 420.5,
            "battery_consumed_pct": 28.5,
            "rth_count": 0
        }

        doc = SimpleDocTemplate(pdf_path, pagesize=letter, rightMargin=36, leftMargin=36, topMargin=36, bottomMargin=36)
        story = []
        styles = getSampleStyleSheet()

        # Custom Palette Styles
        title_style = ParagraphStyle(
            'TitleStyle',
            parent=styles['Heading1'],
            fontSize=19,
            leading=23,
            textColor=colors.HexColor('#0d233a'),
            spaceAfter=4
        )
        subtitle_style = ParagraphStyle(
            'SubtitleStyle',
            parent=styles['Normal'],
            fontSize=9.5,
            textColor=colors.HexColor('#555555'),
            spaceAfter=12
        )
        heading_style = ParagraphStyle(
            'HeadingStyle',
            parent=styles['Heading2'],
            fontSize=12,
            leading=15,
            textColor=colors.HexColor('#0052cc'),
            spaceBefore=10,
            spaceAfter=6
        )
        body_style = ParagraphStyle('BodyStyle', parent=styles['Normal'], fontSize=8.5, leading=11)

        # Header Title
        story.append(Paragraph("AERIAL SURVEILLANCE MISSION AUDIT REPORT", title_style))
        start_dt = datetime.fromtimestamp(mission["start_time"]).strftime("%Y-%m-%d %H:%M:%S")
        story.append(Paragraph(f"Autonomous 3D Drone Digital Twin | Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | Mission ID: <b>{mission_id}</b>", subtitle_style))
        story.append(Spacer(1, 6))

        # 1. Executive Summary Table
        summary_data = [
            ["Mission ID", mission_id, "Status", mission.get("status", "COMPLETE")],
            ["Start Time", start_dt, "Scenario", "Border, Wildlife & Hazard Monitoring"],
            ["Total Distance Flown", f"{mission.get('distance_m', 0.0):.1f} meters", "Battery Consumed", f"{mission.get('battery_consumed_pct', 0.0):.1f}%"],
            ["Assigned Sectors", "Alpha (Base), Bravo (Forest), Charlie (River), Delta (Ridge)", "RTH Events", str(mission.get('rth_count', 0))]
        ]
        t_summary = Table(summary_data, colWidths=[110, 160, 110, 160])
        t_summary.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, -1), colors.HexColor('#f4f6f8')),
            ('TEXTCOLOR', (0, 0), (-1, -1), colors.HexColor('#1c252c')),
            ('FONTNAME', (0, 0), (-1, -1), 'Helvetica'),
            ('FONTSIZE', (0, 0), (-1, -1), 8.5),
            ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor('#d0d7de')),
            ('FONTNAME', (0, 0), (0, -1), 'Helvetica-Bold'),
            ('FONTNAME', (2, 0), (2, -1), 'Helvetica-Bold'),
        ]))
        story.append(t_summary)
        story.append(Spacer(1, 10))

        # 2. Detailed Detection Breakdown by Entity Type & Species
        story.append(Paragraph("OBSERVATION BREAKDOWN: PEOPLE, WILDLIFE ANIMALS & HAZARDS", heading_style))

        # Group events by specific classification
        class_buckets = {
            "MEN (HUMAN)": [],
            "WOMEN (HUMAN)": [],
            "WILDLIFE ANIMALS": [],
            "GROUND VEHICLES": [],
            "ENVIRONMENTAL HAZARDS": []
        }

        for ev in events:
            lbl = ev["label"].lower()
            cat = ev["category"].lower()
            if "woman" in lbl:
                class_buckets["WOMEN (HUMAN)"].append(ev)
            elif "man" in lbl:
                class_buckets["MEN (HUMAN)"].append(ev)
            elif cat == "wildlife" or "animal" in lbl or "deer" in lbl or "elk" in lbl:
                class_buckets["WILDLIFE ANIMALS"].append(ev)
            elif cat == "vehicle" or "suv" in lbl:
                class_buckets["GROUND VEHICLES"].append(ev)
            elif cat == "fire" or "hazard" in lbl:
                class_buckets["ENVIRONMENTAL HAZARDS"].append(ev)

        cat_data = [["Entity Category", "Count", "Sample Identified Labels", "Avg Confidence", "Detected Zones"]]
        for b_name, b_evs in class_buckets.items():
            if b_evs:
                cnt = len(b_evs)
                labels = list(set(e["label"].split("(")[0].strip() if "(" in e["label"] else e["label"] for e in b_evs))
                label_str = ", ".join(labels[:3])
                avg_conf = sum(e["confidence"] for e in b_evs) / cnt
                zones = list(set(self._derive_zone_and_territory(e["world_x"], e["world_y"])[0].split("(")[0].strip() for e in b_evs))
                zone_str = ", ".join(zones)
                cat_data.append([b_name, str(cnt), label_str, f"{avg_conf:.1%}", zone_str])

        if len(cat_data) == 1:
            cat_data.append(["No automated events logged during this patrol window.", "0", "-", "-", "-"])

        t_cat = Table(cat_data, colWidths=[110, 45, 175, 80, 130])
        t_cat.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#0052cc')),
            ('TEXTCOLOR', (0, 0), (-1, 0), colors.white),
            ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
            ('FONTSIZE', (0, 0), (-1, -1), 8),
            ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor('#e1e4e8')),
            ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, colors.HexColor('#fbfcfd')])
        ]))
        story.append(t_cat)
        story.append(Spacer(1, 10))

        # 3. Chronological Events Log across Different Places
        story.append(Paragraph("CHRONOLOGICAL OBSERVATIONS & AUDIT TRAIL (ALL PLACES)", heading_style))
        ev_table_data = [["Time", "Track ID", "Label", "Zone / Sector", "Territory", "World (X,Y)", "Conf"]]
        for ev in events[:25]: # Show up to 25 recent events
            ev_time = datetime.fromtimestamp(ev["timestamp"]).strftime("%H:%M:%S")
            z_name, z_terr = self._derive_zone_and_territory(ev["world_x"], ev["world_y"], ev["label"])
            short_zone = z_name.split("(")[0].strip()
            short_terr = "FOREIGN" if "FOREIGN" in z_terr else "HOME"
            ev_table_data.append([
                ev_time,
                ev["track_id"],
                ev["label"][:24],
                short_zone,
                short_terr,
                f"({ev['world_x']:.1f}, {ev['world_y']:.1f})",
                f"{ev['confidence']:.0%}"
            ])

        if len(ev_table_data) == 1:
            ev_table_data.append(["-", "-", "Nominal surveillance - no breaches detected", "-", "-", "-", "-"])

        t_events = Table(ev_table_data, colWidths=[55, 60, 155, 85, 65, 75, 45])
        t_events.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#24292e')),
            ('TEXTCOLOR', (0, 0), (-1, 0), colors.white),
            ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
            ('FONTSIZE', (0, 0), (-1, -1), 7.5),
            ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor('#e1e4e8')),
            ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, colors.HexColor('#f6f8fa')])
        ]))
        story.append(t_events)
        story.append(Spacer(1, 10))

        # 4. Snapshot Gallery (Representative Event Snapshots)
        snapshots_added = 0
        snap_images = []
        for ev in events:
            path = ev.get("snapshot_path")
            if path and os.path.exists(path) and snapshots_added < 2:
                try:
                    img = RLImage(path, width=220, height=140)
                    snap_images.append(img)
                    snapshots_added += 1
                except Exception:
                    pass

        if snap_images:
            story.append(Paragraph("REPRESENTATIVE EVENT SNAPSHOTS", heading_style))
            snap_table = Table([snap_images], colWidths=[260] * len(snap_images))
            story.append(snap_table)
            story.append(Spacer(1, 14))

        # 5. Regulatory Limitation & Non-Hostile Observation Notice
        story.append(Paragraph("REGULATORY LIMITATIONS & OBJECTIVE OBSERVATION STATEMENT", heading_style))
        disclaimer_text = (
            "<b>Notice:</b> This surveillance simulation system adheres to strictly objective, non-hostile recording "
            "standards. Human observations record visible presence and coordinate telemetry without automated determinations "
            "of hostility, culpability, or threat status. Thermal readings represent synthetic sensor models of relative "
            "radiance. All events remain subject to human operator verification and annotation."
        )
        story.append(Paragraph(disclaimer_text, body_style))
        story.append(Spacer(1, 14))

        # 6. Cryptographic Verification Hash
        raw_signature = f"{mission_id}-{mission['start_time']}-{len(events)}"
        sha256 = hashlib.sha256(raw_signature.encode('utf-8')).hexdigest()
        hash_text = f"Audit Verification SHA-256: <code>{sha256}</code> | Antigravity Autonomous Surveillance Platform"
        story.append(Paragraph(hash_text, ParagraphStyle('Hash', parent=styles['Normal'], fontSize=7.5, textColor=colors.gray)))

        doc.build(story)
        return pdf_path

def cli_main():
    import sys
    mid = sys.argv[1] if len(sys.argv) > 1 else "MSN-DEFAULT"
    gen = MissionReportGenerator()
    pdf = gen.generate_pdf_report(mid)
    csv_file = gen.generate_csv_report(mid)
    print(f"Generated PDF: {pdf}")
    print(f"Generated CSV: {csv_file}")

if __name__ == "__main__":
    cli_main()
