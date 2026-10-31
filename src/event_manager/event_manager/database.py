"""SQLite Data Access Layer for Surveillance Twin Missions, Events, and Personnel Security."""

import sqlite3
import os
import time
from typing import List, Dict, Any, Optional

DEFAULT_DB_PATH = "/home/hari/.gemini/antigravity/scratch/surveillance_drone_ws/data/db/surveillance.db"

class SurveillanceDatabase:
    """Manages persistent SQLite storage for missions, events, telemetry, operator reviews, and personnel security."""

    def __init__(self, db_path: str = DEFAULT_DB_PATH):
        self.db_path = db_path
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self.init_db()

    def get_connection(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path, timeout=10.0)
        conn.row_factory = sqlite3.Row
        return conn

    def init_db(self):
        with self.get_connection() as conn:
            cursor = conn.cursor()

            # Missions Table
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS missions (
                id TEXT PRIMARY KEY,
                scenario TEXT,
                start_time REAL,
                end_time REAL,
                status TEXT,
                assigned_zones TEXT,
                waypoints_total INTEGER,
                distance_m REAL,
                battery_consumed_pct REAL,
                rth_count INTEGER DEFAULT 0,
                summary TEXT
            )
            """)

            # Observations & Events Table
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS events (
                id TEXT PRIMARY KEY,
                mission_id TEXT,
                timestamp REAL,
                category TEXT,
                label TEXT,
                track_id TEXT,
                confidence REAL,
                world_x REAL,
                world_y REAL,
                camera_source TEXT,
                snapshot_path TEXT,
                review_status TEXT DEFAULT 'PENDING_REVIEW',
                operator_notes TEXT DEFAULT '',
                FOREIGN KEY(mission_id) REFERENCES missions(id)
            )
            """)

            # Telemetry History Table
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS telemetry_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                mission_id TEXT,
                timestamp REAL,
                state TEXT,
                x REAL,
                y REAL,
                z REAL,
                speed REAL,
                battery_pct REAL,
                current_zone TEXT
            )
            """)

            # Coverage Footprints Table
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS coverage_records (
                mission_id TEXT,
                zone_id TEXT,
                grid_x INTEGER,
                grid_y INTEGER,
                timestamp REAL,
                PRIMARY KEY (mission_id, zone_id, grid_x, grid_y)
            )
            """)

            # Personnel Identification Audit Logs Table
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS personnel_audit_logs (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp REAL,
                entity_id TEXT,
                personnel_id TEXT,
                full_name TEXT,
                assigned_unit TEXT,
                tag_id TEXT,
                tag_type TEXT,
                identity_state TEXT,
                face_status TEXT,
                confidence REAL,
                world_x REAL,
                world_y REAL,
                details TEXT,
                review_status TEXT DEFAULT 'PENDING_REVIEW',
                operator_notes TEXT DEFAULT ''
            )
            """)

            # Personnel Verifications & Voice Announcements Table
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS personnel_verifications (
                id TEXT PRIMARY KEY,
                timestamp REAL,
                entity_id TEXT,
                action TEXT,
                status TEXT,
                notes TEXT
            )
            """)

            # Policy Security Events Table
            cursor.execute("""
            CREATE TABLE IF NOT EXISTS security_policy_events (
                id TEXT PRIMARY KEY,
                timestamp REAL,
                policy_rule_id TEXT,
                event_category TEXT,
                zone_id TEXT,
                zone_name TEXT,
                target_entity_id TEXT,
                target_name TEXT,
                target_identity_state TEXT,
                severity TEXT,
                description TEXT,
                review_status TEXT DEFAULT 'PENDING_REVIEW',
                operator_notes TEXT DEFAULT ''
            )
            """)

            conn.commit()

    def upsert_mission(self, mission_data: Dict[str, Any]):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
            INSERT INTO missions (id, scenario, start_time, end_time, status, assigned_zones, waypoints_total, distance_m, battery_consumed_pct, rth_count, summary)
            VALUES (:id, :scenario, :start_time, :end_time, :status, :assigned_zones, :waypoints_total, :distance_m, :battery_consumed_pct, :rth_count, :summary)
            ON CONFLICT(id) DO UPDATE SET
                end_time=excluded.end_time,
                status=excluded.status,
                distance_m=excluded.distance_m,
                battery_consumed_pct=excluded.battery_consumed_pct,
                rth_count=excluded.rth_count,
                summary=excluded.summary
            """, mission_data)
            conn.commit()

    def insert_event(self, event_data: Dict[str, Any]):
        data = dict(event_data)
        if "id" not in data and "event_id" in data:
            data["id"] = data["event_id"]
        data.setdefault("camera_source", "forward_rgb")
        data.setdefault("snapshot_path", "")
        data.setdefault("review_status", "PENDING_REVIEW")
        data.setdefault("operator_notes", "")
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
            INSERT OR IGNORE INTO events (id, mission_id, timestamp, category, label, track_id, confidence, world_x, world_y, camera_source, snapshot_path, review_status, operator_notes)
            VALUES (:id, :mission_id, :timestamp, :category, :label, :track_id, :confidence, :world_x, :world_y, :camera_source, :snapshot_path, :review_status, :operator_notes)
            """, data)
            conn.commit()

    def update_event_review(self, event_id: str, review_status: str, operator_notes: str = None):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            if operator_notes is not None:
                cursor.execute("""
                UPDATE events SET review_status = ?, operator_notes = ? WHERE id = ?
                """, (review_status, operator_notes, event_id))
            else:
                cursor.execute("""
                UPDATE events SET review_status = ? WHERE id = ?
                """, (review_status, event_id))
            conn.commit()

    def insert_telemetry_sample(self, sample: Dict[str, Any]):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
            INSERT INTO telemetry_logs (mission_id, timestamp, state, x, y, z, speed, battery_pct, current_zone)
            VALUES (:mission_id, :timestamp, :state, :x, :y, :z, :speed, :battery_pct, :current_zone)
            """, sample)
            conn.commit()

    def get_recent_events(self, limit: int = 50, mission_id: Optional[str] = None) -> List[Dict[str, Any]]:
        with self.get_connection() as conn:
            cursor = conn.cursor()
            if mission_id:
                cursor.execute("SELECT * FROM events WHERE mission_id = ? ORDER BY timestamp DESC LIMIT ?", (mission_id, limit))
            else:
                cursor.execute("SELECT * FROM events ORDER BY timestamp DESC LIMIT ?", (limit,))
            return [dict(row) for row in cursor.fetchall()]

    def get_all_events(self, mission_id: Optional[str] = None) -> List[Dict[str, Any]]:
        with self.get_connection() as conn:
            cursor = conn.cursor()
            if mission_id:
                cursor.execute("SELECT * FROM events WHERE mission_id = ? ORDER BY timestamp DESC", (mission_id,))
            else:
                cursor.execute("SELECT * FROM events ORDER BY timestamp DESC")
            return [dict(row) for row in cursor.fetchall()]

    def get_latest_mission(self) -> Optional[Dict[str, Any]]:
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM missions ORDER BY start_time DESC LIMIT 1")
            row = cursor.fetchone()
            return dict(row) if row else None

    def get_all_missions(self) -> List[Dict[str, Any]]:
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM missions ORDER BY start_time DESC")
            return [dict(row) for row in cursor.fetchall()]

    def get_telemetry_for_mission(self, mission_id: str) -> List[Dict[str, Any]]:
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM telemetry_logs WHERE mission_id = ? ORDER BY timestamp ASC", (mission_id,))
            return [dict(row) for row in cursor.fetchall()]

    # --- Personnel Identification & Security Operations ---

    def insert_personnel_audit_log(self, log_data: Dict[str, Any]):
        data = dict(log_data)
        data.setdefault("details", "")
        data.setdefault("review_status", "PENDING_REVIEW")
        data.setdefault("operator_notes", "")
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
            INSERT INTO personnel_audit_logs (
                timestamp, entity_id, personnel_id, full_name, assigned_unit,
                tag_id, tag_type, identity_state, face_status, confidence,
                world_x, world_y, details, review_status, operator_notes
            ) VALUES (
                :timestamp, :entity_id, :personnel_id, :full_name, :assigned_unit,
                :tag_id, :tag_type, :identity_state, :face_status, :confidence,
                :world_x, :world_y, :details, :review_status, :operator_notes
            )
            """, data)
            conn.commit()

    def get_recent_personnel_audit_logs(self, limit: int = 50) -> List[Dict[str, Any]]:
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM personnel_audit_logs ORDER BY timestamp DESC LIMIT ?", (limit,))
            return [dict(row) for row in cursor.fetchall()]

    def insert_personnel_verification(self, verif_data: Dict[str, Any]):
        data = dict(verif_data)
        if "id" not in data:
            data["id"] = f"VERIF-{int(time.time()*1000)}"
        data.setdefault("timestamp", time.time())
        data.setdefault("status", "RECORDED")
        data.setdefault("notes", "")
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
            INSERT OR REPLACE INTO personnel_verifications (id, timestamp, entity_id, action, status, notes)
            VALUES (:id, :timestamp, :entity_id, :action, :status, :notes)
            """, data)
            conn.commit()

    def get_recent_personnel_verifications(self, limit: int = 50) -> List[Dict[str, Any]]:
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM personnel_verifications ORDER BY timestamp DESC LIMIT ?", (limit,))
            return [dict(row) for row in cursor.fetchall()]

    def insert_security_policy_event(self, event_data: Dict[str, Any]):
        data = dict(event_data)
        if "id" not in data and "event_id" in data:
            data["id"] = data["event_id"]
        elif "id" not in data:
            data["id"] = f"SEC-POL-{int(time.time()*1000)}"
        data.setdefault("timestamp", time.time())
        data.setdefault("policy_rule_id", "RULE-GENERAL")
        data.setdefault("event_category", "POLICY_EVENT")
        data.setdefault("zone_id", "ZONE-N/A")
        data.setdefault("zone_name", "General")
        data.setdefault("target_entity_id", "UNKNOWN")
        data.setdefault("target_name", "Subject")
        data.setdefault("target_identity_state", "UNVERIFIED")
        data.setdefault("severity", "MEDIUM")
        data.setdefault("description", "")
        data.setdefault("review_status", "PENDING_REVIEW")
        data.setdefault("operator_notes", "")
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
            INSERT OR REPLACE INTO security_policy_events (
                id, timestamp, policy_rule_id, event_category, zone_id, zone_name,
                target_entity_id, target_name, target_identity_state, severity,
                description, review_status, operator_notes
            ) VALUES (
                :id, :timestamp, :policy_rule_id, :event_category, :zone_id, :zone_name,
                :target_entity_id, :target_name, :target_identity_state, :severity,
                :description, :review_status, :operator_notes
            )
            """, data)
            conn.commit()

    def get_recent_security_policy_events(self, limit: int = 50) -> List[Dict[str, Any]]:
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("SELECT * FROM security_policy_events ORDER BY timestamp DESC LIMIT ?", (limit,))
            return [dict(row) for row in cursor.fetchall()]

    def update_personnel_review(self, entity_id: str, review_status: str, operator_notes: str = ""):
        with self.get_connection() as conn:
            cursor = conn.cursor()
            cursor.execute("""
            UPDATE personnel_audit_logs
            SET review_status = ?, operator_notes = ?
            WHERE entity_id = ?
            """, (review_status, operator_notes, entity_id))
            cursor.execute("""
            UPDATE security_policy_events
            SET review_status = ?, operator_notes = ?
            WHERE target_entity_id = ?
            """, (review_status, operator_notes, entity_id))
            conn.commit()
