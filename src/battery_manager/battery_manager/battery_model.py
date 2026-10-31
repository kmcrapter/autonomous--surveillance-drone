"""Physics-based battery depletion and dynamic RTH energy calculation."""

import math
import time
from typing import Dict, Any

class BatteryModel:
    """Simulates realistic electrochemical depletion and charging dynamics."""

    def __init__(self,
                 capacity_wh: float = 555.0, # 6S 25,000 mAh @ 22.2V nominal
                 nominal_voltage: float = 22.2,
                 base_power_w: float = 45.0, # Avionics, sensors, gimbal, compute
                 hover_power_w: float = 620.0, # 12.5kg Hexacopter hover power
                 climb_coeff: float = 180.0, # W per m/s climb
                 speed_coeff: float = 12.0, # W per (m/s)^2 horizontal
                 safety_reserve_pct: float = 20.0,
                 charge_rate_pct_sec: float = 2.5):
        self.capacity_wh = capacity_wh
        self.nominal_voltage = nominal_voltage
        self.base_power_w = base_power_w
        self.hover_power_w = hover_power_w
        self.climb_coeff = climb_coeff
        self.speed_coeff = speed_coeff
        self.safety_reserve_pct = safety_reserve_pct
        self.charge_rate_pct_sec = charge_rate_pct_sec

        self.remaining_wh = capacity_wh
        self.percentage = 100.0
        self.voltage = 25.2 # Fully charged 4.2V/cell
        self.current_draw_a = 0.0
        self.total_energy_consumed_wh = 0.0
        self.is_charging = False
        self.last_update_time = time.time()

    def update(self,
               dt: float,
               vx: float,
               vy: float,
               vz: float,
               altitude: float,
               is_flying: bool,
               charging_override: bool = False) -> Dict[str, Any]:
        """Updates battery state over time step dt."""
        now = time.time()
        if dt <= 0:
            dt = max(0.01, now - self.last_update_time)
        self.last_update_time = now

        if charging_override:
            self.is_charging = True
            # Fast inductive charging
            charge_delta_wh = (self.capacity_wh * (self.charge_rate_pct_sec / 100.0)) * dt
            self.remaining_wh = min(self.capacity_wh, self.remaining_wh + charge_delta_wh)
            self.percentage = (self.remaining_wh / self.capacity_wh) * 100.0
            self.voltage = 21.0 + (4.2 * (self.percentage / 100.0))
            self.current_draw_a = -15.0 # Negative indicating current flowing into pack
            power_w = -charge_delta_wh / (dt / 3600.0)
            return self.get_status(power_w, 0.0, 0.0, 0.0, 0.0)

        self.is_charging = False

        if not is_flying:
            # Standby idle power draw on landing pad
            power_w = self.base_power_w * 0.4
        else:
            h_speed_sq = (vx * vx) + (vy * vy)
            climb_pwr = self.climb_coeff * max(0.0, vz)
            drag_pwr = self.speed_coeff * h_speed_sq
            power_w = self.base_power_w + self.hover_power_w + climb_pwr + drag_pwr

        # Energy consumed in Wh
        energy_wh = power_w * (dt / 3600.0)
        self.remaining_wh = max(0.0, self.remaining_wh - energy_wh)
        self.total_energy_consumed_wh += energy_wh
        self.percentage = max(0.0, min(100.0, (self.remaining_wh / self.capacity_wh) * 100.0))

        # Cell discharge voltage curve: 25.2V (100%) to 20.4V (0%)
        self.voltage = 20.4 + (4.8 * (self.percentage / 100.0))
        self.current_draw_a = power_w / max(1.0, self.voltage)

        return self.get_status(power_w, vx, vy, vz, altitude)

    def calculate_rth_requirements(self, x: float, y: float, z: float, transit_speed: float = 4.5) -> Dict[str, float]:
        """Calculates energy and time strictly needed to transit back to Home (0, 0, 0.38) and land."""
        dist_xy = math.sqrt((x * x) + (y * y))
        transit_time_sec = dist_xy / max(1.0, transit_speed)
        descend_time_sec = max(0.0, z - 0.4) / 1.5 # Descend at 1.5 m/s
        landing_time_sec = 8.0 # Final flare, alignment and touchdown

        total_rth_time_sec = transit_time_sec + descend_time_sec + landing_time_sec

        # Power during transit
        p_transit = self.base_power_w + self.hover_power_w + (self.speed_coeff * (transit_speed ** 2))
        energy_needed_wh = p_transit * (total_rth_time_sec / 3600.0)
        energy_needed_pct = (energy_needed_wh / self.capacity_wh) * 100.0

        trigger_threshold_pct = energy_needed_pct + self.safety_reserve_pct

        return {
            "dist_to_home_m": round(dist_xy, 1),
            "rth_time_sec": round(total_rth_time_sec, 1),
            "rth_energy_wh": round(energy_needed_wh, 2),
            "rth_energy_pct": round(energy_needed_pct, 1),
            "rth_trigger_pct": round(trigger_threshold_pct, 1),
            "should_return": self.percentage <= trigger_threshold_pct
        }

    def get_status(self, power_w: float, vx: float, vy: float, vz: float, altitude: float) -> Dict[str, Any]:
        """Returns comprehensive telemetry dictionary."""
        effective_power = max(10.0, power_w if not self.is_charging else self.hover_power_w)
        remaining_hours = self.remaining_wh / effective_power
        remaining_minutes = max(0.0, remaining_hours * 60.0)

        return {
            "percentage": round(self.percentage, 2),
            "voltage": round(self.voltage, 2),
            "current_a": round(self.current_draw_a, 2),
            "power_w": round(power_w, 1),
            "remaining_wh": round(self.remaining_wh, 3),
            "consumed_wh": round(self.total_energy_consumed_wh, 3),
            "remaining_time_min": round(remaining_minutes, 1),
            "is_charging": self.is_charging,
            "capacity_wh": self.capacity_wh
        }
