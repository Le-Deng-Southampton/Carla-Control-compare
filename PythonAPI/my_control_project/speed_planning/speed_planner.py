# uncompyle6 version 3.9.3
# Python bytecode version base 3.7.0 (3394)
# Decompiled from: Python 3.13.13 | packaged by Anaconda, Inc. | (main, Apr 14 2026, 06:12:50) [MSC v.1942 64 bit (AMD64)]
# Embedded file name: D:\WindowsNoEditor\PythonAPI\my_control_project\speed_planning\speed_planner.py
# Compiled at: 2026-07-06 19:57:25
# Size of source mod 2**32: 12501 bytes
from dataclasses import dataclass
import numpy as np

@dataclass
class SpeedPlannerConfig:
    base_target_speed_kmh: float
    min_turn_speed_kmh = 48.0
    min_turn_speed_kmh: float
    recovery_min_speed_kmh = 40.0
    recovery_min_speed_kmh: float
    max_lateral_accel = 18.0
    max_lateral_accel: float
    max_accel = 3.5
    max_accel: float
    max_decel = 9.0
    max_decel: float
    curvature_epsilon = 0.0001
    curvature_epsilon: float
    lateral_error_warning = 1.6
    lateral_error_warning: float
    lateral_error_critical = 2.6
    lateral_error_critical: float
    heading_error_warning_rad = np.radians(12.0)
    heading_error_warning_rad: float
    heading_error_critical_rad = np.radians(18.0)
    heading_error_critical_rad: float
    lateral_error_rate_warning = 0.8
    lateral_error_rate_warning: float
    lateral_error_rate_critical = 1.6
    lateral_error_rate_critical: float
    heading_error_rate_warning_rad = np.radians(8.0)
    heading_error_rate_warning_rad: float
    heading_error_rate_critical_rad = np.radians(16.0)
    heading_error_rate_critical_rad: float
    lateral_error_rate_activation = 0.8
    lateral_error_rate_activation: float
    heading_error_rate_activation_rad = np.radians(8.0)
    heading_error_rate_activation_rad: float
    error_rate_filter_alpha = 0.25
    error_rate_filter_alpha: float
    recovery_hold_steps = 3
    recovery_hold_steps: int
    entry_max_speed_kmh = 70.0
    entry_max_speed_kmh: float
    entry_curvature_threshold = 0.015
    entry_curvature_threshold: float
    entry_full_cap_curvature = 0.03
    entry_full_cap_curvature: float
    emergency_turn_speed_kmh = 36.0
    emergency_turn_speed_kmh: float
    emergency_curvature_threshold = 0.08
    emergency_curvature_threshold: float
    emergency_full_cap_curvature = 0.12
    emergency_full_cap_curvature: float


class CurvatureSpeedPlanner:
    __doc__ = "Plans the fastest target speed allowed by previewed route curvature."

    def __init__(self, config):
        self.config = config
        self.base_target_speed_mps = config.base_target_speed_kmh / 3.6
        self.min_turn_speed_mps = config.min_turn_speed_kmh / 3.6
        self.recovery_min_speed_mps = config.recovery_min_speed_kmh / 3.6
        self.emergency_turn_speed_mps = config.emergency_turn_speed_kmh / 3.6
        self.entry_max_speed_mps = config.entry_max_speed_kmh / 3.6
        self._planned_speed_mps = None
        self._prev_lateral_error_m = None
        self._prev_heading_error_rad = None
        self._filtered_lateral_error_rate = 0.0
        self._filtered_heading_error_rate = 0.0
        self._hold_steps_remaining = 0
        self.last_risk = 0.0
        self.last_reason = "none"

    @property
    def planned_speed_mps(self):
        if self._planned_speed_mps is None:
            return self.base_target_speed_mps
        return self._planned_speed_mps

    def reset(self):
        self._planned_speed_mps = None
        self._prev_lateral_error_m = None
        self._prev_heading_error_rad = None
        self._filtered_lateral_error_rate = 0.0
        self._filtered_heading_error_rate = 0.0
        self._hold_steps_remaining = 0
        self.last_risk = 0.0
        self.last_reason = "none"

    def _target_for_risk(self, risk):
        clipped_risk = float(np.clip(risk, 0.0, 1.0))
        recovery_floor = min(self.recovery_min_speed_mps, self.min_turn_speed_mps)
        return self.base_target_speed_mps - clipped_risk * (self.base_target_speed_mps - recovery_floor)

    def _risk_between(self, value, warning, critical):
        abs_value = abs(value)
        if abs_value <= warning:
            return 0.0
        denom = max(critical - warning, 1e-06)
        return float(np.clip((abs_value - warning) / denom, 0.0, 1.0))

    def _turn_speed_floor(self, max_abs_curvature):
        threshold = max(float(self.config.emergency_curvature_threshold), self.config.curvature_epsilon)
        full = max(float(self.config.emergency_full_cap_curvature), threshold + 1e-06)
        progress = float(np.clip((max_abs_curvature - threshold) / (full - threshold), 0.0, 1.0))
        emergency_floor = min(self.emergency_turn_speed_mps, self.min_turn_speed_mps)
        return self.min_turn_speed_mps - progress * (self.min_turn_speed_mps - emergency_floor)

    def _minimum_speed_floor(self, max_abs_curvature, reason):
        turn_floor = self._turn_speed_floor(max_abs_curvature)
        if reason in ('lateral_error', 'heading_error', 'lateral_error_rate', 'heading_error_rate',
                      'hold'):
            return min(turn_floor, self.recovery_min_speed_mps)
        return turn_floor

    def _raw_target_for_curvature(self, max_abs_curvature):
        if max_abs_curvature <= self.config.curvature_epsilon:
            return 0.0, self.base_target_speed_mps, "none"

        target = self.base_target_speed_mps
        reason = "none"
        turn_floor = self._turn_speed_floor(max_abs_curvature)

        if (
            max_abs_curvature >= self.config.entry_curvature_threshold
            and self.base_target_speed_mps > self.entry_max_speed_mps
        ):
            span = max(
                self.config.entry_full_cap_curvature - self.config.entry_curvature_threshold,
                1e-6,
            )
            entry_risk = float(np.clip(
                (max_abs_curvature - self.config.entry_curvature_threshold) / span,
                0.0,
                1.0,
            ))
            adaptive_entry_speed = self.base_target_speed_mps - entry_risk * (
                self.base_target_speed_mps - self.entry_max_speed_mps
            )
            entry_target = float(np.clip(
                adaptive_entry_speed,
                turn_floor,
                self.base_target_speed_mps,
            ))
            if entry_target < target:
                target = entry_target
                reason = "entry_curvature"

        if max_abs_curvature >= self.config.emergency_curvature_threshold:
            if turn_floor < target:
                target = turn_floor
                reason = "curvature"

        base_lateral_accel = self.base_target_speed_mps ** 2 * max_abs_curvature
        if base_lateral_accel > self.config.max_lateral_accel:
            lateral_limit = np.sqrt(self.config.max_lateral_accel / max_abs_curvature)
            lateral_target = float(np.clip(lateral_limit, turn_floor, self.base_target_speed_mps))
            if lateral_target < target:
                target = lateral_target
                reason = "curvature"
        if reason == "none":
            return 0.0, self.base_target_speed_mps, reason
        risk = self._risk_from_target(target)
        return (float(np.clip(risk, 0.0, 1.0)), target, reason)

    def _risk_from_target(self, target):
        return (self.base_target_speed_mps - target) / max(self.base_target_speed_mps - self.min_turn_speed_mps, 1e-06)

    def _raw_target_for_error(self, error_value, warning, critical):
        risk = self._risk_between(error_value, warning, critical)
        return (risk, self._target_for_risk(risk))

    def _combined_tracking_error_candidate(self, lateral_error_m, heading_error_rad):
        lateral_risk = self._risk_between(lateral_error_m, self.config.lateral_error_warning, self.config.lateral_error_critical)
        heading_risk = self._risk_between(heading_error_rad, self.config.heading_error_warning_rad, self.config.heading_error_critical_rad)
        if lateral_risk <= 0.0 or heading_risk <= 0.0:
            return
        combined_risk = float(np.clip(max(lateral_risk, heading_risk) + 0.25 * min(lateral_risk, heading_risk), 0.0, 1.0))
        reason = "lateral_error" if lateral_risk >= heading_risk else "heading_error"
        return (reason, combined_risk, self._target_for_risk(combined_risk))

    def _error_rate(self, current_value, previous_value, dt):
        if previous_value is None or dt <= 0.0:
            return 0.0
        return max((abs(current_value) - abs(previous_value)) / dt, 0.0)

    def _filtered_error_rate(self, raw_rate, previous_filtered_rate):
        alpha = float(np.clip(self.config.error_rate_filter_alpha, 0.0, 1.0))
        return (1.0 - alpha) * previous_filtered_rate + alpha * raw_rate

    def _active_error_rate(self, filtered_rate, error_value, activation_threshold):
        if abs(error_value) >= activation_threshold:
            return filtered_rate
        return 0.0

    def _tracking_error_candidates(self, lateral_error_m, heading_error_rad, lateral_error_rate, heading_error_rate):
        specs = (
         (
          "lateral_error",
          lateral_error_m,
          self.config.lateral_error_warning,
          self.config.lateral_error_critical),
         (
          "heading_error",
          heading_error_rad,
          self.config.heading_error_warning_rad,
          self.config.heading_error_critical_rad),
         (
          "lateral_error_rate",
          lateral_error_rate,
          self.config.lateral_error_rate_warning,
          self.config.lateral_error_rate_critical),
         (
          "heading_error_rate",
          heading_error_rate,
          self.config.heading_error_rate_warning_rad,
          self.config.heading_error_rate_critical_rad))
        return [(reason, *self._raw_target_for_error(value, warning, critical)) for reason, value, warning, critical in specs]

    def plan_speed_mps(self, curvatures, dt, lateral_error_m=0.0, heading_error_rad=0.0):
        curvature_values = np.asarray((list(curvatures)), dtype=float)
        if curvature_values.size == 0:
            max_abs_curvature = 0.0
        else:
            max_abs_curvature = float(np.max(np.abs(curvature_values)))
        raw_lateral_error_rate = self._error_rate(lateral_error_m, self._prev_lateral_error_m, dt)
        raw_heading_error_rate = self._error_rate(heading_error_rad, self._prev_heading_error_rad, dt)
        self._filtered_lateral_error_rate = self._filtered_error_rate(raw_lateral_error_rate, self._filtered_lateral_error_rate)
        self._filtered_heading_error_rate = self._filtered_error_rate(raw_heading_error_rate, self._filtered_heading_error_rate)
        self._prev_lateral_error_m = lateral_error_m
        self._prev_heading_error_rad = heading_error_rad
        lateral_error_rate = self._active_error_rate(self._filtered_lateral_error_rate, lateral_error_m, self.config.lateral_error_rate_activation)
        heading_error_rate = self._active_error_rate(self._filtered_heading_error_rate, heading_error_rad, self.config.heading_error_rate_activation_rad)
        curvature_risk, curvature_target, curvature_reason = self._raw_target_for_curvature(max_abs_curvature)
        risk_candidates = [
         
          (
           curvature_reason, curvature_risk, curvature_target),
         *self._tracking_error_candidates(lateral_error_m, heading_error_rad, lateral_error_rate, heading_error_rate)]
        combined_error_candidate = self._combined_tracking_error_candidate(lateral_error_m, heading_error_rad)
        if combined_error_candidate is not None:
            risk_candidates.append(combined_error_candidate)
        reason, risk, raw_target = min(risk_candidates, key=(lambda item: item[2]))
        if risk > 0.0:
            self._hold_steps_remaining = self.config.recovery_hold_steps
        elif self._hold_steps_remaining > 0 and self._planned_speed_mps is not None:
            self._hold_steps_remaining -= 1
            reason = "hold"
            raw_target = self.base_target_speed_mps
        else:
            reason = "none"

        speed_floor = self._minimum_speed_floor(max_abs_curvature, reason)
        if self._planned_speed_mps is None:
            self._planned_speed_mps = float(np.clip(raw_target, speed_floor, self.base_target_speed_mps))
            self.last_risk = risk
            self.last_reason = reason if risk > 0.0 else "none"
            return self._planned_speed_mps

        delta = raw_target - self._planned_speed_mps
        if delta >= 0.0:
            max_delta = self.config.max_accel * dt
        else:
            max_delta = self.config.max_decel * dt

        self._planned_speed_mps += float(np.clip(delta, -max_delta, max_delta))
        self._planned_speed_mps = float(np.clip(
            self._planned_speed_mps,
            speed_floor,
            self.base_target_speed_mps,
        ))
        self.last_risk = risk
        self.last_reason = reason if (risk > 0.0 or reason == "hold") else "none"
        return self._planned_speed_mps
