from dataclasses import dataclass

import numpy as np


@dataclass
class SpeedPlannerConfig:
    base_target_speed_kmh: float
    min_turn_speed_kmh: float = 48.0
    max_lateral_accel: float = 18.0
    max_accel: float = 3.5
    max_decel: float = 9.0
    curvature_epsilon: float = 1e-4
    lateral_error_warning: float = 1.6
    lateral_error_critical: float = 2.6
    heading_error_warning_rad: float = np.radians(12.0)
    heading_error_critical_rad: float = np.radians(18.0)
    lateral_error_rate_warning: float = 0.8
    lateral_error_rate_critical: float = 1.6
    heading_error_rate_warning_rad: float = np.radians(8.0)
    heading_error_rate_critical_rad: float = np.radians(16.0)
    lateral_error_rate_activation: float = 0.8
    heading_error_rate_activation_rad: float = np.radians(8.0)
    error_rate_filter_alpha: float = 0.25
    recovery_hold_steps: int = 3
    entry_max_speed_kmh: float = 70.0
    entry_curvature_threshold: float = 0.015
    entry_full_cap_curvature: float = 0.03


class CurvatureSpeedPlanner:
    """Plans the fastest target speed allowed by previewed route curvature."""

    def __init__(self, config):
        self.config = config
        self.base_target_speed_mps = config.base_target_speed_kmh / 3.6
        self.min_turn_speed_mps = config.min_turn_speed_kmh / 3.6
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
        return self.base_target_speed_mps - clipped_risk * (self.base_target_speed_mps - self.min_turn_speed_mps)

    def _risk_between(self, value, warning, critical):
        abs_value = abs(value)
        if abs_value <= warning:
            return 0.0
        denom = max(critical - warning, 1e-6)
        return float(np.clip((abs_value - warning) / denom, 0.0, 1.0))

    def _raw_target_for_curvature(self, max_abs_curvature):
        if max_abs_curvature <= self.config.curvature_epsilon:
            return 0.0, self.base_target_speed_mps, "none"

        target = self.base_target_speed_mps
        reason = "none"

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
                self.min_turn_speed_mps,
                self.base_target_speed_mps,
            ))
            if entry_target < target:
                target = entry_target
                reason = "entry_curvature"

        base_lateral_accel = self.base_target_speed_mps ** 2 * max_abs_curvature
        if base_lateral_accel > self.config.max_lateral_accel:
            lateral_limit = np.sqrt(self.config.max_lateral_accel / max_abs_curvature)
            lateral_target = float(np.clip(lateral_limit, self.min_turn_speed_mps, self.base_target_speed_mps))
            if lateral_target < target:
                target = lateral_target
                reason = "curvature"

        if reason == "none":
            return 0.0, self.base_target_speed_mps, reason

        risk = self._risk_from_target(target)
        return float(np.clip(risk, 0.0, 1.0)), target, reason

    def _risk_from_target(self, target):
        return (self.base_target_speed_mps - target) / max(
            self.base_target_speed_mps - self.min_turn_speed_mps,
            1e-6,
        )

    def _raw_target_for_error(self, error_value, warning, critical):
        risk = self._risk_between(error_value, warning, critical)
        return risk, self._target_for_risk(risk)

    def _error_rate(self, current_value, previous_value, dt):
        if previous_value is None or dt <= 0.0:
            return 0.0
        return max((abs(current_value) - abs(previous_value)) / dt, 0.0)

    def _filtered_error_rate(self, raw_rate, previous_filtered_rate):
        alpha = float(np.clip(self.config.error_rate_filter_alpha, 0.0, 1.0))
        return (1.0 - alpha) * previous_filtered_rate + alpha * raw_rate

    def plan_speed_mps(self, curvatures, dt, lateral_error_m=0.0, heading_error_rad=0.0):
        curvature_values = np.asarray(list(curvatures), dtype=float)
        if curvature_values.size == 0:
            max_abs_curvature = 0.0
        else:
            max_abs_curvature = float(np.max(np.abs(curvature_values)))

        raw_lateral_error_rate = self._error_rate(lateral_error_m, self._prev_lateral_error_m, dt)
        raw_heading_error_rate = self._error_rate(heading_error_rad, self._prev_heading_error_rad, dt)
        self._filtered_lateral_error_rate = self._filtered_error_rate(
            raw_lateral_error_rate,
            self._filtered_lateral_error_rate,
        )
        self._filtered_heading_error_rate = self._filtered_error_rate(
            raw_heading_error_rate,
            self._filtered_heading_error_rate,
        )
        self._prev_lateral_error_m = lateral_error_m
        self._prev_heading_error_rad = heading_error_rad
        lateral_error_rate = (
            self._filtered_lateral_error_rate
            if abs(lateral_error_m) >= self.config.lateral_error_rate_activation
            else 0.0
        )
        heading_error_rate = (
            self._filtered_heading_error_rate
            if abs(heading_error_rad) >= self.config.heading_error_rate_activation_rad
            else 0.0
        )

        curvature_risk, curvature_target, curvature_reason = self._raw_target_for_curvature(max_abs_curvature)
        risk_candidates = [
            (curvature_reason, curvature_risk, curvature_target),
            (
                "lateral_error",
                *self._raw_target_for_error(
                    lateral_error_m,
                    self.config.lateral_error_warning,
                    self.config.lateral_error_critical,
                ),
            ),
            (
                "heading_error",
                *self._raw_target_for_error(
                    heading_error_rad,
                    self.config.heading_error_warning_rad,
                    self.config.heading_error_critical_rad,
                ),
            ),
            (
                "lateral_error_rate",
                *self._raw_target_for_error(
                    lateral_error_rate,
                    self.config.lateral_error_rate_warning,
                    self.config.lateral_error_rate_critical,
                ),
            ),
            (
                "heading_error_rate",
                *self._raw_target_for_error(
                    heading_error_rate,
                    self.config.heading_error_rate_warning_rad,
                    self.config.heading_error_rate_critical_rad,
                ),
            ),
        ]
        reason, risk, raw_target = min(risk_candidates, key=lambda item: item[2])
        if risk > 0.0:
            self._hold_steps_remaining = self.config.recovery_hold_steps
        elif self._hold_steps_remaining > 0 and self._planned_speed_mps is not None:
            self._hold_steps_remaining -= 1
            reason = "hold"
            raw_target = self.base_target_speed_mps
        else:
            reason = "none"

        if self._planned_speed_mps is None:
            self._planned_speed_mps = raw_target
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
            self.min_turn_speed_mps,
            self.base_target_speed_mps,
        ))
        self.last_risk = risk
        self.last_reason = reason if (risk > 0.0 or reason == "hold") else "none"
        return self._planned_speed_mps
