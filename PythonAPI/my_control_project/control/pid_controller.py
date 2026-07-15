import carla
import numpy as np
from agents.navigation.controller import PIDLateralController

from .base import BaseTrackingController
from .longitudinal import PidLongitudinalController


def _resolve_preview_curvature(curvature, blend):
    values = np.asarray(curvature, dtype=float).reshape(-1)
    if values.size == 0:
        return 0.0, 0.0

    current_curvature = float(values[0])
    if values.size == 1:
        return current_curvature, current_curvature

    weights = np.linspace(1.0, 0.40, values.size)
    preview_curvature = float(values[int(np.argmax(np.abs(values) * weights))])
    resolved_blend = float(np.clip(blend, 0.0, 1.0))
    feedforward_curvature = (
        (1.0 - resolved_blend) * current_curvature
        + resolved_blend * preview_curvature
    )
    return current_curvature, feedforward_curvature


class PidControllerAdapter(BaseTrackingController):
    """Project-level adapter around CARLA's built-in PID controller."""

    def __init__(
        self,
        vehicle,
        target_speed=30.0,
        lat_kp=0.72,
        lat_ki=0.005,
        lat_kd=0.25,
        long_kp=0.22,
        long_ki=0.01,
        long_kd=0.14,
        dt=0.05,
        max_throttle=0.45,
        max_brake=0.28,
        max_steering=0.65,
        max_steer_rate=0.65,
        curvature_feedforward_gain=0.0,
        curvature_preview_horizon=10,
        curvature_preview_blend=0.55,
        derivative_filter_alpha=1.0,
        integral_error_limit=np.radians(20.0),
        integral_separation_lateral_error=1.4,
        integral_separation_heading_error=np.radians(18.0),
        speed_scheduling_enabled=True,
        longitudinal_controller=None,
    ):
        self.max_steer = max_steering
        self.max_steer_rate = max_steer_rate
        self.curvature_feedforward_gain = curvature_feedforward_gain
        self.curvature_preview_blend = curvature_preview_blend
        self.derivative_filter_alpha = float(np.clip(derivative_filter_alpha, 0.0, 1.0))
        self.speed_scheduling_enabled = speed_scheduling_enabled
        self.supports_curvature_sequence = True
        self.horizon = max(int(curvature_preview_horizon), 1)
        self.L = 2.90
        self._lat_controller = PIDLateralController(
            vehicle,
            K_P=lat_kp,
            K_D=lat_kd,
            K_I=lat_ki,
            dt=dt,
        )
        self._longitudinal_controller = longitudinal_controller or PidLongitudinalController(
            target_speed_kmh=target_speed,
            kp=long_kp,
            ki=long_ki,
            kd=long_kd,
            dt=dt,
            max_throttle=max_throttle,
            max_brake=max_brake,
        )
        self.past_steering = 0.0
        self.last_speed_profile = "pid_base"
        self.last_curvature_feedforward_scale = 1.0
        self.last_raw_steer = 0.0
        self.last_steer_rate_limit = 0.0
        self.last_steer_rate_limited = False
        self.last_current_curvature = 0.0
        self.last_preview_curvature = 0.0
        self.last_raw_lateral_derivative = 0.0
        self.last_filtered_lateral_derivative = 0.0
        self.last_derivative_filter_applied = False
        self._filtered_lateral_derivative = 0.0

    def _lateral_error_buffer_snapshot(self):
        if not hasattr(self._lat_controller, "_e_buffer"):
            return None
        return tuple(self._lat_controller._e_buffer)

    def _restore_lateral_error_buffer(self, errors):
        if errors is None or not hasattr(self._lat_controller, "_e_buffer"):
            return
        self._lat_controller._e_buffer.clear()
        self._lat_controller._e_buffer.extend(errors)

    def _steer_rate_limit(self):
        return max(float(self.max_steer_rate), 0.0)

    def _filtered_lateral_feedback(self, fallback_steer):
        controller = self._lat_controller
        required_fields = ("_e_buffer", "_dt", "_k_p", "_k_i", "_k_d")
        if not all(hasattr(controller, field) for field in required_fields):
            self.last_raw_lateral_derivative = 0.0
            self.last_filtered_lateral_derivative = self._filtered_lateral_derivative
            self.last_derivative_filter_applied = False
            return fallback_steer

        errors = controller._e_buffer
        if not errors:
            self.last_raw_lateral_derivative = 0.0
            self.last_filtered_lateral_derivative = self._filtered_lateral_derivative
            self.last_derivative_filter_applied = False
            return fallback_steer

        dt = float(controller._dt)
        if len(errors) >= 2 and dt > 0.0:
            raw_derivative = (float(errors[-1]) - float(errors[-2])) / dt
            integral_error = sum(errors) * dt
        else:
            raw_derivative = 0.0
            integral_error = 0.0

        alpha = self.derivative_filter_alpha
        filtered_derivative = (
            (1.0 - alpha) * self._filtered_lateral_derivative
            + alpha * raw_derivative
        )
        self._filtered_lateral_derivative = float(filtered_derivative)
        self.last_raw_lateral_derivative = float(raw_derivative)
        self.last_filtered_lateral_derivative = float(filtered_derivative)

        if alpha >= 1.0:
            self.last_derivative_filter_applied = False
            return fallback_steer

        self.last_derivative_filter_applied = True
        error = float(errors[-1])
        return float(np.clip(
            controller._k_p * error
            + controller._k_d * filtered_derivative
            + controller._k_i * integral_error,
            -1.0,
            1.0,
        ))

    def run_step(
        self,
        vehicle,
        target_waypoint,
        curvature=0.0,
        reference_waypoint=None,
        planned_target_speed_mps=None,
        tracking_errors=None,
    ):
        velocity = vehicle.get_velocity()
        speed_mps = np.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        throttle, brake = self._longitudinal_controller.run_step(
            speed_mps,
            target_speed_mps=planned_target_speed_mps,
        )
        current_curvature, feedforward_curvature = _resolve_preview_curvature(
            curvature,
            self.curvature_preview_blend,
        )
        self.last_curvature_feedforward_scale = 1.0
        steer_ff = (
            self.curvature_feedforward_gain
            * np.arctan(self.L * feedforward_curvature)
        )
        buffer_before = self._lateral_error_buffer_snapshot()
        derivative_before = self._filtered_lateral_derivative
        steer_fb = self._lat_controller.run_step(target_waypoint)
        steer_fb = self._filtered_lateral_feedback(steer_fb)
        requested_steering = steer_fb + steer_ff

        steer_rate_limit = self._steer_rate_limit()
        current_steering = float(np.clip(
            requested_steering,
            self.past_steering - steer_rate_limit,
            self.past_steering + steer_rate_limit,
        ))

        if current_steering >= 0:
            steering = min(self.max_steer, current_steering)
        else:
            steering = max(-self.max_steer, current_steering)
        if buffer_before is not None and self._lat_controller._e_buffer:
            newest_error = float(self._lat_controller._e_buffer[-1])
            limiting_direction = float(requested_steering - steering)
            if abs(limiting_direction) > 1e-9 and newest_error * limiting_direction > 0.0:
                self._restore_lateral_error_buffer(buffer_before)
                self._filtered_lateral_derivative = derivative_before
                self.last_filtered_lateral_derivative = derivative_before

        self.last_raw_steer = float(requested_steering)
        self.last_steer_rate_limit = float(steer_rate_limit)
        self.last_steer_rate_limited = abs(steering - requested_steering) > 1e-9
        self.last_current_curvature = float(current_curvature)
        self.last_preview_curvature = float(feedforward_curvature)

        control = carla.VehicleControl()
        control.steer = steering
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = steering
        return control

    def reset(self):
        self.past_steering = 0.0
        self.last_speed_profile = "pid_base"
        self.last_curvature_feedforward_scale = 1.0
        self.last_raw_steer = 0.0
        self.last_steer_rate_limit = 0.0
        self.last_steer_rate_limited = False
        self.last_current_curvature = 0.0
        self.last_preview_curvature = 0.0
        self.last_raw_lateral_derivative = 0.0
        self.last_filtered_lateral_derivative = 0.0
        self.last_derivative_filter_applied = False
        self._filtered_lateral_derivative = 0.0
        self._longitudinal_controller.reset()
        if hasattr(self._lat_controller, "_e_buffer"):
            self._lat_controller._e_buffer.clear()
