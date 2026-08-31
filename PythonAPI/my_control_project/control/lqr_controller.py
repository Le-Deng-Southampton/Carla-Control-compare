import carla
import numpy as np
from scipy import linalg
from scipy.signal import cont2discrete

from .base import BaseTrackingController
from .longitudinal import PidLongitudinalController
from road_planning.tracking_geometry import compute_path_tracking_errors


def _resolve_preview_curvature(curvature, blend):
    values = np.asarray(curvature, dtype=float).reshape(-1)
    if values.size == 0:
        return 0.0, 0.0
    current = float(values[0])
    if values.size == 1:
        return current, current
    weights = np.linspace(1.0, 0.40, values.size)
    preview = float(values[int(np.argmax(np.abs(values) * weights))])
    blend = float(np.clip(blend, 0.0, 1.0))
    return current, (1.0 - blend) * current + blend * preview


class LqrController(BaseTrackingController):
    """LQR lateral controller with longitudinal PID speed control."""

    supports_curvature_sequence = True

    def __init__(
        self,
        target_speed=30.0,
        q_weights=(2.6, 1.10, 6.0, 2.6),
        r_weight=8.0,
        kp_long=0.6,
        ki_long=0.01,
        kd_long=0.06,
        max_steer=0.55,
        max_steer_rate=0.16,
        curvature_alpha=0.50,
        curvature_preview_horizon=12,
        curvature_preview_blend=0.40,
        max_lateral_accel=0.0,
        min_dynamic_steer_limit=0.12,
        longitudinal_controller=None,
    ):
        print(">>> USING VEHICLE-GRADE LQR (DYNAMIC BICYCLE MODEL) <<<")

        self.Lf = 1.45
        self.Lr = 1.45
        self.L = self.Lf + self.Lr

        self.m = 1750.0
        self.Iz = 2875.0
        self.Cf = 19000.0
        self.Cr = 33000.0

        self.Q = np.diag(q_weights)
        self.R = np.array([[r_weight]])
        self.max_steer = max_steer
        self.max_steer_rate = max_steer_rate
        self.curvature_alpha = curvature_alpha
        self.horizon = max(int(curvature_preview_horizon), 1)
        self.dt = 0.05
        self.curvature_preview_blend = float(np.clip(curvature_preview_blend, 0.0, 1.0))
        self.max_lateral_accel = max(float(max_lateral_accel), 0.0)
        self.min_dynamic_steer_limit = max(float(min_dynamic_steer_limit), 0.0)
        self._longitudinal_controller = longitudinal_controller or PidLongitudinalController(
            target_speed_kmh=target_speed,
            kp=kp_long,
            ki=ki_long,
            kd=kd_long,
        )

        self.reset()

    def reset(self):
        self._prev_steer = 0.0
        self._filtered_curvature = 0.0
        self._prev_e_psi = 0.0
        self._has_prev_error = False
        self._gain_matrix = None
        self._last_gain_speed = -1.0
        self.last_raw_steer = 0.0
        self.last_steer_rate_limit = 0.0
        self.last_steer_rate_limited = False
        self.last_current_curvature = 0.0
        self.last_preview_curvature = 0.0
        self.last_dynamic_steer_limit = self.max_steer
        self.last_gain_update_reason = "reset"
        self._longitudinal_controller.reset()

    def _update_lqr_gain(self, speed_mps):
        if self._gain_matrix is not None and abs(speed_mps - self._last_gain_speed) <= 0.5:
            self.last_gain_update_reason = "cached_speed_band"
            return

        dt = 0.05
        a_cont = np.array([
            [0, 1, speed_mps, 0],
            [
                0,
                -(2 * self.Cf + 2 * self.Cr) / (self.m * speed_mps),
                0,
                -(2 * self.Cf * self.Lf - 2 * self.Cr * self.Lr) / (self.m * speed_mps) - speed_mps,
            ],
            [0, 0, 0, 1],
            [
                0,
                -(2 * self.Cf * self.Lf - 2 * self.Cr * self.Lr) / (self.Iz * speed_mps),
                0,
                -(2 * self.Cf * self.Lf ** 2 + 2 * self.Cr * self.Lr ** 2) / (self.Iz * speed_mps),
            ],
        ])
        b_cont = np.array([
            [0],
            [2 * self.Cf / self.m],
            [0],
            [2 * self.Cf * self.Lf / self.Iz],
        ])

        zeros_c = np.zeros((4, 4))
        zeros_d = np.zeros((4, 1))
        a_disc, b_disc, _, _, _ = cont2discrete((a_cont, b_cont, zeros_c, zeros_d), dt)

        p = linalg.solve_discrete_are(a_disc, b_disc, self.Q, self.R)
        self._gain_matrix = np.linalg.inv(b_disc.T @ p @ b_disc + self.R) @ (b_disc.T @ p @ a_disc)
        self._last_gain_speed = speed_mps
        self.last_gain_update_reason = "speed_change"

    def _dynamic_steer_limit(self, speed_mps):
        if self.max_lateral_accel <= 0.0:
            return float(self.max_steer)
        physical_limit = np.arctan(
            self.L * self.max_lateral_accel / max(float(speed_mps) ** 2, 1e-6)
        )
        return float(np.clip(
            max(self.min_dynamic_steer_limit, physical_limit),
            0.0,
            self.max_steer,
        ))

    def _estimate_error_rates(self, vehicle, tracking_errors, curvature, speed_mps, dt):
        velocity = vehicle.get_velocity()
        reference_yaw = tracking_errors["reference_yaw"]
        e_y_dot = -velocity.x * np.sin(reference_yaw) + velocity.y * np.cos(reference_yaw)

        angular_velocity = getattr(vehicle, "get_angular_velocity", None)
        if angular_velocity is not None:
            yaw_rate = np.radians(angular_velocity().z)
            e_psi_dot = yaw_rate - speed_mps * curvature
        elif self._has_prev_error:
            e_psi_dot = (tracking_errors["e_psi"] - self._prev_e_psi) / dt
        else:
            e_psi_dot = 0.0

        return e_y_dot, e_psi_dot

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
        speed_mps = max(np.sqrt(velocity.x ** 2 + velocity.y ** 2), 1.0)
        reference_waypoint = reference_waypoint or target_waypoint
        if tracking_errors is None:
            tracking_errors = compute_path_tracking_errors(vehicle, reference_waypoint)
        e_y = tracking_errors["e_y"]
        e_psi = tracking_errors["e_psi"]
        current_curvature, preview_curvature = _resolve_preview_curvature(
            curvature,
            self.curvature_preview_blend,
        )

        curvature_alpha = self.curvature_alpha
        self._filtered_curvature = (
            (1.0 - curvature_alpha) * self._filtered_curvature
            + curvature_alpha * preview_curvature
        )

        dt = 0.05
        if self._has_prev_error:
            raw_e_y_dot, raw_e_psi_dot = self._estimate_error_rates(
                vehicle,
                tracking_errors,
                self._filtered_curvature,
                speed_mps,
                dt,
            )
        else:
            raw_e_y_dot = 0.0
            raw_e_psi_dot = 0.0
            self._has_prev_error = True

        e_y_dot = raw_e_y_dot
        e_psi_dot = raw_e_psi_dot
        self._prev_e_psi = e_psi

        self._update_lqr_gain(speed_mps)
        state = np.array([e_y, e_y_dot, e_psi, e_psi_dot])
        steer = float(np.asarray(-self._gain_matrix @ state).item())

        steer_limit = self._dynamic_steer_limit(speed_mps)
        raw_steer = float(steer)
        steer = np.clip(steer, -steer_limit, steer_limit)
        raw_steer_change = steer - self._prev_steer
        steer_rate_limit = max(float(self.max_steer_rate), 1e-6)
        steer_change = np.clip(raw_steer_change, -steer_rate_limit, steer_rate_limit)
        steer = self._prev_steer + steer_change
        self._prev_steer = steer
        self.last_raw_steer = raw_steer
        self.last_steer_rate_limit = float(steer_rate_limit)
        self.last_steer_rate_limited = abs(steer - raw_steer) > 1e-9
        self.last_current_curvature = float(current_curvature)
        self.last_preview_curvature = float(self._filtered_curvature)
        self.last_dynamic_steer_limit = float(steer_limit)

        throttle, brake = self._longitudinal_controller.run_step(
            speed_mps,
            target_speed_mps=planned_target_speed_mps,
        )

        control = carla.VehicleControl()
        control.steer = steer
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False
        control.manual_gear_shift = False
        return control
