import carla
import numpy as np
from scipy import linalg
from scipy.signal import cont2discrete

from .base import BaseTrackingController
from .longitudinal import PidLongitudinalController
from road_planning.tracking_geometry import compute_path_tracking_errors


class LqrController(BaseTrackingController):
    """LQR lateral controller with longitudinal PID speed control."""

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
        derivative_alpha=0.20,
        curvature_alpha=0.50,
        feedforward_gain=1.0,
        turn_in_rate_scale=0.70,
        turn_in_guard_lateral_error=1.0,
        turn_in_guard_heading_error=np.radians(10.0),
        turn_in_guard_max_curvature=0.04,
        inside_error_feedforward_start=0.80,
        inside_error_feedforward_full=1.80,
        inside_error_feedforward_min_scale=0.65,
        inside_error_feedforward_heading_limit=np.radians(4.0),
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
        self.derivative_alpha = derivative_alpha
        self.curvature_alpha = curvature_alpha
        self.feedforward_gain = feedforward_gain
        self.turn_in_rate_scale = turn_in_rate_scale
        self.turn_in_guard_lateral_error = turn_in_guard_lateral_error
        self.turn_in_guard_heading_error = turn_in_guard_heading_error
        self.turn_in_guard_max_curvature = turn_in_guard_max_curvature
        self.inside_error_feedforward_start = inside_error_feedforward_start
        self.inside_error_feedforward_full = inside_error_feedforward_full
        self.inside_error_feedforward_min_scale = inside_error_feedforward_min_scale
        self.inside_error_feedforward_heading_limit = inside_error_feedforward_heading_limit
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
        self._prev_e_y = 0.0
        self._prev_e_psi = 0.0
        self._filtered_e_y_dot = 0.0
        self._filtered_e_psi_dot = 0.0
        self._has_prev_error = False
        self._gain_matrix = None
        self._last_gain_speed = -1.0
        self._longitudinal_controller.reset()

    def _update_lqr_gain(self, speed_mps):
        if self._gain_matrix is not None and abs(speed_mps - self._last_gain_speed) <= 0.5:
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

    def _steer_rate_limit(self, steer_change, e_y, e_psi, curvature):
        rate_limit = self.max_steer_rate
        adding_turn_in = abs(curvature) > 1e-4 and steer_change * curvature > 0.0
        near_centerline = (
            abs(e_y) <= self.turn_in_guard_lateral_error
            and abs(e_psi) <= self.turn_in_guard_heading_error
        )
        guard_curvature = max(float(self.turn_in_guard_max_curvature), 0.0)
        moderate_curve = abs(curvature) <= guard_curvature
        if adding_turn_in and near_centerline and moderate_curve:
            rate_limit *= float(np.clip(self.turn_in_rate_scale, 0.0, 1.0))
        return max(rate_limit, 1e-6)

    def _curvature_feedforward_scale(self, e_y, e_psi, curvature):
        if abs(curvature) <= 1e-4 or e_y * curvature <= 0.0:
            return 1.0
        if abs(e_psi) > self.inside_error_feedforward_heading_limit:
            return 1.0

        start = max(float(self.inside_error_feedforward_start), 0.0)
        full = max(float(self.inside_error_feedforward_full), start + 1e-6)
        min_scale = float(np.clip(self.inside_error_feedforward_min_scale, 0.0, 1.0))
        if abs(e_y) <= start:
            return 1.0

        progress = np.clip((abs(e_y) - start) / (full - start), 0.0, 1.0)
        return 1.0 - progress * (1.0 - min_scale)

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

        dt = 0.05
        if self._has_prev_error:
            raw_e_y_dot, raw_e_psi_dot = self._estimate_error_rates(
                vehicle,
                tracking_errors,
                curvature,
                speed_mps,
                dt,
            )
        else:
            raw_e_y_dot = 0.0
            raw_e_psi_dot = 0.0
            self._has_prev_error = True

        alpha = self.derivative_alpha
        self._filtered_e_y_dot = (1.0 - alpha) * self._filtered_e_y_dot + alpha * raw_e_y_dot
        self._filtered_e_psi_dot = (1.0 - alpha) * self._filtered_e_psi_dot + alpha * raw_e_psi_dot
        e_y_dot = self._filtered_e_y_dot
        e_psi_dot = self._filtered_e_psi_dot
        self._prev_e_y = e_y
        self._prev_e_psi = e_psi

        curvature_alpha = self.curvature_alpha
        self._filtered_curvature = (
            (1.0 - curvature_alpha) * self._filtered_curvature + curvature_alpha * curvature
        )
        feedforward_scale = self._curvature_feedforward_scale(e_y, e_psi, self._filtered_curvature)
        steer_ff = (
            self.feedforward_gain
            * feedforward_scale
            * np.arctan(self.L * self._filtered_curvature)
        )

        self._update_lqr_gain(speed_mps)
        state = np.array([e_y, e_y_dot, e_psi, e_psi_dot])
        steer_fb = float(np.asarray(-self._gain_matrix @ state).item())
        steer = steer_ff + steer_fb

        steer = np.clip(steer, -self.max_steer, self.max_steer)
        raw_steer_change = steer - self._prev_steer
        steer_rate_limit = self._steer_rate_limit(raw_steer_change, e_y, e_psi, curvature)
        steer_change = np.clip(raw_steer_change, -steer_rate_limit, steer_rate_limit)
        steer = self._prev_steer + steer_change
        self._prev_steer = steer

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
