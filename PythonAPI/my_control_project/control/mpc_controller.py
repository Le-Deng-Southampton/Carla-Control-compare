import carla
import numpy as np
from scipy.signal import cont2discrete

from .base import BaseTrackingController
from .longitudinal import PidLongitudinalController
from road_planning.tracking_geometry import compute_path_tracking_errors


class MpcController(BaseTrackingController):
    """Dynamic-bicycle MPC with conditioned curvature preview."""

    supports_curvature_sequence = True

    def __init__(
        self,
        target_speed=30.0,
        horizon=16,
        dt=0.05,
        q_y=12.0,
        q_psi=18.0,
        r_steer=0.8,
        r_steer_rate=0.9,
        kp_long=0.6,
        ki_long=0.01,
        kd_long=0.06,
        max_steer=0.65,
        max_steer_rate=0.30,
        curvature_step_limit=0.0,
        max_lateral_accel=0.0,
        min_dynamic_steer_limit=0.24,
        cornering_stiffness_scale=1.0,
        derivative_alpha=0.25,
        curvature_filter_alpha=1.0,
        debug_stability=False,
        longitudinal_controller=None,
    ):
        self.Lf = 1.45
        self.Lr = 1.45
        self.L = self.Lf + self.Lr
        self.m = 1750.0
        self.Iz = 2875.0
        stiffness_scale = max(float(cornering_stiffness_scale), 1e-6)
        self.Cf = 19000.0 * stiffness_scale
        self.Cr = 33000.0 * stiffness_scale

        self.horizon = max(int(horizon), 1)
        self.dt = float(dt)
        self.q_y = float(q_y)
        self.q_psi = float(q_psi)
        self.r_steer = float(r_steer)
        self.r_steer_rate = float(r_steer_rate)
        self.max_steer = max(float(max_steer), 0.0)
        self.max_steer_rate = max(float(max_steer_rate), 0.0)
        self.curvature_step_limit = max(float(curvature_step_limit), 0.0)
        self.max_lateral_accel = max(float(max_lateral_accel), 0.0)
        self.min_dynamic_steer_limit = max(float(min_dynamic_steer_limit), 0.0)
        self.cornering_stiffness_scale = stiffness_scale
        self.model_type = "dynamic_bicycle"
        self.derivative_alpha = float(np.clip(derivative_alpha, 0.0, 1.0))
        self.curvature_filter_alpha = float(np.clip(curvature_filter_alpha, 0.0, 1.0))
        self.debug_stability = bool(debug_stability)
        self._longitudinal_controller = longitudinal_controller or PidLongitudinalController(
            target_speed_kmh=target_speed,
            kp=kp_long,
            ki=ki_long,
            kd=kd_long,
            dt=dt,
        )
        self.reset()

    def reset(self):
        self._prev_steer = 0.0
        self._filtered_curvature_sequence = None
        self._filtered_e_y_dot = 0.0
        self._filtered_yaw_rate = 0.0
        self._has_dynamic_state = False
        self.last_speed_profile = "mpc_base"
        self.last_raw_steer = 0.0
        self.last_steer_rate_limit = self.max_steer_rate
        self.last_steer_rate_limited = False
        self.last_current_curvature = 0.0
        self.last_preview_curvature = 0.0
        self.last_mpc_solve_mode = "reset"
        self.last_mpc_curvature0 = 0.0
        self.last_mpc_curvature_max = 0.0
        self.last_mpc_active_horizon = self.horizon
        self.last_mpc_steer_limit = self.max_steer
        self._longitudinal_controller.reset()

    def _prediction_horizon(self):
        return self.horizon

    def _build_dynamic_bicycle_step_model(self, speed_mps):
        speed_mps = max(float(speed_mps), 1.0)
        a_cont = np.array([
            [0.0, 1.0, speed_mps, 0.0],
            [
                0.0,
                -(2.0 * self.Cf + 2.0 * self.Cr) / (self.m * speed_mps),
                0.0,
                -(2.0 * self.Cf * self.Lf - 2.0 * self.Cr * self.Lr)
                / (self.m * speed_mps)
                - speed_mps,
            ],
            [0.0, 0.0, 0.0, 1.0],
            [
                0.0,
                -(2.0 * self.Cf * self.Lf - 2.0 * self.Cr * self.Lr)
                / (self.Iz * speed_mps),
                0.0,
                -(2.0 * self.Cf * self.Lf ** 2 + 2.0 * self.Cr * self.Lr ** 2)
                / (self.Iz * speed_mps),
            ],
        ])
        b_steer = np.array([
            [0.0],
            [2.0 * self.Cf / self.m],
            [0.0],
            [2.0 * self.Cf * self.Lf / self.Iz],
        ])
        b_curvature = np.array([[0.0], [0.0], [-speed_mps], [0.0]])
        b_cont = np.hstack((b_steer, b_curvature))
        zeros_c = np.zeros((4, 4))
        zeros_d = np.zeros((4, 2))
        a_disc, b_disc, _, _, _ = cont2discrete(
            (a_cont, b_cont, zeros_c, zeros_d), self.dt
        )
        return a_disc, b_disc[:, [0]], b_disc[:, [1]]

    def _build_prediction_matrices(self, speed_mps):
        a_matrix, b_matrix, curvature_matrix = self._build_dynamic_bicycle_step_model(
            speed_mps
        )
        state_dim = a_matrix.shape[0]
        horizon = self._prediction_horizon()
        phi = np.zeros((horizon * state_dim, state_dim))
        gamma = np.zeros((horizon * state_dim, horizon))
        theta = np.zeros((horizon * state_dim, horizon))
        a_power = np.eye(state_dim)
        for step in range(horizon):
            a_power = a_power @ a_matrix
            row = slice(step * state_dim, (step + 1) * state_dim)
            phi[row, :] = a_power
            for control_step in range(step + 1):
                transition = np.linalg.matrix_power(a_matrix, step - control_step)
                gamma[row, control_step] = (transition @ b_matrix).ravel()
                theta[row, control_step] = (transition @ curvature_matrix).ravel()
        return phi, gamma, theta

    def _build_steer_rate_matrix(self):
        horizon = self._prediction_horizon()
        matrix = np.eye(horizon)
        for idx in range(1, horizon):
            matrix[idx, idx - 1] = -1.0
        return matrix

    def _build_curvature_sequence(self, curvature):
        horizon = self._prediction_horizon()
        if np.isscalar(curvature):
            return np.full(horizon, float(curvature))
        sequence = np.asarray(curvature, dtype=float).reshape(-1)
        if sequence.size == 0:
            return np.zeros(horizon)
        if sequence.size >= horizon:
            return sequence[:horizon]
        return np.pad(sequence, (0, horizon - sequence.size), mode="edge")

    def _condition_curvature_sequence(self, curvature):
        raw = self._build_curvature_sequence(curvature)
        if self._filtered_curvature_sequence is None or (
            self._filtered_curvature_sequence.size != raw.size
        ):
            filtered = raw.copy()
        else:
            alpha = self.curvature_filter_alpha
            filtered = (1.0 - alpha) * self._filtered_curvature_sequence + alpha * raw
        conditioned = filtered.copy()
        if self.curvature_step_limit > 0.0:
            for idx in range(1, conditioned.size):
                delta = np.clip(
                    conditioned[idx] - conditioned[idx - 1],
                    -self.curvature_step_limit,
                    self.curvature_step_limit,
                )
                conditioned[idx] = conditioned[idx - 1] + delta
        self._filtered_curvature_sequence = conditioned.copy()
        return conditioned

    def _dynamic_steer_limit(self, speed_mps):
        if self.max_lateral_accel <= 0.0:
            return self.max_steer
        physical_limit = np.arctan(
            self.L * self.max_lateral_accel / max(float(speed_mps) ** 2, 1e-6)
        )
        return float(np.clip(
            max(self.min_dynamic_steer_limit, physical_limit),
            0.0,
            self.max_steer,
        ))

    def _state_weight_block(self, state_dim):
        if state_dim == 4:
            return np.diag([self.q_y, 0.15 * self.q_y, self.q_psi, 0.10 * self.q_psi])
        return np.diag([self.q_y, self.q_psi])

    def _project_sequence(self, sequence, steer_limit):
        projected = np.asarray(sequence, dtype=float).reshape(-1).copy()
        for idx in range(projected.size):
            previous = self._prev_steer if idx == 0 else projected[idx - 1]
            projected[idx] = np.clip(
                projected[idx],
                previous - self.max_steer_rate,
                previous + self.max_steer_rate,
            )
            projected[idx] = np.clip(projected[idx], -steer_limit, steer_limit)
        return projected

    def _solve_mpc_steering(self, initial_state, speed_mps, curvature):
        initial_state = np.asarray(initial_state, dtype=float).reshape(-1)
        curvature_sequence = self._condition_curvature_sequence(curvature)

        phi, gamma, theta = self._build_prediction_matrices(speed_mps)
        steer_limit = self._dynamic_steer_limit(speed_mps)
        horizon = self._prediction_horizon()
        q_bar = np.kron(
            np.eye(horizon), self._state_weight_block(initial_state.size)
        )
        r_bar = self.r_steer * np.eye(horizon)
        diff_matrix = self._build_steer_rate_matrix()
        prev_term = np.zeros(horizon)
        prev_term[0] = self._prev_steer
        base_state = phi @ initial_state + theta @ curvature_sequence
        hessian = (
            gamma.T @ q_bar @ gamma
            + r_bar
            + self.r_steer_rate * (diff_matrix.T @ diff_matrix)
        )
        gradient = (
            gamma.T @ q_bar @ base_state
            - self.r_steer_rate * (diff_matrix.T @ prev_term)
        )
        regularized_hessian = hessian + 1e-6 * np.eye(horizon)
        steer_sequence = -np.linalg.solve(regularized_hessian, gradient)
        steer_sequence = self._project_sequence(steer_sequence, steer_limit)

        self.last_mpc_solve_mode = "fresh"
        self.last_mpc_curvature0 = float(curvature_sequence[0])
        self.last_mpc_curvature_max = float(np.max(np.abs(curvature_sequence)))
        self.last_mpc_active_horizon = horizon
        self.last_mpc_steer_limit = float(steer_limit)
        return float(steer_sequence[0])

    def _estimate_dynamic_state(self, vehicle, tracking_errors):
        velocity = vehicle.get_velocity()
        reference_yaw = float(tracking_errors.get("reference_yaw", 0.0))
        raw_e_y_dot = -velocity.x * np.sin(reference_yaw) + velocity.y * np.cos(reference_yaw)
        angular_velocity = getattr(vehicle, "get_angular_velocity", None)
        raw_yaw_rate = 0.0
        if angular_velocity is not None:
            raw_yaw_rate = np.radians(float(angular_velocity().z))
        alpha = self.derivative_alpha
        if not self._has_dynamic_state:
            self._filtered_e_y_dot = raw_e_y_dot
            self._filtered_yaw_rate = raw_yaw_rate
            self._has_dynamic_state = True
        else:
            self._filtered_e_y_dot = (
                (1.0 - alpha) * self._filtered_e_y_dot + alpha * raw_e_y_dot
            )
            self._filtered_yaw_rate = (
                (1.0 - alpha) * self._filtered_yaw_rate + alpha * raw_yaw_rate
            )
        return np.array([
            tracking_errors["e_y"],
            self._filtered_e_y_dot,
            tracking_errors["e_psi"],
            self._filtered_yaw_rate,
        ])

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
        initial_state = self._estimate_dynamic_state(vehicle, tracking_errors)

        raw_steer = self._solve_mpc_steering(initial_state, speed_mps, curvature)
        curvature_values = np.asarray(curvature, dtype=float).reshape(-1)
        current_curvature = float(curvature_values[0]) if curvature_values.size else 0.0
        steer = raw_steer
        steer_limit = self._dynamic_steer_limit(speed_mps)
        steer = float(np.clip(steer, -steer_limit, steer_limit))
        steer_delta = float(np.clip(
            steer - self._prev_steer,
            -self.max_steer_rate,
            self.max_steer_rate,
        ))
        steer = float(np.clip(
            self._prev_steer + steer_delta,
            -steer_limit,
            steer_limit,
        ))
        self.last_raw_steer = float(raw_steer)
        self.last_steer_rate_limit = self.max_steer_rate
        self.last_steer_rate_limited = abs(steer - raw_steer) > 1e-9
        self.last_current_curvature = current_curvature
        self.last_preview_curvature = self.last_mpc_curvature_max
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
        if self.debug_stability:
            print(
                "MPC mode={} model={} horizon={} steer={:+.4f} limit={:.4f} curvature={:.5f}".format(
                    self.last_mpc_solve_mode,
                    self.model_type,
                    self.last_mpc_active_horizon,
                    steer,
                    steer_limit,
                    current_curvature,
                )
            )
        return control
