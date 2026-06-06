import carla
import numpy as np

from .base import BaseTrackingController
from .longitudinal import PidLongitudinalController
from road_planning.tracking_geometry import compute_path_tracking_errors


class MpcController(BaseTrackingController):
    """Linear MPC lateral controller with explicit curvature preview."""

    def __init__(
        self,
        target_speed=30.0,
        horizon=8,
        dt=0.05,
        q_y=6.0,
        q_psi=8.0,
        r_steer=1.5,
        r_steer_rate=4.0,
        kp_long=0.6,
        ki_long=0.01,
        kd_long=0.06,
        max_steer=0.6,
        max_steer_rate=0.20,
        longitudinal_controller=None,
    ):
        self.Lf = 1.45
        self.Lr = 1.45
        self.L = self.Lf + self.Lr
        self.horizon = horizon
        self.dt = dt
        self.q_y = q_y
        self.q_psi = q_psi
        self.r_steer = r_steer
        self.r_steer_rate = r_steer_rate
        self.max_steer = max_steer
        self.max_steer_rate = max_steer_rate
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
        self._longitudinal_controller.reset()

    def _build_single_step_model(self, speed_mps):
        a_matrix = np.array([
            [1.0, self.dt * speed_mps],
            [0.0, 1.0],
        ])
        b_matrix = np.array([
            [0.0],
            [self.dt * speed_mps / self.L],
        ])
        curvature_matrix = np.array([
            0.0,
            -self.dt * speed_mps,
        ]).reshape(2, 1)
        return a_matrix, b_matrix, curvature_matrix

    def _build_prediction_matrices(self, speed_mps):
        a_matrix, b_matrix, curvature_matrix = self._build_single_step_model(speed_mps)

        state_dim = a_matrix.shape[0]
        input_dim = b_matrix.shape[1]
        phi = np.zeros((self.horizon * state_dim, state_dim))
        gamma = np.zeros((self.horizon * state_dim, self.horizon * input_dim))
        theta = np.zeros((self.horizon * state_dim, self.horizon))

        a_power = np.eye(state_dim)
        for step in range(self.horizon):
            a_power = a_power @ a_matrix
            row = slice(step * state_dim, (step + 1) * state_dim)
            phi[row, :] = a_power

            for control_step in range(step + 1):
                col = slice(control_step * input_dim, (control_step + 1) * input_dim)
                gamma[row, col] = np.linalg.matrix_power(a_matrix, step - control_step) @ b_matrix
                theta[row, control_step] = (
                    np.linalg.matrix_power(a_matrix, step - control_step) @ curvature_matrix
                ).ravel()

        return phi, gamma, theta

    def _build_steer_rate_matrix(self):
        diff_matrix = np.eye(self.horizon)
        for idx in range(1, self.horizon):
            diff_matrix[idx, idx - 1] = -1.0
        return diff_matrix

    def _build_curvature_sequence(self, curvature):
        if np.isscalar(curvature):
            return np.full(self.horizon, float(curvature))
        sequence = np.asarray(curvature, dtype=float).reshape(-1)
        if len(sequence) >= self.horizon:
            return sequence[: self.horizon]
        if len(sequence) == 0:
            return np.zeros(self.horizon)
        return np.pad(sequence, (0, self.horizon - len(sequence)), mode="edge")

    def _build_reference_steer_sequence(self, curvature_sequence):
        return np.clip(
            np.arctan(self.L * curvature_sequence),
            -self.max_steer,
            self.max_steer,
        )

    def _solve_mpc_steering(self, initial_state, speed_mps, curvature):
        phi, gamma, theta = self._build_prediction_matrices(speed_mps)
        curvature_sequence = self._build_curvature_sequence(curvature)
        reference_steer = self._build_reference_steer_sequence(curvature_sequence)
        q_block = np.diag([self.q_y, self.q_psi])
        q_bar = np.kron(np.eye(self.horizon), q_block)
        r_bar = self.r_steer * np.eye(self.horizon)

        diff_matrix = self._build_steer_rate_matrix()
        prev_term = np.zeros(self.horizon)
        prev_term[0] = self._prev_steer

        base_state = phi @ initial_state + theta @ curvature_sequence
        hessian = gamma.T @ q_bar @ gamma + r_bar + self.r_steer_rate * (diff_matrix.T @ diff_matrix)
        gradient = (
            gamma.T @ q_bar @ base_state
            - r_bar @ reference_steer
            - self.r_steer_rate * (diff_matrix.T @ prev_term)
        )

        regularized_hessian = hessian + 1e-6 * np.eye(hessian.shape[0])
        steer_sequence = -np.linalg.solve(regularized_hessian, gradient)
        return float(steer_sequence[0])

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
        initial_state = np.array([e_y, e_psi])

        steer = self._solve_mpc_steering(initial_state, speed_mps, curvature)
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        steer_delta = np.clip(steer - self._prev_steer, -self.max_steer_rate, self.max_steer_rate)
        steer = self._prev_steer + steer_delta
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
