# uncompyle6 version 3.9.3
# Python bytecode version base 3.7.0 (3394)
# Decompiled from: Python 3.13.13 | packaged by Anaconda, Inc. | (main, Apr 14 2026, 06:12:50) [MSC v.1942 64 bit (AMD64)]
# Embedded file name: D:\WindowsNoEditor\PythonAPI\my_control_project\control\initial_baseline.py
# Compiled at: 2026-07-14 02:57:29
# Size of source mod 2**32: 14061 bytes
"""Executable mixed-provenance historical controller baseline.

PID and LQR reproduce the user's recovered pre-Git source. MPC uses the oldest
complete four-state dynamic-bicycle source snapshot recoverable from the Codex
session history, with the parameters recorded by the first dynamic-MPC run.
Only interface adaptation needed by the current experiment harness is
permitted here.
"""
import carla, numpy as np
from agents.navigation.controller import VehiclePIDController
from scipy import linalg
from .base import BaseTrackingController
from .longitudinal import PidLongitudinalController
from .recovered_four_state_mpc import MpcController as RecoveredFourStateMpcController
from road_planning.tracking_geometry import compute_path_tracking_errors
BASELINE_IMPLEMENTATION = "historical_recovered_four_state_mpc"
PID_LQR_PROVENANCE = "recovered_pre_git_source"
MPC_SOURCE_SNAPSHOT_SHA256 = "6c943e7546924ad2173ea5c8f763e9c1765cb5d14c4cb9038ea8219f9032bb67"
MPC_SOURCE_SESSION = "019f0232-102f-7b90-bad5-4b91b3471aae"
MPC_FIRST_RECORDED_RUN = "run_20260615_150704_seed_221574906_straight"

class InitialPidController(BaseTrackingController):
    __doc__ = "CARLA VehiclePIDController configured by the recovered runner."

    def __init__(self, vehicle, target_speed=30.0, lat_kp=0.72, lat_ki=0.005, lat_kd=0.38, long_kp=0.22, long_ki=0.01, long_kd=0.14, dt=0.05, max_throttle=0.45, max_brake=0.28, max_steering=0.65):
        self.target_speed_kmh = float(target_speed)
        self._pid_controller = VehiclePIDController(vehicle,
          args_lateral={
         'K_P': lat_kp, 'K_D': lat_kd, 'K_I': lat_ki, 'dt': dt},
          args_longitudinal={
         'K_P': long_kp, 'K_D': long_kd, 'K_I': long_ki, 'dt': dt},
          max_throttle=max_throttle,
          max_brake=max_brake,
          max_steering=max_steering)

    def run_step(self, vehicle, target_waypoint, curvature=0.0, reference_waypoint=None, planned_target_speed_mps=None, tracking_errors=None):
        target_speed_kmh = self.target_speed_kmh
        if planned_target_speed_mps is not None:
            target_speed_kmh = float(planned_target_speed_mps) * 3.6
        return self._pid_controller.run_step(target_speed_kmh, target_waypoint)

    def reset(self):
        self._pid_controller.past_steering = 0.0
        if hasattr(self._pid_controller._lat_controller, "_e_buffer"):
            self._pid_controller._lat_controller._e_buffer.clear()
        if hasattr(self._pid_controller._lon_controller, "_error_buffer"):
            self._pid_controller._lon_controller._error_buffer.clear()


class InitialLqrController(BaseTrackingController):
    __doc__ = "Recovered three-state integral LQR with zero curvature feedforward."

    def __init__(self, target_speed=30.0, q_weights=(2.2, 1.1, 0.04), r_weight=8.0, kp_long=0.35, ki_long=0.02, kd_long=0.16, max_steer=0.65, max_steer_rate=0.045, longitudinal_controller=None):
        self.L = 2.9
        self.Q = np.diag(q_weights)
        self.R = np.array([[r_weight]])
        self.max_steer = max_steer
        self.max_steer_rate = max_steer_rate
        self._longitudinal_controller = longitudinal_controller or PidLongitudinalController(target_speed_kmh=target_speed,
          kp=kp_long,
          ki=ki_long,
          kd=kd_long,
          dt=0.05,
          integral_limit=5.0)
        self.reset()

    def reset(self):
        self._prev_steer = 0.0
        self._integral_ey = 0.0
        self._longitudinal_controller.reset()

    def _build_lqr_model(self, speed_mps):
        a_matrix = np.array([
         [
          0.0, speed_mps, 0.0],
         [
          0.0, 0.0, 0.0],
         [
          1.0, 0.0, 0.0]])
        b_matrix = np.array([
         [
          0.0],
         [
          speed_mps / self.L],
         [
          0.0]])
        return (
         a_matrix, b_matrix)

    @staticmethod
    def _compute_recovered_errors(vehicle, target_waypoint):
        transform = vehicle.get_transform()
        yaw = np.radians(transform.rotation.yaw)
        target_transform = target_waypoint.transform
        target_yaw = np.radians(target_transform.rotation.yaw)
        dx = target_transform.location.x - transform.location.x
        dy = target_transform.location.y - transform.location.y
        e_y = np.sin(yaw) * dx - np.cos(yaw) * dy
        e_psi = (yaw - target_yaw + np.pi) % (2.0 * np.pi) - np.pi
        return (float(e_y), float(e_psi))

    def run_step(self, vehicle, target_waypoint, curvature=0.0, reference_waypoint=None, planned_target_speed_mps=None, tracking_errors=None):
        velocity = vehicle.get_velocity()
        speed_mps = max(np.sqrt(velocity.x ** 2 + velocity.y ** 2), 0.5)
        e_y, e_psi = self._compute_recovered_errors(vehicle, target_waypoint)
        self._integral_ey = float(np.clip(self._integral_ey + e_y * 0.05, -0.8, 0.8))
        a_matrix, b_matrix = self._build_lqr_model(speed_mps)
        p_matrix = linalg.solve_continuous_are(a_matrix, b_matrix, self.Q, self.R)
        gain = np.linalg.inv(self.R) @ b_matrix.T @ p_matrix
        state = np.array([e_y, e_psi, self._integral_ey])
        steer = float(np.asarray(-gain @ state).item())
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        steer_change = np.clip(steer - self._prev_steer, -self.max_steer_rate, self.max_steer_rate)
        steer = float(self._prev_steer + steer_change)
        self._prev_steer = steer
        throttle, brake = self._longitudinal_controller.run_step(speed_mps,
          target_speed_mps=planned_target_speed_mps)
        control = carla.VehicleControl()
        control.steer = steer
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False
        control.manual_gear_shift = False
        return control


class InitialTwoStateMpcController(BaseTrackingController):
    __doc__ = "Two-state linear MPC as committed in the initial revision."

    def __init__(self, target_speed=30.0, horizon=12, dt=0.05, q_y=10.0, q_psi=14.0, r_steer=0.9, r_steer_rate=1.2, kp_long=0.6, ki_long=0.01, kd_long=0.06, max_steer=0.65, max_steer_rate=0.3, longitudinal_controller=None):
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
        self._longitudinal_controller = longitudinal_controller or PidLongitudinalController(target_speed_kmh=target_speed,
          kp=kp_long,
          ki=ki_long,
          kd=kd_long,
          dt=dt)
        self.reset()

    def reset(self):
        self._prev_steer = 0.0
        self._longitudinal_controller.reset()

    def _build_single_step_model(self, speed_mps):
        a_matrix = np.array([[1.0, self.dt * speed_mps], [0.0, 1.0]])
        b_matrix = np.array([[0.0], [self.dt * speed_mps / self.L]])
        curvature_matrix = np.array([0.0, -self.dt * speed_mps]).reshape(2, 1)
        return (a_matrix, b_matrix, curvature_matrix)

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
                power = np.linalg.matrix_power(a_matrix, step - control_step)
                gamma[(row, col)] = power @ b_matrix
                theta[(row, control_step)] = (power @ curvature_matrix).ravel()

        return (
         phi, gamma, theta)

    def _build_steer_rate_matrix(self):
        diff_matrix = np.eye(self.horizon)
        for idx in range(1, self.horizon):
            diff_matrix[(idx, idx - 1)] = -1.0

        return diff_matrix

    def _build_curvature_sequence(self, curvature):
        if np.isscalar(curvature):
            return np.full(self.horizon, float(curvature))
        sequence = np.asarray(curvature, dtype=float).reshape(-1)
        if len(sequence) >= self.horizon:
            return sequence[:self.horizon]
        if len(sequence) == 0:
            return np.zeros(self.horizon)
        return np.pad(sequence, (0, self.horizon - len(sequence)), mode="edge")

    def _build_reference_steer_sequence(self, curvature_sequence):
        return np.clip(np.arctan(self.L * curvature_sequence), -self.max_steer, self.max_steer)

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
        gradient = gamma.T @ q_bar @ base_state - r_bar @ reference_steer - self.r_steer_rate * (diff_matrix.T @ prev_term)
        regularized_hessian = hessian + 1e-06 * np.eye(hessian.shape[0])
        steer_sequence = -np.linalg.solve(regularized_hessian, gradient)
        return float(steer_sequence[0])

    def run_step(self, vehicle, target_waypoint, curvature=0.0, reference_waypoint=None, planned_target_speed_mps=None, tracking_errors=None):
        velocity = vehicle.get_velocity()
        speed_mps = max(np.sqrt(velocity.x ** 2 + velocity.y ** 2), 1.0)
        reference_waypoint = reference_waypoint or target_waypoint
        if tracking_errors is None:
            tracking_errors = compute_path_tracking_errors(vehicle, reference_waypoint)
        initial_state = np.array([tracking_errors["e_y"], tracking_errors["e_psi"]])
        steer = self._solve_mpc_steering(initial_state, speed_mps, curvature)
        steer = float(np.clip(steer, -self.max_steer, self.max_steer))
        steer_delta = np.clip(steer - self._prev_steer, -self.max_steer_rate, self.max_steer_rate)
        steer = self._prev_steer + steer_delta
        self._prev_steer = steer
        throttle, brake = self._longitudinal_controller.run_step(speed_mps,
          target_speed_mps=planned_target_speed_mps)
        control = carla.VehicleControl()
        control.steer = steer
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False
        control.manual_gear_shift = False
        return control


def _longitudinal(target_speed, kp, ki, kd, max_throttle=1.0, max_brake=1.0):
    return PidLongitudinalController(target_speed_kmh=target_speed,
      kp=kp,
      ki=ki,
      kd=kd,
      dt=0.05,
      max_throttle=max_throttle,
      max_brake=max_brake)


InitialMpcController = RecoveredFourStateMpcController

def build_initial_controller(controller_name, vehicle, args):
    """Build the frozen initial implementation for a current runtime request."""
    target_speed = float(getattr(args, "target_speed", 30.0))
    if controller_name == "pid":
        return InitialPidController(vehicle=vehicle,
          target_speed=target_speed)
    if controller_name == "lqr":
        return InitialLqrController(target_speed=target_speed,
          longitudinal_controller=(_longitudinal(target_speed, 0.35, 0.02, 0.16)))
    if controller_name == "mpc":
        return InitialMpcController(target_speed=target_speed,
          horizon=16,
          q_y=14.0,
          q_psi=22.0,
          r_steer=1.0,
          r_steer_rate=1.1,
          r_steer_accel=0.08,
          max_steer=0.65,
          max_steer_rate=0.3,
          curvature_step_limit=0.02,
          max_lateral_accel=7.5,
          min_dynamic_steer_limit=0.18,
          cornering_stiffness_scale=1.0,
          min_horizon=8,
          model_type="dynamic_bicycle",
          derivative_alpha=0.25,
          longitudinal_controller=(_longitudinal(target_speed, 0.6, 0.01, 0.06)))
    raise ValueError("Unsupported initial controller {!r}; expected pid, lqr or mpc.".format(controller_name))


INITIAL_CONTROLLER_BUILDERS = {name: lambda vehicle, args, controller_name=name: build_initial_controller(controller_name, vehicle, args) for name in ('lqr',
                                                                                                                                                       'pid',
                                                                                                                                                       'mpc')}
