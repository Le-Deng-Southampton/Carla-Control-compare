# uncompyle6 version 3.9.3
# Python bytecode version base 3.7.0 (3394)
# Decompiled from: Python 3.13.13 | packaged by Anaconda, Inc. | (main, Apr 14 2026, 06:12:50) [MSC v.1942 64 bit (AMD64)]
# Embedded file name: D:\WindowsNoEditor\PythonAPI\my_control_project\control\recovered_four_state_mpc.py
# Compiled at: 2026-07-14 02:56:10
# Size of source mod 2**32: 36515 bytes
import carla, numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize
from scipy.signal import cont2discrete
from .base import BaseTrackingController
from .longitudinal import PidLongitudinalController
from road_planning.tracking_geometry import compute_path_tracking_errors

class MpcController(BaseTrackingController):
    __doc__ = "Linear MPC lateral controller with explicit curvature preview."
    supports_curvature_sequence = True

    def __init__(self, target_speed=30.0, horizon=20, dt=0.05, q_y=14.0, q_psi=22.0, r_steer=1.0, r_steer_rate=1.1, r_steer_accel=0.08, kp_long=0.6, ki_long=0.01, kd_long=0.06, max_steer=0.65, max_steer_rate=0.3, curvature_step_limit=0.02, max_lateral_accel=6.0, min_dynamic_steer_limit=0.28, cornering_stiffness_scale=1.0, min_horizon=10, model_type="dynamic_bicycle", derivative_alpha=0.25, event_trigger_enabled=True, event_trigger_lateral_error=0.08, event_trigger_heading_error=np.radians(1.0), event_trigger_curvature=0.004, event_trigger_speed_delta=0.75, speed_scheduling_enabled=True, longitudinal_controller=None):
        self.Lf = 1.45
        self.Lr = 1.45
        self.L = self.Lf + self.Lr
        self.horizon = horizon
        self.dt = dt
        self.q_y = q_y
        self.q_psi = q_psi
        self.r_steer = r_steer
        self.r_steer_rate = r_steer_rate
        self.r_steer_accel = r_steer_accel
        self.max_steer = max_steer
        self.max_steer_rate = max_steer_rate
        self.curvature_step_limit = curvature_step_limit
        self.max_lateral_accel = max_lateral_accel
        self.min_dynamic_steer_limit = min_dynamic_steer_limit
        self.cornering_stiffness_scale = cornering_stiffness_scale
        self.min_horizon = min(min_horizon, horizon)
        self.model_type = model_type
        self.derivative_alpha = derivative_alpha
        self.event_trigger_enabled = event_trigger_enabled
        self.event_trigger_lateral_error = event_trigger_lateral_error
        self.event_trigger_heading_error = event_trigger_heading_error
        self.event_trigger_curvature = event_trigger_curvature
        self.event_trigger_speed_delta = event_trigger_speed_delta
        self.speed_scheduling_enabled = speed_scheduling_enabled
        self.m = 1750.0
        self.Iz = 2875.0
        self.Cf = 19000.0 * cornering_stiffness_scale
        self.Cr = 33000.0 * cornering_stiffness_scale
        self._speed_profiles = (
         {'name':"mpc_crawl_recovery", 
          'max_speed_kmh':35.0, 
          'q_y':q_y * 1.7, 
          'q_psi':q_psi * 1.45, 
          'r_steer':r_steer * 0.7, 
          'r_steer_rate':r_steer_rate * 1.25, 
          'r_steer_accel':r_steer_accel * 1.1, 
          'max_steer_rate':min(max_steer_rate * 0.85, 0.25), 
          'curvature_step_limit':max(curvature_step_limit, 0.04), 
          'e_y_dot_scale':0.18, 
          'yaw_rate_scale':0.18},
         {'name':"mpc_low_recovery", 
          'max_speed_kmh':55.0, 
          'q_y':q_y * 1.58, 
          'q_psi':q_psi * 1.38, 
          'r_steer':r_steer * 0.74, 
          'r_steer_rate':r_steer_rate * 1.3, 
          'r_steer_accel':r_steer_accel * 1.15, 
          'max_steer_rate':min(max_steer_rate * 0.85, 0.25), 
          'curvature_step_limit':max(curvature_step_limit, 0.034), 
          'e_y_dot_scale':0.18, 
          'yaw_rate_scale':0.2},
         {'name':"mpc_mid_entry", 
          'max_speed_kmh':85.0, 
          'q_y':q_y * 1.18, 
          'q_psi':q_psi * 1.14, 
          'r_steer':r_steer * 1.05, 
          'r_steer_rate':r_steer_rate * 2.25, 
          'r_steer_accel':r_steer_accel * 2.75, 
          'max_steer_rate':min(max_steer_rate * 0.46, 0.16), 
          'curvature_step_limit':max(curvature_step_limit, 0.022), 
          'e_y_dot_scale':0.24, 
          'yaw_rate_scale':0.3},
         {'name':"mpc_high_preview", 
          'max_speed_kmh':float("inf"), 
          'q_y':q_y * 1.0, 
          'q_psi':q_psi * 1.05, 
          'r_steer':r_steer * 1.35, 
          'r_steer_rate':r_steer_rate * 4.6, 
          'r_steer_accel':r_steer_accel * 5.0, 
          'max_steer_rate':max_steer_rate * 0.32, 
          'curvature_step_limit':min(curvature_step_limit, 0.016), 
          'e_y_dot_scale':0.34, 
          'yaw_rate_scale':0.5})
        self._longitudinal_controller = longitudinal_controller or PidLongitudinalController(target_speed_kmh=target_speed,
          kp=kp_long,
          ki=ki_long,
          kd=kd_long,
          dt=dt)
        self.reset()

    def reset(self):
        self._prev_steer = 0.0
        self._filtered_e_y_dot = 0.0
        self._filtered_yaw_rate = 0.0
        self._prev_e_psi = 0.0
        self._has_prev_state = False
        self._active_horizon = self.horizon
        self._last_steer_limit = self.max_steer
        self._cached_steer_sequence = None
        self._cached_solution_speed = None
        self._cached_solution_curvature = None
        self._cached_solution_state = None
        self._cached_solution_profile = None
        self._cached_reuse_count = 0
        self._active_speed_profile = self._base_speed_profile()
        self.last_speed_profile = self._active_speed_profile["name"]
        self._longitudinal_controller.reset()

    def _base_speed_profile(self):
        return {'name':"mpc_base", 
         'q_y':self.q_y, 
         'q_psi':self.q_psi, 
         'r_steer':self.r_steer, 
         'r_steer_rate':self.r_steer_rate, 
         'r_steer_accel':self.r_steer_accel, 
         'max_steer_rate':self.max_steer_rate, 
         'curvature_step_limit':self.curvature_step_limit, 
         'e_y_dot_scale':0.15, 
         'yaw_rate_scale':0.1}

    def _uses_dynamic_bicycle_model(self):
        return self.model_type == "dynamic_bicycle"

    def _prediction_horizon(self):
        return max(int(getattr(self, "_active_horizon", self.horizon)), 1)

    def _select_speed_profile(self, speed_mps):
        if not self.speed_scheduling_enabled:
            return self._base_speed_profile()
        speed_kmh = float(speed_mps) * 3.6
        for profile in self._speed_profiles:
            if speed_kmh < profile["max_speed_kmh"]:
                return profile

        return self._speed_profiles[-1]

    def _build_single_step_model(self, speed_mps):
        if self._uses_dynamic_bicycle_model():
            return self._build_dynamic_bicycle_step_model(speed_mps)
        a_matrix = np.array([
         [
          1.0, self.dt * speed_mps],
         [
          0.0, 1.0]])
        b_matrix = np.array([
         [
          0.0],
         [
          self.dt * speed_mps / self.L]])
        curvature_matrix = np.array([
         0.0,
         -self.dt * speed_mps]).reshape(2, 1)
        return (a_matrix, b_matrix, curvature_matrix)

    def _build_dynamic_bicycle_step_model(self, speed_mps):
        speed_mps = max(float(speed_mps), 1.0)
        a_cont = np.array([
         [
          0.0, 1.0, speed_mps, 0.0],
         [
          0.0,
          -(2 * self.Cf + 2 * self.Cr) / (self.m * speed_mps),
          0.0,
          -(2 * self.Cf * self.Lf - 2 * self.Cr * self.Lr) / (self.m * speed_mps) - speed_mps],
         [
          0.0, 0.0, 0.0, 1.0],
         [
          0.0,
          -(2 * self.Cf * self.Lf - 2 * self.Cr * self.Lr) / (self.Iz * speed_mps),
          0.0,
          -(2 * self.Cf * self.Lf ** 2 + 2 * self.Cr * self.Lr ** 2) / (self.Iz * speed_mps)]])
        b_steer = np.array([
         [
          0.0],
         [
          2 * self.Cf / self.m],
         [
          0.0],
         [
          2 * self.Cf * self.Lf / self.Iz]])
        b_curvature = np.array([
         [
          0.0],
         [
          0.0],
         [
          -speed_mps],
         [
          0.0]])
        b_cont = np.hstack((b_steer, b_curvature))
        zeros_c = np.zeros((4, 4))
        zeros_d = np.zeros((4, 2))
        a_disc, b_disc, _, _, _ = cont2discrete((a_cont, b_cont, zeros_c, zeros_d), self.dt)
        return (a_disc, b_disc[:, [0]], b_disc[:, [1]])

    def _build_prediction_matrices(self, speed_mps):
        a_matrix, b_matrix, curvature_matrix = self._build_single_step_model(speed_mps)
        state_dim = a_matrix.shape[0]
        input_dim = b_matrix.shape[1]
        horizon = self._prediction_horizon()
        phi = np.zeros((horizon * state_dim, state_dim))
        gamma = np.zeros((horizon * state_dim, horizon * input_dim))
        theta = np.zeros((horizon * state_dim, horizon))
        a_power = np.eye(state_dim)
        for step in range(horizon):
            a_power = a_power @ a_matrix
            row = slice(step * state_dim, (step + 1) * state_dim)
            phi[row, :] = a_power
            for control_step in range(step + 1):
                col = slice(control_step * input_dim, (control_step + 1) * input_dim)
                gamma[(row, col)] = np.linalg.matrix_power(a_matrix, step - control_step) @ b_matrix
                theta[(row, control_step)] = (np.linalg.matrix_power(a_matrix, step - control_step) @ curvature_matrix).ravel()

        return (phi, gamma, theta)

    def _build_steer_rate_matrix(self):
        horizon = self._prediction_horizon()
        diff_matrix = np.eye(horizon)
        for idx in range(1, horizon):
            diff_matrix[(idx, idx - 1)] = -1.0

        return diff_matrix

    def _build_steer_accel_matrix(self):
        horizon = self._prediction_horizon()
        if horizon < 3:
            return np.zeros((0, horizon))
        accel_matrix = np.zeros((horizon - 2, horizon))
        for idx in range(horizon - 2):
            accel_matrix[(idx, idx)] = 1.0
            accel_matrix[(idx, idx + 1)] = -2.0
            accel_matrix[(idx, idx + 2)] = 1.0

        return accel_matrix

    def _build_curvature_sequence(self, curvature):
        horizon = self._prediction_horizon()
        if np.isscalar(curvature):
            return np.full(horizon, float(curvature))
        sequence = np.asarray(curvature, dtype=float).reshape(-1)
        if len(sequence) >= horizon:
            return sequence[:horizon]
        if len(sequence) == 0:
            return np.zeros(horizon)
        return np.pad(sequence, (0, horizon - len(sequence)), mode="edge")

    def _condition_curvature_sequence(self, curvature_sequence, step_limit=None):
        raw_sequence = np.asarray(curvature_sequence, dtype=float).reshape(-1)
        if len(raw_sequence) == 0:
            return np.zeros(self._prediction_horizon())
        if step_limit is None:
            step_limit = self.curvature_step_limit
        step_limit = max(float(step_limit), 0.0)
        conditioned = np.zeros_like(raw_sequence)
        conditioned[0] = raw_sequence[0]
        for idx in range(1, len(raw_sequence)):
            delta = raw_sequence[idx] - conditioned[idx - 1]
            if step_limit > 0.0:
                delta = np.clip(delta, -step_limit, step_limit)
            conditioned[idx] = conditioned[idx - 1] + delta

        return conditioned

    def _dynamic_steer_limit(self, speed_mps):
        speed_mps = max(float(speed_mps), 0.1)
        physical_limit = np.arctan(self.L * max(float(self.max_lateral_accel), 0.0) / speed_mps ** 2)
        low_speed_floor = max(float(self.min_dynamic_steer_limit), 0.0)
        high_speed_floor = min(low_speed_floor, 0.1)
        speed_progress = self._error_progress(speed_mps * 3.6, 50.0, 82.0)
        lower_limit = low_speed_floor - speed_progress * (low_speed_floor - high_speed_floor)
        return float(np.clip(max(lower_limit, physical_limit), 0.0, self.max_steer))

    def _build_reference_steer_sequence(self, curvature_sequence, steer_limit=None):
        if steer_limit is None:
            steer_limit = self.max_steer
        steer_limit = float(np.clip(steer_limit, 0.0, self.max_steer))
        return np.clip(np.arctan(self.L * curvature_sequence), -steer_limit, steer_limit)

    def _error_progress(self, value, start, full):
        start = max(float(start), 0.0)
        full = max(float(full), start + 1e-06)
        return float(np.clip((abs(value) - start) / (full - start), 0.0, 1.0))

    def _dominant_curvature(self, curvature):
        values = np.asarray(curvature, dtype=float).reshape(-1)
        if values.size == 0:
            return 0.0
        return float(values[int(np.argmax(np.abs(values)))])

    def _curve_entry_reference_scale(self, initial_state, curvature_sequence):
        state = np.asarray(initial_state, dtype=float).reshape(-1)
        heading_error = state[2] if state.size >= 4 else state[1]
        near_centerline = abs(state[0]) <= 0.6 and abs(heading_error) <= np.radians(6.0)
        if not near_centerline:
            return 1.0
        sequence = np.asarray(curvature_sequence, dtype=float).reshape(-1)
        if sequence.size == 0:
            return 1.0
        current_curvature = float(sequence[0])
        dominant_curvature = self._dominant_curvature(sequence)
        if abs(dominant_curvature) <= abs(current_curvature) + 0.018:
            return 1.0
        entry_progress = self._error_progress(dominant_curvature, 0.02, 0.075)
        return 1.0 + 0.01 * entry_progress

    def _max_abs_curvature(self, curvature):
        values = np.asarray(curvature, dtype=float).reshape(-1)
        if values.size:
            return float(np.max(np.abs(values)))
        return 0.0

    def _heading_error_from_state(self, state):
        state = np.asarray(state, dtype=float).reshape(-1)
        if state.size >= 4:
            return float(state[2])
        if state.size >= 2:
            return float(state[1])
        return 0.0

    def _state_weight_block(self, speed_profile, state_dim):
        if state_dim == 4:
            return np.diag([
             speed_profile["q_y"],
             speed_profile.get("e_y_dot_scale", 0.15) * speed_profile["q_y"],
             speed_profile["q_psi"],
             speed_profile.get("yaw_rate_scale", 0.1) * speed_profile["q_psi"]])
        return np.diag([speed_profile["q_y"], speed_profile["q_psi"]])

    def _current_curvature(self, curvature):
        sequence = self._build_curvature_sequence(curvature)
        if len(sequence):
            return float(sequence[0])
        return 0.0

    def _entry_risk(self, initial_state, curvature):
        state = np.asarray(initial_state, dtype=float).reshape(-1)
        heading_error = state[2] if state.size >= 4 else state[1]
        if abs(state[0]) > 0.75 or abs(heading_error) > np.radians(7.0):
            return 0.0
        return self._error_progress(self._max_abs_curvature(curvature), 0.014, 0.055)

    def _pre_loss_recenter_risk(self, initial_state):
        state = np.asarray(initial_state, dtype=float).reshape(-1)
        heading_error = state[2] if state.size >= 4 else state[1]
        if abs(state[0]) > 1.8 or abs(heading_error) > np.radians(14.0):
            return 0.0
        lateral_risk = self._error_progress(state[0], 0.45, 1.25)
        heading_risk = self._error_progress(heading_error, np.radians(3.0), np.radians(10.0))
        return max(lateral_risk, heading_risk)

    def _near_centerline_risk(self, initial_state):
        state = np.asarray(initial_state, dtype=float).reshape(-1)
        heading_error = state[2] if state.size >= 4 else state[1]
        if abs(state[0]) > 0.8 or abs(heading_error) > np.radians(8.0):
            return 0.0
        return 1.0

    def _scheduled_profile(self, speed_profile, initial_state, curvature):
        entry_risk = self._entry_risk(initial_state, curvature)
        recenter_risk = self._pre_loss_recenter_risk(initial_state)
        near_centerline_risk = self._near_centerline_risk(initial_state)
        profile = dict(speed_profile)
        profile["q_y"] *= 1.0 + 0.08 * entry_risk
        profile["q_psi"] *= 1.0 + 0.14 * entry_risk
        profile["r_steer"] *= 1.0 + 0.12 * entry_risk
        profile["r_steer_rate"] *= 1.0 + 0.45 * entry_risk
        profile["r_steer_accel"] *= 1.0 + 0.25 * entry_risk
        profile["q_y"] *= 1.0 + 0.24 * recenter_risk
        profile["q_psi"] *= 1.0 + 0.18 * recenter_risk
        profile["r_steer"] *= 1.0 + 0.1 * recenter_risk
        profile["r_steer_rate"] *= 1.0 + 0.5 * recenter_risk
        profile["r_steer_accel"] *= 1.0 + 0.35 * recenter_risk
        profile["max_steer_rate"] *= 1.0 - 0.18 * recenter_risk
        profile["r_steer_rate"] *= 1.0 + 0.36 * near_centerline_risk
        profile["r_steer_accel"] *= 1.0 + 0.45 * near_centerline_risk
        profile["max_steer_rate"] *= 1.0 - 0.26 * near_centerline_risk
        return profile

    def _damp_high_speed_steer_output(self, steer, speed_mps, tracking_errors):
        steer = float(steer)
        if not self.speed_scheduling_enabled:
            return steer
            speed_kmh = float(speed_mps) * 3.6
            if speed_kmh < 70.0:
                return steer
            errors = tracking_errors or {}
            e_y = abs(float(errors.get("e_y", 0.0)))
            e_psi = abs(float(errors.get("e_psi", 0.0)))
            if e_y > 1.25 or e_psi > np.radians(6.5):
                return steer
            prev_steer = float(self._prev_steer)
            if abs(prev_steer) < 1e-06:
                return steer
            speed_progress = self._error_progress(speed_kmh, 70.0, 110.0)
            error_progress = max(self._error_progress(e_y, 0.35, 1.25), self._error_progress(e_psi, np.radians(1.5), np.radians(6.5)))
            damping = np.clip(0.28 + 0.25 * speed_progress + 0.12 * (1.0 - error_progress), 0.0, 0.68)
            damped = (1.0 - damping) * steer + damping * prev_steer
            if steer * prev_steer < 0.0 and abs(steer) > 0.025 and abs(prev_steer) > 0.025:
                max_steer_rate = float(self._active_speed_profile.get("max_steer_rate", self.max_steer_rate))
                reversal_limit = max(max_steer_rate * (0.45 - 0.18 * speed_progress), 0.006)
                if abs(prev_steer) > reversal_limit:
                    damped = np.sign(prev_steer) * min(abs(damped), reversal_limit)
        else:
            damped = np.clip(damped, -reversal_limit, reversal_limit)
        hold_deadband = 0.0025 + 0.0025 * speed_progress
        if error_progress < 0.45:
            if abs(damped - prev_steer) <= hold_deadband:
                return prev_steer
        return float(damped)

    def _select_horizon(self, speed_mps, initial_state, curvature):
        entry_risk = self._entry_risk(initial_state, curvature)
        speed_risk = self._error_progress(speed_mps * 3.6, 55.0, 105.0)
        risk = max(entry_risk, speed_risk)
        span = max(int(self.horizon) - int(self.min_horizon), 0)
        return int(round(self.min_horizon + span * risk))

    def _normalize_initial_state(self, initial_state, speed_mps, curvature):
        state = np.asarray(initial_state, dtype=float).reshape(-1)
        if self._uses_dynamic_bicycle_model():
            if state.size >= 4:
                return state[:4]
            if state.size == 2:
                return np.array([state[0], 0.0, state[1], speed_mps * self._current_curvature(curvature)])
            raise ValueError("Dynamic MPC initial_state must contain 2 or 4 values.")
        if state.size >= 2:
            return state[:2]
        raise ValueError("Kinematic MPC initial_state must contain at least 2 values.")

    def _estimate_dynamic_initial_state(self, vehicle, tracking_errors, speed_mps, current_curvature):
        velocity = vehicle.get_velocity()
        reference_yaw = tracking_errors.get("reference_yaw", 0.0)
        raw_e_y_dot = -velocity.x * np.sin(reference_yaw) + velocity.y * np.cos(reference_yaw)
        angular_velocity = getattr(vehicle, "get_angular_velocity", None)
        if angular_velocity is not None:
            raw_yaw_rate = np.radians(angular_velocity().z)
        else:
            if self._has_prev_state:
                raw_yaw_rate = (tracking_errors["e_psi"] - self._prev_e_psi) / self.dt + speed_mps * current_curvature
            else:
                raw_yaw_rate = speed_mps * current_curvature
        alpha = float(np.clip(self.derivative_alpha, 0.0, 1.0))
        if not self._has_prev_state:
            self._filtered_e_y_dot = raw_e_y_dot
            self._filtered_yaw_rate = raw_yaw_rate
            self._has_prev_state = True
        else:
            self._filtered_e_y_dot = (1.0 - alpha) * self._filtered_e_y_dot + alpha * raw_e_y_dot
            self._filtered_yaw_rate = (1.0 - alpha) * self._filtered_yaw_rate + alpha * raw_yaw_rate
        self._prev_e_psi = tracking_errors["e_psi"]
        return np.array([
         tracking_errors["e_y"],
         self._filtered_e_y_dot,
         tracking_errors["e_psi"],
         self._filtered_yaw_rate])

    def _build_sequence_constraints(self, diff_matrix, prev_term, max_steer_rate, steer_limit):
        horizon = self._prediction_horizon()
        rate_limits = np.full(horizon, max(float(max_steer_rate), 0.0))
        rate_limits[0] = max(rate_limits[0], max(abs(self._prev_steer) - float(steer_limit), 0.0))
        return [
         LinearConstraint(diff_matrix, prev_term - rate_limits, prev_term + rate_limits)]

    def _sequence_satisfies_constraints(self, sequence, steer_limit, constraints, tolerance=1e-07):
        sequence = np.asarray(sequence, dtype=float).reshape(-1)
        if np.any(sequence > steer_limit + tolerance) or np.any(sequence < -steer_limit - tolerance):
            return False
        for constraint in constraints:
            values = constraint.A @ sequence
            if np.any(values < constraint.lb - tolerance) or np.any(values > constraint.ub + tolerance):
                return False

        return True

    def _project_steer_sequence(self, sequence, steer_limit, diff_matrix, prev_term, max_steer_rate):
        projected = np.clip(np.asarray(sequence, dtype=float).reshape(-1), -steer_limit, steer_limit)
        rate_limits = np.full(projected.shape[0], max(float(max_steer_rate), 0.0))
        rate_limits[0] = max(rate_limits[0], max(abs(self._prev_steer) - float(steer_limit), 0.0))
        for idx in range(projected.shape[0]):
            previous = self._prev_steer if idx == 0 else projected[idx - 1]
            projected[idx] = np.clip(projected[idx], previous - rate_limits[idx], previous + rate_limits[idx])
            projected[idx] = np.clip(projected[idx], -steer_limit, steer_limit)

        return projected

    def _solve_constrained_quadratic(self, hessian, gradient, initial_guess, steer_limit, diff_matrix, prev_term, max_steer_rate, constraints):
        initial_guess = self._project_steer_sequence(initial_guess, steer_limit, diff_matrix, prev_term, max_steer_rate)
        bounds = Bounds(-float(steer_limit) * np.ones_like(initial_guess), float(steer_limit) * np.ones_like(initial_guess))

        def objective(sequence):
            return 0.5 * float(sequence @ hessian @ sequence) + float(gradient @ sequence)

        def jacobian(sequence):
            return hessian @ sequence + gradient

        result = minimize(objective,
          initial_guess,
          jac=jacobian,
          bounds=bounds,
          constraints=constraints,
          method="SLSQP",
          options={'ftol':1e-07, 
         'maxiter':40,  'disp':False})
        if result.success:
            if np.all(np.isfinite(result.x)):
                if self._sequence_satisfies_constraints((result.x), steer_limit, constraints, tolerance=1e-05):
                    return result.x
        return initial_guess

    def _event_trigger_thresholds(self, speed_mps, speed_profile, initial_state):
        speed_kmh = float(speed_mps) * 3.6
        speed_progress = self._error_progress(speed_kmh, 70.0, 110.0)
        state = np.asarray(initial_state, dtype=float).reshape(-1)
        heading_error = self._heading_error_from_state(state)
        near_centerline = abs(state[0]) < 0.55 and abs(heading_error) < np.radians(4.5)
        state_factor = 1.0 + 2.0 * speed_progress
        if near_centerline:
            state_factor *= 1.5
        if speed_profile.get("name") == "mpc_high_preview":
            state_factor *= 1.25
        return {'lateral':min(self.event_trigger_lateral_error * state_factor, 0.42), 
         'heading':min(self.event_trigger_heading_error * state_factor, np.radians(4.5)), 
         'curvature':min(self.event_trigger_curvature * (1.0 + 1.8 * speed_progress), 0.012), 
         'speed_delta':(self.event_trigger_speed_delta) * (1.0 + 1.2 * speed_progress), 
         'state_delta_y':0.1 + (0.16 * speed_progress), 
         'state_delta_heading':(np.radians)(0.8 + 1.4 * speed_progress), 
         'max_reuse':2 + (int(round(3.0 * speed_progress)))}

    def _can_reuse_solution(self, initial_state, speed_mps, curvature_sequence, speed_profile):
        if not self.event_trigger_enabled or self._cached_steer_sequence is None:
            return False
        elif len(self._cached_steer_sequence) < 2:
            return False
        if self._cached_solution_profile != speed_profile.get("name"):
            return False
        state = np.asarray(initial_state, dtype=float).reshape(-1)
        cached_state = self._cached_solution_state
        cached_curvature = self._cached_solution_curvature
        if cached_state is None or cached_curvature is None:
            return False
        cached_state = np.asarray(cached_state, dtype=float).reshape(-1)
        if cached_state.size != state.size:
            return False
        thresholds = self._event_trigger_thresholds(speed_mps, speed_profile, state)
        if self._cached_reuse_count >= thresholds["max_reuse"]:
            return False
        heading_error = self._heading_error_from_state(state)
        cached_heading_error = self._heading_error_from_state(cached_state)
        if abs(state[0]) > thresholds["lateral"] or abs(heading_error) > thresholds["heading"]:
            return False
        if abs(state[0] - cached_state[0]) > thresholds["state_delta_y"]:
            return False
        if abs(heading_error - cached_heading_error) > thresholds["state_delta_heading"]:
            return False
        if self._cached_solution_speed is None or abs(speed_mps - self._cached_solution_speed) > thresholds["speed_delta"]:
            return False
        curvature_values = np.asarray(curvature_sequence, dtype=float).reshape(-1)
        cached_curvature_values = np.asarray(cached_curvature, dtype=float).reshape(-1)
        if curvature_values.size != cached_curvature_values.size:
            return False
        if curvature_values.size:
            curvature_delta = float(np.max(np.abs(curvature_values - cached_curvature_values)))
            if curvature_delta > thresholds["curvature"]:
                return False
        return True

    def _reuse_cached_solution(self):
        reused = np.concatenate((self._cached_steer_sequence[1:], self._cached_steer_sequence[-1:]))
        self._cached_steer_sequence = reused
        self._cached_reuse_count += 1
        return float(reused[0])

    def _previous_solution_anchor(self, horizon, speed_profile):
        if self._cached_steer_sequence is None:
            return
        if self._cached_solution_profile != speed_profile.get("name"):
            return
        cached_sequence = np.asarray((self._cached_steer_sequence), dtype=float).reshape(-1)
        if cached_sequence.size < 2:
            return
        anchor = np.concatenate((cached_sequence[1:], cached_sequence[-1:]))
        horizon = int(horizon)
        if anchor.size >= horizon:
            return anchor[:horizon]
        return np.pad(anchor, (0, horizon - anchor.size), mode="edge")

    def _sequence_consistency_weight(self, speed_mps, speed_profile, initial_state, curvature_sequence):
        speed_kmh = float(speed_mps) * 3.6
        if speed_kmh < 50.0:
            return 0.0
        state = np.asarray(initial_state, dtype=float).reshape(-1)
        heading_error = self._heading_error_from_state(state)
        speed_progress = self._error_progress(speed_kmh, 55.0, 105.0)
        tracking_release = max(self._error_progress(state[0], 1.0, 1.8), self._error_progress(heading_error, np.radians(7.0), np.radians(13.0)))
        preview_release = 0.0
        cached_curvature = self._cached_solution_curvature
        if cached_curvature is None:
            return 0.0
        curvature_values = np.asarray(curvature_sequence, dtype=float).reshape(-1)
        cached_curvature_values = np.asarray(cached_curvature, dtype=float).reshape(-1)
        if curvature_values.size != cached_curvature_values.size:
            return 0.0
        if curvature_values.size:
            curvature_delta = float(np.max(np.abs(curvature_values - cached_curvature_values)))
            preview_release = self._error_progress(curvature_delta, 0.006, 0.025)
            current_dominant = self._dominant_curvature(curvature_values)
            cached_dominant = self._dominant_curvature(cached_curvature_values)
            if current_dominant * cached_dominant < 0.0 and abs(current_dominant) >= 0.012:
                if abs(cached_dominant) >= 0.012:
                    return 0.0
        base_weight = speed_profile["r_steer_rate"] * (0.45 + 0.85 * speed_progress)
        release = max(tracking_release, preview_release)
        return float(base_weight * (1.0 - 0.65 * release))

    def _solve_mpc_steering(self, initial_state, speed_mps, curvature):
        initial_state = self._normalize_initial_state(initial_state, speed_mps, curvature)
        speed_profile = self._select_speed_profile(speed_mps)
        speed_profile = self._scheduled_profile(speed_profile, initial_state, curvature)
        self._active_speed_profile = speed_profile
        self.last_speed_profile = speed_profile["name"]
        self._active_horizon = self._select_horizon(speed_mps, initial_state, curvature)
        curvature_sequence = self._condition_curvature_sequence((self._build_curvature_sequence(curvature)),
          step_limit=(speed_profile["curvature_step_limit"]))
        self._active_speed_profile = speed_profile
        if self._can_reuse_solution(initial_state, speed_mps, curvature_sequence, speed_profile):
            return self._reuse_cached_solution()
        phi, gamma, theta = self._build_prediction_matrices(speed_mps)
        steer_limit = self._dynamic_steer_limit(speed_mps)
        self._last_steer_limit = steer_limit
        reference_steer = self._build_reference_steer_sequence(curvature_sequence, steer_limit=steer_limit)
        reference_steer = np.clip(reference_steer * self._curve_entry_reference_scale(initial_state, curvature_sequence), -steer_limit, steer_limit)
        q_block = self._state_weight_block(speed_profile, initial_state.size)
        horizon = self._prediction_horizon()
        q_bar = np.kron(np.eye(horizon), q_block)
        r_bar = speed_profile["r_steer"] * np.eye(horizon)
        diff_matrix = self._build_steer_rate_matrix()
        accel_matrix = self._build_steer_accel_matrix()
        prev_term = np.zeros(horizon)
        prev_term[0] = self._prev_steer
        base_state = phi @ initial_state + theta @ curvature_sequence
        r_steer_rate = speed_profile["r_steer_rate"]
        r_steer_accel = speed_profile.get("r_steer_accel", self.r_steer_accel)
        hessian = gamma.T @ q_bar @ gamma + r_bar + r_steer_rate * (diff_matrix.T @ diff_matrix) + r_steer_accel * (accel_matrix.T @ accel_matrix)
        gradient = gamma.T @ q_bar @ base_state - r_bar @ reference_steer - r_steer_rate * (diff_matrix.T @ prev_term)
        regularized_hessian = hessian + 1e-06 * np.eye(hessian.shape[0])
        consistency_anchor = self._previous_solution_anchor(horizon, speed_profile)
        consistency_weight = self._sequence_consistency_weight(speed_mps, speed_profile, initial_state, curvature_sequence)
        if consistency_anchor is not None:
            if consistency_weight > 0.0:
                regularized_hessian = regularized_hessian + consistency_weight * np.eye(horizon)
                gradient = gradient - consistency_weight * consistency_anchor
        else:
            unconstrained_sequence = -np.linalg.solve(regularized_hessian, gradient)
            max_steer_rate = speed_profile["max_steer_rate"]
            constraints = self._build_sequence_constraints(diff_matrix, prev_term, max_steer_rate, steer_limit)
            if self._sequence_satisfies_constraints(unconstrained_sequence, steer_limit, constraints):
                steer_sequence = unconstrained_sequence
            else:
                steer_sequence = self._solve_constrained_quadratic(regularized_hessian, gradient, unconstrained_sequence, steer_limit, diff_matrix, prev_term, max_steer_rate, constraints)
        self._cached_steer_sequence = steer_sequence
        self._cached_solution_speed = speed_mps
        self._cached_solution_curvature = curvature_sequence.copy()
        self._cached_solution_state = initial_state.copy()
        self._cached_solution_profile = speed_profile["name"]
        self._cached_reuse_count = 0
        return float(steer_sequence[0])

    def run_step(self, vehicle, target_waypoint, curvature=0.0, reference_waypoint=None, planned_target_speed_mps=None, tracking_errors=None):
        velocity = vehicle.get_velocity()
        speed_mps = max(np.sqrt(velocity.x ** 2 + velocity.y ** 2), 1.0)
        reference_waypoint = reference_waypoint or target_waypoint
        if tracking_errors is None:
            tracking_errors = compute_path_tracking_errors(vehicle, reference_waypoint)
        else:
            e_y = tracking_errors["e_y"]
            e_psi = tracking_errors["e_psi"]
            current_curvature = self._current_curvature(curvature)
            if self._uses_dynamic_bicycle_model():
                initial_state = self._estimate_dynamic_initial_state(vehicle, tracking_errors, speed_mps, current_curvature)
            else:
                initial_state = np.array([e_y, e_psi])
        steer = self._solve_mpc_steering(initial_state, speed_mps, curvature)
        steer_limit = getattr(self, "_last_steer_limit", self._dynamic_steer_limit(speed_mps))
        steer = float(np.clip(steer, -steer_limit, steer_limit))
        steer = self._damp_high_speed_steer_output(steer, speed_mps, tracking_errors)
        max_steer_rate = self._active_speed_profile["max_steer_rate"]
        steer_delta = np.clip(steer - self._prev_steer, -max_steer_rate, max_steer_rate)
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
