# uncompyle6 version 3.9.3
# Python bytecode version base 3.7.0 (3394)
# Decompiled from: Python 3.13.13 | packaged by Anaconda, Inc. | (main, Apr 14 2026, 06:12:50) [MSC v.1942 64 bit (AMD64)]
# Embedded file name: D:\WindowsNoEditor\PythonAPI\my_control_project\control\authoritative_baseline.py
# Compiled at: 2026-07-14 01:41:06
# Size of source mod 2**32: 13245 bytes
"""Frozen, untuned PID/LQR/MPC baselines used only for comparison runs."""
from collections import deque
import copy, hashlib, inspect, math, carla, numpy as np
from scipy import linalg
from scipy.optimize import minimize
from scipy.signal import cont2discrete
from agents.navigation.controller import PIDLongitudinalController, VehiclePIDController
DT = 0.05
MASS_KG = 1750.0
YAW_INERTIA_KGM2 = 2875.0
LF_M = 1.45
LR_M = 1.45
CF_N_PER_RAD = 19000.0
CR_N_PER_RAD = 33000.0
WHEELBASE_M = LF_M + LR_M
MIN_MODEL_SPEED_MPS = 0.5
PID_LATERAL = {
 'K_P': 1.95, 'K_I': 0.05, 'K_D': 0.2, 'dt': DT}
PID_LONGITUDINAL = {'K_P': 1.0, 'K_I': 0.05, 'K_D': 0.0, 'dt': DT}

def _official_pid_identity():
    path = inspect.getsourcefile(VehiclePIDController) or ""
    digest = ""
    if path:
        with open(path, "rb") as handle:
            digest = hashlib.sha256(handle.read()).hexdigest()
    return {'source_path':path, 
     'sha256':digest}


def authoritative_audit():
    """Return a fresh copy so a caller cannot mutate the frozen manifest."""
    audit = {'label':"authoritative_baseline_v1", 
     'tuned':False, 
     'vehicle':{
      'mass_kg': MASS_KG, 
      'yaw_inertia_kgm2': YAW_INERTIA_KGM2, 
      'lf_m': LF_M, 
      'lr_m': LR_M, 
      'cf_n_per_rad': CF_N_PER_RAD, 
      'cr_n_per_rad': CR_N_PER_RAD}, 
     'pid':{
      'source': '"CARLA_0.9.14_VehiclePIDController"', 
      'lateral': PID_LATERAL, 
      'longitudinal': PID_LONGITUDINAL, 
      'max_throttle': 0.75, 
      'max_brake': 0.3, 
      'max_steering': 0.8, 
      'offset': 0.0}, 
     'lqr':{'source':"Snider_2009_Rajamani_four_state_with_feedforward", 
      'q_diagonal':[
       0.075, 0.0, 0.0, 0.0], 
      'r':1.0, 
      'sample_time_s':DT}, 
     'mpc':{'source':"Artunedo_2024_LTV_NLMPC_1_equivalent_Python_reproduction", 
      'prediction_horizon':11, 
      'control_horizon':3, 
      'lateral_error_weight':1.0, 
      'steering_rate_weight':15.0, 
      'max_iterations':10, 
      'sample_time_s':DT, 
      'assumption_label':"initial_assumption_v1", 
      'assumption_source':"self_defined_for_unreported_parameter", 
      'tuned':False, 
      'minimum_model_speed_mps':MIN_MODEL_SPEED_MPS, 
      'slsqp_ftol':1e-06, 
      'initial_move_sequence':[
       0.0, 0.0, 0.0], 
      'failure_policy':"zero_then_hold_previous_valid"}}
    try:
        audit["pid"]["official_implementation"] = _official_pid_identity()
    except OSError:
        audit["pid"]["official_implementation"] = {'source_path':"", 
         'sha256':""}

    return copy.deepcopy(audit)


def dynamic_bicycle_model(speed_mps):
    """Snider/Rajamani four-state lateral-error model at a fixed speed."""
    speed = max(float(speed_mps), MIN_MODEL_SPEED_MPS)
    cf = 2.0 * CF_N_PER_RAD
    cr = 2.0 * CR_N_PER_RAD
    a = np.array([
     [
      0.0, 1.0, 0.0, 0.0],
     [
      0.0,
      -(cf + cr) / (MASS_KG * speed),
      (cf + cr) / MASS_KG,
      (cr * LR_M - cf * LF_M) / (MASS_KG * speed)],
     [
      0.0, 0.0, 0.0, 1.0],
     [
      0.0,
      (cr * LR_M - cf * LF_M) / (YAW_INERTIA_KGM2 * speed),
      (cf * LF_M - cr * LR_M) / YAW_INERTIA_KGM2,
      -(cf * LF_M ** 2 + cr * LR_M ** 2) / (YAW_INERTIA_KGM2 * speed)]])
    b = np.array([[0.0], [cf / MASS_KG], [0.0], [cf * LF_M / YAW_INERTIA_KGM2]])
    return (a, b)


def _discrete_model(speed_mps):
    a, b = dynamic_bicycle_model(speed_mps)
    zeros_c = np.zeros((4, 4))
    zeros_d = np.zeros((4, 1))
    a_d, b_d, _, _, _ = cont2discrete((a, b, zeros_c, zeros_d), DT)
    return (a_d, b_d)


def _front_wheel_limit_rad(vehicle):
    wheels = list(vehicle.get_physics_control().wheels)
    angles = [float(wheel.max_steer_angle) for wheel in wheels[:2] if float(wheel.max_steer_angle) > 0.0]
    if not angles:
        raise RuntimeError("CARLA vehicle did not expose a positive front-wheel max_steer_angle.")
    return math.radians(min(angles))


def _speed_mps(vehicle):
    velocity = vehicle.get_velocity()
    return float(math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2))


def _tracking_state(vehicle, tracking_errors, curvature):
    velocity = vehicle.get_velocity()
    reference_yaw = float(tracking_errors.get("reference_yaw", 0.0))
    e_y_dot = -velocity.x * math.sin(reference_yaw) + velocity.y * math.cos(reference_yaw)
    angular_velocity = vehicle.get_angular_velocity()
    yaw_rate = math.radians(float(angular_velocity.z))
    speed = _speed_mps(vehicle)
    e_psi_dot = yaw_rate - speed * float(curvature)
    return np.array([
     float(tracking_errors["e_y"]),
     e_y_dot,
     float(tracking_errors["e_psi"]),
     e_psi_dot])


def _feedforward_angle(speed_mps, curvature):
    cf = 2.0 * CF_N_PER_RAD
    cr = 2.0 * CR_N_PER_RAD
    understeer = MASS_KG / WHEELBASE_M * (LR_M / cf - LF_M / cr)
    return (WHEELBASE_M + understeer * float(speed_mps) ** 2) * float(curvature)


class _OfficialLongitudinalMixin:

    def _init_longitudinal(self, vehicle, target_speed_kmh):
        self.target_speed_kmh = float(target_speed_kmh)
        self.longitudinal_controller = PIDLongitudinalController(vehicle, **PID_LONGITUDINAL)

    def _build_control(self, steering_angle_rad, planned_target_speed_mps):
        target_kmh = self.target_speed_kmh
        if planned_target_speed_mps is not None:
            target_kmh = float(planned_target_speed_mps) * 3.6
        else:
            acceleration = float(self.longitudinal_controller.run_step(target_kmh))
            control = carla.VehicleControl()
            if acceleration >= 0.0:
                control.throttle = min(acceleration, 0.75)
                control.brake = 0.0
            else:
                control.throttle = 0.0
            control.brake = min(abs(acceleration), 0.3)
        control.steer = float(np.clip(steering_angle_rad / self.max_steer_angle_rad, -1.0, 1.0))
        control.hand_brake = False
        control.manual_gear_shift = False
        return control


class AuthoritativePidController:
    supports_curvature_sequence = False

    def __init__(self, vehicle, target_speed_kmh):
        self.target_speed_kmh = float(target_speed_kmh)
        self.official_controller = VehiclePIDController(vehicle,
          args_lateral=(dict(PID_LATERAL)),
          args_longitudinal=(dict(PID_LONGITUDINAL)),
          offset=0.0,
          max_throttle=0.75,
          max_brake=0.3,
          max_steering=0.8)

    def run_step(self, vehicle, target_waypoint, curvature=0.0, reference_waypoint=None, planned_target_speed_mps=None, tracking_errors=None):
        del vehicle
        del curvature
        del reference_waypoint
        del tracking_errors
        target_kmh = self.target_speed_kmh if planned_target_speed_mps is None else float(planned_target_speed_mps) * 3.6
        return self.official_controller.run_step(target_kmh, target_waypoint)


class AuthoritativeLqrController(_OfficialLongitudinalMixin):
    supports_curvature_sequence = False

    def __init__(self, vehicle, target_speed_kmh):
        self.q = np.diag([0.075, 0.0, 0.0, 0.0])
        self.r = np.array([[1.0]])
        self.max_steer_angle_rad = _front_wheel_limit_rad(vehicle)
        self._init_longitudinal(vehicle, target_speed_kmh)

    def run_step(self, vehicle, target_waypoint, curvature=0.0, reference_waypoint=None, planned_target_speed_mps=None, tracking_errors=None):
        del target_waypoint
        del reference_waypoint
        if tracking_errors is None:
            raise ValueError("Authoritative LQR requires frozen-runtime tracking_errors.")
        speed = max(_speed_mps(vehicle), MIN_MODEL_SPEED_MPS)
        a_d, b_d = _discrete_model(speed)
        p = linalg.solve_discrete_are(a_d, b_d, self.q, self.r)
        gain = np.linalg.solve(b_d.T @ p @ b_d + self.r, b_d.T @ p @ a_d)
        state = _tracking_state(vehicle, tracking_errors, curvature)
        feedback = -float((gain @ state).item())
        steady_heading_error = LR_M * float(curvature) - LF_M * MASS_KG * speed ** 2 * float(curvature) / (2.0 * CR_N_PER_RAD * WHEELBASE_M)
        feedforward = _feedforward_angle(speed, curvature) + float(gain[(0, 2)]) * steady_heading_error
        steering = float(np.clip(feedforward + feedback, -self.max_steer_angle_rad, self.max_steer_angle_rad))
        return self._build_control(steering, planned_target_speed_mps)


class AuthoritativeMpcController(_OfficialLongitudinalMixin):
    supports_curvature_sequence = False
    prediction_horizon = 11
    control_horizon = 3
    max_iterations = 10

    def __init__(self, vehicle, target_speed_kmh):
        self.max_steer_angle_rad = _front_wheel_limit_rad(vehicle)
        self.max_steer_rate_rad_s = 2.0 * self.max_steer_angle_rad
        self.max_steer_step_rad = 0.1 * self.max_steer_angle_rad
        self.initial_move_sequence = np.zeros(self.control_horizon)
        self._warm_start = self.initial_move_sequence.copy()
        self._last_valid_feedback = 0.0
        self._last_valid_steer = 0.0
        self._has_valid_solution = False
        self.last_mpc_solve_mode = "initial"
        self.mpc_failure_count = 0
        self._init_longitudinal(vehicle, target_speed_kmh)

    def _expanded_sequence(self, moves):
        moves = np.asarray(moves, dtype=float)
        return np.concatenate((moves, np.full(self.prediction_horizon - self.control_horizon, moves[-1])))

    def _objective(self, moves, state, a_d, b_d):
        predicted = np.asarray(state, dtype=float).copy()
        previous = self._last_valid_feedback
        cost = 0.0
        for steering in self._expanded_sequence(moves):
            predicted = a_d @ predicted + b_d[:, 0] * float(steering)
            cost += float(predicted[0] ** 2)
            cost += 15.0 * float((steering - previous) ** 2)
            previous = float(steering)

        return cost

    def _constraints(self):
        limit = self.max_steer_step_rad
        previous = self._last_valid_feedback
        return (
         {'type':"ineq", 
          'fun':lambda moves: np.concatenate((
           np.array([limit - abs(float(moves[0]) - previous)]),
           limit - np.abs(np.diff(moves))))},)

    def _solve(self, state, speed, curvature):
        a_d, b_d = _discrete_model(speed)
        feedforward = float(np.clip(_feedforward_angle(speed, curvature), -self.max_steer_angle_rad, self.max_steer_angle_rad))
        result = minimize((self._objective),
          (self._warm_start),
          args=(
         state, a_d, b_d),
          method="SLSQP",
          bounds=([
         (
          -self.max_steer_angle_rad, self.max_steer_angle_rad)] * self.control_horizon),
          constraints=(self._constraints()),
          options={'maxiter':self.max_iterations, 
         'ftol':1e-06,  'disp':False})
        if result.success:
            if np.all(np.isfinite(result.x)):
                moves = np.asarray((result.x), dtype=float)
                feedback = float(moves[0])
                desired = float(np.clip(feedforward + feedback, -self.max_steer_angle_rad, self.max_steer_angle_rad))
                steering = float(np.clip(desired, self._last_valid_steer - self.max_steer_step_rad, self._last_valid_steer + self.max_steer_step_rad))
                self._warm_start = np.array([moves[1], moves[2], moves[2]])
                self._last_valid_feedback = feedback
                self._last_valid_steer = steering
                self._has_valid_solution = True
                self.last_mpc_solve_mode = "fresh"
                return steering
        self.mpc_failure_count += 1
        self.last_mpc_solve_mode = "hold_previous" if self._has_valid_solution else "initial_zero"
        if self._has_valid_solution:
            return self._last_valid_steer
        return 0.0

    def run_step(self, vehicle, target_waypoint, curvature=0.0, reference_waypoint=None, planned_target_speed_mps=None, tracking_errors=None):
        del target_waypoint
        del reference_waypoint
        if tracking_errors is None:
            raise ValueError("Authoritative MPC requires frozen-runtime tracking_errors.")
        speed = max(_speed_mps(vehicle), MIN_MODEL_SPEED_MPS)
        state = _tracking_state(vehicle, tracking_errors, curvature)
        steering = self._solve(state, speed, curvature)
        return self._build_control(steering, planned_target_speed_mps)


def create_authoritative_controller(name, vehicle, args):
    controller_type = {'pid':AuthoritativePidController, 
     'lqr':AuthoritativeLqrController, 
     'mpc':AuthoritativeMpcController}.get(str(name).lower())
    if controller_type is None:
        raise ValueError("Unsupported authoritative controller: {}".format(name))
    return controller_type(vehicle, target_speed_kmh=(float(args.target_speed)))
