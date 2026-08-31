"""Run the current experiment harness with the mixed historical baseline."""

import os
import sys


PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
CARLA_PYTHON_ROOT = os.path.join(os.path.dirname(PROJECT_ROOT), "carla")
if CARLA_PYTHON_ROOT not in sys.path:
    sys.path.insert(0, CARLA_PYTHON_ROOT)

import control.factory as controller_factory
from control.initial_baseline import (
    BASELINE_IMPLEMENTATION,
    INITIAL_CONTROLLER_BUILDERS,
    MPC_FIRST_RECORDED_RUN,
    MPC_SOURCE_SESSION,
    MPC_SOURCE_SNAPSHOT_SHA256,
    PID_LQR_PROVENANCE,
)


INITIAL_ARGUMENT_VALUES = {
    "controller_implementation": BASELINE_IMPLEMENTATION,
    "pid_lqr_provenance": PID_LQR_PROVENANCE,
    "mpc_provenance_session": MPC_SOURCE_SESSION,
    "mpc_provenance_snapshot_sha256": MPC_SOURCE_SNAPSHOT_SHA256,
    "mpc_provenance_first_recorded_run": MPC_FIRST_RECORDED_RUN,
    "lqr_q_ey": 2.2,
    "lqr_q_ey_dot": 0.0,
    "lqr_q_epsi": 1.1,
    "lqr_q_epsi_dot": 0.0,
    "lqr_q_iey": 0.04,
    "lqr_r": 8.0,
    "lqr_kp_long": 0.35,
    "lqr_ki_long": 0.02,
    "lqr_kd_long": 0.16,
    "lqr_max_steer": 0.65,
    "lqr_max_steer_rate": 0.045,
    "lqr_derivative_alpha": 0.0,
    "lqr_curvature_alpha": 0.0,
    "lqr_feedforward_gain": 0.0,
    "mpc_horizon": 16,
    "mpc_q_y": 14.0,
    "mpc_q_psi": 22.0,
    "mpc_r_steer": 1.0,
    "mpc_r_steer_rate": 1.1,
    "mpc_kp_long": 0.6,
    "mpc_ki_long": 0.01,
    "mpc_kd_long": 0.06,
    "mpc_max_steer": 0.65,
    "mpc_max_steer_rate": 0.30,
    "mpc_curvature_step_limit": 0.02,
    "mpc_max_lateral_accel": 7.5,
    "mpc_min_dynamic_steer_limit": 0.18,
    "mpc_cornering_stiffness_scale": 1.0,
    "mpc_min_horizon": 8,
    "mpc_model_type": "dynamic_bicycle",
    "mpc_derivative_alpha": 0.25,
    "pid_lat_kp": 0.72,
    "pid_lat_ki": 0.005,
    "pid_lat_kd": 0.38,
    "pid_long_kp": 0.22,
    "pid_long_ki": 0.01,
    "pid_long_kd": 0.14,
    "pid_max_throttle": 0.45,
    "pid_max_brake": 0.28,
}


def apply_initial_argument_values(args):
    """Keep run metadata aligned with constants used by the frozen builders."""
    for name, value in INITIAL_ARGUMENT_VALUES.items():
        setattr(args, name, value)
    return args


def main():
    controller_factory.CONTROLLER_BUILDERS.clear()
    controller_factory.CONTROLLER_BUILDERS.update(INITIAL_CONTROLLER_BUILDERS)

    # Import after replacing the factory so CLI controller choices and runtime
    # construction both resolve to the historical implementations.
    import run_my_control

    current_parse_args = run_my_control.parse_args

    def parse_initial_args():
        return apply_initial_argument_values(current_parse_args())

    run_my_control.parse_args = parse_initial_args
    run_my_control.compare_main()


if __name__ == "__main__":
    main()
