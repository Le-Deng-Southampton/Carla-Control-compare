import numpy as np


EXPERIMENT_METADATA_HEADER = [
    "controller",
    "error_provider",
    "route_shape",
    "route_min_length_m",
    "route_length_tolerance",
    "noise_lateral_std",
    "noise_heading_std_deg",
]


def mean(values):
    return sum(values) / len(values) if values else 0.0


def rms(values):
    return float(np.sqrt(mean([value * value for value in values]))) if values else 0.0


def create_metrics():
    return {"abs_ey": [], "abs_epsi": [], "abs_speed_error": [], "steer_delta": []}


def build_experiment_metadata(controller_name, args):
    return [
        controller_name,
        args.error_provider,
        args.route_shape,
        args.route_min_length_m,
        args.route_length_tolerance,
        args.noise_lateral_std,
        args.noise_heading_std_deg,
    ]


def build_summary(controller_name, spawn_index, route_trace, args, metrics):
    abs_ey = metrics["abs_ey"]
    abs_epsi = metrics["abs_epsi"]
    abs_speed_error = metrics["abs_speed_error"]
    steer_delta = metrics["steer_delta"]
    return build_experiment_metadata(controller_name, args) + [
        spawn_index,
        len(route_trace),
        args.target_speed,
        mean(abs_ey),
        rms(abs_ey),
        max(abs_ey or [0.0]),
        np.degrees(mean(abs_epsi)),
        np.degrees(rms(abs_epsi)),
        mean(abs_speed_error),
        mean(steer_delta),
        max(steer_delta or [0.0]),
        args.lqr_q_ey,
        args.lqr_q_ey_dot,
        args.lqr_q_epsi,
        args.lqr_q_epsi_dot,
        args.lqr_r,
        args.lqr_max_steer,
        args.lqr_max_steer_rate,
        args.lqr_derivative_alpha,
        args.lqr_curvature_alpha,
        args.lqr_feedforward_gain,
        args.mpc_horizon,
        args.mpc_q_y,
        args.mpc_q_psi,
        args.mpc_r_steer,
        args.mpc_r_steer_rate,
        args.mpc_kp_long,
        args.mpc_ki_long,
        args.mpc_kd_long,
        args.mpc_max_steer,
        args.mpc_max_steer_rate,
        args.pid_lat_kp,
        args.pid_lat_ki,
        args.pid_lat_kd,
        args.pid_long_kp,
        args.pid_long_ki,
        args.pid_long_kd,
        args.pid_max_throttle,
        args.pid_max_brake,
    ]
