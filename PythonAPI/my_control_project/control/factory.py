import numpy as np

from .lqr_controller import LqrController
from .longitudinal import PidLongitudinalController
from .mpc_controller import MpcController
from .pid_controller import PidControllerAdapter


def get_supported_controller_names():
    return ("lqr", "pid", "mpc")


def create_pid_longitudinal_controller(target_speed_kmh, kp, ki, kd, max_throttle=1.0, max_brake=1.0, dt=0.05):
    return PidLongitudinalController(
        target_speed_kmh=target_speed_kmh,
        kp=kp,
        ki=ki,
        kd=kd,
        dt=dt,
        max_throttle=max_throttle,
        max_brake=max_brake,
    )


def get_arg(args, name, default):
    return getattr(args, name, default)


def create_tracking_controller(controller_name, vehicle, args):
    if controller_name == "lqr":
        return LqrController(
            target_speed=args.target_speed,
            q_weights=(args.lqr_q_ey, args.lqr_q_ey_dot, args.lqr_q_epsi, args.lqr_q_epsi_dot),
            r_weight=args.lqr_r,
            kp_long=args.lqr_kp_long,
            ki_long=args.lqr_ki_long,
            kd_long=args.lqr_kd_long,
            max_steer=args.lqr_max_steer,
            max_steer_rate=args.lqr_max_steer_rate,
            derivative_alpha=get_arg(args, "lqr_derivative_alpha", 0.20),
            curvature_alpha=get_arg(args, "lqr_curvature_alpha", 0.50),
            feedforward_gain=get_arg(args, "lqr_feedforward_gain", 1.0),
            turn_in_rate_scale=get_arg(args, "lqr_turn_in_rate_scale", 0.70),
            turn_in_guard_lateral_error=get_arg(args, "lqr_turn_in_guard_lateral_error", 1.0),
            turn_in_guard_heading_error=np.radians(get_arg(args, "lqr_turn_in_guard_heading_error", 10.0)),
            turn_in_guard_max_curvature=get_arg(args, "lqr_turn_in_guard_max_curvature", 0.04),
            inside_error_feedforward_start=get_arg(args, "lqr_inside_error_feedforward_start", 0.80),
            inside_error_feedforward_full=get_arg(args, "lqr_inside_error_feedforward_full", 1.80),
            inside_error_feedforward_min_scale=get_arg(args, "lqr_inside_error_feedforward_min_scale", 0.65),
            inside_error_feedforward_heading_limit=np.radians(
                get_arg(args, "lqr_inside_error_feedforward_heading_limit", 4.0)
            ),
            longitudinal_controller=create_pid_longitudinal_controller(
                args.target_speed,
                args.lqr_kp_long,
                args.lqr_ki_long,
                args.lqr_kd_long,
            ),
        )

    if controller_name == "pid":
        pid_max_throttle = get_arg(args, "pid_max_throttle", 1.0)
        pid_max_brake = get_arg(args, "pid_max_brake", 0.28)
        return PidControllerAdapter(
            vehicle=vehicle,
            target_speed=args.target_speed,
            lat_kp=args.pid_lat_kp,
            lat_ki=args.pid_lat_ki,
            lat_kd=args.pid_lat_kd,
            long_kp=args.pid_long_kp,
            long_ki=args.pid_long_ki,
            long_kd=args.pid_long_kd,
            dt=0.05,
            max_throttle=pid_max_throttle,
            max_brake=pid_max_brake,
            max_steering=get_arg(args, "pid_max_steer", 0.65),
            max_steer_rate=get_arg(args, "pid_max_steer_rate", 0.65),
            curvature_feedforward_gain=get_arg(args, "pid_curvature_feedforward_gain", 0.0),
            curvature_preview_horizon=get_arg(args, "pid_curvature_preview_horizon", 10),
            curvature_preview_blend=get_arg(args, "pid_curvature_preview_blend", 0.55),
            derivative_filter_alpha=get_arg(args, "pid_derivative_filter_alpha", 1.0),
            longitudinal_controller=create_pid_longitudinal_controller(
                args.target_speed,
                args.pid_long_kp,
                args.pid_long_ki,
                args.pid_long_kd,
                max_throttle=pid_max_throttle,
                max_brake=pid_max_brake,
            ),
        )

    if controller_name == "mpc":
        return MpcController(
            target_speed=args.target_speed,
            horizon=get_arg(args, "mpc_horizon", 16),
            dt=0.05,
            q_y=get_arg(args, "mpc_q_y", 12.0),
            q_psi=get_arg(args, "mpc_q_psi", 18.0),
            r_steer=get_arg(args, "mpc_r_steer", 0.8),
            r_steer_rate=get_arg(args, "mpc_r_steer_rate", 0.9),
            kp_long=get_arg(args, "mpc_kp_long", 0.6),
            ki_long=get_arg(args, "mpc_ki_long", 0.01),
            kd_long=get_arg(args, "mpc_kd_long", 0.06),
            max_steer=get_arg(args, "mpc_max_steer", 0.65),
            max_steer_rate=get_arg(args, "mpc_max_steer_rate", 0.30),
            longitudinal_controller=create_pid_longitudinal_controller(
                args.target_speed,
                get_arg(args, "mpc_kp_long", 0.6),
                get_arg(args, "mpc_ki_long", 0.01),
                get_arg(args, "mpc_kd_long", 0.06),
            ),
        )

    raise ValueError(
        f"Unsupported controller '{controller_name}'. Expected one of {get_supported_controller_names()}."
    )
