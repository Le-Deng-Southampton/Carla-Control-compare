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
            curvature_alpha=get_arg(args, "lqr_curvature_alpha", 0.50),
            curvature_preview_horizon=get_arg(args, "lqr_curvature_preview_horizon", 12),
            curvature_preview_blend=get_arg(args, "lqr_curvature_preview_blend", 0.40),
            max_lateral_accel=get_arg(args, "lqr_max_lateral_accel", 0.0),
            min_dynamic_steer_limit=get_arg(args, "lqr_min_dynamic_steer_limit", 0.12),
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
            derivative_filter_alpha=get_arg(args, "pid_derivative_filter_alpha", 0.50),
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
            curvature_step_limit=get_arg(args, "mpc_curvature_step_limit", 0.0),
            max_lateral_accel=get_arg(args, "mpc_max_lateral_accel", 0.0),
            min_dynamic_steer_limit=get_arg(args, "mpc_min_dynamic_steer_limit", 0.24),
            cornering_stiffness_scale=get_arg(args, "mpc_cornering_stiffness_scale", 1.0),
            derivative_alpha=get_arg(args, "mpc_derivative_alpha", 0.25),
            curvature_filter_alpha=get_arg(args, "mpc_curvature_filter_alpha", 1.0),
            debug_stability=get_arg(args, "debug_mpc_stability", False),
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
