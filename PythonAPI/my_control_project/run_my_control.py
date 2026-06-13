import sys
import os
import random
import argparse
from datetime import datetime

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
PYTHONAPI_ROOT = os.path.dirname(PROJECT_ROOT)
CARLA_PYTHON_ROOT = os.path.join(PYTHONAPI_ROOT, "carla")
if CARLA_PYTHON_ROOT not in sys.path:
    sys.path.insert(0, CARLA_PYTHON_ROOT)

import numpy as np
import carla
import pygame

from control import get_supported_controller_names
from error_providers import get_supported_error_provider_names
from experiment import (
    build_summary,
    configure_world,
    create_display,
    get_vehicle_blueprint,
    resolve_route_setup,
    run_controller_lap,
    save_compare_outputs,
)
from road_planning.route_planner import ROUTE_SHAPES


DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 720
CONTROL_DT = 0.05
LOG_DIR = os.path.join(PROJECT_ROOT, "log")
AUTO_ROUTE_SHAPE = "auto"
DEFAULT_CONTROLLER_ORDER = get_supported_controller_names()


def parse_args():
    parser = argparse.ArgumentParser(description="Run controller comparison on a fixed CARLA route.")
    parser.add_argument(
        "--controllers",
        nargs="+",
        choices=get_supported_controller_names(),
        default=list(DEFAULT_CONTROLLER_ORDER),
        help="Controllers to include in the comparison run.",
    )
    parser.add_argument("--spawn-index", type=int, default=None, help="Optional fixed spawn point index for repeatable tests.")
    parser.add_argument(
        "--destination-index",
        type=int,
        default=None,
        help="Optional fixed destination spawn point index for repeatable tests.",
    )
    parser.add_argument("--target-speed", type=float, default=70.0, help="Target speed in km/h.")
    parser.add_argument(
        "--speed-planner-mode",
        choices=("off", "adaptive"),
        default="adaptive",
        help="Use off for fixed-speed controller-only tests, or adaptive for integrated high-speed safety tests.",
    )
    parser.add_argument("--seed", type=int, default=None, help="Optional random seed used to reproduce a run.")
    parser.add_argument("--route-resolution", type=float, default=1.0, help="Global route waypoint spacing in meters.")
    parser.add_argument(
        "--error-provider",
        choices=get_supported_error_provider_names(),
        default="ground_truth",
        help="Tracking-error source used by the experiment runtime.",
    )
    parser.add_argument(
        "--route-shape",
        choices=(AUTO_ROUTE_SHAPE, *ROUTE_SHAPES),
        default=AUTO_ROUTE_SHAPE,
        help="Geometric route template selected from the CARLA road network.",
    )
    parser.add_argument("--look-ahead", type=float, default=10.0, help="Base look-ahead distance in meters.")
    parser.add_argument(
        "--speed-planner-min-turn-speed",
        type=float,
        default=48.0,
        help="Minimum planned speed in km/h for tight turns.",
    )
    parser.add_argument(
        "--speed-planner-max-lateral-accel",
        type=float,
        default=18.0,
        help="Maximum lateral acceleration in m/s^2 used by curvature speed planning.",
    )
    parser.add_argument(
        "--speed-planner-max-accel",
        type=float,
        default=3.5,
        help="Maximum planned target-speed increase in m/s^2.",
    )
    parser.add_argument(
        "--speed-planner-max-decel",
        type=float,
        default=9.0,
        help="Maximum planned target-speed decrease in m/s^2.",
    )
    parser.add_argument(
        "--speed-planner-lateral-error-warning",
        type=float,
        default=1.6,
        help="Lateral tracking error in meters where protective speed reduction starts.",
    )
    parser.add_argument(
        "--speed-planner-lateral-error-critical",
        type=float,
        default=2.6,
        help="Lateral tracking error in meters where protective speed reduction reaches its minimum.",
    )
    parser.add_argument(
        "--speed-planner-heading-error-warning",
        type=float,
        default=12.0,
        help="Heading error in degrees where protective speed reduction starts.",
    )
    parser.add_argument(
        "--speed-planner-heading-error-critical",
        type=float,
        default=18.0,
        help="Heading error in degrees where protective speed reduction reaches its minimum.",
    )
    parser.add_argument(
        "--speed-planner-lateral-error-rate-warning",
        type=float,
        default=0.8,
        help="Lateral tracking error growth rate in m/s where protective speed reduction starts.",
    )
    parser.add_argument(
        "--speed-planner-lateral-error-rate-critical",
        type=float,
        default=1.6,
        help="Lateral tracking error growth rate in m/s where protective speed reduction reaches its minimum.",
    )
    parser.add_argument(
        "--speed-planner-heading-error-rate-warning",
        type=float,
        default=8.0,
        help="Heading error growth rate in deg/s where protective speed reduction starts.",
    )
    parser.add_argument(
        "--speed-planner-heading-error-rate-critical",
        type=float,
        default=16.0,
        help="Heading error growth rate in deg/s where protective speed reduction reaches its minimum.",
    )
    parser.add_argument(
        "--speed-planner-lateral-error-rate-activation",
        type=float,
        default=0.8,
        help="Minimum lateral error in meters before lateral-error-rate slowdown can trigger.",
    )
    parser.add_argument(
        "--speed-planner-heading-error-rate-activation",
        type=float,
        default=8.0,
        help="Minimum heading error in degrees before heading-error-rate slowdown can trigger.",
    )
    parser.add_argument(
        "--speed-planner-error-rate-alpha",
        type=float,
        default=0.25,
        help="Low-pass blend factor for speed-planner tracking-error growth rates.",
    )
    parser.add_argument(
        "--speed-planner-recovery-hold-steps",
        type=int,
        default=3,
        help="Control steps to keep protective recovery active after risk clears.",
    )
    parser.add_argument(
        "--speed-planner-entry-max-speed",
        type=float,
        default=70.0,
        help="Maximum planned speed in km/h when entering a detected curve at high target speed.",
    )
    parser.add_argument(
        "--speed-planner-entry-curvature-threshold",
        type=float,
        default=0.015,
        help="Preview curvature threshold that activates the entry-corner speed cap.",
    )
    parser.add_argument(
        "--speed-planner-entry-full-cap-curvature",
        type=float,
        default=0.03,
        help="Preview curvature where the entry-corner cap reaches speed-planner-entry-max-speed.",
    )
    parser.add_argument(
        "--collision-zero-speed-timeout",
        type=float,
        default=2.0,
        help="Terminate the current controller after a collision if speed stays near zero for this many seconds.",
    )
    parser.add_argument(
        "--collision-zero-speed-threshold",
        type=float,
        default=0.1,
        help="Speed threshold in m/s used to treat the vehicle as stopped after a collision.",
    )
    parser.add_argument(
        "--route-min-length-m",
        type=float,
        default=1700.0,
        help="Minimum route length in meters before the loop can end.",
    )
    parser.add_argument(
        "--route-max-waypoints",
        type=int,
        default=2000,
        help="Maximum number of waypoints to follow when building the route.",
    )
    parser.add_argument(
        "--route-length-tolerance",
        type=float,
        default=0.05,
        help="Allowed route-length deviation ratio when selecting a shaped route.",
    )
    parser.add_argument(
        "--noise-lateral-std",
        type=float,
        default=0.10,
        help="Standard deviation of lateral-error noise for noisy_ground_truth.",
    )
    parser.add_argument(
        "--noise-heading-std-deg",
        type=float,
        default=1.0,
        help="Standard deviation of heading-error noise in degrees for noisy_ground_truth.",
    )
    parser.add_argument(
        "--perception-delay-steps",
        type=int,
        default=0,
        help="Number of control steps to delay noisy/perception-proxy tracking errors.",
    )
    parser.add_argument(
        "--perception-dropout-probability",
        type=float,
        default=0.0,
        help="Probability of reusing the previous noisy/perception-proxy tracking error sample.",
    )
    parser.add_argument(
        "--perception-smoothing-alpha",
        type=float,
        default=1.0,
        help="Blend factor for noisy/perception-proxy tracking errors; 1.0 disables smoothing.",
    )
    parser.add_argument("--lqr-q-ey", type=float, default=2.6, help="LQR lateral error weight.")
    parser.add_argument("--lqr-q-ey-dot", type=float, default=1.10, help="LQR lateral error derivative weight.")
    parser.add_argument("--lqr-q-epsi", type=float, default=6.0, help="LQR heading error weight.")
    parser.add_argument("--lqr-q-epsi-dot", type=float, default=2.6, help="LQR heading error derivative weight.")
    parser.add_argument("--lqr-r", type=float, default=8.0, help="LQR steering effort weight.")
    parser.add_argument("--lqr-kp-long", type=float, default=0.6, help="LQR controller longitudinal P gain.")
    parser.add_argument("--lqr-ki-long", type=float, default=0.01, help="LQR controller longitudinal I gain.")
    parser.add_argument("--lqr-kd-long", type=float, default=0.06, help="LQR controller longitudinal D gain.")
    parser.add_argument("--lqr-max-steer", type=float, default=0.55, help="Maximum LQR steering command.")
    parser.add_argument(
        "--lqr-max-steer-rate",
        type=float,
        default=0.16,
        help="Maximum LQR steering change per control step.",
    )
    parser.add_argument(
        "--lqr-derivative-alpha",
        type=float,
        default=0.20,
        help="Low-pass blend factor for LQR error derivative states.",
    )
    parser.add_argument(
        "--lqr-curvature-alpha",
        type=float,
        default=0.50,
        help="Low-pass blend factor for LQR curvature feedforward.",
    )
    parser.add_argument(
        "--lqr-feedforward-gain",
        type=float,
        default=1.0,
        help="Gain applied to LQR bicycle-model curvature feedforward.",
    )
    parser.add_argument(
        "--lqr-turn-in-rate-scale",
        type=float,
        default=0.70,
        help="Scale applied to LQR steering-rate limit while adding turn-in near the lane centerline.",
    )
    parser.add_argument(
        "--lqr-turn-in-guard-lateral-error",
        type=float,
        default=1.0,
        help="Maximum lateral error in meters where LQR turn-in rate guarding is active.",
    )
    parser.add_argument(
        "--lqr-turn-in-guard-heading-error",
        type=float,
        default=10.0,
        help="Maximum heading error in degrees where LQR turn-in rate guarding is active.",
    )
    parser.add_argument(
        "--lqr-turn-in-guard-max-curvature",
        type=float,
        default=0.04,
        help="Maximum route curvature where LQR near-centerline turn-in rate guarding is active.",
    )
    parser.add_argument(
        "--lqr-inside-error-feedforward-start",
        type=float,
        default=0.80,
        help="Inside-curve lateral error in meters where LQR curvature feedforward attenuation starts.",
    )
    parser.add_argument(
        "--lqr-inside-error-feedforward-full",
        type=float,
        default=1.80,
        help="Inside-curve lateral error in meters where LQR curvature feedforward reaches its minimum scale.",
    )
    parser.add_argument(
        "--lqr-inside-error-feedforward-min-scale",
        type=float,
        default=0.65,
        help="Minimum LQR curvature feedforward scale when the vehicle is already inside the curve.",
    )
    parser.add_argument(
        "--lqr-inside-error-feedforward-heading-limit",
        type=float,
        default=4.0,
        help="Maximum heading error in degrees where inside-curve LQR feedforward attenuation is allowed.",
    )
    parser.add_argument("--mpc-horizon", type=int, default=16, help="MPC prediction horizon in control steps.")
    parser.add_argument("--mpc-q-y", type=float, default=12.0, help="MPC lateral error weight.")
    parser.add_argument("--mpc-q-psi", type=float, default=18.0, help="MPC heading error weight.")
    parser.add_argument("--mpc-r-steer", type=float, default=0.8, help="MPC steering effort weight.")
    parser.add_argument("--mpc-r-steer-rate", type=float, default=0.9, help="MPC steering-rate effort weight.")
    parser.add_argument("--mpc-kp-long", type=float, default=0.6, help="MPC controller longitudinal P gain.")
    parser.add_argument("--mpc-ki-long", type=float, default=0.01, help="MPC controller longitudinal I gain.")
    parser.add_argument("--mpc-kd-long", type=float, default=0.06, help="MPC controller longitudinal D gain.")
    parser.add_argument("--mpc-max-steer", type=float, default=0.65, help="Maximum MPC steering command.")
    parser.add_argument(
        "--mpc-max-steer-rate",
        type=float,
        default=0.30,
        help="Maximum MPC steering change per control step.",
    )
    parser.add_argument("--pid-lat-kp", type=float, default=0.72, help="PID lateral P gain.")
    parser.add_argument("--pid-lat-ki", type=float, default=0.005, help="PID lateral I gain.")
    parser.add_argument("--pid-lat-kd", type=float, default=0.38, help="PID lateral D gain.")
    parser.add_argument("--pid-long-kp", type=float, default=0.45, help="PID longitudinal P gain.")
    parser.add_argument("--pid-long-ki", type=float, default=0.01, help="PID longitudinal I gain.")
    parser.add_argument("--pid-long-kd", type=float, default=0.10, help="PID longitudinal D gain.")
    parser.add_argument("--pid-max-throttle", type=float, default=1.0, help="Maximum PID throttle command.")
    parser.add_argument("--pid-max-brake", type=float, default=0.28, help="Maximum PID brake command.")
    return parser.parse_args()


def clamp_spawn_index(index, spawn_points):
    return max(0, min(index, len(spawn_points) - 1))


def choose_run_seed(requested_seed):
    if requested_seed is not None:
        return requested_seed
    return random.SystemRandom().randrange(1, 1_000_000_000)


def choose_route_shape(requested_route_shape, rng):
    if requested_route_shape != AUTO_ROUTE_SHAPE:
        return requested_route_shape
    return rng.choice(ROUTE_SHAPES)


def build_run_output_dir(log_dir, run_timestamp, seed, route_shape):
    run_dir_name = f"run_{run_timestamp}_seed_{seed}_{route_shape}"
    return os.path.join(log_dir, run_dir_name)


def build_run_config(args, controller_order, spawn_index, destination_index, destination, route_trace, route_features):
    return {
        "controllers": list(controller_order),
        "destination_index": destination_index,
        "destination_location": {
            "x": destination.x,
            "y": destination.y,
            "z": destination.z,
        },
        "error_provider": args.error_provider,
        "noise_heading_std_deg": args.noise_heading_std_deg,
        "noise_lateral_std": args.noise_lateral_std,
        "perception_delay_steps": args.perception_delay_steps,
        "perception_dropout_probability": args.perception_dropout_probability,
        "perception_smoothing_alpha": args.perception_smoothing_alpha,
        "collision_zero_speed_timeout": args.collision_zero_speed_timeout,
        "collision_zero_speed_threshold": args.collision_zero_speed_threshold,
        "route": {
            "length_m": route_features["length"],
            "route_shape": args.route_shape,
            "route_label": route_features.get("route_label", args.route_shape),
            "sign_changes": route_features["sign_changes"],
            "total_abs_turn_deg": float(np.degrees(route_features["total_abs_turn"])),
            "mean_abs_curvature": route_features.get("mean_abs_curvature", 0.0),
            "max_abs_curvature": route_features.get("max_abs_curvature", 0.0),
            "turn_segments": route_features["turn_segments"],
            "waypoints": len(route_trace),
        },
        "seed": args.seed,
        "spawn_index": spawn_index,
        "target_speed_kmh": args.target_speed,
        "speed_planner": {
            "mode": args.speed_planner_mode,
            "min_turn_speed_kmh": args.speed_planner_min_turn_speed,
            "max_lateral_accel": args.speed_planner_max_lateral_accel,
            "max_accel": args.speed_planner_max_accel,
            "max_decel": args.speed_planner_max_decel,
            "lateral_error_warning": args.speed_planner_lateral_error_warning,
            "lateral_error_critical": args.speed_planner_lateral_error_critical,
            "heading_error_warning_deg": args.speed_planner_heading_error_warning,
            "heading_error_critical_deg": args.speed_planner_heading_error_critical,
            "lateral_error_rate_warning": args.speed_planner_lateral_error_rate_warning,
            "lateral_error_rate_critical": args.speed_planner_lateral_error_rate_critical,
            "heading_error_rate_warning_deg": args.speed_planner_heading_error_rate_warning,
            "heading_error_rate_critical_deg": args.speed_planner_heading_error_rate_critical,
            "lateral_error_rate_activation": args.speed_planner_lateral_error_rate_activation,
            "heading_error_rate_activation_deg": args.speed_planner_heading_error_rate_activation,
            "error_rate_filter_alpha": args.speed_planner_error_rate_alpha,
            "recovery_hold_steps": args.speed_planner_recovery_hold_steps,
            "entry_max_speed_kmh": args.speed_planner_entry_max_speed,
            "entry_curvature_threshold": args.speed_planner_entry_curvature_threshold,
            "entry_full_cap_curvature": args.speed_planner_entry_full_cap_curvature,
        },
        "lqr": {
            "q_ey": args.lqr_q_ey,
            "q_ey_dot": args.lqr_q_ey_dot,
            "q_epsi": args.lqr_q_epsi,
            "q_epsi_dot": args.lqr_q_epsi_dot,
            "r": args.lqr_r,
            "max_steer": args.lqr_max_steer,
            "max_steer_rate": args.lqr_max_steer_rate,
            "derivative_alpha": args.lqr_derivative_alpha,
            "curvature_alpha": args.lqr_curvature_alpha,
            "feedforward_gain": args.lqr_feedforward_gain,
            "turn_in_rate_scale": args.lqr_turn_in_rate_scale,
            "turn_in_guard_lateral_error": args.lqr_turn_in_guard_lateral_error,
            "turn_in_guard_heading_error_deg": args.lqr_turn_in_guard_heading_error,
            "turn_in_guard_max_curvature": args.lqr_turn_in_guard_max_curvature,
            "inside_error_feedforward_start": args.lqr_inside_error_feedforward_start,
            "inside_error_feedforward_full": args.lqr_inside_error_feedforward_full,
            "inside_error_feedforward_min_scale": args.lqr_inside_error_feedforward_min_scale,
            "inside_error_feedforward_heading_limit_deg": args.lqr_inside_error_feedforward_heading_limit,
        },
        "mpc": {
            "horizon": args.mpc_horizon,
            "q_y": args.mpc_q_y,
            "q_psi": args.mpc_q_psi,
            "r_steer": args.mpc_r_steer,
            "r_steer_rate": args.mpc_r_steer_rate,
            "kp_long": args.mpc_kp_long,
            "ki_long": args.mpc_ki_long,
            "kd_long": args.mpc_kd_long,
            "max_steer": args.mpc_max_steer,
            "max_steer_rate": args.mpc_max_steer_rate,
        },
        "pid": {
            "lat_kp": args.pid_lat_kp,
            "lat_ki": args.pid_lat_ki,
            "lat_kd": args.pid_lat_kd,
            "long_kp": args.pid_long_kp,
            "long_ki": args.pid_long_ki,
            "long_kd": args.pid_long_kd,
            "max_throttle": args.pid_max_throttle,
            "max_brake": args.pid_max_brake,
        },
    }


def compare_main():
    args = parse_args()
    controller_order = tuple(args.controllers)
    args.seed = choose_run_seed(args.seed)
    rng = random.Random(args.seed)
    args.route_shape = choose_route_shape(args.route_shape, rng)
    random.seed(args.seed)
    display = create_display("CARLA Controller Comparison", DISPLAY_WIDTH, DISPLAY_HEIGHT)

    client = carla.Client("localhost", 2000)
    client.set_timeout(20.0)
    world = client.get_world()
    original_settings = None

    try:
        original_settings = configure_world(world, CONTROL_DT)
        blueprint_library, vehicle_bp = get_vehicle_blueprint(world)
        spawn_index, destination_index, spawn_point, destination, route_trace, route_features = resolve_route_setup(
            world,
            args,
            clamp_spawn_index,
            rng,
        )
        args.route_label = route_features.get("route_label", args.route_shape)

        print(f"Comparison route built: {len(route_trace)} waypoints.")
        print(
            f"Route template: {args.route_shape}, "
            f"label={args.route_label}, "
            f"length={route_features['length']:.1f} m, "
            f"turn_changes={route_features['sign_changes']}, "
            f"total_turn={np.degrees(route_features['total_abs_turn']):.1f} deg"
        )
        print(f"Error provider: {args.error_provider}")
        print(f"Run seed: {args.seed}")
        print(f"Spawn index: {spawn_index}, destination index: {destination_index}")
        print(f"Destination location: ({destination.x:.1f}, {destination.y:.1f}, {destination.z:.1f})")
        print(f"Experiment order: {', '.join(name.upper() for name in controller_order)}.")

        all_rows = []
        summaries = []
        trajectories = {}

        for controller_name in controller_order:
            rows, xs, ys, metrics, aborted = run_controller_lap(
                controller_name,
                world,
                blueprint_library,
                vehicle_bp,
                spawn_point,
                route_trace,
                args,
                display,
                DISPLAY_WIDTH,
                DISPLAY_HEIGHT,
            )
            all_rows.extend(rows)
            trajectories[controller_name] = (xs, ys)
            summaries.append(build_summary(controller_name, spawn_index, route_trace, args, metrics, route_features))
            if aborted:
                print("Comparison stopped by user before both laps finished.")
                break

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_output_dir = build_run_output_dir(LOG_DIR, timestamp_str, args.seed, args.route_shape)
        run_config = build_run_config(
            args,
            controller_order,
            spawn_index,
            destination_index,
            destination,
            route_trace,
            route_features,
        )
        save_compare_outputs(run_output_dir, all_rows, summaries, trajectories, controller_order, run_config)
    except Exception as exc:
        print(f"An error occurred: {exc}")
    finally:
        if original_settings is not None:
            try:
                world.apply_settings(original_settings)
                print("Server settings restored.")
            except Exception as exc:
                print(f"Failed to restore original settings: {exc}")
        pygame.quit()
        print("Pygame quit.")


if __name__ == "__main__":
    compare_main()
