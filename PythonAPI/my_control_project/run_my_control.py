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
from project_config import (
    AUTO_MAP_NAME,
    AUTO_ROUTE_SHAPE,
    CLI_ARGUMENTS,
    CONTROL_DT,
    CURRENT_MAP_NAME,
    DISPLAY_HEIGHT,
    DISPLAY_WIDTH,
    MAP_ROUTE_PROFILES,
    ROUTE_SHAPE_MAP_PREFERENCES,
    SPEED_PLANNER_CONFIG_FIELDS,
    SUPPORTED_MAPS,
    args_dict,
    controller_args_dict,
)
from road_planning.route_planner import (
    ROUTE_SHAPES,
    estimate_route_max_waypoints,
    speed_adaptive_route_defaults,
)
from road_planning.reference_trajectory import save_reference_trajectory


LOG_DIR = os.path.join(PROJECT_ROOT, "log")
DEFAULT_CONTROLLER_ORDER = get_supported_controller_names()


def _add_configured_arguments(parser):
    for flags, kwargs in CLI_ARGUMENTS:
        parser.add_argument(*flags, **kwargs)


def parse_args():
    parser = argparse.ArgumentParser(description="Run controller comparison on a fixed CARLA route.")
    parser.add_argument(
        "--controllers",
        nargs="+",
        choices=get_supported_controller_names(),
        default=list(DEFAULT_CONTROLLER_ORDER),
        help="Controllers to include in the comparison run.",
    )
    parser.add_argument(
        "--map-name",
        choices=(AUTO_MAP_NAME, CURRENT_MAP_NAME, *SUPPORTED_MAPS),
        default=AUTO_MAP_NAME,
        help="CARLA map to load. Use auto for speed/route-adaptive map selection, or current to keep the loaded world.",
    )
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
    _add_configured_arguments(parser)
    return parser.parse_args()


def clamp_spawn_index(index, spawn_points):
    return max(0, min(index, len(spawn_points) - 1))


def choose_run_seed(requested_seed):
    if requested_seed is not None:
        return requested_seed
    return random.SystemRandom().randrange(1, 1_000_000_000)


def apply_speed_adaptive_route_settings(args):
    profile = speed_adaptive_route_defaults(args.target_speed)
    requested_route_shape = args.route_shape
    requested_route_length = args.route_min_length_m
    requested_max_waypoints = args.route_max_waypoints

    args.route_shape = (
        requested_route_shape
        if requested_route_shape != AUTO_ROUTE_SHAPE
        else profile["route_shape"]
    )
    args.route_shape_source = "speed_adaptive" if requested_route_shape == AUTO_ROUTE_SHAPE else "manual"

    if requested_route_length is None:
        args.route_min_length_m = profile["min_length_m"]
        args.route_length_source = "speed_adaptive"
    else:
        args.route_length_source = "manual"

    if requested_max_waypoints is None:
        args.route_max_waypoints = estimate_route_max_waypoints(
            args.route_min_length_m,
            args.route_resolution,
            args.route_length_tolerance,
        )
        args.route_max_waypoints_source = "speed_adaptive"
    else:
        args.route_max_waypoints_source = "manual"

    args.route_speed_band = profile["speed_band"]
    args.route_auto_shape = profile["route_shape"]
    args.route_auto_min_length_m = profile["min_length_m"]
    args.route_base_min_length_m = profile["min_length_m"]
    args.route_auto_target_duration_s = profile.get("target_duration_s")
    return args


def _base_map_name(map_name):
    if map_name and map_name.endswith("_Opt"):
        return map_name[:-4]
    return map_name


def _map_route_profile(map_name):
    return MAP_ROUTE_PROFILES.get(_base_map_name(map_name), MAP_ROUTE_PROFILES["Town03"])


def choose_map_name(requested_map_name, route_shape, target_speed_kmh):
    if requested_map_name != AUTO_MAP_NAME:
        return requested_map_name

    preferences = ROUTE_SHAPE_MAP_PREFERENCES.get(route_shape, SUPPORTED_MAPS)
    route_profile = speed_adaptive_route_defaults(target_speed_kmh)
    target_length_m = route_profile["min_length_m"]
    speed_band = route_profile["speed_band"]
    scored = []

    for preference_index, map_name in enumerate(preferences):
        map_profile = _map_route_profile(map_name)
        speed_penalty = 0.0 if speed_band in map_profile["speed_bands"] else 2.0
        capacity_penalty = max(0.0, target_length_m - map_profile["max_auto_length_m"]) / 500.0
        short_route_penalty = max(0.0, map_profile["min_auto_length_m"] - target_length_m) / 800.0
        optimized_penalty = 0.05 if map_name.endswith("_Opt") else 0.0
        score = preference_index * 0.25 + speed_penalty + capacity_penalty + short_route_penalty + optimized_penalty
        scored.append((score, map_name))

    scored.sort(key=lambda item: (item[0], item[1]))
    return scored[0][1]


def apply_map_adaptive_route_settings(args):
    selected_map = getattr(args, "selected_map_name", getattr(args, "map_name", None))
    if not selected_map or selected_map == CURRENT_MAP_NAME:
        return args

    map_profile = _map_route_profile(selected_map)
    args.route_map_length_scale = map_profile["length_scale"]
    args.route_map_min_length_m = map_profile["min_auto_length_m"]
    args.route_map_max_length_m = map_profile["max_auto_length_m"]

    if getattr(args, "route_length_source", None) == "manual":
        return args

    base_length_m = getattr(args, "route_base_min_length_m", None)
    if base_length_m is None:
        base_length_m = getattr(args, "route_min_length_m", None)
    if base_length_m is None:
        return args
    scaled_length_m = base_length_m * map_profile["length_scale"]
    args.route_min_length_m = min(
        max(scaled_length_m, map_profile["min_auto_length_m"]),
        map_profile["max_auto_length_m"],
    )
    args.route_length_source = "speed_map_adaptive"

    if getattr(args, "route_max_waypoints_source", None) != "manual":
        args.route_max_waypoints = estimate_route_max_waypoints(
            args.route_min_length_m,
            getattr(args, "route_resolution", 2.0),
            getattr(args, "route_length_tolerance", 0.05),
        )

    return args


def _short_map_name(world):
    try:
        map_name = world.get_map().name
    except Exception:
        return None
    return os.path.basename(str(map_name).replace("\\", "/"))


def load_selected_world(client, args):
    selected_map = choose_map_name(args.map_name, args.route_shape, args.target_speed)
    args.map_name_source = "speed_adaptive" if args.map_name == AUTO_MAP_NAME else "manual"
    args.requested_map_name = args.map_name

    if selected_map == CURRENT_MAP_NAME:
        world = client.get_world()
        args.map_name = _short_map_name(world) or CURRENT_MAP_NAME
        args.selected_map_name = args.map_name
        args.map_name_source = "current"
        apply_map_adaptive_route_settings(args)
        return world

    world = client.get_world()
    current_map = _short_map_name(world)
    if current_map != selected_map:
        print(f"Loading CARLA map: {selected_map}")
        world = client.load_world(selected_map)
    else:
        print(f"Using already loaded CARLA map: {selected_map}")

    args.map_name = selected_map
    args.selected_map_name = selected_map
    apply_map_adaptive_route_settings(args)
    return world


def build_run_output_dir(log_dir, run_timestamp, seed, route_shape):
    run_dir_name = f"run_{run_timestamp}_seed_{seed}_{route_shape}"
    return os.path.join(log_dir, run_dir_name)


def _location_config(location):
    return {"x": location.x, "y": location.y, "z": location.z}


def _reference_path_config(args, route_trace, route_features):
    return {
        "enabled": route_features.get("reference_path_enabled", False),
        "validation_passed": route_features.get("reference_validation_passed", False),
        "sample_spacing_m": route_features.get("reference_sample_spacing_m", args.reference_sample_spacing),
        "smoothing_window": route_features.get("reference_smoothing_window", args.reference_smoothing_window),
        "lane_margin_m": route_features.get("reference_lane_margin_m", args.reference_lane_margin),
        "sample_count": route_features.get("reference_sample_count", len(route_trace)),
        "length_m": route_features.get("reference_length_m", route_features["length"]),
        "lane_boundary_violation_count": route_features.get("reference_lane_boundary_violation_count", 0),
        "min_lane_clearance_m": route_features.get("reference_min_lane_clearance_m", 0.0),
        "mean_centerline_offset_m": route_features.get("reference_mean_centerline_offset_m", 0.0),
        "max_centerline_offset_m": route_features.get("reference_max_centerline_offset_m", 0.0),
        "mean_smoothing_offset_m": route_features.get("reference_mean_smoothing_offset_m", 0.0),
        "max_smoothing_offset_m": route_features.get("reference_max_smoothing_offset_m", 0.0),
        "mean_abs_curvature": route_features.get("reference_mean_abs_curvature", 0.0),
        "max_abs_curvature": route_features.get("reference_max_abs_curvature", 0.0),
        "min_lane_width_m": route_features.get("reference_min_lane_width_m", 0.0),
        "mean_lane_width_m": route_features.get("reference_mean_lane_width_m", 0.0),
    }


def _scene_coverage_config(route_features):
    return {
        "straight_m": route_features.get("reference_straight_m", 0.0),
        "gentle_curve_m": route_features.get("reference_gentle_curve_m", 0.0),
        "moderate_curve_m": route_features.get("reference_moderate_curve_m", 0.0),
        "tight_curve_m": route_features.get("reference_tight_curve_m", 0.0),
        "s_curve_sign_changes": route_features.get("reference_s_curve_sign_changes", 0),
        "junction_samples": route_features.get("reference_junction_samples", 0),
        "narrow_lane_samples": route_features.get("reference_narrow_lane_samples", 0),
    }


def _route_config(args, route_trace, route_features):
    return {
        "length_m": route_features["length"],
        "raw_length_m": route_features.get("raw_route_length", route_features["length"]),
        "route_shape": args.route_shape,
        "route_label": route_features.get("route_label", args.route_shape),
        "route_shape_source": getattr(args, "route_shape_source", "manual"),
        "route_length_source": getattr(args, "route_length_source", "manual"),
        "route_speed_band": getattr(args, "route_speed_band", None),
        "route_auto_shape": getattr(args, "route_auto_shape", args.route_shape),
        "route_auto_min_length_m": getattr(args, "route_auto_min_length_m", args.route_min_length_m),
        "route_auto_target_duration_s": getattr(args, "route_auto_target_duration_s", None),
        "route_base_min_length_m": getattr(args, "route_base_min_length_m", args.route_min_length_m),
        "route_map_length_scale": getattr(args, "route_map_length_scale", None),
        "route_map_min_length_m": getattr(args, "route_map_min_length_m", None),
        "route_map_max_length_m": getattr(args, "route_map_max_length_m", None),
        "route_max_waypoints": args.route_max_waypoints,
        "raw_waypoints": route_features.get("raw_route_waypoints", len(route_trace)),
        "sign_changes": route_features["sign_changes"],
        "total_abs_turn_deg": float(np.degrees(route_features["total_abs_turn"])),
        "mean_abs_curvature": route_features.get("mean_abs_curvature", 0.0),
        "max_abs_curvature": route_features.get("max_abs_curvature", 0.0),
        "turn_segments": route_features["turn_segments"],
        "waypoints": len(route_trace),
        "reference_path": _reference_path_config(args, route_trace, route_features),
        "scene_coverage": _scene_coverage_config(route_features),
    }


def _planner_run_config(args, route_features, trajectory):
    metadata = dict(getattr(trajectory, "metadata", {}) or {})
    requested_mode = route_features.get("planner_mode_requested", args.planner_mode)
    resolved_mode = route_features.get("planner_mode_resolved", args.planner_mode)
    return {
        "mode": resolved_mode,
        "requested_mode": requested_mode,
        "resolved_mode": resolved_mode,
        "fallback_policy": route_features.get(
            "planner_fallback_policy",
            getattr(args, "planner_fallback", "legacy"),
        ),
        "fallback_used": bool(route_features.get("planner_fallback_used", False)),
        "fallback_code": route_features.get("planner_fallback_code", ""),
        "fallback_message": route_features.get("planner_fallback_message", ""),
        "fallback_rejection_counts": route_features.get(
            "planner_fallback_rejection_counts",
            {},
        ),
        "version": int(metadata.get("planner_version", 1)),
        "trajectory_hash": trajectory.content_hash(),
        "planning_duration_s": float(route_features.get("planning_duration_s", 0.0)),
        "config": {
            "output_spacing_m": args.reference_sample_spacing,
            "validation_spacing_m": args.planner_validation_spacing,
            "vehicle_half_width_m": args.planner_vehicle_half_width,
            "lane_margin_m": args.planner_lane_margin,
            "topology_beam_width": args.planner_topology_beam_width,
            "lateral_beam_width": args.planner_lateral_beam_width,
            "candidate_cap": args.planner_candidate_cap,
            "deadline_s": args.planner_deadline,
            "max_abs_curvature_1pm": args.planner_max_curvature,
            "max_abs_curvature_rate_1pm2": args.planner_max_curvature_rate,
            "max_lateral_accel_mps2": args.planner_max_lateral_accel,
            "max_lateral_jerk_mps3": args.planner_max_lateral_jerk,
        },
    }


def assert_frozen_trajectory(trajectory, frozen_hash):
    current_hash = trajectory.content_hash()
    if current_hash != frozen_hash:
        raise RuntimeError(
            "frozen reference trajectory hash mismatch: "
            f"expected {frozen_hash}, got {current_hash}"
        )


def build_run_config(args, controller_order, spawn_index, destination_index, destination, route_trace, route_features, trajectory):
    return {
        "controllers": list(controller_order),
        "destination_index": destination_index,
        "destination_location": _location_config(destination),
        "error_provider": args.error_provider,
        "carla": {
            "host": args.carla_host,
            "port": args.carla_port,
            "timeout": args.carla_timeout,
        },
        "map": {
            "name": getattr(args, "selected_map_name", getattr(args, "map_name", None)),
            "requested": getattr(args, "requested_map_name", getattr(args, "map_name", None)),
            "source": getattr(args, "map_name_source", "manual"),
        },
        "noise_heading_std_deg": args.noise_heading_std_deg,
        "noise_lateral_std": args.noise_lateral_std,
        "perception_delay_steps": args.perception_delay_steps,
        "perception_dropout_probability": args.perception_dropout_probability,
        "perception_smoothing_alpha": args.perception_smoothing_alpha,
        "collision_zero_speed_timeout": args.collision_zero_speed_timeout,
        "collision_zero_speed_threshold": args.collision_zero_speed_threshold,
        "route": _route_config(args, route_trace, route_features),
        "planner": _planner_run_config(args, route_features, trajectory),
        "tracker": {
            "max_rollback_m": args.tracker_max_rollback,
            "hold_steps": args.tracker_hold_steps,
        },
        "seed": args.seed,
        "spawn_index": spawn_index,
        "target_speed_kmh": args.target_speed,
        "speed_planner": args_dict(args, SPEED_PLANNER_CONFIG_FIELDS),
        "lqr": controller_args_dict(args, "lqr"),
        "mpc": controller_args_dict(args, "mpc"),
        "pid": controller_args_dict(args, "pid"),
    }


def compare_main():
    args = parse_args()
    controller_order = tuple(args.controllers)
    args.seed = choose_run_seed(args.seed)
    rng = random.Random(args.seed)
    apply_speed_adaptive_route_settings(args)
    random.seed(args.seed)
    display = create_display("CARLA Controller Comparison", DISPLAY_WIDTH, DISPLAY_HEIGHT)

    client = carla.Client(args.carla_host, args.carla_port)
    client.set_timeout(args.carla_timeout)
    world = load_selected_world(client, args)
    original_settings = None

    try:
        original_settings = configure_world(world, CONTROL_DT)
        blueprint_library, vehicle_bp = get_vehicle_blueprint(world)
        spawn_index, destination_index, spawn_point, destination, route_trace, route_features, trajectory = resolve_route_setup(
            world,
            args,
            clamp_spawn_index,
            rng,
        )
        args.route_label = route_features.get("route_label", args.route_shape)

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_output_dir = build_run_output_dir(LOG_DIR, timestamp_str, args.seed, args.route_shape)
        os.makedirs(run_output_dir, exist_ok=True)
        frozen_hash = trajectory.content_hash()
        save_reference_trajectory(
            os.path.join(run_output_dir, "reference_trajectory.json"),
            trajectory,
            execution_metadata={
                "planning_duration_s": float(route_features.get("planning_duration_s", 0.0)),
                "seed": args.seed,
            },
        )
        run_config = build_run_config(
            args,
            controller_order,
            spawn_index,
            destination_index,
            destination,
            route_trace,
            route_features,
            trajectory,
        )

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
            assert_frozen_trajectory(trajectory, frozen_hash)
            rows, xs, ys, metrics, aborted = run_controller_lap(
                controller_name,
                world,
                blueprint_library,
                vehicle_bp,
                spawn_point,
                route_trace,
                trajectory,
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

        save_compare_outputs(run_output_dir, all_rows, summaries, trajectories, controller_order, run_config)
    except Exception as exc:
        print(f"An error occurred: {exc}")
        raise
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
