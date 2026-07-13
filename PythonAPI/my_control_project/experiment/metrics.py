import json

import numpy as np

from project_config import SUMMARY_CONTROLLER_PARAM_FIELDS, arg_values

from .stability import analyze_stability


EXPERIMENT_METADATA_HEADER = [
    "controller",
    "error_provider",
    "speed_planner_mode",
    "speed_planner_limit_profile",
    "route_shape",
    "route_label",
    "route_min_length_m",
    "route_length_tolerance",
    "noise_lateral_std",
    "noise_heading_std_deg",
    "perception_delay_steps",
    "perception_dropout_probability",
    "perception_smoothing_alpha",
]

SPEED_PLAN_REASONS = [
    "none",
    "fixed_throttle_brake",
    "entry_curvature",
    "curvature",
    "lateral_error",
    "heading_error",
    "lateral_error_rate",
    "heading_error_rate",
    "hold",
]

SUMMARY_METRIC_FIELDS = [
    "spawn_index",
    "route_waypoints",
    "target_speed_kmh",
    "route_total_abs_turn_rad",
    "route_total_abs_turn_deg",
    "route_mean_abs_curvature",
    "route_max_abs_curvature",
    "reference_validation_passed",
    "reference_min_lane_clearance_m",
    "reference_max_centerline_offset_m",
    "scene_straight_m",
    "scene_gentle_curve_m",
    "scene_moderate_curve_m",
    "scene_tight_curve_m",
    "scene_s_curve_sign_changes",
    "scene_junction_samples",
    "mean_abs_e_y",
    "rms_e_y",
    "max_abs_e_y",
    "mean_abs_e_psi_deg",
    "rms_e_psi_deg",
    "mean_abs_speed_error",
    "mean_abs_steer_delta",
    "max_abs_steer_delta",
    "route_completion_pct",
    "collision_count",
    "lane_boundary_violation_count",
    "min_lane_clearance_m",
    "mean_abs_yaw_rate_deg_s",
    "max_abs_yaw_rate_deg_s",
    "mean_abs_lateral_accel",
    "max_abs_lateral_accel",
    "mean_planned_speed_kmh",
    "min_planned_speed_kmh",
    "max_planned_speed_kmh",
    "p95_abs_steer_delta",
    "significant_steer_reversals_per_10s",
    "max_significant_steer_reversals_2s",
    "steer_rate_limit_hit_pct",
    "p95_abs_speed_error",
    "p95_abs_lateral_accel",
    "p95_abs_lateral_jerk",
    "p95_abs_longitudinal_jerk",
    "throttle_brake_switches_per_10s",
    "stability_passed",
    "stability_fail_reasons",
    "stability_fail_windows",
    *[f"speed_plan_reason_{reason}_count" for reason in SPEED_PLAN_REASONS],
]

PLANNER_SUMMARY_FIELDS = [
    "planner_mode",
    "planner_mode_requested",
    "planner_mode_resolved",
    "planner_fallback_policy",
    "planner_fallback_used",
    "planner_fallback_code",
    "planner_fallback_message",
    "planner_fallback_rejection_counts",
    "planner_version",
    "trajectory_hash",
    "planning_duration_s",
    "planner_candidate_count",
    "planner_selected_candidate_id",
    "reference_max_abs_curvature",
    "reference_max_abs_curvature_rate",
    "reference_min_footprint_clearance_m",
    "planner_rejection_counts",
]


def mean(values):
    return sum(values) / len(values) if values else 0.0


def rms(values):
    return float(np.sqrt(mean([value * value for value in values]))) if values else 0.0


def create_metrics():
    return {
        "abs_ey": [],
        "abs_epsi": [],
        "abs_speed_error": [],
        "steer_delta": [],
        "target_speed": [],
        "abs_yaw_rate": [],
        "abs_lateral_accel": [],
        "time": [],
        "speed": [],
        "speed_error": [],
        "steer_delta_signed": [],
        "steer_rate_limit": [],
        "steer_rate_limited": [],
        "lateral_accel_signed": [],
        "longitudinal_accel": [],
        "throttle": [],
        "brake": [],
        "lane_clearance": [],
        "route_completion": [],
        "collision_count": 0,
        "lane_boundary_violation_count": 0,
        "speed_plan_reason_counts": {reason: 0 for reason in SPEED_PLAN_REASONS},
    }


def resolve_route_label(args):
    return getattr(args, "route_label", args.route_shape)


def build_experiment_metadata(controller_name, args):
    speed_limit_profile = getattr(args, "speed_planner_limit_profile", "controller")
    if speed_limit_profile == "controller":
        speed_limit_profile = f"controller:{controller_name}"
    return [
        controller_name,
        args.error_provider,
        args.speed_planner_mode,
        speed_limit_profile,
        args.route_shape,
        resolve_route_label(args),
        args.route_min_length_m,
        args.route_length_tolerance,
        args.noise_lateral_std,
        args.noise_heading_std_deg,
        args.perception_delay_steps,
        args.perception_dropout_probability,
        args.perception_smoothing_alpha,
    ]


def build_summary(controller_name, spawn_index, route_trace, args, metrics, route_features=None):
    abs_ey = metrics["abs_ey"]
    abs_epsi = metrics["abs_epsi"]
    abs_speed_error = metrics["abs_speed_error"]
    steer_delta = metrics["steer_delta"]
    target_speed = metrics["target_speed"]
    abs_yaw_rate = metrics.get("abs_yaw_rate", [])
    abs_lateral_accel = metrics.get("abs_lateral_accel", [])
    lane_clearance = metrics.get("lane_clearance", [])
    route_completion = metrics.get("route_completion", [])
    reason_counts = metrics["speed_plan_reason_counts"]
    route_features = route_features or {}
    stability = analyze_stability(
        metrics,
        speed_planner_mode=args.speed_planner_mode,
        route_shape=args.route_shape,
        error_provider=args.error_provider,
    )
    values = {
        "spawn_index": spawn_index,
        "route_waypoints": len(route_trace),
        "target_speed_kmh": args.target_speed,
        "route_total_abs_turn_rad": route_features.get("total_abs_turn", 0.0),
        "route_total_abs_turn_deg": np.degrees(route_features.get("total_abs_turn", 0.0)),
        "route_mean_abs_curvature": route_features.get("mean_abs_curvature", 0.0),
        "route_max_abs_curvature": route_features.get("max_abs_curvature", 0.0),
        "reference_validation_passed": route_features.get("reference_validation_passed", False),
        "reference_min_lane_clearance_m": route_features.get("reference_min_lane_clearance_m", 0.0),
        "reference_max_centerline_offset_m": route_features.get("reference_max_centerline_offset_m", 0.0),
        "scene_straight_m": route_features.get("reference_straight_m", 0.0),
        "scene_gentle_curve_m": route_features.get("reference_gentle_curve_m", 0.0),
        "scene_moderate_curve_m": route_features.get("reference_moderate_curve_m", 0.0),
        "scene_tight_curve_m": route_features.get("reference_tight_curve_m", 0.0),
        "scene_s_curve_sign_changes": route_features.get("reference_s_curve_sign_changes", 0),
        "scene_junction_samples": route_features.get("reference_junction_samples", 0),
        "mean_abs_e_y": mean(abs_ey),
        "rms_e_y": rms(abs_ey),
        "max_abs_e_y": max(abs_ey or [0.0]),
        "mean_abs_e_psi_deg": np.degrees(mean(abs_epsi)),
        "rms_e_psi_deg": np.degrees(rms(abs_epsi)),
        "mean_abs_speed_error": mean(abs_speed_error),
        "mean_abs_steer_delta": mean(steer_delta),
        "max_abs_steer_delta": max(steer_delta or [0.0]),
        "route_completion_pct": max(route_completion or [0.0]) * 100.0,
        "collision_count": metrics.get("collision_count", 0),
        "lane_boundary_violation_count": metrics.get("lane_boundary_violation_count", 0),
        "min_lane_clearance_m": min(lane_clearance or [0.0]),
        "mean_abs_yaw_rate_deg_s": np.degrees(mean(abs_yaw_rate)),
        "max_abs_yaw_rate_deg_s": np.degrees(max(abs_yaw_rate or [0.0])),
        "mean_abs_lateral_accel": mean(abs_lateral_accel),
        "max_abs_lateral_accel": max(abs_lateral_accel or [0.0]),
        "mean_planned_speed_kmh": mean(target_speed) * 3.6,
        "min_planned_speed_kmh": min(target_speed or [0.0]) * 3.6,
        "max_planned_speed_kmh": max(target_speed or [0.0]) * 3.6,
        **stability,
        **{f"speed_plan_reason_{reason}_count": reason_counts.get(reason, 0) for reason in SPEED_PLAN_REASONS},
        "planner_mode": route_features.get(
            "planner_mode_resolved",
            route_features.get("planner_mode", getattr(args, "planner_mode", "legacy")),
        ),
        "planner_mode_requested": route_features.get(
            "planner_mode_requested",
            getattr(args, "planner_mode", "legacy"),
        ),
        "planner_mode_resolved": route_features.get(
            "planner_mode_resolved",
            route_features.get("planner_mode", getattr(args, "planner_mode", "legacy")),
        ),
        "planner_fallback_policy": route_features.get(
            "planner_fallback_policy",
            getattr(args, "planner_fallback", "legacy"),
        ),
        "planner_fallback_used": bool(route_features.get("planner_fallback_used", False)),
        "planner_fallback_code": route_features.get("planner_fallback_code", ""),
        "planner_fallback_message": route_features.get("planner_fallback_message", ""),
        "planner_fallback_rejection_counts": json.dumps(
            route_features.get("planner_fallback_rejection_counts", {}),
            sort_keys=True,
            separators=(",", ":"),
        ),
        "planner_version": route_features.get("planner_version", 1),
        "trajectory_hash": route_features.get("trajectory_hash", ""),
        "planning_duration_s": route_features.get("planning_duration_s", 0.0),
        "planner_candidate_count": route_features.get("candidate_count", 0),
        "planner_selected_candidate_id": route_features.get("selected_candidate_id", ""),
        "reference_max_abs_curvature": route_features.get("reference_max_abs_curvature", 0.0),
        "reference_max_abs_curvature_rate": route_features.get("reference_max_abs_curvature_rate", 0.0),
        "reference_min_footprint_clearance_m": route_features.get("reference_min_footprint_clearance_m", 0.0),
        "planner_rejection_counts": json.dumps(
            route_features.get("rejection_counts", {}), sort_keys=True, separators=(",", ":")
        ),
    }
    return [
        *build_experiment_metadata(controller_name, args),
        *[values[field] for field in SUMMARY_METRIC_FIELDS],
        *arg_values(args, SUMMARY_CONTROLLER_PARAM_FIELDS),
        *[values[field] for field in PLANNER_SUMMARY_FIELDS],
    ]
