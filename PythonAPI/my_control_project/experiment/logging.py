import csv
import json
import os

import numpy as np

from project_config import STEP_CONTROLLER_PARAM_FIELDS, SUMMARY_CONTROLLER_PARAM_FIELDS, arg_values

from .metrics import (
    EXPERIMENT_METADATA_HEADER,
    PLANNER_SUMMARY_FIELDS,
    SPEED_PLAN_REASONS,
    SUMMARY_METRIC_FIELDS,
    build_experiment_metadata,
)


SUMMARY_TEXT_COLUMNS = [
    "readable_result",
    "readable_test_conditions",
    "readable_controller_summary",
]

STEP_LOG_FIELDS = [
    "time",
    "x",
    "y",
    "speed",
    "speed_error",
    "e_y",
    "e_psi",
    "abs_e_y",
    "abs_e_psi",
    "steer",
    "steer_delta",
    "raw_steer",
    "limited_steer",
    "steer_rate_limit",
    "steer_rate_limited",
    "controller_runtime_ms",
    "throttle",
    "brake",
    "target_speed",
    "speed_plan_risk",
    "speed_plan_reason",
    "controller_speed_profile",
    "current_curvature",
    "preview_curvature",
    "lqr_gain_update_reason",
    "mpc_solve_mode",
    "mpc_cache_reused",
    "mpc_reuse_count",
    "mpc_curvature0",
    "mpc_curvature_max",
    "mpc_active_horizon",
    "mpc_steer_limit",
    "mpc_reference_steer0",
    "yaw_rate",
    "lateral_accel",
    "longitudinal_accel",
    "lane_clearance",
    "lane_boundary_violation",
    "collision_count",
    "route_completion",
    "route_index",
    "closest_route_index",
    "route_error",
    "road_option",
    "target_x",
    "target_y",
    "target_yaw",
]

REFERENCE_STEP_FIELDS = [
    "planner_mode",
    "trajectory_hash",
    "reference_s_m",
    "target_s_m",
    "projection_segment",
    "projection_distance_m",
    "progress_delta_m",
    "reference_state",
    "reference_held",
    "curvature_preview_s_m",
]


LOG_HEADER = [
    *EXPERIMENT_METADATA_HEADER,
    *STEP_LOG_FIELDS,
    *STEP_CONTROLLER_PARAM_FIELDS,
    *REFERENCE_STEP_FIELDS,
]


SUMMARY_HEADER = [
    *EXPERIMENT_METADATA_HEADER,
    *SUMMARY_METRIC_FIELDS,
    *SUMMARY_CONTROLLER_PARAM_FIELDS,
    *PLANNER_SUMMARY_FIELDS,
    *SUMMARY_TEXT_COLUMNS,
]


BASE_SUMMARY_HEADER = SUMMARY_HEADER[: -len(SUMMARY_TEXT_COLUMNS)]

METRIC_DESCRIPTIONS = {
    "mean_abs_e_y": "mean lateral tracking error",
    "rms_e_y": "RMS lateral tracking error",
    "max_abs_e_y": "maximum lateral tracking error",
    "mean_abs_e_psi_deg": "mean heading error",
    "rms_e_psi_deg": "RMS heading error",
    "mean_abs_speed_error": "mean speed error",
    "mean_abs_steer_delta": "mean steering change",
    "max_abs_steer_delta": "maximum steering jump",
}

SCORE_METRICS = [
    ("mean_abs_e_y", 0.30),
    ("rms_e_y", 0.20),
    ("mean_abs_e_psi_deg", 0.20),
    ("mean_abs_speed_error", 0.15),
    ("mean_abs_steer_delta", 0.10),
    ("max_abs_e_y", 0.05),
]

REASON_DESCRIPTIONS = {
    "none": "no speed reduction was active",
    "fixed_throttle_brake": "fixed target speed tracked by throttle/brake control",
    "entry_curvature": "curve entry was detected ahead, so the target speed was capped before turn-in",
    "curvature": "route curvature was high enough to reduce speed through the curve",
    "lateral_error": "lateral tracking error was large enough to trigger protective speed reduction",
    "heading_error": "heading error was large enough to trigger protective speed reduction",
    "lateral_error_rate": "lateral tracking error was growing quickly",
    "heading_error_rate": "heading error was growing quickly",
    "hold": "the planner briefly held a lower speed after risk cleared",
}


def build_step_snapshot(
    vehicle,
    control,
    target_speed_ms,
    previous_steer,
    tracking_errors,
    speed_plan_risk=0.0,
    speed_plan_reason="none",
    controller_debug=None,
    collision_count=0,
    previous_speed=None,
    dt=0.05,
):
    transform = vehicle.get_transform()
    velocity = vehicle.get_velocity()
    speed = np.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
    location = transform.location
    speed_error = target_speed_ms - speed
    steer_delta = control.steer - previous_steer
    angular_velocity = getattr(vehicle, "get_angular_velocity", None)
    yaw_rate = np.radians(angular_velocity().z) if angular_velocity is not None else 0.0
    lateral_accel = speed * yaw_rate
    longitudinal_accel = 0.0
    if previous_speed is not None and dt > 1e-9:
        longitudinal_accel = (speed - float(previous_speed)) / float(dt)
    lane_clearance = tracking_errors.get("lane_clearance_m", 0.0)
    lane_boundary_violation = bool(tracking_errors.get("lane_boundary_violation", False))
    controller_debug = controller_debug or {}

    return {
        "x": location.x,
        "y": location.y,
        "speed": speed,
        "e_y": tracking_errors["e_y"],
        "e_psi": tracking_errors["e_psi"],
        "speed_error": speed_error,
        "steer_delta": steer_delta,
        "raw_steer": controller_debug.get("raw_steer", control.steer),
        "limited_steer": control.steer,
        "steer_rate_limit": controller_debug.get("steer_rate_limit", 0.0),
        "steer_rate_limited": controller_debug.get("steer_rate_limited", False),
        "controller_runtime_ms": controller_debug.get("controller_runtime_ms", 0.0),
        "target_speed": target_speed_ms,
        "speed_plan_risk": speed_plan_risk,
        "speed_plan_reason": speed_plan_reason,
        "controller_speed_profile": controller_debug.get("controller_speed_profile", "base"),
        "current_curvature": controller_debug.get("current_curvature", 0.0),
        "preview_curvature": controller_debug.get("preview_curvature", 0.0),
        "lqr_gain_update_reason": controller_debug.get("lqr_gain_update_reason", "none"),
        "mpc_solve_mode": controller_debug.get("mpc_solve_mode", "not_applicable"),
        "mpc_cache_reused": controller_debug.get("mpc_cache_reused", False),
        "mpc_reuse_count": controller_debug.get("mpc_reuse_count", 0),
        "mpc_curvature0": controller_debug.get("mpc_curvature0", 0.0),
        "mpc_curvature_max": controller_debug.get("mpc_curvature_max", 0.0),
        "mpc_active_horizon": controller_debug.get("mpc_active_horizon", 0),
        "mpc_steer_limit": controller_debug.get("mpc_steer_limit", 0.0),
        "mpc_reference_steer0": controller_debug.get("mpc_reference_steer0", 0.0),
        "yaw_rate": yaw_rate,
        "lateral_accel": lateral_accel,
        "longitudinal_accel": longitudinal_accel,
        "lane_clearance": lane_clearance,
        "lane_boundary_violation": lane_boundary_violation,
        "collision_count": collision_count,
        "target_x": tracking_errors["target_x"],
        "target_y": tracking_errors["target_y"],
        "target_yaw": tracking_errors["target_yaw"],
    }


def append_step_data(rows, positions_x, positions_y, metrics, controller_name, args, route_state, snapshot, control, sim_time):
    route_index, closest_route_index, route_error, road_option, route_length = route_state
    route_completion = route_index / max(route_length - 1, 1)
    metrics["abs_ey"].append(abs(snapshot["e_y"]))
    metrics["abs_epsi"].append(abs(snapshot["e_psi"]))
    metrics["abs_speed_error"].append(abs(snapshot["speed_error"]))
    metrics["steer_delta"].append(abs(snapshot["steer_delta"]))
    metrics["target_speed"].append(snapshot["target_speed"])
    metrics["abs_yaw_rate"].append(abs(snapshot["yaw_rate"]))
    metrics["abs_lateral_accel"].append(abs(snapshot["lateral_accel"]))
    metrics["time"].append(sim_time)
    metrics["speed"].append(snapshot["speed"])
    metrics["speed_error"].append(snapshot["speed_error"])
    metrics["steer_delta_signed"].append(snapshot["steer_delta"])
    metrics["steer_rate_limit"].append(snapshot["steer_rate_limit"])
    metrics["steer_rate_limited"].append(bool(snapshot["steer_rate_limited"]))
    metrics["controller_runtime_ms"].append(snapshot["controller_runtime_ms"])
    metrics["e_y_signed"].append(snapshot["e_y"])
    metrics["e_psi_signed"].append(snapshot["e_psi"])
    metrics["steer_signed"].append(control.steer)
    metrics["current_curvature"].append(snapshot["current_curvature"])
    metrics["lateral_accel_signed"].append(snapshot["lateral_accel"])
    metrics["longitudinal_accel"].append(snapshot["longitudinal_accel"])
    metrics["throttle"].append(control.throttle)
    metrics["brake"].append(control.brake)
    metrics["lane_clearance"].append(snapshot["lane_clearance"])
    metrics["route_completion"].append(route_completion)
    metrics["collision_count"] = max(metrics.get("collision_count", 0), int(snapshot["collision_count"]))
    if snapshot["lane_boundary_violation"]:
        metrics["lane_boundary_violation_count"] = metrics.get("lane_boundary_violation_count", 0) + 1
    reason_counts = metrics["speed_plan_reason_counts"]
    reason = snapshot["speed_plan_reason"]
    reason_counts[reason] = reason_counts.get(reason, 0) + 1
    positions_x.append(snapshot["x"])
    positions_y.append(snapshot["y"])

    values = {
        **snapshot,
        "time": sim_time,
        "abs_e_y": abs(snapshot["e_y"]),
        "abs_e_psi": abs(snapshot["e_psi"]),
        "steer": control.steer,
        "throttle": control.throttle,
        "brake": control.brake,
        "route_completion": route_completion,
        "route_index": route_index,
        "closest_route_index": closest_route_index,
        "route_error": route_error,
        "road_option": str(road_option),
    }
    rows.append([
        *build_experiment_metadata(controller_name, args),
        *[values[field] for field in STEP_LOG_FIELDS],
        *arg_values(args, STEP_CONTROLLER_PARAM_FIELDS),
        *[values.get(field, "") for field in REFERENCE_STEP_FIELDS],
    ])


def log_step(controller_name, route_trace, route_index, snapshot, control):
    print(
        f"{controller_name.upper()} | route: {route_index}/{len(route_trace) - 1}, "
        f"e_y: {snapshot['e_y']:+.2f} m, "
        f"e_psi: {np.degrees(snapshot['e_psi']):+.1f} deg, "
        f"steer: {control.steer:+.3f}, "
        f"d_steer: {snapshot['steer_delta']:+.3f}, "
        f"speed: {snapshot['speed']:.2f} m/s"
    )


def write_csv(path, header, rows):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)


def _summary_records(summaries):
    records = []
    for row in summaries:
        record = {}
        for index, key in enumerate(BASE_SUMMARY_HEADER):
            record[key] = row[index] if index < len(row) else ""
        records.append(record)
    return records


def _as_float(value, default=0.0):
    try:
        if value == "":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _format_float(value, digits=2, suffix=""):
    number = _as_float(value)
    return f"{number:.{digits}f}{suffix}"


def _format_probability(value):
    return f"{_as_float(value) * 100.0:.1f}%"


def _controller_label(controller_name):
    return str(controller_name).upper()


def _speed_planner_label(mode):
    if mode == "adaptive":
        return "adaptive speed planning based on preview curvature, lateral error, and heading error"
    if mode == "off":
        return "fixed target speed with no adaptive speed reduction"
    return str(mode)


def _error_provider_label(name):
    labels = {
        "ground_truth": "ideal ground-truth tracking error",
        "noisy_ground_truth": "ground-truth tracking error with configured noise",
        "perception_proxy": "perception-proxy tracking error",
    }
    return labels.get(name, str(name))


def _route_difficulty(record):
    total_turn = _as_float(record.get("route_total_abs_turn_deg"))
    max_curvature = _as_float(record.get("route_max_abs_curvature"))
    if total_turn < 20.0 and max_curvature < 0.01:
        return "low, close to a straight route"
    if total_turn < 180.0 and max_curvature < 0.04:
        return "medium, with moderate bends"
    return "high, with clear curves or S-curve behavior"


def _score_summaries(records):
    if not records:
        return []

    metric_max = {
        metric: max(_as_float(record.get(metric)) for record in records)
        for metric, _ in SCORE_METRICS
    }
    scored = []
    for record in records:
        score = 0.0
        for metric, weight in SCORE_METRICS:
            maximum = metric_max[metric]
            value = _as_float(record.get(metric))
            normalized = value / maximum if maximum > 1e-9 else 0.0
            score += normalized * weight
        scored.append((score, record))
    return sorted(scored, key=lambda item: (item[0], str(item[1].get("controller"))))


def _metric_leaders(records):
    leaders = {}
    for metric in METRIC_DESCRIPTIONS:
        leaders[metric] = min(records, key=lambda record: _as_float(record.get(metric)))
    return leaders


def _reason_count(record, reason):
    return int(_as_float(record.get(f"speed_plan_reason_{reason}_count")))


def _dominant_speed_reasons(record):
    counts = [(reason, _reason_count(record, reason)) for reason in SPEED_PLAN_REASONS]
    counts = [(reason, count) for reason, count in counts if count > 0]
    if not counts:
        return "no speed-planner trigger was recorded"

    counts.sort(key=lambda item: item[1], reverse=True)
    total = sum(count for _, count in counts)
    parts = []
    for reason, count in counts[:3]:
        percent = 100.0 * count / total if total else 0.0
        parts.append(f"{reason} {count} times ({percent:.1f}%, {REASON_DESCRIPTIONS.get(reason, reason)})")
    return "; ".join(parts)


def _build_conditions_text(first_record, run_config):
    route_config = run_config.get("route", {}) if isinstance(run_config, dict) else {}
    reference_config = route_config.get("reference_path", {}) if isinstance(route_config, dict) else {}
    scene_coverage = route_config.get("scene_coverage", {}) if isinstance(route_config, dict) else {}
    speed_config = run_config.get("speed_planner", {}) if isinstance(run_config, dict) else {}
    length_m = route_config.get("length_m", first_record.get("route_min_length_m"))
    route_label = route_config.get("route_label", first_record.get("route_label"))
    total_turn = route_config.get("total_abs_turn_deg", first_record.get("route_total_abs_turn_deg"))
    mean_curvature = route_config.get("mean_abs_curvature", first_record.get("route_mean_abs_curvature"))
    max_curvature = route_config.get("max_abs_curvature", first_record.get("route_max_abs_curvature"))
    limit_profile = speed_config.get("limit_profile", first_record.get("speed_planner_limit_profile"))
    seed = run_config.get("seed", "") if isinstance(run_config, dict) else ""
    destination_index = run_config.get("destination_index", "") if isinstance(run_config, dict) else ""
    return (
        f"Test conditions: route {first_record.get('route_shape')} / {route_label}; "
        f"length about {_format_float(length_m, 1, ' m')}; "
        f"{first_record.get('route_waypoints')} waypoints; "
        f"total turn {_format_float(total_turn, 1, ' deg')}; "
        f"mean curvature {_format_float(mean_curvature, 4)}; "
        f"max curvature {_format_float(max_curvature, 4)}; "
        f"difficulty {_route_difficulty(first_record)}; "
        f"reference validation {reference_config.get('validation_passed', first_record.get('reference_validation_passed', ''))}; "
        f"minimum lane clearance {_format_float(reference_config.get('min_lane_clearance_m', first_record.get('reference_min_lane_clearance_m')), 2, ' m')}; "
        f"scene coverage straight {_format_float(scene_coverage.get('straight_m', first_record.get('scene_straight_m')), 1, ' m')}, "
        f"gentle curve {_format_float(scene_coverage.get('gentle_curve_m', first_record.get('scene_gentle_curve_m')), 1, ' m')}, "
        f"moderate curve {_format_float(scene_coverage.get('moderate_curve_m', first_record.get('scene_moderate_curve_m')), 1, ' m')}, "
        f"tight curve {_format_float(scene_coverage.get('tight_curve_m', first_record.get('scene_tight_curve_m')), 1, ' m')}; "
        f"target speed {_format_float(first_record.get('target_speed_kmh'), 1, ' km/h')}; "
        f"speed mode {_speed_planner_label(first_record.get('speed_planner_mode'))}; "
        f"speed-limit profile {limit_profile}; "
        f"error source {_error_provider_label(first_record.get('error_provider'))}; "
        f"perception setup: lateral noise {_format_float(first_record.get('noise_lateral_std'), 3, ' m')}, "
        f"heading noise {_format_float(first_record.get('noise_heading_std_deg'), 2, ' deg')}, "
        f"delay {first_record.get('perception_delay_steps')} steps, "
        f"dropout {_format_probability(first_record.get('perception_dropout_probability'))}, "
        f"smoothing alpha {_format_float(first_record.get('perception_smoothing_alpha'), 2)}; "
        f"spawn index {first_record.get('spawn_index')}; "
        f"destination index {destination_index}; "
        f"seed {seed}"
    )


def _build_result_text(scored_records, records):
    if not scored_records:
        return "No valid controller result was produced, so the best controller cannot be determined."

    best_record = scored_records[0][1]
    best_name = _controller_label(best_record.get("controller"))
    leaders = _metric_leaders(records)
    won_metrics = [
        METRIC_DESCRIPTIONS[metric]
        for metric, leader in leaders.items()
        if leader.get("controller") == best_record.get("controller")
    ]
    reason = ", ".join(won_metrics[:4]) if won_metrics else "the overall metric balance"
    if len(won_metrics) > 4:
        reason += f", and {len(won_metrics) - 4} other metrics"

    ranking = "; ".join(
        f"{rank}. {_controller_label(record.get('controller'))} score {score:.3f}"
        for rank, (score, record) in enumerate(scored_records, start=1)
    )
    return (
        f"Result: {best_name} performed best in this test. "
        f"Reason: {best_name} led or was more stable on {reason}. "
        f"Plain-English interpretation: it stayed closer to the route, kept its heading closer to the road, "
        f"tracked speed well, and changed steering more smoothly. "
        f"Ranking: {ranking}. Lower score is better; the score combines lateral error, heading error, "
        f"speed error, steering smoothness, and maximum route deviation."
    )


def _build_controller_text(record):
    text = (
        f"{_controller_label(record.get('controller'))}: "
        f"mean lateral error {_format_float(record.get('mean_abs_e_y'), 3, ' m')}; "
        f"max lateral error {_format_float(record.get('max_abs_e_y'), 3, ' m')}; "
        f"mean heading error {_format_float(record.get('mean_abs_e_psi_deg'), 2, ' deg')}; "
        f"mean speed error {_format_float(record.get('mean_abs_speed_error'), 3, ' m/s')}; "
        f"mean steering change {_format_float(record.get('mean_abs_steer_delta'), 4)}; "
        f"route completion {_format_float(record.get('route_completion_pct'), 1, '%')}; "
        f"collisions {record.get('collision_count')}; "
        f"lane-boundary violations {record.get('lane_boundary_violation_count')}; "
        f"minimum lane clearance {_format_float(record.get('min_lane_clearance_m'), 2, ' m')}; "
        f"peak lateral acceleration {_format_float(record.get('max_abs_lateral_accel'), 2, ' m/s^2')}; "
        f"planned speed range {_format_float(record.get('min_planned_speed_kmh'), 1, ' km/h')} to "
        f"{_format_float(record.get('max_planned_speed_kmh'), 1, ' km/h')}; "
        f"speed-planner triggers: {_dominant_speed_reasons(record)}"
    )
    return text


def build_human_summary(summaries, run_config):
    records = _summary_records(summaries)
    if not records:
        return ["No valid controller result was produced, so the best controller cannot be determined."]

    scored_records = _score_summaries(records)
    return [
        _build_result_text(scored_records, records),
        _build_conditions_text(records[0], run_config),
        *[_build_controller_text(record) for record in records],
    ]


def build_summary_rows_with_text(summaries, run_config):
    records = _summary_records(summaries)
    if not records:
        return []

    scored_records = _score_summaries(records)
    result_text = _build_result_text(scored_records, records)
    conditions_text = _build_conditions_text(records[0], run_config)
    controller_text_by_name = {
        record.get("controller"): _build_controller_text(record)
        for record in records
    }

    rows = []
    for row, record in zip(summaries, records):
        base_row = list(row[: len(BASE_SUMMARY_HEADER)])
        if len(base_row) < len(BASE_SUMMARY_HEADER):
            base_row.extend([""] * (len(BASE_SUMMARY_HEADER) - len(base_row)))
        rows.append([
            *base_row,
            result_text,
            conditions_text,
            controller_text_by_name.get(record.get("controller"), ""),
        ])
    return rows


def plot_compare_trajectories(trajectories, controller_order, run_output_dir):
    try:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, len(controller_order), figsize=(7 * len(controller_order), 6))
        if len(controller_order) == 1:
            axes = [axes]
        plot_specs = [(name, f"{name.upper()} Trajectory", axes[idx]) for idx, name in enumerate(controller_order)]
        for name, title, ax in plot_specs:
            xs, ys = trajectories.get(name, ([], []))
            ax.plot(xs, ys, "-o", markersize=2, linewidth=1.2)
            ax.set_title(title)
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.axis("equal")
            ax.grid(True)
        plt.suptitle("Vehicle Trajectory Comparison")
        plt.tight_layout()
        figure_path = os.path.join(run_output_dir, "trajectory_compare.png")
        plt.savefig(figure_path, dpi=150)
        print(f"Trajectory comparison figure saved to: {figure_path}")
        plt.close(fig)
    except Exception as exc:
        print(f"Could not display trajectory comparison plot: {exc}")


def write_json(path, payload):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)


def save_compare_outputs(run_output_dir, all_rows, summaries, trajectories, controller_order, run_config):
    os.makedirs(run_output_dir, exist_ok=True)
    compare_path = os.path.join(run_output_dir, "step_log.csv")
    write_csv(compare_path, LOG_HEADER, all_rows)
    print(f"Step log saved to: {compare_path}")

    summary_rows = build_summary_rows_with_text(summaries, run_config)
    summary_path = os.path.join(run_output_dir, "summary.csv")
    write_csv(summary_path, SUMMARY_HEADER, summary_rows)
    print(f"Comparison summary saved to: {summary_path}")

    config_path = os.path.join(run_output_dir, "run_config.json")
    write_json(config_path, run_config)
    print(f"Run config saved to: {config_path}")

    human_summary = build_human_summary(summaries, run_config)
    print("")
    print("\n".join(human_summary[:2]))

    if trajectories:
        plot_compare_trajectories(trajectories, controller_order, run_output_dir)
