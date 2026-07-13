from dataclasses import asdict, dataclass
import math
import time

import numpy as np

from .reference_trajectory import ReferenceTrajectory


PLANNER_VERSION = 1
SCORE_WEIGHTS = {
    "length": 3.0,
    "shape": 1.0,
    "offset": 0.5,
    "curvature": 2.0,
    "curvature_rate": 1.5,
    "inverse_clearance": 2.0,
    "lateral_acceleration": 3.0,
}


@dataclass(frozen=True)
class FrenetPlannerConfig:
    output_spacing_m: float = 1.0
    validation_spacing_m: float = 0.25
    vehicle_half_width_m: float = 1.05
    lane_margin_m: float = 0.35
    offset_fractions: tuple = (-0.75, -0.50, -0.25, 0.0, 0.25, 0.50, 0.75)
    transition_lengths_m: tuple = (20.0, 40.0, 60.0)
    lateral_beam_width: int = 64
    candidate_cap: int = 512
    deadline_s: float = 5.0
    max_abs_curvature_1pm: float = 0.20
    max_abs_curvature_rate_1pm2: float = 0.020
    max_lateral_accel_mps2: float = 6.5
    max_lateral_jerk_mps3: float = 10.0


@dataclass(frozen=True)
class CenterlineModel:
    s_m: np.ndarray
    x_m: np.ndarray
    y_m: np.ndarray
    z_m: np.ndarray
    yaw_rad: np.ndarray
    curvature_1pm: np.ndarray
    lane_width_m: np.ndarray
    source_index: np.ndarray
    road_options: tuple


@dataclass(frozen=True)
class _Candidate:
    candidate_id: str
    offset_m: np.ndarray


@dataclass(frozen=True)
class _EvaluatedCandidate:
    candidate_id: str
    offset_m: np.ndarray
    x_m: np.ndarray
    y_m: np.ndarray
    yaw_rad: np.ndarray
    curvature_1pm: np.ndarray
    curvature_rate_1pm2: np.ndarray
    left_clearance_m: np.ndarray
    right_clearance_m: np.ndarray
    speed_cap_mps: np.ndarray
    score_components: dict
    total_score: float


class FrenetPlanningFailure(RuntimeError):
    def __init__(self, code, rejection_counts, message):
        super().__init__(message)
        self.code = str(code)
        self.rejection_counts = dict(rejection_counts)


def _road_option_name(value):
    name = getattr(value, "name", None)
    if name:
        return str(name)
    return str(value).rsplit(".", 1)[-1]


def quintic_transition(s_values, start_s, end_s, start_offset, end_offset):
    s_values = np.asarray(s_values, dtype=float)
    span = max(float(end_s) - float(start_s), 1e-9)
    ratio = np.clip((s_values - float(start_s)) / span, 0.0, 1.0)
    smooth = 6.0 * ratio ** 5 - 15.0 * ratio ** 4 + 10.0 * ratio ** 3
    return float(start_offset) + (float(end_offset) - float(start_offset)) * smooth


def _remove_duplicate_points(route_trace):
    kept = []
    for waypoint, road_option in route_trace:
        location = waypoint.transform.location
        point = np.array([float(location.x), float(location.y), float(getattr(location, "z", 0.0))])
        if kept and np.linalg.norm(point[:2] - kept[-1][0][:2]) < 1e-4:
            continue
        kept.append((point, waypoint, road_option))
    if len(kept) < 2:
        raise FrenetPlanningFailure("invalid_centerline", {}, "Route has fewer than two distinct points.")
    return kept


def build_centerline_model(route_trace, config=None):
    config = config or FrenetPlannerConfig()
    kept = _remove_duplicate_points(route_trace)
    points = np.asarray([item[0] for item in kept], dtype=float)
    segment_lengths = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    raw_s = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    spacing = max(float(config.output_spacing_m), 0.1)
    s_values = np.arange(0.0, raw_s[-1], spacing)
    if not len(s_values) or s_values[-1] < raw_s[-1] - 1e-9:
        s_values = np.append(s_values, raw_s[-1])

    x_values = np.interp(s_values, raw_s, points[:, 0])
    y_values = np.interp(s_values, raw_s, points[:, 1])
    z_values = np.interp(s_values, raw_s, points[:, 2])
    dx = np.gradient(x_values, s_values, edge_order=1)
    dy = np.gradient(y_values, s_values, edge_order=1)
    yaw_values = np.unwrap(np.arctan2(dy, dx))
    curvature = np.gradient(yaw_values, s_values, edge_order=1)

    lane_width_raw = np.asarray([float(getattr(item[1], "lane_width", 3.5) or 3.5) for item in kept])
    lane_width = np.interp(s_values, raw_s, lane_width_raw)
    nearest_raw = np.searchsorted(raw_s, s_values, side="right") - 1
    nearest_raw = np.clip(nearest_raw, 0, len(kept) - 1)
    source_index = np.asarray(
        [int(getattr(kept[index][1], "source_index", index)) for index in nearest_raw],
        dtype=np.int64,
    )
    road_options = tuple(_road_option_name(kept[index][2]) for index in nearest_raw)
    return CenterlineModel(
        s_m=s_values,
        x_m=x_values,
        y_m=y_values,
        z_m=z_values,
        yaw_rad=yaw_values,
        curvature_1pm=curvature,
        lane_width_m=lane_width,
        source_index=source_index,
        road_options=road_options,
    )


def controller_neutral_speed_cap(requested_speed_mps, curvature_1pm, curvature_rate_1pm2, config=None):
    config = config or FrenetPlannerConfig()
    limits = [max(float(requested_speed_mps), 0.0)]
    if abs(float(curvature_1pm)) > 1e-9:
        limits.append(math.sqrt(config.max_lateral_accel_mps2 / abs(float(curvature_1pm))))
    if abs(float(curvature_rate_1pm2)) > 1e-9:
        limits.append((config.max_lateral_jerk_mps3 / abs(float(curvature_rate_1pm2))) ** (1.0 / 3.0))
    return float(min(limits))


def _curve_windows(centerline):
    active = np.abs(centerline.curvature_1pm) >= 0.005
    indices = np.flatnonzero(active)
    if not len(indices):
        return []
    windows = []
    start = previous = int(indices[0])
    for index in indices[1:]:
        index = int(index)
        if centerline.s_m[index] - centerline.s_m[previous] > 10.0:
            windows.append((start, previous))
            start = index
        previous = index
    windows.append((start, previous))
    return windows


def _offset_profile(centerline, fraction, transition_length, config):
    offsets = np.zeros_like(centerline.s_m)
    usable = np.maximum(
        centerline.lane_width_m * 0.5 - config.vehicle_half_width_m - config.lane_margin_m,
        0.0,
    )
    for start_index, end_index in _curve_windows(centerline):
        start_s = float(centerline.s_m[start_index])
        end_s = float(centerline.s_m[end_index])
        mean_curvature = float(np.mean(centerline.curvature_1pm[start_index:end_index + 1]))
        direction = 1.0 if mean_curvature >= 0.0 else -1.0
        target = direction * float(fraction) * float(np.min(usable[start_index:end_index + 1]))
        ramp_in_start = max(float(centerline.s_m[0]), start_s - float(transition_length))
        ramp_out_end = min(float(centerline.s_m[-1]), end_s + float(transition_length))
        incoming = quintic_transition(centerline.s_m, ramp_in_start, start_s, 0.0, target)
        outgoing = quintic_transition(centerline.s_m, end_s, ramp_out_end, target, 0.0)
        window_offset = np.where(centerline.s_m < start_s, incoming, target)
        window_offset = np.where(centerline.s_m > end_s, outgoing, window_offset)
        window_offset = np.where(
            (centerline.s_m >= ramp_in_start) & (centerline.s_m <= ramp_out_end),
            window_offset,
            0.0,
        )
        offsets += window_offset
    return np.clip(offsets, -usable, usable)


def generate_lateral_candidates(centerline, config):
    if not _curve_windows(centerline):
        return [_Candidate("center", np.zeros_like(centerline.s_m))]
    candidates = []
    for fraction in config.offset_fractions:
        for transition in config.transition_lengths_m:
            candidate_id = "offset_{:+.2f}_transition_{:05.1f}".format(float(fraction), float(transition))
            candidates.append(
                _Candidate(candidate_id, _offset_profile(centerline, fraction, transition, config))
            )
    candidates.sort(key=lambda item: item.candidate_id)
    return candidates[:max(int(config.candidate_cap), 1)]


def _candidate_geometry(centerline, offset_m, spacing_m):
    dense_s = np.arange(centerline.s_m[0], centerline.s_m[-1], max(float(spacing_m), 0.05))
    if dense_s[-1] < centerline.s_m[-1] - 1e-9:
        dense_s = np.append(dense_s, centerline.s_m[-1])
    center_x = np.interp(dense_s, centerline.s_m, centerline.x_m)
    center_y = np.interp(dense_s, centerline.s_m, centerline.y_m)
    center_yaw = np.interp(dense_s, centerline.s_m, centerline.yaw_rad)
    dense_offset = np.interp(dense_s, centerline.s_m, offset_m)
    x_values = center_x - dense_offset * np.sin(center_yaw)
    y_values = center_y + dense_offset * np.cos(center_yaw)
    dx = np.gradient(x_values, dense_s, edge_order=1)
    dy = np.gradient(y_values, dense_s, edge_order=1)
    yaw_values = np.unwrap(np.arctan2(dy, dx))
    curvature = np.gradient(yaw_values, dense_s, edge_order=1)
    curvature_rate = np.gradient(curvature, dense_s, edge_order=1)
    return dense_s, dense_offset, x_values, y_values, yaw_values, curvature, curvature_rate


def _increment(rejections, reason):
    rejections[reason] = rejections.get(reason, 0) + 1


def _evaluate_candidate(centerline, candidate, target_speed_mps, route_features, config, rejections):
    geometry = _candidate_geometry(centerline, candidate.offset_m, config.validation_spacing_m)
    dense_s, dense_offset, _, _, _, dense_curvature, dense_rate = geometry
    dense_width = np.interp(dense_s, centerline.s_m, centerline.lane_width_m)
    footprint_left = dense_width * 0.5 - dense_offset - config.vehicle_half_width_m
    footprint_right = dense_width * 0.5 + dense_offset - config.vehicle_half_width_m
    if float(np.min(np.minimum(footprint_left, footprint_right))) < config.lane_margin_m - 1e-9:
        _increment(rejections, "lane_clearance")
        return None
    if float(np.max(np.abs(dense_curvature))) > config.max_abs_curvature_1pm + 1e-9:
        _increment(rejections, "curvature")
        return None
    if float(np.max(np.abs(dense_rate))) > config.max_abs_curvature_rate_1pm2 + 1e-9:
        _increment(rejections, "curvature_rate")
        return None

    output_s = centerline.s_m
    output_offset = candidate.offset_m
    center_yaw = centerline.yaw_rad
    x_values = centerline.x_m - output_offset * np.sin(center_yaw)
    y_values = centerline.y_m + output_offset * np.cos(center_yaw)
    dx = np.gradient(x_values, output_s, edge_order=1)
    dy = np.gradient(y_values, output_s, edge_order=1)
    yaw_values = np.unwrap(np.arctan2(dy, dx))
    curvature = np.gradient(yaw_values, output_s, edge_order=1)
    curvature_rate = np.gradient(curvature, output_s, edge_order=1)
    left_clearance = centerline.lane_width_m * 0.5 - output_offset
    right_clearance = centerline.lane_width_m * 0.5 + output_offset
    speed_cap = np.asarray([
        controller_neutral_speed_cap(target_speed_mps, curvature[index], curvature_rate[index], config)
        for index in range(len(output_s))
    ])

    requested_length = max(float(route_features.get("length", output_s[-1])), 1e-6)
    length_term = abs(float(output_s[-1]) - requested_length) / requested_length
    shape_term = 0.0
    offset_term = float(np.mean(np.abs(output_offset))) / max(float(np.mean(centerline.lane_width_m * 0.5)), 1e-6)
    curvature_term = float(np.mean((curvature / config.max_abs_curvature_1pm) ** 2))
    rate_term = float(np.mean((curvature_rate / config.max_abs_curvature_rate_1pm2) ** 2))
    min_footprint_clearance = float(np.min(np.minimum(left_clearance, right_clearance)) - config.vehicle_half_width_m)
    clearance_term = max(0.0, config.lane_margin_m / max(min_footprint_clearance, 1e-6) - 1.0)
    requested_lateral_accel = target_speed_mps ** 2 * np.abs(curvature)
    accel_term = float(np.mean(np.maximum(requested_lateral_accel / config.max_lateral_accel_mps2 - 1.0, 0.0)))
    components = {
        "length": length_term,
        "shape": shape_term,
        "offset": offset_term,
        "curvature": curvature_term,
        "curvature_rate": rate_term,
        "inverse_clearance": clearance_term,
        "lateral_acceleration": accel_term,
    }
    total = sum(SCORE_WEIGHTS[name] * value for name, value in components.items())
    return _EvaluatedCandidate(
        candidate_id=candidate.candidate_id,
        offset_m=output_offset,
        x_m=x_values,
        y_m=y_values,
        yaw_rad=yaw_values,
        curvature_1pm=curvature,
        curvature_rate_1pm2=curvature_rate,
        left_clearance_m=left_clearance,
        right_clearance_m=right_clearance,
        speed_cap_mps=speed_cap,
        score_components=components,
        total_score=float(total),
    )


def _planner_metadata(route_features, target_speed_kmh, config, selected):
    allowed_route_keys = (
        "route_label",
        "length",
        "total_abs_turn",
        "sign_changes",
        "turn_segments",
    )
    route_metadata = {
        key: route_features[key]
        for key in allowed_route_keys
        if key in route_features
    }
    return {
        "planner_mode": "frenet",
        "planner_version": PLANNER_VERSION,
        "target_speed_kmh": float(target_speed_kmh),
        "config": asdict(config),
        "score_weights": dict(SCORE_WEIGHTS),
        "route": route_metadata,
        "selected_candidate_id": selected.candidate_id,
        "score_components": dict(selected.score_components),
        "total_score": selected.total_score,
    }


def plan_frenet_reference(route_trace, route_features, target_speed_kmh, config=None, clock=time.perf_counter):
    config = config or FrenetPlannerConfig()
    started = float(clock())
    rejections = {}
    centerline = build_centerline_model(route_trace, config)
    if float(clock()) - started > config.deadline_s:
        raise FrenetPlanningFailure("planning_timeout", rejections, "Frenet planning deadline exceeded.")
    candidates = generate_lateral_candidates(centerline, config)
    target_speed_mps = max(float(target_speed_kmh), 0.0) / 3.6
    evaluated = []
    for candidate in candidates:
        if float(clock()) - started > config.deadline_s:
            raise FrenetPlanningFailure("planning_timeout", rejections, "Frenet planning deadline exceeded.")
        result = _evaluate_candidate(centerline, candidate, target_speed_mps, route_features, config, rejections)
        if result is not None:
            evaluated.append(result)
    if not evaluated:
        raise FrenetPlanningFailure("no_feasible_candidate", rejections, "No feasible Frenet trajectory.")
    selected = min(evaluated, key=lambda item: (item.total_score, item.candidate_id))
    metadata = _planner_metadata(route_features, target_speed_kmh, config, selected)
    trajectory = ReferenceTrajectory(
        s_m=centerline.s_m,
        x_m=selected.x_m,
        y_m=selected.y_m,
        z_m=centerline.z_m,
        yaw_rad=selected.yaw_rad,
        curvature_1pm=selected.curvature_1pm,
        curvature_rate_1pm2=selected.curvature_rate_1pm2,
        left_clearance_m=selected.left_clearance_m,
        right_clearance_m=selected.right_clearance_m,
        speed_cap_mps=selected.speed_cap_mps,
        source_index=centerline.source_index,
        road_options=centerline.road_options,
        metadata=metadata,
    )
    finished = float(clock())
    diagnostics = {
        "candidate_count": len(candidates),
        "feasible_candidate_count": len(evaluated),
        "rejection_counts": dict(rejections),
        "selected_candidate_id": selected.candidate_id,
        "score_components": dict(selected.score_components),
        "total_score": selected.total_score,
        "planning_duration_s": max(finished - started, 0.0),
        "candidate_cap_reached": len(candidates) >= config.candidate_cap,
        "reference_max_abs_curvature": float(np.max(np.abs(selected.curvature_1pm))),
        "reference_max_abs_curvature_rate": float(np.max(np.abs(selected.curvature_rate_1pm2))),
        "reference_min_footprint_clearance_m": float(
            np.min(np.minimum(selected.left_clearance_m, selected.right_clearance_m))
            - config.vehicle_half_width_m
        ),
    }
    return trajectory, diagnostics
