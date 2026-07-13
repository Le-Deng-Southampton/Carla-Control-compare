from dataclasses import dataclass
import math

import numpy as np

from .reference_trajectory import trajectory_from_route_trace


@dataclass
class ReferencePathConfig:
    sample_spacing_m: float = 1.0
    smoothing_window: int = 7
    lane_margin_m: float = 0.35
    max_centerline_offset_m: float = 1.25


@dataclass
class ReferenceLocation:
    x: float
    y: float
    z: float = 0.0

    def distance(self, other):
        dx = self.x - other.x
        dy = self.y - other.y
        dz = self.z - getattr(other, "z", 0.0)
        return float(math.sqrt(dx * dx + dy * dy + dz * dz))


@dataclass
class ReferenceRotation:
    yaw: float


@dataclass
class ReferenceTransform:
    location: ReferenceLocation
    rotation: ReferenceRotation


@dataclass
class ReferenceWaypoint:
    transform: ReferenceTransform
    id: int
    lane_width: float
    source_index: int
    centerline_offset_m: float
    left_clearance_m: float
    right_clearance_m: float
    is_junction: bool = False


def _angle_diff(a, b):
    return (a - b + np.pi) % (2 * np.pi) - np.pi


def _location(waypoint):
    return waypoint.transform.location


def _yaw_rad(waypoint):
    return math.radians(waypoint.transform.rotation.yaw)


def _lane_width(waypoint, default=3.5):
    return float(getattr(waypoint, "lane_width", default) or default)


def _is_junction(waypoint):
    value = getattr(waypoint, "is_junction", False)
    return bool(value() if callable(value) else value)


def _route_points(route_trace):
    return np.asarray(
        [[_location(waypoint).x, _location(waypoint).y, getattr(_location(waypoint), "z", 0.0)] for waypoint, _ in route_trace],
        dtype=float,
    )


def _cumulative_distances(points):
    distances = np.zeros(len(points), dtype=float)
    for idx in range(1, len(points)):
        distances[idx] = distances[idx - 1] + float(np.linalg.norm(points[idx, :2] - points[idx - 1, :2]))
    return distances


def _interpolate_angle(a, b, ratio):
    return a + _angle_diff(b, a) * ratio


def _resample_route(route_trace, sample_spacing_m):
    if len(route_trace) < 2:
        return []

    points = _route_points(route_trace)
    distances = _cumulative_distances(points)
    total_length = distances[-1]
    spacing = max(float(sample_spacing_m), 0.25)
    sample_distances = np.arange(0.0, total_length, spacing)
    if len(sample_distances) == 0 or sample_distances[-1] < total_length:
        sample_distances = np.append(sample_distances, total_length)

    samples = []
    segment_idx = 0
    for sample_distance in sample_distances:
        while segment_idx < len(distances) - 2 and distances[segment_idx + 1] < sample_distance:
            segment_idx += 1

        start_distance = distances[segment_idx]
        end_distance = distances[segment_idx + 1]
        denom = max(end_distance - start_distance, 1e-6)
        ratio = float(np.clip((sample_distance - start_distance) / denom, 0.0, 1.0))
        start_wp, road_option = route_trace[segment_idx]
        end_wp, _ = route_trace[segment_idx + 1]
        start_point = points[segment_idx]
        end_point = points[segment_idx + 1]
        center = start_point + ratio * (end_point - start_point)
        yaw = _interpolate_angle(_yaw_rad(start_wp), _yaw_rad(end_wp), ratio)
        lane_width = (1.0 - ratio) * _lane_width(start_wp) + ratio * _lane_width(end_wp)
        is_junction = _is_junction(start_wp) or _is_junction(end_wp)
        samples.append({
            "center": center,
            "yaw": yaw,
            "lane_width": lane_width,
            "road_option": road_option,
            "source_index": segment_idx,
            "is_junction": is_junction,
        })
    return samples


def _smooth_centers(samples, smoothing_window):
    centers = np.asarray([sample["center"] for sample in samples], dtype=float)
    if len(centers) < 3:
        return centers

    window = max(int(smoothing_window), 1)
    if window % 2 == 0:
        window += 1
    radius = window // 2
    smoothed = np.zeros_like(centers)
    for idx in range(len(centers)):
        start = max(0, idx - radius)
        end = min(len(centers), idx + radius + 1)
        smoothed[idx] = np.mean(centers[start:end], axis=0)
    smoothed[0] = centers[0]
    smoothed[-1] = centers[-1]
    return smoothed


def _heading_from_points(points, idx, fallback_yaw):
    if len(points) < 2:
        return fallback_yaw
    if idx == 0:
        delta = points[1] - points[0]
    elif idx == len(points) - 1:
        delta = points[-1] - points[-2]
    else:
        delta = points[idx + 1] - points[idx - 1]
    if np.linalg.norm(delta[:2]) < 1e-6:
        return fallback_yaw
    return float(math.atan2(delta[1], delta[0]))


def _curvature(points, idx):
    if idx <= 0 or idx >= len(points) - 1:
        return 0.0

    p1 = points[idx - 1, :2]
    p2 = points[idx, :2]
    p3 = points[idx + 1, :2]
    cross = (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])
    a = np.linalg.norm(p3 - p2)
    b = np.linalg.norm(p3 - p1)
    c = np.linalg.norm(p2 - p1)
    denom = a * b * c
    if denom < 1e-6:
        return 0.0
    return float(np.clip((2.0 * cross) / denom, -0.2, 0.2))


def _scene_coverage(points, samples):
    if len(points) < 2:
        return {
            "reference_straight_m": 0.0,
            "reference_gentle_curve_m": 0.0,
            "reference_moderate_curve_m": 0.0,
            "reference_tight_curve_m": 0.0,
            "reference_s_curve_sign_changes": 0,
            "reference_junction_samples": 0,
            "reference_narrow_lane_samples": 0,
        }

    buckets = {
        "reference_straight_m": 0.0,
        "reference_gentle_curve_m": 0.0,
        "reference_moderate_curve_m": 0.0,
        "reference_tight_curve_m": 0.0,
    }
    curvature_signs = []
    for idx in range(1, len(points)):
        curvature = abs(_curvature(points, idx))
        segment_length = float(np.linalg.norm(points[idx, :2] - points[idx - 1, :2]))
        if curvature < 0.005:
            buckets["reference_straight_m"] += segment_length
        elif curvature < 0.020:
            buckets["reference_gentle_curve_m"] += segment_length
        elif curvature < 0.050:
            buckets["reference_moderate_curve_m"] += segment_length
        else:
            buckets["reference_tight_curve_m"] += segment_length

        signed_curvature = _curvature(points, idx)
        if abs(signed_curvature) >= 0.006:
            curvature_signs.append(1 if signed_curvature > 0.0 else -1)

    sign_changes = 0
    for idx in range(1, len(curvature_signs)):
        if curvature_signs[idx] != curvature_signs[idx - 1]:
            sign_changes += 1

    buckets.update({
        "reference_s_curve_sign_changes": sign_changes,
        "reference_junction_samples": sum(1 for sample in samples if sample["is_junction"]),
        "reference_narrow_lane_samples": sum(1 for sample in samples if sample["lane_width"] < 3.2),
    })
    return buckets


def build_feasible_reference_trace(route_trace, config=None):
    """Return a smooth, lane-bounded waypoint-like trace plus validation metrics."""
    config = config or ReferencePathConfig()
    if len(route_trace) < 2:
        return route_trace, {
            "reference_path_enabled": False,
            "reference_validation_passed": False,
            "reference_sample_count": len(route_trace),
        }

    samples = _resample_route(route_trace, config.sample_spacing_m)
    smoothed = _smooth_centers(samples, config.smoothing_window)
    feasible_points = []
    reference_trace = []
    centerline_offsets = []
    abs_centerline_offsets = []
    smoothing_offsets = []
    clearances = []
    violation_count = 0

    for idx, sample in enumerate(samples):
        center = sample["center"]
        raw_yaw = sample["yaw"]
        normal = np.array([-math.sin(raw_yaw), math.cos(raw_yaw), 0.0], dtype=float)
        smoothing_vector = smoothed[idx] - center
        smoothing_offsets.append(float(np.linalg.norm(smoothing_vector[:2])))
        requested_offset = float(np.dot(smoothing_vector, normal))
        lane_half_width = max(sample["lane_width"] * 0.5, 0.0)
        max_offset = min(
            max(lane_half_width - config.lane_margin_m, 0.0),
            max(float(config.max_centerline_offset_m), 0.0),
        )
        feasible_offset = float(np.clip(requested_offset, -max_offset, max_offset))
        feasible_point = center + feasible_offset * normal
        feasible_points.append(feasible_point)
        centerline_offsets.append(feasible_offset)
        abs_centerline_offsets.append(abs(feasible_offset))
        left_clearance = lane_half_width - feasible_offset
        right_clearance = lane_half_width + feasible_offset
        clearances.extend([left_clearance, right_clearance])
        if min(left_clearance, right_clearance) + 1e-9 < config.lane_margin_m:
            violation_count += 1

    feasible_points = np.asarray(feasible_points, dtype=float)
    for idx, sample in enumerate(samples):
        yaw = _heading_from_points(feasible_points, idx, sample["yaw"])
        point = feasible_points[idx]
        lane_half_width = max(sample["lane_width"] * 0.5, 0.0)
        offset = centerline_offsets[idx]
        waypoint = ReferenceWaypoint(
            transform=ReferenceTransform(
                location=ReferenceLocation(float(point[0]), float(point[1]), float(point[2])),
                rotation=ReferenceRotation(math.degrees(yaw)),
            ),
            id=idx,
            lane_width=float(sample["lane_width"]),
            source_index=int(sample["source_index"]),
            centerline_offset_m=float(offset),
            left_clearance_m=float(lane_half_width - offset),
            right_clearance_m=float(lane_half_width + offset),
            is_junction=sample["is_junction"],
        )
        reference_trace.append((waypoint, sample["road_option"]))

    curvature_values = [_curvature(feasible_points, idx) for idx in range(len(feasible_points))]
    reference_length = float(sum(
        np.linalg.norm(feasible_points[idx, :2] - feasible_points[idx - 1, :2])
        for idx in range(1, len(feasible_points))
    ))
    min_clearance = min(clearances or [0.0])
    features = {
        "reference_path_enabled": True,
        "reference_validation_passed": violation_count == 0,
        "reference_sample_count": len(reference_trace),
        "reference_length_m": reference_length,
        "reference_smoothing_window": int(config.smoothing_window),
        "reference_sample_spacing_m": float(config.sample_spacing_m),
        "reference_lane_margin_m": float(config.lane_margin_m),
        "reference_lane_boundary_violation_count": int(violation_count),
        "reference_min_lane_clearance_m": float(min_clearance),
        "reference_mean_centerline_offset_m": float(np.mean(abs_centerline_offsets)) if abs_centerline_offsets else 0.0,
        "reference_max_centerline_offset_m": float(max(abs_centerline_offsets or [0.0])),
        "reference_mean_smoothing_offset_m": float(np.mean(smoothing_offsets)) if smoothing_offsets else 0.0,
        "reference_max_smoothing_offset_m": float(max(smoothing_offsets or [0.0])),
        "reference_mean_abs_curvature": float(np.mean(np.abs(curvature_values))) if curvature_values else 0.0,
        "reference_max_abs_curvature": float(max([abs(value) for value in curvature_values] or [0.0])),
        "reference_min_lane_width_m": float(min(sample["lane_width"] for sample in samples)),
        "reference_mean_lane_width_m": float(np.mean([sample["lane_width"] for sample in samples])),
    }
    features.update(_scene_coverage(feasible_points, samples))
    return reference_trace, features


def build_legacy_reference_trajectory(route_trace, config=None, metadata=None):
    reference_trace, features = build_feasible_reference_trace(route_trace, config)
    trajectory = trajectory_from_route_trace(reference_trace, metadata=metadata)
    return trajectory, reference_trace, features
