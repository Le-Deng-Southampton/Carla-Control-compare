import math

import carla
import numpy as np

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.local_planner import RoadOption


ROUTE_SHAPES = ("true_straight", "straight", "gentle_curve", "s_curve", "curvy")
NOMINAL_STRAIGHT_MAX_TURN_RAD = math.radians(20.0)
SPEED_ROUTE_PROFILES = (
    (55.0, "curvy", 85.0, 1100.0, 1450.0, "low"),
    (85.0, "s_curve", 80.0, 1500.0, 1900.0, "medium"),
    (115.0, "gentle_curve", 75.0, 1900.0, 2500.0, "high"),
    (float("inf"), "straight", 70.0, 2400.0, 3600.0, "very_high"),
)


def _angle_diff(a, b):
    return (a - b + np.pi) % (2 * np.pi) - np.pi


def _classify_turn(prev_yaw, next_yaw, threshold=np.radians(12.0)):
    diff = _angle_diff(next_yaw, prev_yaw)
    if abs(diff) < threshold:
        return RoadOption.STRAIGHT
    return RoadOption.LEFT if diff > 0 else RoadOption.RIGHT


def _route_length(route_trace):
    total = 0.0
    for idx in range(1, len(route_trace)):
        prev_loc = route_trace[idx - 1][0].transform.location
        curr_loc = route_trace[idx][0].transform.location
        total += prev_loc.distance(curr_loc)
    return total


def speed_adaptive_route_defaults(target_speed_kmh):
    speed_kmh = max(0.0, float(target_speed_kmh or 0.0))
    speed_mps = max(speed_kmh / 3.6, 1.0)
    for max_speed_kmh, route_shape, target_duration_s, min_length_m, max_length_m, speed_band in SPEED_ROUTE_PROFILES:
        if speed_kmh > max_speed_kmh:
            continue
        return {
            "route_shape": route_shape,
            "min_length_m": min(max(speed_mps * target_duration_s, min_length_m), max_length_m),
            "speed_band": speed_band,
            "target_duration_s": target_duration_s,
        }


def estimate_route_max_waypoints(min_length_m, sampling_resolution, length_tolerance):
    resolution = max(float(sampling_resolution or 1.0), 0.1)
    tolerance = max(float(length_tolerance or 0.0), 0.0)
    waypoint_count = math.ceil(float(min_length_m) * (1.0 + tolerance) / resolution)
    return max(2000, int(math.ceil(waypoint_count * 1.25)) + 20)


def _route_shape_features(route_trace):
    yaw_deltas = []
    turn_signs = []

    for idx in range(1, len(route_trace)):
        prev_yaw = math.radians(route_trace[idx - 1][0].transform.rotation.yaw)
        curr_yaw = math.radians(route_trace[idx][0].transform.rotation.yaw)
        delta = _angle_diff(curr_yaw, prev_yaw)
        yaw_deltas.append(delta)
        if abs(delta) >= math.radians(6.0):
            turn_signs.append(1 if delta > 0 else -1)

    sign_changes = 0
    for idx in range(1, len(turn_signs)):
        if turn_signs[idx] != turn_signs[idx - 1]:
            sign_changes += 1

    total_abs_turn = sum(abs(delta) for delta in yaw_deltas)
    abs_curvatures = []
    for idx, delta in enumerate(yaw_deltas, start=1):
        prev_loc = route_trace[idx - 1][0].transform.location
        curr_loc = route_trace[idx][0].transform.location
        segment_length = max(prev_loc.distance(curr_loc), 1e-6)
        abs_curvatures.append(abs(delta) / segment_length)

    left_turn = sum(max(delta, 0.0) for delta in yaw_deltas)
    right_turn = sum(max(-delta, 0.0) for delta in yaw_deltas)
    dominant_turn = max(left_turn, right_turn)
    minor_turn = min(left_turn, right_turn)

    return {
        "length": _route_length(route_trace),
        "total_abs_turn": total_abs_turn,
        "left_turn": left_turn,
        "right_turn": right_turn,
        "dominant_turn": dominant_turn,
        "minor_turn": minor_turn,
        "sign_changes": sign_changes,
        "turn_segments": len(turn_signs),
        "mean_abs_curvature": sum(abs_curvatures) / len(abs_curvatures) if abs_curvatures else 0.0,
        "max_abs_curvature": max(abs_curvatures or [0.0]),
    }


def classify_route_label(route_shape, features):
    if route_shape == "true_straight":
        return "true_straight"
    if route_shape == "straight" and features["total_abs_turn"] > NOMINAL_STRAIGHT_MAX_TURN_RAD:
        return "nominal_straight_curved_network"
    return route_shape


def validate_route_shape(route_shape, features):
    if route_shape == "true_straight" and features["total_abs_turn"] > NOMINAL_STRAIGHT_MAX_TURN_RAD:
        raise RuntimeError(
            "Failed to build true_straight route: total_abs_turn_deg="
            f"{math.degrees(features['total_abs_turn']):.1f} exceeds "
            f"{math.degrees(NOMINAL_STRAIGHT_MAX_TURN_RAD):.1f} deg."
        )
    if route_shape == "curvy" and features.get("max_abs_curvature", 0.0) < 0.02:
        raise RuntimeError(
            "Failed to build curvy route: max_abs_curvature="
            f"{features.get('max_abs_curvature', 0.0):.5f} is below the 0.02000 1/m "
            "town sharp-curve requirement."
        )


def _shape_penalty(route_shape, features):
    total_abs_turn = features["total_abs_turn"]
    dominant_turn = features["dominant_turn"]
    minor_turn = features["minor_turn"]
    sign_changes = features["sign_changes"]

    if route_shape in ("true_straight", "straight"):
        return total_abs_turn / math.radians(20.0) + sign_changes * 0.7 + minor_turn / math.radians(8.0)

    if route_shape == "gentle_curve":
        target_turn = math.radians(55.0)
        return (
            abs(dominant_turn - target_turn) / target_turn
            + minor_turn / math.radians(18.0)
            + sign_changes * 0.9
        )

    if route_shape == "s_curve":
        target_turn = math.radians(85.0)
        balance_penalty = abs(features["left_turn"] - features["right_turn"]) / max(total_abs_turn, 1e-6)
        return (
            abs(total_abs_turn - target_turn) / target_turn
            + max(0, 1 - sign_changes) * 1.8
            + max(0.0, math.radians(18.0) - minor_turn) / math.radians(18.0)
            + balance_penalty
        )

    if route_shape == "curvy":
        target_turn = math.radians(130.0)
        return (
            max(0, 2 - sign_changes) * 1.4
            + max(0.0, target_turn - total_abs_turn) / target_turn
            + max(0.0, math.radians(20.0) - minor_turn) / math.radians(20.0)
        )

    raise ValueError(f"Unsupported route shape: {route_shape}")


def _speed_suitability_penalty(features, target_speed_kmh, lateral_accel_limit=8.0):
    if target_speed_kmh is None:
        return 0.0

    speed_mps = max(float(target_speed_kmh), 0.0) / 3.6
    if speed_mps < 8.0:
        return 0.0

    curvature_limit = max(float(lateral_accel_limit), 0.1) / max(speed_mps ** 2, 1e-6)
    mean_curvature_limit = curvature_limit * 0.65
    max_abs_curvature = features.get("max_abs_curvature", 0.0)
    mean_abs_curvature = features.get("mean_abs_curvature", 0.0)

    max_penalty = max(0.0, max_abs_curvature / max(curvature_limit, 1e-6) - 1.0)
    mean_penalty = max(0.0, mean_abs_curvature / max(mean_curvature_limit, 1e-6) - 1.0)
    return max_penalty + 0.5 * mean_penalty


def _is_speed_feasible(features, target_speed_kmh, lateral_accel_limit=6.5):
    if target_speed_kmh is None:
        return True
    speed_mps = max(float(target_speed_kmh), 0.0) / 3.6
    if speed_mps < 1.0:
        return True
    predicted_lateral_accel = speed_mps ** 2 * features.get("max_abs_curvature", 0.0)
    return predicted_lateral_accel <= float(lateral_accel_limit) + 1e-9


def _waypoint_key(waypoint):
    return (
        int(getattr(waypoint, "road_id", -1)),
        int(getattr(waypoint, "section_id", -1)),
        int(getattr(waypoint, "lane_id", -1)),
        int(getattr(waypoint, "id", -1)),
    )


def _stable_route_key(state):
    return tuple(_waypoint_key(waypoint) for waypoint, _ in state["route_trace"])


def _beam_bucket(state):
    waypoint = state["current_wp"]
    return (
        int(state.get("last_turn_sign", 0)),
        min(int(state.get("sign_changes", 0)), 2),
        int(getattr(waypoint, "road_id", -1)),
        int(getattr(waypoint, "lane_id", -1)),
    )


def _partial_state_score(state, target_length_m, route_shape, target_speed_kmh):
    length_error = abs(float(state["total_length"]) - float(target_length_m)) / max(float(target_length_m), 1e-6)
    left_turn = float(state.get("left_turn", 0.0))
    right_turn = float(state.get("right_turn", 0.0))
    turn_count = max(len(state["route_trace"]) - 1, 1)
    features = {
        "total_abs_turn": left_turn + right_turn,
        "left_turn": left_turn,
        "right_turn": right_turn,
        "dominant_turn": max(left_turn, right_turn),
        "minor_turn": min(left_turn, right_turn),
        "sign_changes": int(state.get("sign_changes", 0)),
        "turn_segments": int(state.get("turn_segments", 0)),
        "mean_abs_curvature": float(state.get("curvature_sum", 0.0)) / turn_count,
        "max_abs_curvature": float(state.get("max_abs_curvature", 0.0)),
    }
    shape_penalty = _shape_penalty(route_shape, features)
    speed_penalty = _speed_suitability_penalty(features, target_speed_kmh)
    return length_error * 3.0 + shape_penalty * 0.25 + speed_penalty * 0.5


def _select_diverse_beam(states, beam_width, target_length_m, route_shape, target_speed_kmh):
    beam_width = max(int(beam_width), 1)
    ranked = sorted(
        states,
        key=lambda state: (
            _partial_state_score(state, target_length_m, route_shape, target_speed_kmh),
            _stable_route_key(state),
        ),
    )
    buckets = {}
    for state in ranked:
        buckets.setdefault(_beam_bucket(state), []).append(state)

    representatives = [items[0] for _, items in sorted(buckets.items(), key=lambda item: item[0])]
    representatives.sort(
        key=lambda state: (
            _partial_state_score(state, target_length_m, route_shape, target_speed_kmh),
            _stable_route_key(state),
        )
    )
    selected = representatives[:beam_width]
    selected_ids = {id(state) for state in selected}
    if len(selected) < beam_width:
        for state in ranked:
            if id(state) in selected_ids:
                continue
            selected.append(state)
            selected_ids.add(id(state))
            if len(selected) >= beam_width:
                break
    return selected


def _collect_route_candidates(
    start_wp,
    sampling_resolution,
    target_length_m,
    max_waypoints,
    length_tolerance,
    route_shape="straight",
    target_speed_kmh=None,
    beam_width=24,
):
    min_length = target_length_m * (1.0 - length_tolerance)
    max_length = target_length_m * (1.0 + length_tolerance)
    overshoot_limit = max_length * 1.15

    initial_state = {
        "route_trace": [(start_wp, RoadOption.LANEFOLLOW)],
        "current_wp": start_wp,
        "prev_yaw": math.radians(start_wp.transform.rotation.yaw),
        "total_length": 0.0,
        "visited_waypoint_keys": frozenset((_waypoint_key(start_wp),)),
        "visited_edge_keys": frozenset(),
        "left_turn": 0.0,
        "right_turn": 0.0,
        "sign_changes": 0,
        "turn_segments": 0,
        "last_turn_sign": 0,
        "max_abs_curvature": 0.0,
        "curvature_sum": 0.0,
    }
    beam = [initial_state]
    completed = []

    for _ in range(max_waypoints):
        next_beam = []
        for state in beam:
            if state["total_length"] >= min_length and len(state["route_trace"]) >= 20:
                completed.append(state["route_trace"])
                if state["total_length"] >= max_length:
                    continue

            candidates = state["current_wp"].next(sampling_resolution)
            if not candidates:
                continue

            ranked_candidates = sorted(
                candidates,
                key=lambda waypoint: abs(
                    _angle_diff(math.radians(waypoint.transform.rotation.yaw), state["prev_yaw"])
                ),
            )

            for candidate in ranked_candidates[:3]:
                candidate_key = _waypoint_key(candidate)
                edge_key = (_waypoint_key(state["current_wp"]), candidate_key)
                if candidate_key in state["visited_waypoint_keys"] or edge_key in state["visited_edge_keys"]:
                    continue

                segment_length = state["current_wp"].transform.location.distance(candidate.transform.location)
                total_length = state["total_length"] + segment_length
                if total_length > overshoot_limit:
                    continue

                candidate_yaw = math.radians(candidate.transform.rotation.yaw)
                yaw_delta = _angle_diff(candidate_yaw, state["prev_yaw"])
                turn_sign = 0
                if abs(yaw_delta) >= math.radians(6.0):
                    turn_sign = 1 if yaw_delta > 0.0 else -1
                previous_sign = int(state.get("last_turn_sign", 0))
                sign_changes = int(state.get("sign_changes", 0))
                if turn_sign and previous_sign and turn_sign != previous_sign:
                    sign_changes += 1
                effective_sign = turn_sign or previous_sign
                abs_curvature = abs(yaw_delta) / max(segment_length, 1e-6)
                turn_option = _classify_turn(state["prev_yaw"], candidate_yaw)
                next_beam.append({
                    "route_trace": state["route_trace"] + [(candidate, turn_option)],
                    "current_wp": candidate,
                    "prev_yaw": candidate_yaw,
                    "total_length": total_length,
                    "visited_waypoint_keys": state["visited_waypoint_keys"] | frozenset((candidate_key,)),
                    "visited_edge_keys": state["visited_edge_keys"] | frozenset((edge_key,)),
                    "left_turn": float(state.get("left_turn", 0.0)) + max(yaw_delta, 0.0),
                    "right_turn": float(state.get("right_turn", 0.0)) + max(-yaw_delta, 0.0),
                    "sign_changes": sign_changes,
                    "turn_segments": int(state.get("turn_segments", 0)) + int(turn_sign != 0),
                    "last_turn_sign": effective_sign,
                    "max_abs_curvature": max(float(state.get("max_abs_curvature", 0.0)), abs_curvature),
                    "curvature_sum": float(state.get("curvature_sum", 0.0)) + abs_curvature,
                })

        if not next_beam:
            break

        beam = _select_diverse_beam(
            next_beam,
            beam_width=beam_width,
            target_length_m=target_length_m,
            route_shape=route_shape,
            target_speed_kmh=target_speed_kmh,
        )

    if completed:
        return completed

    return [state["route_trace"] for state in beam if len(state["route_trace"]) >= 20]


def _select_shaped_candidate(candidates, route_shape, target_length_m, target_speed_kmh=None):
    scored = []
    for route_trace in candidates:
        features = _route_shape_features(route_trace)
        if not _is_speed_feasible(features, target_speed_kmh):
            continue
        length_penalty = abs(features["length"] - target_length_m) / max(target_length_m, 1e-6)
        shape_penalty = _shape_penalty(route_shape, features)
        speed_penalty = _speed_suitability_penalty(features, target_speed_kmh)
        scored.append((length_penalty * 3.0 + shape_penalty + speed_penalty * 1.5, route_trace, features))

    if not scored:
        return None, None

    scored.sort(key=lambda item: item[0])
    _, route_trace, features = scored[0]
    return route_trace, features


def build_shaped_route(
    carla_map,
    origin,
    destination,
    sampling_resolution,
    min_length_m,
    max_waypoints,
    close_distance,
    route_shape="straight",
    length_tolerance=0.05,
    target_speed_kmh=None,
):
    if route_shape not in ROUTE_SHAPES:
        raise ValueError(f"Unsupported route shape '{route_shape}'. Expected one of {ROUTE_SHAPES}.")

    start_wp = carla_map.get_waypoint(origin, project_to_road=True, lane_type=carla.LaneType.Driving)
    if start_wp is None:
        raise RuntimeError("Failed to find a valid start waypoint.")

    candidates = _collect_route_candidates(
        start_wp,
        sampling_resolution,
        min_length_m,
        max_waypoints,
        length_tolerance,
        route_shape,
        target_speed_kmh,
        beam_width=24,
    )
    route_trace, features = _select_shaped_candidate(candidates, route_shape, min_length_m, target_speed_kmh)
    if route_trace is not None:
        validate_route_shape(route_shape, features)
        features["route_label"] = classify_route_label(route_shape, features)
        return route_trace, features

    print(f"[WARN] No shaped route matched '{route_shape}', falling back to shortest route.")
    planner = GlobalRoutePlanner(carla_map, sampling_resolution)
    route_trace = planner.trace_route(origin, destination)
    if not route_trace:
        raise RuntimeError("Failed to build a fixed global route.")
    features = _route_shape_features(route_trace)
    validate_route_shape(route_shape, features)
    if not _is_speed_feasible(features, target_speed_kmh):
        raise RuntimeError(
            f"Failed to build speed-feasible {route_shape} route: "
            f"max_abs_curvature={features['max_abs_curvature']:.5f} at "
            f"{float(target_speed_kmh):.1f} km/h exceeds the 6.5 m/s^2 lateral-acceleration gate."
        )
    features["route_label"] = classify_route_label(route_shape, features)
    return route_trace, features
