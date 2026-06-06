import math

import carla
import numpy as np

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.local_planner import RoadOption


ROUTE_SHAPES = ("straight", "gentle_curve", "s_curve", "curvy")


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
    }


def _shape_penalty(route_shape, features):
    total_abs_turn = features["total_abs_turn"]
    dominant_turn = features["dominant_turn"]
    minor_turn = features["minor_turn"]
    sign_changes = features["sign_changes"]

    if route_shape == "straight":
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


def _collect_route_candidates(start_wp, sampling_resolution, target_length_m, max_waypoints, length_tolerance, beam_width=12):
    min_length = target_length_m * (1.0 - length_tolerance)
    max_length = target_length_m * (1.0 + length_tolerance)
    overshoot_limit = max_length * 1.15

    initial_state = {
        "route_trace": [(start_wp, RoadOption.LANEFOLLOW)],
        "current_wp": start_wp,
        "prev_yaw": math.radians(start_wp.transform.rotation.yaw),
        "total_length": 0.0,
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
                if any(prev_wp.id == candidate.id for prev_wp, _ in state["route_trace"][-5:]):
                    continue

                segment_length = state["current_wp"].transform.location.distance(candidate.transform.location)
                total_length = state["total_length"] + segment_length
                if total_length > overshoot_limit:
                    continue

                candidate_yaw = math.radians(candidate.transform.rotation.yaw)
                turn_option = _classify_turn(state["prev_yaw"], candidate_yaw)
                next_beam.append({
                    "route_trace": state["route_trace"] + [(candidate, turn_option)],
                    "current_wp": candidate,
                    "prev_yaw": candidate_yaw,
                    "total_length": total_length,
                })

        if not next_beam:
            break

        next_beam.sort(
            key=lambda state: (
                abs(state["total_length"] - target_length_m),
                abs(
                    _angle_diff(
                        state["prev_yaw"],
                        math.radians(start_wp.transform.rotation.yaw),
                    )
                ),
            )
        )
        beam = next_beam[:beam_width]

    if completed:
        return completed

    return [state["route_trace"] for state in beam if len(state["route_trace"]) >= 20]


def _select_shaped_candidate(candidates, route_shape, target_length_m):
    scored = []
    for route_trace in candidates:
        features = _route_shape_features(route_trace)
        length_penalty = abs(features["length"] - target_length_m) / max(target_length_m, 1e-6)
        shape_penalty = _shape_penalty(route_shape, features)
        scored.append((length_penalty * 3.0 + shape_penalty, route_trace, features))

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
    )
    route_trace, features = _select_shaped_candidate(candidates, route_shape, min_length_m)
    if route_trace is not None:
        return route_trace, features

    print(f"[WARN] No shaped route matched '{route_shape}', falling back to shortest route.")
    planner = GlobalRoutePlanner(carla_map, sampling_resolution)
    route_trace = planner.trace_route(origin, destination)
    if not route_trace:
        raise RuntimeError("Failed to build a fixed global route.")
    return route_trace, _route_shape_features(route_trace)
