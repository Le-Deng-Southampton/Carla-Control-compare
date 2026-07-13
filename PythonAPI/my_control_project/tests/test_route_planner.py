import os
import math
import sys
import types
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYTHONAPI_ROOT = os.path.dirname(PROJECT_ROOT)
CARLA_AGENTS_ROOT = os.path.join(PYTHONAPI_ROOT, "carla")
for path in (PROJECT_ROOT, CARLA_AGENTS_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

if "carla" not in sys.modules:
    carla = types.ModuleType("carla")
    carla.LaneType = types.SimpleNamespace(Driving=1)
    sys.modules["carla"] = carla

if "agents.navigation.global_route_planner" not in sys.modules:
    global_route_planner = types.ModuleType("agents.navigation.global_route_planner")
    global_route_planner.GlobalRoutePlanner = object
    sys.modules["agents.navigation.global_route_planner"] = global_route_planner

if "agents.navigation.local_planner" not in sys.modules:
    local_planner = types.ModuleType("agents.navigation.local_planner")
    local_planner.RoadOption = types.SimpleNamespace(STRAIGHT="STRAIGHT", LEFT="LEFT", RIGHT="RIGHT", LANEFOLLOW="LANEFOLLOW")
    sys.modules["agents.navigation.local_planner"] = local_planner

from road_planning.route_planner import (
    _collect_route_candidates,
    _is_speed_feasible,
    _select_diverse_beam,
    _speed_suitability_penalty,
    classify_route_label,
    estimate_route_max_waypoints,
    speed_adaptive_route_defaults,
    validate_route_shape,
)


class FakeLocation:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
        self.z = 0.0

    def distance(self, other):
        return math.hypot(self.x - other.x, self.y - other.y)


class FakeWaypoint:
    def __init__(self, waypoint_id, x, y, yaw_deg):
        self.id = waypoint_id
        self.road_id = 1
        self.section_id = 0
        self.lane_id = 1
        self.transform = types.SimpleNamespace(
            location=FakeLocation(x, y),
            rotation=types.SimpleNamespace(yaw=float(yaw_deg)),
        )
        self._next = []

    def next(self, _sampling_resolution):
        return list(self._next)


class RoutePlannerFeatureTest(unittest.TestCase):
    def test_candidate_search_rejects_loop_longer_than_local_history(self):
        radius = 20.0 / (2.0 * math.pi)
        waypoints = []
        for index in range(20):
            angle = 2.0 * math.pi * index / 20.0
            yaw = math.degrees(angle + math.pi * 0.5)
            waypoints.append(FakeWaypoint(index, radius * math.cos(angle), radius * math.sin(angle), yaw))
        for index in range(19):
            waypoints[index]._next = [waypoints[index + 1]]
        waypoints[-1]._next = [waypoints[0]]

        candidates = _collect_route_candidates(
            waypoints[0],
            1.0,
            20.0,
            100,
            0.05,
            "curvy",
            50.0,
            beam_width=24,
        )

        for route in candidates:
            ids = [waypoint.id for waypoint, _ in route]
            self.assertEqual(len(ids), len(set(ids)))

    def test_diverse_beam_keeps_distinct_turn_history_bucket(self):
        base = {
            "total_length": 8.0,
            "prev_yaw": 0.0,
            "left_turn": 0.0,
            "right_turn": 0.0,
            "sign_changes": 0,
            "max_abs_curvature": 0.0,
            "curvature_sum": 0.0,
            "current_wp": FakeWaypoint(1, 0.0, 0.0, 0.0),
        }
        straight_a = dict(base, route_trace=[(FakeWaypoint(10, 0.0, 0.0, 0.0), "STRAIGHT")], last_turn_sign=0)
        straight_b = dict(base, route_trace=[(FakeWaypoint(11, 0.0, 0.0, 0.0), "STRAIGHT")], last_turn_sign=0)
        s_curve = dict(
            base,
            route_trace=[(FakeWaypoint(12, 0.0, 0.0, 0.0), "LEFT")],
            last_turn_sign=-1,
            sign_changes=1,
            left_turn=0.3,
            right_turn=0.3,
        )

        selected = _select_diverse_beam(
            [straight_a, straight_b, s_curve],
            beam_width=2,
            target_length_m=10.0,
            route_shape="s_curve",
            target_speed_kmh=50.0,
        )

        self.assertEqual(len(selected), 2)
        self.assertTrue(any(state["sign_changes"] == 1 for state in selected))

    def test_straight_route_with_large_turn_is_labelled_as_nominal_not_true_straight(self):
        features = {"total_abs_turn": 1.0}

        label = classify_route_label("straight", features)

        self.assertEqual(label, "nominal_straight_curved_network")

    def test_true_straight_rejects_large_total_turn(self):
        features = {"total_abs_turn": 1.0}

        with self.assertRaises(RuntimeError):
            validate_route_shape("true_straight", features)

    def test_curvy_route_rejects_mislabeled_gentle_fallback(self):
        features = {"total_abs_turn": 1.5, "max_abs_curvature": 0.011}

        with self.assertRaises(RuntimeError):
            validate_route_shape("curvy", features)

    def test_speed_adaptive_route_defaults_choose_safer_high_speed_route(self):
        low_speed = speed_adaptive_route_defaults(45.0)
        high_speed = speed_adaptive_route_defaults(130.0)

        self.assertEqual(low_speed["route_shape"], "curvy")
        self.assertLess(low_speed["min_length_m"], high_speed["min_length_m"])
        self.assertEqual(high_speed["route_shape"], "straight")
        self.assertIn("target_duration_s", high_speed)

    def test_speed_adaptive_route_length_uses_speed_scaled_duration(self):
        medium_speed = speed_adaptive_route_defaults(72.0)
        high_speed = speed_adaptive_route_defaults(108.0)

        self.assertAlmostEqual(medium_speed["min_length_m"], 1600.0)
        self.assertAlmostEqual(high_speed["min_length_m"], 2250.0)
        self.assertGreater(high_speed["min_length_m"], medium_speed["min_length_m"])

    def test_auto_waypoint_capacity_scales_with_route_length(self):
        short_capacity = estimate_route_max_waypoints(1200.0, 1.0, 0.05)
        long_capacity = estimate_route_max_waypoints(3200.0, 1.0, 0.05)

        self.assertEqual(short_capacity, 2000)
        self.assertGreater(long_capacity, short_capacity)

    def test_speed_suitability_penalty_increases_for_tight_high_speed_curves(self):
        gentle_features = {"max_abs_curvature": 0.003, "mean_abs_curvature": 0.0015}
        tight_features = {"max_abs_curvature": 0.04, "mean_abs_curvature": 0.02}

        gentle_penalty = _speed_suitability_penalty(gentle_features, 130.0)
        tight_penalty = _speed_suitability_penalty(tight_features, 130.0)

        self.assertGreater(tight_penalty, gentle_penalty)

    def test_high_speed_route_rejects_curvature_above_acceptance_lateral_acceleration(self):
        self.assertTrue(_is_speed_feasible({"max_abs_curvature": 0.005}, 120.0))
        self.assertFalse(_is_speed_feasible({"max_abs_curvature": 0.010}, 120.0))


if __name__ == "__main__":
    unittest.main()
