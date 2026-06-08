import os
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

from road_planning.route_planner import classify_route_label, validate_route_shape


class RoutePlannerFeatureTest(unittest.TestCase):
    def test_straight_route_with_large_turn_is_labelled_as_nominal_not_true_straight(self):
        features = {"total_abs_turn": 1.0}

        label = classify_route_label("straight", features)

        self.assertEqual(label, "nominal_straight_curved_network")

    def test_true_straight_rejects_large_total_turn(self):
        features = {"total_abs_turn": 1.0}

        with self.assertRaises(RuntimeError):
            validate_route_shape("true_straight", features)


if __name__ == "__main__":
    unittest.main()
