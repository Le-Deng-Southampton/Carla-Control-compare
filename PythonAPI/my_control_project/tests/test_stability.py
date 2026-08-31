import os
import sys
import types
import unittest
from types import SimpleNamespace


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYTHONAPI_ROOT = os.path.dirname(PROJECT_ROOT)
CARLA_AGENTS_ROOT = os.path.join(PYTHONAPI_ROOT, "carla")
for path in (PROJECT_ROOT, CARLA_AGENTS_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

if "carla" not in sys.modules:
    carla = types.ModuleType("carla")
    carla.Transform = lambda *args, **kwargs: None
    carla.VehicleControl = type("VehicleControl", (), {})
    sys.modules["carla"] = carla

if "pygame" not in sys.modules:
    pygame = types.ModuleType("pygame")
    pygame.font = SimpleNamespace(Font=lambda *args, **kwargs: None)
    pygame.HWSURFACE = 0
    pygame.DOUBLEBUF = 0
    pygame.QUIT = 0
    pygame.KEYDOWN = 1
    pygame.K_ESCAPE = 27
    pygame.time = SimpleNamespace(Clock=lambda: None)
    pygame.event = SimpleNamespace(get=lambda: [])
    pygame.display = SimpleNamespace(set_mode=lambda *args, **kwargs: None, set_caption=lambda *args, **kwargs: None, flip=lambda: None)
    pygame.surfarray = SimpleNamespace(make_surface=lambda *args, **kwargs: None)
    pygame.init = lambda: None
    sys.modules["pygame"] = pygame

route_planner = types.ModuleType("road_planning.route_planner")
route_planner.build_shaped_route = lambda *args, **kwargs: ([], {})
sys.modules["road_planning.route_planner"] = route_planner

from experiment.stability import analyze_stability


def build_metrics(
    steer_deltas,
    *,
    speed_mps=20.0,
    lateral_accel=None,
    speed_error_mps=None,
    throttle=None,
    brake=None,
):
    count = len(steer_deltas)
    times = [index * 0.05 for index in range(count)]
    return {
        "time": times,
        "speed": [speed_mps] * count,
        "speed_error": speed_error_mps or [0.0] * count,
        "steer_delta_signed": list(steer_deltas),
        "steer_rate_limit": [0.04] * count,
        "steer_rate_limited": [False] * count,
        "lateral_accel_signed": lateral_accel or [0.0] * count,
        "longitudinal_accel": [0.0] * count,
        "throttle": throttle or [0.2] * count,
        "brake": brake or [0.0] * count,
        "abs_ey": [0.1] * count,
        "route_completion": [min(index / max(count - 1, 1), 1.0) for index in range(count)],
        "collision_count": 0,
        "lane_boundary_violation_count": 0,
    }


class StabilityAnalysisTest(unittest.TestCase):
    def test_stable_run_passes_balanced_acceptance_limits(self):
        metrics = build_metrics([0.002 if index % 2 == 0 else -0.002 for index in range(400)])

        result = analyze_stability(
            metrics,
            speed_planner_mode="off",
            route_shape="true_straight",
            error_provider="ground_truth",
        )

        self.assertTrue(result["stability_passed"])
        self.assertEqual(result["stability_fail_reasons"], "")
        self.assertEqual(result["max_significant_steer_reversals_2s"], 0)
        self.assertEqual(result["significant_steer_reversals_per_10s"], 0.0)

    def test_sustained_alternating_steering_fails_with_specific_reasons(self):
        metrics = build_metrics([0.04 if index % 2 == 0 else -0.04 for index in range(400)])

        result = analyze_stability(
            metrics,
            speed_planner_mode="off",
            route_shape="true_straight",
            error_provider="ground_truth",
        )

        self.assertFalse(result["stability_passed"])
        self.assertIn("steer_reversals_2s", result["stability_fail_reasons"])
        self.assertIn("steer_reversals_rate", result["stability_fail_reasons"])
        self.assertIn("steer_delta_p95", result["stability_fail_reasons"])
        self.assertIn("steer_reversals_2s@", result["stability_fail_windows"])

    def test_perception_proxy_only_relaxes_tracking_error_threshold(self):
        metrics = build_metrics([0.0] * 400)
        metrics["abs_ey"] = [1.2] + [0.55] * 399

        ground_truth = analyze_stability(
            metrics,
            speed_planner_mode="adaptive",
            route_shape="curvy",
            error_provider="ground_truth",
        )
        perception = analyze_stability(
            metrics,
            speed_planner_mode="adaptive",
            route_shape="curvy",
            error_provider="perception_proxy",
        )

        self.assertIn("mean_lateral_error", ground_truth["stability_fail_reasons"])
        self.assertNotIn("mean_lateral_error", perception["stability_fail_reasons"])
        self.assertTrue(perception["stability_passed"])


if __name__ == "__main__":
    unittest.main()
