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
    pygame.display = SimpleNamespace(
        set_mode=lambda *args, **kwargs: None,
        set_caption=lambda *args, **kwargs: None,
        flip=lambda: None,
    )
    pygame.surfarray = SimpleNamespace(make_surface=lambda *args, **kwargs: None)
    pygame.init = lambda: None
    sys.modules["pygame"] = pygame

previous_route_planner = sys.modules.get("road_planning.route_planner")
route_planner = types.ModuleType("road_planning.route_planner")
route_planner.build_shaped_route = lambda *args, **kwargs: ([], {})
sys.modules["road_planning.route_planner"] = route_planner

from experiment.logging import STEP_LOG_FIELDS, build_step_snapshot
from experiment.metrics import SUMMARY_METRIC_FIELDS
from experiment.runtime import (
    is_headless_evaluation,
    is_retryable_route_error,
    next_metric_time,
    timed_controller_step,
)

if previous_route_planner is None:
    sys.modules.pop("road_planning.route_planner", None)
else:
    sys.modules["road_planning.route_planner"] = previous_route_planner


class EvaluationRuntimeTest(unittest.TestCase):
    def test_headless_evaluation_flag_disables_visual_runtime(self):
        self.assertTrue(is_headless_evaluation(SimpleNamespace(evaluation_headless=True)))
        self.assertFalse(is_headless_evaluation(SimpleNamespace()))

    def test_route_shape_rejections_are_retryable_across_spawn_candidates(self):
        self.assertTrue(is_retryable_route_error(
            RuntimeError("Failed to build true_straight route: total_abs_turn_deg=64.4 exceeds 20.0 deg.")
        ))
        self.assertTrue(is_retryable_route_error(
            RuntimeError("Failed to build curvy route: max_abs_curvature=0.001 is below requirement")
        ))
        self.assertFalse(is_retryable_route_error(RuntimeError("programming defect")))

    def test_timed_controller_step_returns_elapsed_milliseconds(self):
        timestamps = iter([10.0, 10.012])

        control, elapsed_ms = timed_controller_step(lambda: "control", clock=timestamps.__next__)

        self.assertEqual(control, "control")
        self.assertAlmostEqual(elapsed_ms, 12.0)

    def test_metric_time_is_lap_relative_and_monotonic_after_simulator_clock_reset(self):
        metrics = {"time": [172.80, 172.85]}

        self.assertAlmostEqual(next_metric_time(metrics, 0.05), 172.90)
        self.assertEqual(next_metric_time({}, 0.05), 0.0)

    def test_step_header_and_snapshot_include_controller_runtime(self):
        self.assertIn("controller_runtime_ms", STEP_LOG_FIELDS)
        location = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        vehicle = SimpleNamespace(
            get_transform=lambda: SimpleNamespace(location=location),
            get_velocity=lambda: SimpleNamespace(x=0.0, y=0.0, z=0.0),
            get_angular_velocity=lambda: SimpleNamespace(z=0.0),
        )
        control = SimpleNamespace(steer=0.0, throttle=0.0, brake=0.0)
        tracking = {
            "e_y": 0.0,
            "e_psi": 0.0,
            "target_x": 0.0,
            "target_y": 0.0,
            "target_yaw": 0.0,
        }

        snapshot = build_step_snapshot(
            vehicle,
            control,
            0.0,
            0.0,
            tracking,
            controller_debug={"controller_runtime_ms": 4.25},
        )

        self.assertEqual(snapshot["controller_runtime_ms"], 4.25)

    def test_summary_schema_exposes_literature_and_realtime_metrics(self):
        required = {
            "iae_e_y_m_s",
            "normalized_iae_e_y_m",
            "steer_spectrum_stability_index",
            "steer_spectrum_comfort_index",
            "max_abs_lateral_jerk_0p5s",
            "controller_runtime_p95_ms",
            "controller_runtime_deadline_miss_pct",
            "un_r79_reference_passed",
        }

        self.assertTrue(required.issubset(set(SUMMARY_METRIC_FIELDS)))


if __name__ == "__main__":
    unittest.main()
