import os
import sys
import types
import unittest
from types import SimpleNamespace

import numpy as np


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYTHONAPI_ROOT = os.path.dirname(PROJECT_ROOT)
CARLA_AGENTS_ROOT = os.path.join(PYTHONAPI_ROOT, "carla")
for path in (PROJECT_ROOT, CARLA_AGENTS_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

if "carla" not in sys.modules:
    carla = types.ModuleType("carla")
    carla.Transform = lambda *args, **kwargs: None
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

for module_name in ("experiment", "experiment.runtime", "error_providers"):
    sys.modules.pop(module_name, None)

from Speed_Planing.speed_planner import CurvatureSpeedPlanner, SpeedPlannerConfig
from experiment.runtime import apply_controller_step, build_curvature_preview, build_speed_planner
from experiment.logging import LOG_HEADER, build_step_snapshot


class CurvatureSpeedPlannerTest(unittest.TestCase):
    def test_straight_road_keeps_base_target_speed(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=72.0))

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05)

        self.assertAlmostEqual(target, 72.0 / 3.6)

    def test_turn_speed_is_limited_by_lateral_acceleration(self):
        planner = CurvatureSpeedPlanner(
            SpeedPlannerConfig(
                base_target_speed_kmh=90.0,
                max_lateral_accel=4.5,
                min_turn_speed_kmh=20.0,
            )
        )

        target = planner.plan_speed_mps(curvatures=[0.05], dt=0.05)

        self.assertAlmostEqual(target, (4.5 / 0.05) ** 0.5)

    def test_uses_largest_preview_curvature_for_early_slowdown(self):
        planner = CurvatureSpeedPlanner(
            SpeedPlannerConfig(
                base_target_speed_kmh=90.0,
                max_lateral_accel=4.5,
                min_turn_speed_kmh=20.0,
            )
        )

        target = planner.plan_speed_mps(curvatures=[0.0, 0.01, -0.04], dt=0.05)

        self.assertAlmostEqual(target, (4.5 / 0.04) ** 0.5)

    def test_speed_recovery_is_limited_after_curve(self):
        planner = CurvatureSpeedPlanner(
            SpeedPlannerConfig(
                base_target_speed_kmh=90.0,
                max_lateral_accel=4.5,
                min_turn_speed_kmh=20.0,
                max_accel=1.0,
            )
        )
        curve_target = planner.plan_speed_mps(curvatures=[0.05], dt=0.05)

        straight_target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05)

        self.assertAlmostEqual(straight_target, curve_target + 0.05)

    def test_default_tuning_keeps_70_kmh_on_medium_curves_when_stable(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))

        medium_curve_target = planner.plan_speed_mps(curvatures=[0.04], dt=0.05)

        self.assertAlmostEqual(medium_curve_target * 3.6, 70.0)

    def test_default_tuning_limits_only_emergency_curvature(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))

        tight_curve_target = planner.plan_speed_mps(curvatures=[0.14], dt=0.05)

        self.assertLess(tight_curve_target * 3.6, 70.0)

    def test_large_lateral_error_triggers_protective_slowdown(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, lateral_error_m=2.6)

        self.assertLess(target * 3.6, 50.0)

    def test_large_heading_error_triggers_protective_slowdown(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, heading_error_rad=np.radians(18.0))

        self.assertLess(target * 3.6, 50.0)

    def test_fast_lateral_error_growth_triggers_protective_slowdown(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))
        planner.plan_speed_mps(curvatures=[0.0], dt=0.05, lateral_error_m=0.0)

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, lateral_error_m=0.09)

        self.assertLess(target * 3.6, 70.0)
        self.assertEqual(planner.last_reason, "lateral_error_rate")
        self.assertGreater(planner.last_risk, 0.0)

    def test_fast_heading_error_growth_triggers_protective_slowdown(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))
        planner.plan_speed_mps(curvatures=[0.0], dt=0.05, heading_error_rad=0.0)

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, heading_error_rad=np.radians(1.0))

        self.assertLess(target * 3.6, 70.0)
        self.assertEqual(planner.last_reason, "heading_error_rate")
        self.assertGreater(planner.last_risk, 0.0)

    def test_hysteresis_delays_full_speed_recovery_after_risk(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))
        planner.plan_speed_mps(curvatures=[0.0], dt=0.05, lateral_error_m=2.6)

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, lateral_error_m=0.0)

        self.assertLess(target * 3.6, 70.0)
        self.assertEqual(planner.last_reason, "hold")


class RuntimeSpeedPlanningTest(unittest.TestCase):
    def test_log_header_includes_speed_plan_observability_fields(self):
        self.assertIn("speed_plan_risk", LOG_HEADER)
        self.assertIn("speed_plan_reason", LOG_HEADER)

    def test_build_speed_planner_uses_runtime_args(self):
        args = SimpleNamespace(
            target_speed=80.0,
            speed_planner_min_turn_speed=30.0,
            speed_planner_max_lateral_accel=4.0,
            speed_planner_max_accel=1.2,
            speed_planner_max_decel=5.5,
            speed_planner_lateral_error_warning=0.8,
            speed_planner_lateral_error_critical=1.6,
            speed_planner_heading_error_warning=6.0,
            speed_planner_heading_error_critical=12.0,
            speed_planner_lateral_error_rate_warning=0.7,
            speed_planner_lateral_error_rate_critical=1.4,
            speed_planner_heading_error_rate_warning=7.0,
            speed_planner_heading_error_rate_critical=14.0,
            speed_planner_recovery_hold_steps=4,
        )

        planner = build_speed_planner(args)

        self.assertAlmostEqual(planner.base_target_speed_mps, 80.0 / 3.6)
        self.assertAlmostEqual(planner.min_turn_speed_mps, 30.0 / 3.6)
        self.assertEqual(planner.config.max_lateral_accel, 4.0)
        self.assertEqual(planner.config.max_accel, 1.2)
        self.assertEqual(planner.config.max_decel, 5.5)
        self.assertEqual(planner.config.lateral_error_warning, 0.8)
        self.assertEqual(planner.config.lateral_error_critical, 1.6)
        self.assertAlmostEqual(planner.config.heading_error_warning_rad, np.radians(6.0))
        self.assertAlmostEqual(planner.config.heading_error_critical_rad, np.radians(12.0))
        self.assertEqual(planner.config.lateral_error_rate_warning, 0.7)
        self.assertEqual(planner.config.lateral_error_rate_critical, 1.4)
        self.assertAlmostEqual(planner.config.heading_error_rate_warning_rad, np.radians(7.0))
        self.assertAlmostEqual(planner.config.heading_error_rate_critical_rad, np.radians(14.0))
        self.assertEqual(planner.config.recovery_hold_steps, 4)

    def test_curvature_preview_samples_current_and_forward_route_points(self):
        route_trace = [(object(), None) for _ in range(8)]

        preview = build_curvature_preview(
            route_trace,
            reference_index=2,
            preview_steps=(0, 2, 4),
            curvature_fn=lambda trace, index: index * 0.01,
        )

        self.assertEqual(preview, [0.02, 0.04, 0.06])

    def test_step_snapshot_uses_dynamic_target_speed_for_error(self):
        vehicle = SimpleNamespace(
            get_transform=lambda: SimpleNamespace(location=SimpleNamespace(x=1.0, y=2.0)),
            get_velocity=lambda: SimpleNamespace(x=8.0, y=0.0, z=0.0),
        )
        control = SimpleNamespace(steer=0.2)
        tracking_errors = {
            "e_y": 0.1,
            "e_psi": 0.02,
            "target_x": 3.0,
            "target_y": 4.0,
            "target_yaw": 0.5,
        }

        snapshot = build_step_snapshot(
            vehicle,
            control,
            10.0,
            0.1,
            tracking_errors,
            speed_plan_risk=0.35,
            speed_plan_reason="heading_error_rate",
        )

        self.assertEqual(snapshot["speed_error"], 2.0)
        self.assertEqual(snapshot["target_speed"], 10.0)
        self.assertEqual(snapshot["speed_plan_risk"], 0.35)
        self.assertEqual(snapshot["speed_plan_reason"], "heading_error_rate")

    def test_apply_controller_step_passes_planned_speed_to_controller(self):
        class RecordingController:
            def __init__(self):
                self.planned_target_speed_mps = None

            def run_step(
                self,
                vehicle,
                target_waypoint,
                curvature=0.0,
                reference_waypoint=None,
                planned_target_speed_mps=None,
                tracking_errors=None,
            ):
                self.planned_target_speed_mps = planned_target_speed_mps
                return object()

        controller = RecordingController()
        route_trace = [
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=0.0, y=0.0))), None),
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=1.0, y=0.0))), None),
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=2.0, y=0.0))), None),
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=3.0, y=0.0))), None),
        ]

        apply_controller_step(
            controller,
            vehicle=object(),
            target_waypoint=object(),
            route_trace=route_trace,
            target_index=1,
            reference_waypoint=object(),
            reference_index=1,
            planned_target_speed_mps=7.5,
        )

        self.assertEqual(controller.planned_target_speed_mps, 7.5)

    def test_apply_controller_step_uses_lookahead_target_curvature(self):
        class RecordingController:
            def __init__(self):
                self.curvature = None

            def run_step(
                self,
                vehicle,
                target_waypoint,
                curvature=0.0,
                reference_waypoint=None,
                planned_target_speed_mps=None,
                tracking_errors=None,
            ):
                self.curvature = curvature
                return object()

        controller = RecordingController()
        route_trace = [
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=0.0, y=0.0))), None),
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=1.0, y=0.0))), None),
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=2.0, y=0.0))), None),
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=2.0, y=1.0))), None),
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=2.0, y=2.0))), None),
        ]

        apply_controller_step(
            controller,
            vehicle=object(),
            target_waypoint=object(),
            route_trace=route_trace,
            target_index=2,
            reference_waypoint=object(),
            reference_index=1,
        )

        self.assertGreater(controller.curvature, 0.0)

    def test_apply_controller_step_passes_tracking_errors_to_controller(self):
        class RecordingController:
            def __init__(self):
                self.tracking_errors = None

            def run_step(
                self,
                vehicle,
                target_waypoint,
                curvature=0.0,
                reference_waypoint=None,
                planned_target_speed_mps=None,
                tracking_errors=None,
            ):
                self.tracking_errors = tracking_errors
                return object()

        controller = RecordingController()
        errors = {"e_y": 1.2, "e_psi": 0.3}
        route_trace = [
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=0.0, y=0.0))), None),
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=1.0, y=0.0))), None),
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=2.0, y=0.0))), None),
        ]

        apply_controller_step(
            controller,
            vehicle=object(),
            target_waypoint=object(),
            route_trace=route_trace,
            target_index=1,
            reference_waypoint=object(),
            reference_index=1,
            tracking_errors=errors,
        )

        self.assertIs(controller.tracking_errors, errors)


if __name__ == "__main__":
    unittest.main()
