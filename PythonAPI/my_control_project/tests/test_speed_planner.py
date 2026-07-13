import os
import sys
import types
import unittest
from types import SimpleNamespace
from unittest import mock

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
    carla.VehicleControl = type("VehicleControl", (), {})
    carla.LaneType = SimpleNamespace(Driving=1)
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

_previous_route_planner = sys.modules.get("road_planning.route_planner")
route_planner = types.ModuleType("road_planning.route_planner")
route_planner.build_shaped_route = lambda *args, **kwargs: ([], {})
sys.modules["road_planning.route_planner"] = route_planner

for module_name in ("experiment", "experiment.runtime", "error_providers"):
    sys.modules.pop(module_name, None)

from speed_planning.speed_planner import CurvatureSpeedPlanner, SpeedPlannerConfig
from experiment.metrics import SPEED_PLAN_REASONS, build_experiment_metadata
from experiment.runtime import (
    SimpleHUD,
    apply_controller_step,
    build_controller_curvature_preview,
    build_curvature_preview,
    build_speed_planner_curvature_preview,
    build_speed_planner,
    resolve_route_setup,
)
import experiment.runtime as runtime_module
from experiment.logging import (
    LOG_HEADER,
    SUMMARY_HEADER,
    build_human_summary,
    build_step_snapshot,
    build_summary_rows_with_text,
)

if _previous_route_planner is None:
    sys.modules.pop("road_planning.route_planner", None)
else:
    sys.modules["road_planning.route_planner"] = _previous_route_planner


def _planner_selection_fixture(planner_mode):
    def waypoint(x_m):
        location = SimpleNamespace(x=float(x_m), y=0.0, z=0.0)
        return SimpleNamespace(
            transform=SimpleNamespace(location=location, rotation=SimpleNamespace(yaw=0.0)),
            lane_width=3.5,
        )

    spawn_points = [
        SimpleNamespace(location=SimpleNamespace(x=0.0, y=0.0, z=0.0)),
        SimpleNamespace(location=SimpleNamespace(x=20.0, y=0.0, z=0.0)),
    ]
    world_map = SimpleNamespace(get_spawn_points=lambda: spawn_points)
    world = SimpleNamespace(get_map=lambda: world_map)
    route_trace = [(waypoint(0.0), "LANEFOLLOW"), (waypoint(20.0), "LANEFOLLOW")]
    route_features = {
        "length": 20.0,
        "route_label": "straight",
        "total_abs_turn": 0.0,
        "sign_changes": 0,
        "turn_segments": 0,
    }
    args = SimpleNamespace(
        spawn_index=0,
        destination_index=1,
        route_resolution=1.0,
        route_min_length_m=20.0,
        route_max_waypoints=100,
        route_shape="straight",
        route_length_tolerance=0.05,
        target_speed=70.0,
        planner_mode=planner_mode,
        planner_validation_spacing=0.25,
        planner_vehicle_half_width=1.05,
        planner_lane_margin=0.35,
        planner_topology_beam_width=24,
        planner_lateral_beam_width=64,
        planner_candidate_cap=512,
        planner_deadline=5.0,
        planner_max_curvature=0.20,
        planner_max_curvature_rate=0.020,
        planner_max_lateral_accel=6.5,
        planner_max_lateral_jerk=10.0,
        reference_sample_spacing=1.0,
        reference_smoothing_window=7,
        reference_lane_margin=0.35,
        reference_max_centerline_offset=1.25,
        controller_name="pid",
    )
    return world, args, route_trace, route_features


class PlannerSelectionTest(unittest.TestCase):
    def test_legacy_mode_builds_only_legacy_trajectory_and_returns_it(self):
        world, args, topology_route, route_features = _planner_selection_fixture("legacy")
        trajectory = SimpleNamespace(content_hash=lambda: "legacy-hash")
        reference_trace = list(topology_route)

        with mock.patch.object(runtime_module, "build_shaped_route", return_value=(topology_route, dict(route_features))) as shaped, \
                mock.patch.object(runtime_module, "build_legacy_reference_trajectory", create=True, return_value=(trajectory, reference_trace, {"planner_mode": "legacy"})) as legacy, \
                mock.patch.object(runtime_module, "plan_frenet_reference", create=True) as frenet:
            result = resolve_route_setup(world, args, lambda index, points: index, mock.Mock())

        self.assertEqual(len(result), 7)
        self.assertIs(result[-1], trajectory)
        legacy.assert_called_once()
        frenet.assert_not_called()
        self.assertEqual(shaped.call_args.kwargs["beam_width"], 24)
        self.assertNotIn("controller", repr(legacy.call_args).lower())

    def test_frenet_mode_builds_only_frenet_trajectory_and_returns_its_trace(self):
        world, args, topology_route, route_features = _planner_selection_fixture("frenet")
        reference_trace = list(reversed(topology_route))
        trajectory = SimpleNamespace(
            content_hash=lambda: "frenet-hash",
            to_route_trace=mock.Mock(return_value=reference_trace),
        )

        with mock.patch.object(runtime_module, "build_shaped_route", return_value=(topology_route, dict(route_features))), \
                mock.patch.object(runtime_module, "build_legacy_reference_trajectory", create=True) as legacy, \
                mock.patch.object(runtime_module, "plan_frenet_reference", create=True, return_value=(trajectory, {"planner_mode": "frenet"})) as frenet:
            result = resolve_route_setup(world, args, lambda index, points: index, mock.Mock())

        self.assertEqual(len(result), 7)
        self.assertIs(result[4], reference_trace)
        self.assertIs(result[-1], trajectory)
        legacy.assert_not_called()
        frenet.assert_called_once()
        planner_config = frenet.call_args.args[3]
        self.assertEqual(planner_config.lateral_beam_width, 64)
        self.assertNotIn("controller", repr(frenet.call_args).lower())


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

    def test_default_tuning_uses_less_aggressive_protective_floor(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, heading_error_rad=np.radians(18.0))

        self.assertAlmostEqual(target * 3.6, 40.0)

    def test_default_tuning_limits_only_emergency_curvature(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))

        tight_curve_target = planner.plan_speed_mps(curvatures=[0.14], dt=0.05)

        self.assertLess(tight_curve_target * 3.6, 70.0)

    def test_very_sharp_curve_can_drop_below_normal_turn_floor(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=120.0))

        target = planner.plan_speed_mps(curvatures=[0.0, 0.112], dt=0.05)

        self.assertLess(target * 3.6, 40.0)
        self.assertGreaterEqual(target * 3.6, 36.0)
        self.assertEqual(planner.last_reason, "curvature")

    def test_entry_corner_cap_limits_high_speed_before_curve(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=120.0))

        entry_target = planner.plan_speed_mps(curvatures=[0.0, 0.03], dt=0.05)

        self.assertAlmostEqual(entry_target * 3.6, 70.0)
        self.assertEqual(planner.last_reason, "entry_curvature")

    def test_entry_corner_cap_scales_with_preview_curvature(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=120.0))

        entry_target = planner.plan_speed_mps(curvatures=[0.0, 0.025], dt=0.05)

        self.assertGreater(entry_target * 3.6, 70.0)
        self.assertLess(entry_target * 3.6, 120.0)
        self.assertEqual(planner.last_reason, "entry_curvature")

    def test_entry_corner_cap_does_not_limit_straight_high_speed(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=120.0))

        straight_target = planner.plan_speed_mps(curvatures=[0.0, 0.01], dt=0.05)

        self.assertAlmostEqual(straight_target * 3.6, 120.0)

    def test_large_lateral_error_triggers_protective_slowdown(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, lateral_error_m=2.6)

        self.assertLess(target * 3.6, 50.0)

    def test_large_heading_error_triggers_protective_slowdown(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, heading_error_rad=np.radians(18.0))

        self.assertLess(target * 3.6, 50.0)

    def test_combined_tracking_errors_trigger_stronger_protective_slowdown(self):
        lateral_only_planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))
        combined_planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))

        lateral_only_target = lateral_only_planner.plan_speed_mps(
            curvatures=[0.0],
            dt=0.05,
            lateral_error_m=2.0,
        )
        combined_target = combined_planner.plan_speed_mps(
            curvatures=[0.0],
            dt=0.05,
            lateral_error_m=2.0,
            heading_error_rad=np.radians(15.0),
        )

        self.assertLess(combined_target, lateral_only_target)
        self.assertEqual(combined_planner.last_reason, "heading_error")
        self.assertGreater(combined_planner.last_risk, lateral_only_planner.last_risk)

    def test_fast_lateral_error_growth_triggers_protective_slowdown(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))
        planner.plan_speed_mps(curvatures=[0.0], dt=0.05, lateral_error_m=0.0)

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, lateral_error_m=0.9)

        self.assertLess(target * 3.6, 70.0)
        self.assertEqual(planner.last_reason, "lateral_error_rate")
        self.assertGreater(planner.last_risk, 0.0)

    def test_small_lateral_error_growth_does_not_trigger_rate_slowdown(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))
        planner.plan_speed_mps(curvatures=[0.0], dt=0.05, lateral_error_m=0.0)

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, lateral_error_m=0.2)

        self.assertAlmostEqual(target * 3.6, 70.0)
        self.assertEqual(planner.last_reason, "none")

    def test_fast_heading_error_growth_triggers_protective_slowdown(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))
        planner.plan_speed_mps(curvatures=[0.0], dt=0.05, heading_error_rad=0.0)

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, heading_error_rad=np.radians(9.0))

        self.assertLess(target * 3.6, 70.0)
        self.assertEqual(planner.last_reason, "heading_error_rate")
        self.assertGreater(planner.last_risk, 0.0)

    def test_small_heading_error_growth_does_not_trigger_rate_slowdown(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))
        planner.plan_speed_mps(curvatures=[0.0], dt=0.05, heading_error_rad=0.0)

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, heading_error_rad=np.radians(3.0))

        self.assertAlmostEqual(target * 3.6, 70.0)
        self.assertEqual(planner.last_reason, "none")

    def test_hysteresis_delays_full_speed_recovery_after_risk(self):
        planner = CurvatureSpeedPlanner(SpeedPlannerConfig(base_target_speed_kmh=70.0))
        planner.plan_speed_mps(curvatures=[0.0], dt=0.05, lateral_error_m=2.6)

        target = planner.plan_speed_mps(curvatures=[0.0], dt=0.05, lateral_error_m=0.0)

        self.assertLess(target * 3.6, 70.0)
        self.assertEqual(planner.last_reason, "hold")


class RuntimeSpeedPlanningTest(unittest.TestCase):
    def test_continuous_reference_step_uses_projection_and_arc_length_target(self):
        reference_waypoint = object()
        target_waypoint = object()
        trajectory = SimpleNamespace(
            waypoint_at=mock.Mock(side_effect=lambda s_m: {
                12.5: reference_waypoint,
                20.0: target_waypoint,
            }[s_m]),
        )
        projection = SimpleNamespace(s_ref_m=12.5, segment_index=4, projection_distance_m=0.3)
        tracker = SimpleNamespace(
            update=mock.Mock(return_value=projection),
            target_sample=mock.Mock(return_value=SimpleNamespace(s_m=20.0, road_option="LEFT")),
        )
        vehicle = SimpleNamespace(
            get_location=lambda: SimpleNamespace(x=3.0, y=4.0),
            get_transform=lambda: SimpleNamespace(rotation=SimpleNamespace(yaw=30.0)),
        )

        result = runtime_module.build_continuous_reference_step(
            tracker,
            trajectory,
            vehicle,
            speed_mps=10.0,
            control_dt=0.05,
            lookahead_m=7.5,
            allow_recovery=False,
        )

        self.assertIs(result.reference_waypoint, reference_waypoint)
        self.assertIs(result.target_waypoint, target_waypoint)
        self.assertIs(result.projection, projection)
        self.assertEqual(result.road_option, "LEFT")
        tracker.update.assert_called_once_with(3.0, 4.0, np.radians(30.0), 10.0, 0.05, allow_recovery=False)
        tracker.target_sample.assert_called_once_with(7.5)

    def test_tracker_curvature_previews_use_physical_metres(self):
        tracker = SimpleNamespace(
            curvature_preview=mock.Mock(side_effect=lambda distances: [float(value) for value in distances])
        )

        controller_preview = build_controller_curvature_preview(
            route_trace=None,
            reference_index=None,
            speed_mps=40.0,
            horizon=4,
            dt=0.05,
            tracker=tracker,
        )
        speed_preview = build_speed_planner_curvature_preview(
            route_trace=None,
            reference_index=None,
            speed_mps=40.0,
            preview_time_step=0.40,
            min_preview_spacing_m=5.0,
            preview_count=4,
            tracker=tracker,
        )

        self.assertEqual(controller_preview, [0.0, 2.0, 4.0, 6.0])
        self.assertEqual(speed_preview, [0.0, 16.0, 32.0, 48.0])
        self.assertEqual(tracker.curvature_preview.call_args_list[0].args[0], [0.0])
        self.assertEqual(tracker.curvature_preview.call_args_list[1].args[0], [0.0, 2.0, 4.0, 6.0])
        self.assertEqual(tracker.curvature_preview.call_args_list[2].args[0], [0.0, 16.0, 32.0, 48.0])

    def test_log_header_includes_speed_plan_observability_fields(self):
        self.assertIn("speed_plan_risk", LOG_HEADER)
        self.assertIn("speed_plan_reason", LOG_HEADER)
        self.assertIn("controller_speed_profile", LOG_HEADER)
        self.assertIn("mpc_cache_reused", LOG_HEADER)
        self.assertIn("mpc_reuse_count", LOG_HEADER)
        self.assertIn("mpc_curvature0", LOG_HEADER)
        self.assertIn("mpc_curvature_max", LOG_HEADER)
        self.assertIn("mpc_active_horizon", LOG_HEADER)
        self.assertIn("mpc_steer_limit", LOG_HEADER)
        self.assertIn("mpc_reference_steer0", LOG_HEADER)
        self.assertIn("raw_steer", LOG_HEADER)
        self.assertIn("limited_steer", LOG_HEADER)
        self.assertIn("steer_rate_limit", LOG_HEADER)
        self.assertIn("steer_rate_limited", LOG_HEADER)
        self.assertIn("current_curvature", LOG_HEADER)
        self.assertIn("preview_curvature", LOG_HEADER)
        self.assertIn("lqr_gain_update_reason", LOG_HEADER)
        self.assertIn("mpc_solve_mode", LOG_HEADER)
        self.assertIn("longitudinal_accel", LOG_HEADER)
        for field in (
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
        ):
            self.assertIn(field, LOG_HEADER)
        for field in (
            "planner_version",
            "trajectory_hash",
            "planning_duration_s",
            "planner_candidate_count",
            "planner_selected_candidate_id",
            "reference_max_abs_curvature",
            "reference_max_abs_curvature_rate",
            "reference_min_footprint_clearance_m",
            "planner_rejection_counts",
        ):
            self.assertIn(field, SUMMARY_HEADER)

    def test_fixed_throttle_brake_reason_is_counted_in_summaries(self):
        self.assertIn("fixed_throttle_brake", SPEED_PLAN_REASONS)
        self.assertIn("speed_plan_reason_fixed_throttle_brake_count", SUMMARY_HEADER)
        self.assertIn("p95_abs_steer_delta", SUMMARY_HEADER)
        self.assertIn("significant_steer_reversals_per_10s", SUMMARY_HEADER)
        self.assertIn("max_significant_steer_reversals_2s", SUMMARY_HEADER)
        self.assertIn("steer_rate_limit_hit_pct", SUMMARY_HEADER)
        self.assertIn("p95_abs_speed_error", SUMMARY_HEADER)
        self.assertIn("p95_abs_lateral_accel", SUMMARY_HEADER)
        self.assertIn("p95_abs_lateral_jerk", SUMMARY_HEADER)
        self.assertIn("p95_abs_longitudinal_jerk", SUMMARY_HEADER)
        self.assertIn("throttle_brake_switches_per_10s", SUMMARY_HEADER)
        self.assertIn("stability_passed", SUMMARY_HEADER)
        self.assertIn("stability_fail_reasons", SUMMARY_HEADER)

    def test_hud_shows_target_speed_mode_and_pedal_outputs(self):
        vehicle = SimpleNamespace(
            get_velocity=lambda: SimpleNamespace(x=10.0, y=0.0, z=0.0),
            get_transform=lambda: SimpleNamespace(
                location=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            ),
        )
        control = SimpleNamespace(steer=-0.1, throttle=0.65, brake=0.0)
        hud = SimpleHUD(1280, 720)

        hud.update(
            vehicle,
            control,
            "pid",
            target_speed_mps=70.0 / 3.6,
            speed_planner_mode="off",
        )
        text = "\n".join(hud.info_text)

        self.assertIn("Controller: PID", text)
        self.assertIn("Speed:  36.0 km/h", text)
        self.assertIn("Target Speed:  70.0 km/h", text)
        self.assertIn("Speed Error: -34.0 km/h", text)
        self.assertIn("Speed Mode: off", text)
        self.assertNotIn("Speed Reason:", text)
        self.assertIn("Throttle: 0.65", text)
        self.assertIn("Brake: 0.00", text)

    def test_human_summary_explains_best_controller_and_conditions(self):
        def make_summary(controller, mean_ey, rms_ey, heading_deg, speed_error, steer_delta):
            values = {key: "" for key in SUMMARY_HEADER}
            values.update(
                {
                    "controller": controller,
                    "error_provider": "ground_truth",
                    "speed_planner_mode": "adaptive",
                    "route_shape": "s_curve",
                    "route_label": "s_curve",
                    "route_min_length_m": 1700.0,
                    "route_length_tolerance": 0.05,
                    "noise_lateral_std": 0.1,
                    "noise_heading_std_deg": 1.0,
                    "perception_delay_steps": 0,
                    "perception_dropout_probability": 0.0,
                    "perception_smoothing_alpha": 1.0,
                    "spawn_index": 12,
                    "route_waypoints": 1800,
                    "target_speed_kmh": 70.0,
                    "route_total_abs_turn_deg": 240.0,
                    "route_mean_abs_curvature": 0.012,
                    "route_max_abs_curvature": 0.045,
                    "mean_abs_e_y": mean_ey,
                    "rms_e_y": rms_ey,
                    "max_abs_e_y": mean_ey * 2.0,
                    "mean_abs_e_psi_deg": heading_deg,
                    "rms_e_psi_deg": heading_deg * 1.2,
                    "mean_abs_speed_error": speed_error,
                    "mean_abs_steer_delta": steer_delta,
                    "max_abs_steer_delta": steer_delta * 2.0,
                    "mean_planned_speed_kmh": 62.0,
                    "min_planned_speed_kmh": 45.0,
                    "max_planned_speed_kmh": 70.0,
                    "speed_plan_reason_none_count": 10,
                    "speed_plan_reason_entry_curvature_count": 5,
                }
            )
            return [values[key] for key in SUMMARY_HEADER if not key.startswith("readable_")]

        lines = build_human_summary(
            [
                make_summary("pid", 0.6, 0.8, 4.0, 1.2, 0.04),
                make_summary("mpc", 0.3, 0.4, 2.0, 0.6, 0.02),
            ],
            {
                "destination_index": 27,
                "seed": 123,
                "route": {
                    "length_m": 1725.0,
                    "route_label": "s_curve",
                    "total_abs_turn_deg": 240.0,
                    "mean_abs_curvature": 0.012,
                    "max_abs_curvature": 0.045,
                },
            },
        )
        text = "\n".join(lines)

        self.assertIn("Result: MPC performed best in this test.", text)
        self.assertIn("Test conditions", text)
        self.assertIn("route s_curve / s_curve", text)
        self.assertIn("speed mode adaptive speed planning", text)

    def test_summary_rows_include_readable_english_text_columns(self):
        def make_summary(controller, mean_ey):
            values = {key: "" for key in SUMMARY_HEADER}
            values.update(
                {
                    "controller": controller,
                    "error_provider": "ground_truth",
                    "speed_planner_mode": "adaptive",
                    "route_shape": "s_curve",
                    "route_label": "s_curve",
                    "route_min_length_m": 1700.0,
                    "route_length_tolerance": 0.05,
                    "noise_lateral_std": 0.1,
                    "noise_heading_std_deg": 1.0,
                    "perception_delay_steps": 0,
                    "perception_dropout_probability": 0.0,
                    "perception_smoothing_alpha": 1.0,
                    "spawn_index": 12,
                    "route_waypoints": 1800,
                    "target_speed_kmh": 70.0,
                    "route_total_abs_turn_deg": 240.0,
                    "route_mean_abs_curvature": 0.012,
                    "route_max_abs_curvature": 0.045,
                    "mean_abs_e_y": mean_ey,
                    "rms_e_y": mean_ey,
                    "max_abs_e_y": mean_ey * 2.0,
                    "mean_abs_e_psi_deg": mean_ey,
                    "rms_e_psi_deg": mean_ey,
                    "mean_abs_speed_error": mean_ey,
                    "mean_abs_steer_delta": mean_ey,
                    "max_abs_steer_delta": mean_ey,
                    "mean_planned_speed_kmh": 62.0,
                    "min_planned_speed_kmh": 45.0,
                    "max_planned_speed_kmh": 70.0,
                    "speed_plan_reason_none_count": 10,
                }
            )
            return [values[key] for key in SUMMARY_HEADER if not key.startswith("readable_")]

        rows = build_summary_rows_with_text(
            [make_summary("pid", 0.6), make_summary("mpc", 0.3)],
            {
                "destination_index": 27,
                "seed": 123,
                "route": {
                    "length_m": 1725.0,
                    "route_label": "s_curve",
                    "total_abs_turn_deg": 240.0,
                    "mean_abs_curvature": 0.012,
                    "max_abs_curvature": 0.045,
                },
            },
        )

        self.assertEqual(len(rows[0]), len(SUMMARY_HEADER))
        self.assertIn("readable_result", SUMMARY_HEADER)
        self.assertIn("Result: MPC performed best", rows[0][SUMMARY_HEADER.index("readable_result")])
        self.assertIn("Test conditions", rows[0][SUMMARY_HEADER.index("readable_test_conditions")])
        self.assertIn("PID", rows[0][SUMMARY_HEADER.index("readable_controller_summary")])

    def test_build_speed_planner_uses_runtime_args(self):
        args = SimpleNamespace(
            target_speed=80.0,
            speed_planner_min_turn_speed=30.0,
            speed_planner_recovery_min_speed=26.0,
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
            speed_planner_lateral_error_rate_activation=0.9,
            speed_planner_heading_error_rate_activation=8.5,
            speed_planner_error_rate_alpha=0.35,
            speed_planner_recovery_hold_steps=4,
            speed_planner_entry_max_speed=75.0,
            speed_planner_entry_curvature_threshold=0.03,
            speed_planner_entry_full_cap_curvature=0.05,
            speed_planner_emergency_turn_speed=28.0,
            speed_planner_emergency_curvature_threshold=0.07,
            speed_planner_emergency_full_cap_curvature=0.11,
        )

        planner = build_speed_planner(args)

        self.assertAlmostEqual(planner.base_target_speed_mps, 80.0 / 3.6)
        self.assertAlmostEqual(planner.min_turn_speed_mps, 30.0 / 3.6)
        self.assertAlmostEqual(planner.recovery_min_speed_mps, 26.0 / 3.6)
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
        self.assertEqual(planner.config.lateral_error_rate_activation, 0.9)
        self.assertAlmostEqual(planner.config.heading_error_rate_activation_rad, np.radians(8.5))
        self.assertEqual(planner.config.error_rate_filter_alpha, 0.35)
        self.assertEqual(planner.config.recovery_hold_steps, 4)
        self.assertEqual(planner.config.entry_max_speed_kmh, 75.0)
        self.assertEqual(planner.config.entry_curvature_threshold, 0.03)
        self.assertEqual(planner.config.entry_full_cap_curvature, 0.05)
        self.assertEqual(planner.config.emergency_turn_speed_kmh, 28.0)
        self.assertEqual(planner.config.emergency_curvature_threshold, 0.07)
        self.assertEqual(planner.config.emergency_full_cap_curvature, 0.11)

    def _default_speed_planner_args(self, limit_profile):
        return SimpleNamespace(
            target_speed=120.0,
            speed_planner_limit_profile=limit_profile,
            speed_planner_min_turn_speed=48.0,
            speed_planner_recovery_min_speed=40.0,
            speed_planner_max_lateral_accel=18.0,
            speed_planner_max_accel=3.5,
            speed_planner_max_decel=9.0,
            speed_planner_lateral_error_warning=1.6,
            speed_planner_lateral_error_critical=2.6,
            speed_planner_heading_error_warning=12.0,
            speed_planner_heading_error_critical=18.0,
            speed_planner_lateral_error_rate_warning=0.8,
            speed_planner_lateral_error_rate_critical=1.6,
            speed_planner_heading_error_rate_warning=8.0,
            speed_planner_heading_error_rate_critical=16.0,
            speed_planner_lateral_error_rate_activation=0.8,
            speed_planner_heading_error_rate_activation=8.0,
            speed_planner_error_rate_alpha=0.25,
            speed_planner_recovery_hold_steps=3,
            speed_planner_entry_max_speed=70.0,
            speed_planner_entry_curvature_threshold=0.015,
            speed_planner_entry_full_cap_curvature=0.03,
            speed_planner_emergency_turn_speed=36.0,
            speed_planner_emergency_curvature_threshold=0.08,
            speed_planner_emergency_full_cap_curvature=0.12,
        )

    def test_controller_speed_limit_profiles_apply_realistic_high_speed_limits(self):
        args = self._default_speed_planner_args("controller")

        pid_planner = build_speed_planner(args, controller_name="pid")
        lqr_planner = build_speed_planner(args, controller_name="lqr")
        mpc_planner = build_speed_planner(args, controller_name="mpc")

        self.assertLess(pid_planner.config.max_lateral_accel, lqr_planner.config.max_lateral_accel)
        self.assertLess(mpc_planner.config.max_lateral_accel, pid_planner.config.max_lateral_accel)
        self.assertLess(mpc_planner.config.entry_max_speed_kmh, pid_planner.config.entry_max_speed_kmh)
        self.assertLessEqual(lqr_planner.config.entry_max_speed_kmh, pid_planner.config.entry_max_speed_kmh)
        self.assertLessEqual(
            mpc_planner.config.entry_curvature_threshold,
            pid_planner.config.entry_curvature_threshold,
        )
        self.assertLessEqual(
            lqr_planner.config.heading_error_rate_warning_rad,
            pid_planner.config.heading_error_rate_warning_rad,
        )
        self.assertLess(
            mpc_planner.config.heading_error_rate_warning_rad,
            pid_planner.config.heading_error_rate_warning_rad,
        )

    def test_global_speed_limit_profile_preserves_same_limits_for_all_controllers(self):
        args = self._default_speed_planner_args("global")

        pid_planner = build_speed_planner(args, controller_name="pid")
        mpc_planner = build_speed_planner(args, controller_name="mpc")

        self.assertEqual(pid_planner.config.max_lateral_accel, 18.0)
        self.assertEqual(mpc_planner.config.max_lateral_accel, 18.0)
        self.assertEqual(pid_planner.config.entry_max_speed_kmh, 70.0)
        self.assertEqual(mpc_planner.config.entry_max_speed_kmh, 70.0)

    def test_missing_speed_limit_profile_defaults_to_controller_limits(self):
        args = self._default_speed_planner_args("controller")
        delattr(args, "speed_planner_limit_profile")

        mpc_planner = build_speed_planner(args, controller_name="mpc")

        self.assertLess(mpc_planner.config.max_lateral_accel, 18.0)
        self.assertLess(mpc_planner.config.entry_max_speed_kmh, 70.0)
        self.assertLess(mpc_planner.config.entry_curvature_threshold, 0.015)

    def test_controller_speed_limit_profile_is_recorded_in_metadata(self):
        args = SimpleNamespace(
            error_provider="ground_truth",
            speed_planner_mode="adaptive",
            speed_planner_limit_profile="controller",
            route_shape="s_curve",
            route_label="s_curve",
            route_min_length_m=1700.0,
            route_length_tolerance=0.05,
            noise_lateral_std=0.1,
            noise_heading_std_deg=1.0,
            perception_delay_steps=0,
            perception_dropout_probability=0.0,
            perception_smoothing_alpha=1.0,
        )

        metadata = build_experiment_metadata("mpc", args)

        self.assertIn("controller:mpc", metadata)

    def test_missing_speed_limit_profile_metadata_defaults_to_controller_limits(self):
        args = SimpleNamespace(
            error_provider="ground_truth",
            speed_planner_mode="adaptive",
            route_shape="s_curve",
            route_label="s_curve",
            route_min_length_m=1700.0,
            route_length_tolerance=0.05,
            noise_lateral_std=0.1,
            noise_heading_std_deg=1.0,
            perception_delay_steps=0,
            perception_dropout_probability=0.0,
            perception_smoothing_alpha=1.0,
        )

        metadata = build_experiment_metadata("lqr", args)

        self.assertIn("controller:lqr", metadata)

    def test_curvature_preview_samples_current_and_forward_route_points(self):
        route_trace = [(object(), None) for _ in range(8)]

        preview = build_curvature_preview(
            route_trace,
            reference_index=2,
            preview_steps=(0, 2, 4),
            curvature_fn=lambda trace, index: index * 0.01,
        )

        self.assertEqual(preview, [0.02, 0.04, 0.06])

    def test_default_curvature_preview_stays_near_curve_entry(self):
        route_trace = [(object(), None) for _ in range(100)]

        preview = build_curvature_preview(
            route_trace,
            reference_index=10,
            curvature_fn=lambda trace, index: index,
        )

        self.assertEqual(preview, [10, 15, 20, 25])

    def test_controller_curvature_preview_uses_speed_scaled_route_offsets(self):
        route_trace = [(object(), None) for _ in range(20)]

        preview = build_controller_curvature_preview(
            route_trace,
            reference_index=3,
            speed_mps=40.0,
            horizon=4,
            dt=0.05,
            curvature_fn=lambda trace, index: index * 0.01,
        )

        self.assertEqual(preview, [0.03, 0.05, 0.07, 0.09])

    def test_controller_curvature_preview_uses_shorter_spacing_on_sharp_curves(self):
        route_trace = [(object(), None) for _ in range(20)]

        preview = build_controller_curvature_preview(
            route_trace,
            reference_index=3,
            speed_mps=40.0,
            horizon=4,
            dt=0.05,
            curvature_fn=lambda trace, index: 0.08 if index == 3 else index * 0.01,
        )

        self.assertEqual(preview, [0.08, 0.04, 0.05, 0.06])

    def test_speed_planner_curvature_preview_extends_with_high_speed(self):
        route_trace = [(object(), None) for _ in range(80)]

        preview = build_speed_planner_curvature_preview(
            route_trace,
            reference_index=10,
            speed_mps=40.0,
            preview_time_step=0.40,
            min_preview_spacing_m=5.0,
            preview_count=4,
            curvature_fn=lambda trace, index: index * 0.01,
        )

        self.assertEqual(preview, [0.10, 0.26, 0.42, 0.58])

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
        self.assertEqual(snapshot["controller_speed_profile"], "base")
        self.assertFalse(snapshot["mpc_cache_reused"])
        self.assertEqual(snapshot["mpc_reuse_count"], 0)
        self.assertEqual(snapshot["mpc_curvature0"], 0.0)
        self.assertEqual(snapshot["mpc_curvature_max"], 0.0)

    def test_step_snapshot_records_controller_speed_profile(self):
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
            controller_debug={
                "controller_speed_profile": "lqr_low_recovery",
            },
        )

        self.assertEqual(snapshot["controller_speed_profile"], "lqr_low_recovery")

    def test_step_snapshot_records_mpc_diagnostics(self):
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
            controller_debug={
                "controller_speed_profile": "mpc_low_recovery",
                "mpc_cache_reused": True,
                "mpc_reuse_count": 2,
                "mpc_curvature0": 0.01,
                "mpc_curvature_max": 0.03,
                "mpc_active_horizon": 12,
                "mpc_steer_limit": 0.28,
                "mpc_reference_steer0": 0.015,
                "raw_steer": 0.26,
                "steer_rate_limit": 0.04,
                "steer_rate_limited": True,
                "current_curvature": 0.012,
                "preview_curvature": 0.031,
                "lqr_gain_update_reason": "speed_delta",
                "mpc_solve_mode": "fresh",
            },
            previous_speed=7.5,
            dt=0.05,
        )

        self.assertEqual(snapshot["controller_speed_profile"], "mpc_low_recovery")
        self.assertTrue(snapshot["mpc_cache_reused"])
        self.assertEqual(snapshot["mpc_reuse_count"], 2)
        self.assertEqual(snapshot["mpc_curvature0"], 0.01)
        self.assertEqual(snapshot["mpc_curvature_max"], 0.03)
        self.assertEqual(snapshot["mpc_active_horizon"], 12)
        self.assertEqual(snapshot["mpc_steer_limit"], 0.28)
        self.assertEqual(snapshot["mpc_reference_steer0"], 0.015)
        self.assertEqual(snapshot["raw_steer"], 0.26)
        self.assertEqual(snapshot["limited_steer"], 0.2)
        self.assertEqual(snapshot["steer_rate_limit"], 0.04)
        self.assertTrue(snapshot["steer_rate_limited"])
        self.assertEqual(snapshot["current_curvature"], 0.012)
        self.assertEqual(snapshot["preview_curvature"], 0.031)
        self.assertEqual(snapshot["lqr_gain_update_reason"], "speed_delta")
        self.assertEqual(snapshot["mpc_solve_mode"], "fresh")
        self.assertAlmostEqual(snapshot["longitudinal_accel"], 10.0)

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

    def test_apply_controller_step_uses_reference_curvature_matching_tracking_errors(self):
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
            target_index=3,
            reference_waypoint=object(),
            reference_index=2,
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

    def test_apply_controller_step_passes_curvature_sequence_to_preview_capable_controller(self):
        class RecordingController:
            supports_curvature_sequence = True

            def __init__(self):
                self.horizon = 4
                self.dt = 0.05
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
        vehicle = SimpleNamespace(get_velocity=lambda: SimpleNamespace(x=40.0, y=0.0, z=0.0))
        route_trace = [
            (SimpleNamespace(transform=SimpleNamespace(location=SimpleNamespace(x=float(index), y=0.0))), None)
            for index in range(12)
        ]

        apply_controller_step(
            controller,
            vehicle=vehicle,
            target_waypoint=object(),
            route_trace=route_trace,
            target_index=3,
            reference_waypoint=object(),
            reference_index=3,
        )

        self.assertIsInstance(controller.curvature, list)
        self.assertEqual(len(controller.curvature), 4)


if __name__ == "__main__":
    unittest.main()
