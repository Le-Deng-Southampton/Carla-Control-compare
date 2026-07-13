import importlib
import os
import sys
import types
import unittest
from unittest import mock


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYTHONAPI_ROOT = os.path.dirname(PROJECT_ROOT)
CARLA_AGENTS_ROOT = os.path.join(PYTHONAPI_ROOT, "carla")
for path in (PROJECT_ROOT, CARLA_AGENTS_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)


def install_runtime_stubs():
    if "carla" not in sys.modules:
        carla = types.ModuleType("carla")

        class VehicleControl:
            pass

        carla.VehicleControl = VehicleControl
        sys.modules["carla"] = carla

    if "pygame" not in sys.modules:
        sys.modules["pygame"] = types.ModuleType("pygame")

    error_providers = types.ModuleType("error_providers")
    error_providers.get_supported_error_provider_names = lambda: ("ground_truth", "noisy_ground_truth", "perception_proxy")
    sys.modules["error_providers"] = error_providers

    experiment = types.ModuleType("experiment")
    experiment.build_summary = None
    experiment.configure_world = None
    experiment.create_display = None
    experiment.get_vehicle_blueprint = None
    experiment.resolve_route_setup = None
    experiment.run_controller_lap = None
    experiment.save_compare_outputs = None
    sys.modules["experiment"] = experiment

    route_planner = types.ModuleType("road_planning.route_planner")
    route_planner.ROUTE_SHAPES = ("true_straight", "straight", "gentle_curve", "curvy", "s_curve")
    route_planner.speed_adaptive_route_defaults = lambda target_speed_kmh: {
        "route_shape": "straight" if target_speed_kmh >= 120.0 else "s_curve",
        "min_length_m": 2400.0 if target_speed_kmh >= 120.0 else 1700.0,
        "speed_band": "very_high" if target_speed_kmh >= 120.0 else "medium",
        "target_duration_s": 70.0 if target_speed_kmh >= 120.0 else 80.0,
    }
    route_planner.estimate_route_max_waypoints = lambda min_length_m, sampling_resolution, length_tolerance: int(min_length_m) + 300
    sys.modules["road_planning.route_planner"] = route_planner


class RunArgsTest(unittest.TestCase):
    def setUp(self):
        install_runtime_stubs()
        sys.modules.pop("run_my_control", None)

    def test_planner_and_tracker_cli_defaults_enable_safe_frenet_fallback(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.planner_mode, "frenet")
        self.assertEqual(args.planner_fallback, "legacy")
        self.assertEqual(args.planner_validation_spacing, 0.25)
        self.assertEqual(args.planner_vehicle_half_width, 1.05)
        self.assertEqual(args.planner_lane_margin, 0.35)
        self.assertEqual(args.planner_topology_beam_width, 24)
        self.assertEqual(args.planner_lateral_beam_width, 64)
        self.assertEqual(args.planner_candidate_cap, 512)
        self.assertEqual(args.planner_deadline, 5.0)
        self.assertEqual(args.planner_max_curvature, 0.20)
        self.assertEqual(args.planner_max_curvature_rate, 0.020)
        self.assertEqual(args.planner_max_lateral_accel, 6.5)
        self.assertEqual(args.planner_max_lateral_jerk, 10.0)
        self.assertEqual(args.tracker_max_rollback, 2.0)
        self.assertEqual(args.tracker_hold_steps, 3)

    def test_frenet_strict_policy_can_disable_fallback(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(
            sys,
            "argv",
            ["run_my_control.py", "--planner-mode", "frenet", "--planner-fallback", "error"],
        ):
            args = run_my_control.parse_args()

        self.assertEqual(args.planner_fallback, "error")

    def test_frenet_planner_can_be_selected_without_changing_strict_speed_limits(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(
            sys,
            "argv",
            [
                "run_my_control.py",
                "--planner-mode",
                "frenet",
                "--speed-planner-limit-profile",
                "global",
            ],
        ):
            args = run_my_control.parse_args()

        self.assertEqual(args.planner_mode, "frenet")
        self.assertEqual(args.speed_planner_limit_profile, "global")

    def test_run_config_records_frozen_planner_hash_version_config_and_duration(self):
        run_my_control = importlib.import_module("run_my_control")
        with mock.patch.object(sys, "argv", ["run_my_control.py", "--planner-mode", "frenet"]):
            args = run_my_control.parse_args()
        args.seed = 123
        args.selected_map_name = "Town05"
        trajectory = types.SimpleNamespace(
            metadata={"planner_mode": "frenet", "planner_version": 1},
            content_hash=lambda: "frozen-hash",
        )
        waypoint = types.SimpleNamespace()
        route_trace = [(waypoint, "LANEFOLLOW"), (waypoint, "LANEFOLLOW")]
        route_features = {
            "length": 20.0,
            "sign_changes": 0,
            "total_abs_turn": 0.0,
            "turn_segments": 0,
            "planning_duration_s": 0.125,
            "planner_mode_requested": "frenet",
            "planner_mode_resolved": "legacy",
            "planner_fallback_policy": "legacy",
            "planner_fallback_used": True,
            "planner_fallback_code": "no_feasible_candidate",
            "planner_fallback_message": "No feasible Frenet trajectory.",
            "planner_fallback_rejection_counts": {"curvature_rate": 21},
        }
        destination = types.SimpleNamespace(x=20.0, y=0.0, z=0.0)

        config = run_my_control.build_run_config(
            args,
            ("pid", "lqr", "mpc"),
            0,
            1,
            destination,
            route_trace,
            route_features,
            trajectory,
        )

        self.assertEqual(config["planner"]["mode"], "legacy")
        self.assertEqual(config["planner"]["requested_mode"], "frenet")
        self.assertEqual(config["planner"]["resolved_mode"], "legacy")
        self.assertEqual(config["planner"]["fallback_policy"], "legacy")
        self.assertTrue(config["planner"]["fallback_used"])
        self.assertEqual(config["planner"]["fallback_code"], "no_feasible_candidate")
        self.assertEqual(config["planner"]["fallback_rejection_counts"], {"curvature_rate": 21})
        self.assertEqual(config["planner"]["version"], 1)
        self.assertEqual(config["planner"]["trajectory_hash"], "frozen-hash")
        self.assertEqual(config["planner"]["planning_duration_s"], 0.125)
        self.assertEqual(config["planner"]["config"]["candidate_cap"], 512)
        self.assertEqual(config["tracker"]["hold_steps"], 3)

    def test_frozen_hash_guard_rejects_changed_trajectory_before_lap(self):
        run_my_control = importlib.import_module("run_my_control")
        hashes = iter(("frozen-hash", "changed-hash"))
        trajectory = types.SimpleNamespace(content_hash=lambda: next(hashes))
        frozen_hash = trajectory.content_hash()

        with self.assertRaisesRegex(RuntimeError, "frozen reference trajectory hash mismatch"):
            run_my_control.assert_frozen_trajectory(trajectory, frozen_hash)

    def test_lqr_cli_defaults_match_high_speed_tuned_controller_params(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.lqr_q_ey, 3.4)
        self.assertEqual(args.lqr_q_ey_dot, 1.30)
        self.assertEqual(args.lqr_q_epsi, 7.2)
        self.assertEqual(args.lqr_q_epsi_dot, 4.2)
        self.assertEqual(args.lqr_r, 7.4)
        self.assertEqual(args.lqr_max_steer, 0.60)
        self.assertEqual(args.lqr_max_steer_rate, 0.22)
        self.assertEqual(args.lqr_curvature_alpha, 0.60)
        self.assertEqual(args.lqr_feedforward_gain, 1.0)
        self.assertEqual(args.lqr_curvature_preview_horizon, 12)
        self.assertEqual(args.lqr_curvature_preview_blend, 0.40)

    def test_mpc_cli_defaults_use_longer_high_speed_preview(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.mpc_horizon, 20)
        self.assertEqual(args.mpc_q_y, 14.0)
        self.assertEqual(args.mpc_q_psi, 22.0)
        self.assertEqual(args.mpc_r_steer, 1.0)
        self.assertEqual(args.mpc_r_steer_rate, 0.9)
        self.assertEqual(args.mpc_max_steer, 0.65)
        self.assertEqual(args.mpc_max_steer_rate, 0.08)
        self.assertEqual(args.mpc_curvature_step_limit, 0.02)
        self.assertEqual(args.mpc_max_lateral_accel, 6.0)
        self.assertEqual(args.mpc_min_dynamic_steer_limit, 0.24)
        self.assertEqual(args.mpc_cornering_stiffness_scale, 1.0)
        self.assertEqual(args.mpc_min_horizon, 10)
        self.assertFalse(args.mpc_adaptive_horizon_enabled)
        self.assertEqual(args.mpc_model_type, "kinematic")
        self.assertEqual(args.mpc_derivative_alpha, 0.25)
        self.assertEqual(args.mpc_event_trigger_curvature, 0.008)
        self.assertEqual(args.mpc_curvature_filter_alpha, 0.20)
        self.assertEqual(args.mpc_curvature_feedforward_gain, 1.0)
        self.assertFalse(args.mpc_small_error_steer_deadband_enabled)
        self.assertEqual(args.mpc_small_error_lateral_threshold, 0.08)
        self.assertEqual(args.mpc_small_error_heading_threshold, 0.8)
        self.assertEqual(args.mpc_small_error_steer_hold_delta, 0.006)
        self.assertEqual(args.mpc_small_error_current_curvature_threshold, 0.010)
        self.assertFalse(args.debug_mpc_stability)

    def test_mpc_adaptive_horizon_can_be_enabled(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py", "--mpc-enable-adaptive-horizon"]):
            args = run_my_control.parse_args()

        self.assertTrue(args.mpc_adaptive_horizon_enabled)

    def test_speed_planner_cli_defaults_enable_curvature_based_dynamic_speed(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.speed_planner_min_turn_speed, 48.0)
        self.assertEqual(args.speed_planner_recovery_min_speed, 40.0)
        self.assertEqual(args.speed_planner_max_lateral_accel, 18.0)
        self.assertEqual(args.speed_planner_max_accel, 3.5)
        self.assertEqual(args.speed_planner_max_decel, 9.0)
        self.assertEqual(args.speed_planner_lateral_error_warning, 1.6)
        self.assertEqual(args.speed_planner_lateral_error_critical, 2.6)
        self.assertEqual(args.speed_planner_heading_error_warning, 12.0)
        self.assertEqual(args.speed_planner_heading_error_critical, 18.0)
        self.assertEqual(args.speed_planner_lateral_error_rate_warning, 0.8)
        self.assertEqual(args.speed_planner_lateral_error_rate_critical, 1.6)
        self.assertEqual(args.speed_planner_heading_error_rate_warning, 8.0)
        self.assertEqual(args.speed_planner_heading_error_rate_critical, 16.0)
        self.assertEqual(args.speed_planner_lateral_error_rate_activation, 0.8)
        self.assertEqual(args.speed_planner_heading_error_rate_activation, 8.0)
        self.assertEqual(args.speed_planner_error_rate_alpha, 0.25)
        self.assertEqual(args.speed_planner_recovery_hold_steps, 3)
        self.assertEqual(args.speed_planner_entry_max_speed, 70.0)
        self.assertEqual(args.speed_planner_entry_curvature_threshold, 0.015)
        self.assertEqual(args.speed_planner_entry_full_cap_curvature, 0.03)
        self.assertEqual(args.speed_planner_emergency_turn_speed, 36.0)
        self.assertEqual(args.speed_planner_emergency_curvature_threshold, 0.08)
        self.assertEqual(args.speed_planner_emergency_full_cap_curvature, 0.12)
        self.assertEqual(args.speed_planner_mode, "adaptive")
        self.assertEqual(args.speed_planner_limit_profile, "controller")

    def test_controller_only_speed_planner_mode_can_be_selected(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py", "--speed-planner-mode", "off"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.speed_planner_mode, "off")

    def test_global_speed_planner_limit_profile_can_be_selected(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py", "--speed-planner-limit-profile", "global"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.speed_planner_limit_profile, "global")

    def test_perception_proxy_cli_defaults_are_disabled(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.perception_delay_steps, 0)
        self.assertEqual(args.perception_dropout_probability, 0.0)
        self.assertEqual(args.perception_smoothing_alpha, 1.0)

    def test_carla_connection_cli_defaults_can_be_overridden(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(
            sys,
            "argv",
            ["run_my_control.py", "--carla-host", "127.0.0.1", "--carla-port", "2100", "--carla-timeout", "5"],
        ):
            args = run_my_control.parse_args()

        self.assertEqual(args.carla_host, "127.0.0.1")
        self.assertEqual(args.carla_port, 2100)
        self.assertEqual(args.carla_timeout, 5.0)

    def test_pid_longitudinal_defaults_are_tuned_for_70_kmh_tracking(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.pid_long_kp, 0.45)
        self.assertEqual(args.pid_long_ki, 0.01)
        self.assertEqual(args.pid_long_kd, 0.10)

    def test_route_shape_and_length_default_to_speed_adaptive_auto(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py", "--target-speed", "130"]):
            args = run_my_control.parse_args()
        run_my_control.apply_speed_adaptive_route_settings(args)

        self.assertEqual(args.route_shape, "straight")
        self.assertEqual(args.route_min_length_m, 2400.0)
        self.assertEqual(args.route_max_waypoints, 2700)
        self.assertEqual(args.route_shape_source, "speed_adaptive")
        self.assertEqual(args.route_length_source, "speed_adaptive")

    def test_manual_route_shape_and_length_override_speed_adaptive_defaults(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(
            sys,
            "argv",
            [
                "run_my_control.py",
                "--target-speed",
                "130",
                "--route-shape",
                "s_curve",
                "--route-min-length-m",
                "1500",
                "--route-max-waypoints",
                "1800",
            ],
        ):
            args = run_my_control.parse_args()
        run_my_control.apply_speed_adaptive_route_settings(args)

        self.assertEqual(args.route_shape, "s_curve")
        self.assertEqual(args.route_min_length_m, 1500.0)
        self.assertEqual(args.route_max_waypoints, 1800)
        self.assertEqual(args.route_shape_source, "manual")
        self.assertEqual(args.route_length_source, "manual")

    def test_map_name_defaults_to_auto_and_accepts_supported_town(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()
        self.assertEqual(args.map_name, "auto")

        with mock.patch.object(sys, "argv", ["run_my_control.py", "--map-name", "Town10HD_Opt"]):
            args = run_my_control.parse_args()
        self.assertEqual(args.map_name, "Town10HD_Opt")

    def test_auto_map_choice_uses_route_preferences(self):
        run_my_control = importlib.import_module("run_my_control")

        selected = run_my_control.choose_map_name("auto", "curvy", 45.0)

        self.assertIn(selected, ("Town05", "Town05_Opt", "Town03", "Town03_Opt", "Town10HD", "Town10HD_Opt"))

    def test_auto_map_choice_prefers_high_speed_capable_map(self):
        run_my_control = importlib.import_module("run_my_control")

        selected = run_my_control.choose_map_name("auto", "straight", 130.0)

        self.assertIn(selected, ("Town04", "Town04_Opt", "Town05", "Town05_Opt", "Town10HD", "Town10HD_Opt"))

    def test_map_adaptive_route_settings_clamp_auto_length_to_map_capacity(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py", "--target-speed", "130"]):
            args = run_my_control.parse_args()
        run_my_control.apply_speed_adaptive_route_settings(args)
        args.selected_map_name = "Town01"

        run_my_control.apply_map_adaptive_route_settings(args)

        self.assertEqual(args.route_min_length_m, 1800.0)
        self.assertEqual(args.route_length_source, "speed_map_adaptive")
        self.assertEqual(args.route_max_waypoints, 2100)

    def test_map_adaptive_route_settings_preserve_manual_length(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py", "--route-min-length-m", "1500"]):
            args = run_my_control.parse_args()
        run_my_control.apply_speed_adaptive_route_settings(args)
        args.selected_map_name = "Town05"

        run_my_control.apply_map_adaptive_route_settings(args)

        self.assertEqual(args.route_min_length_m, 1500.0)
        self.assertEqual(args.route_length_source, "manual")

    def test_current_map_keeps_loaded_world_without_reloading(self):
        run_my_control = importlib.import_module("run_my_control")

        args = types.SimpleNamespace(map_name="current", route_shape="s_curve", target_speed=70.0)
        world = types.SimpleNamespace(get_map=lambda: types.SimpleNamespace(name="Carla/Maps/Town03"))
        client = types.SimpleNamespace(get_world=mock.Mock(return_value=world), load_world=mock.Mock())

        selected_world = run_my_control.load_selected_world(client, args)

        self.assertIs(selected_world, world)
        self.assertEqual(args.map_name, "Town03")
        client.load_world.assert_not_called()

    def test_manual_map_loads_requested_world(self):
        run_my_control = importlib.import_module("run_my_control")

        args = types.SimpleNamespace(map_name="Town10HD", route_shape="straight", target_speed=120.0)
        old_world = types.SimpleNamespace(get_map=lambda: types.SimpleNamespace(name="Carla/Maps/Town03"))
        new_world = types.SimpleNamespace(get_map=lambda: types.SimpleNamespace(name="Carla/Maps/Town10HD"))
        client = types.SimpleNamespace(get_world=mock.Mock(return_value=old_world), load_world=mock.Mock(return_value=new_world))

        selected_world = run_my_control.load_selected_world(client, args)

        self.assertIs(selected_world, new_world)
        self.assertEqual(args.map_name, "Town10HD")
        client.load_world.assert_called_once_with("Town10HD")


if __name__ == "__main__":
    unittest.main()
