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
    sys.modules["road_planning.route_planner"] = route_planner


class RunArgsTest(unittest.TestCase):
    def setUp(self):
        install_runtime_stubs()
        sys.modules.pop("run_my_control", None)

    def test_lqr_cli_defaults_match_high_speed_tuned_controller_params(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.lqr_q_ey, 2.6)
        self.assertEqual(args.lqr_q_ey_dot, 1.10)
        self.assertEqual(args.lqr_q_epsi, 6.0)
        self.assertEqual(args.lqr_q_epsi_dot, 2.6)
        self.assertEqual(args.lqr_r, 8.0)
        self.assertEqual(args.lqr_max_steer, 0.55)
        self.assertEqual(args.lqr_max_steer_rate, 0.16)
        self.assertEqual(args.lqr_curvature_alpha, 0.50)
        self.assertEqual(args.lqr_feedforward_gain, 1.0)
        self.assertEqual(args.lqr_turn_in_rate_scale, 0.70)
        self.assertEqual(args.lqr_turn_in_guard_lateral_error, 1.0)
        self.assertEqual(args.lqr_turn_in_guard_heading_error, 10.0)
        self.assertEqual(args.lqr_turn_in_guard_max_curvature, 0.04)
        self.assertEqual(args.lqr_inside_error_feedforward_start, 0.80)
        self.assertEqual(args.lqr_inside_error_feedforward_full, 1.80)
        self.assertEqual(args.lqr_inside_error_feedforward_min_scale, 0.65)
        self.assertEqual(args.lqr_inside_error_feedforward_heading_limit, 4.0)

    def test_mpc_cli_defaults_use_longer_high_speed_preview(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.mpc_horizon, 16)
        self.assertEqual(args.mpc_q_y, 12.0)
        self.assertEqual(args.mpc_q_psi, 18.0)
        self.assertEqual(args.mpc_r_steer, 0.8)
        self.assertEqual(args.mpc_r_steer_rate, 0.9)
        self.assertEqual(args.mpc_max_steer, 0.65)
        self.assertEqual(args.mpc_max_steer_rate, 0.30)

    def test_speed_planner_cli_defaults_enable_curvature_based_dynamic_speed(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.speed_planner_min_turn_speed, 48.0)
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
        self.assertEqual(args.speed_planner_mode, "adaptive")

    def test_controller_only_speed_planner_mode_can_be_selected(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py", "--speed-planner-mode", "off"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.speed_planner_mode, "off")

    def test_perception_proxy_cli_defaults_are_disabled(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.perception_delay_steps, 0)
        self.assertEqual(args.perception_dropout_probability, 0.0)
        self.assertEqual(args.perception_smoothing_alpha, 1.0)

    def test_pid_longitudinal_defaults_are_tuned_for_70_kmh_tracking(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.pid_long_kp, 0.45)
        self.assertEqual(args.pid_long_ki, 0.01)
        self.assertEqual(args.pid_long_kd, 0.10)


if __name__ == "__main__":
    unittest.main()
