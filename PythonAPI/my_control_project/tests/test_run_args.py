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
    error_providers.get_supported_error_provider_names = lambda: ("ground_truth", "noisy_ground_truth")
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
    route_planner.ROUTE_SHAPES = ("straight", "gentle_curve", "curvy", "s_curve")
    sys.modules["road_planning.route_planner"] = route_planner


class RunArgsTest(unittest.TestCase):
    def setUp(self):
        install_runtime_stubs()
        sys.modules.pop("run_my_control", None)

    def test_lqr_cli_defaults_match_high_speed_tuned_controller_params(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.lqr_q_ey, 2.8)
        self.assertEqual(args.lqr_q_ey_dot, 1.20)
        self.assertEqual(args.lqr_q_epsi, 4.5)
        self.assertEqual(args.lqr_q_epsi_dot, 3.0)
        self.assertEqual(args.lqr_r, 10.0)
        self.assertEqual(args.lqr_max_steer, 0.55)
        self.assertEqual(args.lqr_max_steer_rate, 0.16)
        self.assertEqual(args.lqr_feedforward_gain, 1.0)

    def test_speed_planner_cli_defaults_enable_curvature_based_dynamic_speed(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.speed_planner_min_turn_speed, 38.0)
        self.assertEqual(args.speed_planner_max_lateral_accel, 16.0)
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
        self.assertEqual(args.speed_planner_recovery_hold_steps, 3)

    def test_pid_longitudinal_defaults_are_tuned_for_70_kmh_tracking(self):
        run_my_control = importlib.import_module("run_my_control")

        with mock.patch.object(sys, "argv", ["run_my_control.py"]):
            args = run_my_control.parse_args()

        self.assertEqual(args.pid_long_kp, 0.45)
        self.assertEqual(args.pid_long_ki, 0.01)
        self.assertEqual(args.pid_long_kd, 0.10)


if __name__ == "__main__":
    unittest.main()
