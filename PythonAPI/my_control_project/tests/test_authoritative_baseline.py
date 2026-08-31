import importlib.util
import os
import sys
import types
import unittest

import numpy as np


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)


class FakeVehicle:
    def __init__(self, max_steer_angle=70.0):
        self.max_steer_angle = max_steer_angle

    def get_physics_control(self):
        wheel = types.SimpleNamespace(max_steer_angle=self.max_steer_angle)
        return types.SimpleNamespace(wheels=[wheel, wheel, wheel, wheel])


class FakeVehicleControl:
    def __init__(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.hand_brake = False
        self.manual_gear_shift = False


class FakeVehiclePidController:
    def __init__(self, vehicle, **kwargs):
        self.vehicle = vehicle
        self.kwargs = kwargs


class FakeLongitudinalController:
    def __init__(self, vehicle, **kwargs):
        self.vehicle = vehicle
        self.kwargs = kwargs


class AuthoritativeBaselineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._saved_modules = {
            name: sys.modules.get(name)
            for name in (
                "carla",
                "agents",
                "agents.navigation",
                "agents.navigation.controller",
            )
        }
        carla = types.ModuleType("carla")
        carla.VehicleControl = FakeVehicleControl
        agents = types.ModuleType("agents")
        navigation = types.ModuleType("agents.navigation")
        official = types.ModuleType("agents.navigation.controller")
        official.VehiclePIDController = FakeVehiclePidController
        official.PIDLongitudinalController = FakeLongitudinalController
        sys.modules["carla"] = carla
        sys.modules["agents"] = agents
        sys.modules["agents.navigation"] = navigation
        sys.modules["agents.navigation.controller"] = official
        module_path = os.path.join(PROJECT_ROOT, "control", "authoritative_baseline.py")
        spec = importlib.util.spec_from_file_location("authoritative_baseline", module_path)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    @classmethod
    def tearDownClass(cls):
        for name, module in cls._saved_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    def test_audit_locks_the_agreed_initial_parameters(self):
        audit = self.module.authoritative_audit()
        self.assertEqual(audit["label"], "authoritative_baseline_v1")
        self.assertFalse(audit["tuned"])
        self.assertEqual(audit["vehicle"], {"mass_kg": 1750.0, "yaw_inertia_kgm2": 2875.0, "lf_m": 1.45, "lr_m": 1.45, "cf_n_per_rad": 19000.0, "cr_n_per_rad": 33000.0})
        self.assertEqual(audit["pid"]["lateral"], {"K_P": 1.95, "K_I": 0.05, "K_D": 0.2, "dt": 0.05})
        self.assertEqual(audit["pid"]["longitudinal"], {"K_P": 1.0, "K_I": 0.05, "K_D": 0.0, "dt": 0.05})
        self.assertEqual(audit["lqr"]["q_diagonal"], [0.075, 0.0, 0.0, 0.0])
        self.assertEqual(audit["lqr"]["r"], 1.0)
        self.assertEqual(audit["mpc"]["prediction_horizon"], 11)
        self.assertEqual(audit["mpc"]["control_horizon"], 3)
        self.assertEqual(audit["mpc"]["steering_rate_weight"], 15.0)
        self.assertEqual(audit["mpc"]["max_iterations"], 10)
        self.assertEqual(audit["mpc"]["assumption_label"], "initial_assumption_v1")

    def test_pid_delegates_to_the_official_carla_controller_with_frozen_arguments(self):
        controller = self.module.AuthoritativePidController(FakeVehicle(), target_speed_kmh=30.0)
        self.assertEqual(controller.official_controller.kwargs["args_lateral"], {"K_P": 1.95, "K_I": 0.05, "K_D": 0.2, "dt": 0.05})
        self.assertEqual(controller.official_controller.kwargs["args_longitudinal"], {"K_P": 1.0, "K_I": 0.05, "K_D": 0.0, "dt": 0.05})
        self.assertEqual(controller.official_controller.kwargs["max_throttle"], 0.75)
        self.assertEqual(controller.official_controller.kwargs["max_brake"], 0.3)
        self.assertEqual(controller.official_controller.kwargs["max_steering"], 0.8)

    def test_dynamic_model_and_lqr_shapes_are_four_state(self):
        a, b = self.module.dynamic_bicycle_model(12.0)
        controller = self.module.AuthoritativeLqrController(FakeVehicle(), target_speed_kmh=30.0)
        self.assertEqual(a.shape, (4, 4))
        self.assertEqual(b.shape, (4, 1))
        self.assertTrue(np.array_equal(np.diag(controller.q), np.array([0.075, 0.0, 0.0, 0.0])))
        self.assertEqual(controller.r.shape, (1, 1))

    def test_mpc_uses_paper_horizons_and_runtime_wheel_limit(self):
        controller = self.module.AuthoritativeMpcController(FakeVehicle(70.0), target_speed_kmh=30.0)
        self.assertEqual(controller.prediction_horizon, 11)
        self.assertEqual(controller.control_horizon, 3)
        self.assertEqual(controller.max_iterations, 10)
        self.assertAlmostEqual(controller.max_steer_angle_rad, np.radians(70.0))
        self.assertAlmostEqual(controller.max_steer_rate_rad_s, 2.0 * np.radians(70.0))
        self.assertEqual(controller.initial_move_sequence.tolist(), [0.0, 0.0, 0.0])

    def test_mpc_cost_contains_only_lateral_error_and_steering_rate(self):
        controller = self.module.AuthoritativeMpcController(FakeVehicle(70.0), target_speed_kmh=30.0)
        a, b = self.module._discrete_model(10.0)
        self.assertEqual(controller._objective(np.zeros(3), np.zeros(4), a, b), 0.0)

    def test_factory_rejects_unknown_controller(self):
        args = types.SimpleNamespace(target_speed=30.0)
        with self.assertRaisesRegex(ValueError, "Unsupported authoritative controller"):
            self.module.create_authoritative_controller("stanley", FakeVehicle(), args)


if __name__ == "__main__":
    unittest.main()
