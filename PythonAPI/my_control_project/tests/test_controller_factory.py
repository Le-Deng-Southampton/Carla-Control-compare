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

    class VehicleControl:
        pass

    carla.VehicleControl = VehicleControl
    sys.modules["carla"] = carla

from control.factory import create_tracking_controller


class FakeControl:
    steer = 0.0


class FakeVehicle:
    def get_control(self):
        return FakeControl()


def build_args(**overrides):
    values = {
        "target_speed": 50.0,
        "lqr_q_ey": 2.6,
        "lqr_q_ey_dot": 1.10,
        "lqr_q_epsi": 6.0,
        "lqr_q_epsi_dot": 2.6,
        "lqr_r": 8.0,
        "lqr_kp_long": 0.6,
        "lqr_ki_long": 0.01,
        "lqr_kd_long": 0.06,
        "lqr_max_steer": 0.55,
        "lqr_max_steer_rate": 0.18,
        "pid_lat_kp": 0.72,
        "pid_lat_ki": 0.005,
        "pid_lat_kd": 0.38,
        "pid_long_kp": 0.45,
        "pid_long_ki": 0.01,
        "pid_long_kd": 0.10,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class ControllerFactoryTest(unittest.TestCase):
    def test_pid_uses_full_throttle_limit_by_default(self):
        controller = create_tracking_controller("pid", FakeVehicle(), build_args())

        self.assertEqual(controller._longitudinal_controller.max_throttle, 1.0)

    def test_pid_throttle_limit_can_be_configured(self):
        controller = create_tracking_controller(
            "pid",
            FakeVehicle(),
            build_args(pid_max_throttle=0.75, pid_max_brake=0.33),
        )

        self.assertEqual(controller._longitudinal_controller.max_throttle, 0.75)
        self.assertEqual(controller._longitudinal_controller.max_brake, 0.33)

    def test_pid_uses_tuned_longitudinal_defaults(self):
        controller = create_tracking_controller("pid", FakeVehicle(), build_args())

        self.assertEqual(controller._longitudinal_controller.kp, 0.45)
        self.assertEqual(controller._longitudinal_controller.ki, 0.01)
        self.assertEqual(controller._longitudinal_controller.kd, 0.10)

    def test_pid_longitudinal_gains_can_be_overridden(self):
        controller = create_tracking_controller(
            "pid",
            FakeVehicle(),
            build_args(pid_long_kp=0.31, pid_long_ki=0.02, pid_long_kd=0.09),
        )

        self.assertEqual(controller._longitudinal_controller.kp, 0.31)
        self.assertEqual(controller._longitudinal_controller.ki, 0.02)
        self.assertEqual(controller._longitudinal_controller.kd, 0.09)


if __name__ == "__main__":
    unittest.main()
