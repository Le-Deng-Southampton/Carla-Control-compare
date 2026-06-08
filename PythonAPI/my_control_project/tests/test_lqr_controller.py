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

    class VehicleControl:
        pass

    carla.VehicleControl = VehicleControl
    sys.modules["carla"] = carla

from control.lqr_controller import LqrController


class StaticVehicle:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, speed=10.0, velocity_y=0.0, yaw_rate=0.0):
        self._transform = SimpleNamespace(
            location=SimpleNamespace(x=x, y=y),
            rotation=SimpleNamespace(yaw=yaw),
        )
        self._velocity = SimpleNamespace(x=speed, y=velocity_y, z=0.0)
        self._angular_velocity = SimpleNamespace(x=0.0, y=0.0, z=yaw_rate)

    def get_transform(self):
        return self._transform

    def get_velocity(self):
        return self._velocity

    def get_angular_velocity(self):
        return self._angular_velocity


def make_waypoint(x=0.0, y=0.0, yaw=0.0):
    return SimpleNamespace(
        transform=SimpleNamespace(
            location=SimpleNamespace(x=x, y=y),
            rotation=SimpleNamespace(yaw=yaw),
        )
    )


class LqrControllerTest(unittest.TestCase):
    def test_default_params_are_tuned_for_high_speed_without_aggressive_saturation(self):
        controller = LqrController()

        controller._update_lqr_gain(50.0 / 3.6)

        np.testing.assert_allclose(controller.Q.diagonal(), [2.6, 1.10, 6.0, 2.6])
        self.assertEqual(controller.R.item(), 8.0)
        self.assertEqual(controller.max_steer, 0.55)
        self.assertEqual(controller.max_steer_rate, 0.18)
        self.assertEqual(controller.curvature_alpha, 0.55)
        self.assertEqual(controller.feedforward_gain, 1.12)
        self.assertLess(controller._gain_matrix[0, 0], 0.8)
        self.assertLess(controller._gain_matrix[0, 2], 4.4)

    def test_default_curve_feedforward_uses_tuned_bicycle_reference(self):
        controller = LqrController(
            q_weights=(0.0, 0.0, 0.0, 0.0),
            r_weight=1.0,
            max_steer=0.65,
            max_steer_rate=0.65,
        )
        vehicle = StaticVehicle()
        waypoint = make_waypoint()
        curvature = 0.025

        for _ in range(30):
            control = controller.run_step(vehicle, waypoint, curvature=curvature)

        self.assertAlmostEqual(control.steer, controller.feedforward_gain * np.arctan(controller.L * curvature), places=3)

    def test_first_sample_does_not_turn_lateral_offset_into_derivative_spike(self):
        controller = LqrController(
            q_weights=(0.01, 10.0, 0.01, 0.01),
            r_weight=1.0,
            max_steer=0.65,
            max_steer_rate=0.65,
        )
        vehicle = StaticVehicle(y=1.0)
        waypoint = make_waypoint()

        control = controller.run_step(vehicle, waypoint, curvature=0.0)

        self.assertLess(abs(control.steer), 0.1)

    def test_lateral_error_rate_comes_from_vehicle_motion_not_reference_jump(self):
        controller = LqrController(
            q_weights=(0.01, 10.0, 0.01, 0.01),
            r_weight=1.0,
            max_steer=0.65,
            max_steer_rate=0.65,
        )
        waypoint = make_waypoint()
        controller.run_step(StaticVehicle(y=0.0, velocity_y=0.0), waypoint, curvature=0.0)

        control = controller.run_step(StaticVehicle(y=1.0, velocity_y=-3.0), waypoint, curvature=0.0)

        self.assertGreater(control.steer, 0.1)

    def test_uses_provided_tracking_errors_when_available(self):
        controller = LqrController(
            q_weights=(10.0, 0.0, 0.0, 0.0),
            r_weight=1.0,
            max_steer=0.65,
            max_steer_rate=0.65,
        )
        vehicle = StaticVehicle(y=0.0)
        waypoint = make_waypoint()

        control = controller.run_step(
            vehicle,
            waypoint,
            curvature=0.0,
            tracking_errors={
                "e_y": 1.0,
                "e_psi": 0.0,
                "reference_yaw": 0.0,
            },
        )

        self.assertLess(control.steer, -0.1)

    def test_zero_error_constant_curvature_feedforward_converges_to_bicycle_reference(self):
        controller = LqrController(
            q_weights=(0.0, 0.0, 0.0, 0.0),
            r_weight=1.0,
            max_steer=0.65,
            max_steer_rate=0.65,
            feedforward_gain=1.0,
        )
        vehicle = StaticVehicle()
        waypoint = make_waypoint()
        curvature = 0.04

        for _ in range(30):
            control = controller.run_step(vehicle, waypoint, curvature=curvature)

        self.assertAlmostEqual(control.steer, np.arctan(controller.L * curvature), places=3)


if __name__ == "__main__":
    unittest.main()
