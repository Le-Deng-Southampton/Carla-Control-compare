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

from control.pid_controller import PidControllerAdapter


class RecordingLateralController:
    def __init__(self, steer=0.0):
        self.steer = steer
        self.parameters = []
        self.targets = []
        self._e_buffer = []

    def change_parameters(self, k_p, k_i, k_d, dt):
        self.parameters.append((k_p, k_i, k_d, dt))

    def run_step(self, target_waypoint):
        self.targets.append(target_waypoint)
        return self.steer


class RecordingLongitudinalController:
    def __init__(self):
        self.targets = []

    def reset(self):
        pass

    def run_step(self, speed_mps, target_speed_mps=None):
        self.targets.append(target_speed_mps)
        return 0.0, 0.0


class StaticVehicle:
    def __init__(self, speed=10.0):
        self.speed = speed
        self._control = SimpleNamespace(steer=0.0)

    def get_control(self):
        return self._control

    def get_velocity(self):
        return SimpleNamespace(x=self.speed, y=0.0, z=0.0)

    def get_angular_velocity(self):
        return SimpleNamespace(z=0.0)


class VehicleWithResidualSteer(StaticVehicle):
    def __init__(self, speed=10.0, steer=-0.65):
        super().__init__(speed=speed)
        self._control = SimpleNamespace(steer=steer)


def make_waypoint():
    return SimpleNamespace()


def make_controller(vehicle, lateral_controller, **overrides):
    controller = PidControllerAdapter(
        vehicle=vehicle,
        longitudinal_controller=RecordingLongitudinalController(),
        **overrides,
    )
    controller._lat_controller = lateral_controller
    return controller


class PidControllerAdapterTest(unittest.TestCase):
    def _assert_base_diagnostics(self, controller, **context):
        with self.subTest(expectation="base_speed_profile", **context):
            self.assertEqual(controller.last_speed_profile, "pid_base")
        with self.subTest(expectation="bounded_feedforward_scale", **context):
            self.assertGreaterEqual(controller.last_curvature_feedforward_scale, 1.0)
            self.assertLessEqual(controller.last_curvature_feedforward_scale, 2.5)

    def test_fixed_pid_does_not_retune_lateral_gains(self):
        for speed_kmh in (30.0, 45.0, 105.0):
            for curvature in (0.0, 0.08):
                vehicle = StaticVehicle(speed=speed_kmh / 3.6)
                lateral = RecordingLateralController()
                controller = make_controller(
                    vehicle,
                    lateral,
                    lat_kp=1.0,
                    lat_ki=0.20,
                    lat_kd=0.50,
                    max_steer_rate=1.0,
                )

                controller.run_step(vehicle, make_waypoint(), curvature=curvature)

                with self.subTest(
                    expectation="no_gain_retuning",
                    speed_kmh=speed_kmh,
                    curvature=curvature,
                ):
                    self.assertEqual(lateral.parameters, [])
                self._assert_base_diagnostics(
                    controller,
                    speed_kmh=speed_kmh,
                    curvature=curvature,
                )

    def test_steer_rate_limit_is_fixed_across_speed_and_curvature(self):
        for speed_kmh in (30.0, 50.0, 70.0, 90.0, 120.0):
            for curvature in (0.0, 0.08):
                vehicle = StaticVehicle(speed=speed_kmh / 3.6)
                controller = make_controller(
                    vehicle,
                    RecordingLateralController(steer=1.0),
                    max_steer_rate=0.10,
                    curvature_feedforward_gain=0.0,
                )

                control = controller.run_step(
                    vehicle,
                    make_waypoint(),
                    curvature=curvature,
                )

                with self.subTest(speed_kmh=speed_kmh, curvature=curvature):
                    self.assertAlmostEqual(controller.last_steer_rate_limit, 0.10)
                    self.assertAlmostEqual(control.steer, 0.10)

    def test_negative_steer_rate_limit_is_treated_as_zero(self):
        vehicle = StaticVehicle(speed=30.0 / 3.6)
        controller = make_controller(
            vehicle,
            RecordingLateralController(steer=1.0),
            max_steer_rate=-0.25,
            curvature_feedforward_gain=0.0,
        )

        control = controller.run_step(vehicle, make_waypoint())

        self.assertEqual(controller.last_steer_rate_limit, 0.0)
        self.assertEqual(control.steer, 0.0)

    def test_fixed_preview_produces_equal_steering_at_low_and_high_speed(self):
        curvature = [0.0, 0.0, 0.08]
        controls = {}
        for speed_kmh in (30.0, 105.0):
            vehicle = StaticVehicle(speed=speed_kmh / 3.6)
            controller = make_controller(
                vehicle,
                RecordingLateralController(steer=0.0),
                max_steer_rate=1.0,
                curvature_feedforward_gain=1.0,
                curvature_preview_blend=0.30,
                speed_scheduling_enabled=False,
            )

            controls[speed_kmh] = controller.run_step(
                vehicle,
                make_waypoint(),
                curvature=curvature,
            )
            self._assert_base_diagnostics(controller, speed_kmh=speed_kmh)

        with self.subTest(expectation="speed_independent_preview"):
            self.assertEqual(controls[30.0].steer, controls[105.0].steer)

    def test_curvature_feedforward_adds_bicycle_reference_when_centered(self):
        vehicle = StaticVehicle(speed=50.0 / 3.6)
        controller = make_controller(
            vehicle,
            RecordingLateralController(steer=0.0),
            max_steer_rate=1.0,
            curvature_feedforward_gain=1.0,
            curvature_preview_blend=0.0,
            speed_scheduling_enabled=False,
        )

        control = controller.run_step(
            vehicle,
            make_waypoint(),
            curvature=0.04,
            tracking_errors={"e_y": 0.0, "e_psi": 0.0},
        )

        self._assert_base_diagnostics(controller)
        self.assertAlmostEqual(control.steer, np.arctan(controller.L * 0.04), places=6)

    def test_pid_initialization_ignores_residual_vehicle_steer(self):
        vehicle = VehicleWithResidualSteer(speed=30.0 / 3.6, steer=-0.65)
        controller = make_controller(
            vehicle,
            RecordingLateralController(steer=0.65),
            max_steer_rate=0.10,
            curvature_feedforward_gain=0.0,
        )

        control = controller.run_step(vehicle, make_waypoint(), curvature=0.0)

        self._assert_base_diagnostics(controller)
        self.assertLessEqual(abs(control.steer), 0.10)

    def test_tracking_errors_do_not_replace_carla_waypoint_pid_feedback(self):
        vehicle = StaticVehicle(speed=90.0 / 3.6)
        waypoint = make_waypoint()
        lateral = RecordingLateralController(steer=0.24)
        controller = make_controller(
            vehicle,
            lateral,
            max_steer_rate=1.0,
            curvature_feedforward_gain=0.0,
            speed_scheduling_enabled=False,
        )

        control = controller.run_step(
            vehicle,
            waypoint,
            tracking_errors={"e_y": 4.0, "e_psi": np.radians(25.0)},
        )

        self.assertEqual(lateral.targets, [waypoint])
        self.assertAlmostEqual(control.steer, 0.24)

if __name__ == "__main__":
    unittest.main()
