import os
import sys
import types
import unittest
from types import SimpleNamespace


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

if "carla" not in sys.modules:
    carla = types.ModuleType("carla")

    class VehicleControl:
        pass

    carla.VehicleControl = VehicleControl
    sys.modules["carla"] = carla

agents = types.ModuleType("agents")
navigation = types.ModuleType("agents.navigation")
controller_module = types.ModuleType("agents.navigation.controller")


class StubPIDLateralController:
    def __init__(self, *args, **kwargs):
        self._e_buffer = []

    def run_step(self, target_waypoint):
        return 0.0


controller_module.PIDLateralController = StubPIDLateralController
sys.modules["agents"] = agents
sys.modules["agents.navigation"] = navigation
sys.modules["agents.navigation.controller"] = controller_module

from control.lqr_controller import LqrController
from control.mpc_controller import MpcController
from control.pid_controller import PidControllerAdapter


class RecordingLongitudinalController:
    def __init__(self):
        self.targets = []

    def reset(self):
        pass

    def run_step(self, speed_mps, target_speed_mps=None):
        self.targets.append(target_speed_mps)
        return 0.0, 0.0


class StaticVehicle:
    def __init__(self, speed=8.0):
        self._transform = SimpleNamespace(
            location=SimpleNamespace(x=0.0, y=0.0),
            rotation=SimpleNamespace(yaw=0.0),
        )
        self._velocity = SimpleNamespace(x=speed, y=0.0, z=0.0)
        self._angular_velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self._control = SimpleNamespace(steer=0.0)

    def get_transform(self):
        return self._transform

    def get_velocity(self):
        return self._velocity

    def get_angular_velocity(self):
        return self._angular_velocity

    def get_control(self):
        return self._control


def make_waypoint():
    return SimpleNamespace(
        transform=SimpleNamespace(
            location=SimpleNamespace(x=0.0, y=0.0),
            rotation=SimpleNamespace(yaw=0.0),
        )
    )


class DynamicTargetPropagationTest(unittest.TestCase):
    def test_lqr_passes_planned_target_speed_to_longitudinal_controller(self):
        longitudinal = RecordingLongitudinalController()
        controller = LqrController(longitudinal_controller=longitudinal)

        controller.run_step(StaticVehicle(), make_waypoint(), planned_target_speed_mps=7.0)

        self.assertEqual(longitudinal.targets[-1], 7.0)

    def test_mpc_passes_planned_target_speed_to_longitudinal_controller(self):
        longitudinal = RecordingLongitudinalController()
        controller = MpcController(longitudinal_controller=longitudinal)

        controller.run_step(StaticVehicle(), make_waypoint(), planned_target_speed_mps=7.0)

        self.assertEqual(longitudinal.targets[-1], 7.0)

    def test_pid_passes_planned_target_speed_to_longitudinal_controller(self):
        longitudinal = RecordingLongitudinalController()
        vehicle = StaticVehicle()
        controller = PidControllerAdapter(vehicle=vehicle, longitudinal_controller=longitudinal)
        controller._lat_controller = StubPIDLateralController()

        controller.run_step(vehicle, make_waypoint(), planned_target_speed_mps=7.0)

        self.assertEqual(longitudinal.targets[-1], 7.0)


if __name__ == "__main__":
    unittest.main()
