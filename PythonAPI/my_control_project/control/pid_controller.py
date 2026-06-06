import carla
import numpy as np
from agents.navigation.controller import PIDLateralController

from .base import BaseTrackingController
from .longitudinal import PidLongitudinalController


class PidControllerAdapter(BaseTrackingController):
    """Project-level adapter around CARLA's built-in PID controller."""

    def __init__(
        self,
        vehicle,
        target_speed=30.0,
        lat_kp=0.72,
        lat_ki=0.005,
        lat_kd=0.38,
        long_kp=0.22,
        long_ki=0.01,
        long_kd=0.14,
        dt=0.05,
        max_throttle=0.45,
        max_brake=0.28,
        max_steering=0.65,
        longitudinal_controller=None,
    ):
        self.max_steer = max_steering
        self._lat_controller = PIDLateralController(
            vehicle,
            K_P=lat_kp,
            K_D=lat_kd,
            K_I=lat_ki,
            dt=dt,
        )
        self._longitudinal_controller = longitudinal_controller or PidLongitudinalController(
            target_speed_kmh=target_speed,
            kp=long_kp,
            ki=long_ki,
            kd=long_kd,
            dt=dt,
            max_throttle=max_throttle,
            max_brake=max_brake,
        )
        self.past_steering = vehicle.get_control().steer

    def run_step(
        self,
        vehicle,
        target_waypoint,
        curvature=0.0,
        reference_waypoint=None,
        planned_target_speed_mps=None,
        tracking_errors=None,
    ):
        velocity = vehicle.get_velocity()
        speed_mps = np.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        throttle, brake = self._longitudinal_controller.run_step(
            speed_mps,
            target_speed_mps=planned_target_speed_mps,
        )
        current_steering = self._lat_controller.run_step(target_waypoint)

        if current_steering > self.past_steering + 0.1:
            current_steering = self.past_steering + 0.1
        elif current_steering < self.past_steering - 0.1:
            current_steering = self.past_steering - 0.1

        if current_steering >= 0:
            steering = min(self.max_steer, current_steering)
        else:
            steering = max(-self.max_steer, current_steering)

        control = carla.VehicleControl()
        control.steer = steering
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False
        control.manual_gear_shift = False
        self.past_steering = steering
        return control

    def reset(self):
        self.past_steering = 0.0
        self._longitudinal_controller.reset()
        if hasattr(self._lat_controller, "_e_buffer"):
            self._lat_controller._e_buffer.clear()
