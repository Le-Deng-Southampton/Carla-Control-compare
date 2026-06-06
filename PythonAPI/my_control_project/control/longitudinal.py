from abc import ABC, abstractmethod

import numpy as np


class BaseLongitudinalController(ABC):
    """Common interface for speed controllers that output throttle and brake."""

    @abstractmethod
    def run_step(self, speed_mps):
        """Return a (throttle, brake) tuple for the current vehicle speed."""

    def reset(self):
        """Reset controller state when needed."""


class PidLongitudinalController(BaseLongitudinalController):
    """PID speed controller that maps speed error to CARLA throttle/brake values."""

    def __init__(
        self,
        target_speed_kmh=30.0,
        kp=0.6,
        ki=0.01,
        kd=0.06,
        dt=0.05,
        max_throttle=1.0,
        max_brake=1.0,
        integral_limit=5.0,
    ):
        self.target_speed_mps = target_speed_kmh / 3.6
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.integral_limit = integral_limit
        self.reset()

    def reset(self):
        self._integral_speed_error = 0.0
        self._prev_speed_error = 0.0

    def run_step(self, speed_mps, target_speed_mps=None):
        active_target_speed_mps = self.target_speed_mps if target_speed_mps is None else target_speed_mps
        speed_error = active_target_speed_mps - speed_mps
        self._integral_speed_error = np.clip(
            self._integral_speed_error + speed_error * self.dt,
            -self.integral_limit,
            self.integral_limit,
        )
        speed_error_deriv = (speed_error - self._prev_speed_error) / self.dt
        self._prev_speed_error = speed_error

        throttle_brake = (
            self.kp * speed_error
            + self.ki * self._integral_speed_error
            + self.kd * speed_error_deriv
        )
        if throttle_brake >= 0.0:
            return float(np.clip(throttle_brake, 0.0, self.max_throttle)), 0.0
        return 0.0, float(np.clip(-throttle_brake, 0.0, self.max_brake))
