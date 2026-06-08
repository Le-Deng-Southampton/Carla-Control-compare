import numpy as np

from .base import BaseErrorProvider
from .ground_truth import GroundTruthErrorProvider


class NoisyGroundTruthErrorProvider(BaseErrorProvider):
    """Ground-truth tracking errors with configurable perception-like distortion."""

    def __init__(
        self,
        lateral_std=0.1,
        heading_std_deg=1.0,
        seed=None,
        delay_steps=0,
        dropout_probability=0.0,
        smoothing_alpha=1.0,
    ):
        self._base_provider = GroundTruthErrorProvider()
        self._lateral_std = lateral_std
        self._heading_std_rad = np.radians(heading_std_deg)
        self._rng = np.random.default_rng(seed)
        self._delay_steps = max(0, int(delay_steps))
        self._dropout_probability = float(np.clip(dropout_probability, 0.0, 1.0))
        self._smoothing_alpha = float(np.clip(smoothing_alpha, 0.0, 1.0))
        self._history = []
        self._last_output = None

    def _add_noise(self, errors):
        return {
            **errors,
            "e_y": errors["e_y"] + self._rng.normal(0.0, self._lateral_std),
            "e_psi": errors["e_psi"] + self._rng.normal(0.0, self._heading_std_rad),
        }

    def _delay(self, errors):
        self._history.append(errors)
        if len(self._history) <= self._delay_steps:
            return self._history[0]
        return self._history[-self._delay_steps - 1]

    def _dropout(self, errors):
        if self._last_output is None:
            return errors
        if self._rng.random() < self._dropout_probability:
            return self._last_output
        return errors

    def _smooth(self, errors):
        if self._last_output is None or self._smoothing_alpha >= 1.0:
            return errors

        alpha = self._smoothing_alpha
        smoothed = dict(errors)
        for key in ("e_y", "e_psi", "target_x", "target_y", "target_yaw"):
            smoothed[key] = alpha * errors[key] + (1.0 - alpha) * self._last_output[key]
        return smoothed

    def compute(self, vehicle, target_waypoint, route_trace, route_index, reference_waypoint=None):
        errors = self._base_provider.compute(
            vehicle,
            target_waypoint,
            route_trace,
            route_index,
            reference_waypoint=reference_waypoint,
        )
        errors = self._add_noise(errors)
        errors = self._delay(errors)
        errors = self._dropout(errors)
        errors = self._smooth(errors)
        self._last_output = errors
        return errors
