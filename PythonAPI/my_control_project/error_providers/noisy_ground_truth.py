import numpy as np

from .base import BaseErrorProvider
from .ground_truth import GroundTruthErrorProvider


class NoisyGroundTruthErrorProvider(BaseErrorProvider):
    """Ground-truth tracking errors with controlled Gaussian noise injection."""

    def __init__(self, lateral_std=0.1, heading_std_deg=1.0, seed=None):
        self._base_provider = GroundTruthErrorProvider()
        self._lateral_std = lateral_std
        self._heading_std_rad = np.radians(heading_std_deg)
        self._rng = np.random.default_rng(seed)

    def compute(self, vehicle, target_waypoint, route_trace, route_index, reference_waypoint=None):
        errors = self._base_provider.compute(
            vehicle,
            target_waypoint,
            route_trace,
            route_index,
            reference_waypoint=reference_waypoint,
        )
        return {
            **errors,
            "e_y": errors["e_y"] + self._rng.normal(0.0, self._lateral_std),
            "e_psi": errors["e_psi"] + self._rng.normal(0.0, self._heading_std_rad),
        }
