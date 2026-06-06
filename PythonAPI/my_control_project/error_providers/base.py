from abc import ABC, abstractmethod


class BaseErrorProvider(ABC):
    """Common interface for tracking-error providers."""

    @abstractmethod
    def compute(self, vehicle, target_waypoint, route_trace, route_index, reference_waypoint=None):
        """Return a dict containing tracking errors and target reference metadata."""
