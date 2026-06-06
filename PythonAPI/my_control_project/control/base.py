from abc import ABC, abstractmethod


class BaseTrackingController(ABC):
    """Common interface for all project-level tracking controllers."""

    @abstractmethod
    def run_step(
        self,
        vehicle,
        target_waypoint,
        curvature=0.0,
        reference_waypoint=None,
        planned_target_speed_mps=None,
        tracking_errors=None,
    ):
        """Return a CARLA VehicleControl for the current step."""

    def reset(self):
        """Reset controller state when needed."""
