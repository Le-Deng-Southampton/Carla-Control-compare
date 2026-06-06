import numpy as np

from .base import BaseErrorProvider
from road_planning.tracking_geometry import compute_path_tracking_errors


class GroundTruthErrorProvider(BaseErrorProvider):
    def compute(self, vehicle, target_waypoint, route_trace, route_index, reference_waypoint=None):
        reference_waypoint = reference_waypoint or route_trace[route_index][0]
        tracking_errors = compute_path_tracking_errors(vehicle, reference_waypoint)
        target_loc = target_waypoint.transform.location
        target_yaw = np.radians(target_waypoint.transform.rotation.yaw)
        return {
            "e_y": tracking_errors["e_y"],
            "e_psi": tracking_errors["e_psi"],
            "target_x": target_loc.x,
            "target_y": target_loc.y,
            "target_yaw": target_yaw,
            "reference_x": tracking_errors["reference_x"],
            "reference_y": tracking_errors["reference_y"],
            "reference_yaw": tracking_errors["reference_yaw"],
        }
