import numpy as np


def normalize_angle(angle_rad):
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


def compute_path_tracking_errors(vehicle, reference_waypoint):
    vehicle_transform = vehicle.get_transform()
    vehicle_yaw = np.radians(vehicle_transform.rotation.yaw)
    vehicle_location = vehicle_transform.location

    reference_transform = reference_waypoint.transform
    reference_location = reference_transform.location
    reference_yaw = np.radians(reference_transform.rotation.yaw)

    delta_x = vehicle_location.x - reference_location.x
    delta_y = vehicle_location.y - reference_location.y
    lateral_error = -delta_x * np.sin(reference_yaw) + delta_y * np.cos(reference_yaw)
    heading_error = normalize_angle(vehicle_yaw - reference_yaw)

    return {
        "e_y": lateral_error,
        "e_psi": heading_error,
        "reference_x": reference_location.x,
        "reference_y": reference_location.y,
        "reference_yaw": reference_yaw,
    }
