import math
import os
import sys
import unittest
from types import SimpleNamespace


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from road_planning.reference_path import ReferencePathConfig, build_feasible_reference_trace


def waypoint(x, y, yaw_deg, lane_width=3.5, is_junction=False, waypoint_id=0):
    return SimpleNamespace(
        id=waypoint_id,
        lane_width=lane_width,
        is_junction=is_junction,
        transform=SimpleNamespace(
            location=SimpleNamespace(
                x=float(x),
                y=float(y),
                z=0.0,
                distance=lambda other: math.sqrt((float(x) - other.x) ** 2 + (float(y) - other.y) ** 2),
            ),
            rotation=SimpleNamespace(yaw=float(yaw_deg)),
        ),
    )


class ReferencePathTest(unittest.TestCase):
    def test_smoothing_builds_dense_waypoint_like_reference_trace(self):
        route = [
            (waypoint(0.0, 0.0, 0.0, waypoint_id=0), "LANEFOLLOW"),
            (waypoint(10.0, 0.0, 0.0, waypoint_id=1), "LANEFOLLOW"),
        ]

        reference_trace, features = build_feasible_reference_trace(
            route,
            ReferencePathConfig(sample_spacing_m=1.0, smoothing_window=5),
        )

        self.assertGreater(len(reference_trace), len(route))
        self.assertTrue(features["reference_validation_passed"])
        self.assertAlmostEqual(reference_trace[0][0].transform.location.x, 0.0)
        self.assertAlmostEqual(reference_trace[-1][0].transform.location.x, 10.0)

    def test_lane_boundary_clamp_keeps_smoothed_path_feasible(self):
        route = [
            (waypoint(0.0, 0.0, 0.0, lane_width=3.0, waypoint_id=0), "LANEFOLLOW"),
            (waypoint(4.0, 1.4, 0.0, lane_width=3.0, waypoint_id=1), "LANEFOLLOW"),
            (waypoint(8.0, 0.0, 0.0, lane_width=3.0, waypoint_id=2), "LANEFOLLOW"),
        ]

        reference_trace, features = build_feasible_reference_trace(
            route,
            ReferencePathConfig(
                sample_spacing_m=1.0,
                smoothing_window=3,
                lane_margin_m=0.45,
                max_centerline_offset_m=2.0,
            ),
        )

        self.assertTrue(features["reference_validation_passed"])
        self.assertGreaterEqual(features["reference_min_lane_clearance_m"], 0.45)
        self.assertLessEqual(features["reference_max_centerline_offset_m"], 1.05)
        for reference_wp, _ in reference_trace:
            self.assertGreaterEqual(reference_wp.left_clearance_m, 0.45)
            self.assertGreaterEqual(reference_wp.right_clearance_m, 0.45)

    def test_scene_coverage_records_curves_sign_changes_and_junction_samples(self):
        route = [
            (waypoint(0.0, 0.0, 0.0, waypoint_id=0), "LANEFOLLOW"),
            (waypoint(3.0, 1.0, 20.0, waypoint_id=1), "LEFT"),
            (waypoint(6.0, 0.0, -20.0, is_junction=True, waypoint_id=2), "RIGHT"),
            (waypoint(9.0, -1.0, -20.0, is_junction=True, waypoint_id=3), "RIGHT"),
            (waypoint(12.0, 0.0, 20.0, waypoint_id=4), "LEFT"),
        ]

        _, features = build_feasible_reference_trace(
            route,
            ReferencePathConfig(sample_spacing_m=1.0, smoothing_window=3),
        )

        curved_distance = (
            features["reference_gentle_curve_m"]
            + features["reference_moderate_curve_m"]
            + features["reference_tight_curve_m"]
        )
        self.assertGreater(curved_distance, 0.0)
        self.assertGreaterEqual(features["reference_s_curve_sign_changes"], 1)
        self.assertGreater(features["reference_junction_samples"], 0)


if __name__ == "__main__":
    unittest.main()
