import math
import os
import sys
import unittest

import numpy as np


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from road_planning.reference_tracker import ReferenceTracker, ReferenceTrackingFailure
from road_planning.reference_trajectory import ReferenceTrajectory


def trajectory_from_points(points, yaws=None, curvatures=None):
    points = np.asarray(points, dtype=float)
    distances = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    s_values = np.concatenate(([0.0], np.cumsum(distances)))
    if yaws is None:
        dx = np.gradient(points[:, 0], s_values)
        dy = np.gradient(points[:, 1], s_values)
        yaws = np.unwrap(np.arctan2(dy, dx))
    if curvatures is None:
        curvatures = np.zeros(len(points))
    return ReferenceTrajectory(
        s_m=s_values,
        x_m=points[:, 0],
        y_m=points[:, 1],
        z_m=np.zeros(len(points)),
        yaw_rad=np.asarray(yaws, dtype=float),
        curvature_1pm=np.asarray(curvatures, dtype=float),
        curvature_rate_1pm2=np.zeros(len(points)),
        left_clearance_m=np.full(len(points), 1.75),
        right_clearance_m=np.full(len(points), 1.75),
        speed_cap_mps=np.full(len(points), 30.0),
        source_index=np.arange(len(points)),
        road_options=tuple("LANEFOLLOW" for _ in points),
        metadata={"planner_mode": "test"},
    )


class ReferenceTrackerTest(unittest.TestCase):
    def test_projects_continuously_and_is_invariant_to_sample_density(self):
        sparse = trajectory_from_points([(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)])
        dense = trajectory_from_points([(float(x), 0.0) for x in range(21)])

        sparse_result = ReferenceTracker(sparse).update(5.0, 1.0, 0.0, 10.0, 0.05)
        dense_result = ReferenceTracker(dense).update(5.0, 1.0, 0.0, 10.0, 0.05)

        self.assertAlmostEqual(sparse_result.s_ref_m, 5.0)
        self.assertAlmostEqual(sparse_result.lateral_error_m, 1.0)
        self.assertAlmostEqual(sparse_result.s_ref_m, dense_result.s_ref_m)
        self.assertAlmostEqual(ReferenceTracker(sparse).trajectory.sample_at(15.0).x_m, 15.0)

    def test_crossing_uses_prior_progress_and_heading_to_select_branch(self):
        crossing = trajectory_from_points(
            [(-10.0, 0.0), (0.0, 0.0), (10.0, 0.0), (0.0, 0.0), (-10.0, 0.0)],
            yaws=[0.0, 0.0, 0.0, math.pi, math.pi],
        )
        tracker = ReferenceTracker(crossing)
        first = tracker.update(9.0, 0.0, 0.0, 10.0, 0.05)
        second = tracker.update(0.0, 0.0, math.pi, 10.0, 0.5)

        self.assertGreater(first.s_ref_m, 18.0)
        self.assertGreater(second.s_ref_m, 29.0)
        self.assertEqual(second.segment_index, 2)

    def test_normal_progress_is_monotonic_and_recovery_allows_bounded_rollback(self):
        trajectory = trajectory_from_points([(0.0, 0.0), (20.0, 0.0)])
        tracker = ReferenceTracker(trajectory, max_rollback_m=2.0)
        tracker.update(10.0, 0.0, 0.0, 5.0, 0.1)

        normal = tracker.update(9.0, 0.0, 0.0, 5.0, 0.1)
        recovered = tracker.update(9.0, 0.0, 0.0, 0.5, 0.1, allow_recovery=True)

        self.assertAlmostEqual(normal.s_ref_m, 10.0)
        self.assertAlmostEqual(recovered.s_ref_m, 9.0)
        self.assertEqual(recovered.state, "rollback")

    def test_large_progress_jump_is_held_instead_of_accepted(self):
        trajectory = trajectory_from_points([(0.0, 0.0), (100.0, 0.0)])
        tracker = ReferenceTracker(trajectory)
        tracker.update(0.0, 0.0, 0.0, 1.0, 0.05)

        result = tracker.update(50.0, 0.0, 0.0, 1.0, 0.05)

        self.assertTrue(result.held)
        self.assertAlmostEqual(result.s_ref_m, 0.0)
        self.assertEqual(result.state, "held_progress_jump")

    def test_invalid_projection_holds_three_cycles_then_fails(self):
        trajectory = trajectory_from_points([(0.0, 0.0), (20.0, 0.0)])
        tracker = ReferenceTracker(trajectory, hold_steps=3)
        tracker.update(2.0, 0.0, 0.0, 3.0, 0.1)

        for _ in range(3):
            held = tracker.update(1000.0, 1000.0, 0.0, 3.0, 0.1)
            self.assertTrue(held.held)
        with self.assertRaisesRegex(ReferenceTrackingFailure, "off_route"):
            tracker.update(1000.0, 1000.0, 0.0, 3.0, 0.1)

    def test_preview_uses_physical_distance_and_completion_needs_endpoint_proximity(self):
        trajectory = trajectory_from_points(
            [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0)],
            curvatures=[0.0, 0.1, 0.2],
        )
        tracker = ReferenceTracker(trajectory)
        tracker.update(5.0, 0.0, 0.0, 5.0, 0.1)

        self.assertAlmostEqual(tracker.target_sample(10.0).s_m, 15.0)
        self.assertEqual(tracker.preview_s_m([0.0, 5.0, 50.0]), [5.0, 10.0, 20.0])
        self.assertEqual(tracker.curvature_preview([0.0, 5.0]), [0.05, 0.1])
        self.assertFalse(tracker.is_complete(20.0, 20.0))

        tracker.update(19.0, 0.0, 0.0, 5.0, 1.0)
        self.assertTrue(tracker.is_complete(20.0, 0.0))


if __name__ == "__main__":
    unittest.main()
