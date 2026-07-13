import json
import math
import os
import sys
import tempfile
import unittest

import numpy as np


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from road_planning.reference_trajectory import (
    ReferenceTrajectory,
    load_reference_trajectory,
    save_reference_trajectory,
    trajectory_from_route_trace,
)


def make_trajectory(x_values=None, metadata=None):
    x_values = x_values or [0.0, 5.0, 10.0]
    return ReferenceTrajectory(
        s_m=np.array([0.0, 5.0, 10.0]),
        x_m=np.array(x_values),
        y_m=np.array([0.0, 2.0, 4.0]),
        z_m=np.zeros(3),
        yaw_rad=np.radians([179.0, 180.0, 181.0]),
        curvature_1pm=np.array([0.0, 0.02, 0.04]),
        curvature_rate_1pm2=np.array([0.0, 0.004, 0.004]),
        left_clearance_m=np.array([1.0, 1.1, 1.2]),
        right_clearance_m=np.array([1.2, 1.1, 1.0]),
        speed_cap_mps=np.array([20.0, 18.0, 16.0]),
        source_index=np.array([0, 1, 2]),
        road_options=("LANEFOLLOW", "LEFT", "LEFT"),
        metadata=metadata or {"planner_mode": "test", "planner_version": 1},
    )


class ReferenceTrajectoryTest(unittest.TestCase):
    def test_sample_at_interpolates_by_arc_length_and_unwrapped_yaw(self):
        sample = make_trajectory().sample_at(7.5)

        self.assertAlmostEqual(sample.s_m, 7.5)
        self.assertAlmostEqual(sample.x_m, 7.5)
        self.assertAlmostEqual(sample.y_m, 3.0)
        self.assertAlmostEqual(sample.curvature_1pm, 0.03)
        self.assertAlmostEqual(math.degrees(sample.yaw_rad), -179.5)
        self.assertEqual(sample.source_index, 2)
        self.assertEqual(sample.road_option, "LEFT")

    def test_sample_at_clamps_to_trajectory_bounds(self):
        trajectory = make_trajectory()

        self.assertAlmostEqual(trajectory.sample_at(-3.0).s_m, 0.0)
        self.assertAlmostEqual(trajectory.sample_at(30.0).s_m, 10.0)

    def test_rejects_non_increasing_arc_length_and_mismatched_arrays(self):
        values = make_trajectory().to_dict()
        values["trajectory"]["s_m"] = [0.0, 1.0, 1.0]
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            ReferenceTrajectory.from_dict(values)

        values = make_trajectory().to_dict()
        values["trajectory"]["x_m"] = [0.0, 1.0]
        with self.assertRaisesRegex(ValueError, "same length"):
            ReferenceTrajectory.from_dict(values)

    def test_arrays_and_metadata_are_immutable_copies(self):
        source = np.array([0.0, 5.0, 10.0])
        trajectory = make_trajectory(x_values=source.tolist())
        source[1] = 99.0

        self.assertEqual(trajectory.x_m[1], 5.0)
        with self.assertRaises(ValueError):
            trajectory.x_m[1] = 99.0
        with self.assertRaises(TypeError):
            trajectory.metadata["new"] = "value"

    def test_hash_excludes_execution_metadata_but_includes_geometry(self):
        first = make_trajectory()
        second = make_trajectory()
        changed = make_trajectory(x_values=[0.0, 5.1, 10.0])

        self.assertEqual(first.content_hash(), second.content_hash())
        self.assertNotEqual(first.content_hash(), changed.content_hash())

    def test_json_round_trip_preserves_hash_and_execution_metadata(self):
        trajectory = make_trajectory()
        with tempfile.TemporaryDirectory() as temp_dir:
            path = os.path.join(temp_dir, "reference_trajectory.json")
            save_reference_trajectory(path, trajectory, {"planning_duration_s": 1.23})
            loaded = load_reference_trajectory(path)
            with open(path, "r", encoding="utf-8") as handle:
                raw = json.load(handle)

        self.assertEqual(loaded.content_hash(), trajectory.content_hash())
        self.assertEqual(loaded.road_options, trajectory.road_options)
        self.assertEqual(raw["execution"]["planning_duration_s"], 1.23)

    def test_route_trace_adapter_builds_waypoint_like_samples(self):
        def waypoint(x, y, waypoint_id):
            location = type("Location", (), {"x": x, "y": y, "z": 0.0})()
            rotation = type("Rotation", (), {"yaw": 0.0})()
            transform = type("Transform", (), {"location": location, "rotation": rotation})()
            return type(
                "Waypoint",
                (),
                {"id": waypoint_id, "lane_width": 3.5, "source_index": waypoint_id, "transform": transform},
            )()

        route = [(waypoint(0.0, 0.0, 0), "LANEFOLLOW"), (waypoint(5.0, 0.0, 1), "LEFT")]
        trajectory = trajectory_from_route_trace(route, metadata={"planner_mode": "legacy"})
        replay_trace = trajectory.to_route_trace()

        self.assertEqual(len(replay_trace), 2)
        self.assertAlmostEqual(replay_trace[1][0].transform.location.x, 5.0)
        self.assertEqual(replay_trace[1][1], "LEFT")


if __name__ == "__main__":
    unittest.main()
