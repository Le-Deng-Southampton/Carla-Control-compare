import math
import os
import sys
import unittest
from types import SimpleNamespace

import numpy as np


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from road_planning.frenet_planner import (
    FrenetPlannerConfig,
    FrenetPlanningFailure,
    build_centerline_model,
    controller_neutral_speed_cap,
    plan_frenet_reference,
    quintic_transition,
)


def waypoint(x, y, yaw_deg, waypoint_id, lane_width=3.5, is_junction=False):
    location = SimpleNamespace(x=float(x), y=float(y), z=0.0)
    return SimpleNamespace(
        id=waypoint_id,
        road_id=1,
        section_id=0,
        lane_id=1,
        lane_width=float(lane_width),
        is_junction=is_junction,
        transform=SimpleNamespace(
            location=location,
            rotation=SimpleNamespace(yaw=float(yaw_deg)),
        ),
    )


def straight_route(spacing=5.0, length=100.0, lane_width=3.5):
    values = np.arange(0.0, length + spacing * 0.5, spacing)
    return [
        (waypoint(value, 0.0, 0.0, index, lane_width=lane_width), "LANEFOLLOW")
        for index, value in enumerate(values)
    ]


def constant_curvature_route(radius=50.0, length=100.0, spacing=1.0):
    arc = np.arange(0.0, length + spacing * 0.5, spacing)
    theta = arc / float(radius)
    return [
        (
            waypoint(
                radius * math.sin(angle),
                radius * (1.0 - math.cos(angle)),
                math.degrees(angle),
                index,
            ),
            "LANEFOLLOW",
        )
        for index, angle in enumerate(theta)
    ]


def abrupt_curvature_route():
    points = [
        (value, 0.0, 0.0)
        for value in np.arange(-30.0, 0.0, 1.0)
    ]
    for arc in np.arange(0.0, 20.1, 1.0):
        angle = arc / 10.0
        points.append(
            (
                10.0 * math.sin(angle),
                10.0 * (1.0 - math.cos(angle)),
                math.degrees(angle),
            )
        )
    return [
        (waypoint(x, y, yaw, index), "LANEFOLLOW")
        for index, (x, y, yaw) in enumerate(points)
    ]


def route_features():
    return {
        "length": 100.0,
        "route_label": "straight",
        "total_abs_turn": 0.0,
        "sign_changes": 0,
        "turn_segments": 0,
    }


class FrenetPlannerTest(unittest.TestCase):
    def test_quintic_transition_has_flat_position_heading_and_curvature_boundaries(self):
        s_values = np.linspace(0.0, 40.0, 4001)
        offsets = quintic_transition(s_values, 0.0, 40.0, 0.0, 0.6)
        first = np.gradient(offsets, s_values)
        second = np.gradient(first, s_values)

        self.assertAlmostEqual(offsets[0], 0.0, places=10)
        self.assertAlmostEqual(offsets[-1], 0.6, places=10)
        self.assertLess(abs(first[0]), 1e-5)
        self.assertLess(abs(first[-1]), 1e-5)
        self.assertLess(abs(second[0]), 1e-3)
        self.assertLess(abs(second[-1]), 1e-3)

    def test_centerline_resampling_is_invariant_to_raw_waypoint_density(self):
        dense = build_centerline_model(straight_route(spacing=0.5), FrenetPlannerConfig())
        sparse = build_centerline_model(straight_route(spacing=2.0), FrenetPlannerConfig())

        self.assertTrue(np.all(np.diff(dense.s_m) > 0.0))
        self.assertEqual(len(dense.s_m), len(sparse.s_m))
        self.assertLess(float(np.max(np.abs(dense.x_m - sparse.x_m))), 0.03)
        self.assertLess(float(np.max(np.abs(dense.y_m - sparse.y_m))), 0.03)

    def test_constant_curvature_route_is_not_rejected_by_dense_interpolation_aliasing(self):
        trajectory, diagnostics = plan_frenet_reference(
            constant_curvature_route(),
            dict(route_features(), route_label="constant_curvature", total_abs_turn=2.0),
            target_speed_kmh=50.0,
        )

        self.assertLessEqual(
            diagnostics["reference_max_abs_curvature_rate"],
            FrenetPlannerConfig().max_abs_curvature_rate_1pm2,
        )
        self.assertGreater(len(trajectory.s_m), 2)

    def test_genuine_output_grid_curvature_rate_step_is_rejected(self):
        with self.assertRaises(FrenetPlanningFailure) as raised:
            plan_frenet_reference(
                abrupt_curvature_route(),
                dict(route_features(), length=50.0, route_label="curvature_step"),
                target_speed_kmh=30.0,
            )

        self.assertGreater(raised.exception.rejection_counts.get("curvature_rate", 0), 0)

    def test_vehicle_footprint_rejects_lane_that_only_fits_center_point(self):
        with self.assertRaises(FrenetPlanningFailure) as raised:
            plan_frenet_reference(
                straight_route(lane_width=2.5),
                route_features(),
                target_speed_kmh=50.0,
            )

        self.assertEqual(raised.exception.code, "no_feasible_candidate")
        self.assertGreater(raised.exception.rejection_counts.get("lane_clearance", 0), 0)

    def test_controller_neutral_speed_cap_uses_curvature_and_curvature_rate(self):
        config = FrenetPlannerConfig()
        lateral_limited = controller_neutral_speed_cap(40.0, 0.04, 0.0, config)
        jerk_limited = controller_neutral_speed_cap(40.0, 0.0, 0.02, config)

        self.assertAlmostEqual(lateral_limited, math.sqrt(6.5 / 0.04))
        self.assertAlmostEqual(jerk_limited, (10.0 / 0.02) ** (1.0 / 3.0))

    def test_plan_is_deterministic_and_ignores_controller_metadata(self):
        first, first_diagnostics = plan_frenet_reference(
            straight_route(),
            dict(route_features(), controller_name="pid"),
            target_speed_kmh=50.0,
        )
        second, second_diagnostics = plan_frenet_reference(
            straight_route(),
            dict(route_features(), controller_name="mpc"),
            target_speed_kmh=50.0,
        )

        self.assertEqual(first.content_hash(), second.content_hash())
        self.assertEqual(first_diagnostics["selected_candidate_id"], second_diagnostics["selected_candidate_id"])
        self.assertIn("score_components", first_diagnostics)
        self.assertGreater(first_diagnostics["candidate_count"], 0)

    def test_timeout_returns_failure_instead_of_partial_trajectory(self):
        ticks = iter([0.0, 6.0, 6.0, 6.0])

        with self.assertRaises(FrenetPlanningFailure) as raised:
            plan_frenet_reference(
                straight_route(),
                route_features(),
                target_speed_kmh=50.0,
                clock=lambda: next(ticks),
            )

        self.assertEqual(raised.exception.code, "planning_timeout")


if __name__ == "__main__":
    unittest.main()
