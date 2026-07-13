import os
import unittest
import importlib.util


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODULE_PATH = os.path.join(PROJECT_ROOT, "experiment", "planner_comparison.py")
SPEC = importlib.util.spec_from_file_location("planner_comparison_under_test", MODULE_PATH)
PLANNER_COMPARISON = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PLANNER_COMPARISON)
pair_planner_rows = PLANNER_COMPARISON.pair_planner_rows


def comparison_row(planner_mode, controller="pid", trajectory_hash=None, **overrides):
    row = {
        "status": "ok",
        "requested_map": "Town05",
        "seed": "123",
        "requested_route_shape": "s_curve",
        "requested_speed_kmh": "70",
        "speed_planner_mode": "off",
        "error_provider": "ground_truth",
        "controller": controller,
        "planner_mode": planner_mode,
        "trajectory_hash": trajectory_hash or planner_mode + "-hash",
        "rms_e_y": "1.0" if planner_mode == "legacy" else "0.8",
        "p95_abs_steer_delta": "0.010" if planner_mode == "legacy" else "0.008",
        "significant_steer_reversals_per_10s": "3.0" if planner_mode == "legacy" else "2.0",
        "route_completion_pct": "100",
        "collision_count": "0",
        "lane_boundary_violation_count": "0",
        "min_lane_clearance_m": "0.5",
        "stability_passed": "True",
    }
    row.update(overrides)
    return row


class PlannerComparisonTest(unittest.TestCase):
    def test_pairs_legacy_and_frenet_by_scenario_controller_and_mode(self):
        paired = pair_planner_rows([comparison_row("legacy"), comparison_row("frenet")])

        self.assertEqual(len(paired), 1)
        self.assertAlmostEqual(paired[0]["rms_e_y_improvement_pct"], 20.0)
        self.assertAlmostEqual(paired[0]["p95_steer_delta_change"], -0.002)
        self.assertFalse(paired[0]["hard_safety_regression"])

    def test_refuses_pair_with_mismatched_controller_or_missing_hash(self):
        bad_legacy = comparison_row("legacy", trajectory_hash="")
        bad_legacy["trajectory_hash"] = ""

        with self.assertRaisesRegex(ValueError, "trajectory_hash"):
            pair_planner_rows([bad_legacy, comparison_row("frenet")])

        with self.assertRaisesRegex(ValueError, "complete legacy/frenet pair"):
            pair_planner_rows([comparison_row("legacy", controller="pid"), comparison_row("frenet", controller="lqr")])

    def test_marks_hard_safety_regression(self):
        paired = pair_planner_rows([
            comparison_row("legacy"),
            comparison_row("frenet", collision_count="1"),
        ])

        self.assertTrue(paired[0]["hard_safety_regression"])
        self.assertIn("collision_count", paired[0]["hard_safety_reasons"])


if __name__ == "__main__":
    unittest.main()
