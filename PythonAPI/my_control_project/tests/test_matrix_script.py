import os
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MATRIX_SCRIPT = os.path.join(PROJECT_ROOT, "scripts", "run_test_matrix.ps1")


def read_matrix_script():
    with open(MATRIX_SCRIPT, encoding="utf-8") as handle:
        return handle.read()


class StabilityMatrixScriptTest(unittest.TestCase):
    def test_stability_catalog_contains_approved_ten_scenarios(self):
        script = read_matrix_script()

        self.assertIn('ValidateSet("smoke", "extended", "stability")', script)
        for scenario in (
            "urban_30_true_straight",
            "urban_30_curvy",
            "urban_50_true_straight",
            "urban_50_s_curve",
            "urban_50_curvy",
            "road_70_true_straight",
            "road_70_s_curve",
            "road_70_curvy",
            "highway_120_true_straight",
            "highway_120_gentle_curve",
        ):
            self.assertIn(scenario, script)

    def test_stability_matrix_runs_both_modes_and_enforces_summary_gate(self):
        script = read_matrix_script()

        self.assertIn('@("off", "adaptive")', script)
        self.assertIn("stability_passed", script)
        self.assertIn("stability_fail_reasons", script)
        self.assertIn("p95_abs_steer_delta", script)
        self.assertIn("p95_abs_lateral_jerk", script)
        self.assertIn("exit 1", script)

    def test_stability_matrix_pairs_legacy_and_frenet_under_strict_limits(self):
        script = read_matrix_script()

        self.assertIn('[ValidateSet("legacy", "frenet", "both")]', script)
        self.assertIn('[string]$PlannerMode = "both"', script)
        self.assertIn('@("legacy", "frenet")', script)
        self.assertIn('"--planner-mode", $casePlannerMode', script)
        self.assertIn('"--speed-planner-limit-profile", "global"', script)
        self.assertIn("planner_mode = $PlannerMode", script)
        self.assertIn("planner_comparison.py", script)
        self.assertIn("_paired.csv", script)
        self.assertIn("hard_safety_regression", script)


if __name__ == "__main__":
    unittest.main()
