import csv
import importlib.util
import os
import tempfile
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODULE_PATH = os.path.join(PROJECT_ROOT, "experiment", "initial_vs_optimized.py")
SPEC = importlib.util.spec_from_file_location("initial_vs_optimized", MODULE_PATH)
initial_vs_optimized = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(initial_vs_optimized)


FIELDS = [
    "evaluation_case_id", "controller", "layer", "status", "trajectory_hash",
    "normalized_iae_e_y_m", "max_abs_e_y", "hard_gate_passed",
]


def write_rows(path, rows):
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


class InitialVsOptimizedComparisonTest(unittest.TestCase):
    def test_computes_paired_lower_is_better_improvement_and_pass_rate_points(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            initial_path = os.path.join(temp_dir, "initial.csv")
            optimized_path = os.path.join(temp_dir, "optimized.csv")
            common = {
                "evaluation_case_id": "case_a", "controller": "pid",
                "layer": "integrated", "status": "ok", "trajectory_hash": "abc",
            }
            write_rows(initial_path, [dict(common, normalized_iae_e_y_m="0.2", max_abs_e_y="1.0", hard_gate_passed="False")])
            write_rows(optimized_path, [dict(common, normalized_iae_e_y_m="0.1", max_abs_e_y="0.8", hard_gate_passed="True")])

            rows = initial_vs_optimized.compute_comparison(initial_path, optimized_path)

        total = next(row for row in rows if row["controller"] == "pid" and row["layer"] == "all")
        self.assertEqual(total["paired_laps"], 1)
        self.assertAlmostEqual(total["normalized_iae_e_y_m_improvement_pct"], 50.0)
        self.assertAlmostEqual(total["max_abs_e_y_improvement_pct"], 20.0)
        self.assertAlmostEqual(total["hard_gate_pass_rate_change_pp"], 100.0)

    def test_rejects_nonidentical_reference_trajectories(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            initial_path = os.path.join(temp_dir, "initial.csv")
            optimized_path = os.path.join(temp_dir, "optimized.csv")
            base = {
                "evaluation_case_id": "case_a", "controller": "mpc", "layer": "integrated",
                "status": "ok", "normalized_iae_e_y_m": "0.2", "max_abs_e_y": "1.0",
                "hard_gate_passed": "True",
            }
            write_rows(initial_path, [dict(base, trajectory_hash="initial")])
            write_rows(optimized_path, [dict(base, trajectory_hash="optimized")])

            with self.assertRaisesRegex(ValueError, "trajectory hash"):
                initial_vs_optimized.compute_comparison(initial_path, optimized_path)

    def test_html_report_labels_regressions_instead_of_calling_them_improvements(self):
        rows = [{
            "controller": "pid", "layer": "all", "paired_laps": 90,
            "initial_normalized_iae_e_y_m_mean": 0.1,
            "optimized_normalized_iae_e_y_m_mean": 0.2,
            "normalized_iae_e_y_m_improvement_pct": -100.0,
            "initial_max_abs_e_y_mean": 0.5,
            "optimized_max_abs_e_y_mean": 0.4,
            "max_abs_e_y_improvement_pct": 20.0,
            "steer_spectrum_stability_index_improvement_pct": -5.0,
            "steer_spectrum_comfort_index_improvement_pct": 10.0,
            "controller_runtime_p95_ms_improvement_pct": -30.0,
            "initial_hard_gate_pass_rate_pct": 50.0,
            "optimized_hard_gate_pass_rate_pct": 60.0,
            "hard_gate_pass_rate_change_pp": 10.0,
        }]

        html = initial_vs_optimized.render_html(rows, "initial.csv", "optimized.csv")

        self.assertIn("<!doctype html>", html.lower())
        self.assertIn("优化后变差", html)
        self.assertIn("同轨迹严格配对", html)


if __name__ == "__main__":
    unittest.main()
