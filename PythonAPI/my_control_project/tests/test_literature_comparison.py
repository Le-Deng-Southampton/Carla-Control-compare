import csv
import importlib.util
import os
import tempfile
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODULE_PATH = os.path.join(PROJECT_ROOT, "experiment", "literature_comparison.py")
SPEC = importlib.util.spec_from_file_location("literature_comparison", MODULE_PATH)
literature_comparison = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(literature_comparison)


class LiteratureComparisonTest(unittest.TestCase):
    def test_integrated_results_are_compared_with_lower_is_better_formula(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            raw_path = os.path.join(temp_dir, "raw_lap_results.csv")
            with open(raw_path, "w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=[
                    "layer", "controller", "status", "normalized_iae_e_y_m",
                    "max_abs_e_y", "steer_spectrum_stability_index",
                    "steer_spectrum_comfort_index",
                ])
                writer.writeheader()
                writer.writerow({
                    "layer": "integrated", "controller": "pid", "status": "ok",
                    "normalized_iae_e_y_m": 0.075, "max_abs_e_y": 0.248,
                    "steer_spectrum_stability_index": 0.061,
                    "steer_spectrum_comfort_index": 0.129,
                })
                writer.writerow({
                    "layer": "controller_only", "controller": "pid", "status": "ok",
                    "normalized_iae_e_y_m": 99, "max_abs_e_y": 99,
                    "steer_spectrum_stability_index": 99,
                    "steer_spectrum_comfort_index": 99,
                })

            rows = literature_comparison.compute_comparison(raw_path)

        pid = next(row for row in rows if row["controller"] == "PID")
        self.assertEqual(pid["sample_count"], 1)
        self.assertAlmostEqual(pid["iae_reference_delta_pct"], 50.0)
        self.assertAlmostEqual(pid["mle_reference_delta_pct"], 50.0)
        self.assertEqual(pid["strict_foundational_improvement_pct"], "N/A")

    def test_report_states_that_foundational_improvement_is_not_calculable(self):
        markdown = literature_comparison.render_markdown([])

        self.assertIn("不能计算", markdown)
        self.assertIn("不是 0%", markdown)
        self.assertIn("跨研究参考差值", markdown)


if __name__ == "__main__":
    unittest.main()
