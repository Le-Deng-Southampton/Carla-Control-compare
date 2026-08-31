import csv
import importlib.util
import os
import sys
import tempfile
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

MODULE_PATH = os.path.join(PROJECT_ROOT, "experiment", "evaluation_aggregate.py")
SPEC = importlib.util.spec_from_file_location("evaluation_aggregate_under_test", MODULE_PATH)
AGGREGATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AGGREGATE)

aggregate_results = AGGREGATE.aggregate_results
pareto_front = AGGREGATE.pareto_front
relative_degradation = AGGREGATE.relative_degradation
summarize_robustness = AGGREGATE.summarize_robustness
validate_fair_triplets = AGGREGATE.validate_fair_triplets
wilson_interval = AGGREGATE.wilson_interval


def row(controller, trajectory_hash, case_id="case-1", status="ok"):
    return {
        "evaluation_case_id": case_id,
        "controller": controller,
        "trajectory_hash": trajectory_hash,
        "status": status,
    }


class EvaluationAggregateTest(unittest.TestCase):
    def test_triplet_requires_three_controllers_and_one_hash(self):
        with self.assertRaisesRegex(ValueError, "trajectory_hash"):
            validate_fair_triplets([row("pid", "a"), row("lqr", "a"), row("mpc", "b")])

        with self.assertRaisesRegex(ValueError, "controllers"):
            validate_fair_triplets([row("pid", "a"), row("lqr", "a")])

    def test_failed_rows_remain_in_robustness_denominator(self):
        summary = summarize_robustness([{"status": "ok"}, {"status": "failed"}])
        self.assertEqual(summary["total"], 2)
        self.assertEqual(summary["successes"], 1)
        self.assertEqual(summary["success_rate_pct"], 50.0)
        self.assertLess(summary["wilson_lower_pct"], 50.0)
        self.assertGreater(summary["wilson_upper_pct"], 50.0)

    def test_wilson_and_relative_degradation_have_expected_direction(self):
        lower, upper = wilson_interval(9, 10)
        self.assertLess(lower, 0.9)
        self.assertGreater(upper, 0.9)
        self.assertEqual(relative_degradation(1.2, 1.0), 20.0)
        self.assertEqual(relative_degradation(12.0, 10.0, lower_is_better=False), -20.0)

    def test_pareto_front_keeps_non_dominated_rows(self):
        rows = [
            {"id": "a", "error": 1, "time": 2},
            {"id": "b", "error": 2, "time": 1},
            {"id": "c", "error": 3, "time": 3},
        ]
        self.assertEqual({item["id"] for item in pareto_front(rows, ("error", "time"))}, {"a", "b"})

    def test_aggregate_preserves_failed_row_and_writes_review_artifacts(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            manifest_path = os.path.join(temp_dir, "manifest.json")
            raw_path = os.path.join(temp_dir, "raw.csv")
            output_dir = os.path.join(temp_dir, "out")
            with open(manifest_path, "w", encoding="utf-8-sig") as handle:
                handle.write('{"version":1,"cases":[]}')
            fields = [
                "evaluation_case_id", "controller", "trajectory_hash", "status", "layer",
                "normalized_iae_e_y_m", "controller_runtime_p95_ms", "collision_count",
                "lane_boundary_violation_count", "route_completion_pct",
            ]
            rows = [
                {**row("pid", "h"), "layer": "integrated", "normalized_iae_e_y_m": 0.2, "controller_runtime_p95_ms": 1, "collision_count": 0, "lane_boundary_violation_count": 0, "route_completion_pct": 100},
                {**row("lqr", "h", status="failed"), "layer": "integrated", "normalized_iae_e_y_m": "", "controller_runtime_p95_ms": "", "collision_count": 1, "lane_boundary_violation_count": 0, "route_completion_pct": 40},
                {**row("mpc", "h"), "layer": "integrated", "normalized_iae_e_y_m": 0.1, "controller_runtime_p95_ms": 10, "collision_count": 0, "lane_boundary_violation_count": 0, "route_completion_pct": 100},
            ]
            with open(raw_path, "w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)

            result = aggregate_results(manifest_path, raw_path, output_dir)

            self.assertEqual(result["raw_rows"], 3)
            for name in (
                "raw_lap_results.csv", "paired_results.csv", "controller_summary.csv",
                "robustness_results.csv", "failure_windows.json",
            ):
                self.assertTrue(os.path.exists(os.path.join(output_dir, name)), name)


if __name__ == "__main__":
    unittest.main()
