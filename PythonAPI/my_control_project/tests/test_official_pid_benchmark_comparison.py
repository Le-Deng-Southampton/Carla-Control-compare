import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXPERIMENT_ROOT = os.path.join(PROJECT_ROOT, "experiment")
if EXPERIMENT_ROOT not in sys.path:
    # Append instead of prepending so experiment/logging.py cannot shadow the
    # Python standard-library logging module in later tests.
    sys.path.append(EXPERIMENT_ROOT)

import official_pid_benchmark_comparison as comparison


def row(case_id, controller, value=1.0):
    result = {
        "evaluation_case_id": case_id,
        "controller": controller,
        "status": "ok",
        "layer": "controller_only",
        "trajectory_hash": "hash_" + case_id,
        "normalized_iae_e_y_m": str(value),
        "hard_gate_passed": "True",
    }
    for field in comparison.INVARIANT_FIELDS:
        result.setdefault(field, "same")
    result["trajectory_hash"] = "hash_" + case_id
    return result


class OfficialPidBenchmarkComparisonTest(unittest.TestCase):
    def test_rejects_non_official_controller_in_reference_population(self):
        with self.assertRaisesRegex(ValueError, "only controller"):
            comparison._case_index([row("a", "lqr")], expected_controller="pid")

    def test_pairwise_positive_means_candidate_is_better(self):
        optimized = {}
        for controller, value in (("pid", 2.0), ("lqr", 1.0), ("mpc", 1.5)):
            optimized[("a", controller)] = row("a", controller, value)
        pairs = comparison._optimized_pairs(optimized, "lqr", "pid", "all")
        _, reference, candidate, improvement, _ = comparison.bootstrap_metric(
            pairs, "normalized_iae_e_y_m", True, 50
        )
        self.assertEqual(reference, 2.0)
        self.assertEqual(candidate, 1.0)
        self.assertEqual(improvement, 50.0)

    def test_absolute_summary_keeps_official_and_three_optimized_populations(self):
        official = [row("a", "pid", 2.0)]
        optimized = [row("a", name, value) for name, value in (("pid", 1.8), ("lqr", 1.0), ("mpc", 1.5))]
        summary = comparison.build_absolute_summary(official, optimized)
        overall = {item["controller"] for item in summary if item["layer"] == "all"}
        self.assertEqual(overall, {"carla_0.9.14_official_pid", "pid", "lqr", "mpc"})

    def test_accepts_declared_system_speed_profiles_but_rejects_wrong_profile(self):
        official = []
        optimized = []
        for index in range(90):
            case_id = "case_{:02d}".format(index)
            layer = ("controller_only", "integrated", "integrated_stress")[index // 30]
            baseline = row(case_id, "pid")
            baseline["layer"] = layer
            baseline["speed_planner_limit_profile"] = "global" if layer == "controller_only" else "controller:pid"
            official.append(baseline)
            for controller in ("pid", "lqr", "mpc"):
                candidate = row(case_id, controller)
                candidate["layer"] = layer
                candidate["speed_planner_limit_profile"] = "global" if layer == "controller_only" else "controller:" + controller
                optimized.append(candidate)
        comparison.validate_populations(official, optimized)
        optimized[-1]["speed_planner_limit_profile"] = "controller:pid"
        with self.assertRaisesRegex(ValueError, "optimized_speed_planner_limit_profile"):
            comparison.validate_populations(official, optimized)

    def test_recomputes_missing_hard_gate_with_shared_definition(self):
        item = row("a", "pid")
        item["route_completion_pct"] = "98.0"
        item["hard_gate_passed"] = ""
        computed = comparison.with_recomputed_hard_gates([item], "official")
        self.assertFalse(computed[0]["hard_gate_passed"])
        self.assertIn("route_incomplete", computed[0]["hard_gate_reasons"])


if __name__ == "__main__":
    unittest.main()
