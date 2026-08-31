import hashlib
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXPERIMENT_ROOT = os.path.join(PROJECT_ROOT, "experiment")
if EXPERIMENT_ROOT not in sys.path:
    sys.path.append(EXPERIMENT_ROOT)

from official_pid_evaluation import _add_gate_fields, build_outputs, validate


def _row(tmp_path, index, layer, official_hash):
    run_dir = tmp_path / "run_{}".format(index)
    run_dir.mkdir()
    config = {
        "controller_implementation": "authoritative_baseline",
        "authoritative_baseline_audit": {
            "label": "authoritative_baseline_v1",
            "pid": {
                "source": '"CARLA_0.9.14_VehiclePIDController"',
                "lateral": {"K_P": 1.95, "K_I": 0.05, "K_D": 0.2, "dt": 0.05},
                "longitudinal": {"K_P": 1.0, "K_I": 0.05, "K_D": 0.0, "dt": 0.05},
                "max_throttle": 0.75,
                "max_brake": 0.3,
                "max_steering": 0.8,
                "offset": 0.0,
                "official_implementation": {"sha256": official_hash},
            },
        },
    }
    (run_dir / "run_config.json").write_text(json.dumps(config), encoding="utf-8")
    row = {
        "evaluation_case_id": "case_{}".format(index),
        "scenario": "scenario_{}".format(index % 10),
        "layer": layer,
        "controller": "pid",
        "status": "ok",
        "run_dir": str(run_dir),
        "trajectory_hash": "hash_{}".format(index),
        "reference_validation_passed": "True",
        "spectral_metrics_valid": "True",
        "stability_passed": "True",
        "un_r79_reference_passed": "True",
        "collision_count": "0",
        "lane_boundary_violation_count": "0",
        "route_completion_pct": "100",
    }
    metrics = (
        "normalized_iae_e_y_m", "max_abs_e_y", "normalized_iae_e_psi_deg",
        "max_abs_e_psi_deg", "rms_speed_error_mps", "mean_abs_steer_delta",
        "significant_steer_reversals_per_10s", "p95_abs_lateral_accel",
        "p95_abs_lateral_jerk", "p95_abs_longitudinal_jerk",
        "throttle_brake_switches_per_10s", "steer_spectrum_stability_index",
        "steer_spectrum_comfort_index", "controller_runtime_p95_ms",
        "controller_runtime_deadline_miss_pct",
    )
    row.update({metric: str(0.1 + index / 1000.0) for metric in metrics})
    return row


class OfficialPidEvaluationTest(unittest.TestCase):
    def test_complete_official_matrix_validates_and_summarizes(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp_path = Path(directory)
            official = tmp_path / "controller.py"
            official.write_text("official", encoding="utf-8")
            official_hash = hashlib.sha256(official.read_bytes()).hexdigest()
            rows = []
            for layer in ("controller_only", "integrated", "integrated_stress"):
                rows.extend(_row(tmp_path, len(rows), layer, official_hash) for _ in range(30))
            result = validate(rows, str(official))
            self.assertTrue(result["ready_to_share"])
            enriched = _add_gate_fields(rows)
            metric_rows, pass_rows, failures, worst = build_outputs(enriched, iterations=100)
            overall = next(item for item in pass_rows if item["group_type"] == "overall" and item["check"] == "hard_gate_passed")
            self.assertEqual(overall["successes"], 90)
            self.assertEqual(len(worst), 10)
            self.assertEqual(failures, [])
            self.assertTrue(any(item["metric"] == "normalized_iae_e_y_m" for item in metric_rows))

    def test_hard_gate_failure_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            tmp_path = Path(directory)
            official = tmp_path / "controller.py"
            official.write_text("official", encoding="utf-8")
            official_hash = hashlib.sha256(official.read_bytes()).hexdigest()
            row = _row(tmp_path, 0, "controller_only", official_hash)
            row["collision_count"] = "1"
            enriched = _add_gate_fields([row])[0]
            self.assertFalse(enriched["hard_gate_passed"])
            self.assertEqual(enriched["hard_gate_reasons"], "collision")


if __name__ == "__main__":
    unittest.main()
