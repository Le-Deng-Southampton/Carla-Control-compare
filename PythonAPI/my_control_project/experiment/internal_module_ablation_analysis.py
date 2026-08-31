"""Classify controller-internal modules from paired 90-case leave-one-out runs."""

import argparse
import csv
import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
CARLA_PYTHON_ROOT = os.path.join(os.path.dirname(PROJECT_ROOT), "carla")
sys.path = [path for path in sys.path if os.path.abspath(path or os.curdir) != SCRIPT_DIR]
for path in (CARLA_PYTHON_ROOT, PROJECT_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

from experiment.accurate_parallel_comparison import bootstrap_metric, bootstrap_rate
from experiment.evaluation_aggregate import hard_gate_result


METRICS = (
    "normalized_iae_e_y_m",
    "normalized_iae_e_psi_deg",
    "max_abs_e_y",
    "mean_abs_steer_delta",
    "p95_abs_lateral_jerk",
    "controller_runtime_p95_ms",
)


def _read_csv(path):
    with open(path, "r", encoding="utf-8-sig", newline="") as handle:
        return [dict(row) for row in csv.DictReader(handle)]


def _write_csv(path, rows):
    rows = list(rows)
    fields = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        if not fields:
            return
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _load_variant(item, expected_cases):
    path = os.path.join(item["output_dir"], "run_status.csv")
    rows = _read_csv(path)
    if len(rows) != expected_cases:
        raise ValueError(
            "{}/{} must contain {} rows, got {}".format(
                item["controller"], item["variant"], expected_cases, len(rows)
            )
        )
    by_case = {}
    for row in rows:
        case_id = row.get("evaluation_case_id", "")
        if not case_id or case_id in by_case:
            raise ValueError("missing or duplicate evaluation_case_id in {}".format(path))
        passed, reasons = hard_gate_result(row)
        row["hard_gate_passed"] = "true" if passed else "false"
        row["hard_gate_reasons"] = ";".join(reasons)
        by_case[case_id] = row
    return by_case


def _paired(without_module, full):
    if set(without_module) != set(full):
        raise ValueError("ablation and full profiles do not contain identical cases")
    pairs = []
    audit_fields = (
        "scenario",
        "layer",
        "requested_speed_kmh",
        "requested_route_shape",
        "requested_map",
        "seed",
    )
    for case_id in sorted(full):
        ablated = without_module[case_id]
        complete = full[case_id]
        for field in audit_fields:
            if str(ablated.get(field, "")) != str(complete.get(field, "")):
                raise ValueError("case {} differs in {}".format(case_id, field))
        hashes = (ablated.get("trajectory_hash", ""), complete.get("trajectory_hash", ""))
        if all(hashes) and hashes[0] != hashes[1]:
            raise ValueError("case {} trajectory_hash mismatch".format(case_id))
        pairs.append((ablated, complete))
    return pairs


def _classification(lateral_point, lateral_ci, hard_gate_delta, threshold):
    if hard_gate_delta is not None and hard_gate_delta < 0.0:
        return "negative"
    if hard_gate_delta is not None and hard_gate_delta > 0.0:
        return "positive"
    if lateral_point is None or lateral_ci is None or None in lateral_ci:
        return "no_significant_effect"
    if lateral_point >= threshold and lateral_ci[0] > 0.0:
        return "positive"
    if lateral_point <= -threshold and lateral_ci[1] < 0.0:
        return "negative"
    return "no_significant_effect"


def analyze(manifest_path, output_dir):
    with open(manifest_path, "r", encoding="utf-8-sig") as handle:
        manifest = json.load(handle)
    expected = int(manifest["expected_cases_per_variant"])
    iterations = int(manifest.get("bootstrap_iterations", 5000))
    threshold = float(manifest["classification"]["practical_effect_threshold_pct"])
    variants = manifest["variants"]
    loaded = {
        (item["controller"], item["variant"]): _load_variant(item, expected)
        for item in variants
    }
    effects = []
    metric_rows = []
    for item in variants:
        if item["variant"] == "full":
            continue
        controller = item["controller"]
        full = loaded[(controller, "full")]
        ablated = loaded[(controller, item["variant"])]
        pairs = _paired(ablated, full)
        lateral = None
        for metric in METRICS:
            n, ablated_mean, full_mean, point, interval = bootstrap_metric(
                pairs, metric, True, iterations
            )
            record = {
                "controller": controller,
                "module": item["removed_module"],
                "metric": metric,
                "paired_cases": n,
                "mean_without_module": ablated_mean,
                "mean_with_module": full_mean,
                "module_improvement_pct": point,
                "bootstrap_ci95_low_pct": interval[0] if interval else None,
                "bootstrap_ci95_high_pct": interval[1] if interval else None,
            }
            metric_rows.append(record)
            if metric == "normalized_iae_e_y_m":
                lateral = record
        ablated_rate, full_rate, gate_delta, gate_interval = bootstrap_rate(
            pairs, "hard_gate_passed", iterations
        )
        classification = _classification(
            lateral["module_improvement_pct"],
            (
                lateral["bootstrap_ci95_low_pct"],
                lateral["bootstrap_ci95_high_pct"],
            ),
            gate_delta,
            threshold,
        )
        effects.append({
            "controller": controller,
            "module": item["removed_module"],
            "classification": classification,
            "paired_cases": len(pairs),
            "lateral_iae_improvement_pct": lateral["module_improvement_pct"],
            "lateral_iae_ci95_low_pct": lateral["bootstrap_ci95_low_pct"],
            "lateral_iae_ci95_high_pct": lateral["bootstrap_ci95_high_pct"],
            "hard_gate_rate_without_module_pct": ablated_rate,
            "hard_gate_rate_with_module_pct": full_rate,
            "hard_gate_rate_delta_pp": gate_delta,
            "hard_gate_delta_ci95_low_pp": gate_interval[0],
            "hard_gate_delta_ci95_high_pp": gate_interval[1],
        })
    os.makedirs(output_dir, exist_ok=True)
    _write_csv(os.path.join(output_dir, "module_effects.csv"), effects)
    _write_csv(os.path.join(output_dir, "module_metric_effects.csv"), metric_rows)
    with open(os.path.join(output_dir, "module_effects.json"), "w", encoding="utf-8") as handle:
        json.dump(effects, handle, indent=2, ensure_ascii=False)
    return effects


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    analyze(args.manifest, args.output_dir)


if __name__ == "__main__":
    main()
