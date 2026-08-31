"""Paired aggregation and hard-gate evaluation for controller experiments."""

import argparse
import csv
import json
import math
import os
from collections import defaultdict
from statistics import mean, median


CONTROLLERS = {"pid", "lqr", "mpc"}
PRIMARY_OBJECTIVES = (
    "normalized_iae_e_y_m",
    "normalized_iae_e_psi_deg",
    "controller_runtime_p95_ms",
)


def _case_id(row):
    return str(row.get("evaluation_case_id") or row.get("case_id") or "")


def validate_fair_triplets(rows):
    """Require PID/LQR/MPC triplets to share one frozen reference trajectory."""
    groups = defaultdict(list)
    for item in rows:
        groups[_case_id(item)].append(item)
    for case_id, items in groups.items():
        controllers = {str(item.get("controller", "")).lower() for item in items}
        if controllers != CONTROLLERS or len(items) != 3:
            raise ValueError(
                f"evaluation case {case_id!r} controllers must be exactly pid/lqr/mpc"
            )
        hashes = {str(item.get("trajectory_hash", "")) for item in items}
        if len(hashes) != 1 or "" in hashes:
            raise ValueError(
                f"evaluation case {case_id!r} trajectory_hash must match across controllers"
            )
    return True


def _validate_aggregate_triplets(rows):
    """Validate complete triplets while allowing all-controller failed placeholders."""
    groups = defaultdict(list)
    for item in rows:
        groups[_case_id(item)].append(item)
    for case_id, items in groups.items():
        controllers = {str(item.get("controller", "")).lower() for item in items}
        if controllers != CONTROLLERS or len(items) != 3:
            raise ValueError(
                f"evaluation case {case_id!r} controllers must be exactly pid/lqr/mpc"
            )
        hashes = {
            str(item.get("trajectory_hash", ""))
            for item in items
            if str(item.get("trajectory_hash", ""))
        }
        if len(hashes) > 1:
            raise ValueError(
                f"evaluation case {case_id!r} trajectory_hash must match across controllers"
            )
        if all(_is_success(item) for item in items) and len(hashes) != 1:
            raise ValueError(
                f"evaluation case {case_id!r} completed rows require trajectory_hash"
            )
    return True


def wilson_interval(successes, total, z=1.959963984540054):
    successes = int(successes)
    total = int(total)
    if total <= 0:
        return 0.0, 0.0
    if successes < 0 or successes > total:
        raise ValueError("successes must be between zero and total")
    p = successes / float(total)
    denominator = 1.0 + z * z / total
    center = (p + z * z / (2.0 * total)) / denominator
    margin = z * math.sqrt(
        (p * (1.0 - p) + z * z / (4.0 * total)) / total
    ) / denominator
    return center - margin, center + margin


def _is_success(row):
    if str(row.get("status", "ok")).lower() not in {"ok", "passed", "success"}:
        return False
    explicit = row.get("hard_gate_passed")
    if explicit not in (None, ""):
        return _as_bool(explicit)
    return True


def summarize_robustness(rows):
    rows = list(rows)
    successes = sum(1 for item in rows if _is_success(item))
    total = len(rows)
    lower, upper = wilson_interval(successes, total)
    return {
        "successes": successes,
        "total": total,
        "success_rate_pct": 100.0 * successes / total if total else 0.0,
        "wilson_lower_pct": 100.0 * lower,
        "wilson_upper_pct": 100.0 * upper,
        "robustness_90pct_gate_passed": total > 0 and successes / float(total) >= 0.90,
    }


def relative_degradation(value, baseline, lower_is_better=True):
    value = float(value)
    baseline = float(baseline)
    if baseline == 0.0:
        if value == 0.0:
            return 0.0
        return math.inf if (value > 0.0) == lower_is_better else -math.inf
    change = (value - baseline) / abs(baseline) * 100.0
    return round(change if lower_is_better else -change, 12)


def pareto_front(rows, objectives):
    objectives = tuple(objectives)
    result = []
    for item in rows:
        if any(item.get(key) in (None, "") for key in objectives):
            continue
        dominated = False
        for other in rows:
            if other is item or any(other.get(key) in (None, "") for key in objectives):
                continue
            no_worse = all(float(other[key]) <= float(item[key]) for key in objectives)
            strictly_better = any(float(other[key]) < float(item[key]) for key in objectives)
            if no_worse and strictly_better:
                dominated = True
                break
        if not dominated:
            result.append(item)
    return result


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "passed", "ok"}


def _as_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def hard_gate_result(row):
    reasons = []
    if str(row.get("status", "ok")).lower() not in {"ok", "passed", "success"}:
        reasons.append("experiment_failed")
    if _as_float(row.get("collision_count")) > 0.0:
        reasons.append("collision")
    if _as_float(row.get("lane_boundary_violation_count")) > 0.0:
        reasons.append("lane_boundary_violation")
    if _as_float(row.get("route_completion_pct"), 100.0) < 99.0:
        reasons.append("route_incomplete")
    if str(row.get("tracking_failure_reason", "")).strip():
        reasons.append("tracking_failure")
    if str(row.get("spectral_metrics_valid", "true")).lower() in {"false", "0"}:
        reasons.append("spectral_metric_invalid")
    return not reasons, reasons


def _read_csv(path):
    with open(path, "r", encoding="utf-8-sig", newline="") as handle:
        return [dict(item) for item in csv.DictReader(handle)]


def _write_csv(path, rows, preferred_fields=()):
    rows = list(rows)
    fields = list(preferred_fields)
    for item in rows:
        for key in item:
            if key not in fields:
                fields.append(key)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        if not fields:
            handle.write("")
            return
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _numeric_values(rows, field):
    result = []
    for item in rows:
        value = item.get(field)
        if value in (None, ""):
            continue
        try:
            result.append(float(value))
        except (TypeError, ValueError):
            continue
    return result


def _paired_rows(rows):
    groups = defaultdict(list)
    for item in rows:
        groups[_case_id(item)].append(item)
    paired = []
    for case_id, items in groups.items():
        pid = next((item for item in items if str(item.get("controller", "")).lower() == "pid"), None)
        for item in items:
            output = dict(item)
            output["evaluation_case_id"] = case_id
            for metric in PRIMARY_OBJECTIVES:
                values = sorted(
                    (float(other[metric]), str(other.get("controller", "")))
                    for other in items
                    if other.get(metric) not in (None, "")
                )
                output[f"{metric}_rank"] = next(
                    (index + 1 for index, (_, controller) in enumerate(values)
                     if controller == str(item.get("controller", ""))),
                    "",
                )
                if pid and pid.get(metric) not in (None, "") and item.get(metric) not in (None, ""):
                    output[f"{metric}_degradation_vs_pid_pct"] = relative_degradation(
                        item[metric], pid[metric]
                    )
                else:
                    output[f"{metric}_degradation_vs_pid_pct"] = ""
            paired.append(output)
    return paired


def _controller_summaries(rows):
    groups = defaultdict(list)
    for item in rows:
        groups[(str(item.get("layer", "")), str(item.get("controller", "")))].append(item)
    summaries = []
    aggregate_metrics = (
        "normalized_iae_e_y_m",
        "normalized_iae_e_psi_deg",
        "rms_speed_error_mps",
        "steer_spectrum_stability_index",
        "steer_spectrum_comfort_index",
        "max_abs_lateral_accel_0p5s",
        "max_abs_lateral_jerk_0p5s",
        "controller_runtime_p95_ms",
        "controller_runtime_p99_ms",
    )
    for (layer, controller), items in sorted(groups.items()):
        robustness = summarize_robustness(items)
        output = {"layer": layer, "controller": controller, **robustness}
        for metric in aggregate_metrics:
            values = _numeric_values(items, metric)
            output[metric] = mean(values) if values else ""
            output[f"{metric}_median"] = median(values) if values else ""
        summaries.append(output)

    eligible = [item for item in summaries if item["successes"] == item["total"]]
    objectives = tuple(
        key for key in PRIMARY_OBJECTIVES if all(item.get(key) not in (None, "") for item in eligible)
    )
    pareto_ids = {
        (item["layer"], item["controller"])
        for item in (pareto_front(eligible, objectives) if objectives else [])
    }
    for item in summaries:
        item["pareto_optimal"] = (item["layer"], item["controller"]) in pareto_ids
    return summaries


def aggregate_results(manifest_path, raw_path, output_dir):
    """Validate and aggregate long-form lap results into review-ready artifacts."""
    os.makedirs(output_dir, exist_ok=True)
    with open(manifest_path, "r", encoding="utf-8-sig") as handle:
        manifest = json.load(handle)
    rows = _read_csv(raw_path)

    for item in rows:
        passed, reasons = hard_gate_result(item)
        item["hard_gate_passed"] = passed
        item["hard_gate_reasons"] = ";".join(reasons)
    _validate_aggregate_triplets(rows)

    paired = _paired_rows(rows)
    summaries = _controller_summaries(rows)
    robustness = []
    grouped = defaultdict(list)
    for item in rows:
        grouped[(str(item.get("layer", "")), str(item.get("controller", "")))].append(item)
    for (layer, controller), items in sorted(grouped.items()):
        robustness.append({"layer": layer, "controller": controller, **summarize_robustness(items)})

    failure_windows = []
    for item in rows:
        if _is_success(item):
            continue
        raw_windows = item.get("stability_fail_windows", "")
        try:
            windows = json.loads(raw_windows) if raw_windows else []
        except (TypeError, json.JSONDecodeError):
            windows = raw_windows
        failure_windows.append({
            "evaluation_case_id": _case_id(item),
            "controller": item.get("controller", ""),
            "status": item.get("status", ""),
            "hard_gate_reasons": item.get("hard_gate_reasons", ""),
            "windows": windows,
        })

    _write_csv(os.path.join(output_dir, "raw_lap_results.csv"), rows)
    _write_csv(os.path.join(output_dir, "paired_results.csv"), paired)
    _write_csv(os.path.join(output_dir, "controller_summary.csv"), summaries)
    _write_csv(os.path.join(output_dir, "robustness_results.csv"), robustness)
    with open(os.path.join(output_dir, "failure_windows.json"), "w", encoding="utf-8") as handle:
        json.dump(
            {"manifest_version": manifest.get("version"), "failures": failure_windows},
            handle,
            ensure_ascii=False,
            indent=2,
        )
    return {
        "raw_rows": len(rows),
        "paired_rows": len(paired),
        "controller_summaries": len(summaries),
        "failure_rows": len(failure_windows),
    }


def main():
    parser = argparse.ArgumentParser(description="Aggregate paired controller evaluation results.")
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--raw", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    print(json.dumps(aggregate_results(args.manifest, args.raw, args.output_dir), sort_keys=True))


if __name__ == "__main__":
    main()
