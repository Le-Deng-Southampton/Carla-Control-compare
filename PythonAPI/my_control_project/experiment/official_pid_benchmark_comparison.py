"""Compare three optimized controllers with CARLA 0.9.14's official PID.

The official population contains one VehiclePIDController result per case.  The
optimized population contains PID/LQR/MPC triplets for the same frozen cases.
All comparisons are paired by evaluation_case_id and rejected when route or
experimental-condition invariants differ.
"""

import argparse
import csv
import json
import os
from collections import defaultdict

from accurate_parallel_comparison import (
    HIGHER_IS_BETTER,
    INVARIANT_FIELDS,
    LAYERS,
    LOWER_IS_BETTER,
    PASS_FIELDS,
    bootstrap_metric,
    bootstrap_rate,
    file_sha256,
    mean,
    number,
    read_rows,
    truth,
    write_rows,
)
from evaluation_aggregate import hard_gate_result


CONTROLLERS = ("pid", "lqr", "mpc")
PAIRWISE = (("lqr", "pid"), ("mpc", "pid"), ("mpc", "lqr"))
SYSTEM_LAYERS = ("integrated", "integrated_stress")
SPEED_PROFILE_INVARIANTS = tuple(
    field for field in INVARIANT_FIELDS if field != "speed_planner_limit_profile"
)
KEY_METRICS = (
    "normalized_iae_e_y_m",
    "max_abs_e_y",
    "normalized_iae_e_psi_deg",
    "rms_speed_error_mps",
    "mean_abs_steer_delta",
    "significant_steer_reversals_per_10s",
    "p95_abs_lateral_accel",
    "p95_abs_lateral_jerk",
    "controller_runtime_p95_ms",
    "route_completion_pct",
)


def _case_index(rows, expected_controller=None):
    result = {}
    for row in rows:
        case_id = row.get("evaluation_case_id", "")
        if not case_id:
            raise ValueError("A row is missing evaluation_case_id.")
        if expected_controller is not None and row.get("controller") != expected_controller:
            raise ValueError("Official population must contain only controller={!r}.".format(expected_controller))
        key = (case_id, row.get("controller", ""))
        if key in result:
            raise ValueError("Duplicate result key: {}".format(key))
        result[key] = row
    return result


def with_recomputed_hard_gates(rows, population_label):
    """Apply one identical hard-gate definition to both input populations.

    The historical official-PID CSV predates the aggregate field, whereas the
    optimized CSV contains it.  Recomputing from the underlying raw safety and
    completion fields prevents a missing derived column from changing the
    reported pass rate.
    """
    materialized = []
    for row in rows:
        result = dict(row)
        passed, reasons = hard_gate_result(result)
        declared = result.get("hard_gate_passed")
        if str(declared or "").strip() and truth(declared) != passed:
            raise ValueError(
                "{} hard-gate mismatch for {}".format(
                    population_label, result.get("evaluation_case_id", "")
                )
            )
        result["hard_gate_passed"] = passed
        result["hard_gate_reasons"] = ";".join(reasons)
        materialized.append(result)
    return materialized


def _expected_speed_profile(row, controller):
    """Return the declared speed-planner profile for a valid paired row.

    Controller-only cases deliberately use the same global profile.  The two
    system layers evaluate each controller with its own declared profile; this
    is a system-package comparison, not a pure controller-law comparison.
    """
    layer = row.get("layer", "")
    if layer == "controller_only":
        return "global"
    if layer in SYSTEM_LAYERS:
        return "controller:{}".format(controller)
    raise ValueError("Unexpected evaluation layer: {!r}".format(layer))


def validate_populations(official_rows, optimized_rows):
    if len(official_rows) != 90:
        raise ValueError("Official CARLA PID full matrix must contain 90 rows, got {}.".format(len(official_rows)))
    if len(optimized_rows) != 270:
        raise ValueError("Optimized full matrix must contain 270 rows, got {}.".format(len(optimized_rows)))

    official = _case_index(official_rows, expected_controller="pid")
    optimized = _case_index(optimized_rows)
    official_cases = {case_id for case_id, _ in official}
    optimized_cases = defaultdict(set)
    for case_id, controller in optimized:
        optimized_cases[case_id].add(controller)
    if set(optimized_cases) != official_cases:
        raise ValueError("Official and optimized case-id populations differ.")
    invalid_triplets = [case_id for case_id, names in optimized_cases.items() if names != set(CONTROLLERS)]
    if invalid_triplets:
        raise ValueError("Optimized cases are not complete PID/LQR/MPC triplets: {}".format(invalid_triplets[:5]))

    mismatches = []
    for case_id in sorted(official_cases):
        baseline = official[(case_id, "pid")]
        if baseline.get("status") != "ok":
            mismatches.append((case_id, "official_status", baseline.get("status"), "ok"))
        triplet = [optimized[(case_id, controller)] for controller in CONTROLLERS]
        for candidate in triplet:
            if candidate.get("status") != "ok":
                mismatches.append((case_id, candidate.get("controller"), candidate.get("status"), "ok"))
            for field in SPEED_PROFILE_INVARIANTS:
                if str(baseline.get(field, "")).strip() != str(candidate.get(field, "")).strip():
                    mismatches.append((case_id, field, baseline.get(field), candidate.get(field)))
            expected_baseline_profile = _expected_speed_profile(baseline, "pid")
            expected_candidate_profile = _expected_speed_profile(candidate, candidate.get("controller", ""))
            if str(baseline.get("speed_planner_limit_profile", "")).strip() != expected_baseline_profile:
                mismatches.append((case_id, "official_speed_planner_limit_profile", baseline.get("speed_planner_limit_profile"), expected_baseline_profile))
            if str(candidate.get("speed_planner_limit_profile", "")).strip() != expected_candidate_profile:
                mismatches.append((case_id, "optimized_speed_planner_limit_profile", candidate.get("speed_planner_limit_profile"), expected_candidate_profile))
        hashes = {row.get("trajectory_hash", "") for row in [baseline] + triplet}
        if len(hashes) != 1 or not next(iter(hashes), ""):
            mismatches.append((case_id, "trajectory_hash_set", sorted(hashes), "one non-empty hash"))
    if mismatches:
        raise ValueError("Paired-condition mismatch: {}".format(mismatches[:8]))
    return official, optimized


def _pairs(official, optimized, controller, layer):
    keys = sorted(case_id for case_id, name in official if name == "pid")
    result = []
    for case_id in keys:
        baseline = official[(case_id, "pid")]
        if layer != "all" and baseline.get("layer") != layer:
            continue
        result.append((baseline, optimized[(case_id, controller)]))
    return result


def build_official_comparison(official, optimized, iterations):
    metric_rows = []
    rate_rows = []
    metrics = list(LOWER_IS_BETTER.items()) + list(HIGHER_IS_BETTER.items())
    for controller in CONTROLLERS:
        for layer in LAYERS:
            pairs = _pairs(official, optimized, controller, layer)
            for metric, label in metrics:
                lower = metric in LOWER_IS_BETTER
                n, baseline_mean, candidate_mean, point, interval = bootstrap_metric(
                    pairs, metric, lower, iterations
                )
                if n is None:
                    continue
                metric_rows.append({
                    "candidate": controller,
                    "reference": "carla_0.9.14_official_pid",
                    "layer": layer,
                    "metric": metric,
                    "metric_label": label,
                    "direction": "lower_is_better" if lower else "higher_is_better",
                    "paired_laps": n,
                    "official_mean": baseline_mean,
                    "optimized_mean": candidate_mean,
                    "absolute_change": candidate_mean - baseline_mean,
                    "improvement_pct": point,
                    "bootstrap_ci95_low_pct": interval[0],
                    "bootstrap_ci95_high_pct": interval[1],
                })
            for field, label in PASS_FIELDS.items():
                baseline_rate, candidate_rate, point, interval = bootstrap_rate(pairs, field, iterations)
                rate_rows.append({
                    "candidate": controller,
                    "reference": "carla_0.9.14_official_pid",
                    "layer": layer,
                    "rate": field,
                    "rate_label": label,
                    "paired_laps": len(pairs),
                    "official_rate_pct": baseline_rate,
                    "optimized_rate_pct": candidate_rate,
                    "change_pp": point,
                    "bootstrap_ci95_low_pp": interval[0],
                    "bootstrap_ci95_high_pp": interval[1],
                })
    return metric_rows, rate_rows


def _optimized_pairs(optimized, candidate, reference, layer):
    case_ids = sorted({case_id for case_id, _ in optimized})
    result = []
    for case_id in case_ids:
        first = optimized[(case_id, reference)]
        if layer != "all" and first.get("layer") != layer:
            continue
        result.append((first, optimized[(case_id, candidate)]))
    return result


def build_optimized_pairwise(optimized, iterations):
    rows = []
    metrics = list(LOWER_IS_BETTER.items()) + list(HIGHER_IS_BETTER.items())
    for candidate, reference in PAIRWISE:
        for layer in LAYERS:
            pairs = _optimized_pairs(optimized, candidate, reference, layer)
            for metric, label in metrics:
                lower = metric in LOWER_IS_BETTER
                n, reference_mean, candidate_mean, point, interval = bootstrap_metric(
                    pairs, metric, lower, iterations
                )
                if n is None:
                    continue
                rows.append({
                    "candidate": candidate,
                    "reference": reference,
                    "layer": layer,
                    "metric": metric,
                    "metric_label": label,
                    "direction": "lower_is_better" if lower else "higher_is_better",
                    "paired_laps": n,
                    "reference_mean": reference_mean,
                    "candidate_mean": candidate_mean,
                    "improvement_pct": point,
                    "bootstrap_ci95_low_pct": interval[0],
                    "bootstrap_ci95_high_pct": interval[1],
                })
    return rows


def build_absolute_summary(official_rows, optimized_rows):
    populations = [("carla_0.9.14_official_pid", official_rows)]
    populations.extend((controller, [row for row in optimized_rows if row.get("controller") == controller]) for controller in CONTROLLERS)
    rows = []
    for name, population in populations:
        for layer in LAYERS:
            selected = population if layer == "all" else [row for row in population if row.get("layer") == layer]
            row = {"controller": name, "layer": layer, "laps": len(selected)}
            for metric in KEY_METRICS:
                values = [number(item.get(metric)) for item in selected]
                row[metric] = mean(value for value in values if value is not None)
            for field in PASS_FIELDS:
                available = [item for item in selected if str(item.get(field, "")).strip()]
                row[field + "_rate_pct"] = mean(100.0 if truth(item.get(field)) else 0.0 for item in available)
            rows.append(row)
    return rows


def _lookup(rows, candidate, layer, metric):
    return next(row for row in rows if row["candidate"] == candidate and row["layer"] == layer and row["metric"] == metric)


def _rate_lookup(rows, candidate, layer, rate):
    return next(row for row in rows if row["candidate"] == candidate and row["layer"] == layer and row["rate"] == rate)


def _fmt(value, digits=3):
    return "N/A" if value is None else ("{:.%df}" % digits).format(value)


def _signed(value, digits=1, suffix="%"):
    return "N/A" if value is None else ("{:+.%df}{}" % digits).format(value, suffix)


def render_report(metric_rows, rate_rows, pairwise_rows, absolute_rows, validation):
    lines = [
        "# CARLA 0.9.14 官方 PID 基准与三种优化控制器完整比较",
        "",
        "## 比较口径",
        "",
        "基准是 CARLA 0.9.14 `LocalPlanner` 实际使用的官方 `VehiclePIDController`。CARLA 0.9.14 没有官方 LQR 或 MPC，因此本报告用同一个官方 PID 作为三种优化控制器的统一参考。正改善率表示误差、振荡或耗时下降；通过率采用百分点变化。所有结论来自 90 个严格配对 case 和 5,000 次配对 bootstrap。",
        "",
        "## 相对官方 PID 的总体结果",
        "",
        "| 优化控制器 | 横向 IAE | 最大横向误差 | 航向 IAE | 速度 RMS | 横向 jerk P95 | P95 耗时 | 硬门限 | 稳定性 | UN R79参考 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for controller in CONTROLLERS:
        values = [_lookup(metric_rows, controller, "all", metric)["improvement_pct"] for metric in (
            "normalized_iae_e_y_m", "max_abs_e_y", "normalized_iae_e_psi_deg",
            "rms_speed_error_mps", "p95_abs_lateral_jerk", "controller_runtime_p95_ms",
        )]
        rates = [_rate_lookup(rate_rows, controller, "all", field)["change_pp"] for field in PASS_FIELDS]
        lines.append("| {} | {} | {} | {} | {} | {} | {} | {} pp | {} pp | {} pp |".format(
            controller.upper(), *[_signed(value) for value in values], *[_signed(value, suffix="") for value in rates]
        ))

    lines.extend(["", "## 三种优化控制器绝对值", "",
                  "| 控制器 | 横向 IAE (m) | 最大横向误差 (m) | 航向 IAE (deg) | 速度 RMS (m/s) | 反转 (/10s) | jerk P95 | P95 耗时 (ms) | 硬门限 |",
                  "|---|---:|---:|---:|---:|---:|---:|---:|---:|"])
    overall = {row["controller"]: row for row in absolute_rows if row["layer"] == "all"}
    for controller in CONTROLLERS:
        row = overall[controller]
        lines.append("| {} | {} | {} | {} | {} | {} | {} | {} | {:.1f}% |".format(
            controller.upper(), _fmt(row["normalized_iae_e_y_m"]), _fmt(row["max_abs_e_y"]),
            _fmt(row["normalized_iae_e_psi_deg"]), _fmt(row["rms_speed_error_mps"]),
            _fmt(row["significant_steer_reversals_per_10s"]), _fmt(row["p95_abs_lateral_jerk"]),
            _fmt(row["controller_runtime_p95_ms"]), row["hard_gate_passed_rate_pct"],
        ))

    lines.extend(["", "## 三种优化控制器两两比较", "",
                  "下表中正值表示候选控制器优于参考控制器；不使用不透明综合分数。",
                  "", "| 候选 vs 参考 | 横向 IAE | 航向 IAE | 速度 RMS | 反转 | jerk P95 | P95 耗时 |",
                  "|---|---:|---:|---:|---:|---:|---:|"])
    for candidate, reference in PAIRWISE:
        values = [next(row for row in pairwise_rows if row["candidate"] == candidate and row["reference"] == reference and row["layer"] == "all" and row["metric"] == metric)["improvement_pct"] for metric in (
            "normalized_iae_e_y_m", "normalized_iae_e_psi_deg", "rms_speed_error_mps",
            "significant_steer_reversals_per_10s", "p95_abs_lateral_jerk", "controller_runtime_p95_ms",
        )]
        lines.append("| {} vs {} | {} | {} | {} | {} | {} | {} |".format(candidate.upper(), reference.upper(), *[_signed(value) for value in values]))

    lines.extend(["", "## 分层横向 IAE：相对官方 PID", "",
                  "| 优化控制器 | controller-only | integrated | integrated-stress |",
                  "|---|---:|---:|---:|"])
    for controller in CONTROLLERS:
        values = [_lookup(metric_rows, controller, layer, "normalized_iae_e_y_m")["improvement_pct"] for layer in LAYERS[1:]]
        lines.append("| {} | {} | {} | {} |".format(controller.upper(), *[_signed(value) for value in values]))

    lines.extend(["", "## 审计与限制", "",
                  "- 官方基准：CARLA 0.9.14 `VehiclePIDController`，采用 `LocalPlanner` 默认参数；不是本项目 PID 的旧版本。",
                  "- 官方基准 90/90 行、优化控制器 270/270 行全部执行成功；case、地图、路线哈希、种子、车辆扰动和感知扰动严格匹配。",
                  "- controller-only 层隔离控制律；integrated 与 integrated-stress 层允许各优化控制器使用其对应速度规划限制，因此表示完整系统效果。",
                  "- CARLA 0.9.14 没有官方 LQR/MPC，不能报告“官方 LQR/MPC 提升率”。",
                  "- 源码 SHA-256：官方控制器 `{}`；优化 PID `{}`；LQR `{}`；MPC `{}`。".format(
                      validation["source_hashes"]["carla_official_controller"],
                      validation["source_hashes"]["optimized_pid"],
                      validation["source_hashes"]["optimized_lqr"],
                      validation["source_hashes"]["optimized_mpc"],
                  ), ""])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--official", required=True)
    parser.add_argument("--optimized", required=True)
    parser.add_argument("--project-root", required=True)
    parser.add_argument("--carla-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--bootstrap-iterations", type=int, default=5000)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    official_rows = with_recomputed_hard_gates(read_rows(args.official), "official")
    optimized_rows = with_recomputed_hard_gates(read_rows(args.optimized), "optimized")
    official, optimized = validate_populations(official_rows, optimized_rows)
    metric_rows, rate_rows = build_official_comparison(official, optimized, args.bootstrap_iterations)
    pairwise_rows = build_optimized_pairwise(optimized, args.bootstrap_iterations)
    absolute_rows = build_absolute_summary(official_rows, optimized_rows)

    source_hashes = {
        "carla_official_controller": file_sha256(os.path.join(args.carla_root, "agents", "navigation", "controller.py")),
        "carla_official_local_planner": file_sha256(os.path.join(args.carla_root, "agents", "navigation", "local_planner.py")),
        "optimized_pid": file_sha256(os.path.join(args.project_root, "control", "pid_controller.py")),
        "optimized_lqr": file_sha256(os.path.join(args.project_root, "control", "lqr_controller.py")),
        "optimized_mpc": file_sha256(os.path.join(args.project_root, "control", "mpc_controller.py")),
    }
    validation = {
        "status": "ready_to_share",
        "official_rows": len(official_rows),
        "optimized_rows": len(optimized_rows),
        "paired_cases_per_candidate": 90,
        "condition_mismatches": 0,
        "bootstrap_iterations": args.bootstrap_iterations,
        "official_source": os.path.abspath(args.official),
        "optimized_source": os.path.abspath(args.optimized),
        "official_definition": "CARLA 0.9.14 LocalPlanner default VehiclePIDController",
        "speed_profile_comparison_scope": {
            "controller_only": "strictly matched global speed-planner profile",
            "integrated_and_stress": "controller-specific declared speed-planner profiles; system-package comparison",
        },
        "source_hashes": source_hashes,
    }
    write_rows(os.path.join(args.output_dir, "official_benchmark_metric_comparison.csv"), metric_rows)
    write_rows(os.path.join(args.output_dir, "official_benchmark_pass_rate_comparison.csv"), rate_rows)
    write_rows(os.path.join(args.output_dir, "optimized_pairwise_metric_comparison.csv"), pairwise_rows)
    write_rows(os.path.join(args.output_dir, "absolute_summary.csv"), absolute_rows)
    with open(os.path.join(args.output_dir, "validation.json"), "w", encoding="utf-8") as handle:
        json.dump(validation, handle, ensure_ascii=False, indent=2)
    report = render_report(metric_rows, rate_rows, pairwise_rows, absolute_rows, validation)
    with open(os.path.join(args.output_dir, "comparison_report.md"), "w", encoding="utf-8") as handle:
        handle.write(report)
    print(json.dumps(validation, ensure_ascii=False))


if __name__ == "__main__":
    main()
