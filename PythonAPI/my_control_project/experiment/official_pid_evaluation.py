"""Audit and summarize the CARLA 0.9.14 official PID evaluation matrix."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import random
from collections import Counter, defaultdict
from statistics import mean, median


EXPECTED_LAYERS = {
    "controller_only": 30,
    "integrated": 30,
    "integrated_stress": 30,
}

METRICS = (
    ("normalized_iae_e_y_m", "横向时间归一化 IAE", "m"),
    ("max_abs_e_y", "最大绝对横向误差", "m"),
    ("normalized_iae_e_psi_deg", "航向时间归一化 IAE", "deg"),
    ("max_abs_e_psi_deg", "最大绝对航向误差", "deg"),
    ("rms_speed_error_mps", "速度 RMS 误差", "m/s"),
    ("mean_abs_steer_delta", "平均转向变化量", "normalized steer/step"),
    ("significant_steer_reversals_per_10s", "显著转向反转", "/10 s"),
    ("p95_abs_lateral_accel", "横向加速度 P95", "m/s^2"),
    ("p95_abs_lateral_jerk", "横向 jerk P95", "m/s^3"),
    ("p95_abs_longitudinal_jerk", "纵向 jerk P95", "m/s^3"),
    ("throttle_brake_switches_per_10s", "油门/制动切换", "/10 s"),
    ("steer_spectrum_stability_index", "转向稳定频带指标", "index"),
    ("steer_spectrum_comfort_index", "转向舒适频带指标", "index"),
    ("controller_runtime_p95_ms", "控制器运行时间 P95", "ms"),
    ("controller_runtime_deadline_miss_pct", "控制周期超时比例", "%"),
    ("route_completion_pct", "路线完成率", "%"),
)

PASS_FIELDS = (
    ("hard_gate_passed", "硬门槛"),
    ("stability_passed", "稳定性检查"),
    ("un_r79_reference_passed", "UN R79 参考检查"),
    ("reference_validation_passed", "参考轨迹验证"),
)


def _read_csv(path):
    with open(path, "r", encoding="utf-8-sig", newline="") as handle:
        return [dict(row) for row in csv.DictReader(handle)]


def _write_csv(path, rows):
    rows = list(rows)
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with open(path, "w", encoding="utf-8-sig", newline="") as handle:
        if not fields:
            return
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _number(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _truth(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "passed", "ok", "success"}


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _percentile(values, probability):
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * probability
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def _bootstrap_mean_ci(values, iterations, seed):
    if not values:
        return None, None
    rng = random.Random(seed)
    samples = []
    count = len(values)
    for _ in range(iterations):
        samples.append(mean(values[rng.randrange(count)] for _ in range(count)))
    return _percentile(samples, 0.025), _percentile(samples, 0.975)


def _wilson(successes, total, z=1.959963984540054):
    if total <= 0:
        return 0.0, 0.0
    proportion = successes / float(total)
    denominator = 1.0 + z * z / total
    center = (proportion + z * z / (2.0 * total)) / denominator
    margin = z * math.sqrt(
        (proportion * (1.0 - proportion) + z * z / (4.0 * total)) / total
    ) / denominator
    return 100.0 * (center - margin), 100.0 * (center + margin)


def _hard_gate(row):
    reasons = []
    if str(row.get("status", "")).strip().lower() != "ok":
        reasons.append("experiment_failed")
    if (_number(row.get("collision_count")) or 0.0) > 0.0:
        reasons.append("collision")
    if (_number(row.get("lane_boundary_violation_count")) or 0.0) > 0.0:
        reasons.append("lane_boundary_violation")
    completion = _number(row.get("route_completion_pct"))
    if completion is None or completion < 99.0:
        reasons.append("route_incomplete")
    if str(row.get("tracking_failure_reason", "")).strip():
        reasons.append("tracking_failure")
    if not _truth(row.get("spectral_metrics_valid", True)):
        reasons.append("spectral_metric_invalid")
    return not reasons, reasons


def _audit_configs(rows, official_controller_path):
    configs = []
    missing = []
    for row in rows:
        path = os.path.join(row.get("run_dir", ""), "run_config.json")
        if not os.path.isfile(path):
            missing.append(path)
            continue
        with open(path, "r", encoding="utf-8-sig") as handle:
            configs.append(json.load(handle))
    expected_hash = _sha256(official_controller_path)
    labels = set()
    implementations = set()
    source_labels = set()
    recorded_hashes = set()
    parameter_sets = set()
    for config in configs:
        implementations.add(config.get("controller_implementation"))
        audit = config.get("authoritative_baseline_audit", {})
        labels.add(audit.get("label"))
        pid = audit.get("pid", {})
        source_labels.add(pid.get("source"))
        recorded_hashes.add(pid.get("official_implementation", {}).get("sha256"))
        parameter_sets.add(json.dumps({
            "lateral": pid.get("lateral"),
            "longitudinal": pid.get("longitudinal"),
            "max_throttle": pid.get("max_throttle"),
            "max_brake": pid.get("max_brake"),
            "max_steering": pid.get("max_steering"),
            "offset": pid.get("offset"),
        }, sort_keys=True))
    return {
        "config_count": len(configs),
        "missing_config_count": len(missing),
        "controller_implementations": sorted(str(value) for value in implementations),
        "audit_labels": sorted(str(value) for value in labels),
        "source_labels": sorted(str(value) for value in source_labels),
        "recorded_implementation_hashes": sorted(str(value) for value in recorded_hashes),
        "actual_implementation_hash": expected_hash,
        "official_source_path": os.path.abspath(official_controller_path),
        "parameter_set_count": len(parameter_sets),
        "passed": (
            len(configs) == len(rows)
            and not missing
            and implementations == {"authoritative_baseline"}
            and labels == {"authoritative_baseline_v1"}
            and source_labels == {'"CARLA_0.9.14_VehiclePIDController"'}
            and recorded_hashes == {expected_hash}
            and len(parameter_sets) == 1
        ),
    }


def validate(rows, official_controller_path):
    errors = []
    warnings = []
    case_ids = [row.get("evaluation_case_id", "") for row in rows]
    layer_counts = Counter(row.get("layer", "") for row in rows)
    scenario_counts = Counter(row.get("scenario", "") for row in rows)
    if len(rows) != 90:
        errors.append("expected 90 rows, got {}".format(len(rows)))
    if len(set(case_ids)) != len(case_ids) or "" in case_ids:
        errors.append("case IDs are missing or duplicated")
    if set(row.get("controller", "") for row in rows) != {"pid"}:
        errors.append("population is not official PID-only")
    if any(str(row.get("status", "")).lower() != "ok" for row in rows):
        errors.append("one or more experiments did not finish with status=ok")
    if dict(layer_counts) != EXPECTED_LAYERS:
        errors.append("layer counts differ from 30/30/30: {}".format(dict(layer_counts)))
    if any(not row.get("trajectory_hash", "") for row in rows):
        errors.append("one or more trajectory hashes are blank")
    if any(not _truth(row.get("reference_validation_passed")) for row in rows):
        errors.append("one or more frozen reference trajectories failed validation")
    critical_fields = [metric for metric, _, _ in METRICS]
    missing_metrics = {
        field: sum(_number(row.get(field)) is None for row in rows)
        for field in critical_fields
    }
    missing_metrics = {key: value for key, value in missing_metrics.items() if value}
    if missing_metrics:
        errors.append("critical metrics contain missing/non-finite values: {}".format(missing_metrics))
    source_audit = _audit_configs(rows, official_controller_path)
    if not source_audit["passed"]:
        errors.append("official implementation/config audit failed")
    route_geometry_warnings = []
    minimum_turn_by_shape = {"gentle_curve": 10.0, "curvy": 30.0, "s_curve": 45.0}
    for row in rows:
        requested = row.get("requested_route_shape", "")
        minimum_turn = minimum_turn_by_shape.get(requested)
        total_turn = _number(row.get("route_total_abs_turn_deg"))
        if minimum_turn is not None and (total_turn is None or total_turn < minimum_turn):
            route_geometry_warnings.append({
                "evaluation_case_id": row.get("evaluation_case_id"),
                "scenario": row.get("scenario"),
                "layer": row.get("layer"),
                "requested_route_shape": requested,
                "route_total_abs_turn_deg": total_turn,
                "route_mean_abs_curvature": _number(row.get("route_mean_abs_curvature")),
                "route_max_abs_curvature": _number(row.get("route_max_abs_curvature")),
                "scene_s_curve_sign_changes": _number(row.get("scene_s_curve_sign_changes")),
                "warning": "requested curved route has insufficient realized total turn",
            })
    if route_geometry_warnings:
        warnings.append(
            "{} cases have requested curve labels but insufficient realized route turning; retain actual geometry in interpretation".format(
                len(route_geometry_warnings)
            )
        )
    return {
        "ready_to_share": not errors,
        "row_count": len(rows),
        "unique_case_count": len(set(case_ids)),
        "layer_counts": dict(layer_counts),
        "scenario_counts": dict(scenario_counts),
        "missing_metrics": missing_metrics,
        "errors": errors,
        "warnings": warnings,
        "route_geometry_warnings": route_geometry_warnings,
        "source_audit": source_audit,
    }


def _add_gate_fields(rows):
    output = []
    for source in rows:
        row = dict(source)
        passed, reasons = _hard_gate(row)
        row["hard_gate_passed"] = passed
        row["hard_gate_reasons"] = ";".join(reasons)
        output.append(row)
    return output


def _metric_summaries(rows, group_type, group_name, iterations):
    output = []
    for metric, label, unit in METRICS:
        values = [value for value in (_number(row.get(metric)) for row in rows) if value is not None]
        low, high = _bootstrap_mean_ci(
            values, iterations, "{}:{}:{}".format(group_type, group_name, metric)
        )
        output.append({
            "group_type": group_type,
            "group": group_name,
            "metric": metric,
            "metric_label": label,
            "unit": unit,
            "n": len(values),
            "mean": mean(values) if values else None,
            "median": median(values) if values else None,
            "p90": _percentile(values, 0.90),
            "p95": _percentile(values, 0.95),
            "maximum": max(values) if values else None,
            "bootstrap_mean_ci95_low": low,
            "bootstrap_mean_ci95_high": high,
        })
    return output


def _pass_summaries(rows, group_type, group_name):
    output = []
    for field, label in PASS_FIELDS:
        successes = sum(_truth(row.get(field)) for row in rows)
        low, high = _wilson(successes, len(rows))
        output.append({
            "group_type": group_type,
            "group": group_name,
            "check": field,
            "check_label": label,
            "successes": successes,
            "total": len(rows),
            "pass_rate_pct": 100.0 * successes / len(rows) if rows else 0.0,
            "wilson_ci95_low_pct": low,
            "wilson_ci95_high_pct": high,
        })
    return output


def build_outputs(rows, iterations):
    metric_rows = []
    pass_rows = []
    groups = [("overall", "all", rows)]
    for layer in EXPECTED_LAYERS:
        groups.append(("layer", layer, [row for row in rows if row.get("layer") == layer]))
    for scenario in sorted({row.get("scenario", "") for row in rows}):
        groups.append(("scenario", scenario, [row for row in rows if row.get("scenario") == scenario]))
    for group_type, group_name, selected in groups:
        metric_rows.extend(_metric_summaries(selected, group_type, group_name, iterations))
        pass_rows.extend(_pass_summaries(selected, group_type, group_name))

    failure_reasons = Counter()
    for row in rows:
        for field in ("hard_gate_reasons", "stability_fail_reasons"):
            for reason in str(row.get(field, "")).replace(",", ";").split(";"):
                if reason.strip():
                    failure_reasons[(field, reason.strip())] += 1
    failure_rows = [
        {"source": source, "reason": reason, "case_count": count}
        for (source, reason), count in sorted(failure_reasons.items())
    ]
    worst = sorted(
        rows,
        key=lambda row: (
            _number(row.get("normalized_iae_e_y_m")) or -math.inf,
            _number(row.get("p95_abs_lateral_jerk")) or -math.inf,
        ),
        reverse=True,
    )[:10]
    worst_rows = [{
        "evaluation_case_id": row.get("evaluation_case_id"),
        "scenario": row.get("scenario"),
        "layer": row.get("layer"),
        "normalized_iae_e_y_m": row.get("normalized_iae_e_y_m"),
        "max_abs_e_y": row.get("max_abs_e_y"),
        "p95_abs_lateral_jerk": row.get("p95_abs_lateral_jerk"),
        "route_completion_pct": row.get("route_completion_pct"),
        "hard_gate_passed": row.get("hard_gate_passed"),
        "hard_gate_reasons": row.get("hard_gate_reasons"),
        "stability_fail_reasons": row.get("stability_fail_reasons"),
        "run_dir": row.get("run_dir"),
    } for row in worst]
    return metric_rows, pass_rows, failure_rows, worst_rows


def _lookup(rows, group_type, group, metric):
    return next(row for row in rows if row["group_type"] == group_type and row["group"] == group and row["metric"] == metric)


def _lookup_pass(rows, group_type, group, check):
    return next(row for row in rows if row["group_type"] == group_type and row["group"] == group and row["check"] == check)


def render_markdown(validation, metric_rows, pass_rows, failure_rows, worst_rows):
    lateral = _lookup(metric_rows, "overall", "all", "normalized_iae_e_y_m")
    heading = _lookup(metric_rows, "overall", "all", "normalized_iae_e_psi_deg")
    speed = _lookup(metric_rows, "overall", "all", "rms_speed_error_mps")
    runtime = _lookup(metric_rows, "overall", "all", "controller_runtime_p95_ms")
    gate = _lookup_pass(pass_rows, "overall", "all", "hard_gate_passed")
    lines = [
        "# CARLA 0.9.14 官方 PID 控制器表现评测",
        "",
        "## 技术摘要",
        "",
        "本报告仅描述 CARLA 0.9.14 官方 `VehiclePIDController` 在冻结路线矩阵中的表现，不包含正在升级的 PID/LQR/MPC，也不计算提升比例。",
        "90 个工况覆盖纯控制器、系统集成和集成压力三层；程序完成状态与硬门槛通过状态分开统计。",
        "",
        "- 数据状态：{}（{} 行，{} 个唯一工况）。".format("可共享" if validation["ready_to_share"] else "不可共享", validation["row_count"], validation["unique_case_count"]),
        "- 总体横向归一化 IAE：{:.4f} m（bootstrap 95% CI {:.4f}–{:.4f}）。".format(lateral["mean"], lateral["bootstrap_mean_ci95_low"], lateral["bootstrap_mean_ci95_high"]),
        "- 总体航向归一化 IAE：{:.4f} deg；速度 RMS 误差：{:.4f} m/s。".format(heading["mean"], speed["mean"]),
        "- 控制器逐圈 P95 运行时间的总体均值：{:.4f} ms。".format(runtime["mean"]),
        "- 硬门槛通过：{}/{}（{:.1f}%，Wilson 95% CI {:.1f}%–{:.1f}%）。".format(gate["successes"], gate["total"], gate["pass_rate_pct"], gate["wilson_ci95_low_pct"], gate["wilson_ci95_high_pct"]),
        "- 路线几何警告：{} 个标称曲线路况的实际总转角低于预设检查阈值；这些工况保留，但按实际几何解释。".format(len(validation.get("route_geometry_warnings", []))),
        "",
        "## 分层结果",
        "",
        "| 评测层 | 横向 IAE 均值 (m) | 航向 IAE 均值 (deg) | 速度 RMS (m/s) | 横向 jerk P95 | P95 耗时 (ms) | 硬门槛 |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for layer in EXPECTED_LAYERS:
        lat = _lookup(metric_rows, "layer", layer, "normalized_iae_e_y_m")
        head = _lookup(metric_rows, "layer", layer, "normalized_iae_e_psi_deg")
        spd = _lookup(metric_rows, "layer", layer, "rms_speed_error_mps")
        jerk = _lookup(metric_rows, "layer", layer, "p95_abs_lateral_jerk")
        run = _lookup(metric_rows, "layer", layer, "controller_runtime_p95_ms")
        passed = _lookup_pass(pass_rows, "layer", layer, "hard_gate_passed")
        lines.append("| {} | {:.4f} | {:.4f} | {:.4f} | {:.4f} | {:.4f} | {}/{} ({:.1f}%) |".format(
            layer, lat["mean"], head["mean"], spd["mean"], jerk["mean"], run["mean"],
            passed["successes"], passed["total"], passed["pass_rate_pct"]
        ))
    lines.extend([
        "",
        "## 失败与最差工况",
        "",
    ])
    if failure_rows:
        for item in failure_rows:
            lines.append("- `{}` / `{}`：{} 个工况。".format(item["source"], item["reason"], item["case_count"]))
    else:
        lines.append("- 未记录硬门槛或稳定性失败原因。")
    lines.extend([
        "",
        "最差横向 IAE 工况：",
        "",
        "| 工况 | 层 | 场景 | 横向 IAE (m) | 最大横向误差 (m) | jerk P95 | 完成率 |",
        "|---|---|---|---:|---:|---:|---:|",
    ])
    for item in worst_rows[:5]:
        lines.append("| {} | {} | {} | {} | {} | {} | {}% |".format(
            item["evaluation_case_id"], item["layer"], item["scenario"],
            item["normalized_iae_e_y_m"], item["max_abs_e_y"],
            item["p95_abs_lateral_jerk"], item["route_completion_pct"]
        ))
    lines.extend([
        "",
        "## 口径与限制",
        "",
        "- 官方对象是 CARLA 0.9.14 `agents.navigation.controller.VehiclePIDController`，参数来自同版本 `LocalPlanner` 默认配置；源码哈希已逐圈审计。",
        "- 纯控制器层关闭项目速度规划；系统层和压力层保留项目实验框架的目标点与速度规划。因此本报告评估官方底层 PID 在该框架下的表现，不等于完整评测 CARLA 官方路径规划器。",
        "- 硬门槛为：执行成功、无碰撞、无越界、路线完成率至少 99%、无跟踪失败且频谱指标有效。`UN R79` 仅为参考检查，不构成法规认证。",
        "- 均值区间为工况级非参数 bootstrap；通过率区间为 Wilson 95% 区间。结果是对本路线、车辆、频率和扰动矩阵的描述性证据，不能外推为真实道路性能。",
        "",
        "## 下一步",
        "",
        "待三种控制方式升级冻结后，使用同一 90 工况清单、同一轨迹哈希和相同随机种子执行配对测试，再计算相对官方 PID 的提升比例与三者间差异。",
    ])
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="Completed raw_lap_results.csv or run_status.csv")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--official-controller", required=True)
    parser.add_argument("--bootstrap-iterations", type=int, default=5000)
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    rows = _read_csv(args.input)
    validation = validate(rows, args.official_controller)
    if not validation["ready_to_share"]:
        with open(os.path.join(args.output_dir, "validation.json"), "w", encoding="utf-8") as handle:
            json.dump(validation, handle, ensure_ascii=False, indent=2)
        raise ValueError("Official PID dataset failed validation: {}".format(validation["errors"]))
    rows = _add_gate_fields(rows)
    metric_rows, pass_rows, failure_rows, worst_rows = build_outputs(rows, args.bootstrap_iterations)
    _write_csv(os.path.join(args.output_dir, "official_raw_validated.csv"), rows)
    _write_csv(os.path.join(args.output_dir, "official_metric_summary.csv"), metric_rows)
    _write_csv(os.path.join(args.output_dir, "official_pass_summary.csv"), pass_rows)
    _write_csv(os.path.join(args.output_dir, "official_failure_reasons.csv"), failure_rows)
    _write_csv(os.path.join(args.output_dir, "official_worst_cases.csv"), worst_rows)
    _write_csv(os.path.join(args.output_dir, "official_route_geometry_warnings.csv"), validation.get("route_geometry_warnings", []))
    with open(os.path.join(args.output_dir, "validation.json"), "w", encoding="utf-8") as handle:
        json.dump(validation, handle, ensure_ascii=False, indent=2)
    with open(os.path.join(args.output_dir, "official_pid_report.md"), "w", encoding="utf-8") as handle:
        handle.write(render_markdown(validation, metric_rows, pass_rows, failure_rows, worst_rows))
    print(json.dumps({"status": "ready_to_share", "rows": len(rows), "output_dir": os.path.abspath(args.output_dir)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
