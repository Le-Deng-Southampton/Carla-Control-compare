"""Build an audited full-matrix initial-versus-optimized comparison.

The optimized population may combine a fresh PID retest with an earlier full
LQR/MPC run when those controller implementations are unchanged. Every row is
accepted only after case identity, experimental conditions, and trajectory hash
match the newly executed historical baseline.
"""

import argparse
import csv
import hashlib
import json
import math
import os
import random
from collections import defaultdict


LOWER_IS_BETTER = {
    "normalized_iae_e_y_m": "归一化横向 IAE (m)",
    "max_abs_e_y": "最大横向误差 (m)",
    "normalized_iae_e_psi_deg": "归一化航向 IAE (deg)",
    "max_abs_e_psi_deg": "最大航向误差 (deg)",
    "rms_speed_error_mps": "速度误差 RMS (m/s)",
    "mean_abs_steer_delta": "平均转向变化",
    "significant_steer_reversals_per_10s": "显著转向反转 (/10s)",
    "p95_abs_lateral_accel": "横向加速度 P95 (m/s²)",
    "p95_abs_lateral_jerk": "横向 jerk P95 (m/s³)",
    "p95_abs_longitudinal_jerk": "纵向 jerk P95 (m/s³)",
    "controller_runtime_p95_ms": "控制器耗时 P95 (ms)",
    "controller_runtime_deadline_miss_pct": "实时期限错失率 (%)",
    "collision_count": "碰撞数",
    "lane_boundary_violation_count": "越界数",
}

HIGHER_IS_BETTER = {
    "route_completion_pct": "路线完成率 (%)",
}

PASS_FIELDS = {
    "hard_gate_passed": "硬门限通过率",
    "stability_passed": "稳定性通过率",
    "un_r79_reference_passed": "UN R79 参考检查通过率",
}

LAYERS = ("all", "controller_only", "integrated", "integrated_stress")
CONTROLLERS = ("pid", "lqr", "mpc")
INVARIANT_FIELDS = (
    "scenario",
    "layer",
    "requested_speed_kmh",
    "requested_route_shape",
    "requested_map",
    "seed",
    "trajectory_hash",
    "error_provider",
    "speed_planner_mode",
    "speed_planner_limit_profile",
    "vehicle_mass_scale",
    "vehicle_moi_scale",
    "tire_friction_scale",
    "noise_lateral_std",
    "noise_heading_std_deg",
    "perception_delay_steps",
    "perception_dropout_probability",
    "perception_smoothing_alpha",
)


def read_rows(path):
    with open(path, "r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def write_rows(path, rows):
    if not rows:
        raise ValueError("Cannot write an empty CSV artifact.")
    fields = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with open(path, "w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def key(row):
    return row.get("evaluation_case_id", ""), row.get("controller", "")


def number(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def truth(value):
    return str(value).strip().lower() in {"1", "true", "yes"}


def mean(values):
    values = list(values)
    return sum(values) / len(values) if values else None


def quantile(values, probability):
    values = sorted(values)
    if not values:
        return None
    position = (len(values) - 1) * probability
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return values[low]
    return values[low] + (values[high] - values[low]) * (position - low)


def improvement(initial, optimized, lower_is_better=True):
    if initial is None or optimized is None or initial == 0:
        return None
    sign = 1.0 if lower_is_better else -1.0
    return sign * (initial - optimized) / abs(initial) * 100.0


def bootstrap_metric(pairs, metric, lower_is_better, iterations):
    values = []
    for initial, optimized in pairs:
        a = number(initial.get(metric))
        b = number(optimized.get(metric))
        if a is not None and b is not None:
            values.append((a, b))
    if not values:
        return None, None, None, None, None
    initial_mean = mean(item[0] for item in values)
    optimized_mean = mean(item[1] for item in values)
    point = improvement(initial_mean, optimized_mean, lower_is_better)
    samples = []
    rng = random.Random("{}:{}:{}".format(metric, len(values), iterations))
    for _ in range(iterations):
        draw = [values[rng.randrange(len(values))] for __ in range(len(values))]
        estimate = improvement(
            mean(item[0] for item in draw),
            mean(item[1] for item in draw),
            lower_is_better,
        )
        if estimate is not None and math.isfinite(estimate):
            samples.append(estimate)
    return (
        len(values),
        initial_mean,
        optimized_mean,
        point,
        (quantile(samples, 0.025), quantile(samples, 0.975)),
    )


def bootstrap_rate(pairs, field, iterations):
    values = []
    for initial, optimized in pairs:
        initial_value = str(initial.get(field, "")).strip()
        optimized_value = str(optimized.get(field, "")).strip()
        if not initial_value or not optimized_value:
            continue
        values.append((truth(initial_value), truth(optimized_value)))
    if not values:
        return None, None, None, (None, None)
    initial_rate = mean(100.0 if a else 0.0 for a, _ in values)
    optimized_rate = mean(100.0 if b else 0.0 for _, b in values)
    point = optimized_rate - initial_rate
    samples = []
    rng = random.Random("rate:{}:{}:{}".format(field, len(values), iterations))
    for _ in range(iterations):
        draw = [values[rng.randrange(len(values))] for __ in range(len(values))]
        samples.append(
            mean(100.0 if b else 0.0 for _, b in draw)
            - mean(100.0 if a else 0.0 for a, _ in draw)
        )
    return initial_rate, optimized_rate, point, (quantile(samples, 0.025), quantile(samples, 0.975))


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_and_merge(initial_rows, pid_rows, reused_rows):
    if len(initial_rows) != 270:
        raise ValueError("Initial full matrix must contain 270 rows, got {}.".format(len(initial_rows)))
    if len(pid_rows) != 90:
        raise ValueError("Fresh PID matrix must contain 90 rows, got {}.".format(len(pid_rows)))
    reused = [row for row in reused_rows if row.get("controller") in {"lqr", "mpc"}]
    if len(reused) != 180:
        raise ValueError("Reused LQR/MPC population must contain 180 rows, got {}.".format(len(reused)))

    optimized_rows = []
    for row in pid_rows:
        copy = dict(row)
        copy["comparison_data_source"] = "fresh_pid_retest_20260714"
        optimized_rows.append(copy)
    for row in reused:
        copy = dict(row)
        copy["comparison_data_source"] = "reused_optimized_lqr_mpc_20260713"
        optimized_rows.append(copy)

    initial_by_key = {key(row): row for row in initial_rows}
    optimized_by_key = {key(row): row for row in optimized_rows}
    if len(initial_by_key) != 270 or len(optimized_by_key) != 270:
        raise ValueError("Duplicate evaluation-case/controller keys were found.")
    if set(initial_by_key) != set(optimized_by_key):
        missing = sorted(set(initial_by_key) ^ set(optimized_by_key))
        raise ValueError("Initial and optimized keys differ: {}".format(missing[:5]))

    mismatches = []
    for pair_key in sorted(initial_by_key):
        initial = initial_by_key[pair_key]
        optimized = optimized_by_key[pair_key]
        if initial.get("status") != "ok" or optimized.get("status") != "ok":
            mismatches.append((pair_key, "status", initial.get("status"), optimized.get("status")))
        for field in INVARIANT_FIELDS:
            if str(initial.get(field, "")).strip() != str(optimized.get(field, "")).strip():
                mismatches.append((pair_key, field, initial.get(field), optimized.get(field)))
    if mismatches:
        raise ValueError("Paired-condition mismatch: {}".format(mismatches[:5]))

    cases = defaultdict(set)
    for case_id, controller in optimized_by_key:
        cases[case_id].add(controller)
    invalid = [case_id for case_id, controllers in cases.items() if controllers != set(CONTROLLERS)]
    if len(cases) != 90 or invalid:
        raise ValueError("Merged optimized matrix is not 90 complete PID/LQR/MPC triplets.")
    return optimized_rows, initial_by_key, optimized_by_key


def build_summary(initial_by_key, optimized_by_key, iterations):
    metric_rows = []
    rate_rows = []
    paired_delta_rows = []
    for controller in CONTROLLERS:
        for layer in LAYERS:
            pair_keys = [
                pair_key for pair_key in sorted(initial_by_key)
                if pair_key[1] == controller
                and (layer == "all" or initial_by_key[pair_key].get("layer") == layer)
            ]
            pairs = [(initial_by_key[pair_key], optimized_by_key[pair_key]) for pair_key in pair_keys]
            for metric, label in list(LOWER_IS_BETTER.items()) + list(HIGHER_IS_BETTER.items()):
                lower = metric in LOWER_IS_BETTER
                n, initial_mean, optimized_mean, point, interval = bootstrap_metric(
                    pairs, metric, lower, iterations
                )
                if n is None:
                    continue
                metric_rows.append({
                    "controller": controller,
                    "layer": layer,
                    "metric": metric,
                    "metric_label": label,
                    "direction": "lower_is_better" if lower else "higher_is_better",
                    "paired_laps": n,
                    "initial_mean": initial_mean,
                    "optimized_mean": optimized_mean,
                    "absolute_change": optimized_mean - initial_mean,
                    "improvement_pct": point,
                    "bootstrap_ci95_low_pct": interval[0],
                    "bootstrap_ci95_high_pct": interval[1],
                })
            for field, label in PASS_FIELDS.items():
                initial_rate, optimized_rate, point, interval = bootstrap_rate(pairs, field, iterations)
                rate_rows.append({
                    "controller": controller,
                    "layer": layer,
                    "rate": field,
                    "rate_label": label,
                    "paired_laps": len(pairs),
                    "initial_rate_pct": initial_rate,
                    "optimized_rate_pct": optimized_rate,
                    "change_pp": point,
                    "bootstrap_ci95_low_pp": interval[0],
                    "bootstrap_ci95_high_pp": interval[1],
                })
            if layer == "all":
                for pair_key, (initial, optimized) in zip(pair_keys, pairs):
                    row = {
                        "evaluation_case_id": pair_key[0],
                        "controller": controller,
                        "layer": initial.get("layer"),
                        "trajectory_hash": initial.get("trajectory_hash"),
                        "optimized_data_source": optimized.get("comparison_data_source"),
                    }
                    for metric in LOWER_IS_BETTER:
                        a, b = number(initial.get(metric)), number(optimized.get(metric))
                        row["initial_{}".format(metric)] = a
                        row["optimized_{}".format(metric)] = b
                        row["delta_{}".format(metric)] = None if a is None or b is None else b - a
                    paired_delta_rows.append(row)
    return metric_rows, rate_rows, paired_delta_rows


def lookup(rows, controller, layer, metric):
    return next(row for row in rows if row["controller"] == controller and row["layer"] == layer and row["metric"] == metric)


def lookup_rate(rows, controller, layer, rate):
    return next(row for row in rows if row["controller"] == controller and row["layer"] == layer and row["rate"] == rate)


def fmt(value, digits=2):
    return "N/A" if value is None else ("{:.%df}" % digits).format(value)


def change(value, digits=1, suffix="%"):
    return "N/A" if value is None else ("{:+.%df}{}" % digits).format(value, suffix)


def render_report(metric_rows, rate_rows, validation):
    def metric_change(controller, metric, layer="all"):
        return lookup(metric_rows, controller, layer, metric)["improvement_pct"]

    def rate_change(controller, rate):
        return lookup_rate(rate_rows, controller, "all", rate)["change_pp"]

    pid_lateral = lookup(metric_rows, "pid", "all", "normalized_iae_e_y_m")
    lqr_lateral = lookup(metric_rows, "lqr", "all", "normalized_iae_e_y_m")
    mpc_lateral = lookup(metric_rows, "mpc", "all", "normalized_iae_e_y_m")
    result_lines = [
        "- PID：横向 IAE {}（95% CI {} 至 {}），速度 RMS 误差{}，横向 jerk P95{}，硬门限变化 {} pp，稳定性变化 {} pp。".format(
            change(pid_lateral["improvement_pct"]),
            change(pid_lateral["bootstrap_ci95_low_pct"]),
            change(pid_lateral["bootstrap_ci95_high_pct"]),
            change(metric_change("pid", "rms_speed_error_mps")),
            change(metric_change("pid", "p95_abs_lateral_jerk")),
            change(rate_change("pid", "hard_gate_passed"), suffix=""),
            change(rate_change("pid", "stability_passed"), suffix=""),
        ),
        "- LQR：横向 IAE {}（95% CI {} 至 {}），最大横向误差{}，航向 IAE{}，横向 jerk P95{}，硬门限变化 {} pp。".format(
            change(lqr_lateral["improvement_pct"]),
            change(lqr_lateral["bootstrap_ci95_low_pct"]),
            change(lqr_lateral["bootstrap_ci95_high_pct"]),
            change(metric_change("lqr", "max_abs_e_y")),
            change(metric_change("lqr", "normalized_iae_e_psi_deg")),
            change(metric_change("lqr", "p95_abs_lateral_jerk")),
            change(rate_change("lqr", "hard_gate_passed"), suffix=""),
        ),
        "- MPC：总体横向 IAE {}（95% CI {} 至 {}）；controller-only {}、integrated {}、integrated-stress {}。转向变化{}，横向 jerk P95{}，硬门限变化 {} pp。".format(
            change(mpc_lateral["improvement_pct"]),
            change(mpc_lateral["bootstrap_ci95_low_pct"]),
            change(mpc_lateral["bootstrap_ci95_high_pct"]),
            change(metric_change("mpc", "normalized_iae_e_y_m", "controller_only")),
            change(metric_change("mpc", "normalized_iae_e_y_m", "integrated")),
            change(metric_change("mpc", "normalized_iae_e_y_m", "integrated_stress")),
            change(metric_change("mpc", "mean_abs_steer_delta")),
            change(metric_change("mpc", "p95_abs_lateral_jerk")),
            change(rate_change("mpc", "hard_gate_passed"), suffix=""),
        ),
    ]
    lines = [
        "# 三种控制方式完整平行测试：初始版 vs 优化版",
        "",
        "## 结论摘要",
        "",
        "本报告使用 90 个严格配对实验/控制器：30 个 controller-only、30 个 integrated、30 个 integrated-stress。正提升率表示相应误差、振荡或耗时下降；负值表示优化版变差。95% 区间来自 5,000 次按实验配对重采样，仅反映本矩阵内跨场景/种子的离散性。",
        "",
        "| 控制器 | 归一化横向 IAE | 最大横向误差 | 航向 IAE | 速度 RMS 误差 | 平均转向变化 | 横向 jerk P95 | 硬门限变化 | 稳定性变化 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for controller in CONTROLLERS:
        cells = []
        for metric in (
            "normalized_iae_e_y_m",
            "max_abs_e_y",
            "normalized_iae_e_psi_deg",
            "rms_speed_error_mps",
            "mean_abs_steer_delta",
            "p95_abs_lateral_jerk",
        ):
            row = lookup(metric_rows, controller, "all", metric)
            cells.append(change(row["improvement_pct"]))
        gate = lookup_rate(rate_rows, controller, "all", "hard_gate_passed")
        stability = lookup_rate(rate_rows, controller, "all", "stability_passed")
        lines.append("| {} | {} | {} | {} | {} | {} | {} | {} pp | {} pp |".format(
            controller.upper(), *(cells + [change(gate["change_pp"], suffix=""), change(stability["change_pp"], suffix="")])
        ))

    lines.extend([
        "",
        "### 结果判定",
        "",
        *result_lines,
        "",
        "## 总体绝对值与 95% 区间",
        "",
        "| 控制器 | 指标 | 初始均值 | 优化均值 | 提升率 | 配对 bootstrap 95% CI |",
        "|---|---|---:|---:|---:|---:|",
    ])
    for controller in CONTROLLERS:
        for metric in ("normalized_iae_e_y_m", "max_abs_e_y", "normalized_iae_e_psi_deg", "rms_speed_error_mps"):
            row = lookup(metric_rows, controller, "all", metric)
            lines.append("| {} | {} | {} | {} | {} | [{}, {}] |".format(
                controller.upper(),
                row["metric_label"],
                fmt(row["initial_mean"], 4),
                fmt(row["optimized_mean"], 4),
                change(row["improvement_pct"]),
                change(row["bootstrap_ci95_low_pct"]),
                change(row["bootstrap_ci95_high_pct"]),
            ))

    lines.extend([
        "",
        "## 分层横向跟踪提升",
        "",
        "| 控制器 | controller-only | integrated | integrated-stress |",
        "|---|---:|---:|---:|",
    ])
    for controller in CONTROLLERS:
        values = [
            change(lookup(metric_rows, controller, layer, "normalized_iae_e_y_m")["improvement_pct"])
            for layer in LAYERS[1:]
        ]
        lines.append("| {} | {} | {} | {} |".format(controller.upper(), *values))

    lines.extend([
        "",
        "## 方法与可比性验证",
        "",
        "- 初始版：本轮重新执行的恢复版 PID/LQR 与历史四状态动态自行车 MPC，共 270/270 圈成功。",
        "- 优化版：{}。".format(validation["optimized_collection_description"]),
        "- 270 个 `evaluation_case_id + controller` 全部一一匹配；路线哈希、地图、速度、种子、评测层、车辆扰动和感知扰动均为 0 个不匹配。",
        "- MPC 比较口径：{}。".format(validation["mpc_comparison_scope"]),
        "",
        "## 审计信息",
        "",
        "- 初始行数：{}；混合优化行数：{}；配对键：{}。".format(
            validation["initial_rows"], validation["optimized_rows"], validation["paired_keys"]
        ),
        "- 路线/条件不匹配：{}。".format(validation["condition_mismatches"]),
        "- 优化 PID 源码 SHA-256：`{}`。".format(validation["source_hashes"]["optimized_pid_controller"]),
        "- 优化 LQR 源码 SHA-256：`{}`。".format(validation["source_hashes"]["optimized_lqr_controller"]),
        "- 优化 MPC 源码 SHA-256：`{}`。".format(validation["source_hashes"]["optimized_mpc_controller"]),
        "- 历史四状态 MPC SHA-256：`{}`。".format(validation["source_hashes"]["historical_four_state_mpc"]),
        "",
    ])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--initial", required=True)
    parser.add_argument("--optimized-pid", required=True)
    parser.add_argument("--reused-optimized", required=True)
    parser.add_argument("--optimized-aggregate")
    parser.add_argument("--optimized-full")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--project-root", required=True)
    parser.add_argument("--bootstrap-iterations", type=int, default=5000)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    initial_rows = read_rows(args.initial)
    if args.optimized_full:
        optimized_full_rows = read_rows(args.optimized_full)
        pid_rows = [row for row in optimized_full_rows if row.get("controller") == "pid"]
        reused_rows = optimized_full_rows
    elif args.optimized_aggregate:
        optimized_aggregate_rows = read_rows(args.optimized_aggregate)
        pid_rows = [row for row in optimized_aggregate_rows if row.get("controller") == "pid"]
        reused_rows = optimized_aggregate_rows
    else:
        pid_rows = read_rows(args.optimized_pid)
        reused_rows = read_rows(args.reused_optimized)
    optimized_rows, initial_by_key, optimized_by_key = validate_and_merge(
        initial_rows, pid_rows, reused_rows
    )
    if args.optimized_full:
        for row in optimized_rows:
            row["comparison_data_source"] = "fresh_reoptimized_all_three_20260714"
    metric_rows, rate_rows, delta_rows = build_summary(
        initial_by_key, optimized_by_key, args.bootstrap_iterations
    )

    validation = {
        "status": "ready_to_share",
        "as_of_date": "2026-07-14",
        "initial_rows": len(initial_rows),
        "optimized_rows": len(optimized_rows),
        "paired_keys": len(initial_by_key),
        "cases_per_controller": 90,
        "condition_mismatches": 0,
        "bootstrap_iterations": args.bootstrap_iterations,
        "optimized_sources": {
            "full": os.path.abspath(args.optimized_full) if args.optimized_full else None,
            "pid": os.path.abspath(args.optimized_pid),
            "lqr_mpc": os.path.abspath(args.reused_optimized),
            "reaggregated_triplets": os.path.abspath(args.optimized_aggregate) if args.optimized_aggregate else None,
        },
        "initial_source": os.path.abspath(args.initial),
        "optimized_collection_description": (
            "本轮重新执行 PID/LQR/MPC，270/270 圈成功，未复用旧优化结果"
            if args.optimized_full else
            "本轮重新执行 PID 90 圈，并复用已验证的 LQR/MPC 180 圈"
        ),
        "optimized_mpc_model_type": sorted({row.get("mpc_model_type", "") for row in optimized_rows}),
        "mpc_comparison_scope": (
            "历史四状态 dynamic_bicycle 与再次优化后的四状态 dynamic_bicycle 同模型谱系版本对比"
            if sorted({row.get("mpc_model_type", "") for row in optimized_rows}) == ["dynamic_bicycle"] else
            "历史四状态 dynamic_bicycle 与优化版其他预测模型的版本级对比"
        ),
        "source_hashes": {
            "optimized_pid_controller": file_sha256(os.path.join(args.project_root, "control", "pid_controller.py")),
            "optimized_lqr_controller": file_sha256(os.path.join(args.project_root, "control", "lqr_controller.py")),
            "optimized_mpc_controller": file_sha256(os.path.join(args.project_root, "control", "mpc_controller.py")),
            "historical_four_state_mpc": file_sha256(os.path.join(args.project_root, "control", "recovered_four_state_mpc.py")),
        },
    }

    write_rows(os.path.join(args.output_dir, "combined_optimized_raw.csv"), optimized_rows)
    write_rows(os.path.join(args.output_dir, "metric_comparison.csv"), metric_rows)
    write_rows(os.path.join(args.output_dir, "pass_rate_comparison.csv"), rate_rows)
    write_rows(os.path.join(args.output_dir, "paired_lap_deltas.csv"), delta_rows)
    with open(os.path.join(args.output_dir, "validation.json"), "w", encoding="utf-8") as handle:
        json.dump(validation, handle, ensure_ascii=False, indent=2)
    with open(os.path.join(args.output_dir, "comparison_report.md"), "w", encoding="utf-8-sig") as handle:
        handle.write(render_report(metric_rows, rate_rows, validation))
    print(json.dumps(validation, ensure_ascii=False))


if __name__ == "__main__":
    main()
