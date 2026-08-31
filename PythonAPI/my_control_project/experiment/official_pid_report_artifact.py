"""Build a canonical MCP report artifact for the official CARLA 0.9.14 PID run."""

from __future__ import annotations

import argparse
import csv
import json
import os
from datetime import datetime, timezone


LAYER_LABELS = {
    "controller_only": "纯控制器",
    "integrated": "系统集成",
    "integrated_stress": "集成压力",
}


def _read_csv(path):
    with open(path, "r", encoding="utf-8-sig", newline="") as handle:
        return [dict(row) for row in csv.DictReader(handle)]


def _read_json(path):
    with open(path, "r", encoding="utf-8-sig") as handle:
        return json.load(handle)


def _number(value):
    return float(value)


def _truth(value):
    return str(value).strip().lower() in {"true", "1", "yes", "passed", "ok"}


def _metric(rows, group_type, group, name):
    return next(row for row in rows if row["group_type"] == group_type and row["group"] == group and row["metric"] == name)


def _pass(rows, group_type, group, name):
    return next(row for row in rows if row["group_type"] == group_type and row["group"] == group and row["check"] == name)


def build_artifact(input_dir):
    raw = _read_csv(os.path.join(input_dir, "official_raw_validated.csv"))
    metrics = _read_csv(os.path.join(input_dir, "official_metric_summary.csv"))
    passes = _read_csv(os.path.join(input_dir, "official_pass_summary.csv"))
    worst = _read_csv(os.path.join(input_dir, "official_worst_cases.csv"))
    validation = _read_json(os.path.join(input_dir, "validation.json"))
    generated = datetime.now(timezone.utc).isoformat()

    overall_lateral = _metric(metrics, "overall", "all", "normalized_iae_e_y_m")
    overall_heading = _metric(metrics, "overall", "all", "normalized_iae_e_psi_deg")
    overall_speed = _metric(metrics, "overall", "all", "rms_speed_error_mps")
    overall_runtime = _metric(metrics, "overall", "all", "controller_runtime_p95_ms")
    overall_gate = _pass(passes, "overall", "all", "hard_gate_passed")

    headline = [{
        "case_count": len(raw),
        "lateral_iae_m": _number(overall_lateral["mean"]),
        "heading_iae_deg": _number(overall_heading["mean"]),
        "speed_rms_mps": _number(overall_speed["mean"]),
        "hard_gate_rate": _number(overall_gate["pass_rate_pct"]) / 100.0,
        "runtime_p95_ms": _number(overall_runtime["mean"]),
    }]

    layer_rows = []
    layer_pass_rows = []
    for layer, label in LAYER_LABELS.items():
        layer_rows.append({
            "layer": label,
            "layer_id": layer,
            "case_count": 30,
            "lateral_iae_m": _number(_metric(metrics, "layer", layer, "normalized_iae_e_y_m")["mean"]),
            "heading_iae_deg": _number(_metric(metrics, "layer", layer, "normalized_iae_e_psi_deg")["mean"]),
            "speed_rms_mps": _number(_metric(metrics, "layer", layer, "rms_speed_error_mps")["mean"]),
            "lateral_jerk_p95": _number(_metric(metrics, "layer", layer, "p95_abs_lateral_jerk")["mean"]),
            "runtime_p95_ms": _number(_metric(metrics, "layer", layer, "controller_runtime_p95_ms")["mean"]),
        })
        check = _pass(passes, "layer", layer, "hard_gate_passed")
        layer_pass_rows.append({
            "layer": label,
            "layer_id": layer,
            "successes": int(check["successes"]),
            "total": int(check["total"]),
            "pass_rate_pct": _number(check["pass_rate_pct"]),
            "wilson_low_pct": _number(check["wilson_ci95_low_pct"]),
            "wilson_high_pct": _number(check["wilson_ci95_high_pct"]),
        })

    scenario_rows = []
    for scenario in sorted({row["scenario"] for row in raw}):
        selected = [row for row in raw if row["scenario"] == scenario]
        scenario_rows.append({
            "scenario": scenario,
            "case_count": len(selected),
            "lateral_iae_m": _number(_metric(metrics, "scenario", scenario, "normalized_iae_e_y_m")["mean"]),
            "max_lateral_error_m": _number(_metric(metrics, "scenario", scenario, "max_abs_e_y")["mean"]),
            "heading_iae_deg": _number(_metric(metrics, "scenario", scenario, "normalized_iae_e_psi_deg")["mean"]),
            "speed_rms_mps": _number(_metric(metrics, "scenario", scenario, "rms_speed_error_mps")["mean"]),
            "hard_gate_rate_pct": _number(_pass(passes, "scenario", scenario, "hard_gate_passed")["pass_rate_pct"]),
        })
    scenario_rows.sort(key=lambda row: row["lateral_iae_m"], reverse=True)

    worst_rows = [{
        "case_id": row["evaluation_case_id"],
        "layer": LAYER_LABELS[row["layer"]],
        "scenario": row["scenario"],
        "lateral_iae_m": _number(row["normalized_iae_e_y_m"]),
        "max_lateral_error_m": _number(row["max_abs_e_y"]),
        "lateral_jerk_p95": _number(row["p95_abs_lateral_jerk"]),
        "route_completion_pct": _number(row["route_completion_pct"]),
        "hard_gate_passed": _truth(row["hard_gate_passed"]),
        "failure_reason": row["hard_gate_reasons"] or row["stability_fail_reasons"],
    } for row in worst]

    source = {
        "id": "official_pid_results",
        "label": "CARLA 0.9.14 官方 PID 90 工况结果",
        "path": "official_raw_validated.csv",
        "description": "冻结路线与随机种子矩阵下，官方 VehiclePIDController 的逐工况验证结果。",
        "query": {
            "engine": "duckdb",
            "language": "sql",
            "sql": "SELECT * FROM read_csv_auto('official_raw_validated.csv', header=true)",
            "description": "读取全部 90 个已通过来源、完整性和数值审计的官方 PID 工况。",
            "tables_used": ["official_raw_validated.csv"],
            "filters": ["controller = 'pid'", "status = 'ok'", "90 个冻结工况，不删除硬门槛失败工况"],
            "metric_definitions": {
                "lateral_iae_m": "横向误差绝对值的时间积分除以有效时长，单位 m。",
                "hard_gate_rate": "同时满足执行成功、无碰撞、无越界、路线完成率≥99%、无跟踪失败且频谱有效的工况数/全部工况数。",
                "runtime_p95_ms": "每个工况内控制器 run_step 耗时的第 95 百分位；报告展示其跨工况均值。",
            },
        },
    }

    charts = [
        {
            "id": "layer_lateral_error", "title": "各评测层横向跟踪误差",
            "subtitle": "每层 30 个工况；柱高为时间归一化横向 IAE 的工况均值，越低越好。",
            "type": "bar", "dataset": "layer_summary", "sourceId": source["id"],
            "encodings": {
                "x": {"field": "layer", "type": "nominal", "label": "评测层"},
                "y": {"field": "lateral_iae_m", "type": "quantitative", "label": "横向 IAE", "unit": "m"},
            },
            "valueFormat": "number", "layout": "full",
        },
        {
            "id": "layer_gate_rate", "title": "各评测层硬门槛通过率",
            "subtitle": "每层 30 个工况；精确 Wilson 区间见同源分层表。",
            "type": "bar", "dataset": "layer_pass", "sourceId": source["id"],
            "encodings": {
                "x": {"field": "layer", "type": "nominal", "label": "评测层"},
                "y": {"field": "pass_rate_pct", "type": "quantitative", "label": "通过率", "unit": "%"},
            },
            "valueFormat": "number", "layout": "full",
        },
        {
            "id": "scenario_lateral_error", "title": "各场景横向跟踪误差",
            "subtitle": "按场景均值从高到低排序；同源数据保留工况数、航向误差、速度误差和硬门槛通过率。",
            "type": "bar", "dataset": "scenario_summary", "sourceId": source["id"],
            "encodings": {
                "x": {"field": "scenario", "type": "nominal", "label": "场景"},
                "y": {"field": "lateral_iae_m", "type": "quantitative", "label": "横向 IAE", "unit": "m"},
            },
            "valueFormat": "number", "layout": "full",
        },
    ]

    tables = [
        {
            "id": "layer_table", "title": "分层精确汇总",
            "subtitle": "均值用于描述层级表现；通过率保留 Wilson 95% 区间。",
            "dataset": "layer_table", "sourceId": source["id"], "layout": "full", "density": "dense",
            "defaultSort": {"field": "lateral_iae_m", "direction": "desc"},
            "columns": [
                {"field": "layer", "label": "评测层"},
                {"field": "case_count", "label": "工况数"},
                {"field": "lateral_iae_m", "label": "横向 IAE", "format": "number", "unit": "m"},
                {"field": "heading_iae_deg", "label": "航向 IAE", "format": "number", "unit": "deg"},
                {"field": "speed_rms_mps", "label": "速度 RMS", "format": "number", "unit": "m/s"},
                {"field": "lateral_jerk_p95", "label": "横向 jerk P95", "format": "number", "unit": "m/s³"},
                {"field": "runtime_p95_ms", "label": "P95 耗时", "format": "number", "unit": "ms"},
                {"field": "pass_rate_pct", "label": "硬门槛", "format": "number", "unit": "%"},
                {"field": "wilson_low_pct", "label": "Wilson 下界", "format": "number", "unit": "%"},
            ],
        },
        {
            "id": "worst_table", "title": "横向 IAE 最高的 10 个工况",
            "subtitle": "保留场景、层级、最大误差、jerk、完成率和失败原因，便于回放定位。",
            "dataset": "worst_cases", "sourceId": source["id"], "layout": "full", "density": "dense",
            "defaultSort": {"field": "lateral_iae_m", "direction": "desc"},
            "columns": [
                {"field": "case_id", "label": "工况"},
                {"field": "layer", "label": "层"},
                {"field": "scenario", "label": "场景"},
                {"field": "lateral_iae_m", "label": "横向 IAE", "format": "number", "unit": "m"},
                {"field": "max_lateral_error_m", "label": "最大横向误差", "format": "number", "unit": "m"},
                {"field": "lateral_jerk_p95", "label": "jerk P95", "format": "number", "unit": "m/s³"},
                {"field": "route_completion_pct", "label": "完成率", "format": "number", "unit": "%"},
                {"field": "failure_reason", "label": "失败原因"},
            ],
        },
    ]

    cards = [
        {"id": "cases", "description": "完成来源与数值审计的冻结工况总数。", "dataset": "headline", "sourceId": source["id"], "metrics": [{"label": "评测工况", "field": "case_count", "format": "number"}]},
        {"id": "lateral", "description": "90 个工况的时间归一化横向 IAE 均值。", "dataset": "headline", "sourceId": source["id"], "metrics": [{"label": "横向 IAE", "field": "lateral_iae_m", "format": "number", "unit": "m"}]},
        {"id": "speed", "description": "90 个工况的速度 RMS 误差均值。", "dataset": "headline", "sourceId": source["id"], "metrics": [{"label": "速度 RMS", "field": "speed_rms_mps", "format": "number", "unit": "m/s"}]},
        {"id": "gate", "description": "通过全部安全与有效性硬门槛的工况比例。", "dataset": "headline", "sourceId": source["id"], "metrics": [{"label": "硬门槛通过率", "field": "hard_gate_rate", "format": "percent"}]},
        {"id": "runtime", "description": "逐工况控制器 P95 运行时间的跨工况均值。", "dataset": "headline", "sourceId": source["id"], "metrics": [{"label": "P95 运行时间", "field": "runtime_p95_ms", "format": "number", "unit": "ms"}]},
    ]

    summary_text = (
        "## 官方 PID 在 90 个冻结工况中的总体表现\n\n"
        "CARLA 0.9.14 官方 `VehiclePIDController` 的横向归一化 IAE 均值为 "
        "**{:.4f} m**（bootstrap 95% CI {:.4f}–{:.4f}），航向归一化 IAE 均值为 **{:.4f} deg**，"
        "速度 RMS 误差均值为 **{:.4f} m/s**。硬门槛通过 **{}/{}（{:.1f}%）**；"
        "这与 90/90 个程序执行成功是两个不同口径。"
    ).format(
        _number(overall_lateral["mean"]), _number(overall_lateral["bootstrap_mean_ci95_low"]),
        _number(overall_lateral["bootstrap_mean_ci95_high"]), _number(overall_heading["mean"]),
        _number(overall_speed["mean"]), int(overall_gate["successes"]), int(overall_gate["total"]),
        _number(overall_gate["pass_rate_pct"]),
    )

    blocks = [
        {"id": "title", "type": "markdown", "body": "# CARLA 0.9.14 官方 PID 控制器表现评测"},
        {"id": "summary", "type": "markdown", "sourceId": source["id"], "body": summary_text},
        {"id": "headline", "type": "metric-strip", "cardIds": ["cases", "lateral", "speed", "gate", "runtime"]},
        {"id": "scope", "type": "markdown", "body": "## 范围只包含官方 PID\n\n本报告暂不运行、也不讨论正在升级的 PID/LQR/MPC。官方对象为 CARLA 0.9.14 `agents.navigation.controller.VehiclePIDController`；同版本 `LocalPlanner` 默认参数和源码 SHA-256 已逐工况核验。"},
        {"id": "layers", "type": "markdown", "sourceId": source["id"], "body": "## 三层矩阵区分控制器本体与系统影响\n\n纯控制器层关闭项目速度规划；系统集成层启用实验框架速度规划；压力层再加入车辆参数、感知噪声、延迟和丢帧扰动。每层均为 30 个工况。"},
        {"id": "layer_error_chart", "type": "chart", "chartId": "layer_lateral_error", "layout": "full"},
        {"id": "layer_gate_chart", "type": "chart", "chartId": "layer_gate_rate", "layout": "full"},
        {"id": "layer_table_block", "type": "table", "tableId": "layer_table", "layout": "full"},
        {"id": "scenarios", "type": "markdown", "sourceId": source["id"], "body": "## 场景拆分暴露总体均值不能显示的薄弱点\n\n场景图按横向 IAE 均值排序；同源数据保留每个场景的样本数、航向误差、速度误差和硬门槛通过率。路线几何审计发现 **{} 个**标称曲线路况的实际总转角低于预设检查阈值；这些工况未被删除，但必须按实际几何而非标签解释。".format(len(validation.get("route_geometry_warnings", [])))},
        {"id": "scenario_chart", "type": "chart", "chartId": "scenario_lateral_error", "layout": "full"},
        {"id": "worst", "type": "markdown", "sourceId": source["id"], "body": "## 最差工况保留到可回放粒度\n\n下表不是删掉的离群值，而是横向 IAE 最高的 10 个工况；工况 ID 可回连对应运行目录、逐步日志和轨迹图。"},
        {"id": "worst_table_block", "type": "table", "tableId": "worst_table", "layout": "full"},
        {"id": "method", "type": "markdown", "body": "## 方法、口径与限制\n\n硬门槛要求：执行成功、无碰撞、无车道边界违规、路线完成率至少 99%、无跟踪失败且频谱指标有效。均值置信区间采用工况级非参数 bootstrap（5,000 次），通过率采用 Wilson 95% 区间。`UN R79` 仅作横向加速度/jerk 参考检查，不构成认证。\n\n本测试在项目冻结路线与目标点接口中调用官方底层 PID，因此不等同于完整评测 CARLA 官方路径规划器，也不能外推为真实道路性能。"},
        {"id": "next", "type": "markdown", "body": "## 下一步等待升级版冻结\n\n待三种控制方式升级完成后，复用相同 90 工况、相同轨迹哈希和随机种子执行严格配对测试，再计算相对官方 PID 的提升比例与三者间差异。"},
        {"id": "questions", "type": "markdown", "body": "## 后续需要回答的问题\n\n- 升级版在纯控制器层的收益是否仍能延续到系统集成和压力层？\n- 平均误差改善是否伴随 jerk、转向反转或运行时间恶化？\n- 最差场景的硬门槛失败能否在配对运行中被消除？"},
    ]

    merged_layer_table = []
    pass_by_id = {row["layer_id"]: row for row in layer_pass_rows}
    for row in layer_rows:
        merged_layer_table.append({**row, **pass_by_id[row["layer_id"]]})

    title = "CARLA 0.9.14 官方 PID 控制器表现评测"
    return {
        "surface": "report",
        "manifest": {
            "version": 1,
            "surface": "report",
            "title": title,
            "description": "CARLA 0.9.14 官方 VehiclePIDController 的 90 工况技术评测。",
            "generatedAt": generated,
            "cards": cards,
            "charts": charts,
            "tables": tables,
            "sources": [source],
            "blocks": blocks,
        },
        "snapshot": {
            "version": 1,
            "generatedAt": generated,
            "status": "ready" if validation.get("ready_to_share") else "blocked",
            "datasets": {
                "headline": headline,
                "layer_summary": layer_rows,
                "layer_pass": layer_pass_rows,
                "layer_table": merged_layer_table,
                "scenario_summary": scenario_rows,
                "worst_cases": worst_rows,
            },
        },
        "sources": [source],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    artifact = build_artifact(args.input_dir)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(artifact, handle, ensure_ascii=False, indent=2, allow_nan=False)
    print(os.path.abspath(args.output))


if __name__ == "__main__":
    main()
