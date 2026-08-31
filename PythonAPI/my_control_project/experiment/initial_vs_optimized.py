"""Paired comparison of the frozen initial Git controllers and optimized ones."""

import argparse
import csv
import html
import math
import os
from collections import defaultdict


LOWER_IS_BETTER_METRICS = (
    "normalized_iae_e_y_m",
    "max_abs_e_y",
    "normalized_iae_e_psi_deg",
    "rms_speed_error_mps",
    "steer_spectrum_stability_index",
    "steer_spectrum_comfort_index",
    "p95_abs_lateral_accel",
    "p95_abs_lateral_jerk",
    "p95_abs_longitudinal_jerk",
    "controller_runtime_p95_ms",
    "collision_count",
    "lane_boundary_violation_count",
)


def _read_rows(path):
    with open(path, "r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def _key(row):
    return row.get("evaluation_case_id", ""), row.get("controller", "")


def _number(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _truth(value):
    return str(value).strip().lower() in {"1", "true", "yes"}


def _mean(values):
    values = [value for value in values if value is not None]
    return sum(values) / len(values) if values else None


def _improvement(initial, optimized):
    if initial is None or optimized is None or initial == 0:
        return None
    return (initial - optimized) / abs(initial) * 100.0


def _summarize(controller, layer, pairs, initial_population, optimized_population):
    row = {
        "controller": controller,
        "layer": layer,
        "paired_laps": len(pairs),
        "initial_population_laps": len(initial_population),
        "optimized_population_laps": len(optimized_population),
    }
    for metric in LOWER_IS_BETTER_METRICS:
        initial_mean = _mean(_number(initial.get(metric)) for initial, _ in pairs)
        optimized_mean = _mean(_number(optimized.get(metric)) for _, optimized in pairs)
        row["initial_{}_mean".format(metric)] = initial_mean
        row["optimized_{}_mean".format(metric)] = optimized_mean
        row["{}_improvement_pct".format(metric)] = _improvement(initial_mean, optimized_mean)

    initial_pass = _mean(100.0 if _truth(item.get("hard_gate_passed")) else 0.0 for item in initial_population)
    optimized_pass = _mean(100.0 if _truth(item.get("hard_gate_passed")) else 0.0 for item in optimized_population)
    row["initial_hard_gate_pass_rate_pct"] = initial_pass
    row["optimized_hard_gate_pass_rate_pct"] = optimized_pass
    row["hard_gate_pass_rate_change_pp"] = (
        optimized_pass - initial_pass
        if initial_pass is not None and optimized_pass is not None
        else None
    )
    return row


def compute_comparison(initial_csv, optimized_csv):
    initial_rows = _read_rows(initial_csv)
    optimized_rows = _read_rows(optimized_csv)
    initial_by_key = {_key(row): row for row in initial_rows}
    optimized_by_key = {_key(row): row for row in optimized_rows}
    shared_keys = sorted(set(initial_by_key) & set(optimized_by_key))
    if not shared_keys:
        raise ValueError("No matching evaluation-case/controller pairs were found.")

    pairs_by_group = defaultdict(list)
    for key in shared_keys:
        initial = initial_by_key[key]
        optimized = optimized_by_key[key]
        if initial.get("status") != "ok" or optimized.get("status") != "ok":
            continue
        initial_hash = initial.get("trajectory_hash", "").strip()
        optimized_hash = optimized.get("trajectory_hash", "").strip()
        if not initial_hash or initial_hash != optimized_hash:
            raise ValueError(
                "Reference trajectory hash differs for {} / {}.".format(*key)
            )
        controller = key[1]
        layer = initial.get("layer", "")
        pairs_by_group[(controller, layer)].append((initial, optimized))
        pairs_by_group[(controller, "all")].append((initial, optimized))

    controllers = sorted({key[1] for key in shared_keys})
    layers = ["all", "controller_only", "integrated", "integrated_stress"]
    output = []
    for controller in controllers:
        for layer in layers:
            initial_population = [
                row for row in initial_rows
                if row.get("controller") == controller and (layer == "all" or row.get("layer") == layer)
            ]
            optimized_population = [
                row for row in optimized_rows
                if row.get("controller") == controller and (layer == "all" or row.get("layer") == layer)
            ]
            pairs = pairs_by_group.get((controller, layer), [])
            if pairs or initial_population or optimized_population:
                output.append(_summarize(
                    controller, layer, pairs, initial_population, optimized_population
                ))
    return output


def write_csv(rows, path):
    if not rows:
        raise ValueError("Comparison produced no rows.")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _display(value):
    return "N/A" if value is None else "{:.2f}".format(value)


def render_markdown(rows, initial_csv, optimized_csv):
    lines = [
        "# 初始 Git 基线与优化控制方式的同条件对比",
        "",
        "本表只使用两次评测中 `evaluation_case_id + controller` 相同且参考轨迹哈希一致的配对圈次。正的提升率表示误差/振荡/计算耗时下降；通过率使用百分点差。",
        "",
        "| 控制器 | 范围 | 配对圈次 | 横向 IAE 提升 | 最大横向误差提升 | 稳定频带指标提升 | 舒适频带指标提升 | 硬门限通过率变化 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {controller} | {layer} | {paired_laps} | {iae}% | {mle}% | {stability}% | {comfort}% | {gate} pp |".format(
                controller=row["controller"].upper(),
                layer=row["layer"],
                paired_laps=row["paired_laps"],
                iae=_display(row.get("normalized_iae_e_y_m_improvement_pct")),
                mle=_display(row.get("max_abs_e_y_improvement_pct")),
                stability=_display(row.get("steer_spectrum_stability_index_improvement_pct")),
                comfort=_display(row.get("steer_spectrum_comfort_index_improvement_pct")),
                gate=_display(row.get("hard_gate_pass_rate_change_pp")),
            )
        )
    lines.extend([
        "",
        "## 可审计输入",
        "",
        "- 初始实现：`{}`".format(os.path.abspath(initial_csv)),
        "- 优化实现：`{}`".format(os.path.abspath(optimized_csv)),
        "",
        "论文结果只作为外部参照，不参与上述严格优化百分比计算。",
    ])
    return "\n".join(lines) + "\n"


def render_html(rows, initial_csv, optimized_csv):
    overall = [row for row in rows if row.get("layer") == "all"]

    def number(value, digits=3):
        return "N/A" if value is None else ("{:,.%df}" % digits).format(value)

    def delta(value, suffix="%"):
        if value is None:
            return '<span class="neutral">N/A</span>'
        css = "good" if value > 0 else "bad" if value < 0 else "neutral"
        label = "改善" if value > 0 else "优化后变差" if value < 0 else "不变"
        return '<span class="{}">{:+.1f}{} · {}</span>'.format(css, value, suffix, label)

    cards = []
    for row in overall:
        cards.append("""
        <article class="card">
          <div class="card-head"><h2>{controller}</h2><span>{laps} 对同轨迹圈次</span></div>
          <div class="metric hero"><label>归一化横向 IAE</label><b>{iae_initial} → {iae_opt}</b>{iae_delta}</div>
          <div class="metric"><label>最大横向误差均值</label><b>{max_initial} m → {max_opt} m</b>{max_delta}</div>
          <div class="metric"><label>硬门限通过率</label><b>{gate_initial}% → {gate_opt}%</b>{gate_delta}</div>
          <div class="mini"><span>稳定频带 {stability}</span><span>舒适频带 {comfort}</span><span>p95 计算耗时 {runtime}</span></div>
        </article>""".format(
            controller=html.escape(str(row["controller"]).upper()),
            laps=row["paired_laps"],
            iae_initial=number(row.get("initial_normalized_iae_e_y_m_mean")),
            iae_opt=number(row.get("optimized_normalized_iae_e_y_m_mean")),
            iae_delta=delta(row.get("normalized_iae_e_y_m_improvement_pct")),
            max_initial=number(row.get("initial_max_abs_e_y_mean")),
            max_opt=number(row.get("optimized_max_abs_e_y_mean")),
            max_delta=delta(row.get("max_abs_e_y_improvement_pct")),
            gate_initial=number(row.get("initial_hard_gate_pass_rate_pct"), 1),
            gate_opt=number(row.get("optimized_hard_gate_pass_rate_pct"), 1),
            gate_delta=delta(row.get("hard_gate_pass_rate_change_pp"), " pp"),
            stability=delta(row.get("steer_spectrum_stability_index_improvement_pct")),
            comfort=delta(row.get("steer_spectrum_comfort_index_improvement_pct")),
            runtime=delta(row.get("controller_runtime_p95_ms_improvement_pct")),
        ))

    layer_rows = []
    for row in rows:
        layer_rows.append("<tr><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td></tr>".format(
            html.escape(str(row["controller"]).upper()),
            html.escape(str(row["layer"])),
            row["paired_laps"],
            delta(row.get("normalized_iae_e_y_m_improvement_pct")),
            delta(row.get("max_abs_e_y_improvement_pct")),
        ))

    return """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>控制方式初始—优化对比评测</title><style>
:root{{--bg:#090b0f;--panel:#141922;--panel2:#1b2330;--line:#2a3443;--text:#f6f8fb;--muted:#99a6b8;--blue:#69a7ff;--good:#53d18c;--bad:#ff8b72}}
*{{box-sizing:border-box}} body{{margin:0;background:var(--bg);color:var(--text);font-family:Inter,"Segoe UI","Microsoft YaHei",sans-serif;line-height:1.55}}
main{{max-width:1240px;margin:auto;padding:56px 36px 72px}} .eyebrow{{color:var(--blue);font-weight:700;letter-spacing:.12em;text-transform:uppercase}}
h1{{font-size:42px;line-height:1.15;margin:12px 0}} .lead{{color:var(--muted);max-width:880px;font-size:18px}}
.notice{{margin:28px 0;padding:18px 22px;border:1px solid #735c2d;background:#241f15;border-radius:16px;color:#ffd889}}
.grid{{display:grid;grid-template-columns:repeat(3,1fr);gap:18px;margin:28px 0 42px}} .card{{background:linear-gradient(145deg,var(--panel2),var(--panel));border:1px solid var(--line);border-radius:22px;padding:22px;box-shadow:0 16px 50px #0006}}
.card-head{{display:flex;justify-content:space-between;align-items:center}} .card h2{{margin:0;font-size:26px}} .card-head span,.metric label{{color:var(--muted);font-size:13px}}
.metric{{border-top:1px solid var(--line);padding:15px 0;display:grid;gap:5px}} .metric.hero{{margin-top:14px}} .metric b{{font-size:20px}} .good{{color:var(--good)}} .bad{{color:var(--bad)}} .neutral{{color:var(--muted)}}
.mini{{display:grid;gap:8px;font-size:13px}} section{{margin-top:42px}} h2{{font-size:26px}} table{{width:100%;border-collapse:collapse;background:var(--panel);border-radius:16px;overflow:hidden}} th,td{{padding:12px 14px;border-bottom:1px solid var(--line);text-align:left}} th{{color:var(--muted);font-size:12px;text-transform:uppercase}}
.sources{{font-family:Consolas,monospace;color:var(--muted);font-size:12px;word-break:break-all}} .provenance{{display:grid;grid-template-columns:repeat(3,1fr);gap:14px}} .provenance div{{background:var(--panel);border:1px solid var(--line);padding:18px;border-radius:16px}}
@media(max-width:900px){{.grid,.provenance{{grid-template-columns:1fr}}h1{{font-size:32px}}main{{padding:32px 18px}}}}
</style></head><body><main>
<div class="eyebrow">Paired controller evaluation · 2026-07-14</div><h1>三种控制方式：初始代码 vs 优化实现</h1>
<p class="lead">同轨迹严格配对：相同车辆、路线、速度、随机种子、控制周期和评价标准；仅替换控制器实现。正值表示相应“越低越好”指标下降，负值明确标为优化后变差。</p>
<div class="notice"><strong>结论：</strong>当前修改不能称为“三种控制器全面优化”。LQR、MPC 的最大偏差与硬门限通过率改善，但总体 IAE 变差；PID 的总体 IAE、最大偏差与通过率均退化。应保留有益改动并针对退化项重新调参。</div>
<div class="grid">{cards}</div>
<section><h2>分层结果</h2><table><thead><tr><th>控制器</th><th>评测层</th><th>配对圈次</th><th>IAE 变化</th><th>最大误差变化</th></tr></thead><tbody>{layer_rows}</tbody></table></section>
<section><h2>原始代码与论文谱系</h2><div class="provenance"><div><b>PID</b><p>直接软件来源可高置信追溯至 CARLA 官方 PIDLateralController；理论源流可引 Minorsky，但不存在唯一对应论文代码。</p></div><div><b>LQR</b><p>四状态动态自行车、DARE 与曲率前馈最接近 Snider（2009）/Rajamani 方法谱系；参数不一致，不能声称逐行来自论文。</p></div><div><b>MPC</b><p>二状态运动学误差模型与 condensed 线性 MPC 属通用教材框架；无引用或代码指纹可唯一锁定论文。</p></div></div></section>
<section><h2>审计口径</h2><p>初始基线来自 Git 首次提交 <code>c0a60d6d</code>。基础论文没有共同 CARLA 测试条件，因此不能计算严格论文提升百分比；论文表格只能作为跨研究参照，不计入本页提升率。</p><p class="sources">Initial: {initial}<br>Optimized: {optimized}</p></section>
</main></body></html>""".format(
        cards="".join(cards),
        layer_rows="".join(layer_rows),
        initial=html.escape(os.path.abspath(initial_csv)),
        optimized=html.escape(os.path.abspath(optimized_csv)),
    )


def main():
    parser = argparse.ArgumentParser(description="Compare initial and optimized controller matrices.")
    parser.add_argument("--initial", required=True)
    parser.add_argument("--optimized", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    rows = compute_comparison(args.initial, args.optimized)
    os.makedirs(args.output_dir, exist_ok=True)
    csv_path = os.path.join(args.output_dir, "initial_vs_optimized_comparison.csv")
    markdown_path = os.path.join(args.output_dir, "initial_vs_optimized_comparison.md")
    html_path = os.path.join(args.output_dir, "controller_optimization_comparison_report.html")
    write_csv(rows, csv_path)
    with open(markdown_path, "w", encoding="utf-8") as handle:
        handle.write(render_markdown(rows, args.initial, args.optimized))
    with open(html_path, "w", encoding="utf-8") as handle:
        handle.write(render_html(rows, args.initial, args.optimized))
    print(csv_path)
    print(markdown_path)
    print(html_path)


if __name__ == "__main__":
    main()
