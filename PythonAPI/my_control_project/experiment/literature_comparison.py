"""Cross-study reference comparison against Artuñedo et al. (2024)."""

import csv


# Table 4, best-IAE setups, T1--T3 mean. These are external references and
# deliberately are not treated as the repository's historical baseline.
REFERENCE = {
    "LQR": {"iae": 0.171, "mle": 0.671, "stability": 0.106, "comfort": 0.253},
    "PID": {"iae": 0.150, "mle": 0.496, "stability": 0.122, "comfort": 0.258},
    "MPC": {"iae": 0.055, "mle": 0.263, "stability": 0.222, "comfort": 0.234},
}


def _mean(rows, field):
    values = [float(row[field]) for row in rows if row.get(field) not in (None, "")]
    return sum(values) / len(values) if values else None


def _lower_delta(reference, current):
    return (reference - current) / reference * 100.0


def compute_comparison(raw_csv):
    with open(raw_csv, "r", encoding="utf-8-sig", newline="") as handle:
        source = list(csv.DictReader(handle))
    output = []
    for controller, reference in REFERENCE.items():
        rows = [
            row for row in source
            if row.get("status") == "ok"
            and row.get("layer") == "integrated"
            and row.get("controller", "").upper() == controller
        ]
        means = {
            "iae": _mean(rows, "normalized_iae_e_y_m"),
            "mle": _mean(rows, "max_abs_e_y"),
            "stability": _mean(rows, "steer_spectrum_stability_index"),
            "comfort": _mean(rows, "steer_spectrum_comfort_index"),
        }
        item = {
            "controller": controller,
            "sample_count": len(rows),
            "strict_foundational_improvement_pct": "N/A",
        }
        for key, value in means.items():
            item["current_{}".format(key)] = value
            item["{}_reference_delta_pct".format(key)] = (
                _lower_delta(reference[key], value) if value is not None else None
            )
        output.append(item)
    return output


def render_markdown(rows):
    lines = [
        "# 与公开论文数据的外部参照",
        "",
        "基础论文没有在本项目的 CARLA 车辆、路线和扰动条件下报告共同指标，因此严格的基础论文提升百分比不能计算；这不是 0%。",
        "Artuñedo 等（2024）的数值仅标为跨研究参考差值，不能替代同条件初始代码基线。",
        "",
    ]
    if rows:
        lines.extend([
            "| 控制器 | 综合层样本 | IAE 跨研究差值 | 最大误差跨研究差值 | Mε 差值 | Mζ 差值 |",
            "|---|---:|---:|---:|---:|---:|",
        ])
        for row in rows:
            def display(key):
                value = row.get(key)
                return "N/A" if value is None else "{:.2f}%".format(value)
            lines.append("| {controller} | {sample_count} | {iae} | {mle} | {stability} | {comfort} |".format(
                controller=row["controller"],
                sample_count=row["sample_count"],
                iae=display("iae_reference_delta_pct"),
                mle=display("mle_reference_delta_pct"),
                stability=display("stability_reference_delta_pct"),
                comfort=display("comfort_reference_delta_pct"),
            ))
    return "\n".join(lines) + "\n"
