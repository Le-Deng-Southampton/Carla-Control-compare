import csv
import json
import os

import numpy as np

from .metrics import EXPERIMENT_METADATA_HEADER, SPEED_PLAN_REASONS, build_experiment_metadata


HUMAN_SUMMARY_FILENAME = "human_summary.txt"

LOG_HEADER = [
    *EXPERIMENT_METADATA_HEADER,
    "time", "x", "y", "speed", "speed_error", "e_y", "e_psi", "abs_e_y", "abs_e_psi",
    "steer", "steer_delta", "throttle", "brake", "target_speed", "speed_plan_risk", "speed_plan_reason",
    "route_index", "closest_route_index", "route_error", "road_option",
    "target_x", "target_y", "target_yaw",
    "lqr_q_ey", "lqr_q_ey_dot", "lqr_q_epsi", "lqr_q_epsi_dot", "lqr_r",
    "lqr_kp_long", "lqr_ki_long", "lqr_kd_long",
    "lqr_max_steer", "lqr_max_steer_rate",
    "lqr_derivative_alpha", "lqr_curvature_alpha", "lqr_feedforward_gain",
    "mpc_horizon", "mpc_q_y", "mpc_q_psi", "mpc_r_steer", "mpc_r_steer_rate",
    "mpc_kp_long", "mpc_ki_long", "mpc_kd_long",
    "mpc_max_steer", "mpc_max_steer_rate",
    "pid_lat_kp", "pid_lat_ki", "pid_lat_kd",
    "pid_long_kp", "pid_long_ki", "pid_long_kd",
    "pid_max_throttle", "pid_max_brake",
]


SUMMARY_HEADER = [
    *EXPERIMENT_METADATA_HEADER,
    "spawn_index", "route_waypoints", "target_speed_kmh",
    "route_total_abs_turn_rad", "route_total_abs_turn_deg",
    "route_mean_abs_curvature", "route_max_abs_curvature",
    "mean_abs_e_y", "rms_e_y", "max_abs_e_y",
    "mean_abs_e_psi_deg", "rms_e_psi_deg", "mean_abs_speed_error",
    "mean_abs_steer_delta", "max_abs_steer_delta",
    "mean_planned_speed_kmh", "min_planned_speed_kmh", "max_planned_speed_kmh",
    *[f"speed_plan_reason_{reason}_count" for reason in SPEED_PLAN_REASONS],
    "lqr_q_ey", "lqr_q_ey_dot", "lqr_q_epsi", "lqr_q_epsi_dot", "lqr_r",
    "lqr_max_steer", "lqr_max_steer_rate",
    "lqr_derivative_alpha", "lqr_curvature_alpha", "lqr_feedforward_gain",
    "mpc_horizon", "mpc_q_y", "mpc_q_psi", "mpc_r_steer", "mpc_r_steer_rate",
    "mpc_kp_long", "mpc_ki_long", "mpc_kd_long",
    "mpc_max_steer", "mpc_max_steer_rate",
    "pid_lat_kp", "pid_lat_ki", "pid_lat_kd",
    "pid_long_kp", "pid_long_ki", "pid_long_kd",
    "pid_max_throttle", "pid_max_brake",
]


METRIC_DESCRIPTIONS = {
    "mean_abs_e_y": ("平均横向偏差", "m", "车辆平均偏离路线中心线的距离，越小越贴线"),
    "rms_e_y": ("横向偏差稳定性", "m", "对较大偏差更敏感，越小表示路线跟踪更稳定"),
    "max_abs_e_y": ("最大横向偏差", "m", "本次测试中最严重的一次横向偏离，越小越安全"),
    "mean_abs_e_psi_deg": ("平均航向偏差", "deg", "车头方向与路线方向的平均夹角，越小越顺着路走"),
    "rms_e_psi_deg": ("航向偏差稳定性", "deg", "对较大航向偏差更敏感，越小表示转向更稳定"),
    "mean_abs_speed_error": ("平均速度误差", "m/s", "实际速度与目标速度的平均差距，越小越接近目标速度"),
    "mean_abs_steer_delta": ("平均转向变化", "", "相邻控制周期方向盘命令变化，越小越平顺"),
    "max_abs_steer_delta": ("最大转向突变", "", "本次测试中最大一次方向盘命令跳变，越小越平顺"),
}

SCORE_METRICS = [
    ("mean_abs_e_y", 0.30),
    ("rms_e_y", 0.20),
    ("mean_abs_e_psi_deg", 0.20),
    ("mean_abs_speed_error", 0.15),
    ("mean_abs_steer_delta", 0.10),
    ("max_abs_e_y", 0.05),
]

REASON_DESCRIPTIONS = {
    "none": "未触发限速，车辆按目标速度行驶",
    "entry_curvature": "前方弯道入口，需要提前降速",
    "curvature": "当前或预瞄路线曲率较大，需要降速过弯",
    "lateral_error": "横向偏差较大，需要保护性降速",
    "heading_error": "车头方向偏差较大，需要保护性降速",
    "lateral_error_rate": "横向偏差增长过快，需要保护性降速",
    "heading_error_rate": "航向偏差增长过快，需要保护性降速",
    "hold": "风险刚解除，短暂保持较低速度避免反复加减速",
}


def build_step_snapshot(
    vehicle,
    control,
    target_speed_ms,
    previous_steer,
    tracking_errors,
    speed_plan_risk=0.0,
    speed_plan_reason="none",
):
    transform = vehicle.get_transform()
    velocity = vehicle.get_velocity()
    speed = np.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
    location = transform.location
    speed_error = target_speed_ms - speed
    steer_delta = control.steer - previous_steer

    return {
        "x": location.x,
        "y": location.y,
        "speed": speed,
        "e_y": tracking_errors["e_y"],
        "e_psi": tracking_errors["e_psi"],
        "speed_error": speed_error,
        "steer_delta": steer_delta,
        "target_speed": target_speed_ms,
        "speed_plan_risk": speed_plan_risk,
        "speed_plan_reason": speed_plan_reason,
        "target_x": tracking_errors["target_x"],
        "target_y": tracking_errors["target_y"],
        "target_yaw": tracking_errors["target_yaw"],
    }


def append_step_data(rows, positions_x, positions_y, metrics, controller_name, args, route_state, snapshot, control, sim_time):
    route_index, closest_route_index, route_error, road_option = route_state
    metrics["abs_ey"].append(abs(snapshot["e_y"]))
    metrics["abs_epsi"].append(abs(snapshot["e_psi"]))
    metrics["abs_speed_error"].append(abs(snapshot["speed_error"]))
    metrics["steer_delta"].append(abs(snapshot["steer_delta"]))
    metrics["target_speed"].append(snapshot["target_speed"])
    reason_counts = metrics["speed_plan_reason_counts"]
    reason = snapshot["speed_plan_reason"]
    reason_counts[reason] = reason_counts.get(reason, 0) + 1
    positions_x.append(snapshot["x"])
    positions_y.append(snapshot["y"])

    rows.append(build_experiment_metadata(controller_name, args) + [
        sim_time,
        snapshot["x"],
        snapshot["y"],
        snapshot["speed"],
        snapshot["speed_error"],
        snapshot["e_y"],
        snapshot["e_psi"],
        abs(snapshot["e_y"]),
        abs(snapshot["e_psi"]),
        control.steer,
        snapshot["steer_delta"],
        control.throttle,
        control.brake,
        snapshot["target_speed"],
        snapshot["speed_plan_risk"],
        snapshot["speed_plan_reason"],
        route_index,
        closest_route_index,
        route_error,
        str(road_option),
        snapshot["target_x"],
        snapshot["target_y"],
        snapshot["target_yaw"],
        args.lqr_q_ey,
        args.lqr_q_ey_dot,
        args.lqr_q_epsi,
        args.lqr_q_epsi_dot,
        args.lqr_r,
        args.lqr_kp_long,
        args.lqr_ki_long,
        args.lqr_kd_long,
        args.lqr_max_steer,
        args.lqr_max_steer_rate,
        args.lqr_derivative_alpha,
        args.lqr_curvature_alpha,
        args.lqr_feedforward_gain,
        args.mpc_horizon,
        args.mpc_q_y,
        args.mpc_q_psi,
        args.mpc_r_steer,
        args.mpc_r_steer_rate,
        args.mpc_kp_long,
        args.mpc_ki_long,
        args.mpc_kd_long,
        args.mpc_max_steer,
        args.mpc_max_steer_rate,
        args.pid_lat_kp,
        args.pid_lat_ki,
        args.pid_lat_kd,
        args.pid_long_kp,
        args.pid_long_ki,
        args.pid_long_kd,
        args.pid_max_throttle,
        args.pid_max_brake,
    ])


def log_step(controller_name, route_trace, route_index, snapshot, control):
    print(
        f"{controller_name.upper()} | route: {route_index}/{len(route_trace) - 1}, "
        f"e_y: {snapshot['e_y']:+.2f} m, "
        f"e_psi: {np.degrees(snapshot['e_psi']):+.1f} deg, "
        f"steer: {control.steer:+.3f}, "
        f"d_steer: {snapshot['steer_delta']:+.3f}, "
        f"speed: {snapshot['speed']:.2f} m/s"
    )


def write_csv(path, header, rows):
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)


def write_text(path, lines):
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))
        handle.write("\n")


def _summary_records(summaries):
    records = []
    for row in summaries:
        record = {}
        for index, key in enumerate(SUMMARY_HEADER):
            record[key] = row[index] if index < len(row) else ""
        records.append(record)
    return records


def _as_float(value, default=0.0):
    try:
        if value == "":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _format_float(value, digits=2, suffix=""):
    number = _as_float(value)
    return f"{number:.{digits}f}{suffix}"


def _format_probability(value):
    return f"{_as_float(value) * 100.0:.1f}%"


def _controller_label(controller_name):
    return str(controller_name).upper()


def _speed_planner_label(mode):
    if mode == "adaptive":
        return "自适应速度规划：根据前方弯道、横向偏差和航向偏差自动降低目标速度"
    if mode == "off":
        return "固定目标速度：不启用动态降速，仅比较控制器本身"
    return str(mode)


def _error_provider_label(name):
    labels = {
        "ground_truth": "理想真实误差输入",
        "noisy_ground_truth": "带噪声的真实误差输入",
        "perception_proxy": "感知代理误差输入",
    }
    return labels.get(name, str(name))


def _route_difficulty(record):
    total_turn = _as_float(record.get("route_total_abs_turn_deg"))
    max_curvature = _as_float(record.get("route_max_abs_curvature"))
    if total_turn < 20.0 and max_curvature < 0.01:
        return "低：接近直线，主要考验速度保持和小幅修正"
    if total_turn < 180.0 and max_curvature < 0.04:
        return "中：包含温和弯道，考验转向平顺性"
    return "高：弯道或 S 弯明显，考验路线跟踪、提前降速和转向稳定性"


def _score_summaries(records):
    if not records:
        return []

    metric_max = {
        metric: max(_as_float(record.get(metric)) for record in records)
        for metric, _ in SCORE_METRICS
    }
    scored = []
    for record in records:
        score = 0.0
        for metric, weight in SCORE_METRICS:
            maximum = metric_max[metric]
            value = _as_float(record.get(metric))
            normalized = value / maximum if maximum > 1e-9 else 0.0
            score += normalized * weight
        scored.append((score, record))
    return sorted(scored, key=lambda item: (item[0], str(item[1].get("controller"))))


def _metric_leaders(records):
    leaders = {}
    for metric in METRIC_DESCRIPTIONS:
        leaders[metric] = min(records, key=lambda record: _as_float(record.get(metric)))
    return leaders


def _reason_count(record, reason):
    return int(_as_float(record.get(f"speed_plan_reason_{reason}_count")))


def _dominant_speed_reasons(record):
    counts = [(reason, _reason_count(record, reason)) for reason in SPEED_PLAN_REASONS]
    counts = [(reason, count) for reason, count in counts if count > 0]
    if not counts:
        return "没有记录到速度规划触发原因"

    counts.sort(key=lambda item: item[1], reverse=True)
    total = sum(count for _, count in counts)
    parts = []
    for reason, count in counts[:3]:
        percent = 100.0 * count / total if total else 0.0
        parts.append(f"{reason} {count} 次 ({percent:.1f}%，{REASON_DESCRIPTIONS.get(reason, reason)})")
    return "；".join(parts)


def _build_conditions_section(first_record, run_config):
    route_config = run_config.get("route", {}) if isinstance(run_config, dict) else {}
    length_m = route_config.get("length_m", first_record.get("route_min_length_m"))
    route_label = route_config.get("route_label", first_record.get("route_label"))
    total_turn = route_config.get("total_abs_turn_deg", first_record.get("route_total_abs_turn_deg"))
    mean_curvature = route_config.get("mean_abs_curvature", first_record.get("route_mean_abs_curvature"))
    max_curvature = route_config.get("max_abs_curvature", first_record.get("route_max_abs_curvature"))
    seed = run_config.get("seed", "") if isinstance(run_config, dict) else ""
    destination_index = run_config.get("destination_index", "") if isinstance(run_config, dict) else ""

    return [
        "测试条件",
        f"- 路线类型：{first_record.get('route_shape')} / {route_label}",
        f"- 路线长度：约 {_format_float(length_m, 1, ' m')}，路径点 {first_record.get('route_waypoints')} 个",
        f"- 弯道强度：总转角 {_format_float(total_turn, 1, ' deg')}，平均曲率 {_format_float(mean_curvature, 4)}，最大曲率 {_format_float(max_curvature, 4)}",
        f"- 难度判断：{_route_difficulty(first_record)}",
        f"- 目标速度：{_format_float(first_record.get('target_speed_kmh'), 1, ' km/h')}",
        f"- 速度规划：{_speed_planner_label(first_record.get('speed_planner_mode'))}",
        f"- 误差输入：{_error_provider_label(first_record.get('error_provider'))}",
        f"- 感知条件：横向噪声 {_format_float(first_record.get('noise_lateral_std'), 3, ' m')}，航向噪声 {_format_float(first_record.get('noise_heading_std_deg'), 2, ' deg')}，延迟 {first_record.get('perception_delay_steps')} 步，丢帧概率 {_format_probability(first_record.get('perception_dropout_probability'))}，平滑系数 {_format_float(first_record.get('perception_smoothing_alpha'), 2)}",
        f"- 起点/终点索引：spawn {first_record.get('spawn_index')}，destination {destination_index}",
        f"- 随机种子：{seed}",
    ]


def _build_ranking_section(scored_records):
    lines = [
        "控制器排名",
        "评分说明：分数越低越好；评分综合横向偏差、航向偏差、速度误差、转向平顺性和最大偏离。",
    ]
    for rank, (score, record) in enumerate(scored_records, start=1):
        lines.append(
            f"{rank}. {_controller_label(record.get('controller'))}：综合分 {score:.3f}，"
            f"平均横向偏差 {_format_float(record.get('mean_abs_e_y'), 3, ' m')}，"
            f"平均航向偏差 {_format_float(record.get('mean_abs_e_psi_deg'), 2, ' deg')}，"
            f"平均速度误差 {_format_float(record.get('mean_abs_speed_error'), 3, ' m/s')}，"
            f"平均转向变化 {_format_float(record.get('mean_abs_steer_delta'), 4)}"
        )
    return lines


def _build_best_controller_section(best_record, records):
    leaders = _metric_leaders(records)
    best_name = _controller_label(best_record.get("controller"))
    won_metrics = [
        METRIC_DESCRIPTIONS[metric][0]
        for metric, leader in leaders.items()
        if leader.get("controller") == best_record.get("controller")
    ]
    if won_metrics:
        reason = "，".join(won_metrics[:4])
        if len(won_metrics) > 4:
            reason += f"等 {len(won_metrics)} 项"
    else:
        reason = "各项指标整体更均衡"

    return [
        "结论",
        f"本次测试中表现最好的是 {best_name}。",
        f"主要原因：{best_name} 在 {reason} 上表现领先或更稳定。",
        f"通俗理解：它更少偏离路线，车头更接近道路方向，速度跟踪和方向盘变化也更平顺。",
    ]


def _build_detail_section(records):
    lines = ["关键指标解释"]
    for record in records:
        lines.append(
            f"- {_controller_label(record.get('controller'))}："
            f"平均偏线 {_format_float(record.get('mean_abs_e_y'), 3, ' m')}，"
            f"最大偏线 {_format_float(record.get('max_abs_e_y'), 3, ' m')}，"
            f"平均航向偏差 {_format_float(record.get('mean_abs_e_psi_deg'), 2, ' deg')}，"
            f"平均速度误差 {_format_float(record.get('mean_abs_speed_error'), 3, ' m/s')}，"
            f"动态目标速度范围 {_format_float(record.get('min_planned_speed_kmh'), 1, ' km/h')} - {_format_float(record.get('max_planned_speed_kmh'), 1, ' km/h')}；"
            f"速度规划触发：{_dominant_speed_reasons(record)}"
        )
    return lines


def build_human_summary(summaries, run_config):
    records = _summary_records(summaries)
    if not records:
        return ["本次运行没有生成有效的控制器结果，无法判断最佳控制方式。"]

    scored_records = _score_summaries(records)
    best_record = scored_records[0][1]
    lines = []
    lines.extend(_build_best_controller_section(best_record, records))
    lines.append("")
    lines.extend(_build_conditions_section(records[0], run_config))
    lines.append("")
    lines.extend(_build_ranking_section(scored_records))
    lines.append("")
    lines.extend(_build_detail_section(records))
    lines.append("")
    lines.append("原始数据")
    lines.append("- step_log.csv：逐控制周期数据，适合画图或做细节追踪")
    lines.append("- summary.csv：每个控制器的统计指标")
    lines.append("- run_config.json：本次实验参数和路线信息")
    return lines


def plot_compare_trajectories(trajectories, controller_order, run_output_dir):
    try:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, len(controller_order), figsize=(7 * len(controller_order), 6))
        if len(controller_order) == 1:
            axes = [axes]
        plot_specs = [(name, f"{name.upper()} Trajectory", axes[idx]) for idx, name in enumerate(controller_order)]
        for name, title, ax in plot_specs:
            xs, ys = trajectories.get(name, ([], []))
            ax.plot(xs, ys, "-o", markersize=2, linewidth=1.2)
            ax.set_title(title)
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.axis("equal")
            ax.grid(True)
        plt.suptitle("Vehicle Trajectory Comparison")
        plt.tight_layout()
        figure_path = os.path.join(run_output_dir, "trajectory_compare.png")
        plt.savefig(figure_path, dpi=150)
        print(f"Trajectory comparison figure saved to: {figure_path}")
        plt.close(fig)
    except Exception as exc:
        print(f"Could not display trajectory comparison plot: {exc}")


def write_json(path, payload):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)


def save_compare_outputs(run_output_dir, all_rows, summaries, trajectories, controller_order, run_config):
    os.makedirs(run_output_dir, exist_ok=True)
    compare_path = os.path.join(run_output_dir, "step_log.csv")
    write_csv(compare_path, LOG_HEADER, all_rows)
    print(f"Step log saved to: {compare_path}")

    summary_path = os.path.join(run_output_dir, "summary.csv")
    write_csv(summary_path, SUMMARY_HEADER, summaries)
    print(f"Comparison summary saved to: {summary_path}")

    config_path = os.path.join(run_output_dir, "run_config.json")
    write_json(config_path, run_config)
    print(f"Run config saved to: {config_path}")

    human_summary_path = os.path.join(run_output_dir, HUMAN_SUMMARY_FILENAME)
    human_summary = build_human_summary(summaries, run_config)
    write_text(human_summary_path, human_summary)
    print(f"Human-readable summary saved to: {human_summary_path}")
    print("")
    print("\n".join(human_summary[:4]))

    if trajectories:
        plot_compare_trajectories(trajectories, controller_order, run_output_dir)
