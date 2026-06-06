import csv
import json
import os

import numpy as np

from .metrics import EXPERIMENT_METADATA_HEADER, build_experiment_metadata


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
    "mean_abs_e_y", "rms_e_y", "max_abs_e_y",
    "mean_abs_e_psi_deg", "rms_e_psi_deg", "mean_abs_speed_error",
    "mean_abs_steer_delta", "max_abs_steer_delta",
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

    if trajectories:
        plot_compare_trajectories(trajectories, controller_order, run_output_dir)
