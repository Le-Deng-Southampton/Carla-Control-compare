# uncompyle6 version 3.9.3
# Python bytecode version base 3.7.0 (3394)
# Decompiled from: Python 3.13.13 | packaged by Anaconda, Inc. | (main, Apr 14 2026, 06:12:50) [MSC v.1942 64 bit (AMD64)]
# Embedded file name: D:\WindowsNoEditor\PythonAPI\my_control_project\experiment\stability.py
# Compiled at: 2026-07-13 17:39:49
# Size of source mod 2**32: 8752 bytes
import numpy as np
SIGNIFICANT_STEER_DELTA = 0.01
ANALYSIS_START_SECONDS = 5.0
ANALYSIS_END_MARGIN_SECONDS = 2.0
MIN_ANALYSIS_SPEED_MPS = 2.7777777777777777
SMOOTHING_WINDOW_SAMPLES = 5

def _as_array(metrics, name, count, default=0.0):
    values = metrics.get(name, [])
    if len(values) == count:
        return np.asarray(values, dtype=float)
    return np.full(count, (float(default)), dtype=float)


def _percentile(values, percentile):
    if len(values):
        return float(np.percentile(values, percentile))
    return 0.0


def _moving_average(values, window=SMOOTHING_WINDOW_SAMPLES):
    values = np.asarray(values, dtype=float)
    if values.size < window or window <= 1:
        return values.copy()
    left = window // 2
    right = window - left - 1
    padded = np.pad(values, (left, right), mode="edge")
    return np.convolve(padded, (np.ones(window) / window), mode="valid")


def _jerk(values, times):
    values = _moving_average(values)
    times = np.asarray(times, dtype=float)
    if values.size < 2 or times.size < 2:
        return np.array([], dtype=float)
    dt = np.diff(times)
    valid = dt > 1e-09
    result = np.zeros((values.size - 1), dtype=float)
    result[valid] = np.diff(values)[valid] / dt[valid]
    return result[valid]


def _significant_reversal_times(times, steer_delta):
    if len(steer_delta) < 2:
        return np.array([], dtype=float)
    previous = steer_delta[:-1]
    current = steer_delta[1:]
    significant = (np.abs(previous) >= SIGNIFICANT_STEER_DELTA) & (np.abs(current) >= SIGNIFICANT_STEER_DELTA) & (previous * current < 0.0)
    return times[1:][significant]


def _max_events_in_window_details(event_times, window_seconds):
    if len(event_times) == 0:
        return (0, None, None)
    maximum = 0
    maximum_start = None
    maximum_end = None
    left = 0
    for right, event_time in enumerate(event_times):
        while event_time - event_times[left] > window_seconds:
            left += 1

        count = right - left + 1
        if count > maximum:
            maximum = count
            maximum_start = float(event_times[left])
            maximum_end = float(event_time)

    return (
     int(maximum), maximum_start, maximum_end)


def _pedal_switch_rate(throttle, brake, duration):
    last_mode = None
    switches = 0
    for throttle_value, brake_value in zip(throttle, brake):
        mode = None
        if throttle_value > 0.1 and brake_value <= 0.05:
            mode = "throttle"
        else:
            if brake_value > 0.05:
                if throttle_value <= 0.1:
                    mode = "brake"
            elif mode is None:
                continue
            if last_mode is not None and mode != last_mode:
                switches += 1
            last_mode = mode

    return 10.0 * switches / max(float(duration), 1e-09)


def _analysis_indices(metrics):
    times = np.asarray((metrics.get("time", [])), dtype=float)
    if times.size == 0:
        return (
         times, np.array([], dtype=int))
    speed = _as_array(metrics, "speed", times.size)
    end_time = times[-1] - ANALYSIS_END_MARGIN_SECONDS
    mask = (times >= times[0] + ANALYSIS_START_SECONDS) & (times <= end_time) & (speed >= MIN_ANALYSIS_SPEED_MPS)
    return (
     times, np.flatnonzero(mask))


def analyze_stability(metrics, speed_planner_mode, route_shape, error_provider):
    times, indices = _analysis_indices(metrics)
    if indices.size:
        selected_times = times[indices]
    else:
        selected_times = np.array([], dtype=float)
    duration = selected_times[-1] - selected_times[0] if selected_times.size > 1 else 0.0
    count = times.size

    def selected(name, default=0.0):
        values = _as_array(metrics, name, count, default)
        if indices.size:
            return values[indices]
        return np.array([], dtype=float)

    steer_delta = selected("steer_delta_signed")
    reversal_times = _significant_reversal_times(selected_times, steer_delta)
    reversals_per_10s = 10.0 * len(reversal_times) / max(duration, 1e-09) if duration else 0.0
    max_reversals_2s, reversal_window_start, reversal_window_end = _max_events_in_window_details(reversal_times, 2.0)
    speed_error_kmh = np.abs(selected("speed_error")) * 3.6
    lateral_accel = selected("lateral_accel_signed")
    longitudinal_accel = selected("longitudinal_accel")
    lateral_jerk = _jerk(lateral_accel, selected_times)
    longitudinal_jerk = _jerk(longitudinal_accel, selected_times)
    abs_ey = np.abs(selected("abs_ey"))
    throttle = selected("throttle")
    brake = selected("brake")
    rate_limit = np.abs(selected("steer_rate_limit"))
    explicit_hits = selected("steer_rate_limited") > 0.5
    inferred_hits = (rate_limit > 1e-09) & (np.abs(steer_delta) >= 0.95 * rate_limit)
    rate_hit_pct = 100.0 * float(np.mean(explicit_hits | inferred_hits)) if indices.size else 0.0
    tracking_scale = 1.25 if error_provider == "perception_proxy" else 1.0
    curved_route = route_shape in ('s_curve', 'curvy')
    mean_error_limit = (0.5 if curved_route else 0.35) * tracking_scale
    max_error_limit = (1.5 if curved_route else 1.0) * tracking_scale
    speed_mean_limit = 3.0 if speed_planner_mode == "off" else 5.0
    speed_p95_limit = 5.0 if speed_planner_mode == "off" else 8.0
    result = {'p95_abs_steer_delta':_percentile(np.abs(steer_delta), 95), 
     'significant_steer_reversals_per_10s':float(reversals_per_10s), 
     'max_significant_steer_reversals_2s':int(max_reversals_2s), 
     'steer_rate_limit_hit_pct':float(rate_hit_pct), 
     'p95_abs_speed_error':_percentile(speed_error_kmh, 95), 
     'p95_abs_lateral_accel':_percentile(np.abs(lateral_accel), 95), 
     'p95_abs_lateral_jerk':_percentile(np.abs(lateral_jerk), 95), 
     'p95_abs_longitudinal_jerk':_percentile(np.abs(longitudinal_jerk), 95), 
     'throttle_brake_switches_per_10s':float(_pedal_switch_rate(throttle, brake, duration))}
    reasons = []
    route_completion_pct = 100.0 * max(metrics.get("route_completion", [0.0]) or [0.0])
    if route_completion_pct < 98.0:
        reasons.append("route_completion")
    if int(metrics.get("collision_count", 0)) != 0:
        reasons.append("collision")
    if int(metrics.get("lane_boundary_violation_count", 0)) != 0:
        reasons.append("lane_boundary_violation")
    if max_reversals_2s > 4:
        reasons.append("steer_reversals_2s")
    if reversals_per_10s > 5.0:
        reasons.append("steer_reversals_rate")
    if result["p95_abs_steer_delta"] > 0.03:
        reasons.append("steer_delta_p95")
    if rate_hit_pct > 5.0:
        reasons.append("steer_rate_limit_hits")
    if result["p95_abs_lateral_accel"] > 4.0:
        reasons.append("lateral_accel_p95")
    if len(lateral_accel):
        if float(np.max(np.abs(lateral_accel))) > 6.5:
            reasons.append("lateral_accel_max")
    if result["p95_abs_lateral_jerk"] > 10.0:
        reasons.append("lateral_jerk_p95")
    if len(abs_ey):
        if float(np.mean(abs_ey)) > mean_error_limit:
            reasons.append("mean_lateral_error")
    if len(abs_ey):
        if float(np.max(abs_ey)) > max_error_limit:
            reasons.append("max_lateral_error")
    if len(speed_error_kmh):
        if float(np.mean(speed_error_kmh)) > speed_mean_limit:
            reasons.append("mean_speed_error")
    if result["p95_abs_speed_error"] > speed_p95_limit:
        reasons.append("speed_error_p95")
    if result["throttle_brake_switches_per_10s"] > 2.0:
        reasons.append("throttle_brake_switches")
    if result["p95_abs_longitudinal_jerk"] > 5.0:
        reasons.append("longitudinal_jerk_p95")
    if indices.size == 0:
        reasons.append("insufficient_analysis_samples")
    result["stability_passed"] = not reasons
    result["stability_fail_reasons"] = ";".join(reasons)
    analysis_start = float(selected_times[0]) if selected_times.size else 0.0
    analysis_end = float(selected_times[-1]) if selected_times.size else 0.0
    windows = []
    for reason in reasons:
        if reason == "steer_reversals_2s" and reversal_window_start is not None:
            start, end = reversal_window_start, reversal_window_end
        else:
            if reason == "max_lateral_error" and abs_ey.size:
                center = float(selected_times[int(np.argmax(abs_ey))])
                start, end = max(analysis_start, center - 1.0), min(analysis_end, center + 1.0)
            else:
                if reason == "lateral_accel_max" and lateral_accel.size:
                    center = float(selected_times[int(np.argmax(np.abs(lateral_accel)))])
                    start, end = max(analysis_start, center - 1.0), min(analysis_end, center + 1.0)
                else:
                    start, end = analysis_start, analysis_end
        windows.append(f"{reason}@{start:.2f}-{end:.2f}")

    result["stability_fail_windows"] = "|".join(windows)
    return result
