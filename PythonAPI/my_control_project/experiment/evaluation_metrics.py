# uncompyle6 version 3.9.3
# Python bytecode version base 3.7.0 (3394)
# Decompiled from: Python 3.13.13 | packaged by Anaconda, Inc. | (main, Apr 14 2026, 06:12:50) [MSC v.1942 64 bit (AMD64)]
# Embedded file name: D:\WindowsNoEditor\PythonAPI\my_control_project\experiment\evaluation_metrics.py
# Compiled at: 2026-07-13 20:21:32
# Size of source mod 2**32: 9796 bytes
import math, numpy as np
from scipy import signal
DEFAULT_SAMPLE_HZ = 20.0
SPECTRAL_WINDOW_SECONDS = 5.0
EVALUATION_SUMMARY_FIELDS = [
 'iae_e_y_m_s', 
 'normalized_iae_e_y_m', 
 'p95_abs_e_y', 
 'signed_mean_e_y', 
 'iae_e_psi_rad_s', 
 'normalized_iae_e_psi_deg', 
 'p95_abs_e_psi_deg', 
 'max_abs_e_psi_deg', 
 'iae_speed_error_m', 
 'normalized_iae_speed_error_mps', 
 'rms_speed_error_mps', 
 'initial_settling_time_s', 
 'initial_settled', 
 'spectral_metrics_valid', 
 'spectral_metrics_reason', 
 'stability_band_peak_power', 
 'comfort_band_peak_power', 
 'steer_spectrum_stability_index', 
 'steer_spectrum_comfort_index', 
 'max_abs_lateral_accel_0p5s', 
 'max_abs_lateral_jerk_0p5s', 
 'controller_runtime_mean_ms', 
 'controller_runtime_p50_ms', 
 'controller_runtime_p95_ms', 
 'controller_runtime_p99_ms', 
 'controller_runtime_max_ms', 
 'controller_runtime_deadline_miss_count', 
 'controller_runtime_deadline_miss_pct', 
 'un_r79_reference_passed']

def _as_float_array(values):
    return np.asarray(values, dtype=float).reshape(-1)


def _aligned(times, values):
    times = _as_float_array(times)
    values = _as_float_array(values)
    if times.size != values.size:
        raise ValueError("times and values must have the same length")
    if times.size and not (np.all(np.isfinite(times)) and np.all(np.isfinite(values))):
        raise ValueError("times and values must be finite")
    if times.size > 1:
        if np.any(np.diff(times) <= 0.0):
            raise ValueError("times must be strictly increasing")
    return (
     times, values)


def _percentile(values, percentile):
    values = _as_float_array(values)
    if values.size:
        return float(np.percentile(values, percentile))
    return 0.0


def _rms(values):
    values = _as_float_array(values)
    if values.size:
        return float(np.sqrt(np.mean(values * values)))
    return 0.0


def integral_absolute_error(times, values):
    times, values = _aligned(times, values)
    if times.size < 2:
        return 0.0
    widths = np.diff(times)
    heights = 0.5 * (np.abs(values[:-1]) + np.abs(values[1:]))
    return float(np.sum(widths * heights))


def time_normalized_iae(times, values):
    times, values = _aligned(times, values)
    if times.size < 2:
        return 0.0
    duration = float(times[-1] - times[0])
    if duration > 0.0:
        return integral_absolute_error(times, values) / duration
    return 0.0


def settling_time(times, lateral_error, heading_error, hold_seconds=2.0, lateral_limit_m=0.1, heading_limit_rad=math.radians(1.0)):
    times, lateral_error = _aligned(times, lateral_error)
    heading_times, heading_error = _aligned(times, heading_error)
    if not np.array_equal(times, heading_times):
        raise ValueError("lateral and heading errors must use identical timestamps")
    inside = (np.abs(lateral_error) <= float(lateral_limit_m)) & (np.abs(heading_error) <= float(heading_limit_rad))
    for start in np.flatnonzero(inside):
        required_end = times[start] + float(hold_seconds)
        end = int(np.searchsorted(times, required_end, side="left"))
        if end < times.size and np.all(inside[start:end + 1]):
            return float(times[start] - times[0])

    return


def _invalid_spectral_result(reason):
    return {
     'spectral_metrics_valid': False, 
     'spectral_metrics_reason': reason, 
     'stability_band_peak_power': 0.0, 
     'comfort_band_peak_power': 0.0, 
     'steer_spectrum_stability_index': 0.0, 
     'steer_spectrum_comfort_index': 0.0}


def spectral_steering_indices(times, steer, sample_hz=DEFAULT_SAMPLE_HZ):
    times, steer = _aligned(times, steer)
    sample_hz = float(sample_hz)
    if sample_hz <= 0.0:
        raise ValueError("sample_hz must be positive")
    window_samples = int(round(SPECTRAL_WINDOW_SECONDS * sample_hz))
    if times.size < window_samples:
        return _invalid_spectral_result("insufficient_duration")
    expected_dt = 1.0 / sample_hz
    if np.any(np.diff(times) > expected_dt * 1.05):
        return _invalid_spectral_result("timestamp_gap")
    grid = np.arange(times[0], times[-1] + expected_dt * 0.5, expected_dt)
    samples = np.interp(grid, times, steer)
    if samples.size < window_samples:
        return _invalid_spectral_result("insufficient_duration")
    frequencies, _, spectrum = signal.stft(samples,
      fs=sample_hz,
      window="hann",
      nperseg=window_samples,
      noverlap=(window_samples // 2),
      boundary=None,
      padded=False)
    power = np.abs(spectrum) ** 2
    stability_power = power[(frequencies >= 1.1) & (frequencies < 4.0)]
    comfort_power = power[(frequencies >= 4.0) & (frequencies <= 10.0)]
    stability_peaks = np.max(stability_power, axis=0) if stability_power.size else np.zeros(power.shape[1])
    comfort_peaks = np.max(comfort_power, axis=0) if comfort_power.size else np.zeros(power.shape[1])
    tiny = np.finfo(float).tiny
    stability_db = 10.0 * np.log10(np.maximum(stability_peaks, tiny))
    comfort_db = 10.0 * np.log10(np.maximum(comfort_peaks, tiny))
    stability_index = 0.015 * np.maximum(stability_db + 80.0, 0.0)
    comfort_index = 0.04 * np.maximum(comfort_db + 80.0, 0.0)
    return {'spectral_metrics_valid':True, 
     'spectral_metrics_reason':"", 
     'stability_band_peak_power':float(np.mean(stability_peaks)), 
     'comfort_band_peak_power':float(np.max(comfort_peaks)), 
     'steer_spectrum_stability_index':float(np.mean(stability_index)), 
     'steer_spectrum_comfort_index':float(np.max(comfort_index))}


def moving_average_peak(values, window_samples):
    values = _as_float_array(values)
    window_samples = max(int(window_samples), 1)
    if not values.size:
        return 0.0
    if values.size < window_samples:
        return float(abs(np.mean(values)))
    averaged = np.convolve(values, (np.ones(window_samples) / window_samples), mode="valid")
    return float(np.max(np.abs(averaged)))


def runtime_statistics(runtime_ms, deadline_ms=50.0):
    values = _as_float_array(runtime_ms)
    misses = values > float(deadline_ms)
    return {'controller_runtime_mean_ms':float(np.mean(values)) if (values.size) else 0.0, 
     'controller_runtime_p50_ms':_percentile(values, 50), 
     'controller_runtime_p95_ms':_percentile(values, 95), 
     'controller_runtime_p99_ms':_percentile(values, 99), 
     'controller_runtime_max_ms':float(np.max(values)) if (values.size) else 0.0, 
     'controller_runtime_deadline_miss_count':int(np.sum(misses)), 
     'controller_runtime_deadline_miss_pct':100.0 * float(np.mean(misses)) if (values.size) else 0.0}


def _series(metrics, name, count):
    values = _as_float_array(metrics.get(name, []))
    if values.size == count:
        return values
    return np.zeros(count, dtype=float)


def _jerk(times, acceleration):
    if times.size < 2:
        return np.array([], dtype=float)
    return np.diff(acceleration) / np.diff(times)


def evaluate_step_series(metrics, sample_hz=DEFAULT_SAMPLE_HZ, deadline_ms=50.0):
    times = _as_float_array(metrics.get("time", []))
    count = times.size
    if count > 1:
        time_steps = np.diff(times)
        if np.any(time_steps < 0.0):
            reset_index = int(np.flatnonzero(time_steps < 0.0)[0]) + 1
            raise ValueError(
                "metric timestamps must not move backwards: "
                f"sample {reset_index - 1}={times[reset_index - 1]:.6f}, "
                f"sample {reset_index}={times[reset_index]:.6f}"
            )
    lateral_error = _series(metrics, "e_y_signed", count)
    heading_error = _series(metrics, "e_psi_signed", count)
    speed_error = _series(metrics, "speed_error", count)
    steer = _series(metrics, "steer_signed", count)
    lateral_accel = _series(metrics, "lateral_accel_signed", count)
    longitudinal_accel = _series(metrics, "longitudinal_accel", count)
    runtime_ms = _series(metrics, "controller_runtime_ms", count)
    if count > 1 and np.any(time_steps == 0.0):
        # A terminal event can append one final sample at the current simulator
        # timestamp.  Keep the first sample for each timestamp so failed laps
        # still produce a summary without creating a zero-duration jerk step.
        keep = np.r_[True, time_steps > 0.0]
        times = times[keep]
        lateral_error = lateral_error[keep]
        heading_error = heading_error[keep]
        speed_error = speed_error[keep]
        steer = steer[keep]
        lateral_accel = lateral_accel[keep]
        longitudinal_accel = longitudinal_accel[keep]
        runtime_ms = runtime_ms[keep]
        count = times.size
    half_second_samples = max(int(round(0.5 * float(sample_hz))), 1)
    lateral_jerk = _jerk(times, lateral_accel)
    longitudinal_jerk = _jerk(times, longitudinal_accel)
    settle = settling_time(times, lateral_error, heading_error) if count else None
    result = {'iae_e_y_m_s':integral_absolute_error(times, lateral_error), 
     'normalized_iae_e_y_m':time_normalized_iae(times, lateral_error), 
     'p95_abs_e_y':_percentile(np.abs(lateral_error), 95), 
     'signed_mean_e_y':float(np.mean(lateral_error)) if count else 0.0, 
     'iae_e_psi_rad_s':integral_absolute_error(times, heading_error), 
     'normalized_iae_e_psi_deg':(math.degrees)(time_normalized_iae(times, heading_error)), 
     'rms_e_psi_deg':(math.degrees)(_rms(heading_error)), 
     'p95_abs_e_psi_deg':(math.degrees)(_percentile(np.abs(heading_error), 95)), 
     'max_abs_e_psi_deg':(math.degrees)(float(np.max(np.abs(heading_error))) if count else 0.0), 
     'iae_speed_error_m':integral_absolute_error(times, speed_error), 
     'normalized_iae_speed_error_mps':time_normalized_iae(times, speed_error), 
     'rms_speed_error_mps':_rms(speed_error), 
     'initial_settling_time_s':settle if (settle is not None) else (-1.0), 
     'initial_settled':settle is not None, 
     'max_abs_lateral_accel_0p5s':moving_average_peak(lateral_accel, half_second_samples), 
     'max_abs_lateral_jerk_0p5s':moving_average_peak(lateral_jerk, half_second_samples), 
     'p95_abs_longitudinal_jerk':_percentile(np.abs(longitudinal_jerk), 95)}
    result.update(spectral_steering_indices(times, steer, sample_hz=sample_hz))
    result.update(runtime_statistics(runtime_ms, deadline_ms=deadline_ms))
    result["un_r79_reference_passed"] = result["max_abs_lateral_accel_0p5s"] <= 3.0 and result["max_abs_lateral_jerk_0p5s"] <= 5.0
    return result
