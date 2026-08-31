import math
import os
import sys
import unittest
import importlib.util

import numpy as np


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

MODULE_PATH = os.path.join(PROJECT_ROOT, "experiment", "evaluation_metrics.py")
SPEC = importlib.util.spec_from_file_location("evaluation_metrics_under_test", MODULE_PATH)
EVALUATION_METRICS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVALUATION_METRICS)

evaluate_step_series = EVALUATION_METRICS.evaluate_step_series
integral_absolute_error = EVALUATION_METRICS.integral_absolute_error
moving_average_peak = EVALUATION_METRICS.moving_average_peak
runtime_statistics = EVALUATION_METRICS.runtime_statistics
settling_time = EVALUATION_METRICS.settling_time
spectral_steering_indices = EVALUATION_METRICS.spectral_steering_indices
time_normalized_iae = EVALUATION_METRICS.time_normalized_iae


class EvaluationMetricsTest(unittest.TestCase):
    def test_integral_and_normalized_iae_use_real_timestamps(self):
        times = [0.0, 0.1, 0.3]
        values = [1.0, 1.0, 1.0]

        self.assertAlmostEqual(integral_absolute_error(times, values), 0.3)
        self.assertAlmostEqual(time_normalized_iae(times, values), 1.0)

    def test_settling_time_requires_continuous_two_second_hold(self):
        times = np.arange(0.0, 4.05, 0.05)
        lateral = np.where(times < 1.0, 0.2, 0.05)
        heading = np.where(times < 1.0, math.radians(2.0), math.radians(0.5))

        self.assertAlmostEqual(settling_time(times, lateral, heading), 1.0)

    def test_settling_time_returns_none_when_hold_breaks(self):
        times = np.arange(0.0, 4.05, 0.05)
        lateral = np.full(times.size, 0.05)
        lateral[30] = 0.2
        lateral[70] = 0.2
        heading = np.zeros(times.size)

        self.assertIsNone(settling_time(times, lateral, heading))

    def test_spectral_indices_separate_two_and_six_hz(self):
        times = np.arange(0.0, 20.0, 0.05)
        low = spectral_steering_indices(times, np.sin(2.0 * np.pi * 2.0 * times))
        high = spectral_steering_indices(times, np.sin(2.0 * np.pi * 6.0 * times))

        self.assertTrue(low["spectral_metrics_valid"])
        self.assertTrue(high["spectral_metrics_valid"])
        self.assertGreater(low["stability_band_peak_power"], low["comfort_band_peak_power"])
        self.assertGreater(high["comfort_band_peak_power"], high["stability_band_peak_power"])

    def test_frequency_metric_rejects_gap_larger_than_one_cycle(self):
        times = np.r_[np.arange(0.0, 5.0, 0.05), np.arange(5.2, 10.0, 0.05)]
        result = spectral_steering_indices(times, np.zeros(times.size))

        self.assertFalse(result["spectral_metrics_valid"])
        self.assertEqual(result["spectral_metrics_reason"], "timestamp_gap")

    def test_moving_average_peak_uses_requested_window(self):
        values = [0.0, 10.0, 0.0, 0.0]

        self.assertAlmostEqual(moving_average_peak(values, window_samples=2), 5.0)

    def test_runtime_statistics_reports_deadline_misses(self):
        result = runtime_statistics([1.0, 10.0, 55.0, 100.0], deadline_ms=50.0)

        self.assertEqual(result["controller_runtime_deadline_miss_count"], 2)
        self.assertAlmostEqual(result["controller_runtime_deadline_miss_pct"], 50.0)
        self.assertAlmostEqual(result["controller_runtime_p50_ms"], 32.5)
        self.assertAlmostEqual(result["controller_runtime_max_ms"], 100.0)

    def test_evaluate_step_series_exposes_accuracy_comfort_and_runtime(self):
        times = np.arange(0.0, 10.0, 0.05)
        metrics = {
            "time": times,
            "e_y_signed": np.full(times.size, 0.1),
            "e_psi_signed": np.full(times.size, math.radians(0.5)),
            "speed_error": np.full(times.size, 1.0),
            "steer_signed": np.zeros(times.size),
            "lateral_accel_signed": np.full(times.size, 2.0),
            "longitudinal_accel": np.zeros(times.size),
            "controller_runtime_ms": np.full(times.size, 5.0),
        }

        result = evaluate_step_series(metrics)

        self.assertAlmostEqual(result["iae_e_y_m_s"], 0.995)
        self.assertAlmostEqual(result["normalized_iae_e_y_m"], 0.1)
        self.assertAlmostEqual(result["rms_e_psi_deg"], 0.5)
        self.assertAlmostEqual(result["controller_runtime_p95_ms"], 5.0)
        self.assertAlmostEqual(result["max_abs_lateral_accel_0p5s"], 2.0)

    def test_evaluate_step_series_coalesces_terminal_duplicate_timestamp(self):
        metrics = {
            "time": [0.0, 0.05, 0.10, 0.10],
            "e_y_signed": [0.0, 0.1, 0.2, 1.0],
            "e_psi_signed": [0.0, 0.0, 0.0, 0.0],
            "speed_error": [0.0, 0.0, 0.0, 0.0],
            "steer_signed": [0.0, 0.0, 0.0, 1.0],
            "lateral_accel_signed": [0.0, 0.0, 0.0, 10.0],
            "longitudinal_accel": [0.0, 0.0, 0.0, 0.0],
            "controller_runtime_ms": [1.0, 1.0, 1.0, 100.0],
        }

        result = evaluate_step_series(metrics)

        self.assertAlmostEqual(result["normalized_iae_e_y_m"], 0.1)
        self.assertAlmostEqual(result["controller_runtime_max_ms"], 1.0)

    def test_evaluate_step_series_rejects_time_moving_backwards(self):
        with self.assertRaisesRegex(ValueError, "must not move backwards"):
            evaluate_step_series({"time": [0.0, 0.1, 0.05]})


if __name__ == "__main__":
    unittest.main()
