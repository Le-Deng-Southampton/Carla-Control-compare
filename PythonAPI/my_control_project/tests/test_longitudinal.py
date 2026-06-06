import os
import sys
import unittest
import importlib.util


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

LONGITUDINAL_PATH = os.path.join(PROJECT_ROOT, "control", "longitudinal.py")
spec = importlib.util.spec_from_file_location("longitudinal", LONGITUDINAL_PATH)
longitudinal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(longitudinal)
PidLongitudinalController = longitudinal.PidLongitudinalController


class PidLongitudinalControllerTest(unittest.TestCase):
    def test_default_target_speed_is_project_baseline(self):
        controller = PidLongitudinalController()

        self.assertAlmostEqual(controller.target_speed_mps, 30.0 / 3.6)

    def test_accelerates_when_vehicle_is_below_target_speed(self):
        controller = PidLongitudinalController(
            target_speed_kmh=36.0,
            kp=0.5,
            ki=0.0,
            kd=0.0,
            dt=0.1,
            max_throttle=1.0,
            max_brake=1.0,
        )

        throttle, brake = controller.run_step(speed_mps=8.0)

        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)

    def test_brakes_when_vehicle_is_above_target_speed(self):
        controller = PidLongitudinalController(
            target_speed_kmh=36.0,
            kp=0.5,
            ki=0.0,
            kd=0.0,
            dt=0.1,
            max_throttle=1.0,
            max_brake=1.0,
        )

        throttle, brake = controller.run_step(speed_mps=12.0)

        self.assertEqual(throttle, 0.0)
        self.assertGreater(brake, 0.0)

    def test_run_step_accepts_dynamic_target_speed_for_current_step(self):
        controller = PidLongitudinalController(
            target_speed_kmh=36.0,
            kp=0.5,
            ki=0.0,
            kd=0.0,
            dt=0.1,
            max_throttle=1.0,
            max_brake=1.0,
        )

        throttle, brake = controller.run_step(speed_mps=8.0, target_speed_mps=6.0)

        self.assertEqual(throttle, 0.0)
        self.assertGreater(brake, 0.0)
        self.assertAlmostEqual(controller.target_speed_mps, 36.0 / 3.6)

    def test_reset_clears_pid_state(self):
        controller = PidLongitudinalController(
            target_speed_kmh=36.0,
            kp=0.5,
            ki=0.1,
            kd=0.0,
            dt=0.1,
            max_throttle=1.0,
            max_brake=1.0,
        )
        controller.run_step(speed_mps=8.0)
        expected_after_reset = controller.run_step(speed_mps=9.0)

        controller.reset()
        actual_after_reset = controller.run_step(speed_mps=9.0)

        self.assertNotEqual(actual_after_reset, expected_after_reset)
        self.assertEqual(actual_after_reset, (0.51, 0.0))


if __name__ == "__main__":
    unittest.main()
