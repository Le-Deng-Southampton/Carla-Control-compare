import os
import sys
import unittest
import importlib.util


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

TERMINATION_PATH = os.path.join(PROJECT_ROOT, "experiment", "termination.py")
spec = importlib.util.spec_from_file_location("termination", TERMINATION_PATH)
termination = importlib.util.module_from_spec(spec)
spec.loader.exec_module(termination)
CollisionZeroSpeedTerminator = termination.CollisionZeroSpeedTerminator


class CollisionZeroSpeedTerminatorTest(unittest.TestCase):
    def test_does_not_terminate_when_zero_speed_happens_without_collision(self):
        terminator = CollisionZeroSpeedTerminator(timeout_seconds=2.0, speed_threshold_mps=0.1)

        self.assertFalse(terminator.update(speed_mps=0.0, sim_time=10.0, collision_count=0))
        self.assertFalse(terminator.update(speed_mps=0.0, sim_time=13.0, collision_count=0))

    def test_does_not_terminate_before_zero_speed_timeout_after_collision(self):
        terminator = CollisionZeroSpeedTerminator(timeout_seconds=2.0, speed_threshold_mps=0.1)

        self.assertFalse(terminator.update(speed_mps=0.0, sim_time=10.0, collision_count=1))
        self.assertFalse(terminator.update(speed_mps=0.0, sim_time=11.9, collision_count=1))

    def test_terminates_after_collision_when_speed_stays_zero_for_timeout(self):
        terminator = CollisionZeroSpeedTerminator(timeout_seconds=2.0, speed_threshold_mps=0.1)

        self.assertFalse(terminator.update(speed_mps=0.0, sim_time=10.0, collision_count=1))

        self.assertTrue(terminator.update(speed_mps=0.0, sim_time=12.0, collision_count=1))

    def test_resets_zero_speed_timer_when_vehicle_moves_again(self):
        terminator = CollisionZeroSpeedTerminator(timeout_seconds=2.0, speed_threshold_mps=0.1)

        self.assertFalse(terminator.update(speed_mps=0.0, sim_time=10.0, collision_count=1))
        self.assertFalse(terminator.update(speed_mps=0.2, sim_time=11.0, collision_count=1))

        self.assertFalse(terminator.update(speed_mps=0.0, sim_time=12.0, collision_count=1))
        self.assertFalse(terminator.update(speed_mps=0.0, sim_time=13.9, collision_count=1))
        self.assertTrue(terminator.update(speed_mps=0.0, sim_time=14.0, collision_count=1))


if __name__ == "__main__":
    unittest.main()
