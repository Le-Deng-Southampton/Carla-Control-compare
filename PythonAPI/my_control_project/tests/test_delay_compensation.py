import os
import sys
import types
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYTHONAPI_ROOT = os.path.dirname(PROJECT_ROOT)
CARLA_AGENTS_ROOT = os.path.join(PYTHONAPI_ROOT, "carla")
for path in (PROJECT_ROOT, CARLA_AGENTS_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

if "carla" not in sys.modules:
    carla = types.ModuleType("carla")
    carla.VehicleControl = type("VehicleControl", (), {})
    sys.modules["carla"] = carla

from control.delay_compensation import compensate_tracking_errors


class DelayCompensationTest(unittest.TestCase):
    def test_zero_delay_preserves_errors_without_mutating_input(self):
        errors = {"e_y": 1.25, "e_psi": -0.20, "reference_yaw": 0.4}

        compensated = compensate_tracking_errors(
            errors,
            e_y_dot=3.0,
            e_psi_dot=0.5,
            delay_steps=0,
            dt=0.05,
        )

        self.assertEqual(compensated, errors)
        self.assertIsNot(compensated, errors)

    def test_two_steps_projects_lateral_and_heading_errors_forward(self):
        compensated = compensate_tracking_errors(
            {"e_y": 1.0, "e_psi": -0.10},
            e_y_dot=2.5,
            e_psi_dot=-0.40,
            delay_steps=2,
            dt=0.05,
        )

        self.assertAlmostEqual(compensated["e_y"], 1.25)
        self.assertAlmostEqual(compensated["e_psi"], -0.14)


if __name__ == "__main__":
    unittest.main()
