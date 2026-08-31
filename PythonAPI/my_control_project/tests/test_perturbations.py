import importlib.util
import os
import sys
import unittest
from types import SimpleNamespace


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

MODULE_PATH = os.path.join(PROJECT_ROOT, "experiment", "perturbations.py")
SPEC = importlib.util.spec_from_file_location("perturbations_under_test", MODULE_PATH)
PERTURBATIONS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PERTURBATIONS)

apply_vehicle_perturbation = PERTURBATIONS.apply_vehicle_perturbation
generate_perturbation = PERTURBATIONS.generate_perturbation
validate_perturbation = PERTURBATIONS.validate_perturbation

from project_config import CLI_ARGUMENTS


class FakeVehicle:
    def __init__(self):
        self.physics = SimpleNamespace(
            mass=1600.0,
            moi=2400.0,
            wheels=[SimpleNamespace(tire_friction=2.0) for _ in range(4)],
        )
        self.applied = None

    def get_physics_control(self):
        return self.physics

    def apply_physics_control(self, physics):
        self.applied = physics


class PerturbationTest(unittest.TestCase):
    def test_same_seed_generates_identical_versioned_manifest(self):
        first = generate_perturbation(42)
        second = generate_perturbation(42)

        self.assertEqual(first, second)
        self.assertEqual(first["version"], 1)
        self.assertEqual(first["seed"], 42)

    def test_generated_scales_respect_declared_bounds(self):
        for seed in range(100):
            item = generate_perturbation(seed)
            self.assertGreaterEqual(item["vehicle_mass_scale"], 0.8)
            self.assertLessEqual(item["vehicle_mass_scale"], 1.2)
            self.assertGreaterEqual(item["vehicle_moi_scale"], 0.8)
            self.assertLessEqual(item["vehicle_moi_scale"], 1.2)
            self.assertGreaterEqual(item["tire_friction_scale"], 0.5)
            self.assertLessEqual(item["tire_friction_scale"], 1.17)
            self.assertGreaterEqual(item["perception_delay_steps"], 0)
            self.assertLessEqual(item["perception_delay_steps"], 4)
            self.assertGreaterEqual(item["perception_dropout_probability"], 0.0)
            self.assertLessEqual(item["perception_dropout_probability"], 0.1)

    def test_validation_rejects_out_of_range_scale(self):
        item = generate_perturbation(1)
        item["vehicle_mass_scale"] = 2.0

        with self.assertRaisesRegex(ValueError, "vehicle_mass_scale"):
            validate_perturbation(item)

    def test_apply_vehicle_perturbation_returns_absolute_audit_values(self):
        vehicle = FakeVehicle()
        config = {
            "vehicle_mass_scale": 1.1,
            "vehicle_moi_scale": 0.9,
            "tire_friction_scale": 0.5,
        }

        audit = apply_vehicle_perturbation(vehicle, config)

        self.assertIs(vehicle.applied, vehicle.physics)
        self.assertAlmostEqual(vehicle.physics.mass, 1760.0)
        self.assertAlmostEqual(vehicle.physics.moi, 2160.0)
        self.assertTrue(all(wheel.tire_friction == 1.0 for wheel in vehicle.physics.wheels))
        self.assertEqual(audit["nominal_mass"], 1600.0)
        self.assertEqual(audit["applied_mass"], 1760.0)
        self.assertEqual(audit["nominal_tire_friction"], [2.0] * 4)
        self.assertEqual(audit["applied_tire_friction"], [1.0] * 4)

    def test_cli_declares_evaluation_and_physics_arguments(self):
        flags = {flag for argument_flags, _ in CLI_ARGUMENTS for flag in argument_flags}

        self.assertIn("--evaluation-case-id", flags)
        self.assertIn("--vehicle-mass-scale", flags)
        self.assertIn("--vehicle-moi-scale", flags)
        self.assertIn("--tire-friction-scale", flags)


if __name__ == "__main__":
    unittest.main()
