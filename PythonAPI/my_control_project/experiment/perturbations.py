# uncompyle6 version 3.9.3
# Python bytecode version base 3.7.0 (3394)
# Decompiled from: Python 3.13.13 | packaged by Anaconda, Inc. | (main, Apr 14 2026, 06:12:50) [MSC v.1942 64 bit (AMD64)]
# Embedded file name: D:\WindowsNoEditor\PythonAPI\my_control_project\experiment\perturbations.py
# Compiled at: 2026-07-13 20:32:45
# Size of source mod 2**32: 4126 bytes
"""Deterministic, bounded perturbations for controller robustness evaluation."""
import argparse, json, numpy as np
PERTURBATION_VERSION = 1
PERTURBATION_BOUNDS = {
 'vehicle_mass_scale': (0.8, 1.2), 
 'vehicle_moi_scale': (0.8, 1.2), 
 'tire_friction_scale': (0.5, 1.17), 
 'noise_lateral_std': (0.0, 0.08), 
 'noise_heading_std_deg': (0.0, 1.5), 
 'perception_delay_steps': (0, 4), 
 'perception_dropout_probability': (0.0, 0.1), 
 'perception_smoothing_alpha': (0.25, 1.0)}

def generate_perturbation(seed):
    """Return a reproducible Monte Carlo case within the declared bounds."""
    seed = int(seed)
    rng = np.random.RandomState(seed)
    return {'version':PERTURBATION_VERSION, 
     'seed':seed, 
     'vehicle_mass_scale':float(rng.uniform(0.8, 1.2)), 
     'vehicle_moi_scale':float(rng.uniform(0.8, 1.2)), 
     'tire_friction_scale':float(rng.uniform(0.5, 1.17)), 
     'noise_lateral_std':float(rng.uniform(0.0, 0.08)), 
     'noise_heading_std_deg':float(rng.uniform(0.0, 1.5)), 
     'perception_delay_steps':int(rng.randint(0, 5)), 
     'perception_dropout_probability':float(rng.uniform(0.0, 0.1)), 
     'perception_smoothing_alpha':float(rng.uniform(0.25, 1.0))}


def validate_perturbation(config):
    """Validate a complete generated perturbation and return a normalized copy."""
    normalized = dict(config)
    if int(normalized.get("version", -1)) != PERTURBATION_VERSION:
        raise ValueError(f"version must be {PERTURBATION_VERSION}")
    if "seed" not in normalized:
        raise ValueError("seed is required")
    for name, (lower, upper) in PERTURBATION_BOUNDS.items():
        if name not in normalized:
            raise ValueError(f"{name} is required")
        value = normalized[name]
        numeric = int(value) if name == "perception_delay_steps" else float(value)
        if not numeric < lower:
            if numeric > upper:
                raise ValueError(f"{name} must be within [{lower}, {upper}], got {value}")
            normalized[name] = numeric

    normalized["version"] = PERTURBATION_VERSION
    normalized["seed"] = int(normalized["seed"])
    return normalized


def _validated_scale(config, name, lower, upper):
    value = float(config.get(name, 1.0))
    if value < lower or value > upper:
        raise ValueError(f"{name} must be within [{lower}, {upper}], got {value}")
    return value


def apply_vehicle_perturbation(vehicle, config):
    """Apply mass, yaw inertia and tire-friction scales and return an audit record."""
    mass_scale = _validated_scale(config, "vehicle_mass_scale", 0.8, 1.2)
    moi_scale = _validated_scale(config, "vehicle_moi_scale", 0.8, 1.2)
    friction_scale = _validated_scale(config, "tire_friction_scale", 0.5, 1.17)
    physics = vehicle.get_physics_control()
    nominal_mass = float(physics.mass)
    nominal_moi = float(physics.moi)
    nominal_friction = [float(wheel.tire_friction) for wheel in physics.wheels]
    physics.mass = nominal_mass * mass_scale
    physics.moi = nominal_moi * moi_scale
    for wheel, value in zip(physics.wheels, nominal_friction):
        wheel.tire_friction = value * friction_scale

    vehicle.apply_physics_control(physics)
    return {'vehicle_mass_scale':mass_scale, 
     'vehicle_moi_scale':moi_scale, 
     'tire_friction_scale':friction_scale, 
     'nominal_mass':nominal_mass, 
     'applied_mass':round(float(physics.mass), 12), 
     'nominal_moi':nominal_moi, 
     'applied_moi':round(float(physics.moi), 12), 
     'nominal_tire_friction':nominal_friction, 
     'applied_tire_friction':[round(float(wheel.tire_friction), 12) for wheel in physics.wheels]}


def main():
    parser = argparse.ArgumentParser(description="Generate one deterministic evaluation perturbation.")
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--json", action="store_true", help="Emit the manifest as JSON.")
    args = parser.parse_args()
    print(json.dumps((generate_perturbation(args.seed)), sort_keys=True))


if __name__ == "__main__":
    main()
