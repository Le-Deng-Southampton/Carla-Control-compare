from .ground_truth import GroundTruthErrorProvider
from .noisy_ground_truth import NoisyGroundTruthErrorProvider


def get_supported_error_provider_names():
    return ("ground_truth", "noisy_ground_truth")


def create_error_provider(provider_name="ground_truth", args=None):
    if provider_name == "ground_truth":
        return GroundTruthErrorProvider()
    if provider_name == "noisy_ground_truth":
        if args is None:
            raise ValueError("args is required to create the noisy_ground_truth error provider.")
        return NoisyGroundTruthErrorProvider(
            lateral_std=args.noise_lateral_std,
            heading_std_deg=args.noise_heading_std_deg,
            seed=args.seed + 1000,
        )
    raise ValueError(
        f"Unsupported error provider '{provider_name}'. Expected one of {get_supported_error_provider_names()}."
    )
