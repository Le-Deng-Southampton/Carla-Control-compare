from .ground_truth import GroundTruthErrorProvider
from .noisy_ground_truth import NoisyGroundTruthErrorProvider


def get_supported_error_provider_names():
    return ("ground_truth", "noisy_ground_truth", "perception_proxy")


def create_error_provider(provider_name="ground_truth", args=None):
    if provider_name == "ground_truth":
        return GroundTruthErrorProvider()
    if provider_name in ("noisy_ground_truth", "perception_proxy"):
        if args is None:
            raise ValueError(f"args is required to create the {provider_name} error provider.")
        return NoisyGroundTruthErrorProvider(
            lateral_std=args.noise_lateral_std,
            heading_std_deg=args.noise_heading_std_deg,
            seed=args.seed + 1000,
            delay_steps=args.perception_delay_steps,
            dropout_probability=args.perception_dropout_probability,
            smoothing_alpha=args.perception_smoothing_alpha,
        )
    raise ValueError(
        f"Unsupported error provider '{provider_name}'. Expected one of {get_supported_error_provider_names()}."
    )
