import os
import sys
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from error_providers.noisy_ground_truth import NoisyGroundTruthErrorProvider


class SequenceErrorProvider:
    def __init__(self, samples):
        self.samples = list(samples)
        self.index = 0

    def compute(self, *args, **kwargs):
        sample = self.samples[self.index]
        self.index = min(self.index + 1, len(self.samples) - 1)
        return dict(sample)


def sample(e_y, e_psi):
    return {
        "e_y": e_y,
        "e_psi": e_psi,
        "target_x": 1.0,
        "target_y": 2.0,
        "target_yaw": 0.3,
        "reference_x": 1.0,
        "reference_y": 2.0,
        "reference_yaw": 0.3,
    }


class NoisyGroundTruthErrorProviderTest(unittest.TestCase):
    def test_delay_reuses_older_tracking_error_sample(self):
        provider = NoisyGroundTruthErrorProvider(
            lateral_std=0.0,
            heading_std_deg=0.0,
            delay_steps=1,
        )
        provider._base_provider = SequenceErrorProvider([sample(1.0, 0.1), sample(2.0, 0.2)])

        first = provider.compute(None, None, None, 0)
        second = provider.compute(None, None, None, 0)

        self.assertEqual(first["e_y"], 1.0)
        self.assertEqual(second["e_y"], 1.0)

    def test_smoothing_blends_perception_proxy_errors(self):
        provider = NoisyGroundTruthErrorProvider(
            lateral_std=0.0,
            heading_std_deg=0.0,
            smoothing_alpha=0.5,
        )
        provider._base_provider = SequenceErrorProvider([sample(0.0, 0.0), sample(2.0, 0.2)])

        provider.compute(None, None, None, 0)
        second = provider.compute(None, None, None, 0)

        self.assertEqual(second["e_y"], 1.0)
        self.assertEqual(second["e_psi"], 0.1)


if __name__ == "__main__":
    unittest.main()
