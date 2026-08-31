import os
import sys
import types
import unittest
from types import SimpleNamespace

import numpy as np


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PYTHONAPI_ROOT = os.path.dirname(PROJECT_ROOT)
CARLA_AGENTS_ROOT = os.path.join(PYTHONAPI_ROOT, "carla")
for path in (PROJECT_ROOT, CARLA_AGENTS_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

if "carla" not in sys.modules:
    carla = types.ModuleType("carla")

    class VehicleControl:
        pass

    carla.VehicleControl = VehicleControl
    sys.modules["carla"] = carla

from control.factory import create_tracking_controller
from control.mpc_controller import MpcController


def build_controller_args(**overrides):
    values = {
        "target_speed": 30.0,
        "lqr_q_ey": 7.5,
        "lqr_q_ey_dot": 0.20,
        "lqr_q_epsi": 6.0,
        "lqr_q_epsi_dot": 0.10,
        "lqr_r": 1.8,
        "lqr_kp_long": 9.0,
        "lqr_ki_long": 8.0,
        "lqr_kd_long": 7.0,
        "lqr_max_steer": 0.65,
        "lqr_max_steer_rate": 0.30,
        "lqr_curvature_alpha": 0.35,
        "pid_lat_kp": 0.72,
        "pid_lat_ki": 0.005,
        "pid_lat_kd": 0.38,
        "pid_long_kp": 0.22,
        "pid_long_ki": 0.01,
        "pid_long_kd": 0.14,
        "mpc_horizon": 12,
        "mpc_q_y": 10.0,
        "mpc_q_psi": 14.0,
        "mpc_r_steer": 0.9,
        "mpc_r_steer_rate": 1.2,
        "mpc_kp_long": 0.31,
        "mpc_ki_long": 0.02,
        "mpc_kd_long": 0.09,
        "mpc_max_steer": 0.65,
        "mpc_max_steer_rate": 0.30,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


class ResearchMpcControllerTest(unittest.TestCase):
    def test_dynamic_model_uses_fixed_horizon(self):
        controller = MpcController(horizon=20)

        phi, gamma, theta = controller._build_prediction_matrices(speed_mps=30.0)

        self.assertEqual(controller.model_type, "dynamic_bicycle")
        self.assertEqual(phi.shape, (80, 4))
        self.assertEqual(gamma.shape, (80, 20))
        self.assertEqual(theta.shape, (80, 20))

    def test_curvature_filter_and_step_limit_are_operational(self):
        controller = MpcController(
            horizon=4,
            curvature_filter_alpha=1.0,
            curvature_step_limit=0.01,
        )

        conditioned = controller._condition_curvature_sequence([0.0, 0.04, 0.08, 0.08])

        np.testing.assert_allclose(conditioned, [0.0, 0.01, 0.02, 0.03])

    def test_default_params_use_high_speed_preview_tuning(self):
        controller = MpcController()

        self.assertTrue(controller.supports_curvature_sequence)
        self.assertEqual(controller.horizon, 16)
        self.assertEqual(controller.q_y, 12.0)
        self.assertEqual(controller.q_psi, 18.0)
        self.assertEqual(controller.r_steer, 0.8)
        self.assertEqual(controller.r_steer_rate, 0.9)
        self.assertEqual(controller.max_steer, 0.65)
        self.assertEqual(controller.max_steer_rate, 0.30)
        self.assertEqual(controller.model_type, "dynamic_bicycle")

    def test_zero_state_has_no_removed_reference_steer_term(self):
        controller = MpcController(
            horizon=5,
            q_y=0.0,
            q_psi=0.0,
            r_steer=1.0,
            r_steer_rate=0.0,
        )

        steer = controller._solve_mpc_steering(
            initial_state=np.zeros(4),
            speed_mps=10.0,
            curvature=0.04,
        )

        self.assertAlmostEqual(steer, 0.0, places=6)

    def test_prediction_model_accepts_curvature_sequence_as_a_planned_disturbance(self):
        controller = MpcController(horizon=4)

        phi, gamma, theta = controller._build_prediction_matrices(speed_mps=8.0)

        self.assertEqual(phi.shape, (16, 4))
        self.assertEqual(gamma.shape, (16, 4))
        self.assertEqual(theta.shape, (16, 4))

    def test_factory_uses_mpc_longitudinal_parameters_independent_from_lqr(self):
        controller = create_tracking_controller(
            "mpc",
            vehicle=None,
            args=build_controller_args(
                lqr_kp_long=9.0,
                lqr_ki_long=8.0,
                lqr_kd_long=7.0,
                mpc_kp_long=0.31,
                mpc_ki_long=0.02,
                mpc_kd_long=0.09,
            ),
        )

        self.assertEqual(controller._longitudinal_controller.kp, 0.31)
        self.assertEqual(controller._longitudinal_controller.ki, 0.02)
        self.assertEqual(controller._longitudinal_controller.kd, 0.09)

    def test_factory_passes_lqr_curvature_conditioning_parameters(self):
        controller = create_tracking_controller(
            "lqr",
            vehicle=None,
            args=build_controller_args(
                lqr_curvature_alpha=0.42,
                lqr_curvature_preview_horizon=9,
                lqr_curvature_preview_blend=0.30,
            ),
        )

        self.assertEqual(controller.curvature_alpha, 0.42)
        self.assertEqual(controller.horizon, 9)
        self.assertEqual(controller.curvature_preview_blend, 0.30)


if __name__ == "__main__":
    unittest.main()
