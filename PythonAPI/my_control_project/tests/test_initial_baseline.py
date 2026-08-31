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

# Another test module installs a deliberately minimal agents controller stub.
# This baseline needs CARLA's combined VehiclePIDController, so reload the real
# package when discovery reaches this module after that stub.
controller_stub = sys.modules.get("agents.navigation.controller")
if controller_stub is not None and not hasattr(controller_stub, "VehiclePIDController"):
    sys.modules.pop("agents.navigation.controller", None)
    sys.modules.pop("agents.navigation", None)
    sys.modules.pop("agents", None)

from control.initial_baseline import (
    BASELINE_IMPLEMENTATION,
    InitialLqrController,
    InitialMpcController,
    InitialPidController,
    MPC_FIRST_RECORDED_RUN,
    MPC_SOURCE_SESSION,
    MPC_SOURCE_SNAPSHOT_SHA256,
    build_initial_controller,
)
from run_initial_baseline import apply_initial_argument_values


class FakeVehicle:
    def get_world(self):
        return SimpleNamespace()

    def get_control(self):
        return SimpleNamespace(steer=0.0)


class InitialBaselineTest(unittest.TestCase):
    def test_baseline_records_mixed_historical_provenance(self):
        self.assertEqual(BASELINE_IMPLEMENTATION, "historical_recovered_four_state_mpc")
        self.assertEqual(MPC_SOURCE_SESSION, "019f0232-102f-7b90-bad5-4b91b3471aae")
        self.assertEqual(
            MPC_SOURCE_SNAPSHOT_SHA256,
            "6c943e7546924ad2173ea5c8f763e9c1765cb5d14c4cb9038ea8219f9032bb67",
        )
        self.assertEqual(MPC_FIRST_RECORDED_RUN, "run_20260615_150704_seed_221574906_straight")

    def test_recovered_pid_uses_vehicle_pid_controller_parameters(self):
        controller = InitialPidController(FakeVehicle(), target_speed=30.0)

        pid = controller._pid_controller
        self.assertEqual(pid._lat_controller._k_p, 0.72)
        self.assertEqual(pid._lat_controller._k_i, 0.005)
        self.assertEqual(pid._lat_controller._k_d, 0.38)
        self.assertEqual(pid._lon_controller._k_p, 0.22)
        self.assertEqual(pid._lon_controller._k_i, 0.01)
        self.assertEqual(pid._lon_controller._k_d, 0.14)
        self.assertEqual(pid.max_throt, 0.45)
        self.assertEqual(pid.max_brake, 0.28)
        self.assertEqual(pid.max_steer, 0.65)

    def test_builder_uses_recovered_pid_lqr_and_first_four_state_mpc_parameters(self):
        args = SimpleNamespace(target_speed=70.0)

        pid = build_initial_controller("pid", FakeVehicle(), args)
        lqr = build_initial_controller("lqr", FakeVehicle(), args)
        mpc = build_initial_controller("mpc", FakeVehicle(), args)

        self.assertEqual(pid._pid_controller._lon_controller._k_p, 0.22)
        np.testing.assert_allclose(np.diag(lqr.Q), [2.2, 1.1, 0.04])
        self.assertEqual(lqr.R[0, 0], 8.0)
        self.assertEqual(lqr.L, 2.9)
        self.assertEqual(lqr.max_steer, 0.65)
        self.assertEqual(lqr.max_steer_rate, 0.045)
        self.assertEqual(lqr._longitudinal_controller.kp, 0.35)
        self.assertEqual(lqr._longitudinal_controller.ki, 0.02)
        self.assertEqual(lqr._longitudinal_controller.kd, 0.16)
        self.assertEqual(mpc.horizon, 16)
        self.assertEqual(mpc.q_y, 14.0)
        self.assertEqual(mpc.q_psi, 22.0)
        self.assertEqual(mpc.r_steer, 1.0)
        self.assertEqual(mpc.r_steer_rate, 1.1)
        self.assertEqual(mpc.max_lateral_accel, 7.5)
        self.assertEqual(mpc.min_dynamic_steer_limit, 0.18)
        self.assertEqual(mpc.min_horizon, 8)
        self.assertEqual(mpc.model_type, "dynamic_bicycle")

    def test_recovered_lqr_model_matches_three_state_integral_equations(self):
        controller = InitialLqrController()

        a_matrix, b_matrix = controller._build_lqr_model(10.0)

        np.testing.assert_allclose(a_matrix, [[0.0, 10.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        np.testing.assert_allclose(b_matrix, [[0.0], [10.0 / 2.9], [0.0]])

    def test_mpc_model_matches_recovered_four_state_dynamic_equations(self):
        controller = InitialMpcController()

        a_matrix, b_matrix, curvature_matrix = controller._build_single_step_model(10.0)

        self.assertEqual(a_matrix.shape, (4, 4))
        self.assertEqual(b_matrix.shape, (4, 1))
        self.assertEqual(curvature_matrix.shape, (4, 1))
        self.assertLess(curvature_matrix[2, 0], 0.0)

    def test_entrypoint_records_the_parameters_actually_applied_by_frozen_code(self):
        args = SimpleNamespace(lqr_q_ey=99.0, mpc_horizon=99)

        result = apply_initial_argument_values(args)

        self.assertIs(result, args)
        self.assertEqual(args.lqr_q_ey, 2.2)
        self.assertEqual(args.lqr_q_epsi, 1.1)
        self.assertEqual(args.lqr_q_iey, 0.04)
        self.assertEqual(args.lqr_r, 8.0)
        self.assertEqual(args.lqr_max_steer_rate, 0.045)
        self.assertEqual(args.mpc_horizon, 16)
        self.assertEqual(args.mpc_r_steer_rate, 1.1)
        self.assertEqual(args.mpc_model_type, "dynamic_bicycle")
        self.assertEqual(args.mpc_max_lateral_accel, 7.5)
        self.assertEqual(args.mpc_min_dynamic_steer_limit, 0.18)
        self.assertEqual(args.pid_lat_kp, 0.72)
        self.assertEqual(args.pid_long_kp, 0.22)
        self.assertEqual(args.pid_max_throttle, 0.45)

    def test_mpc_retains_recovered_four_state_vehicle_parameters(self):
        controller = InitialLqrController()
        mpc = InitialMpcController()

        self.assertEqual(controller.L, 2.9)
        self.assertEqual(mpc.Lf, 1.45)
        self.assertEqual(mpc.Lr, 1.45)


if __name__ == "__main__":
    unittest.main()
