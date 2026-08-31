import os
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT_PATH = os.path.join(PROJECT_ROOT, "scripts", "run_controller_evaluation.ps1")


def read_script():
    with open(SCRIPT_PATH, "r", encoding="utf-8-sig") as handle:
        return handle.read()


class ControllerEvaluationScriptTest(unittest.TestCase):
    def test_script_declares_sixty_base_and_thirty_stress_cases(self):
        text = read_script()
        self.assertIn("10 base scenarios x 3 seeds x 2 layers", text)
        self.assertIn("3 stress scenarios x 10 perturbations", text)
        self.assertIn("$baseCases.Count -ne 60", text)
        self.assertIn("$stressCases.Count -ne 30", text)
        self.assertIn("Sort-Object map_name, scenario, seed, layer", text)

    def test_script_forces_legacy_and_distinguishes_limit_profiles(self):
        text = read_script()
        self.assertIn('"--planner-mode", "legacy"', text)
        self.assertIn('"--evaluation-headless"', text)
        self.assertIn('"--speed-planner-limit-profile", "global"', text)
        self.assertIn('"--speed-planner-limit-profile", "controller"', text)
        self.assertIn('"--route-min-length-m", [string]$case.route_length_m', text)
        self.assertIn('Speed = 70; Length = 300; Route = "s_curve"', text)
        self.assertIn('Name = "road_70_gentle_curve"; Speed = 70; Length = 300; Route = "gentle_curve"; Map = "Town05_Opt"', text)
        self.assertIn('Name = "urban_50_curvy"; Speed = 50; Length = 250; Route = "curvy"; Map = "Town05_Opt"; Seeds = @(26050301, 26050302, 26050103)', text)
        self.assertIn('Name = "highway_90_s_curve"; Speed = 90; Length = 300; Route = "s_curve"; Map = "Town04"', text)
        self.assertIn('Name = "stress_90_s_curve"; Speed = 90; Length = 300; Route = "s_curve"; Map = "Town04"', text)
        self.assertIn('Route = "curvy"; Map = "Town05_Opt"', text)
        self.assertIn('Name = "smoke_70_s_curve_integrated"; Speed = 70; Length = 300; Route = "s_curve"; Map = "Town04"', text)

    def test_script_supports_resume_and_validates_completed_triplets(self):
        text = read_script()
        self.assertIn("[switch]$Resume", text)
        self.assertIn("Test-CompletedCase", text)
        self.assertIn("$validCaseIds.ContainsKey", text)
        self.assertIn("trajectory_hash", text)
        self.assertIn("run_status.csv", text)

    def test_smoke_set_is_exactly_two_experiments(self):
        text = read_script()
        self.assertIn("smoke_30_straight_controller_only", text)
        self.assertIn("smoke_70_s_curve_integrated", text)
        self.assertIn("$cases.Count -ne 2", text)

    def test_rpc_loss_stops_matrix_after_persisting_current_failure(self):
        text = read_script()
        self.assertIn("Test-CarlaRpcAvailable", text)
        self.assertIn("$stopForRpcLoss", text)
        self.assertIn("Get-Process -Name CarlaUE4", text)
        self.assertIn("& $singleRun -HealthCheck -HealthPort $Port", text)
        self.assertIn('throw "CARLA RPC became unavailable', text)

    def test_script_can_select_a_frozen_controller_implementation(self):
        text = read_script()
        self.assertIn('[string]$SingleRunScript = ""', text)
        self.assertIn('[ValidateSet("optimized", "authoritative_baseline")]', text)
        self.assertIn('[string]$ImplementationLabel = "optimized"', text)
        self.assertIn('controller_implementation = $ImplementationLabel', text)
        self.assertIn('"--controller-implementation", $ImplementationLabel', text)
        self.assertIn('$singleRun = [IO.Path]::GetFullPath($SingleRunScript)', text)

    def test_script_accepts_audited_additional_controller_arguments(self):
        text = read_script()
        self.assertIn('[string[]]$AdditionalRunArgs = @()', text)
        self.assertIn('additional_run_args = @($AdditionalRunArgs)', text)
        self.assertIn('$runArgs = @($runArgs + $AdditionalRunArgs)', text)


if __name__ == "__main__":
    unittest.main()
