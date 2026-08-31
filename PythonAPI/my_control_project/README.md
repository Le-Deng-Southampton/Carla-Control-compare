# Controller Evaluation Project

This directory contains the custom evaluation layer described in the repository
[README](../../README.md). It runs traceable, matched CARLA experiments for the
PID, LQR, and MPC controller platforms.

## Entry points

- `run_my_control.py` parses the experiment configuration, prepares one frozen
  reference, and runs the selected controller laps.
- `scripts/run_my_control.ps1` launches the entry point through the pinned
  `carla37` Conda environment.
- `scripts/run_controller_evaluation.ps1` builds controller-only, integrated,
  and robustness matrices.
- `scripts/run_internal_module_ablation_90.ps1` runs matched optimisation-module
  ablation variants and produces their analysis.
- `scripts/run_test_matrix.ps1` compares legacy and Frenet planning across smoke,
  extended, or stability scenario sets.

## Code map

```text
my_control_project/
+-- run_my_control.py
+-- project_config.py
+-- control/
|   +-- pid_controller.py
|   +-- lqr_controller.py
|   +-- mpc_controller.py
|   +-- longitudinal.py
|   +-- authoritative_baseline.py
+-- road_planning/
|   +-- route_planner.py
|   +-- frenet_planner.py
|   +-- reference_trajectory.py
|   +-- reference_tracker.py
+-- speed_planning/
|   +-- speed_planner.py
+-- error_providers/
|   +-- ground_truth.py
|   +-- noisy_ground_truth.py
+-- experiment/
|   +-- runtime.py
|   +-- logging.py
|   +-- metrics.py
|   +-- stability.py
|   +-- termination.py
|   +-- evaluation_aggregate.py
|   +-- internal_module_ablation_analysis.py
|   +-- official_pid_benchmark_comparison.py
+-- scripts/
+-- tests/
```

`Speed_Planing/` is retained as a compatibility import path. New code should use
the correctly named `speed_planning/` package.

## Comparison contract

A matched comparison is built around these invariants:

1. CARLA advances in synchronous 0.05 s steps.
2. The route and reference are completed before any controller lap.
3. The saved trajectory fingerprint is checked before each lap.
4. Map, reference, requested speed, seed, error provider, time base, stopping
   rules, and metrics are recorded.
5. Controller, error-provider, and speed-planner state is reset between laps.
6. Completion and hard-gate checks are evaluated before soft performance
   metrics.

The controller platforms intentionally do not use identical feedback states.
PID follows CARLA's waypoint angular-error convention; LQR uses projected
lateral/heading errors and their rates; MPC uses projected errors with a
filtered yaw-rate state. The result is therefore a matched comparison of
complete controller platforms, not a causal comparison of three mathematical
laws with identical feedback variables.

## Direct use

From the repository root, prepare the environment once:

```powershell
powershell -ExecutionPolicy Bypass -File .\PythonAPI\setup_carla37.ps1
```

With CARLA listening on `localhost:2000`, run all selected controllers on one
strictly checked reference:

```powershell
.\PythonAPI\my_control_project\scripts\run_my_control.ps1 `
  --planner-mode frenet `
  --planner-fallback error `
  --speed-planner-mode off `
  --speed-planner-limit-profile global `
  --map-name Town04 `
  --route-shape true_straight `
  --target-speed 70 `
  --seed 37321498 `
  --error-provider ground_truth `
  --controllers pid lqr mpc
```

Use `python run_my_control.py --help` through the `carla37` environment to see
all route, planner, speed-planner, error-provider, vehicle-perturbation, and
controller parameters.

## Generated evidence

Each normal run creates a timestamped directory under `log/` containing
`reference_trajectory.json`, `step_log.csv`, `summary.csv`, `run_config.json`,
`trajectory_compare.png`, and `human_summary.txt`. Batch workflows add manifests,
status files, aggregate CSV files, paired comparisons, confidence intervals, and
Markdown reports. The complete `log/` tree is generated evidence and is ignored
by Git.

## Test suite

From the repository root, use a Python 3.8+ development environment containing
the project dependencies:

```powershell
python -m unittest discover -s .\PythonAPI\my_control_project\tests -v
```

The tests validate implementation contracts and regression behaviour. They do
not replace closed-loop CARLA experiments or establish real-vehicle safety.
The CARLA runtime remains pinned to `carla37`; the tests use newer
`unittest.mock` call-inspection properties and should not be run under Python
3.7.
