# My Control Project Quick Start

This folder is the controller-comparison project entry point for the CARLA
Windows package. It keeps the MSc project code separate from the official
CARLA examples and runs LQR, PID, and MPC controllers on the same route for
fair comparison.

## What matters

- Main entry point: `run_my_control.py`
- Run script: `scripts/run_my_control.ps1`
- Controller code: `control/`
- Dynamic target-speed planning: `Speed_Planing/`
- Route and tracking geometry: `road_planning/`
- Error sources: `error_providers/`
- Runtime, logging, metrics, and termination logic: `experiment/`
- Regression tests: `tests/`
- Experiment outputs: `log/`

## Project structure

```text
my_control_project/
+-- run_my_control.py          # Parses arguments, connects to CARLA, runs comparison
+-- scripts/
|   +-- run_my_control.ps1      # Conda/PYTHONPATH launcher
+-- control/                   # LQR, PID adapter, MPC, shared longitudinal PID
+-- Speed_Planing/             # Curvature/error-based dynamic target-speed planner
+-- road_planning/             # Shaped route generation and tracking geometry
+-- error_providers/           # ground_truth and noisy_ground_truth error sources
+-- experiment/                # Runtime loop, collision termination, metrics, logs
+-- tests/                     # Unit/regression tests for current project modules
+-- legacy/                    # Compatibility wrapper for older imports
+-- log/                       # Per-run CSV, JSON, trajectory figure, console outputs
```

## Start

Set up or refresh the CARLA Python environment:

```powershell
cd D:\WindowsNoEditor\PythonAPI
.\setup_carla37.ps1
```

Run the controller comparison:

```powershell
cd D:\WindowsNoEditor\PythonAPI\my_control_project
.\scripts\run_my_control.ps1
```

Example with explicit parameters:

```powershell
.\scripts\run_my_control.ps1 --target-speed 70 --route-shape gentle_curve --controllers lqr pid mpc
```

## Current default behavior

- Runs `lqr`, `pid`, and `mpc` in one launch.
- Generates a random route shape unless `--route-shape` is provided.
- Keeps the route fixed within one launch for fair controller comparison.
- Uses dynamic target-speed planning based on route curvature, tracking error,
  and tracking-error growth rate. Stable sections stay at `--target-speed`; the
  planner only reduces speed for curvature, lateral error, heading error,
  lateral-error growth, or heading-error growth risk. After risk clears it holds
  recovery for a few control steps before ramping back to the maximum speed.
- Terminates a controller lap if a collision is followed by sustained near-zero speed.
- Saves each run under `log/run_<timestamp>_seed_<seed>_<route_shape>/`.

Each run folder normally contains:

- `step_log.csv`
- `summary.csv`
- `run_config.json`
- `trajectory_compare.png`

`step_log.csv` includes `speed_plan_risk` and `speed_plan_reason` so each
speed reduction can be traced to `curvature`, `lateral_error`, `heading_error`,
`lateral_error_rate`, `heading_error_rate`, or `none`.

## Tests

Run focused project tests from `D:\WindowsNoEditor`:

```powershell
python -m unittest discover -s PythonAPI\my_control_project\tests -v
```

The tests cover controller factory wiring, longitudinal PID behavior, dynamic
target-speed propagation, speed planning, LQR/MPC internals, run arguments, and
collision-based termination logic.

## Desktop path

The desktop shortcut `C:\Users\LENOVO\Desktop\CARLA Quick Start.lnk` calls
`D:\WindowsNoEditor\Start-CARLA-Project.bat`, which then runs
`D:\WindowsNoEditor\Launch-CARLA-Project.ps1` and launches this script.
