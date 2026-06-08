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
+-- error_providers/           # ground_truth, noisy_ground_truth, perception_proxy error sources
+-- experiment/                # Runtime loop, collision termination, metrics, logs
+-- tests/                     # Unit/regression tests for current project modules
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

Controller-only comparison, without dynamic speed planning:

```powershell
.\scripts\run_my_control.ps1 --speed-planner-mode off --target-speed 70 --route-shape true_straight --controllers lqr pid mpc
```

Integrated high-speed system comparison, with adaptive speed planning:

```powershell
.\scripts\run_my_control.ps1 --speed-planner-mode adaptive --target-speed 120 --route-shape s_curve --controllers lqr pid mpc
```

## Current default behavior

- Runs `lqr`, `pid`, and `mpc` in one launch.
- Generates a random route shape unless `--route-shape` is provided. Use
  `true_straight` for strict straight-road experiments. The `straight` route is
  a nominal route template and may still follow a curved road-network segment.
- Keeps the route fixed within one launch for fair controller comparison.
- Separates two experiment modes. `--speed-planner-mode off` keeps the fixed
  target speed and is intended for pure controller comparison. The default
  `--speed-planner-mode adaptive` uses dynamic target-speed planning based on
  preview route curvature, tracking error, and tracking-error growth rate.
  Stable sections stay at `--target-speed`. When a curve is detected ahead,
  `--speed-planner-entry-max-speed` caps entry speed before turn-in. The default
  cap is 70 km/h, applied progressively from
  `--speed-planner-entry-curvature-threshold 0.015` to
  `--speed-planner-entry-full-cap-curvature 0.03`.
- Supports perception-oriented error studies. `ground_truth` is the ideal upper
  bound, while `noisy_ground_truth` and `perception_proxy` can add lateral noise,
  heading noise, delay, dropout, and smoothing to tracking errors.
- Terminates a controller lap if a collision is followed by sustained near-zero speed.
- Saves each run under `log/run_<timestamp>_seed_<seed>_<route_shape>/`.

Each run folder normally contains:

- `step_log.csv`
- `summary.csv`
- `human_summary.txt`
- `run_config.json`
- `trajectory_compare.png`

`step_log.csv` includes `speed_plan_risk` and `speed_plan_reason` so each
speed reduction can be traced to `entry_curvature`, `curvature`, `lateral_error`,
`heading_error`, `lateral_error_rate`, `heading_error_rate`, `hold`, or `none`.
`summary.csv` also records the speed-planner mode, route label, route turn and
curvature statistics, planned-speed statistics, and speed-plan reason counts.
`human_summary.txt` translates those metrics into a plain-language Chinese
result: the best-performing controller in that run, why it ranked first, and
the test conditions under which that result was observed.

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
The launcher starts CARLA if localhost port `2000` is not already available,
waits for the simulator server, then runs `scripts/run_my_control.ps1`. Console
output is saved to `log/console_desktop_<timestamp>.txt` for later debugging.
