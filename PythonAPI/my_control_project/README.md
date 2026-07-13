# My Control Project Quick Start

This folder is the controller-comparison project entry point for the CARLA
Windows package. It keeps the MSc project code separate from the official
CARLA examples and runs LQR, PID, and MPC controllers on the same route for
fair comparison.

## What matters

- Main entry point: `run_my_control.py`
- Run script: `scripts/run_my_control.ps1`
- Controller code: `control/`
- Dynamic target-speed planning: `speed_planning/`
- Route and tracking geometry: `road_planning/`
- Error sources: `error_providers/`
- Runtime, logging, metrics, and termination logic: `experiment/`
- Regression tests: `tests/`
- Project notes and report documents: `docs/`
- Experiment outputs: `log/`

## Project structure

```text
my_control_project/
+-- run_my_control.py          # Parses arguments, connects to CARLA, runs comparison
+-- scripts/
|   +-- run_my_control.ps1      # Conda/PYTHONPATH launcher
+-- control/                   # LQR, PID adapter, MPC, shared longitudinal PID
+-- speed_planning/            # Curvature/error-based dynamic target-speed planner
+-- road_planning/             # Shaped route generation and tracking geometry
+-- error_providers/           # ground_truth, noisy_ground_truth, perception_proxy error sources
+-- experiment/                # Runtime loop, collision termination, metrics, logs
+-- tests/                     # Unit/regression tests for current project modules
+-- docs/                      # Project notes and report documents
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

Run on an explicit CARLA map:

```powershell
.\scripts\run_my_control.ps1 --map-name Town10HD --target-speed 100 --route-shape gentle_curve --controllers lqr pid mpc
```

Controller-only comparison, without dynamic speed planning:

```powershell
.\scripts\run_my_control.ps1 --speed-planner-mode off --target-speed 70 --route-shape true_straight --controllers lqr pid mpc
```

Integrated high-speed system comparison, with adaptive speed planning:

```powershell
.\scripts\run_my_control.ps1 --speed-planner-mode adaptive --target-speed 120 --route-shape s_curve --controllers lqr pid mpc
```

Strict constrained Frenet planning without fallback:

```powershell
.\scripts\run_my_control.ps1 --planner-mode frenet --planner-fallback error --speed-planner-limit-profile global --controllers lqr pid mpc
```

Ordinary runs request `--planner-mode frenet` by default and allow only expected
`FrenetPlanningFailure` cases to use `--planner-fallback legacy`. Use
`--planner-fallback error` for strict experiments. Both planner modes build the
reference before any controller lap. The runner writes
`reference_trajectory.json`, records its SHA-256 hash, and checks that same
in-memory trajectory immediately before each PID/LQR/MPC lap. Run configuration
and summaries distinguish requested and resolved planner modes and record any
fallback code and rejection counts. Use `--speed-planner-limit-profile global`
for a strict controller-only comparison; use the controller-specific profile
only for an integrated system comparison. Curvature limits remain `0.20 1/m`
and curvature-rate limits remain `0.020 1/m^2`; no threshold was relaxed.

## Current default behavior

- Runs `lqr`, `pid`, and `mpc` in one launch.
- Generates a speed-adaptive route when `--route-shape auto` is used. Lower
  speeds select shorter, more curved routes; high speeds select longer,
  straighter routes. Use `true_straight` for strict straight-road experiments.
  The `straight` route is a nominal route template and may still follow a
  curved road-network segment.
- Uses `--map-name auto` by default. The runner loads a map suited to the
  selected speed, route shape, and required route length. You can also pass
  `--map-name current` to keep the already loaded CARLA world, or choose one of
  `Town01`, `Town01_Opt`, `Town02`, `Town02_Opt`, `Town03`, `Town03_Opt`,
  `Town04`, `Town04_Opt`, `Town05`, `Town05_Opt`, `Town10HD`, or
  `Town10HD_Opt`.
- Chooses `--route-min-length-m` and `--route-max-waypoints` automatically from
  `--target-speed` unless those values are provided explicitly. The automatic
  route length starts from a target drive duration scaled by speed, then is
  adjusted to the selected map's practical route capacity.
- Keeps the route fixed within one launch for fair controller comparison.
- Uses `--speed-planner-limit-profile controller` by default, so adaptive
  speed planning applies different safety/performance limits for PID, LQR, and
  MPC. PID stays closest to the global limits, LQR gets modestly looser heading
  and error-rate protection, and MPC keeps stricter tracking-error protection
  while allowing a less conservative curve-entry cap to use its tracking margin
  in realistic ultra-high-speed runs. Use `--speed-planner-limit-profile global`
  when you need all controllers to share exactly the same speed-planner limits.
- Scales speed-planner curvature preview with vehicle speed, so 100+ km/h runs
  see far enough ahead to cap curve-entry speed before the controller is already
  at its steering limits.
- Separates two experiment modes. `--speed-planner-mode off` keeps the fixed
  target speed and tracks it through the longitudinal throttle/brake controller;
  it does not directly overwrite vehicle velocity. This mode is intended for
  pure controller comparison. The default `--speed-planner-mode adaptive` uses
  dynamic target-speed planning based on preview route curvature, tracking
  error, and tracking-error growth rate.
  Stable sections stay at `--target-speed`. When a curve is detected ahead,
  `--speed-planner-entry-max-speed` caps entry speed before turn-in. The default
  cap is 70 km/h, applied progressively from
  `--speed-planner-entry-curvature-threshold 0.015` to
  `--speed-planner-entry-full-cap-curvature 0.03`. The default protective
  minimum is 48 km/h, with a short curvature preview to avoid slowing down too
  early before turn-in. Tracking-error-rate slowdowns are filtered and require
  a meaningful absolute tracking error before activating, which avoids braking
  on small transient route-index jumps. LQR feedforward blends in a short
  speed-scaled curvature preview, so it can start building bicycle-model
  steering before the nearest route point is already deep in the curve. LQR
  continuously interpolates Q/R, steering bounds, and preview smoothing from
  vehicle speed; its Riccati gain is recomputed only after a meaningful speed
  change, avoiding error/curvature-bin gain jumps. MPC keeps the
  same linear predictive controller structure, but its curvature disturbance
  preview is sampled along the route using the current speed and prediction
  horizon so high-speed runs can react before the closest-point curvature has
  fully changed. MPC solves from the current state every control cycle and
  continuously schedules objective weights and steering-rate limits, so cached
  and fresh solutions cannot alternate into periodic steering jumps.
- Supports perception-oriented error studies. `ground_truth` is the ideal upper
  bound, while `noisy_ground_truth` and `perception_proxy` can add lateral noise,
  heading noise, delay, dropout, and smoothing to tracking errors.
- Terminates a controller lap if a collision is followed by sustained near-zero speed.
- Saves each run under `log/run_<timestamp>_seed_<seed>_<route_shape>/`.

Each run folder normally contains:

- `step_log.csv`
- `summary.csv`
- `run_config.json`
- `reference_trajectory.json`
- `trajectory_compare.png`

`step_log.csv` includes raw and limited steering, steering-rate-limit status,
current/preview curvature, LQR gain-update reason, MPC solve mode, longitudinal
acceleration, `speed_plan_risk`, and `speed_plan_reason` so each
speed reduction or fixed-speed command can be traced to
`fixed_throttle_brake`, `entry_curvature`, `curvature`, `lateral_error`,
`heading_error`, `lateral_error_rate`, `heading_error_rate`, `hold`, or `none`.
`controller_speed_profile` records which speed-segmented controller profile was
active for each sample.
`summary.csv` also records the speed-planner mode, speed-limit profile, route
label, route turn and curvature statistics, planned-speed statistics, and
speed-plan reason counts. It also reports P95 steering delta, significant
steering reversals, rate-limit hit percentage, speed/lateral-acceleration P95,
lateral and longitudinal jerk, pedal switching, and a machine-readable
`stability_passed`/`stability_fail_reasons` verdict.
It also includes English readable text columns: `readable_result`,
`readable_test_conditions`, and `readable_controller_summary`, so someone
unfamiliar with the raw project parameters can see which controller performed
best, why it ranked first, and the conditions under which the result was
observed.
The live HUD shows actual speed, target speed, speed error, planner mode,
steering, throttle, and brake. HUD speed error is actual speed minus target
speed, so overspeed is positive and underspeed is negative.

## Tests

Run focused project tests from `D:\WindowsNoEditor`:

```powershell
python -m unittest discover -s PythonAPI\my_control_project\tests -v
```

Run the fixed-seed stability matrix (10 road/speed scenarios in fixed-speed and
adaptive modes, plus three perception-proxy reruns):

```powershell
powershell -ExecutionPolicy Bypass -File PythonAPI\my_control_project\scripts\run_test_matrix.ps1 -ScenarioSet stability -PlannerMode both
```

The matrix writes raw and `_paired.csv` results under `log/`. Legacy and Frenet
runs reuse the same map, seed, route shape, target speed, speed-planner mode,
error provider, and controller, while enforcing global speed-planner limits and
`--planner-fallback error`. The matrix rejects a row whose resolved planner does
not match the requested strict planner.
It exits with code 1 when a required row fails or the paired comparison finds a
collision, lane-boundary, route-generation, or minimum-clearance regression.
Low-speed curve cases use Town05 sharp town bends. High-speed curve cases use
long Town10HD/Town10HD_Opt gentle-curve candidates and reject any route whose
target-speed curvature would exceed 6.5 m/s², rather than relabelling a short
U-turn as a valid high-speed bend.

The tests cover controller factory wiring, longitudinal PID behavior, dynamic
target-speed propagation, speed planning, LQR/MPC internals, run arguments, and
collision-based termination logic.

## Desktop path

The desktop shortcut `C:\Users\LENOVO\Desktop\CARLA Quick Start.lnk` calls
`D:\WindowsNoEditor\Start-CARLA-Project.bat`, which then runs
`D:\WindowsNoEditor\Launch-CARLA-Project.ps1` and launches this script.
The launcher starts CARLA if localhost port `2000` is not already available,
waits for the simulator server, then runs `scripts/run_my_control.ps1`. When no
explicit `--speed-planner-mode` is supplied, the desktop launcher asks whether
to run pure fixed-speed controller comparison or high-speed adaptive
real-situation simulation. Console output is saved to
`log/console_desktop_<timestamp>.txt` for later debugging.
