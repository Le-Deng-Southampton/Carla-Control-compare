# Frenet Production Enablement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the false dense-grid curvature-rate rejection, safely request Frenet by default, and preserve strict no-fallback planner comparisons.

**Architecture:** Split candidate validation into dense occupancy geometry and output-grid differential geometry. Resolve planner selection once before controller laps, catch only structured Frenet planning failures under an explicit fallback policy, and report requested and resolved modes separately through configuration and summaries.

**Tech Stack:** Python 3, NumPy, `unittest`, PowerShell, CARLA Python API.

## Global Constraints

- Keep `abs(curvature) <= 0.20 1/m` and `abs(curvature_rate) <= 0.020 1/m^2` unchanged.
- Keep lane footprint and clearance validation on the configured `0.25 m` dense grid.
- Compute curvature and curvature rate on the frozen output grid, default `1.0 m`.
- Do not modify PID, LQR, or MPC control laws or gains.
- Catch only `FrenetPlanningFailure` for fallback; unexpected exceptions must propagate.
- Ordinary runs request Frenet and allow legacy fallback; strict matrix runs use `--planner-fallback error`.
- Preserve one immutable trajectory hash across all controller laps.
- Do not stage or modify unrelated user-owned working-tree changes.

---

### Task 1: Two-Scale Frenet Candidate Validation

**Files:**
- Modify: `PythonAPI/my_control_project/road_planning/frenet_planner.py`
- Test: `PythonAPI/my_control_project/tests/test_frenet_planner.py`

**Interfaces:**
- Consumes: `CenterlineModel`, `_Candidate`, and `FrenetPlannerConfig`.
- Produces: `_candidate_output_geometry(centerline, offset_m)` for differential geometry and `_dense_candidate_occupancy(centerline, offset_m, spacing_m)` for dense clearance coordinates.
- Preserves: `plan_frenet_reference(...) -> tuple[ReferenceTrajectory, dict]`.

- [ ] **Step 1: Add a smooth constant-curvature regression fixture and failing test**

Add the following helper and test to `test_frenet_planner.py`:

```python
def constant_curvature_route(radius=50.0, length=100.0, spacing=1.0):
    arc = np.arange(0.0, length + spacing * 0.5, spacing)
    theta = arc / float(radius)
    return [
        (
            waypoint(
                radius * math.sin(angle),
                radius * (1.0 - math.cos(angle)),
                math.degrees(angle),
                index,
            ),
            "LANEFOLLOW",
        )
        for index, angle in enumerate(theta)
    ]


def test_constant_curvature_route_is_not_rejected_by_dense_interpolation_aliasing(self):
    trajectory, diagnostics = plan_frenet_reference(
        constant_curvature_route(),
        dict(route_features(), route_label="constant_curvature", total_abs_turn=2.0),
        target_speed_kmh=50.0,
    )

    self.assertLessEqual(
        diagnostics["reference_max_abs_curvature_rate"],
        FrenetPlannerConfig().max_abs_curvature_rate_1pm2,
    )
    self.assertGreater(len(trajectory.s_m), 2)
```

- [ ] **Step 2: Run the regression and verify RED**

Run:

```powershell
python -m unittest PythonAPI.my_control_project.tests.test_frenet_planner.FrenetPlannerTest.test_constant_curvature_route_is_not_rejected_by_dense_interpolation_aliasing -v
```

Expected: FAIL with `FrenetPlanningFailure: No feasible Frenet trajectory` and `curvature_rate` rejections.

- [ ] **Step 3: Separate dense occupancy from output differential geometry**

Replace `_candidate_geometry` with two focused helpers:

```python
def _candidate_output_geometry(centerline, offset_m):
    output_s = centerline.s_m
    output_offset = np.asarray(offset_m, dtype=float)
    center_yaw = centerline.yaw_rad
    x_values = centerline.x_m - output_offset * np.sin(center_yaw)
    y_values = centerline.y_m + output_offset * np.cos(center_yaw)
    dx = np.gradient(x_values, output_s, edge_order=1)
    dy = np.gradient(y_values, output_s, edge_order=1)
    yaw_values = np.unwrap(np.arctan2(dy, dx))
    curvature = np.gradient(yaw_values, output_s, edge_order=1)
    curvature_rate = np.gradient(curvature, output_s, edge_order=1)
    return x_values, y_values, yaw_values, curvature, curvature_rate


def _dense_candidate_occupancy(centerline, offset_m, spacing_m):
    dense_s = np.arange(centerline.s_m[0], centerline.s_m[-1], max(float(spacing_m), 0.05))
    if dense_s[-1] < centerline.s_m[-1] - 1e-9:
        dense_s = np.append(dense_s, centerline.s_m[-1])
    dense_offset = np.interp(dense_s, centerline.s_m, offset_m)
    dense_width = np.interp(dense_s, centerline.s_m, centerline.lane_width_m)
    return dense_s, dense_offset, dense_width
```

In `_evaluate_candidate`, use `_dense_candidate_occupancy` only for footprint clearance. Use `_candidate_output_geometry` for curvature and curvature-rate rejection, output trajectory fields, scoring, and speed caps. Record the two spacings in diagnostics:

```python
"occupancy_validation_spacing_m": float(config.validation_spacing_m),
"differential_validation_spacing_m": float(config.output_spacing_m),
```

- [ ] **Step 4: Run the focused test and verify GREEN**

Run the command from Step 2.

Expected: PASS; maximum output-grid curvature rate remains at or below `0.020`.

- [ ] **Step 5: Add and verify a genuine curvature-rate rejection**

Add a tangent-continuous route that changes from a straight to a 10 m radius arc:

```python
def abrupt_curvature_route():
    points = [
        (value, 0.0, 0.0)
        for value in np.arange(-30.0, 0.0, 1.0)
    ]
    for arc in np.arange(0.0, 20.1, 1.0):
        angle = arc / 10.0
        points.append((10.0 * math.sin(angle), 10.0 * (1.0 - math.cos(angle)), math.degrees(angle)))
    return [
        (waypoint(x, y, yaw, index), "LANEFOLLOW")
        for index, (x, y, yaw) in enumerate(points)
    ]


def test_genuine_output_grid_curvature_rate_step_is_rejected(self):
    with self.assertRaises(FrenetPlanningFailure) as raised:
        plan_frenet_reference(
            abrupt_curvature_route(),
            dict(route_features(), length=50.0, route_label="curvature_step"),
            target_speed_kmh=30.0,
        )

    self.assertGreater(raised.exception.rejection_counts.get("curvature_rate", 0), 0)
```

Run both new tests and then the complete planner test module. Expected: both PASS, and the existing lane-clearance test still proves dense occupancy validation.

- [ ] **Step 6: Commit the isolated geometry fix**

```powershell
git add -- PythonAPI/my_control_project/road_planning/frenet_planner.py PythonAPI/my_control_project/tests/test_frenet_planner.py
git commit -m "fix: validate Frenet derivatives on output grid"
```

---

### Task 2: Explicit Planner Fallback and Audit Metadata

**Files:**
- Modify: `PythonAPI/my_control_project/project_config.py`
- Modify: `PythonAPI/my_control_project/experiment/runtime.py`
- Modify: `PythonAPI/my_control_project/run_my_control.py`
- Modify: `PythonAPI/my_control_project/experiment/metrics.py`
- Test: `PythonAPI/my_control_project/tests/test_run_args.py`
- Test: `PythonAPI/my_control_project/tests/test_speed_planner.py`

**Interfaces:**
- Consumes: `FrenetPlanningFailure`, `plan_frenet_reference`, and `build_legacy_reference_trajectory`.
- Produces: CLI `--planner-fallback {legacy,error}`; route features `planner_mode_requested`, `planner_mode_resolved`, `planner_fallback_policy`, `planner_fallback_used`, `planner_fallback_code`, `planner_fallback_message`, and `planner_fallback_rejection_counts`.
- Preserves: `resolve_route_setup(...)` return shape and `planner_mode` as the resolved planner mode.

- [ ] **Step 1: Write failing CLI default tests**

Change the default assertion and add fallback assertions in `test_run_args.py`:

```python
self.assertEqual(args.planner_mode, "frenet")
self.assertEqual(args.planner_fallback, "legacy")
```

Add an explicit strict-policy parse test:

```python
def test_frenet_strict_policy_can_disable_fallback(self):
    run_my_control = importlib.import_module("run_my_control")
    with mock.patch.object(
        sys,
        "argv",
        ["run_my_control.py", "--planner-mode", "frenet", "--planner-fallback", "error"],
    ):
        args = run_my_control.parse_args()
    self.assertEqual(args.planner_fallback, "error")
```

Run the two tests. Expected: FAIL because the current default is legacy and the fallback argument does not exist.

- [ ] **Step 2: Add versioned fallback configuration**

In `project_config.py` set:

```python
"planner_mode": "frenet",
"planner_fallback": "legacy",
```

Add:

```python
(("--planner-fallback",), {
    "choices": ("legacy", "error"),
    "default": DEFAULTS["planner_fallback"],
    "help": "Expected Frenet failure policy: use legacy for normal runs or error for strict comparisons.",
}),
```

Run the CLI tests. Expected: PASS.

- [ ] **Step 3: Write failing route-resolution fallback tests**

Extend `_planner_selection_fixture` with `planner_fallback="legacy"`. Add tests that mock `plan_frenet_reference` to raise:

```python
FrenetPlanningFailure("no_feasible_candidate", {"curvature_rate": 21}, "No feasible Frenet trajectory.")
```

For `legacy`, require `build_legacy_reference_trajectory` to run once and assert:

```python
self.assertEqual(route_features["planner_mode_requested"], "frenet")
self.assertEqual(route_features["planner_mode_resolved"], "legacy")
self.assertEqual(route_features["planner_mode"], "legacy")
self.assertTrue(route_features["planner_fallback_used"])
self.assertEqual(route_features["planner_fallback_code"], "no_feasible_candidate")
self.assertEqual(route_features["planner_fallback_rejection_counts"], {"curvature_rate": 21})
```

For `planner_fallback="error"`, require the same exception object to propagate and legacy to remain uncalled. Add another test with `RuntimeError("programming defect")` and require it to propagate even under `legacy` policy.

Run these new tests. Expected: FAIL because `resolve_route_setup` currently has no fallback.

- [ ] **Step 4: Implement structured fallback in route setup**

Create a local `build_legacy()` closure inside `resolve_route_setup` so explicit legacy mode and fallback share exactly one construction path. Initialize audit features before selection:

```python
requested_mode = arg_value(args, "planner_mode")
fallback_policy = arg_value(args, "planner_fallback")
planner_audit = {
    "planner_mode_requested": requested_mode,
    "planner_mode_resolved": requested_mode,
    "planner_fallback_policy": fallback_policy,
    "planner_fallback_used": False,
    "planner_fallback_code": "",
    "planner_fallback_message": "",
    "planner_fallback_rejection_counts": {},
}
```

Wrap only `plan_frenet_reference(...)`:

```python
try:
    trajectory, planner_features = plan_frenet_reference(...)
    reference_trace = trajectory.to_route_trace()
except FrenetPlanningFailure as exc:
    if fallback_policy != "legacy":
        raise
    trajectory, reference_trace, planner_features = build_legacy()
    planner_audit.update(
        planner_mode_resolved="legacy",
        planner_fallback_used=True,
        planner_fallback_code=exc.code,
        planner_fallback_message=str(exc),
        planner_fallback_rejection_counts=dict(exc.rejection_counts),
    )
```

After construction, update `route_features` with `planner_features` and `planner_audit`, set `planner_mode` from `planner_mode_resolved`, and then compute the resolved hash. Do not catch `Exception` or `RuntimeError`.

Run all planner-selection tests. Expected: PASS.

- [ ] **Step 5: Write failing run-config and summary audit tests**

In `test_run_args.py`, supply fallback route features and require `_planner_run_config` to contain:

```python
"requested_mode": "frenet",
"resolved_mode": "legacy",
"fallback_policy": "legacy",
"fallback_used": True,
"fallback_code": "no_feasible_candidate",
```

In the existing summary test in `test_speed_planner.py`, require the new planner summary fields and ensure `planner_mode == "legacy"` for a resolved fallback.

Run the focused tests. Expected: FAIL because these audit fields are not reported.

- [ ] **Step 6: Propagate audit fields through configuration and summaries**

Update `_planner_run_config` in `run_my_control.py` to use route features:

```python
"mode": route_features.get("planner_mode_resolved", args.planner_mode),
"requested_mode": route_features.get("planner_mode_requested", args.planner_mode),
"resolved_mode": route_features.get("planner_mode_resolved", args.planner_mode),
"fallback_policy": route_features.get("planner_fallback_policy", args.planner_fallback),
"fallback_used": bool(route_features.get("planner_fallback_used", False)),
"fallback_code": route_features.get("planner_fallback_code", ""),
"fallback_message": route_features.get("planner_fallback_message", ""),
"fallback_rejection_counts": route_features.get("planner_fallback_rejection_counts", {}),
```

Add these names to `PLANNER_SUMMARY_FIELDS` in `metrics.py`:

```python
"planner_mode_requested",
"planner_mode_resolved",
"planner_fallback_policy",
"planner_fallback_used",
"planner_fallback_code",
"planner_fallback_message",
"planner_fallback_rejection_counts",
```

Populate them in `build_summary`, JSON-encoding rejection counts deterministically. Keep `planner_mode` equal to the resolved mode. Run the focused tests and the full unit suite.

- [ ] **Step 7: Commit fallback and audit behavior**

```powershell
git add -- PythonAPI/my_control_project/project_config.py PythonAPI/my_control_project/experiment/runtime.py PythonAPI/my_control_project/run_my_control.py PythonAPI/my_control_project/experiment/metrics.py PythonAPI/my_control_project/tests/test_run_args.py PythonAPI/my_control_project/tests/test_speed_planner.py
git commit -m "feat: enable auditable Frenet fallback"
```

---

### Task 3: Strict Matrix Policy and User Documentation

**Files:**
- Modify: `PythonAPI/my_control_project/scripts/run_test_matrix.ps1`
- Modify: `PythonAPI/my_control_project/tests/test_matrix_script.py`
- Modify: `PythonAPI/my_control_project/README.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: CLI `--planner-fallback error`.
- Produces: strict matrix invocation that cannot relabel fallback legacy runs as Frenet.

- [ ] **Step 1: Write a failing strict-matrix assertion**

Add to `test_stability_matrix_pairs_legacy_and_frenet_under_strict_limits`:

```python
self.assertIn('"--planner-fallback", "error"', script)
```

Run the test. Expected: FAIL because the argument is absent.

- [ ] **Step 2: Make every matrix case strict**

Add the argument beside planner mode in `$runArgs`:

```powershell
"--planner-mode", $casePlannerMode,
"--planner-fallback", "error",
```

When importing each summary row, verify the resolved mode equals `$casePlannerMode`; otherwise create a failed matrix record. Run `test_matrix_script.py`. Expected: PASS.

- [ ] **Step 3: Update documentation**

Document:

- Frenet is the ordinary requested default after the smoke gate.
- `--planner-fallback legacy` is the ordinary availability policy.
- `--planner-fallback error` is required for strict comparisons.
- Requested and resolved modes plus fallback reasons are logged.
- Safety thresholds remain unchanged and smoke success is not a performance claim.

Run `git diff --check` on the four files.

- [ ] **Step 4: Commit strict comparison behavior and docs**

```powershell
git add -- PythonAPI/my_control_project/scripts/run_test_matrix.ps1 PythonAPI/my_control_project/tests/test_matrix_script.py PythonAPI/my_control_project/README.md README.md
git commit -m "docs: enable strict Frenet rollout"
```

---

### Task 4: Verification and CARLA Rollout Gate

**Files:**
- Verify all files changed in Tasks 1–3.
- Conditionally modify: `PythonAPI/my_control_project/project_config.py` and its default test only if the smoke gate fails and the default must be returned to legacy.

**Interfaces:**
- Produces evidence that the official route is usable with Frenet and no fallback.

- [ ] **Step 1: Run complete static verification**

```powershell
python -m unittest discover -s PythonAPI\my_control_project\tests -q
python -m compileall -q PythonAPI\my_control_project
git diff --check
```

Expected: all unit tests pass, compilation exits 0, and `git diff --check` emits no errors.

- [ ] **Step 2: Run the explicit no-fallback Town04 smoke gate**

```powershell
.\PythonAPI\my_control_project\scripts\run_my_control.ps1 --map-name Town04 --seed 26050101 --target-speed 50 --route-shape straight --planner-mode frenet --planner-fallback error --speed-planner-limit-profile global --controllers lqr pid mpc
```

Expected: route generation succeeds, all controllers run, no fallback is used, and a run directory contains `reference_trajectory.json`, `run_config.json`, and `summary.csv`.

- [ ] **Step 3: Verify the smoke artifacts**

Inspect the newest matching run and require:

- `planner.requested_mode == "frenet"`;
- `planner.resolved_mode == "frenet"`;
- `planner.fallback_used == false`;
- `reference_max_abs_curvature <= 0.20`;
- `reference_max_abs_curvature_rate <= 0.020`;
- all PID/LQR/MPC rows have one identical non-empty trajectory hash;
- collision and lane-boundary violation counts are zero;
- each controller wrote route-completion and stability fields.

If the smoke gate fails, do not weaken limits. Use `superpowers:systematic-debugging`, keep or restore the requested default to legacy, and report the actual blocker.

- [ ] **Step 4: Re-run final verification after any gate-driven edit**

Run the complete commands from Step 1 again. Review `git status --short`, `git diff --stat`, and the commit list to ensure no unrelated dirty files were staged.

- [ ] **Step 5: Commit only a gate-driven default rollback if required**

If and only if smoke fails after the validated geometry fix:

```powershell
git add -- PythonAPI/my_control_project/project_config.py PythonAPI/my_control_project/tests/test_run_args.py PythonAPI/my_control_project/README.md README.md
git commit -m "docs: keep legacy default after Frenet gate failure"
```

If smoke passes, no additional commit is needed because Task 2 already enabled the Frenet default.
