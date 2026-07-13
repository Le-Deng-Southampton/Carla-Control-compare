# Explainable Frenet Planning and Lane-Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a deterministic, controller-independent Frenet reference planner and continuous arc-length tracker that improve paired CARLA route-planning and lane-tracking results without changing PID, LQR, or MPC control laws.

**Architecture:** Keep CARLA topology and the current shaped-route search as the legal road/lane layer. Convert the chosen topology route into either a legacy or constrained Frenet `ReferenceTrajectory`, serialize and hash it before controller laps, then give each lap a fresh `ReferenceTracker` over that same immutable trajectory. Runtime controllers continue to receive waypoint-like targets and tracking-error dictionaries, while all projection and preview geometry comes from physical arc length.

**Tech Stack:** Python 3.7, NumPy, CARLA Python API, `unittest`, PowerShell, standard-library JSON/CSV/hashlib.

## Global Constraints

- Preserve all unrelated and pre-existing uncommitted workspace changes; stage only files named by the active task.
- Do not change PID, LQR, or MPC control laws, constructor names, or controller factory names.
- Keep `legacy` available and opt-in `frenet` until paired simulator gates pass.
- Use no new third-party dependency beyond NumPy and the existing CARLA stack.
- Use `1.0 m` output spacing and `0.25 m` feasibility-validation spacing.
- Use Model 3 planning half-width `1.05 m` and body-to-boundary margin `0.35 m`.
- Use lateral-offset fractions `(-0.75, -0.50, -0.25, 0.00, 0.25, 0.50, 0.75)` and transition lengths `(20.0, 40.0, 60.0)` metres.
- Use topology beam width `24`, lateral-profile beam width `64`, candidate cap `512`, and planning deadline `5.0 s`.
- Enforce `abs(curvature) <= 0.20 1/m`, `abs(curvature_rate) <= 0.020 1/m^2`, lateral acceleration `<= 6.5 m/s^2`, and lateral-jerk speed cap `<= 10.0 m/s^3`.
- Use normalized score weights: length `3.0`, shape `1.0`, offset `0.5`, curvature `2.0`, curvature rate `1.5`, inverse clearance `2.0`, and lateral acceleration `3.0`.
- Strict controller comparisons use `--speed-planner-limit-profile global` and identical trajectory hashes for PID, LQR, and MPC.
- Follow RED, GREEN, focused regression, then full-suite and CARLA matrix verification.

## File Structure

Create:

- `PythonAPI/my_control_project/road_planning/reference_trajectory.py` — immutable trajectory model, interpolation, canonical hashing, JSON replay, and waypoint adapters.
- `PythonAPI/my_control_project/road_planning/frenet_planner.py` — centerline model, quintic lateral profiles, constraint validation, candidate scoring, and structured failures.
- `PythonAPI/my_control_project/road_planning/reference_tracker.py` — continuous segment projection, monotonic progress, recovery, preview, and completion state.
- `PythonAPI/my_control_project/experiment/planner_comparison.py` — pair legacy/Frenet summary rows and calculate planner deltas.
- `PythonAPI/my_control_project/tests/test_reference_trajectory.py`
- `PythonAPI/my_control_project/tests/test_frenet_planner.py`
- `PythonAPI/my_control_project/tests/test_reference_tracker.py`
- `PythonAPI/my_control_project/tests/test_planner_comparison.py`

Modify:

- `PythonAPI/my_control_project/road_planning/route_planner.py` — deterministic diverse beam state and full-path loop rejection.
- `PythonAPI/my_control_project/road_planning/reference_path.py` — retain legacy smoothing and expose conversion through the common trajectory interface.
- `PythonAPI/my_control_project/project_config.py` — planner/tracker defaults and CLI options.
- `PythonAPI/my_control_project/run_my_control.py` — freeze/save trajectory before laps and record planner configuration.
- `PythonAPI/my_control_project/experiment/runtime.py` — build selected trajectory and use `ReferenceTracker` for all controllers.
- `PythonAPI/my_control_project/experiment/logging.py` — append projection/preview observability fields.
- `PythonAPI/my_control_project/experiment/metrics.py` — append planner metadata and trajectory hash to summaries.
- `PythonAPI/my_control_project/experiment/__init__.py` — export paired-comparison helpers if needed by the runner.
- `PythonAPI/my_control_project/scripts/run_test_matrix.ps1` — run both planners with global limits and create paired output.
- `PythonAPI/my_control_project/tests/test_route_planner.py`
- `PythonAPI/my_control_project/tests/test_run_args.py`
- `PythonAPI/my_control_project/tests/test_speed_planner.py`
- `PythonAPI/my_control_project/tests/test_matrix_script.py`
- `PythonAPI/my_control_project/README.md`
- `README.md`

---

### Task 1: Immutable Reference Trajectory and Replay Artifact

**Files:**
- Create: `PythonAPI/my_control_project/road_planning/reference_trajectory.py`
- Create: `PythonAPI/my_control_project/tests/test_reference_trajectory.py`
- Modify: `PythonAPI/my_control_project/road_planning/reference_path.py`

**Interfaces:**
- Produces: `TrajectorySample`, `TrajectoryWaypoint`, `ReferenceTrajectory`, `trajectory_from_route_trace()`, `ReferenceTrajectory.to_route_trace()`, `save_reference_trajectory()`, and `load_reference_trajectory()`.
- Consumes later: Frenet planner, reference tracker, runtime, run-config builder, and paired experiments.

- [ ] **Step 1: Write failing interpolation and immutability tests**

Add tests that build a three-point synthetic trajectory at `s=(0, 5, 10)` and require exact physical interpolation, heading wrap interpolation, clamping at both ends, equal-length validation, strictly increasing `s`, and read-only NumPy arrays.

```python
def test_sample_at_interpolates_by_arc_length(self):
    trajectory = make_trajectory(
        s=[0.0, 5.0, 10.0],
        x=[0.0, 5.0, 10.0],
        y=[0.0, 2.0, 4.0],
        yaw=[np.radians(179.0), np.radians(180.0), np.radians(181.0)],
        curvature=[0.0, 0.02, 0.04],
    )
    sample = trajectory.sample_at(7.5)
    self.assertAlmostEqual(sample.x_m, 7.5)
    self.assertAlmostEqual(sample.y_m, 3.0)
    self.assertAlmostEqual(sample.curvature_1pm, 0.03)
    self.assertAlmostEqual(np.degrees(sample.yaw_rad), 180.5)

def test_trajectory_rejects_non_increasing_arc_length(self):
    with self.assertRaisesRegex(ValueError, "strictly increasing"):
        make_trajectory(s=[0.0, 1.0, 1.0])
```

- [ ] **Step 2: Run interpolation tests to verify RED**

Run:

```powershell
python -m unittest PythonAPI.my_control_project.tests.test_reference_trajectory -v
```

Expected: import failure for missing `road_planning.reference_trajectory`.

- [ ] **Step 3: Implement the trajectory model and waypoint adapter**

Implement these exact public types and signatures:

```python
@dataclass(frozen=True)
class TrajectorySample:
    s_m: float
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    curvature_1pm: float
    curvature_rate_1pm2: float
    left_clearance_m: float
    right_clearance_m: float
    speed_cap_mps: float
    source_index: int
    road_option: str

@dataclass(frozen=True)
class ReferenceTrajectory:
    s_m: np.ndarray
    x_m: np.ndarray
    y_m: np.ndarray
    z_m: np.ndarray
    yaw_rad: np.ndarray
    curvature_1pm: np.ndarray
    curvature_rate_1pm2: np.ndarray
    left_clearance_m: np.ndarray
    right_clearance_m: np.ndarray
    speed_cap_mps: np.ndarray
    source_index: np.ndarray
    road_options: tuple
    metadata: dict

    def sample_at(self, query_s_m: float) -> TrajectorySample:
        query = float(np.clip(query_s_m, self.s_m[0], self.s_m[-1]))
        index = min(int(np.searchsorted(self.s_m, query, side="right") - 1), len(self.s_m) - 2)
        span = max(float(self.s_m[index + 1] - self.s_m[index]), 1e-12)
        ratio = (query - float(self.s_m[index])) / span
        yaw = self.yaw_rad[index] + ratio * (self.yaw_rad[index + 1] - self.yaw_rad[index])
        interp = lambda values: float(values[index] + ratio * (values[index + 1] - values[index]))
        nearest = index if ratio < 0.5 else index + 1
        return TrajectorySample(
            query, interp(self.x_m), interp(self.y_m), interp(self.z_m),
            float((yaw + np.pi) % (2.0 * np.pi) - np.pi),
            interp(self.curvature_1pm), interp(self.curvature_rate_1pm2),
            interp(self.left_clearance_m), interp(self.right_clearance_m),
            interp(self.speed_cap_mps), int(self.source_index[nearest]),
            self.road_options[nearest],
        )

    def content_hash(self) -> str:
        return hashlib.sha256(self._canonical_bytes()).hexdigest()
```

Also implement `waypoint_at(s_m) -> TrajectoryWaypoint`, `to_dict() -> dict`,
`to_route_trace() -> List[Tuple[TrajectoryWaypoint, object]]`,
`trajectory_from_route_trace(route_trace, metadata=None) -> ReferenceTrajectory`,
`save_reference_trajectory(path, trajectory, execution_metadata=None) -> None`,
and `load_reference_trajectory(path) -> ReferenceTrajectory` with exactly those
signatures. `TrajectoryWaypoint` must expose `.transform.location`,
`.transform.rotation.yaw`, `.id`, `.lane_width`, `.source_index`,
`.left_clearance_m`, and `.right_clearance_m` for existing consumers.

Use `np.interp` for scalar arrays and interpolate unwrapped yaw before normalizing. In `__post_init__`, copy every array to the required dtype, validate lengths/finite values/strictly increasing `s`, set `writeable=False`, replace the field with `object.__setattr__`, and wrap copied metadata with `types.MappingProxyType`. Import `List`, `Tuple`, `Dict`, and `Mapping` from `typing`; do not use Python 3.9 built-in generic syntax.

- [ ] **Step 4: Write failing canonical-hash and round-trip tests**

```python
def test_hash_excludes_execution_duration_but_includes_geometry(self):
    first = make_trajectory(metadata={"planner_version": 1})
    second = make_trajectory(metadata={"planner_version": 1})
    self.assertEqual(first.content_hash(), second.content_hash())
    changed = replace_geometry(second, x=[0.0, 1.1, 2.0])
    self.assertNotEqual(first.content_hash(), changed.content_hash())

def test_json_round_trip_preserves_hash_and_road_options(self):
    save_reference_trajectory(self.path, trajectory, {"planning_duration_s": 1.23})
    loaded = load_reference_trajectory(self.path)
    self.assertEqual(loaded.content_hash(), trajectory.content_hash())
    self.assertEqual(loaded.road_options, trajectory.road_options)
```

- [ ] **Step 5: Implement canonical hashing and JSON replay**

Hash, in declared field order, canonical little-endian float64/int64 bytes plus key-sorted compact UTF-8 JSON for deterministic metadata. Explicitly remove `planning_duration_s`, timestamps, paths, and output-directory values from hashed metadata. Save those values under a separate `execution` object in JSON.

- [ ] **Step 6: Convert the existing legacy reference trace through the common model**

Keep `build_feasible_reference_trace()` behavior and return type intact. Add a thin helper:

```python
def build_legacy_reference_trajectory(route_trace, config=None, metadata=None):
    reference_trace, features = build_feasible_reference_trace(route_trace, config)
    trajectory = trajectory_from_route_trace(reference_trace, metadata=metadata)
    return trajectory, reference_trace, features
```

- [ ] **Step 7: Run focused tests to verify GREEN**

```powershell
python -m unittest PythonAPI.my_control_project.tests.test_reference_trajectory PythonAPI.my_control_project.tests.test_reference_path -v
```

Expected: all trajectory and existing legacy-reference tests pass.

- [ ] **Step 8: Commit Task 1**

```powershell
git add -- PythonAPI/my_control_project/road_planning/reference_trajectory.py PythonAPI/my_control_project/road_planning/reference_path.py PythonAPI/my_control_project/tests/test_reference_trajectory.py
git commit -m "feat: add replayable reference trajectories"
```

### Task 2: Constrained Frenet Candidate Planner

**Files:**
- Create: `PythonAPI/my_control_project/road_planning/frenet_planner.py`
- Create: `PythonAPI/my_control_project/tests/test_frenet_planner.py`

**Interfaces:**
- Consumes: `ReferenceTrajectory` from Task 1 and CARLA waypoint-like route traces.
- Produces: `FrenetPlannerConfig`, `FrenetPlanningFailure`, `quintic_transition()`, `build_centerline_model()`, and `plan_frenet_reference()`.

- [ ] **Step 1: Write failing quintic-boundary tests**

```python
def test_quintic_transition_has_zero_first_and_second_derivatives_at_ends(self):
    s = np.linspace(0.0, 40.0, 161)
    d = quintic_transition(s, 0.0, 40.0, 0.0, 0.6)
    first = np.gradient(d, s)
    second = np.gradient(first, s)
    self.assertAlmostEqual(d[0], 0.0, places=9)
    self.assertAlmostEqual(d[-1], 0.6, places=9)
    self.assertLess(abs(first[0]), 1e-3)
    self.assertLess(abs(first[-1]), 1e-3)
    self.assertLess(abs(second[0]), 1e-3)
    self.assertLess(abs(second[-1]), 1e-3)
```

- [ ] **Step 2: Verify RED, then implement the normalized quintic smoothstep**

Run the focused test and confirm the missing symbol. Implement `h(u)=6u^5-15u^4+10u^3`, clamped to `[0,1]`, and return `d0 + (d1-d0)*h(u)`.

- [ ] **Step 3: Write failing centerline density-invariance tests**

Build the same L-shaped road at raw spacings `0.5 m` and `2.0 m`. Require both centerline models sampled at `1.0 m` to agree within `0.03 m`, have unwrapped yaw, and contain no duplicate arc-length samples.

- [ ] **Step 4: Implement `FrenetPlannerConfig` and centerline construction**

```python
@dataclass(frozen=True)
class FrenetPlannerConfig:
    output_spacing_m: float = 1.0
    validation_spacing_m: float = 0.25
    vehicle_half_width_m: float = 1.05
    lane_margin_m: float = 0.35
    offset_fractions: tuple = (-0.75, -0.50, -0.25, 0.0, 0.25, 0.50, 0.75)
    transition_lengths_m: tuple = (20.0, 40.0, 60.0)
    lateral_beam_width: int = 64
    candidate_cap: int = 512
    deadline_s: float = 5.0
    max_abs_curvature_1pm: float = 0.20
    max_abs_curvature_rate_1pm2: float = 0.020
    max_lateral_accel_mps2: float = 6.5
    max_lateral_jerk_mps3: float = 10.0

@dataclass(frozen=True)
class CenterlineModel:
    s_m: np.ndarray
    x_m: np.ndarray
    y_m: np.ndarray
    z_m: np.ndarray
    yaw_rad: np.ndarray
    curvature_1pm: np.ndarray
    lane_width_m: np.ndarray
    source_index: np.ndarray
    road_options: tuple
```

Remove consecutive points closer than `1e-4 m`, resample by cumulative distance, unwrap yaw, and compute signed curvature from yaw gradient over `s`.

- [ ] **Step 5: Write failing footprint, curvature, curvature-rate, and speed-cap tests**

Create fixtures where a center-only offset fits but `vehicle_half_width + margin` does not; where curvature exceeds `0.20`; and where a sharp curvature step exceeds `0.020`. Require rejection counts keyed by `lane_clearance`, `curvature`, and `curvature_rate`. Require speed caps to equal the minimum of target, lateral-acceleration, and lateral-jerk limits.

- [ ] **Step 6: Implement candidate expansion and validation**

Detect curve windows at `abs(curvature) >= 0.005`, merge gaps shorter than `10 m`, and expand each lateral-profile beam state with the fixed offset fractions and transition lengths. Use quintic ramps into and out of each window; positive `d` follows the centerline left normal. Retain the lowest partial geometric-cost states up to `64`, evaluate no more than `512` complete candidates, and use stable tuple IDs.

For each dense validation sample, compute:

```python
usable = lane_width_m * 0.5 - config.vehicle_half_width_m - config.lane_margin_m
left_clearance = lane_width_m * 0.5 - d - config.vehicle_half_width_m
right_clearance = lane_width_m * 0.5 + d - config.vehicle_half_width_m
curvature_rate = np.gradient(curvature, s_m)
speed_cap = min(
    requested_speed_mps,
    np.sqrt(6.5 / abs(curvature)) if abs(curvature) > 1e-9 else np.inf,
    np.cbrt(10.0 / abs(curvature_rate)) if abs(curvature_rate) > 1e-9 else np.inf,
)
```

- [ ] **Step 7: Write failing deterministic scoring and timeout tests**

Assert stable candidate IDs break exact ties, all score components are recorded, changing a controller name supplied only in irrelevant metadata cannot change geometry/hash, and an injected clock crossing the deadline raises `FrenetPlanningFailure(code="planning_timeout")` with no trajectory.

- [ ] **Step 8: Implement scoring and public planner**

```python
class FrenetPlanningFailure(RuntimeError):
    def __init__(self, code, rejection_counts, message):
        super().__init__(message)
        self.code = str(code)
        self.rejection_counts = dict(rejection_counts)

def plan_frenet_reference(
    route_trace,
    route_features,
    target_speed_kmh,
    config=None,
    clock=time.perf_counter,
) -> Tuple[ReferenceTrajectory, Dict[str, object]]:
    config = config or FrenetPlannerConfig()
    started = clock()
    centerline = build_centerline_model(route_trace, config)
    candidates = generate_lateral_candidates(centerline, config)
    evaluated, rejection_counts = evaluate_candidates(
        centerline, candidates, target_speed_kmh, route_features, config
    )
    if clock() - started > config.deadline_s:
        raise FrenetPlanningFailure("planning_timeout", rejection_counts, "Frenet planning deadline exceeded.")
    if not evaluated:
        raise FrenetPlanningFailure("no_feasible_candidate", rejection_counts, "No feasible Frenet trajectory.")
    selected = min(evaluated, key=lambda item: (item.total_score, item.candidate_id))
    trajectory = candidate_to_trajectory(centerline, selected, target_speed_kmh, config)
    return trajectory, build_planner_diagnostics(selected, evaluated, rejection_counts, started, clock())
```

Normalize and weight the seven score terms exactly as specified globally. Exclude controller fields from metadata. Return the selected trajectory plus planner diagnostics containing candidate count, rejection counts, component scores, selected candidate ID, and cap state.

- [ ] **Step 9: Verify Task 2 GREEN**

```powershell
python -m unittest PythonAPI.my_control_project.tests.test_frenet_planner PythonAPI.my_control_project.tests.test_reference_trajectory -v
```

Expected: all Frenet and trajectory tests pass.

- [ ] **Step 10: Commit Task 2**

```powershell
git add -- PythonAPI/my_control_project/road_planning/frenet_planner.py PythonAPI/my_control_project/tests/test_frenet_planner.py
git commit -m "feat: add constrained Frenet reference planner"
```

### Task 3: Deterministic and Diverse Topology Search

**Files:**
- Modify: `PythonAPI/my_control_project/road_planning/route_planner.py`
- Modify: `PythonAPI/my_control_project/tests/test_route_planner.py`

**Interfaces:**
- Produces: the existing `build_shaped_route()` return contract with better candidate coverage; `_collect_route_candidates()` gains `route_shape`, `target_speed_kmh`, and `beam_width=24`.
- Consumes: existing CARLA waypoint `.next()`, IDs, road/section/lane IDs when available.

- [ ] **Step 1: Write failing full-loop and diversity tests**

Use fake waypoint graphs. One fixture revisits a waypoint after more than five hops and must be rejected. Another offers many straight branches plus one S-curve branch; beam pruning must retain at least one state from each non-empty turn-history bucket.

```python
def test_candidate_search_rejects_long_loop(self):
    candidates = _collect_route_candidates(start, 1.0, 20.0, 100, 0.05, "curvy", 50.0, beam_width=24)
    self.assertTrue(all(len({wp.id for wp, _ in route}) == len(route) for route in candidates))

def test_beam_keeps_turn_history_diversity(self):
    candidates = _collect_route_candidates(start, 1.0, 12.0, 100, 0.05, "s_curve", 50.0, beam_width=4)
    features = [_route_shape_features(route) for route in candidates]
    self.assertTrue(any(item["sign_changes"] >= 1 for item in features))
```

- [ ] **Step 2: Verify RED**

```powershell
python -m unittest PythonAPI.my_control_project.tests.test_route_planner -v
```

Expected: new tests fail because only the last five waypoint IDs are checked and beam pruning is length-first.

- [ ] **Step 3: Implement partial-state summaries and loop keys**

Add immutable state fields `visited_waypoint_ids`, `visited_edge_keys`, `left_turn`, `right_turn`, `sign_changes`, `max_abs_curvature`, and `curvature_sum`. Use `(road_id, section_id, lane_id, waypoint_id)` when fields exist and waypoint ID otherwise.

- [ ] **Step 4: Implement deterministic bucketed beam pruning**

Bucket by `(turn_sign, min(sign_changes, 2), road_id, lane_id)`. Sort each bucket using normalized partial length/shape/speed cost plus stable route-ID tuple. Retain one best state per bucket up to beam width, then fill remaining slots globally. Do not early-prune a shape merely because it has not yet accumulated the final turn count.

- [ ] **Step 5: Pass planner intent into candidate collection**

Update `build_shaped_route()` to call:

```python
candidates = _collect_route_candidates(
    start_wp,
    sampling_resolution,
    min_length_m,
    max_waypoints,
    length_tolerance,
    route_shape,
    target_speed_kmh,
    beam_width=24,
)
```

Keep final `_select_shaped_candidate()` feasibility and scoring as the definitive gate.

- [ ] **Step 6: Verify GREEN and regression**

```powershell
python -m unittest PythonAPI.my_control_project.tests.test_route_planner -v
```

Expected: all old and new route-planner tests pass.

- [ ] **Step 7: Commit Task 3**

```powershell
git add -- PythonAPI/my_control_project/road_planning/route_planner.py PythonAPI/my_control_project/tests/test_route_planner.py
git commit -m "feat: preserve diverse feasible route branches"
```

### Task 4: Continuous Arc-Length Reference Tracker

**Files:**
- Create: `PythonAPI/my_control_project/road_planning/reference_tracker.py`
- Create: `PythonAPI/my_control_project/tests/test_reference_tracker.py`

**Interfaces:**
- Consumes: `ReferenceTrajectory` from Task 1.
- Produces: `ProjectionResult`, `ReferenceTracker.__init__()`, `ReferenceTracker.update()`, `target_sample()`, `curvature_preview()`, and `is_complete()`.

- [ ] **Step 1: Write failing continuous-projection tests**

Test a vehicle halfway along a sparse segment and require `s_ref=5.0`, not either endpoint. Repeat with a dense equivalent trajectory and require the same projection and target within `1e-6`.

```python
result = tracker.update(x_m=5.0, y_m=1.0, yaw_rad=0.0, speed_mps=10.0, control_dt=0.05)
self.assertAlmostEqual(result.s_ref_m, 5.0)
self.assertAlmostEqual(result.lateral_error_m, 1.0)
self.assertAlmostEqual(tracker.target_sample(10.0).s_m, 15.0)
```

- [ ] **Step 2: Verify RED, then implement segment projection**

Define:

```python
@dataclass(frozen=True)
class ProjectionResult:
    s_ref_m: float
    reference_sample: TrajectorySample
    segment_index: int
    projection_distance_m: float
    lateral_error_m: float
    heading_error_rad: float
    progress_delta_m: float
    state: str
    held: bool
```

Implement `ReferenceTracker.__init__(trajectory, max_rollback_m=2.0,
hold_steps=3)`, `ReferenceTracker.update(x_m, y_m, yaw_rad, speed_mps, control_dt,
allow_recovery=False) -> ProjectionResult`, `target_sample(lookahead_m) ->
TrajectorySample`, `curvature_preview(distances_m) -> List[float]`, and
`is_complete(x_m, y_m) -> bool` with exactly those signatures.

Project with clamped dot product onto each candidate segment and compute signed lateral error from the interpolated tangent normal.

- [ ] **Step 3: Write failing crossing, opposite-lane, monotonicity, and recovery tests**

Create a figure-eight fixture with two geometrically coincident segments. Require prior `s` plus heading to select the correct branch. Reject candidates with heading mismatch over 90 degrees. Require normal progress to remain monotonic, allow at most `2 m` rollback only with `allow_recovery=True`, and reject a jump larger than `max(5, 3*v*dt)` on a branched path.

- [ ] **Step 4: Implement bounded search and hold/failure state**

Use the normal `previous_s-2` to `previous_s+max(120, 3*speed)` window. When off-route or no valid segment, use at most `previous_s-10` to `previous_s+200`. Hold the last valid reference for three cycles with state `held`; the fourth consecutive failure raises `ReferenceTrackingFailure` with the last failure reason.

- [ ] **Step 5: Write failing preview and completion tests**

Require curvature samples at exact requested physical offsets. Require completion only when remaining arc length is at most `2 m` and endpoint distance is at most `8 m`; a target sample clamped to the endpoint alone must not complete.

- [ ] **Step 6: Implement preview and completion**

Use `trajectory.sample_at(min(s_ref + distance, total_length))`. Record the actual clamped sample positions so logging can expose them.

- [ ] **Step 7: Verify Task 4 GREEN**

```powershell
python -m unittest PythonAPI.my_control_project.tests.test_reference_tracker -v
```

Expected: all projection, crossing, recovery, preview, and completion tests pass.

- [ ] **Step 8: Commit Task 4**

```powershell
git add -- PythonAPI/my_control_project/road_planning/reference_tracker.py PythonAPI/my_control_project/tests/test_reference_tracker.py
git commit -m "feat: track frozen routes by continuous arc length"
```

### Task 5: Planner Configuration, Freezing, and Run Artifact

**Files:**
- Modify: `PythonAPI/my_control_project/project_config.py`
- Modify: `PythonAPI/my_control_project/run_my_control.py`
- Modify: `PythonAPI/my_control_project/experiment/runtime.py`
- Modify: `PythonAPI/my_control_project/tests/test_run_args.py`
- Modify: `PythonAPI/my_control_project/tests/test_speed_planner.py`

**Interfaces:**
- Consumes: planners from Tasks 1–3.
- Produces: `--planner-mode`, exact planner/tracker CLI configuration, a saved `reference_trajectory.json`, and a route setup that returns the immutable trajectory.

- [ ] **Step 1: Write failing CLI-default and override tests**

Require `planner_mode == "legacy"`, all version-one numeric defaults, and explicit `--planner-mode frenet`. Require strict-comparison configuration to accept `--speed-planner-limit-profile global` unchanged.

- [ ] **Step 2: Verify RED**

```powershell
python -m unittest PythonAPI.my_control_project.tests.test_run_args -v
```

Expected: missing planner arguments and defaults.

- [ ] **Step 3: Add planner and tracker configuration**

Add `DEFAULTS` entries and `CLI_ARGUMENTS` for:

```text
planner_mode, planner_validation_spacing, planner_vehicle_half_width,
planner_lane_margin, planner_topology_beam_width, planner_lateral_beam_width,
planner_candidate_cap, planner_deadline, planner_max_curvature,
planner_max_curvature_rate, planner_max_lateral_accel, planner_max_lateral_jerk,
tracker_max_rollback, tracker_hold_steps
```

Expose `--planner-mode {legacy,frenet}` with default `legacy`. Keep offset fractions, transition lengths, and scoring weights as versioned constants in `frenet_planner.py`, not free-form CLI strings.

- [ ] **Step 4: Write failing route-setup selection tests**

Patch `build_legacy_reference_trajectory` and `plan_frenet_reference`. Assert `resolve_route_setup()` calls exactly one selected planner, returns both `route_trace` and `ReferenceTrajectory`, and passes controller-neutral inputs only.

- [ ] **Step 5: Implement planner selection in `resolve_route_setup()`**

After `build_shaped_route()`, construct either:

```python
if arg_value(args, "planner_mode") == "frenet":
    trajectory, planner_features = plan_frenet_reference(
        route_trace, route_features, arg_value(args, "target_speed"), config
    )
    reference_trace = trajectory.to_route_trace()
else:
    trajectory, reference_trace, planner_features = build_legacy_reference_trajectory(
        route_trace, legacy_config, metadata
    )
```

Merge features without deleting raw topology features. Return
`spawn_index, destination_index, spawn_point, destination, reference_trace, route_features, trajectory`.

- [ ] **Step 6: Write failing freeze-and-hash run-config tests**

Require `build_run_config()` to contain planner mode/version/hash/config and require a mismatched hash before a controller lap to raise an error. Require execution duration to be recorded but excluded from hash.

- [ ] **Step 7: Freeze the artifact before controller execution**

Move timestamp/output-directory creation to immediately after route setup. Create the directory, write `reference_trajectory.json`, store `frozen_hash = trajectory.content_hash()`, and assert the same hash immediately before each controller lap. Pass the same in-memory trajectory object into every lap.

- [ ] **Step 8: Verify Task 5 GREEN**

```powershell
python -m unittest PythonAPI.my_control_project.tests.test_run_args PythonAPI.my_control_project.tests.test_speed_planner -v
```

Expected: planner CLI, selection, run-config, and existing speed-planner tests pass.

- [ ] **Step 9: Commit Task 5**

```powershell
git add -- PythonAPI/my_control_project/project_config.py PythonAPI/my_control_project/run_my_control.py PythonAPI/my_control_project/experiment/runtime.py PythonAPI/my_control_project/tests/test_run_args.py PythonAPI/my_control_project/tests/test_speed_planner.py
git commit -m "feat: freeze controller-neutral planner artifacts"
```

### Task 6: Integrate Continuous Tracking into Every Controller Lap

**Files:**
- Modify: `PythonAPI/my_control_project/experiment/runtime.py`
- Modify: `PythonAPI/my_control_project/tests/test_speed_planner.py`

**Interfaces:**
- Consumes: `ReferenceTracker` and frozen trajectory.
- Produces: the existing controller `run_step()` call contract with continuous reference waypoints/errors and physical-distance curvature preview.

- [ ] **Step 1: Write failing runtime projection and physical-preview tests**

Patch a synthetic trajectory and tracker result. Assert the error provider receives the exact interpolated projection waypoint, controller receives the exact arc-length target waypoint, and speed/controller curvature previews are sampled at requested metres rather than list indices.

- [ ] **Step 2: Verify RED**

```powershell
python -m unittest PythonAPI.my_control_project.tests.test_speed_planner.RuntimeSpeedPlanningTest -v
```

Expected: new tests fail because runtime still calls `select_route_target()` and index-based preview functions.

- [ ] **Step 3: Replace index-based target selection in the lap loop**

Change the signature to
`run_controller_lap(controller_name, world, blueprint_library, vehicle_bp,
spawn_point, route_trace, trajectory, args, display, display_width,
display_height)`. Create
`ReferenceTracker(trajectory, max_rollback_m=args.tracker_max_rollback,
hold_steps=args.tracker_hold_steps)` for each lap. On
each tick:

```python
projection = tracker.update(
    vehicle_loc.x,
    vehicle_loc.y,
    np.radians(vehicle.get_transform().rotation.yaw),
    speed_ms,
    getattr(args, "control_dt", CONTROL_DT),
    allow_recovery=collision_count[0] > 0 or speed_ms < 1.0,
)
target_sample = tracker.target_sample(compute_lookahead(args.look_ahead, speed_ms))
reference_waypoint = trajectory.waypoint_at(projection.s_ref_m)
target_waypoint = trajectory.waypoint_at(target_sample.s_m)
```

Use `projection.segment_index` for backward-compatible log indices only.

- [ ] **Step 4: Replace curvature-index functions with arc-length requests**

Controller preview distances are `[step * max(speed_mps*dt*curve_scale, 1.0) for step in range(horizon)]`. Speed-planner preview distances are `[step * max(speed_mps*0.80, 7.0) for step in range(6)]`. Pass `tracker.curvature_preview(distances)` into existing controller and speed-planner interfaces.

- [ ] **Step 5: Add deterministic failure and completion handling**

Convert sustained `ReferenceTrackingFailure` into a lap result with failure reason. Replace `route_index >= len(route_trace)-2` with `tracker.is_complete(vehicle_loc.x, vehicle_loc.y)`. Do not report a held or failed reference as route completion.

- [ ] **Step 6: Verify runtime GREEN and focused regressions**

```powershell
python -m unittest PythonAPI.my_control_project.tests.test_speed_planner PythonAPI.my_control_project.tests.test_error_providers PythonAPI.my_control_project.tests.test_dynamic_target_propagation -v
```

Expected: all runtime, provider, and target-propagation tests pass.

- [ ] **Step 7: Commit Task 6**

```powershell
git add -- PythonAPI/my_control_project/experiment/runtime.py PythonAPI/my_control_project/tests/test_speed_planner.py
git commit -m "feat: use continuous reference tracking at runtime"
```

### Task 7: Planner Observability and Paired Experiment Matrix

**Files:**
- Create: `PythonAPI/my_control_project/experiment/planner_comparison.py`
- Create: `PythonAPI/my_control_project/tests/test_planner_comparison.py`
- Modify: `PythonAPI/my_control_project/experiment/logging.py`
- Modify: `PythonAPI/my_control_project/experiment/metrics.py`
- Modify: `PythonAPI/my_control_project/experiment/__init__.py`
- Modify: `PythonAPI/my_control_project/scripts/run_test_matrix.ps1`
- Modify: `PythonAPI/my_control_project/tests/test_speed_planner.py`
- Modify: `PythonAPI/my_control_project/tests/test_matrix_script.py`

**Interfaces:**
- Consumes: planner metadata, trajectory hash, and `ProjectionResult` debug values.
- Produces: appended step/summary fields and `pair_planner_rows(rows) -> List[Dict[str, object]]` plus a `_paired.csv` matrix artifact.

- [ ] **Step 1: Write failing log-header and summary tests**

Require appended step fields:

```text
planner_mode, trajectory_hash, reference_s_m, target_s_m,
projection_segment, projection_distance_m, progress_delta_m,
reference_state, reference_held, curvature_preview_s_m
```

Require summary fields for planner version, hash, planning duration, candidate count, selected ID, maximum curvature/rate, minimum footprint clearance, and rejection counts.

- [ ] **Step 2: Verify RED and append logging fields without reordering existing columns**

Run `test_speed_planner`; confirm missing fields. Append new fields at the end of headers and row builders. Serialize preview positions and rejection counts as compact JSON strings.

- [ ] **Step 3: Write failing paired-delta tests**

```python
def test_pairs_legacy_and_frenet_by_scenario_controller_and_mode(self):
    paired = pair_planner_rows([legacy_row, frenet_row])
    self.assertEqual(len(paired), 1)
    self.assertAlmostEqual(paired[0]["rms_e_y_improvement_pct"], 20.0)
    self.assertAlmostEqual(paired[0]["p95_steer_delta_change"], -0.002)

def test_refuses_pair_with_mismatched_controller_or_missing_hash(self):
    with self.assertRaisesRegex(ValueError, "trajectory_hash"):
        pair_planner_rows([bad_legacy_row, frenet_row])
```

- [ ] **Step 4: Implement paired comparison**

Group rows by map, seed, requested route shape, target speed, speed mode, error provider, and controller. Require one legacy and one Frenet row. Calculate route-generation status, stability delta, `rms_e_y` improvement percentage, P95 steering delta change, steering-reversal change, route-completion change, and hard-safety regressions.

Expose a small CLI:

```powershell
python PythonAPI/my_control_project/experiment/planner_comparison.py --input matrix.csv --output matrix_paired.csv
```

- [ ] **Step 5: Write failing matrix-script tests**

Require the stability catalog to run both `legacy` and `frenet`, pass `--speed-planner-limit-profile global`, preserve the same seed/map/shape/speed tuple, and invoke paired comparison after raw CSV creation.

- [ ] **Step 6: Update the PowerShell matrix**

Add `PlannerMode` with `legacy`, `frenet`, or `both`, defaulting to `both` for the `stability` set. Add planner mode to records and commands. Generate raw matrix first, then paired CSV. Exit nonzero for execution/stability failures and for any paired hard-safety regression.

- [ ] **Step 7: Verify Task 7 GREEN**

```powershell
python -m unittest PythonAPI.my_control_project.tests.test_planner_comparison PythonAPI.my_control_project.tests.test_matrix_script PythonAPI.my_control_project.tests.test_speed_planner -v
```

Expected: all logging, paired comparison, and matrix tests pass.

- [ ] **Step 8: Commit Task 7**

```powershell
git add -- PythonAPI/my_control_project/experiment/planner_comparison.py PythonAPI/my_control_project/experiment/logging.py PythonAPI/my_control_project/experiment/metrics.py PythonAPI/my_control_project/experiment/__init__.py PythonAPI/my_control_project/scripts/run_test_matrix.ps1 PythonAPI/my_control_project/tests/test_planner_comparison.py PythonAPI/my_control_project/tests/test_matrix_script.py PythonAPI/my_control_project/tests/test_speed_planner.py
git commit -m "feat: compare legacy and Frenet planner outcomes"
```

### Task 8: Documentation and Full Verification

**Files:**
- Modify: `PythonAPI/my_control_project/README.md`
- Modify: `README.md`
- Verify all files changed by Tasks 1–7.

**Interfaces:**
- Consumes: completed planner, tracker, CLI, artifacts, and matrix workflow.
- Produces: user-facing commands and fresh verification evidence.

- [ ] **Step 1: Document planner modes and strict comparison**

Document:

```powershell
# Opt-in Frenet single comparison
.\scripts\run_my_control.ps1 --planner-mode frenet --speed-planner-limit-profile global --controllers lqr pid mpc

# Paired legacy/Frenet stability matrix
powershell -ExecutionPolicy Bypass -File .\scripts\run_test_matrix.ps1 -ScenarioSet stability -PlannerMode both
```

Explain that `reference_trajectory.json` is frozen before laps, all controllers must share its hash, and `legacy` remains default until simulator gates pass.

- [ ] **Step 2: Run the complete unit suite**

```powershell
python -m unittest discover -s PythonAPI\my_control_project\tests -v
```

Expected: zero failures and zero errors, including all pre-existing 141 tests and new planner/tracker tests.

- [ ] **Step 3: Run static syntax verification**

```powershell
python -m compileall -q PythonAPI\my_control_project
```

Expected: exit code 0 and no syntax errors.

- [ ] **Step 4: Run paired smoke verification with CARLA**

```powershell
powershell -ExecutionPolicy Bypass -File PythonAPI\my_control_project\scripts\run_test_matrix.ps1 -ScenarioSet smoke -PlannerMode both
```

Expected: every controller run completes, trajectory artifacts exist, strict runs use the global limit profile, and paired CSV is created.

- [ ] **Step 5: Run the full paired stability matrix**

```powershell
powershell -ExecutionPolicy Bypass -File PythonAPI\my_control_project\scripts\run_test_matrix.ps1 -ScenarioSet stability -PlannerMode both
```

Require:

- no new collision, lane-boundary, route-completion, or lateral-acceleration failure;
- Frenet stability-pass count not lower than legacy;
- paired median `rms_e_y` improvement at least `5%`;
- aggregate P95 steering delta increase no greater than `max(2%, 0.0001)`;
- aggregate significant steering-reversal increase no greater than `0.1 events/10 s`;
- route-generation success not lower than legacy.

If CARLA is unavailable, record simulator verification as pending and do not make simulator-level improvement claims or switch the default planner.

- [ ] **Step 6: Inspect requirements and diff**

```powershell
git diff --check
git status --short
git diff --stat ff1ebd54..HEAD
```

Confirm no control-law file changed and no unrelated user file was staged or committed.

- [ ] **Step 7: Commit documentation**

```powershell
git add -- README.md PythonAPI/my_control_project/README.md
git commit -m "docs: explain reproducible Frenet comparisons"
```

- [ ] **Step 8: Apply the rollout gate**

Keep `planner_mode="legacy"` unless every unit, smoke, and full paired stability gate passes. If all gates pass, change only the default to `frenet`, update the two CLI-default tests and documentation, rerun the complete unit suite, and commit:

```powershell
git add -- PythonAPI/my_control_project/project_config.py PythonAPI/my_control_project/tests/test_run_args.py README.md PythonAPI/my_control_project/README.md
git commit -m "feat: enable validated Frenet planner by default"
```

Do not perform this step on unit evidence alone.
