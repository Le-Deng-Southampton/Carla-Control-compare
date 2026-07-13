# Frenet Production Enablement Design

## Objective

Make the constrained Frenet planner usable as the normal project planner
without weakening its physical safety limits or the interpretability of the
PID, LQR, and MPC comparison experiments.

The implementation must remove the false curvature-rate rejection observed on
the Town04 smoke route, make Frenet the requested planner for ordinary runs,
and preserve a clearly reported legacy fallback for expected planning
failures. Strict comparison runs must never fall back silently.

## Confirmed Root Cause

The current planner creates a reference trajectory on a 1.0 m output grid,
linearly interpolates that polyline to a 0.25 m validation grid, and then
computes yaw, curvature, and curvature rate with successive numerical
gradients on the dense grid. Linear interpolation does not add geometric
information between the 1.0 m samples. Instead, its segment-heading changes
become concentrated around interpolation knots, so the second derivative
contains sampling artifacts.

The saved Town04 reference demonstrates the scale of the problem. Its maximum
absolute curvature rate is approximately `0.00249 1/m^2` when evaluated on the
1.0 m trajectory grid. Densifying the same coordinates to 0.25 m with linear
interpolation before differentiating produces an artificial maximum of about
`0.0573 1/m^2`, which exceeds the unchanged `0.020 1/m^2` safety limit. No new
road bend is introduced by the interpolation, so the dense-grid peak is not a
valid reason to reject the route.

## Selected Approach: Two-Scale Validation

Validation will separate geometric occupancy checks from differential-geometry
checks:

- Evaluate lane width, vehicle footprint, lateral offset, and minimum lane
  clearance on the existing 0.25 m dense grid. These quantities benefit from
  dense spatial sampling and do not require numerical differentiation.
- Build the candidate coordinates, yaw, curvature, and curvature rate on the
  actual 1.0 m output grid used to freeze and replay the reference trajectory.
- Apply the existing maximum absolute curvature limit of `0.20 1/m` and maximum
  absolute curvature-rate limit of `0.020 1/m^2` to those output-grid values.
- Keep the same output-grid values for scoring, speed-cap calculation,
  diagnostics, serialization, hashing, and controller preview. Validation and
  execution therefore describe the same differential geometry.

This change does not relax either threshold. It removes a discretization
inconsistency while retaining dense lane-clearance validation.

### Rejected Alternatives

Raising the curvature-rate threshold would make the smoke route pass by
weakening a safety limit and would hide genuinely abrupt paths. It is not
acceptable.

A continuous cubic-spline geometry model with analytic derivatives could
provide sub-meter differential geometry, but it adds interpolation policy,
overshoot handling, lane-boundary validation, and new numerical dependencies
that are unnecessary for this defect. It can be considered later if a 1.0 m
frozen reference is replaced by a genuinely continuous reference model.

## Planner Selection and Fallback

Ordinary project runs will request Frenet by default. A new explicit fallback
policy will distinguish production availability from strict experiments:

- `--planner-mode {legacy,frenet}` defaults to `frenet`.
- `--planner-fallback {legacy,error}` defaults to `legacy` for ordinary runs.
- With `--planner-fallback legacy`, only an expected `FrenetPlanningFailure`
  such as `no_feasible_candidate` or `planning_timeout` triggers construction
  of a legacy reference trajectory.
- Unexpected exceptions are never caught as fallback conditions; they remain
  visible failures so programming defects are not hidden.
- Paired and strict comparison scripts pass `--planner-fallback error`. A
  Frenet planning failure is then recorded as a failed Frenet case rather than
  converted into a legacy result.
- Explicit `--planner-mode legacy` constructs legacy directly and does not
  report a fallback.

Changing the default to Frenet is conditional on the Town04 CARLA smoke gate
passing after the geometry fix. If that gate fails, the implementation remains
opt-in while the failure is investigated; it must not be described as enabled
by default.

## Data Flow and Reporting

Route setup will preserve both requested and resolved planner identity:

1. CARLA topology produces the controller-neutral legal route trace.
2. The requested planner attempts to build one immutable reference trajectory.
3. An expected Frenet failure either propagates (`error`) or invokes the legacy
   builder (`legacy`).
4. The resolved trajectory is serialized before any controller lap.
5. PID, LQR, and MPC each receive a fresh tracker over the same in-memory
   trajectory and must observe the same trajectory hash.

Run configuration, trajectory execution metadata, per-controller summaries,
and paired experiment rows will expose:

- `planner_mode_requested`;
- `planner_mode_resolved`;
- `planner_fallback_policy`;
- `planner_fallback_used`;
- `planner_fallback_code` and message when used;
- Frenet rejection counts when available;
- the resolved trajectory hash.

Existing `planner_mode` fields used by comparisons will represent the resolved
planner. This prevents a fallback legacy run from being mislabeled as Frenet.
The requested mode remains available as a separate audit field.

## Interpretability Guarantees

The change remains controller-neutral:

- No PID, LQR, or MPC control law or gain is modified by this work.
- Candidate generation, selection, fallback, and trajectory freezing occur
  before the first controller lap.
- Every controller in one run tracks the same resolved trajectory hash.
- Strict paired experiments prohibit fallback and group results by resolved
  planner mode.
- Production fallback results are not eligible to count as Frenet successes in
  legacy-versus-Frenet comparisons.

## Error Handling

`FrenetPlanningFailure` will retain its structured code and rejection counts.
Fallback metadata will be constructed from that exception without changing the
exception raised under the `error` policy. Unsupported fallback values remain
argument-parser errors.

If legacy construction also fails after an allowed Frenet fallback, the legacy
exception propagates. The system will not retry repeatedly or return a partial
trajectory.

## Testing and Rollout Gates

### Unit Regression Tests

- Reproduce the aliasing defect with a smooth 1.0 m sampled path: dense lane
  validation must pass and the planner must not reject it due only to the
  0.25 m interpolation derivative artifact.
- Retain a genuinely abrupt heading/curvature fixture that exceeds
  `0.020 1/m^2`; it must still be rejected as `curvature_rate`.
- Verify lane clearance is still evaluated at 0.25 m spacing.
- Verify Frenet success performs no fallback.
- Verify `FrenetPlanningFailure` falls back only under the `legacy` policy and
  records requested/resolved modes, failure code, rejection counts, and hash.
- Verify the `error` policy propagates the same structured planning failure.
- Verify an unexpected exception never triggers fallback.
- Verify explicit legacy mode never invokes Frenet.
- Verify all controllers receive the same resolved trajectory hash.
- Verify strict matrix commands include `--planner-fallback error`.

### Static and Repository Verification

- Run the complete Python unit-test suite.
- Run Python byte-code compilation for `PythonAPI/my_control_project`.
- Run `git diff --check`.

### CARLA Smoke Gate

Run the existing Town04, seed `26050101`, approximately 1500 m, 50 km/h smoke
scenario with explicit Frenet mode and `--planner-fallback error`. The gate
requires:

- Frenet produces a feasible trajectory without fallback;
- the stored maximum curvature and curvature rate remain within their unchanged
  limits;
- PID, LQR, and MPC use the same trajectory hash;
- each controller run starts and completes without collision or lane-boundary
  violation;
- route-completion and stability metrics are written normally.

Only after this gate passes will the configuration default change to Frenet.
The full paired stability matrix remains the evidence required for performance
claims; smoke success establishes usability and safe default activation, not
superior tracking performance.

## Scope

This work changes Frenet candidate validation, planner selection/fallback,
reporting, tests, scripts, and documentation. It does not retune controllers,
change route topology search, relax safety limits, add new interpolation
dependencies, or claim that Frenet outperforms legacy before paired matrix
evidence exists.
