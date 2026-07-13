# Explainable Frenet Planning and Lane-Tracking Design

## Goal

Improve route planning and lane tracking across the existing PID, LQR, and MPC
comparison project without weakening experimental interpretability. The new
planner must produce a controller-independent, deterministic reference
trajectory that is frozen before each controller lap and replayed identically
for all three controllers.

Success is measured across the existing fixed-seed ten-scenario stability
matrix. The primary outcome is balanced robustness: stability pass count,
route completion, lateral tracking error, steering smoothness, and route
generation success must improve or remain within the explicit acceptance gates
defined below.

## Evidence and Rationale

The current project already resamples and smooths CARLA waypoints, previews
curvature, and enforces speed-dependent route feasibility. Its remaining
structural weaknesses are:

- Runtime tracking selects the closest discrete waypoint and advances the
  target by waypoint index. Crossings, hairpins, and sampling-density changes
  can therefore create reference-index jumps and inconsistent physical preview
  distances.
- The shaped-route beam search prunes partial routes primarily by length and
  terminal heading, then evaluates route shape and speed suitability only after
  collection. A geometrically useful branch can be removed before its complete
  cost is known.
- Moving-average centerline smoothing is lane-bounded but does not explicitly
  constrain curvature continuity, curvature rate, or the vehicle footprint.

The design follows these primary sources:

- Werling, Ziegler, Kammel, and Thrun, *Optimal Trajectory Generation for
  Dynamic Street Scenarios in a Frenet Frame*, ICRA 2010,
  DOI `10.1109/ROBOT.2010.5509799`. This supports separating road-relative
  longitudinal progress and lateral motion, continuous path projection, and
  constraint-based candidate selection.
- Falcone, Borrelli, Asgari, Tseng, and Hrovat, *Predictive Active Steering
  Control for Autonomous Vehicle Systems*, IEEE TCST 2007,
  DOI `10.1109/TCST.2007.894653`. This supports finite-horizon reference preview
  and explicit steering constraints for trajectory tracking.
- Kong, Pfeiffer, Schildbach, and Borrelli, *Kinematic and Dynamic Vehicle
  Models for Autonomous Driving Control Design*, IEEE IV 2015,
  DOI `10.1109/IVS.2015.7225830`. This supports retaining computationally
  tractable bicycle-model control while limiting reference speed by predicted
  lateral acceleration.
- CARLA's `GlobalRoutePlanner` constructs a directed road-topology graph and
  uses A* with a distance heuristic. The new layer retains CARLA topology
  semantics rather than replacing the map model.

Applying the Frenet formulation in the distance domain is a project-specific
adaptation: the route is static during a controller-comparison lap, so the
lateral profile is parameterized by centerline arc length `s` and frozen before
control begins.

## Scope

### In scope

- A selectable `frenet` route/reference planner alongside the existing
  `legacy` implementation.
- Speed- and shape-aware topology candidate search with deterministic
  diversity and full-path loop rejection.
- Arc-length centerline modeling and distance-domain quintic lateral candidate
  profiles.
- Vehicle-footprint, lane-boundary, curvature, curvature-rate, lateral-
  acceleration, and route-shape constraints.
- A frozen, serializable `ReferenceTrajectory` shared by PID, LQR, and MPC.
- Continuous segment projection, monotonic route progress, and arc-length
  preview for tracking and speed planning.
- Planner diagnostics, deterministic replay, paired legacy/Frenet experiments,
  and additional unit and integration tests.

### Out of scope

- Dynamic obstacle avoidance, lane changing, overtaking, traffic-light
  behavior, and online behavior replanning.
- Controller-specific trajectory scoring or geometry.
- Replacing PID, LQR, or MPC control laws as part of this feature.
- Adding a heavyweight optimization dependency when the required bounded
  candidate search and polynomial evaluation can be implemented with Python
  and NumPy already used by the project.

## Version-One Defaults

All defaults are explicit CLI/config values and are written into the trajectory
artifact. Version one uses:

- centerline/output spacing: `1.0 m`;
- feasibility validation spacing: `0.25 m`;
- Model 3 planning half-width: `1.05 m`;
- body-to-lane-boundary safety margin: `0.35 m`;
- terminal lateral-offset fractions of the locally usable envelope:
  `-0.75, -0.50, -0.25, 0.00, 0.25, 0.50, 0.75`;
- quintic transition lengths: `20 m`, `40 m`, and `60 m`, clipped to the
  available continuous road section;
- topology beam width: `24` with at least one retained state per non-empty
  deterministic turn-history bucket;
- combined lateral-profile beam width: `64`;
- maximum fully evaluated Frenet candidates: `512`;
- planning deadline: `5.0 s` of wall-clock time;
- absolute curvature limit: `0.20 1/m`;
- absolute curvature-rate limit: `0.020 1/m^2`;
- lateral-acceleration limit: `6.5 m/s^2`;
- lateral-jerk speed-cap limit: `10.0 m/s^3`.

The controller-neutral speed cap at each sample is the minimum of the requested
target speed, `sqrt(6.5 / abs(curvature))`, and
`cbrt(10.0 / abs(curvature_rate))`, with zero curvature or curvature rate
treated as unbounded for its respective term.

Normalized score weights are `3.0` route-length error, `1.0` route-shape
mismatch, `0.5` lateral-offset magnitude, `2.0` squared curvature, `1.5`
squared curvature rate, `2.0` inverse clearance, and `3.0` lateral-acceleration
margin. Length is normalized by requested route length, offset by usable lane
envelope, curvature and curvature rate by their limits, clearance by the
configured safety margin, and lateral acceleration by `6.5 m/s^2`.

These values are initial project defaults, not controller-specific tuning. A
future change must increment the planner-version field so artifacts and results
remain comparable.

## Architecture

The route-to-control pipeline has four boundaries:

```text
CARLA topology route candidates
    -> continuous arc-length centerline model
    -> constrained Frenet candidates and controller-neutral selection
    -> frozen ReferenceTrajectory
    -> shared ReferenceTracker
    -> PID / LQR / MPC
```

CARLA topology determines which road and lane sequence is legal. The Frenet
planner determines a smooth, lane-bounded geometric reference inside that legal
corridor. Runtime tracking projects the vehicle onto the frozen result but
never changes its geometry.

The planner may read the map, lane widths, vehicle footprint, requested route
shape, target speed, configuration, and random seed. It must not read the
controller name, controller gains, steering output, or live tracking errors.

Each scenario is planned once. The resulting trajectory is serialized with a
content hash and then loaded for every controller lap. A controller comparison
is invalid if the trajectory hashes differ.

## Core Data Model

`ReferenceTrajectory` is the immutable interface between planning and control.
It contains equally sized arrays for:

- arc length `s_m`;
- world position `x_m`, `y_m`, and `z_m`;
- unwrapped reference heading `yaw_rad`;
- signed curvature `curvature_1pm`;
- curvature rate `curvature_rate_1pm2`;
- left and right lane clearance before subtracting the vehicle footprint;
- controller-neutral recommended speed cap;
- source CARLA road, section, lane, waypoint, and road-option identifiers;
- junction state and source topology-candidate identifier.

It also contains immutable planner metadata: planner mode and version, map,
seed, configuration, vehicle dimensions, total length, feasibility results,
score components, deterministic candidate count, and content hash. Wall-clock
planning duration is execution metadata and is not part of the content hash.

The hash uses canonical little-endian float64 trajectory arrays plus
key-sorted UTF-8 JSON for deterministic planner metadata. Timestamps, output
paths, planning duration, and machine-specific values are excluded.

The existing waypoint-like controller interface remains available through
interpolated reference waypoint objects. This avoids unrelated controller
factory changes while allowing all error and preview geometry to come from the
same continuous trajectory.

## Topology Candidate Search

The existing shaped-route behavior remains available, but partial beam states
gain controller-neutral geometric summaries:

- accumulated length and length error;
- accumulated absolute turn, left/right turn, and turn-sign changes;
- partial maximum and mean curvature estimates;
- estimated lateral acceleration at the requested target speed;
- visited waypoint and road/lane identifiers;
- junction count and terminal heading.

The beam is partitioned into deterministic diversity buckets based on turn
history and road/lane branch. Each bucket retains its best states before the
remaining global beam slots are filled. This prevents a set of nearly identical
straight branches from removing every curve candidate.

Repeated waypoint or directed road/lane edges are rejected across the full
partial path unless the map topology explicitly requires a legal repeated
junction edge. The existing five-waypoint local loop check is insufficient for
long loops and remains only as a fast early check.

Speed infeasibility and impossible route-shape bounds are pruned early only
when the partial state cannot recover by extension. Otherwise the final route
is evaluated after reaching the configured length interval. Candidate ordering
uses stable tuple keys, never object identity or nondeterministic set order.

## Continuous Centerline Model

Every topology candidate is resampled by physical arc length. Duplicate and
near-zero-length segments are removed before tangent calculation. Heading is
unwrapped across `-pi/pi`, and local tangent and normal vectors are derived from
the resampled geometry.

The model exposes interpolation by arbitrary `s`, so downstream code never
converts metres to waypoint indices. Sampling density is an accuracy setting,
not part of the physical controller behavior.

At road-section and junction boundaries, the model preserves legal lane
connectivity and flags discontinuities that cannot be smoothed inside the lane
corridor. It does not bridge unrelated nearby roads in world coordinates.

## Frenet Candidate Generation

The lateral path is represented by distance-domain quintic profiles `d(s)`.
Each profile constrains lateral position, first derivative, and second
derivative at its boundaries, providing continuous position, heading, and
curvature when combined with the continuous centerline.

Candidate terminal offsets and transition lengths are drawn from fixed,
versioned grids. They include centerline travel and limited inside-curve
smoothing. Offset bounds are computed independently at each sample from:

```text
usable half width = lane half width - vehicle half width - safety margin
```

No candidate may enter an adjacent lane merely because its center point remains
inside the original lane. Junction samples use their mapped corridor and a
conservative width when CARLA does not provide a stable lane boundary.

Candidate generation is bounded by a configurable maximum candidate count and
a planning deadline. Reaching the deterministic candidate cap returns the best
fully evaluated feasible candidate after exactly that prefix of the stable
candidate order and records the cap. Reaching the wall-clock deadline returns a
structured `planning_timeout` failure and no trajectory, because accepting a
machine-speed-dependent partial result would violate replay determinism.

## Feasibility Constraints

A candidate is feasible only when all of the following hold at the planner's
validation resolution:

- the complete vehicle footprint retains the configured lane safety margin;
- absolute curvature remains below the vehicle steering-geometry limit;
- absolute curvature rate remains below the configured comfort and actuation
  limit;
- predicted lateral acceleration `v^2 * abs(curvature)` remains at or below
  `6.5 m/s^2` at the candidate's controller-neutral speed cap;
- heading, curvature, and position remain continuous at profile boundaries;
- route length and requested route-shape requirements are satisfied;
- samples remain connected to the source legal topology corridor.

If no candidate is feasible, the planner raises a structured failure containing
candidate rejection counts grouped by reason. `frenet` mode does not silently
return a mislabeled legacy fallback. The caller may explicitly retry another
spawn point or explicitly select `legacy` mode.

## Controller-Neutral Scoring

Feasible candidates are ranked by a fixed weighted sum of normalized terms:

- route-length error;
- requested route-shape mismatch;
- integrated lateral-offset magnitude;
- integrated squared curvature;
- integrated squared curvature rate;
- inverse minimum lane-clearance margin;
- lateral-acceleration margin at target speed.

Weights, normalization constants, component values, and the stable candidate ID
are saved in trajectory metadata. No score term contains controller identity,
controller parameters, live error, or controller output. Exact ties are broken
by stable candidate ID.

## Runtime Reference Tracking

Each controller lap creates a fresh `ReferenceTracker` over the same immutable
trajectory. The tracker retains only progress and recovery state.

### Continuous projection

The vehicle reference point is projected onto path segments in a bounded window
around the previous segment. Candidate projections are ranked by distance,
heading consistency, and forward progress. At crossings and adjacent opposite
lanes, heading consistency and prior arc length disambiguate geometrically close
segments.

Normal driving keeps progress monotonic. A small configured rollback is allowed
only during low-speed or collision recovery. A larger apparent jump is rejected
and logged instead of being accepted as route progress.

Version one searches from `previous_s - 2 m` through
`previous_s + max(120 m, 3 * speed_mps)` and rejects a one-cycle forward jump
larger than `max(5 m, 3 * speed_mps * control_dt)` unless every skipped segment
is inside the same unbranched road/lane sequence. Projection candidates whose
path tangent differs from vehicle heading by more than `90 degrees` are
ineligible during forward driving.

Projection produces continuous `s_ref`, world reference position, heading,
signed lateral error, heading error, lane clearance, segment index, projection
distance, and recovery status.

### Arc-length preview

The public target point is interpolated at:

```text
s_target = min(s_ref + lookahead_m, trajectory_length)
```

Controller and speed-planner curvature previews request samples by physical
distance or prediction time. PID, LQR, and MPC may retain different intrinsic
prediction horizons because that is part of each controller design, but every
sample comes from the same trajectory and the same `s_ref`.

### Runtime failure handling

- If projection exceeds the nominal route corridor, the tracker enters
  `off_route` and retries within a bounded expanded window.
- NaN data, no valid segment, or an invalid progress jump causes the last valid
  reference to be held for a configured small number of cycles while the reason
  is logged.
- Sustained failure terminates the lap with a non-success reason.
- Route completion requires both sufficient arc-length progress and proximity
  to the final reference point. A preview target clamped to the path end cannot
  complete the lap by itself.

The expanded recovery search covers at most `previous_s - 10 m` through
`previous_s + 200 m`. The last valid reference may be held for at most `3`
control cycles. Completion requires remaining arc length at most `2 m` and
endpoint distance at most the existing `8 m` route-close distance.

## Experiment Design and Interpretability

The primary evaluation is factorial:

- planner: `legacy`, `frenet`;
- controller: `pid`, `lqr`, `mpc`;
- speed mode: `off`, `adaptive`;
- scenario: the existing ten fixed map/speed/shape/seed stability scenarios;
- perception: the existing fixed perception-proxy reruns.

Strict controller comparisons use
`speed_planner_limit_profile=global`. Controller-specific adaptive speed limits
remain available only in a separately labeled integrated-system experiment.

For each planner/scenario pair, the system saves the trajectory before running
controllers and asserts the same hash for PID, LQR, and MPC. Reports separate:

- planner main effect;
- controller main effect;
- planner/controller interaction;
- controller-neutral route-generation failures.

This structure prevents an easier route from being reported as a better
controller and makes any controller-specific interaction visible rather than
hidden.

## Logging and Observability

Trajectory metadata adds:

- planner mode and version;
- trajectory content hash and source topology candidate ID;
- planning duration and candidate count;
- feasibility rejection counts by reason;
- total and component candidate scores;
- maximum curvature and curvature rate;
- minimum footprint-adjusted lane clearance;
- planned maximum lateral acceleration.

Per-step logging adds:

- `reference_s_m` and `target_s_m`;
- projection segment and projection distance;
- progress delta;
- reference hold, rollback, and recovery state;
- physical arc-length locations used for curvature preview.

Paired experiment summaries add legacy-to-Frenet deltas for route-generation
success, RMS lateral error, P95 steering delta, steering reversals, route
completion, and stability verdict.

## Verification

### Unit verification

- Identical inputs produce identical trajectory samples, deterministic
  metadata, and hash; wall-clock duration is explicitly excluded.
- Vehicle-footprint lane bounds reject center-only false positives.
- Quintic candidates satisfy boundary position, heading, and curvature
  conditions.
- Curvature and curvature-rate limits reject invalid candidates.
- Structured failures report the correct grouped rejection reasons.
- Arc-length interpolation is invariant, within numerical tolerance, to raw
  CARLA waypoint sampling density.
- Projection selects the correct branch on straight, curved, S-curve,
  crossing, hairpin, and adjacent-opposite-lane fixtures.
- Normal progress is monotonic; bounded rollback and sustained recovery failure
  behave exactly as configured.
- Equal physical preview distances return equal targets under different path
  sample spacings.
- End-of-route completion requires progress and endpoint proximity.

### Existing safety gates

Every accepted simulator result must preserve the current stability gates,
including:

- route completion at least `98%`;
- zero collisions and zero lane-boundary violations;
- maximum absolute lateral acceleration at most `6.5 m/s^2` and P95 at most
  `4.0 m/s^2`;
- P95 absolute steering delta at most `0.03`;
- existing lateral-error, steering-reversal, speed-error, pedal-switching, and
  lateral/longitudinal jerk limits.

### Relative optimization gates

Across paired fixed-seed results:

- the Frenet stability-pass count must not be lower than legacy;
- median `rms_e_y` must improve by at least `5%`;
- aggregate P95 steering delta may increase by at most the larger of `2%` or
  `0.0001` absolute steering command;
- aggregate significant steering reversals may increase by at most
  `0.1 events/10 s`;
- route-generation success must not be lower than legacy;
- no controller may acquire a new collision, lane-boundary, route-completion,
  or lateral-acceleration failure.

An aggregate improvement cannot compensate for a new hard-safety failure. If
the relative gates fail, `legacy` remains the default and the report identifies
the failing planner, controller, scenario, and metric.

## Compatibility and Rollout

- Add an explicit planner-mode argument with `legacy` retained throughout
  implementation and evaluation.
- Do not change existing controller public constructors or factory names.
- Preserve existing CSV columns and append new columns so current analysis
  scripts remain readable.
- Freeze the validated Frenet trajectory artifact before controller execution;
  a missing or mismatched hash aborts strict comparison.
- Make `frenet` the default only after the full unit suite and paired simulator
  gates pass. Until then, it is an opt-in experimental mode.

## Risks and Mitigations

- **Junction corridor ambiguity:** use source topology identifiers and
  conservative corridor widths; reject smoothing that bridges unrelated roads.
- **Search growth:** cap topology beam width, polynomial grids, candidate
  counts, and planning time; record all cap hits.
- **Over-smoothing route character:** retain explicit shape-coverage constraints
  and compare route labels and scene coverage before accepting a candidate.
- **Controller-comparison confounding:** freeze and hash trajectories, use the
  global speed-limit profile, and report integrated-system runs separately.
- **Dirty existing worktree:** implementation must preserve unrelated user
  changes and stage only files belonging to approved tasks.
