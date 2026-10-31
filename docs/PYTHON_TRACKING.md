# Sequential tracking

This increment implements the computational loop in
`RailwayExtraction.cpp:530–874`, before its visualization callbacks and separate
recognition postpass. It integrates the previously ported components using the
source call order. Templates, recognition, toes, automatic branch scheduling,
bidirectional GUI orchestration and switch-box detection remain subsequent work.
Inactive loop/cross calls remain inactive. No original C++ source is modified.

## Run and use

```bash
.venv/bin/python -m railway_mls tracking-demo
.venv/bin/python -m railway_mls tracking-demo --side-rails --max-steps 3
.venv/bin/python -m unittest discover -s tests -v
```

With seed 1, the default example initializes 6,192 raw noisy ground/web/head points,
accepts eight tracking steps, saves nine segments including initialization, and
reaches `insufficient_window_points`. It consumes 21,400 raw random draws across
initialization and tracking and travels about 7.879 input distance units. It uses
the original defaults. It reports `reference_comparable=False` because the
single-rail MDL path has the previously documented missing native intermediates.
The example uses that explicit port policy; it is not a claim that undefined
native reads have portable results.

The secondary-rail example has all three MDL candidates initialized over its first
three steps and returns `reference_comparable=True`, with a three-segment active
stack. The step limit is an optional diagnostic boundary, not a source termination
rule. Removing it allows the source count/geometry rules to stop the loop.

```python
from railway_mls import GridIndex, MsvcRand, initialize_tracks, track_railway
from railway_mls.synthetic import tracking_scene

scene = tracking_scene()
grid = GridIndex(scene.cloud, scene.config.grid_size)
random = MsvcRand(1)  # One stream across all stages.
initialized = initialize_tracks(scene.seed, grid, config=scene.config, rng=random)
result = track_railway(initialized, grid, config=scene.config, rng=random)
print(result.status, result.accepted_steps, result.distance_from_start)
segments = result.layer.track_segments
```

`track_railway` takes an initialization result whose source return is true and a
grid sharing its point cloud. It accepts `config`, `rng`, `track_direction`,
`main_track`, and `max_steps`. Supply the same configuration and RNG used during
initialization. Defaults on a newly constructed RNG begin a new stream.
`track_direction` changes source observation choices; it **does not reverse seed
geometry**. Reversing geometry and joining the two runs belong to later orchestration.
`main_track=False` resets accumulated distance; recognition is not called for
either flag in this loop-only increment.

For resumable diagnostics, construct
`SequentialTracker(initialized.layer, grid, initialized.initial_heights, ...)`
and call `run(max_steps=...)` repeatedly. Each limit counts additional loop
attempts, stopping earlier on a terminal condition. The object retains its RNG,
heights, active stack and first-iteration state. Creating a new tracker is a new
invocation, not a continuation. Completed stacks can be supplied as a shared list
to preserve the original engine's storage across later branch invocations.

`TrackingResult` exposes the live layer, step diagnostics, completed and active
stacks, cumulative distance, latest posterior heights, terminal status, accepted
count and comparison scope. Each `TrackingStep` records its current filter states,
window attempts, head extractions, models, observation choice, supporting fits,
observation vectors, updates, gauge result and accepted or rejected candidate.
These diagnostics retain point-reference arrays; real-data memory/runtime profiling
is still pending. The new CLI writes nonfinite numbers as JSON null with an
explicit status rather than emitting invalid JSON.

## Source mapping

Paths are relative to `ADE code/PCLProject/PCLProject/PCLProject/`.

| Source | Python | Behavior |
| --- | --- | --- |
| `RailwayExtraction.cpp:1255–1283` | `generate_tracking_segment` | Inherited orientations, predicted centers, shared ordered 5x5 neighborhood |
| `RailwayExtraction.cpp:561–663` | `SequentialTracker` prediction/retries | Identity current covariance; separate count and head retries |
| `RailwayExtraction.cpp:665–724` | Model/observation selection and stacks | Both MDL calls, turnout branch, early predicted snapshots, normal-region flush |
| `RailwayExtraction.cpp:726–822` | Observation/update, gauge and stops | Support vectors, raw posterior heights, endpoint replacement, partial side effects |
| `RailwayExtraction.cpp:833–872` | Layer appends and current-state copying | Vertex heights, boundary destination, accepted segments, DuplicatePolyPair semantics |
| `LIDARPT.h:1031–1036` | Current segment snapshot | Copies windows, gauge and center without replacing current distance-from-start |

## Prediction, retry and inference order

The first mode comes from the first initialized line's endpoints. Later modes
come from the two latest accepted first-rail centers. Each current position uses
the latest accepted window center. Current covariances remain identity: native
assignments from `hatP` to `curP` are commented out. Updated covariance remains
available in diagnostics but is not fed into the next prediction.

The base prediction uses `config.track_window_length`. New windows inherit the
previous stored orientations, not recomputed endpoint angles. Their memberships
come from one shared ordered 5x5 neighborhood about the predicted pair midpoint.
The neighborhood size stays five even when the window grows.

Point shortage uses **`count < min_points`** initially, then tries lengths
`base + k*base` for k=1…5 (2–6 times base). A retry requires **both counts strictly
greater than** the threshold. Equality can therefore pass the initial check and
fail a retry. Exhaustion returns `insufficient_window_points` before extraction.

After head extraction, initial shortage similarly uses `< min_track_points`.
Head retries restart at twice base, even if point retries already used a larger
window. Each prediction uses the same current positions, identity covariances and
old heights. It does not extend from a previous retry's predicted point. Existing
heads are passed back into extraction: append-without-clear behavior, duplicate
point references, sorting and class-3 writes are preserved. Retry acceptance uses
strict `>`; exhaustion returns `insufficient_head_points` with existing label
changes retained.

Both MDL calls occur next. If either returns more than one selected line, the
turnout branch selects main/side observations and runs the 500-attempt fit on each
selected observation. Otherwise it fits all extracted heads in each window.
The support mean and descending 15% height form the Kalman observations. The
fitter's RMS XY sigma is passed directly to the original update.

## Gauge, cached state and stopping

The temporary lines run from current centers to Kalman posterior points.
The next height variables are saved from those **raw posterior Z values**, before
gauge adjustment, and feed subsequent extraction even if the final rail endpoint Z
changes. Weighted gauge adjustment uses current centers as references and the
configured gauge, without the observation selector's separate +0.02 bias.

After adjustment, the source replaces each line's **begin** with the previous
window center and refreshes **only coefficients**. Its angle and cached center
remain those stored by the gauge routine. Python preserves that state, including
cached centers that differ from the endpoint midpoint. New window orientation
uses the stored angle; its center becomes the adjusted line end. New saved windows
return to the global base length regardless of which retry length succeeded.

The source then performs these mutations/checks in order:

1. Append rail end points and their pair midpoint to layer vectors.
2. Add XY distance between old/new pair midpoints to cumulative distance.
3. Update the candidate windows, gauge and distance-from-start.
4. Reject if absolute gauge error is **greater than 0.15** (`gauge_difference`).
5. Reject if first-rail movement is **less than 0.3** (`trajectory_end`).
6. Set polygon Z to each line's old begin Z, append boundary edges, and save the
   accepted segment. Advance current geometry with the source copy semantics.

A rejected candidate can therefore leave one extra entry in each rail vector and
an increased cumulative distance, while no segment or boundaries are appended.
The port does not roll back those source side effects. Stop-check equality does
not reject. Boundary edges go to the normal or turnout list based on each flag.
Point memberships are not reassigned after candidate geometry moves.

## Turnout-stack state

`TrackStack.track_segments` preserves predicted segment snapshots. In a turnout
branch, side tags come from nonempty side point vectors, both main memberships
are cleared and **both** windows are marked turnout even if only one had multiple
models. The stack copy is taken **before** updating with posterior/gauge geometry
and before stop checks. Its center and polygon are the predicted ones; it has no
initialized member line or distance-from-start. Python uses `None` for those
undefined fields instead of inventing final geometry or cumulative distance.

On a later single-model region, a stack is retained only if its length is
**greater than two**; shorter stacks are discarded. The active list is cleared
before posterior updates and geometric stop checks. A terminal active stack is
**not flushed** automatically. Stack segments copy window/line value state while
keeping shared references to original cloud rows, matching native pointer identity.

This storage prepares later template recognition. It does not assign template
types, toe points or branch seeds. The source recognition postpass starts after
the loop, and is not replaced with stub classifications in the Python API.

## Undefined and nonfinite paths

The previous component contracts still apply. A usable single-model MDL selection
can continue with `source_uninitialized_intermediates`; comparison scope stays
false. A missing selected model, undefined head fit, unresolved three-line
observation tie or unusable observation fit terminates with a specific
`undefined_source_*` status. Existing point-label/grouping mutations remain
visible. Those native undefined paths are not run as an oracle.

Nonfinite predictions stop before the source would convert NaN/Inf coordinates
to grid integers. A defined nonfinite gauge result retains the source's current
iteration mutations: NaN comparisons do not trigger the gauge/movement rejection,
so vectors, boundaries and the segment are appended. Python then reports
`nonfinite_tracking_geometry` before a subsequent unsafe grid query. It does not
repair the geometry or substitute a gauge target. These numerical diagnostics are
separate from natural source stopping conditions and user-requested `step_limit`.

## Verification and remaining work

The suite passes **171 test methods**, including **55** using compiled original
C++ extracts. Seven new native comparison methods exercise multiple actual loop
iterations with fully initialized MDL candidates, both observation directions,
single/multiple-model paths, branch distance reset, point and accumulating-head
retries, cached output state, exact memberships/labels, active stacks, boundary
endpoints, trajectory-stop side effects, and curved/graded input. Inputs include
three close candidate rails so those MDL executions avoid uninitialized reads.

The oracle compiles the original loop through `DuplicatePolyPair`, adds only an
accepted-step test limit and diagnostic copies, and excludes print/progress/render
callbacks plus the separate recognition postpass. It calls the real source grid,
GMM, extraction, MDL, observation, fitting, Kalman and gauge routines. It is not a
second reimplementation of the orchestration. Component fixtures additionally
check mixed one/multiple-model branches and nonfinite stop policy. Synthetic
integration checks natural termination, all four geometric modes, gaps, curves,
grade, equality thresholds, duplicate retries, stack transitions and resume state.

Next implement stack signatures and source template recognition, then toe/branch
integration and bidirectional orchestration. Separately invoked source features
remain in the port plan. Real-data accuracy, performance and published-metric
reproduction remain pending; passing synthetic and extract comparisons does not
establish those results or complete Windows-binary equivalence.
