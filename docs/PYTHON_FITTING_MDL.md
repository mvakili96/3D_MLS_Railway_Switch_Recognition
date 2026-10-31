# Line fitting, grouping and MDL

This increment adds the remaining fitting variants, source point growing and
greedy one/two/three-line MDL selection. It preserves the active code's objective,
thresholds, ordering and shared-label effects. Observation selection and paired-line
adjustment are implemented in the [following increment](PYTHON_OBSERVATION_GAUGE.md).
The Kalman tracker and switch recognition remain future increments.

## Run the synthetic example

From the repository root, with the editable installation already available:

```bash
.venv/bin/python -m railway_mls mdl-demo
.venv/bin/python -m railway_mls mdl-demo --weight 0.05
.venv/bin/python -m railway_mls mdl-demo --weight 0
.venv/bin/python -m unittest discover -s tests -v
```

The example creates 270 oblique rail-head candidates on three parallel lines with
100, 90 and 80 points. It starts at the local candidate stage, without claiming
to perform GMM classification or end-to-end tracing. All three candidate models
are initialized, so the native comparisons avoid missing-candidate reads.

With seed 1 and the default MDL weight 0.4, the example selects three lines with
membership counts `[100, 90, 80]`. Changing only the weight to 0.05 selects two;
weight 0 selects one. All three runs consume 690 raw draws and mark 170 visited
points with segment ID 0. The first line is not grouped, so its 100 IDs stay -1.
The JSON reports models, scores, source flags, selection and native-comparison scope.

The angle is 45 degrees to exercise a defined fixed-angle path. The source's
unusual 0/90-degree coefficient formulas are still present and tested separately.

## API

```python
from railway_mls import ExtractionConfig, MsvcRand, segment_track_points_mdl
from railway_mls.synthetic import parallel_head_cloud

cloud = parallel_head_cloud()
result = segment_track_points_mdl(
    cloud.select(), mode=1, angle=45,
    config=ExtractionConfig(), rng=MsvcRand(seed=1),
)
if result.success:
    selected_lines = result.lines
    scores = [hypothesis.score for hypothesis in result.hypotheses]
    native_scope_checked = result.reference_comparable
```

Use one owned RNG across stages to preserve draw consumption. `MDLResult` exposes
all candidate lines, source eligibility flags, raw/scaled closeness, complexity,
scores, fit attempts and grouping results. `Line3D.members` holds the point
references assigned by the source's `MemPts` operations.

Additional entry points:

- `fit_line_ransac(points, threshold=0.06, rng=...)`: the 500-attempt overload with
  support and RMS XY uncertainty. Its `LineFitResult` includes `sigma`.
- `fit_fixed_angle(points, angle, threshold=0.06, rng=..., previous=None)`: N
  attempts for N input references. `previous` supplies initialized line/support
  outputs for source refits; no new winner retains those outputs.
- `fitting.fit_loop_cross_line(...)`: the dormant loop/cross fitting variant,
  with its contrast, minimum support and length gates. It is available separately
  and tested; it is not activated in MDL or a loop/cross-recognition workflow.
- `group_points(points, threshold=0.15)`: selected component, all visited
  components, retained components and the source's stopping-limit diagnostic.
- `mdl.compute_closeness(points, lines)` and `mdl.choose_mdl_model(scores)`:
  source scoring and strict selection rules, available independently.

`LineFitResult.success` reflects a successful source fit or initialized-output
reuse. A too-short loop/cross result can still expose its initialized line while
reporting failure. A fresh no-model result never fabricates endpoint geometry.

## Source mapping

Source paths are relative to `ADE code/PCLProject/PCLProject/PCLProject/`.

| Source | Python | Preserved behavior |
| --- | --- | --- |
| `RailwayExtraction.cpp:2942–3043` | `fitting.fit_line_ransac` | 500 attempts, first support winner, all-input bounds, 5% height and RMS XY sigma |
| `RailwayExtraction.cpp:3060–3141` | Existing `fit_rail_head_line` | Its 100 attempts and 15% height remain unchanged |
| `RailwayExtraction.cpp:3158–3208` | `fitting.fit_fixed_angle` | Fixed orientation coefficients, N draws, support bounds, 5% height and output reuse |
| `RailwayExtraction.cpp:3227–3308` | `fitting.fit_loop_cross_line` | Dormant standalone variant with source gates |
| `LIDARPT.h:612–692` | Shared endpoint helper in `fitting` | Signed slope branch and endpoint formulas, followed by recomputation |
| `RailwayExtraction.cpp:3501–3573` | `grouping.group_points` | Seed order, recursive DFS order, shared ID mutation, retention and selection |
| `RailwayExtraction.cpp:3574–3583` | Fitting bounds helper | Source min/max sentinel initialization |
| `RailwayExtraction.cpp:1807–1960` | `mdl.segment_track_points_mdl` | Greedy extraction, grouping/refits, flags, penalties and selection |
| `RailwayExtraction.cpp:1976–1990` | `mdl.compute_closeness` | Minimum absolute XY residuals, not squared distances |

## Fitting rules

Both free-angle RANSAC overloads consume two raw C-rand values per attempt, reduce
them modulo input size, and skip equal sample indices without drawing replacements.
Support uses strict XY distance `< threshold`; equal support counts retain the
first winning model. No least-squares refinement is activated.

The 500-attempt overload sets a flat endpoint height from index `int(N*0.05)` of
descending support heights and extends to the bounds of all input points. Its
sigma is `sqrt(sum(distance_xy**2) / N)` over the returned support, using the
recomputed final line. It is a standard deviation, without a degrees-of-freedom
adjustment. The older 100-attempt overload keeps its separate 15% height rule.

Fixed-angle fitting consumes N raw draws, uses support-only bounds and a 5% height.
The source clears support/overwrites the line only when a new candidate wins.
Passing `previous` retains initialized outputs if no new winner is found; the
source can therefore return true even when the new input supplies no support.

The loop/cross variant also requires `inner_count*1.4 >= outer_count`, where the
outer threshold is twice the inner threshold. Its minimum support is
`max(20 // skip_rate, 15)`, and its final XY length must be at least 0.5. Standard
fixed-angle fitting does not impose those gates. Skip rates must be positive
integers at the Python boundary; native zero division is not emulated.

## Grouping and point identity

`SegmentPoints` resets IDs only for its input references. Each new seed is the
first still-unvisited reference in input order. Growth is XY-only with strict
distance `< threshold`. The Python implementation uses iterative scan frames
to produce the recursive depth-first order without Python's recursion-depth limit.
It does not introduce a different adjacency threshold or reorder neighbors.

The source never increments `SegID`: every visited row receives **0**, including
points from components later discarded. Distinct coincident rows remain distinct
point identities; repeated references to the same row are visited once. Class
codes and non-input rows are unchanged.

Keep the first component unconditionally. Later components are retained only if
their size is **greater than 100**, independently of `skip_rate` or the MDL support
threshold. The selected component is the largest retained one. Consequently a
later component of size 100 can be discarded in favor of a first component of size
2. Growth stops after the retained count becomes greater than 1000 (1001 retained
components); unvisited input rows then remain -1.

As with equal-height sorting, native `std::sort` leaves equal-size component order
unspecified. Python uses stable ties. Native-order claims cover distinct-size
fixtures; stable tie selection is explicit. Grouping still has quadratic scan
cost in the worst case; performance on real local windows remains to be measured.

## MDL rules

The active algorithm uses fixed-angle fits. It greedily extracts the first model,
removes support by **point identity**, and tries second and third models only if
the remaining reference count is strictly greater than `50 // skip_rate`.
Coordinate equality is not pointer equality; original order and remaining duplicate
references are preserved.

The first support is not grouped. Later raw support is grown using the source's
retention rules. A chosen component must also have strictly more than
`50 // skip_rate` references before refitting. The source stores the grown component
as `line.MemPts`, even when the refit returns a different support set. Those
memberships, rather than coordinate-derived replacements, are retained in Python.
The second and third line flags are cleared for lengths below 0.2. The first has
no corresponding length gate.

For each hypothesis, closeness is the sequential sum of each point's minimum
absolute XY line distance. The source scales it by `skip_rate / (2*log(2))`, and
computes complexity with `log10(2**line_count)/log10(2)`. The score is
`weight*scaled_closeness + (1-weight)*complexity`.

Invalid second/third flags assign a closeness penalty of `1e100` before scaling.
The third closeness is recomputed after the false-second assignment, so a valid
third candidate can overwrite that penalty and include an invalid-but-initialized
second line. This can select three lines even when two candidate geometries are
redundant. The implementation preserves this source rule.

Selection uses strict comparisons. If neither model 1 nor model 2 is strictly
smaller than both other scores, the source selects model 3. In particular,
`scores=(1, 1, 100)` selects **3**, not either tied minimum. NaN comparisons can
reach the same fallback. There is no extra valid-model tie breaker or MDL label
assignment from commented-out code.

## Missing candidates and native undefined reads

The C++ routine measures and scores `ln2`/`ln3` even when no fit initialized them.
Later flag checks often replace those scores with `1e100`, but the earlier native
reads still have no portable result. Python does not allocate dummy zero lines or
read arbitrary values to imitate this.

For a hypothesis the source explicitly invalidates, Python applies its assigned
`1e100` penalty directly. Unavailable raw closeness is `None`. This gives a usable,
initialized one/two-line selection when explicit scores select one, while exposing
the difference through `status="source_uninitialized_intermediates"`,
`uninitialized_candidates`, and `reference_comparable=False`.

If score fall-through selects a missing line, the result is
`undefined_source_selected_line`, with the requested `selected_count` preserved
but an empty `lines` tuple. Failed attempted fits similarly return
`undefined_source_fit` without manufacturing a model. Grouping label updates made
before a later failure remain visible.

`MDLResult.success` means an initialized selection is available; it is not a
claim that every native intermediate access was defined. `reference_comparable`
is true only for the fully initialized `ok` scope. Future tracking should retain
these diagnostics when consuming usable selections. This is an explicit handling
of undefined native intermediates, not a correction based on the paper.

## Verification and next stage

At completion of this increment, the suite passed **94 test methods**, including **27** methods comparing
against compiled extracts of the original C++. New comparisons cover:

- 500-attempt models, support identities, sigma and random consumption;
- fixed-angle/loop-crossover bounds, gates, heights and initialized-output reuse;
- DFS order, pointer duplicates, thresholds, retention and shared label mutation;
- fully initialized MDL scores, flags, memberships and one/two/three selections;
- orientation modes, skip scaling, duplicate references and the invalid-second /
  valid-third source path.

The harness captures existing MDL local diagnostics without altering score math
or branches. Native tests do not execute the missing-candidate undefined cases;
Python's documented policy for those cases is checked separately. The long-chain
test also verifies iterative growing beyond Python's usual recursion depth.

Observation selection and paired-line/gauge adjustment are now available in the
[following increment](PYTHON_OBSERVATION_GAUGE.md), with Kalman initialization and
sequential tracking still pending. No full rail-path or
switch-recognition result is claimed by this local fitting stage.
