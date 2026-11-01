# Switch-box detection and plane fitting

This increment ports `DetectSwitchBox`, its plane-fitting dependencies and the
active main-track call after toe detection. The unpacked C++ remains authoritative.
An explicit initialized toe fixture makes this stage runnable without the database
or a workaround for [LIM-001](LIMITATIONS.md#lim-001-uninitialized-toe-reference-lines).

## Run and use

```bash
.venv/bin/python -m railway_mls switch-box-demo
.venv/bin/python -m railway_mls switch-box-demo --seed 2
```

The default fixture supplies a toe and two planar point patches with strict-band
distractors: 2,316 input points, 1,156 accepted points per patch, two boxes and
1,200 raw random draws. Exactly 2,312 cloud rows receive class code 5. The result
is a component demonstration with synthetic surfaces, not an empirical switch
benchmark or automatic recovery of the native uninitialized toe references.

```python
from railway_mls import GridIndex, MsvcRand, TrackLayer, detect_switch_boxes
from railway_mls.synthetic import switch_box_scene

scene = switch_box_scene()
layer, pool, random = TrackLayer(), [], MsvcRand(1)
result = detect_switch_boxes(
    scene.toes, GridIndex(scene.cloud, scene.config.grid_size),
    config=scene.config, rng=random, layer=layer, boxes=pool,
)
print(result.status, len(result.new_boxes), random.draw_count)
```

The public API also exposes `SwitchBox`, `SwitchBoxCandidates`, `SwitchBoxBatch`,
`PlaneParameters`, `PlaneEigenFit`, `PlaneFitResult`, `switch_box_candidates`,
`plane_from_three_points`, `plane_from_eigen`, `plane_distances` and
`fit_plane_ransac`. `SwitchBoxDetection` retains region/height memberships, both
fit results, the selected group and the acceptance/rejection reason.

## Source mapping

| Original source | Python | Preserved contract |
| --- | --- | --- |
| `RailwayExtraction.cpp:2494–2599` | `switch_boxes.detect_switch_boxes` | Two-phase candidate construction, ordered lookup, strict height band, two fits/grouping, gates, labels, centroid and appends |
| `LIDARPT.h:1056–1076` | `models.SwitchBox.generate_region` | Four square corners at angle +45/+135/+225/+315 degrees, with size/2/cos(45) radius |
| `RailwayExtraction.cpp:4104–4117`; `LIDARPT.h:122–152` | `geometry.points_in_convex_polygon` and ordered selection | Distinct `pnpoly2` all-cross-product-tag predicate, including the fixed `1e-10` zero test |
| `RailwayExtraction.cpp:3912–3916` | `planes.plane_distances` | Absolute normalized plane distance with native IEEE arithmetic |
| `RailwayExtraction.cpp:4133–4185` | `planes.fit_plane_ransac` | 100 attempts, three draws each, no degenerate-sample skip, refit every attempt, strict first best and in/out retention |
| `RailwayExtraction.cpp:4204–4226` | `planes.plane_from_three_points` | Negative unnormalized cross product and source offset/sign |
| `RailwayExtraction.cpp:4242–4285`; bundled `Jama/jama_eig.h:tred2,tql2` | `planes.plane_from_eigen`, private symmetric solver | Incremental centroid, ordered scatter accumulation and the first source-sorted eigenvector |
| `RailwayExtraction.cpp:1007–1008` | `SequentialTracker.run`, shared `TrackingEngine.switch_boxes` | Main-only postpass after all toe appends, before outer branch scheduling |

## Candidate geometry and membership

The source builds **all** candidates before evaluating any. It skips only toes
whose global `used` is true; type, stored angle/mode, score and center are not
additional filters. Only the right/left rail points are needed.

For each unused toe, the right-to-left span is extended about its midpoint by
**2.53 in XYZ**. Each extended endpoint becomes a region center, in begin/end
order. A line at the span's folded angle runs from center +0.5 along that angle to
center -0.5. Buffer width 1 creates a 1-by-2 rectangle. Its reference elevation is
the mean of the extended span's endpoint Z values. The declared `orthoDist=2` is
unused; it is not substituted for the active factor or radius.

Each center gathers an ordered **3x3** grid neighborhood, retaining the existing
non-unit insertion/query rules. `GetPntsInsidePolygon` calls `pnpoly2`, distinct
from the tracking windows' ray-crossing `pnpoly`. Every edge cross-product gets a
tag -1/0/+1, with zero only when absolute cross product is **below `1e-10`**.
Membership requires all tags to agree. Ordinary rectangle edges/corners are
excluded; a degenerate polygon with all-zero tags qualifies. No conventional
edge-inclusive polygon helper or tolerance scaling is added.

The height filter is strictly **`reference < z < reference + 0.10`**. Both bounds
are excluded. Source grid/point order is preserved through filtering and fitting.

## Plane fitting and output reuse

Three sampled points define coefficients from the negative cross product of
`second-first` and `third-second`; they are not normalized. Distance normalizes
the normal, retaining zero-normal NaNs.

Each 100-attempt fit consumes three raw draws per attempt, including duplicates.
It selects support at distance **<0.008**, refits that support using the bundled
JAMA solver, and reselects from the whole current input at the same threshold.
The first strictly largest refined support wins; equal counts do not replace it.
There is no final additional refit after choosing that winner.

The eigen refit uses the source running centroid `(i*mean + point)/(i+1)`, not a
replacement mean reduction. Scatter uses sequential row accumulation. The Python
3x3 symmetric solver follows the reached Householder/QL branches, signs, strict
eigenvalue ordering and repeated-eigenvalue basis from the bundled header. Native
comparisons include that actual header. A generic eigensolver could pick a
different null-space basis and change the later horizontal-plane decision.

Empty refit support is **defined**: zero scatter and origin centroid produce the
plane `x=0`. Thus repeated/collinear samples cannot simply be skipped. Empty,
singleton and tied-eigenvalue cases have source-specific results.

`ExtractPlaneByRANSAC` returns immediately for fewer than four input references,
**before** clearing output support or changing coefficients. With enough inputs,
it clears support but retains old coefficients if no refined support wins.
`PlaneFitResult` records `parameters_written`, `support_cleared`, counts and best
iteration. Missing coefficients remain `None`; no default horizontal plane is
created. An initialized old plane can remain valid despite a no-op call.

## Constraints, labels and appends

After the first fit, source grouping uses XY distance **0.1**, including its first
component bias, later-component >100 rule, DFS order and shared SegmentID writes.
The second plane fit receives the selected group and the first call's outputs.
It can therefore retain those outputs when the group is smaller than four.

The source first rejects when **`abs(c) < 0.999`**, then when selected group size
is **less than integer `1000 / skip_rate`**. Equality passes. These constants are
independent of the line-fitting and grouping configuration. Positive skip rates
above 1,000 yield a zero size threshold, as in source integer division.

Acceptance labels **the pre-second-fit selected group** with class code 5 and
computes its sequential XYZ centroid. The second fit's support is not substituted
for those labels or the centroid. The final square has size **0.7**, candidate
angle and all corner Z set to centroid Z. The native final `referHgt` is never
assigned; Python represents it as `None`.

Each box is appended by value to the engine pool and current layer. Existing
boxes are retained, positions are not deduplicated, and toe used markers do not
change. Grouping mutations survive rejection. Engine layer reset retains the
global box pool; repeated main passes can append duplicate detections.

## Tracking integration and limitations

Natural main tracking completion now runs recognition, toe detection and switch
boxes in that order on one RNG/cloud. Results expose `switch_box_detection`;
`TrackingResult.reference_comparable` includes its diagnostic scope. Branch calls
report the disabled main-track gate without fitting. Empty-layer returns,
undefined tracking exits and diagnostic limits defer the postpass. A completed
tracker is not processed again merely by reading/rerunning its result.

Sparse candidates can leave native `PlnPara` uninitialized before `fabs(c)`.
Python still performs the preceding defined fit/group/refit calls, then records
`undefined_source_switch_box_plane`. It emits no fabricated box for that candidate
and can inspect later candidates; batch status becomes `partial_undefined_source`
and comparison scope is false. This is an explicit handling of undefined source
execution, not proof of equivalent whole-call native outputs.

Missing toe endpoints abort candidate construction before any candidate is fitted.
Nonfinite candidate geometry stops before unsafe grid conversion. Nonfinite
scatter/eigen arithmetic stops before the native invalid-index/stall path,
retaining prior effects and consumed draws. No regularization is added. API
boundaries require matching grid size and a positive integer skip rate.

The [limitations register](LIMITATIONS.md) retains LIM-001 and adds LIM-025–LIM-027
for these output, heuristic/lifecycle and numerical findings. All remain open for
the review after reconstruction.

## Verification and next stage

The full suite passes **294 test methods**, including **96** using original C++
extracts. This increment adds 35 methods, including 13 native comparison methods.
They exercise coefficients/bases, repeated draws, retained outputs, candidate
geometry, polygon semantics, height endpoints, exact 999/1,000 group thresholds,
tilt/skip-rate gates, identities, labels, centroid/corners and existing appends.
Complete native detection comparisons use initialized plane reads; unknown plane
outputs are never printed by the separate native fitting oracle.

Raw integration checks execute the actual postpass before branch scheduling,
verify engine/layer persistence and repeated appends, and check limit/branch gates.
The new demo works without the database. Original Windows-binary, real-data
accuracy and performance validation remain pending. Next implement grade/curve
analysis and finish the remaining computational inventory and pipeline audit.
