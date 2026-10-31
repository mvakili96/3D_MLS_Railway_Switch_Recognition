# Observation selection and paired-line adjustment

This increment ports `DetermineObservation`, `AdjustTwoLines`,
`AdjustTwoLinesWithObservation`, and the support-to-observation calculation inside
`Trackingrailway`. The current unpacked C++ source remains authoritative. No
adaptive tracking loop or switch recognition is activated by these components.
Kalman recurrence and full initialization are now available in the
[following increment](PYTHON_KALMAN_INITIALIZATION.md).

## Runnable example

```bash
.venv/bin/python -m railway_mls observation-demo
.venv/bin/python -m unittest discover -s tests -v
```

The deterministic example enters at the head-candidate stage, with 540 points in
two oblique neighborhoods. Each contains three noisy rails of 100, 90 and 80
points. It runs MDL separately in both neighborhoods, selects main observations,
fits each with the 500-attempt RANSAC, forms observation vectors, and demonstrates
the weighted gauge routine on those lines. Full tracking will apply that routine
after a Kalman update; this component example does not stand in for that update.

With seed 1 it selects three candidates per neighborhood and 100 observations per
rail, returns finite state vectors and adjusted endpoints, and consumes 3,380 raw
MSVC random draws. The ignored middle candidate in an outer three-line selection
is intentional source behavior. No database or real data is required.

## Source mapping and API

Paths below are relative to `ADE code/PCLProject/PCLProject/PCLProject/`.

| Source | Python | Contract |
| --- | --- | --- |
| `RailwayExtraction.cpp:1302–1793` | `determine_observation` | Main and side point vectors, directional choices, source target gauge + 0.02 |
| `RailwayExtraction.cpp:2130–2188` | `adjust_two_lines` | Unweighted paired adjustment; endpoint ordering by mode |
| `RailwayExtraction.cpp:2210–2330` | `adjust_two_lines_with_observation` | Cross-support weighted distance; endpoint ordering by references |
| `RailwayExtraction.cpp:726–757` | `observation_from_support` | Sequential mean XY and descending 15%-index Z |
| `LIDARPT.h:557–578,694–702` | Line/reference ordering and gauge placement helpers | Strict XY reference comparison, XYZ extension and cached parameters |

All four functions and their result classes are exported from `railway_mls`.
`determine_observation(lines1, lines2, guide1, guide2, *, track_mode=0,
stack_size=0, line_mode=1, config=None, cloud=None)` consumes one to three
initialized candidates per window. Their `Line3D.members` must refer to one owning
cloud. If every membership is absent, supply `cloud` explicitly for empty outputs.
Missing memberships represent empty native `MemPts`, not inferred support.

`ObservationSelection` reports the two observation selections, two left selections,
two right selections, chosen original indices/lines, ranked original indices,
nearest ranks, target distance and discarded local-adjustment diagnostics.
Point order and repeated IDs are preserved. It does not mutate labels or input
line geometry. A one/one selection returns immediately, without sorting or local
adjustment, exactly as in the source.

Both adjustment functions **mutate their two input lines**, preserve memberships
and labels, and return `GaugeAdjustment` diagnostics: common angle, projected gap,
scale, support distances, normalized errors, weighted distance and status. Defaults
use `config.track_gauge` and `config.track_window_length`. Pass `target_distance`
to reproduce the caller's particular target, including the selector's +0.02 bias.
Optional `modes=(m1,m2)` supplies source direction modes explicitly; otherwise they
are derived from incoming endpoints.

`adjust_two_lines_with_observation` requires two support selections and
`references=(ref1,ref2)`. Its `line_mode` argument is intentionally unused, as in
the native function. The nearest endpoint to each reference in XY becomes begin;
equal distances retain the existing order.

`observation_from_support(points)` returns a `SupportObservation` containing the
state point, full mean, and a descending-height support selection. It computes
the mean in **incoming reference order**, before sorting, and uses
`support[int(N*0.15)].z`. It does not use the fitter's separate 5% elevation rule.
Empty support returns `point=None`, `mean=None`, `status="no_support"`; the native
tracker breaks before constructing an observation in that case. Equal-height
support uses the port's documented stable sorting policy.

## Preserved selection rules

Candidates are ordered by signed distance of their **stored centers** to each
guide. A positive XY cross product negates the distance. The source sorts these
values, then rescans the original list from its beginning for each value. Equal
signed values therefore repeat the **first matching candidate**, potentially
omitting another candidate entirely. Python preserves that lookup rather than
deduplicating or returning a conventional sorted permutation.

The first absolute-distance minimum determines the nearest rank. Selection then
uses source direction/count branches. In forward tracking, two/three-line anchor
choices switch from nearest to the last signed rank at stack size **2**. In reverse
tracking, a two-line anchor switches to rank zero only after stack size **2**;
a three-line reverse anchor always uses the nearest rank. One/three and three/one
branches also differ by direction, as the source specifies.

When choosing a rail against an anchored other rail, the error is
`abs((gauge + .02) - distance(candidate.center, anchor))`. Two candidates use a
strict comparison and select the second signed rank on a tie. Three candidates
require a unique strict minimum. Side vectors contain only the extreme ranks:
an outer main choice skips the middle candidate; a middle choice retains both
extremes. Commented point-label assignments remain inactive.

The final source `AdjustTwoLines` call adjusts **local copies**, whose geometry is
not returned. Python exposes that calculation as `local_adjustment` while keeping
the selected candidates and returned point vectors unchanged. Its numerical status
does not invalidate otherwise defined point vectors.

## Stored geometry and gauge arithmetic

`Line3D` now supports an explicitly stored center. `update_center()` corresponds
to native `GetCenterPt()`; assigning `.center` sets the stored value. Once cached,
endpoint edits do not implicitly update it. A newly constructed line retains the
previous midpoint convenience until a center is explicitly cached. Fitted lines
now cache their center at the same point as the source fit. Endpoint/reference
ordering leaves stored angle, coefficients and center unchanged.

The two gauge functions use different source angle-wrap rules. Both refresh
centers from endpoints, build common-angle coefficients, project the first center
onto the second line, extend the transversal, place flat endpoint pairs of global
window length, and leave the resulting stored angle/coefficients intact. They do
not recompute those fields from final endpoints. Existing 0/90-degree coefficient
quirks remain present, including coefficients that do not describe the final
endpoints. Transversal extension scales XYZ even though distances use XY.

The basic scale is `target / projected_gap`. The weighted routine instead gathers
distances from support 1 to line 2, followed by support 2 to line 1. For each
distance `d`, compute `e=abs(target-d)` and `z=(e-min(e))/(max(e)-min(e))`.
If `z == 0`, contribution and weight are `d` and `1`; otherwise they are `d/z`
and `1/z`. Sequential totals produce `weighted_distance=sum(contribution)/sum(weight)`.
The scale is **target / weighted_distance**, not target / projected_gap. The
adjusted geometric gap consequently need not equal the target.

No epsilon, ordinary-mean fallback, least-squares replacement or post-adjustment
normalization is introduced.

## Undefined and nonfinite cases

- A tied minimum among three matching candidates leaves a source local line
  uninitialized before its discarded gauge adjustment. Python retains the defined
  observation/side vectors, leaves the unresolved line unset, skips that unsafe
  adjustment and reports `unresolved_gauge_tie`.
- Empty observation memberships report `empty_observation_members`.
- Unsupported candidate counts, negative/noninteger stack sizes, mixed ownership,
  or nonfinite signed guide distances are rejected at the Python boundary. Native
  unsupported/uninitialized sorting paths are not assigned invented geometry.
- A zero basic gap retains IEEE infinity/NaN and reports `nonfinite_adjustment`.
- Uniform weighted errors produce `0/0` normalization; empty support produces
  `0/0` weighted distance. Both retain NaNs and report
  `nonfinite_weighted_distance`. Other nonfinite placements are also explicit.

These policies expose native arithmetic and undefined access separately. A usable
selection or finite synthetic demo does not establish end-to-end tracking or
reproduction of the paper's evaluation results.

## Verification and next increment

At completion of this increment, the suite passed **117 test methods**, including **36** comparing against
compiled original C++ extracts. New comparisons exercise all nine candidate-count
pairs, both travel directions, stack boundaries, reversed/oblique/vertical guides,
large world coordinates, stored centers, duplicate signed distances, two-line ties,
all 16 direction-mode pairs for both gauge functions, angle wrapping, cached output
state, memberships, elevation scaling and native nonfinite results.

For a three-line minimum tie, the harness compiles the original selection prefix
and stops before the subsequent uninitialized local-line read. That checks the
defined returned vectors without claiming the full undefined native execution has
a portable result. Resolved cases execute the complete original function. Gauge
diagnostic assignments only copy existing intermediate values; source formulas
and branches are unchanged. The original application and its source files remain
unchanged and the Windows application is not built.

Kalman prediction/update and complete seed initialization are now available in the
[following increment](PYTHON_KALMAN_INITIALIZATION.md). Next implement the sequential
tracking loop, adaptive windows and stopping/state transitions.
Keep the weighted post-Kalman gauge call in that source position when integrating.
