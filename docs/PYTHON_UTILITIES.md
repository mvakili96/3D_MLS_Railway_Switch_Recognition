# Standalone source utilities

This increment completes eight utility contracts identified by the source audit.
The original C++ files, native baseline hash, thresholds and inactive scheduling
remain unchanged. Python functions return values in place of native output
references; returned vectors replace outputs rather than append to caller data.
Native output-buffer aliasing is not exposed by this API or claimed by the tests.

## Source mapping

Locations are in `ADE code/PCLProject/PCLProject/PCLProject/RailwayExtraction.cpp`.

| Native definition | Python entry point | Retained behavior |
| --- | --- | --- |
| `ComputeGMMParaAndCluster`, line 1172 | `gmm.classify_segment` | Both three-component fits precede both clustering calls; last component supplies heads, empty/partial outputs remain visible, no label mutations or draws. |
| `DeleteSegPntsCmpWithPreHgt`, line 1208 | `heads.filter_segment_by_previous_heights` | Both upper erasures precede both lower erasures; strict comparisons retain equality, order and duplicate references. Only main memberships change. |
| `GetMinMaxXY`, line 3574 | `geometry.min_max_xy` | Native output order, empty sentinels, bounds clamped by sentinel initialization and first-equal zero signs. Fitting delegates to this helper. |
| `ReCount`, line 3585 | `geometry.recount` | Source int32 branches and truncated division; negative current adds total once, even when the result remains negative. |
| `ComputeInnerAngle`, line 3641 | `geometry.inner_angle` | Normalize XY arms separately, clamp the dot product and return degrees; ignore Z and retain IEEE degeneracy. |
| `ComputeIntersectionPoint3D`, line 3692 | `geometry.intersection_point_3d` | Literal coefficient formula, `A2 == 1` Y branch and Z=0; parallel/coincident cases retain NaN/Inf. Inactive helper only. |
| `DistanceBetweenPointAndLine3D`, line 3860 | `geometry.distance_to_line_3d` | Recomputed endpoint midpoint and explicitly supplied stored unit vector; stale/nonunit vectors are used unchanged. |
| `GetLnsFromVtx`, line 4296 | `geometry.lines_from_vertices` | Ordered cyclic edges, zero endpoint tags, empty membership and absent native caches. Initialization/tracking use the same helper. |

These names and `BoundaryLine` are exported from `railway_mls`. The four unused
helpers are classified `unused_helper_ported`. The intersection is
`inactive_ported`; `DetectLoopCross` and `ValidateLoopCross` remain
`inactive_pending`. No standalone filtering or loop/cross action is inserted
into tracking. Initialization reuses paired classification for its existing GMM
action, with the same results and random consumption.

## Paired GMM and height erasure

```python
from railway_mls import classify_segment, filter_segment_by_previous_heights

first, second = classify_segment(segment)
parameters1 = first.fit.parameters
heads1 = first.heads

remaining1, remaining2 = filter_segment_by_previous_heights(
    segment,
    track_heights=(previous_rail1_z, previous_rail2_z),
    ground_heights=(previous_ground1_z, previous_ground2_z),
    track_threshold=upper_allowance,
    ground_threshold=lower_allowance,
)
```

Both windows need assigned memberships, including explicit empty selections.
`classify_segment` returns two `WindowClassification` results with fitted
parameters, iteration/stop diagnostics, all clusters and shared point references.
It changes no memberships, labels, intensity, geometry or random state.
Existing parameter/head destinations are represented by replacing caller values
with the returned vectors, not by exposing mutable aliased native output buffers.

Empty native GMM fits have three NaN parameter triples after 101 iterations; their
three empty clusters are defined. The source wrapper still selects the last
cluster. Nonempty degeneracy and early partial clustering are preserved. The
general `fit_height_gmm` and single-window `classify_window` APIs retain their
nonempty-input requirement. Initialization and paired classification now share
the same empty-fit handling.

Height erasure deletes `z > previous_track_height + track_threshold` for rail 1
and then rail 2, followed by `z < previous_ground_height - ground_threshold` for
both rails. It does not use the active head extractor's fixed 10 cm/25 cm band.
NaN comparison bounds erase nothing in the corresponding pass; infinity and
negative thresholds are retained. In particular, using negated `>`/`<` masks
preserves NaN behavior that inclusive keep comparisons would lose.

Only `track1.members` and `track2.members` are replaced. Side memberships, fitted
lines, centers, window poses, cloud labels and original selections remain intact.
The API validates both assigned memberships and both height-pair lengths before
mutation. Duplicate IDs stay in their original order, matching vector erasure.

## Scalar geometry and integer cycling

`min_max_xy(points)` returns `(min_x, max_x, min_y, max_y)`. Empty selections return
`(1e100, -1e100, 1e100, -1e100)`. Values beyond these sentinels follow the native
initialization rather than ordinary mathematical bounds. Equal finite values
retain their first occurrence, including the sign of zero. Owned PointCloud
coordinates remain finite under the existing input contract.

`recount(total, current)` follows the native branches rather than Python modulo:

```python
recount(4, 5)    # 1
recount(4, -1)   # 3
recount(4, -9)   # -5: one addition, not repeated wrapping
recount(0, -1)   # -1: this branch performs no division
```

Arguments must fit signed int32. A zero divisor reached by the first branch raises
`ZeroDivisionError`; an overflowing native result raises `OverflowError`. Out-of-
range arguments raise `OverflowError` at the API boundary. These inputs have no
portable native result and are never executed by the native test oracle. Negative
totals are not rejected where source arithmetic is defined.

`inner_angle(start, middle, end)` uses separately normalized XY arms and returns
0–180 degrees for ordinary finite inputs. Coincident arms retain NaN, unlike the
source's separate folded-angle helper, which explicitly substitutes zero. Huge
or tiny arms retain source floating-point overflow/underflow effects. Z and point
tags are ignored; no alternate bearing formula is substituted.

`intersection_point_3d(a1, b1, c1, a2, b2, c2)` keeps the source's literal algebra:
`x=(b1*c2-b2*c1)/(a1*b2-b1*a2)`, followed by `y=a2*x+c2` unless `a2==1`, in which
case `y=a1*x+c1`. It is not a corrected general line solver. Z and the new point
tag are zero. Parallel/coincident cases preserve IEEE NaN/Inf instead of producing
a fallback intersection or changing the inactive loop/cross workflow.

## Endpoint-only boundaries and stored direction

`lines_from_vertices(vertices)` returns a new tuple of `BoundaryLine` objects.
Empty input produces no edges; one vertex produces a degenerate closing edge;
repeated/closing vertices are not removed. Source order is retained.

Native `POINT3D` assignment copies XYZ but leaves destination tags unchanged.
The newly created native edge endpoints therefore have tag 0 even when input
vertices are marked. Python now matches that behavior. The source creates no
coefficients, angle, center, unit-vector XYZ or D3D cache here. `BoundaryLine`
records these as `None`; `members=()` reflects the native initialized empty vector.
Its frozen values do not invent a geometry cache. Initialization and accepted
tracking boundaries use this representation in both normal and turnout lists.

Callers that intentionally compute derived geometry can construct
`Line3D(boundary.begin, boundary.end)`. This is an explicit computation, separate
from the endpoint-only native action. The native rendering path reads endpoints
only; existing trajectory results and endpoint comparisons are unchanged.

The 3D distance function requires an explicit stored vector:

```python
from railway_mls import distance_to_line_3d

distance = distance_to_line_3d(point, line, unit_vector=stored_native_unit_vector)
```

The source never refreshes or normalizes `m_UnitVec` in this helper. The Python API
therefore requires it instead of assuming `line.unit_vector` represents a valid
source cache. A supplied `None` raises `ValueError`; an explicitly supplied stale,
zero, nonunit or nonfinite vector goes through the literal formula. The stored
center is ignored and the endpoint midpoint is recomputed. A derived direction
should only be supplied when reproducing a source action that initialized it.

## Verification and remaining scope

```bash
python -m railway_mls utilities-demo
python -m railway_mls source-audit --repo .
python -m unittest tests.test_source_utilities -v
```

The demo covers all eight helpers, uses no random draws and reports absent caches
separately from NaN through the existing strict-JSON annotations. Tests compile
the actual source bodies with the established point/container shims. GMM iteration
diagnostics copy the existing counter before its output assignments; boundary
comparisons print only initialized endpoints/tags and membership counts.

Fifteen new native-reference test methods cover replacement GMM outputs and
duplicates, empty/degenerate fits, erasure equality/IEEE cases, bounds sentinels
and signed zero, defined negative/int32 cycling, marked polygon edges, angle
degeneracy/clamping, stored-vector distance and both literal intersection branches.
Additional checks cover explicit invalid-source guards, shared helper use,
non-mutated state, coverage updates and CLI output.

Four complete pipeline summaries were captured before the change. Afterward,
straight, turnout, component and missing-reference scenarios match the earlier
reported geometry/tags, statuses/findings, random counts/state, input fingerprints,
class counts and grade/GWAD results; only the coverage dispositions differ.
Boundary cache representation is intentionally more faithful to the source.

The scoped audit now has no `unused_helper_pending` or `ported_reached_behavior`
definitions. Legacy I/O/conversion/export, inactive loop/cross workflows and
header/call-path review remain explicit scope gaps. Whole-application comparison,
real-data evaluation and MLS-scale profiling remain unverified. All limitation
IDs remain open, including the utility policies recorded as LIM-033.
