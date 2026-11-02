# Standalone loop/cross components

Implemented on `feat/loop-cross-components`. The unpacked C++ remains the
baseline. Detection and validation are explicit component calls: the source's
commented automatic calls remain inactive, and no engine stage is added.

| Source | Python |
| --- | --- |
| `RailwayExtraction.cpp:2404`, `DetectLoopCross` | `loop_cross.detect_loop_cross` |
| `RailwayExtraction.cpp:2342`, `ValidateLoopCross` | `loop_cross.validate_loop_cross` |
| `LIDARPT.h`, `GetNeighborVtxBND`, `LoopCross` | Existing `TrackSegment.generate_neighbor_regions`, `LoopCross` |
| Fixed-angle fitter and intersection | Existing `fit_loop_cross_line`, `intersection_point_3d` |

The first two definitions now have `inactive_ported` dispositions. Their
initialization/tracking/validation calls at source lines 512, 827 and 1016 remain
commented out. That increment inventoried 201 definitions and 19 named header
types. The later [legacy I/O increment](PYTHON_LEGACY_IO.md) expands to 219.
Coordinate conversion, unused PCL point filters and call-path review remain outstanding;
`complete_reconstruction` is still false.

## Detection API and ordered filtering

```python
from railway_mls import MsvcRand, detect_loop_cross, validate_loop_cross

pool = []
result = detect_loop_cross(
    segment, cloud.select(ordered_ids), ground_height, track_height, reference_line,
    config=config, rng=MsvcRand(1), loop_crosses=pool,
)
validation = validate_loop_cross(pool)
```

Supply explicit initialized geometry and an owning cloud. Share the same random
source across successive actions to preserve process-style consumption. A seed
does not recover an unknown native session. The caller owns the mutable pool;
results contain value snapshots of its records/lines. Point selections retain
shared cloud rows and source order, including duplicates.

Detection returns immediately if either window is marked as turnout. Otherwise
it generates regions on a segment copy using `(window_length + 0.1, 0.50,
window_width)`. It collects **middle-left then middle-right** points with the
source `pnpoly2` predicate. The middle polygon is unused. Coincident-center
nonfinite regions preserve that predicate's IEEE behavior rather than triggering
an invented early rejection.

Height rejection is `z > ground - 0.1`, then `z < ground - 2 * 0.1`.
Equality survives; a NaN ground bound rejects nothing. Both distances from the
stored rail coefficients are evaluated before rejecting either distance `< 0.3`.
Zero normals preserve IEEE NaN/Inf behavior. Track height is unused.

The fitter uses the **stored reference angle + 90**, without angle wrapping,
distance threshold `0.07` and `N` raw random draws. Existing fitter rules retain
the support/count-ratio gate, `max(20 // skip_rate, 15)` minimum, descending-height
support, 5% elevation and 0.5 XY-length rejection. The fitter's positive
`skip_rate` API requirement remains; invalid nonempty calls raise, without a
claim about native invalid-domain behavior. Empty fits return before that read.

## Projection, append gate and undefined source reads

Native detection projects endpoints **before checking the fitter's Boolean**.
A false `line_too_short` result has initialized endpoints and is still projected,
then discarded. Empty, no-model and insufficient-support results can leave the
output uninitialized; Python reports `undefined_source_loop_fit_line`, keeping
the actual prior selections and consumed draws. It never manufactures a line.

Projection uses the literal A2-dependent intersection helper and restores each
endpoint's old Z. Only coefficients and center are refreshed. The fitted angle
remains cached, even when noncanonical rail coefficients produce a projected
line with a different derived angle. No direction cache is invented.

Successful fits form chainage as `segment.distance_from_start -
distance_xy(projected_center, reference_end)`. An empty pool appends immediately.
Otherwise only its **last** record is compared, using both center-to-other-line
distances and the last line's **stored** center. Append requires their mean
strictly `> 1`; equality and NaN suppress it. No sorting, global deduplication,
labels, grid queries or validation postpass are added.

Missing ground, rail coefficients, reference angle/endpoint, segment chainage or
previous-line center produce explicit `undefined_source_*` statuses only when
their reads are reached. They leave the caller pool intact. Initialized
nonfinite projection can still append, as in the source, with status
`nonfinite_loop_cross_geometry`. `reference_comparable` excludes undefined
results and explicitly flagged nonfinite appended geometry; initialized IEEE
duplicate gates retain their source result. This is not a whole-binary
equivalence claim.

The result exposes regions, both region selections, concatenated/height/rail
selections, fitter output, projected geometry, candidate, duplicate distances,
append flag and pool snapshots. Caller segment regions, rail references,
class/segment labels and intensity remain unchanged.

## Validation is an integer phase test

For each index `i`, native validation starts a group with `i`, then visits every
other `j` in original order. Its literal arithmetic is:

```python
distance = abs(chainage[j] - chainage[i])
residual = ((int(distance) * 100) % 2500) / 100
keep = residual < 2 or residual > 23
```

Truncation occurs **before** multiplication. For valid nonnegative differences,
integer residues 0, 1 and 24 are kept; this is not a fractional 25-unit phase
test. The first strictly largest group wins, and the pool is replaced with value
copies in that group's order. It is a star neighborhood, not a clique or connected
component: `[0, 1.5, 3]` selects indices `[1, 0, 2]`. Nothing is sorted.

NaN/Inf/out-of-int32 casts and signed multiplication overflow have explicit
undefined-source statuses and `failed_pair` diagnostics. The original pool is
unchanged because native replacement happens after all comparisons. Empty input
returns `empty`; a singleton returns `ok` without reading its chainage, including
unknown or nonfinite values. Validation uses no random draws or geometric reads.

## Demo and comparison scope

```bash
./.venv/bin/python -m railway_mls loop-cross-demo
./.venv/bin/python -m unittest tests.test_loop_cross -v
```

Our synthetic fixture supplies two initialized rail references and 66 points in
inner bands plus distractors. It is a component fixture added for the port, not
an authors' raw-data generator. Default detection filters 60 candidates, fits and
appends one record with 60 draws. A repeated call consumes another 60 and is
suppressed. Singleton validation retains it. A separate 14-point attempt reports
the undefined failed-fit output after 14 draws.

Tests compile both original bodies and compare ordered memberships/support,
height/distance boundaries, fit flags, projection/cached fields, chainage,
last-record duplicate rules, validation groups/order, labels and raw consumption.
An instrumented prefix stops before uninitialized fit endpoints are read; a full
native detector executes only for initialized fit outputs or turnout returns.
The same supplied raw draws are replayed from the start for the full pass.
Diagnostics copy locals without changing formulas or decisions. Equal-height
sort, floating environment, cache and whole-application limitations still apply.

See **LIM-035** in [the limitations register](LIMITATIONS.md). All earlier entries
remain open for the requested review after full reconstruction. Synthetic/native
component agreement establishes neither real MLS accuracy nor published results.
