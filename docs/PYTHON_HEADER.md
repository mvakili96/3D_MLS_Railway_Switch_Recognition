# Inline source geometry and state coverage

The audit now inventories **201 definitions**: 124 engine/UI definitions and all
**77 uncommented inline definitions** in `LIDARPT.h`. It also maps all **19 named
top-level header types**. The two block-commented rotation functions remain
explicitly excluded. This is scoped lexical coverage, not a general C++ parser
or a numerical coverage percentage. Native files and their baseline hash remain
unchanged. The rules are in `tools/header_coverage.py`; generated dispositions
are packaged in `railway_mls/source_coverage.json`.

## Added and completed behavior

| Source area | Python behavior |
| --- | --- |
| `bIsNan`, `FLOOR_double` | NaN self-inequality; float32 narrowing followed by truncation and positive half-up behavior. The latter is not mathematical floor. Undefined integer conversions raise. |
| `LiDPOINT` | `LegacyLiDPoint` stores default metadata, unknown XYZ until assigned, setters, equality and metadata-resetting arithmetic. Native division reads uninitialized output XYZ; Python raises `undefined_source_lidpoint_division`. |
| `POINT2D` | `Point2D` distinguishes default flag false from the XY constructor's unknown flag. Arithmetic creates default-flag results. Zero division returns a value copy including the flag. Tolerant `IsEqual` and exact `isEqual` are separate. |
| `POINT3D` | Explicit XYZ value API; setters, XYZ-only assignment, sign and normalization. Assignment/sign/normalization retain destination tags; arithmetic creates tag 0. Frozen Python values return replacements. |
| `Line3D` | Independent coefficient/angle refresh, angle/point coefficients, tagged swaps, vertex checks, extension, Z-zeroing 3D bounds, cached-center/radius endpoints and stored-unit refresh. |
| `TrackWindow` | Reset retains an assigned selection's cloud ownership and leaves side state untouched. Bounds preserve sentinels, last-equal zero signs and output point tags. |
| `TrackSegment` | Explicit gauge/stored-center updates, five neighbor polygons and destination-aware `DuplicatePolyPair`. The source no-op swap remains unchanged. |
| State/comparators | LoopCross and remaining layer pools are represented without scheduling inactive work. Higher-Z/larger-group predicates, parameter assignment and existing state types have explicit mappings. |

The generator rejects unclassified header additions. `source-audit` checks
definition IDs/signatures, type inventory, native hashes and Python targets.
Malformed header syntax reports drift rather than invented coverage.

## Point and line state

```python
from railway_mls import Line3D, Point2D, Point3D, source_float_round

source_float_round(-1.8)        # -1: native truncation branch
source_float_round(1.49999999)  # 2: narrows to float32 1.5
Point2D().flag                 # False
Point2D(1, 2).flag             # None: native XY-constructor flag is unknown
Point3D(3, 4, 0, 17).normalized().point_type  # 17

line = Line3D(Point3D(0, 0, 2, 7), Point3D(2, 1, 4, 9))
extended = line.extend_endpoints(2)
line.swap_endpoints()          # XYZ swap; tags stay in slots 7 and 9
line.update_unit_vector()      # explicitly initializes the native unit cache
```

`Line3D(begin, end)` remains an initialized convenience constructor.
`BoundaryLine` represents endpoint-only output with absent caches. Unknown native
default coordinates are not assigned guessed positions. `extend_endpoints`
refreshes the original center and returns an endpoint-only BoundaryLine.
`is_vertex` compares XYZ with strict 1e-5 tolerance; only an end match swaps.
Swaps retain endpoint slot tags without refreshing other cached fields.

`set_ending_points_3d` follows native signed-slope branches and sets both Z values
to zero, except when both coefficients are zero and nothing changes.
`set_ending_points_by_center_and_length` requires an initialized stored center;
begin is placed on the positive angle side and end on the negative side.

`unit_vector` is derived; `stored_unit_vector` is the native mutable cache, retained
until `update_unit_vector` is called. Active head refinement records that refresh
where the source invokes `GetUnitVec`. Segment `center` is likewise a computed
convenience; `stored_center` records literal native center updates. Seed creation,
initialization and tracking record those updates at their source call sites.
Uninitialized D3D remains `None` unless explicitly supplied.

## Bounds, regions and assignment

`TrackWindow.member_bounds(maximum=None, minimum=None)` returns native-order
maximum/minimum points. Explicit output points retain their tags. Empty bounds
remain -1e100/+1e100. Unlike strict-update `GetMinMaxXY`, the header uses
`min(new, old)` / `max(new, old)`: equal zeros retain the **last** input sign.
Values beyond initial sentinels follow source clamping. Input state is unchanged.

`generate_neighbor_regions(length, width, window_width)` replaces the five
middle/middle-left/middle-right/left/right polygons, retaining source width
factors, axis-buffer rules and Z=0. Coincident centers retain nonfinite XY and
report `nonfinite_neighbor_vertices`. The inactive loop/cross caller remains
unscheduled. Region generation does not refresh the stored segment center.

`destination.duplicate_from(source)` copies windows, gauge and stored center,
preserving destination distance and neighbor regions. Cloud rows stay shared;
windows/lines use value assignment. Scalar point assignments retain destination
tags. A long-enough destination vertex vector guarantees native XYZ-only
assignment for the overlapping entries. With a longer source, unknown destination
capacity can change tag transport; Python chooses value transport and reports
`capacity_dependent_tags` / `reference_comparable=False` if alternatives differ.
Comparisons cover defined fields, not absent caches or unknown allocation history.

These policies extend LIM-031. They do not emulate arbitrary native layouts,
dangling pointers or output-buffer aliasing. Grid/PtSegment retain existing
ordered array backends; immutable planes represent coefficient copies. Field and
constructor policies for all 19 types are individually recorded in the manifest.

## Run and validation

```bash
source .venv/bin/activate
python -m railway_mls header-demo
python -m railway_mls source-audit --repo .
python -m unittest tests.test_header -v
```

The demo uses deterministic constructed values and no random draws or external
data. It executes the Python legacy-division guard; the undefined native division
is never executed by the oracle. Sixteen native-reference test methods compile
actual header bodies with the established point/container shims. They cover
rounding, tagged points, legacy metadata, line/cache operations, five neighbor
regions, IEEE cases, bounds and defined segment assignment. Additional tests cover
unknown fields, capacity-sensitive copies, reset ownership, drift and CLI output.
Comparisons never print absent caches or uninitialized constructor flags.

All four integrated scenarios retain prior trajectory/grade/GWAD results,
statuses/findings, RNG counts/state and per-point class/segment label hashes.
Coverage metadata and stack fingerprints change intentionally: signature schema
**2** includes the newly represented cache/neighbor state and is not comparable
to earlier unversioned stack fingerprints. Cloud input hashing is unchanged.

All limitations remain open, including new LIM-034. Inactive loop/cross workflows,
legacy data/conversion/export interfaces, whole-application comparison, real-data
accuracy and MLS-scale profiling remain separate work. Inventory consistency
does not establish native application equivalence or published performance.
