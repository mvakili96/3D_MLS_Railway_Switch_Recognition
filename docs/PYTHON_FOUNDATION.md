# Python foundation: behavior specification and implementation map

The current unpacked C++ code is authoritative. This first increment implements
point/state storage, geometry, paired seed windows, and spatial membership. It
does not itself implement local inference, MDL, Kalman tracking, switch recognition,
toes, branch scheduling, grade analysis, or the Windows GUI. The subsequent
[GMM/head extraction increment](PYTHON_GMM_HEADS.md) now adds local classification
and the required rail-head RANSAC fit.

## Run it

The project requires Python 3.10 or newer and NumPy. From the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python -m railway_mls demo
python -m unittest discover -s tests -v
```

The installed console command `railway-mls demo` runs the same example. The demo
creates 1,460 synthetic points in memory, constructs paired rail windows, builds
the grid, and selects 180 points in each window. Ground, web and head heights are
present, but no class prediction or tracing is performed. No input file, database,
viewer, network service, or dataset download is needed to run the demo.

For the initial verification, the existing local NumPy installation was reused
through a `.venv` made with `--system-site-packages`; an editable installation was
performed with `--no-index --no-build-isolation --no-deps`. That check used Python
3.12.4 and NumPy 1.26.4. The normal installation command above resolves declared
dependencies in the selected environment.

## Source map

Source paths in this table are relative to
`ADE code/PCLProject/PCLProject/PCLProject/`.

| Source | Python | Preserved behavior |
| --- | --- | --- |
| `RailwayExtraction.cpp:24–37` | `config.ExtractionConfig` | Ten extraction defaults plus grid size and skip rate; no implicit filtering or thinning |
| `PointXYZDouble.h:9–68` | `points.PointCloud`, `PointSelection` | Float64 XYZ; int32 intensity/class/segment fields; class 0 and segment -1 defaults; distinct rows remain distinct identities |
| `LIDARPT.h:324–425` | `points.Point3D` | Coordinate arithmetic, dot/cross products, normalization, strict absolute `1e-5` IsEqual |
| `LIDARPT.h:441–502`; `RailwayExtraction.cpp:5247` | `geometry.source_angle`, `line_parameters`, `line_parameters_from_angle` | Folded angles and the source's unusual axis-aligned angle/point coefficients |
| `LIDARPT.h:516–555,596–610,717–786` | `geometry.Line3D` | Endpoint ordering, side topology, ordered line-buffer polygons with vertex Z=0 |
| `LIDARPT.h:115–127`; `RailwayExtraction.cpp:3613` | `geometry.points_in_polygon`, `polygon_area` | Strict ray crossing, original boundary asymmetry, XY-only membership and source area formula |
| `RailwayExtraction.cpp:3729–3858,3977` | `geometry.perpendicular_foot`, distances, `direction_mode` | Source projection elevation, point/line distances and quadrant classification |
| `LIDARPT.h:788–805` | `models.GMMParameters` | Parameter container defaults only; no EM implementation |
| `LIDARPT.h:814–943` | `models.TrackWindow` | Window vertices, member references, side polygons, reset semantics and empty member-bound sentinels |
| `LIDARPT.h:945–1054` | `models.TrackSegment` | Paired windows, XY gauge and midpoint; source SwapWnd no-op; neighbor-region polygons deferred |
| `LIDARPT.h:1223–1252` | `models.TrackLayer`, `InitialLocation` | First subset of geometry/layer storage and explicit seed/optional second seed; recognition-specific fields deferred |
| `RailwayExtraction.cpp:351–382` | `models.seed_windows` | Initial placement of paired windows only; second center initially Z=0; no fitting or completed initialization |
| `RailwayExtraction.cpp:154–195,3936–3964` | `spatial.Bounds3D`, `GridIndex` | Cell numbering, insertion rounding, maximum clamping, neighborhood sentinels and traversal order |
| `RailwayExtraction.cpp:1264–1282` | `GridIndex.assign_segment_members` | Gather candidates around the segment midpoint, then assign each window's members in candidate order |

The constructors and Python names provide a usable API rather than a mechanical
renaming of every C++ field. Only the listed operations have been ported. In
particular, `Line3D.update_parameters()` explicitly recomputes coefficients and
angle after endpoint edits, just as the source calls its recomputation methods.
The Python line center and unit vector are calculated on access; later tracking
code must explicitly model any source dependency on cached intermediate values.

## Data and state contracts

- `PointCloud(xyz, intensity=None, class_codes=None, segment_ids=None)` owns an
  `(N, 3)` float64 array in original row order. It accepts an explicit `(0, 3)`
  empty array. A row index is the equivalent of the original point pointer;
  equal XYZ values never merge rows. No coordinate normalization is performed.
- XYZ is copied and exposed read-only so a grid cannot silently become stale.
  Build a new cloud/grid if coordinate data changes. This is an API constraint,
  not an algorithmic point transformation.
- Intensity, class codes and segment IDs are aligned int32 arrays. Optional
  supplied values must be integral and in range. The default values reproduce
  `PointXYZDouble` construction. Setting labels never reorders points.
- `PointSelection` holds one owning cloud and an ordered array of row indices.
  Selections can overlap or contain repeated indices. Call `set_class_code()` or
  `set_segment_id()` to update the owner's labels; `selection.xyz` is a gathered
  array, not another mutable point store.
- `Point3D` is an immutable value, separate from a cloud row's mutable labels.
  Call `is_equal()` for the source's approximate comparison; ordinary dataclass
  equality compares the entire value exactly. Coordinates are required because
  the C++ default leaves them uninitialized. Zero-vector normalization retains
  IEEE NaNs instead of inventing a direction.
- `InitialLocation` takes an explicit point, angle in degrees, and source mode.
  `seed_windows()` creates geometry only. It does not claim successful track
  initialization or populate fitted member lines.
- `TrackWindow.assign_members()` consumes a selection, tests XY only and retains
  its order. Height and intensity do not participate in this geometric operation.
  Generate vertices before assigning members. The source's later classification
  stages will perform their own filtering.
- `GridIndex` accepts optional explicit bounds, because source readers can use
  file-header bounds rather than bounds computed from retained points. Bounds
  must be finite, ordered and contain all points. Empty clouds require explicit
  bounds. Grid size must be finite and positive. These checks report invalid API
  inputs instead of attempting the C++'s invalid indexing/allocation cases.
- The grid stores occupied cells only, while reproducing logical dense cell IDs,
  cell extents, point order and neighbor traversal. No performance claim is made;
  runtime/memory profiling with real clouds remains future work. Python integer
  arithmetic does not emulate native integer overflow; equivalence claims apply
  to inputs where the original arithmetic and indexing are valid.

## Source quirks retained

1. Angles are not ordinary 0–360 bearings. For example northeast and southwest
   lines both have angle 45 degrees; a separate mode records the quadrant.
   Coincident XY endpoints produce angle 0 after the source's NaN fallback.
2. `Get2DLineParameterWithAngleAndPt(0, x, y)` uses X as C, and its 90-degree
   branch uses Y. The Python helper intentionally retains those coefficients.
3. Grid insertion computes `int(min(limit, floor(displacement) / grid_size))`;
   queries use `floor(displacement / grid_size)`. They differ for non-unit sizes.
4. Insertion clamps points at the maximum bound into the last cell, but query
   coordinates at the maximum are not clamped.
5. Neighbor traversal has X as the outer loop and Y as the inner loop. Invalid
   positions remain `-1` in `neighbor_indices()`. Gathering skips those sentinels,
   preserving cell order and then original point order within each cell.
6. Polygon boundaries follow the source's strict ray-crossing predicate rather
   than an all-edges-inclusive rule. Window membership uses XY, regardless of Z.
7. Buffer polygons have Z=0, even when line/window centers have another elevation.
   Paired seed construction initializes the second center's Z to zero as well.
8. `SwapWnd` compares a coordinate to itself and never swaps. `swap_windows()`
   preserves that no-op. Inactive source behavior has not been enabled.
9. `GetSideVtx` buffers its middle polygon by **side width**, not center width.
   `ResetTrackWindow` leaves side polygons/members and pose in place.

No algorithm correction or paper-based substitution is included in this increment.

## Verification

The standard-library test suite covers precision, shared labels, duplicate point
identity, analytic distances, all source modes, boundary membership, reset behavior,
non-unit grid insertion, ordered neighborhoods, explicit bounds and synthetic
integration. Run the command above from the repository root.

When `c++` is available, eight additional test methods compile and execute actual
extracts of the original `LIDARPT.h` and `RailwayExtraction.cpp`. The tests compare
angles, coefficients, endpoint ordering, window/side polygons, seed placement,
grid assignments, neighbor lists, projections and polygon predicates across many
cases. Continuous geometric comparisons use explicit small numerical tolerances;
cell IDs and membership predicates compare exactly.

The harness substitutes plain point/cloud declarations for PCL/Windows types and
adds an explicit allocation-size cast needed by GCC for the source's double-valued
array length. It does not change the tested arithmetic or branches. Compilation
and execution happen in a temporary directory, without modifying original sources.
These checks validate the extracted operations, not the complete Windows binary
or end-to-end railway extraction. They skip if the compiler or original source is
unavailable; the other component tests still run.

Initial verification: **28 test methods passed**, including all eight C++ comparison
methods; the demo selected **180/180** points; the editable install succeeded.

## Subsequent increments

The custom height EM-GMM and rail-head extraction are now implemented in the
[GMM/head extraction increment](PYTHON_GMM_HEADS.md), including its required
100-attempt RANSAC line fit. The [fitting/MDL increment](PYTHON_FITTING_MDL.md) now
adds the other fitting variants, source grouping and MDL selection. The
[observation/gauge increment](PYTHON_OBSERVATION_GAUGE.md) adds observation/side
selection, support state vectors and both gauge routines. The complete suite
currently passes 143 test methods. Full initialization and the filter components
are available in the [Kalman/initialization increment](PYTHON_KALMAN_INITIALIZATION.md).
The next work is sequential tracking. Downstream
recognition models and cached tracking state will be specified with their stage.

Packaging and array interfaces follow the official
[setuptools pyproject guidance](https://setuptools.pypa.io/en/latest/userguide/pyproject_config.html)
and [NumPy array conversion reference](https://numpy.org/doc/stable/reference/generated/numpy.asarray.html).
