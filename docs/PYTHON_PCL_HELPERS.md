# Standalone PointXYZ helpers

Implemented on `feat/pcl-point-cloud-helpers`. The unpacked C++ and its bundled
PCL 1.8.0 headers remain the baseline. Both helpers are standalone: the original
application has no active call sites, so neither is inserted into readers,
coordinate conversion or tracking. The runtime still needs only NumPy.

| Source | Python |
| --- | --- |
| `LASFileReader.cpp:113`, `DecreaseDensity` | `pcl_helpers.decrease_density` |
| `LASFileReader.cpp:129`, `Transform` | `pcl_helpers.center_point_cloud` |
| PCL `VoxelGrid<PointXYZ>`, fixed default settings | Float32 bounds/key operations, checked integer arithmetic, voxel sorting and float32 centroid accumulation |
| PCL `compute3DCentroid`, affine `transformPointCloud` | Sequential double centroid, double identity/translation expression, float32 output |

This increment completed the former pending helper gaps. The subsequent
[reconstruction audit](RECONSTRUCTION_AUDIT.md) expands the inventory to 265
definitions with no pending definition/file dispositions and completes the
agreed computational scope review. Inventory consistency does not prove numerical
equivalence with the Windows app. All limitation IDs remain open, including
**LIM-038** and the subsequent boundary/review entry **LIM-039**.

## Explicit PointXYZ state and bridges

```python
from railway_mls import PclXYZCloud, center_point_cloud, decrease_density

cloud = PclXYZCloud([[.01, .02, .03], [.03, .04, .05], [.21, .02, .03]],
                    width=3, height=1)
reduced = decrease_density(cloud)
centered = center_point_cloud(cloud)
```

`PclXYZCloud` owns read-only **float32** XYZ, explicit `is_dense`, width/height,
`PclHeader` (uint32 sequence, uint64 stamp, frame ID), float32 sensor origin and
quaternion in XYZW order. PCL default metadata is width/height 0/0, dense true,
origin (0,0,0,0), orientation (0,0,0,1). Point-vector length is independent of
width/height, as in PCL. Density is not inferred from values; NaN/Inf are permitted
in this type. Sensor pose is copied unchanged by both helpers.

The existing algorithm `PointCloud` stays float64 with labels/intensity. Explicit
`PclXYZCloud.from_core(core)` narrows XYZ and creates width N/height 1. PointXYZ
has no labels/intensity; the bridge does not transfer or mutate them.
`pcl_cloud.as_core()` creates a new float64 core cloud with default class 0,
segment -1 and intensity 0, retaining the existing finite-coordinate requirement.
There is no hidden narrowing, label aggregation or nonfinite repair in tracking.

## Fixed voxel filter

Source leaf size is **0.1f** on each axis, with inverse 10.0f. Default PCL settings
include all XYZ data, no distance-field filter, no saved leaf layout and no minimum
support beyond each nonempty voxel. Non-dense inputs skip nonfinite rows for
bounds and ordinary voxel creation; ordinary output becomes dense/unorganized
with height 1. Output order follows increasing linear voxel key: X varies fastest,
then Y, then Z. Duplicate input points contribute separately.

Bounds, differences and coordinate products round in float32 before source
integer casts. World-coordinate voxel bounds are floored, then relative voxel
coordinates use the source's double `floor` minus an explicitly float32 minimum
bound. The implementation retains that order instead of dividing float64 points
by a decimal leaf size. Per-voxel XYZ is summed sequentially in float32 and divided
by the float32 point count. General NumPy mean/pairwise reduction is not used.

The original extent guard computes int64 counts from float32 bounding spans. If
their safe product exceeds int32 maximum, PCL warns and returns a copy of the
**entire original cloud**. Python returns `extent_fallback`, preserving invalid
non-dense rows, dimensions, density and metadata. It does not invent a coarser leaf
or omit invalid rows on that fallback path.

Unsafe int64 extent casts/products, int32 bounds/divisions/multipliers and per-row
index arithmetic return explicit `undefined_source_voxel_*` statuses with no
fabricated output cloud. Available bounds/counts/indices and failed-row diagnostics
remain inspectable. Empty or all-invalid non-dense inputs retain sentinel extrema;
their float32 span overflows before the first integer cast. They therefore report
`undefined_source_voxel_extent_cast`, rather than claiming a safe native empty
filter result. Null helper inputs report the wrapper's dereference limitation.

Dense promises finite data. Contradictory nonfinite dense inputs return
`unsupported_source_dense_nonfinite` for voxel filtering: Eigen scalar/SIMD NaN
bounds can alter the later cast/fallback path. This is an explicit portable input
boundary, not a claim that every target runtime must reach undefined behavior.

### Equal-key sort order

PCL sorts by voxel key with `std::sort`, which does not promise equal-key order.
Different permutations can change float32 sums. Python's default is stable input
order within each voxel. Potentially sensitive voxels (over two nonidentical
points) produce `ok_with_native_sort_ties` and a false comparison flag. Pair sums
and identical-value groups do not depend on that permutation for XYZ output;
diagnostic row order still describes the selected Python ordering.

For a matched native run, pass its eligible sorted row permutation:

```python
matched = decrease_density(cloud, native_sorted_rows=native_order)
```

The sequence must contain each eligible row exactly once in voxel-key order. This
replays actual native accumulation without pretending the host runtime's sort
matches the historical MSVC permutation. Groups and provenance rows are returned
for inspection. A supplied order is a caller-provided comparison input, not proof
that it came from the historical executable. No guessed native tie order is added.

## Centering uses the literal affine expression

`Transform` computes a double centroid from float32 points, then puts its negative
XYZ into an identity `Affine3d`. The 45-degree rotation is commented out. The
double four-term matrix expression is evaluated for each coordinate and narrowed
to float32. Organization, density, header and sensor pose are preserved. The
sensor origin/orientation are **not** translated with the points.

Non-dense centroids skip invalid points; transformation copies invalid rows
unchanged. All-invalid non-dense input divides zero by zero, retains a NaN XYZ
centroid with fourth component 1 and returns the unchanged invalid rows.
Dense input executes arithmetic on every row, including `0 * NaN/Inf` in the
identity matrix's off-diagonal terms. Replacing that expression with simple
coordinate subtraction would change those results.

Empty input returns before resetting the wrapper's initial
`Vector4d::Identity()`: the stored centroid is **(1,0,0,0)** and the unused matrix
translation is (-1,0,0). Output remains empty with copied metadata. Nonfinite
centering returns `nonfinite_pcl_centering`, preserving the initialized values
without a finite core-cloud conversion. Numerical, not NaN-payload, equivalence
is the scope of those IEEE comparisons.

## Frozen reference, notices and tests

`pcl_reference.json` packages **14 original C++ extracts** and normalized hashes
for **19 selected PCL/Eigen files**. The original 41 project/51 Jama/etc hashes and
baseline ID remain unchanged. `PCL_LICENSE.txt` retains the bundled BSD notices;
both resources ship in the wheel. Regeneration is explicit:

```bash
./.venv/bin/python tools/update_pcl_reference.py
./.venv/bin/python tools/update_source_coverage.py
```

The first command requires the local ignored installation bundle and does not
modify it. Normal runtime and source auditing use the frozen package resources.
Source audit additionally checks the selected original dependencies when present;
absence of the large ignored installers is allowed. These are selected numerical
sources, not an exhaustive Eigen/Boost/stdlib include or ABI inventory.

The C++ oracle compiles the frozen PCL bounds, accumulator, centroid, filter,
voxel sorting and affine bodies against the **bundled Eigen headers**. PointXYZ
containers and its single compatible XYZ field/trait dispatch use small shims;
unused filter branches throw if reached. Diagnostic assignments copy locals,
and supplied native sort order enables matched-order comparisons. The original
wrapper bodies execute with a diagnostic filter reference. Unsafe cases compare
only a prefix before integer casts. These comparisons require a C++ compiler and
the bundled Eigen; they skip explicitly if either is absent. Python operation
and its contract tests have no native dependency.

```bash
./.venv/bin/python -m railway_mls pcl-helpers-demo
./.venv/bin/python -m unittest tests.test_pcl_helpers -v
```

The demo uses generated float32 points, ordinary voxel centroids/centering,
metadata, the extent fallback, empty paths and sort-order diagnostics. Tests
compare exact finite float32 bits, double centroids/matrices, shape/density and
sensor metadata, negative/large coordinates, native sort replay and initialized
IEEE behavior. Additional checks cover integer guards, adapters/ownership,
unchanged pipeline scheduling, packaging/audit and CLI output. These component
checks do not close platform, sorting, whole-application or real-data limitations.
See [LIM-038](LIMITATIONS.md) for the post-reconstruction review requirements.
