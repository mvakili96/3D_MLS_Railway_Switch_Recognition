# Source coordinate conversion

Implemented on `feat/coordinate-conversion`. The original C++ remains the
baseline. Conversion is an explicit action; tracking never changes coordinates
or infers a coordinate reference system automatically. No new dependency is
required beyond the existing NumPy core. LAS/LAZ input still uses the optional
decoder extra described in [the I/O guide](PYTHON_LEGACY_IO.md).

| Source | Python |
| --- | --- |
| Nine functions in `UTM.cpp` | `coordinates.deg_to_rad`, `rad_to_deg`, `arc_length_of_meridian`, `utm_central_meridian`, `footpoint_latitude`, `map_lat_lon_to_xy`, `map_xy_to_lat_lon`, `lat_lon_to_utm_xy`, `utm_xy_to_lat_lon` |
| `DataConvertor.cpp:21,254,295` | `ConverterEllipsoid`, `ecef_to_geodetic`, `geodetic_to_ecef` |
| `DataConvertor.cpp:39,82,149,188` | `CoordinateConverter.convert_ecef_to_utm`, `convert_utm_to_ecef`, both callback/no-callback paths |
| `DataConvertor.cpp:303,321,326` | `format_converted_points`, `write_converted_points` |
| `MainForm.cpp:466,504` | `source_menu_conversion`, with explicit input and optional save path |
| `DataConverstionForm.cpp:50,150` | `convert_legacy_files` |

The audit now covers **246 definitions**: the prior 124 engine/UI, 77 inline
header and 18 reader definitions, plus 11 converter, nine UTM and seven conversion
dialog definitions. All 19 header types and 41 project/51 numerical dependency
hashes remain recorded. Conversion-only dialog/control handlers are explicitly
presentation. The two unused PCL voxel/centroid wrappers remain pending.
`complete_reconstruction` remains false until the remaining work and call-path
review are complete. Inventory consistency is not whole-application equivalence.

## Units, constants and raw helpers

The active `UTM.h` macros select **double**, despite its old float32 comment.
`LatLonToUTMXY` actually accepts **degrees**, despite the radian parameter comment:
its body calls `DegToRad`. `UTMXYToLatLon` returns **radians**. Both raw transverse
Mercator helpers use radians. ECEF/geodetic helpers use radians and retain height.
The Python names/signatures document those actual units.

UTM uses `pi=3.14159265358979`, major axis 6378137.0, minor axis 6356752.314 and
scale 0.9996. The ECEF converter uses its separate PI literal
3.1415926535897932 and all eight literal constructor constants, available through
`ConverterEllipsoid`. UTM coefficients, scalar operation order, two-branch ECEF
inverse and initialized IEEE NaN/Inf behavior are preserved. No substitute
projection library, latitude clamp, origin repair or constant recomputation is
introduced. Source UTM header `sm_EccSquared` is not used by its formulas.

Forward UTM retains a requested zone in 1–60; other values trigger
`floor((longitude + 180) / 6) + 1`. It does not clamp that result: longitude +180
can produce 61, and out-of-domain longitudes can produce other zones. Inverse
UTM uses the supplied zone and explicit hemisphere without such auto-selection.
Python requires zone arguments to fit the source int32 type. An automatic-zone
NaN/Inf/out-of-int32 cast raises `UndefinedCoordinateArithmetic` with status
`undefined_source_auto_zone_cast`; no native undefined cast is used as an oracle.

## Persistent conversion state

```python
from railway_mls import CoordinateConverter, load_point_cloud

loaded = load_point_cloud("data/ecef.pts")
converter = CoordinateConverter(loaded.cloud)
projected = converter.convert_ecef_to_utm(32)
utm_cloud = projected.as_cloud()

reverse = CoordinateConverter(utm_cloud).convert_utm_to_ecef(32, False)
```

Each call appends input points to `converted_points`, then converts **every**
record in that buffer through both stages. It does not clear old results. Thus
reusing the same instance reinterprets old output as the new source frame. Use a
fresh converter for an independent action; repeated-history behavior remains
available for source comparison. Input XYZ stays untouched. Class, segment and
intensity values are copied at append time and retained through conversion.
Returned records are value snapshots, independent of the mutable converter.

Forward conversion stores geodetic **degrees** and height as an intermediate,
then projected XY. Reverse conversion stores geodetic **radians** with the old
height, then ECEF XYZ. Callbacks can observe these reached states. Each callback
overload uses nominal total `3 * input_count`, emits every tenth operation across
all three loops and has **no forced final event**. With an old buffer of size M
before append and N input points, actual operations are `N + 2 * (M + N)`;
progress can exceed the nominal total. Empty input still reprocesses old output.
The native progress-bar control can fail on out-of-range values; Python exposes
the raw callback values without recreating that display limit. Exceptions retain
the reached mutable buffer and prior callback effects.

At the ECEF origin, initialized arithmetic yields NaN coordinates. This returns
`nonfinite_conversion`, with a false comparison flag. `as_cloud()` applies the
existing core's finite-coordinate requirement and raises for those values.
Repeated nonfinite buffers can later reach an undefined automatic-zone cast.
Neither case is repaired with an invented location. Forward per-point zones are
exposed diagnostically; the original converter ignores those returns and its
export does not store zone/hemisphere/CRS metadata. Mixed-zone output remains
possible and is not relabeled as a single inferred CRS.

## Source menus and batch binding

`source_menu_conversion(path, direction, destination=None)` preserves MainForm's
hardcoded zone **32** and inverse `southhemi=false`. Forward menus load LAS;
inverse menus load PTS. Conversion and optional saved PTS use no callbacks.
Explicit paths replace dialogs. Managed cancellation, stale-dialog reentry and
uninitialized converter-pointer reads are outside this interface's claim.

`convert_legacy_files(paths, destination, direction, zone, ...)` follows both
conversion-dialog loops. Exactly `.las` selects LAS, exactly `.pts` selects PTS,
and **every other suffix** selects legacy count-prefixed CSV. Therefore that
source batch dispatcher does not have the general loader's `.laz` handling.
Only exactly `.csv` produces comma-separated `.csv` output; other suffixes produce
PTS. Names append `_utm` or `_ecef`. Missing files are skipped but remain in the
nominal aggregate total `3 * file_count`.

The batch inverse also hardcodes **false** for southern hemisphere. This source
binding is preserved; the general converter and CLI require a caller-supplied
inverse hemisphere. Forward aggregate progress advances after reading,
conversion and saving. Inverse aggregate progress advances its second step
**before** conversion, as in the source. Reader, converter and writer retain
their separate callback cadences. Python validates paths/filesystem operations
and requires explicit overwrite, rather than simulating silent native stream
failures or recreating the GUI's progress bars and text fields.

## Export and CLI

Converted PTS/CSV has a count header and fixed three-decimal XYZ followed by
**intensity**. Class/segment metadata is retained in memory and omitted from
this source file format. Default delimiter is a space; the callback/delimiter
overload also supports comma or another ASCII delimiter. Writers flush each row
before callbacks, emit only every tenth row and have no final completion callback.
Initialized nonfinite output remains inspectable/writeable with
`nonfinite_coordinate_export`; portable nonfinite spelling and Windows newline
byte identity are excluded from comparison. Existing outputs require explicit
`overwrite=True`/`--overwrite`. Rounded files lose precision and are not lossless
algorithm checkpoints.

```bash
./.venv/bin/python -m railway_mls conversion-demo
# Once real inputs and their coordinate frames are known:
./.venv/bin/python -m railway_mls convert-cloud data/ecef.pts outputs/utm.pts --direction ecef-to-utm --zone 32
./.venv/bin/python -m railway_mls convert-cloud data/utm.pts outputs/ecef.pts --direction utm-to-ecef --zone 32 --hemisphere north
```

The demo uses twelve generated geodetic/ECEF points, forward/inverse conversion,
source progress/export, repeated-buffer behavior and the origin diagnostic.
Strict JSON distinguishes nonfinite values from absent state through typed
JSON-pointer annotations. The file command reports sampling, counts, used zones,
hemisphere and output status. General CLI inverse conversion requires an explicit
hemisphere; it does not silently copy the source dialog's hardcoded false.

## Comparison evidence and limitations

The oracle compiles the **complete original UTM.cpp and all DataConvertor.cpp
bodies**, with simple PCL point/container shims. It compares scalar helpers,
projection sweeps, both ECEF inverse branches, callback-visible intermediate
state, all conversion/export overloads, labels, repeated/mixed buffers, empty
input with prior output, source-zone edges and initialized IEEE cases. Unsafe
casts are excluded. The batch loop is checked against explicit source scheduling,
dispatch/naming and file fixtures; managed GUI execution is not claimed.

Tests cover core-cloud boundaries, callback failures, snapshots, rounded text,
flush timing, overwrite policies, CLI validation and expanded inventory parsing.
Component agreement does not establish global geodetic accuracy or matched
Windows/libm behavior. Remaining PCL helpers, complete call-path review,
whole-application/native reference, real data and profiling remain separate work.
See **LIM-037** in [the limitations register](LIMITATIONS.md). All earlier IDs
remain open for the requested review after full reconstruction.
