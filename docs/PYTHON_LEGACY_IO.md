# Legacy point-cloud I/O

Implemented on `feat/legacy-point-cloud-io`. The current source snapshot remains
the baseline. Readers and exporters preserve its record order, field assignment,
bounds and logical text formats. Core extraction remains independent of files.

| Source | Python |
| --- | --- |
| `TXTFileReader.cpp:21,70,128,186` | `LegacyTextReader.read`, `loaded_data` |
| `LASFileReader.cpp:18,59,108,150,169` | `LegacyLasReader.read`, `loaded_data`, `read_bounds` |
| `RailwayExtraction.cpp:108–143` | `ReaderBounds`, `LegacyPointCloud.grid_bounds` |
| `TXTFileReader.cpp:191,208,247` | `format_rail_vector`, `write_rail_vector` |
| `MainForm.cpp:149,172` | `write_layer_vectors`, with an explicitly selected layer |
| `MainForm.cpp:443,577`; `HashMap.h` | `LegacyFileSession.open_pts`, `load_file` |

The audit expands from 201 to **219 definitions**: 124 engine/UI, 77 inline header
and all 18 definitions in the two reader files. It retains 19 named header types
and the unchanged 41 project/51 dependency hashes. The original `savePoints`
handler is empty; `LoadPointCloud` only calls grid generation. Their dispositions
now reflect those bodies. No point exporter is invented for the empty handler.

Both readers' actual loading paths are implemented. Two unused PCL wrappers,
`DecreaseDensity` (0.1 float voxel-centroid filtering) and `Transform` (centroid
translation, rotation commented out), remain `native_filter_pending`. They are
not part of either loading path. Coordinate-conversion workflows and call-path
review also remain pending. No full-reconstruction claim is made.

## Dependencies and commands

The existing NumPy-only installation supports PTS/CSV, exports, the I/O demo and
all core algorithm stages. LAS/LAZ uses an optional decoder extra:

```bash
./.venv/bin/python -m pip install -e ".[las]"
./.venv/bin/python -m railway_mls legacy-io-demo
./.venv/bin/python -m railway_mls inspect-cloud data/example.pts
./.venv/bin/python -m railway_mls inspect-cloud data/example.laz --chunk-size 1000000
```

The extra uses laspy with lazrs for compressed records. Reading is chunked;
decoded retained records are still assembled into one owning in-memory cloud.
See the official [laspy reading documentation](https://laspy.readthedocs.io/en/latest/basic.html)
and [optional LAZ installation](https://laspy.readthedocs.io/en/latest/installation.html).
This increment was checked with laspy 2.7.0/lazrs 0.8.2 and NumPy 1.26.4.

`legacy-io-demo` creates temporary synthetic text fixtures, reads 12 PTS records,
formats all three vector variants, reads the seven-row paired export as legacy
CSV and reports an undefined indexed case. Files are cleaned up afterward.
The demo needs no real inputs or LAS extra. `inspect-cloud` only loads/reports
counts, source-record indices, initialized labels and reader bounds; extraction
still requires an explicitly supplied seed. It reports unknown units/CRS as null.

## Record and bounds contracts

PTS begins with a count, then `x y z intensity` records. CSV also begins with a
count and has comma-delimited records. It is not an ordinary header-row database
CSV. Both retain every declared record, ignore extra fields/trailing records and
initialize class 0/segment -1. CSV's fourth field is **intensity**, even when it
originated as a vector point-type tag; a fifth exported index is ignored.

Source tokenization uses only the literal space or comma delimiter, collapsing
repetitions and skipping empty fields. Tabs are not PTS field separators. Numeric
prefixes follow C-locale `atoi`/`atof` behavior; nonnumeric prefixes become zero.
A negative or nonnumeric count reads no records. Native 200-byte line buffers
permit at most 199 characters after Windows CRLF translation. Python rejects
overlong records rather than simulating a failed stream and stale token reads.
Text decoding is ASCII, with explicit errors for absent fields/records, embedded
NULs, unsupported integer ranges and nonfinite core coordinates. These are input
policies, not fabricated native values or a general-purpose robust CSV parser.

LAS/LAZ retains **zero-based record indices 0, 2, 4, ...**, including the first
record, with parity continuing across chunk boundaries. It preserves scaled
double XYZ and intensity, but imports no classification, source ID, RGB, return
number or other fields into algorithm labels. No intensity threshold is active.
Header extrema include all original records, including skipped points; bounds
are not recomputed from the retained sample. LAS and LAZ share source format
identifier `las` because both decode through the same legacy LAS reader path.
The optional modern decoder does not establish equivalence with libLAS's version,
compression backend, malformed-file handling or supported point-format set.

Text readers reset only X/Y extrema, retaining Z. Their default point constructor
initializes Z=0, so it stays zero across ordinary reads. Empty XY extrema stay
`+1e100`/`-1e100`. `ReaderBounds` preserves these literal outputs. The native grid
uses only X/Y; Python's validated `Bounds3D` also requires enclosing Z. Therefore
`loaded.grid_bounds()` explicitly expands Z to contain the loaded data while
retaining source XY. It never overwrites reported bounds, changes coordinates or
performs a CRS transform. Empty clouds need explicitly chosen grid bounds;
invalid/stale XY header bounds are rejected by the existing grid contract.

The reader commits its loaded-cloud pointer after reading all records. Every
tenth **input** record invokes the reader callback, followed by an unconditional
final `(declared_count, declared_count)`, even when that duplicates a periodic
event. LAS counts skipped records too. Periodic callbacks see the previous
loaded-data pointer; the final event sees the new one. Reader callbacks exclude
the separate grid/display progress UI. Failed Python reads leave the old loaded
cloud available; text bounds retain updates reached before failure.

## Using loaded data

```python
from railway_mls import InitialLocation, Point3D, load_point_cloud, run_pipeline

loaded = load_point_cloud("data/example.pts")
seed = InitialLocation(Point3D(x, y, z), angle, mode)  # Explicit caller input.
result = run_pipeline(loaded.cloud, seed, bounds=loaded.grid_bounds())
```

`record_indices` is a read-only mapping back to original file rows. The owning
cloud retains double XYZ and shared mutable algorithm labels. Input CRS and units
are not inferred or transformed. Pass known metadata through `PipelineOptions`.

For source-like file switching, `LegacyFileSession.load_file(path)` accepts the
source's case-sensitive `.las`, `.laz` and `.pts` suffixes. Exact filename strings
key the cache. `HashMap::get` inserts a null miss, including unsupported names;
successful reentry reuses the same engine, cloud, layers, labels and RNG. All
session engines share an explicitly owned RNG. Direct `open_pts(path)` always
creates a fresh engine without registering it in that cache. Dialogs, tree-view
selection and rendering are excluded; a caller supplies paths and selected layers.
Python validates grid construction before committing a new session entry. Native
exception/failure-side-effect equivalence is not claimed.

## Three source export formats

```python
from railway_mls import format_rail_vector, write_layer_vectors

text = format_rail_vector(center, left, right, indexed=True)
paths = write_layer_vectors("outputs/track.csv", layer)
```

All variants use a count header and fixed **three-decimal** XYZ. Default export
writes centers with their point-type tags. Paired export writes all centers,
then all rail1 rows, then rail2, using tag **100 on both rails**. Unequal vector
lengths are allowed in that unindexed variant.

Indexed export declares `3 * len(left)` rows. For each left record it writes left
(tag 100), center (its own tag), then right (tag 200), adding the same zero-based
index to all three. It skips exactly **one** center tagged 2 before writing that
center; consecutive markers are not all skipped. Extra tail records are ignored.
Missing center/right records return `undefined_source_indexed_*`, the failed
iteration and the defined logical text prefix. Native unsafe indexing is never
executed in comparisons. The file writer rejects that result before opening an
output; it does not write a completed fabricated vector file.

Layer export follows MainForm's `_with_track_points.csv` naming. Unindexed output
writes center and paired files; indexed output writes only the paired filename.
The new writer requires explicit `overwrite=True` to replace an existing file.
Native writers truncate automatically. Output uses logical LF/ASCII text; Windows
CRLF byte identity and platform-specific nonfinite number spelling are outside
the text-parity claim. Initialized nonfinite points still produce logical output,
with `nonfinite_legacy_export` and a false comparison flag. Multi-file writing can leave the first file if a later
write fails. No transaction or database schema is implied. Rounded vector outputs
are results, not raw MLS inputs or lossless reconstruction checkpoints.

## Validation and remaining limitations

The C++ oracle compiles all original TXT reader/export bodies, LAS reader/header
methods and the three engine bounds adapters. PCL data containers are small
shims. The libLAS shim supplies already-decoded records, so comparisons establish
record selection, field assignment, header bounds, callback cadence and export
format/order, not codec identity. Generated LAS/LAZ files exercise the real Python
decoder separately, including scales/offsets, skipped extrema and odd chunks.
The historical MIN/MAX macro inclusion chain is not established. The oracle uses
conditional comparisons; equal signed-zero bounds remain a runtime/macro policy,
without a matched Windows parity claim.

Tests also cover malformed inputs, loaded-data identity, file-cache state, output
ownership/overwrite, CLI behavior, inventory drift and a full-precision synthetic
PTS roundtrip into the existing initialization/pipeline with identical initialized
vectors, labels and random state. No native undefined read is used as an oracle.
See **LIM-036** and updated LIM-014/LIM-015 in [the register](LIMITATIONS.md).
All limitations remain open for the requested post-reconstruction review.
