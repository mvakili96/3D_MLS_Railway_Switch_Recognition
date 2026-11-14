# Python rail evaluator: reconstruction and validation

The separate **2018 MFC/LiDAS rail evaluation command** is reconstructed in
`railway_mls.evaluation`. This adds evaluation to the existing Python tracker;
it does not replace the tracker's frozen 2017 ADE/PCL reference. The supplied
2018 source is authoritative for this evaluator, including its metric quirks.

All six planned computational milestones are implemented: source contracts,
input adapters, geometry/assignment, metrics, batch execution and validation.
The changes are together on `feat/evaluation-reconstruction` for review. No
original data, source, workbook or saved predictions are edited.

## Running it

From the repository root, activate the existing environment and install the LAS
extra if it is not already available:

```bash
source .venv/bin/activate
python -m pip install -e '.[las]'
python -m railway_mls evaluation-demo
python -m railway_mls evaluation-audit --repo .
```

The demo uses five generated points to exercise all four source outcome codes
and an unassigned reference point. It is a computational smoke test.

Evaluate every supplied Vancouver annotation part using the saved original
predictions, and reconcile the historical workbook/outcomes:

```bash
python -m railway_mls evaluate-rails \
  --manifest configs/evaluation-vancouver.json \
  --output outputs/vancouver-evaluation-run-01 \
  --historical
```

Choose a new output directory for each run; existing directories are never
overwritten. To run a smaller case, append `--case P8_01` and use a different
output directory. Repeat `--case` for multiple cases. `--chunk-size` controls
working memory, defaulting to 250,000 records; it does not thin the cloud.
`--point-outcomes` additionally writes a CSV row for every input record.

The manifest's dataset root is relative to the manifest file. `--data-root`
overrides it when the local dataset has moved. Its SHA256 checks protect the
specific supplied reference/model version; do not remove a mismatch merely to
make an unrelated dataset appear equivalent.

For one explicit native-format case:

```bash
python -m railway_mls evaluate-rails \
  --las /path/to/annotated.las \
  --left /path/to/Path1_Left.TXT \
  --right /path/to/Path1_Right.TXT \
  --output outputs/evaluation-single
```

The only runtime dependencies are NumPy and the existing optional laspy/lazrs
extra. A C++ compiler is needed for native comparison tests, not scoring. No
Windows, MFC, Open3D, Excel, pandas or openpyxl installation is needed.

## Inputs and API

The public entry points are:

```python
from railway_mls.evaluation import (
    ReferencePoints, RailSegments, evaluate_rails, evaluate_chunks,
)
from railway_mls.evaluation.io import iter_reference_las, read_rail_segments

left = read_rail_segments("left.txt")
right = read_rail_segments("right.txt")
result = evaluate_chunks(iter_reference_las("reference.las"), left, right)
print(result.summary())
```

`ReferencePoints` owns read-only float64 XYZ, raw classification bytes and
increasing original record IDs. `RailSegments` owns read-only `(N, 2, 3)`
endpoints in file order. `evaluate_rails` accepts one in-memory reference;
`evaluate_chunks` accepts consecutive chunks. Every call starts a fresh session.
`collect_points=True` explicitly retains point outcomes in memory; the default
streams statistics without retaining the full cloud. Batch point CSVs stream
through a callback instead.

Unlike the tracking reader, the evaluator reads **all LAS records** and keeps
the original annotation byte. Exactly byte `3` is positive; all other values
are background. Legacy flag bits are not masked off. LAS layouts through 1.3,
point formats 0–5 are accepted; newer layouts are rejected. LAZ decoding is a
portable convenience with the same record/coordinate contracts. Truncation and
nonfinite coordinates fail explicitly. Coordinates stay in their original
world frame; there is no recentering or coordinate conversion.

Native model files contain a segment count followed by six comma-separated
begin/end coordinates per segment. Their filename extension is not significant.
Canonical finite files preserve the source values and order. Malformed tokens,
missing/extra rows and nonfinite endpoints are rejected; the source's unsafe
token reads and permissive numeric-prefix conversion are not emulated.

The checked-in manifest selects 24 numbered annotation parts for paths 1–9 and
18 saved model files. Five head-only aggregates and identical directory copies
are excluded. Parts P6_01/P6_02 and P7_01/P7_02 contain the same ordered XYZ with
complementary positive labels: both parts are retained as independent sessions.
Their other-part positives therefore act as background under each session's
literal source labeling rule. Part-wise totals are not globally unique scene
counts or a certified complete-label precision benchmark.

## Preserved source behavior

| Operation | Reconstructed behavior |
| --- | --- |
| Candidate assignment | Native 0.3 m buffer polygon and `pnpoly2`, including its `1e-10` cross-product classification. Edge points are generally excluded. |
| Membership precedence | First eligible segment wins: all left segments in file order, then all right segments. No nearest-segment reassignment. |
| Positive pass | Horizontal distance `<= 0.12 m` and absolute vertical residual `<= 0.05 m`. |
| Background pass | Strict `< 0.12 m` and `< 0.05 m`. |
| Source counters | Positive pass/fail increments `nTP`/`nFP`; background pass/fail increments `nFN`/`nTN`. The source FP/FN names differ from conventional classification. |
| Printed percentages | `source_precision_percent = 100*nTP/(nTP+nFP)`; `source_recall_percent = 100*nTP/(nTP+nFN)`. The first is assigned-positive success percentage. |
| Vertical residual | Perpendicular XY foot, unsigned distance from the segment beginning, and native height interpolation. |
| Error statistics | All assigned positives, including failures. Absolute vertical mean; signed vertical residuals around that absolute mean for the source standard deviation. Sample denominator `count-1`. |
| Unassigned records | Excluded from source counters/statistics; positive/background counts reported separately. |
| Empty/singular statistics | Native IEEE NaN/Inf/signed-zero behavior retained and described in JSON's `nonfinite_values`. |
| Repeated calls | Fresh memberships per case; native GUI append/re-evaluation pointer accumulation is excluded from this batch interface. |

The X-sorted index is only a candidate filter. The exact polygon predicate and
original segment precedence determine membership. Degenerate/tiny/nonfinite
polygons use a full check because the native predicate can accept collinear
points outside an ordinary polygon bounding box. Positive residuals are spooled
to temporary files by segment, in original point order, then reduced in the
source's order in both statistics passes. Memory is bounded by the chunk size
and segment count; temporary disk use scales with assigned positive records.

Point output segment IDs enumerate left then right segments from zero; `-1`
means unassigned. Outcome codes 1/3/4/2 follow the source. Outcome **0** means
unassigned and has no native initialized `ClassCodeRe` equivalent. Historical
TPFN files use a separate **1/2 success/failure** convention and are never read
as reference classifications.

## Source mapping and audit

The package's `reference.json` freezes 35 files from the 2018 evaluation project
and records 12 computational dispositions plus inactive/presentation/separate
algorithm boundaries. The independent reference fingerprint is:

`2b73b2b3313d64eaa65026dd15cb4da6144fc1109279ac58cf43a8e03400d85c`

| Native source | Python contract |
| --- | --- |
| `ViewerDoc.cpp:4916`, `OnHeadptsandtrackOpenpoints` | `io.iter_reference_las`; full-resolution, immutable annotations, fresh session |
| `ViewerDoc.cpp:4996/5045`, two vector loaders | `io.read_rail_segments`; count-prefixed ordered endpoints |
| `las/LIDARPT.h:462/712` line parameter/buffer helpers | `geometry.prepare_line` |
| `las/LIDARPT.h:120`, `pnpoly2`; `ViewerDoc.cpp:4609` polygon selection | `geometry.polygon_membership`, `geometry.assign_points` |
| `ViewerDoc.cpp:4362/4387/4404/4452` projection and distance helpers | `geometry.residuals` |
| `ViewerDoc.cpp:5094`, `OnHeadptsandtrackEvaluation` | `core.evaluate_chunks` and `evaluate_rails` |

The reference fingerprint in run reports identifies the expected implementation
baseline. `evaluation-audit --repo .` separately checks the locally supplied
source files against it. A missing source package reports `unverified`, not a
passing audit; an installed evaluator can still score without that package.

The native test oracle compiles the original geometry helpers, polygon selector
and evaluation command with small data/MFC presentation shims. Diagnostics copy
the source summary locals before its existing printf calls. The original
arithmetic, loops, membership and thresholds are preserved. This tests the
computational source extracts on Linux, not the Windows executable or LAS SDK.

## Vancouver validation checkpoint — 2026-11-14

The full batch completed **24/24 cases**, reading **38,768,029 records**, including
**4,710,223 reference-positive records counted by part**. It assigned 4,699,754
positive records and excluded 10,469. The older workbook/TPFN denominator is
4,699,760: the assigned count differs by just six records (three on path 1, two
on path 2, one on path 8). This narrows the earlier denominator question; it
does not establish exact artifact/version equivalence.

| Path | Assigned positives | Source positive passes | Assigned-positive success % |
| --- | ---: | ---: | ---: |
| 1 | 1,414,659 | 1,122,690 | 79.3612 |
| 2 | 1,374,208 | 1,060,300 | 77.1572 |
| 3 | 558,862 | 467,053 | 83.5722 |
| 4 | 237,913 | 184,134 | 77.3955 |
| 5 | 329,073 | 248,745 | 75.5896 |
| 6 | 263,770 | 188,705 | 71.5415 |
| 7 | 321,860 | 181,461 | 56.3789 |
| 8 | 81,887 | 69,952 | 85.4250 |
| 9 | 117,522 | 98,406 | 83.7341 |

These values evaluate **saved original predictions**, not Python tracker output.
They differ substantially from the historical headline. All nine historical
TPFN success/failure counts do match the older workbook exactly; its displayed
nine-path macro recall is 99.4633%, while pooled historical success is 99.7347%.
Thresholds, vertical gating/statistics and artifact/protocol correspondence
require further investigation. No parameter was changed to force agreement.

Verification includes 16 synthetic native-comparison test methods and one
real-data comparison method covering **11,705 actual records across all 24
parts**, with every saved segment for each path. Memberships, outcome codes and
integer counts match exactly; numerical values match with `atol=1e-9` and
`rtol=1e-12`. Full Path 8 metrics/counts are identical for chunk sizes 65,536 and
250,000. Synthetic tests also compare indexing with full scans, chunk sizes,
threshold equality, overlaps, singular geometry and repeatability.

The full regression suite passed **602 test methods**, including **42 evaluation
tests**, with the local source/data and optional LAS dependency available. The
original source and reconstruction audits also passed. A built wheel was checked
for its evaluator/reference manifest and exercised directly without adding data
files to the distribution.

The full real-data report and per-case/path CSVs are local and ignored at
`outputs/vancouver-evaluation-source2018-verified/`. Reproduce them with the command
above and a new output directory. Failures, input SHA256 mismatches and missing
historical files produce an explicit failed report/exit status. Nonfinite
source statistics are reported separately and are not hidden as zeros.
Partial manifest selections are marked incomplete for their path; their deltas
against full-path historical counts must not be interpreted as comparable scores.

## Evaluating future Python predictions

One explicit layer from an existing pipeline `vectors.csv` can be evaluated:

```bash
python -m railway_mls evaluate-rails \
  --las /path/to/matched-reference.las \
  --python-vectors outputs/my-tracking-run/vectors.csv --layer 0 \
  --output outputs/python-prediction-evaluation
```

`read_python_vectors(path, layer=...)` and `rails_from_layer(layer)` convert
consecutive vertices within each selected rail. They never join layers. Native
rail1/rail2 identity is mapped to evaluator left/right precedence; it is not
inferred geographically. A CSV's vertex IDs must be contiguous from zero.
Tracking-policy provenance can be supplied in programmatic case metadata;
when absent it is explicitly reported as not supplied.

The current viewer covers a small local trace, whereas these reference models
span nine paths. Generating Python predictions over matched coverage is the
next project. Full-scene generation, switch-type scoring, the additional
100-region case mapping, corrected metrics and running the original Windows
application are deferred. [LIM-012](LIMITATIONS.md) and all existing limitation
IDs remain open; passing this reconstruction does not resolve them.
