# Railway track and switch recognition from 3D MLS

This repository preserves the existing C++ implementation of railway tracking
and switch-region recognition from 3D mobile laser scanning (MLS) data. The Python
implementation provides point/state models, geometry, paired rail windows,
spatial lookup, the custom height GMM, paired rail-head extraction, line fitting,
source point grouping, MDL model selection, observation selection and paired-line
gauge adjustment, full seed initialization, Kalman components and sequential
tracking with window retries, stopping rules, turnout stacks and source template
recognition, toe detection and endpoint refinement, branch scheduling and
bidirectional orchestration, plane fitting, switch-box detection, grade/curve
analysis and source GWAD row/text generation. An integrated in-memory pipeline,
synthetic workflow, diagnostic outputs and explicit source coverage audit are
available. The [computational reconstruction audit](docs/RECONSTRUCTION_AUDIT.md)
is complete; all retained limitations remain open for review and real-data
validation is pending. The current
unpacked C++ code is the source of truth: preserve its computational behavior and
document proposed corrections separately.

## Repository contents

- `ADE code/PCLProject/PCLProject/PCLProject/`: original C++/CLI application,
  railway algorithm, geometry/state models, readers, exporters, and UI resources.
  The small `Jama/` and `etc/` numerical helpers are retained because the source
  uses them directly.
- `ADE code/PCLProject/PCLProject/Visualizer.sln` and project configuration:
  original Windows project/build references.
- `Multi-Railway_Track_and_Switch_Region_Recognition_Using_Mobile_Laser_Scanning_Data.pdf`:
  reference paper.
- [PYTHON_PORT_PLAN.md](PYTHON_PORT_PLAN.md): source audit, behavior differences,
  and staged implementation plan.
- [docs/LIMITATIONS.md](docs/LIMITATIONS.md): persistent limitations register and
  required review after full reconstruction, including the toe-reference issue.
- [docs/RECONSTRUCTION_AUDIT.md](docs/RECONSTRUCTION_AUDIT.md): completed scope
  review, call-path evidence, numerical input adapters and validation boundaries.
- [docs/PYTHON_VIEWER.md](docs/PYTHON_VIEWER.md): interactive Vancouver/LAS viewer,
  two-point picking, fitted starting windows and live Play/Pause/Step controls.
- `src/railway_mls/`: installable Python foundation, configuration and synthetic CLI.
- `tests/`: component/integration checks and comparisons with original C++ extracts.
- [docs/PYTHON_FOUNDATION.md](docs/PYTHON_FOUNDATION.md): implemented behavior,
  source mapping, preserved quirks and remaining scope.
- [docs/PYTHON_GMM_HEADS.md](docs/PYTHON_GMM_HEADS.md): source GMM, rail-head
  extraction, controlled random draws, API and numerical behavior.
- [docs/PYTHON_FITTING_MDL.md](docs/PYTHON_FITTING_MDL.md): fitting variants,
  source grouping rules, MDL scores and missing-candidate diagnostics.
- [docs/PYTHON_OBSERVATION_GAUGE.md](docs/PYTHON_OBSERVATION_GAUGE.md): observation
  and side-point selection, support state vectors, gauge formulas and native ties.
- [docs/PYTHON_KALMAN_INITIALIZATION.md](docs/PYTHON_KALMAN_INITIALIZATION.md): complete
  seed initialization, filter formulas, bundled matrix behavior and numerical cases.
- [docs/PYTHON_TRACKING.md](docs/PYTHON_TRACKING.md): sequential loop, retry order,
  accepted/rejected candidates, stack snapshots and native-comparison scope.
- [docs/PYTHON_RECOGNITION.md](docs/PYTHON_RECOGNITION.md): all 20 templates, stack
  signatures, labels, ambiguity, disabled context and tracking postpass integration.
- [docs/PYTHON_TOES.md](docs/PYTHON_TOES.md): toe grouping, fitting, refinement,
  projection and the source's uninitialized stack-reference limitation.
- [docs/PYTHON_ORCHESTRATION.md](docs/PYTHON_ORCHESTRATION.md): engine state,
  branch seeds/filters, center tags and source bidirectional vector assembly.
- [docs/PYTHON_SWITCH_BOXES.md](docs/PYTHON_SWITCH_BOXES.md): candidate regions,
  source JAMA plane fits, grouping/gates, class-5 labels and tracking integration.
- [docs/PYTHON_GRADES.md](docs/PYTHON_GRADES.md): vertical division, source curve
  formulas, point-tag behavior, grade reports and GWAD output structure.
- [docs/PYTHON_PIPELINE.md](docs/PYTHON_PIPELINE.md): unified API/CLI, supplied-state
  provenance, JSON/CSV artifacts, source audit and explicitly pending scope.
- [docs/PYTHON_UTILITIES.md](docs/PYTHON_UTILITIES.md): standalone paired GMM/height
  filtering, scalar geometry, index cycling and endpoint-only boundary state.
- [docs/PYTHON_LOOP_CROSS.md](docs/PYTHON_LOOP_CROSS.md): standalone loop/cross
  detection, integer-phase validation, projection and undefined-fit diagnostics.
- [docs/PYTHON_LEGACY_IO.md](docs/PYTHON_LEGACY_IO.md): PTS/CSV/LAS/LAZ readers,
  source sampling/bounds, file switching/cache and three legacy vector formats.
- [docs/PYTHON_COORDINATES.md](docs/PYTHON_COORDINATES.md): source ECEF/geodetic/UTM
  formulas, persistent conversion state, batch/menu bindings and converted exports.
- [docs/PYTHON_PCL_HELPERS.md](docs/PYTHON_PCL_HELPERS.md): standalone float32 voxel
  centroids, double-centroid translation, metadata, sort replay and integer guards.
- [docs/PYTHON_HEADER.md](docs/PYTHON_HEADER.md): inline definition/type inventory,
  legacy points, tagged line/cache operations and segment state helpers.

The original application uses Windows Forms/C++/CLI, PCL, VTK, Boost and LAS
libraries. Project files contain machine-specific paths. Retaining them documents
the existing setup; it does not provide a portable build of the Windows application.
The Python foundation runs independently of those native dependencies.

## Run the implemented Python stages

For the interactive desktop demo, from a graphical terminal:

```bash
source .venv/bin/activate
python -m pip install -e ".[viewer]"
python -m railway_mls view
```

With the local `Vancouver/` data present, this opens `Thales_11_20Las/16.las`.
Click **Use Vancouver source example**, **Confirm start**, then **Play**. You can
also **Pick two points** with Ctrl-click to define a start and direction. The
viewer traces the main path, returns to detected switch points and traces the
other eligible paths. Gold marks the main centerline and purple marks branches.
The default 30 m radius covers the full example tile. Loading it uses more memory
and time than the earlier smaller crops; progress appears in the viewer.
Use **Restart with larger region** if a smaller crop reaches its edge.
Uncheck **Trace other switch paths** before initialization, or run
`python -m railway_mls view --main-only`, to compare the earlier main-only demo.
Rendering uses a bounded preview; inference uses the source's even-record selection.
The viewer reports any omitted LAS header-rounding records. Raw files stay local
and unchanged. Switch tracing uses an explicit viewer recovery of missing source
references from saved fitted rails; the strict computational baseline is retained.
See [the viewer guide](docs/PYTHON_VIEWER.md) for setup, other files and limits.

From the repository root, using Python 3.10 or newer:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python -m railway_mls demo
python -m railway_mls heads-demo
python -m railway_mls mdl-demo
python -m railway_mls observation-demo
python -m railway_mls initialization-demo
python -m railway_mls tracking-demo
python -m railway_mls recognition-demo
python -m railway_mls toe-demo
python -m railway_mls branch-demo
python -m railway_mls bidirectional-demo
python -m railway_mls switch-box-demo
python -m railway_mls grade-demo
python -m railway_mls pipeline-demo
python -m railway_mls pipeline-demo --scenario components --output outputs/components-001
python -m railway_mls source-audit --repo .
python -m railway_mls reconstruction-audit --repo .
python -m railway_mls utilities-demo
python -m railway_mls header-demo
python -m railway_mls loop-cross-demo
python -m railway_mls legacy-io-demo
python -m railway_mls conversion-demo
python -m railway_mls pcl-helpers-demo
python -m unittest discover -s tests -v
```

The demo generates 1,460 synthetic points with ground/web/head heights and clutter,
then constructs two rail windows and selects 180 points in each. It performs
geometry and spatial membership. `heads-demo` adds the source height GMM and its
required 100-attempt line fit, then extracts 60 head points per rail, labels them
with class code 3 and reports updated heights. No real input data is required.
`mdl-demo` fits and groups 270 synthetic candidates and selects three lines at the
default weight. `mdl-demo --weight 0.05` selects two and `--weight 0` selects one
from the same initialized candidates. It reports source flags, memberships and
scores. `observation-demo` uses 540 noisy synthetic head candidates in two
neighborhoods, selects three models per neighborhood, chooses 100 observation
points per rail, fits supporting lines, forms observation vectors and applies the
source weighted gauge adjustment. `initialization-demo` initializes paired rails
from 1,098 raw synthetic ground/web/head points, saves one segment and eight boundary
edges, and demonstrates prediction/update with controlled synthetic observations.
Initialization consumes 400 raw random draws. `tracking-demo` continues from raw
synthetic initialization through sequential inference, saves eight additional
segments, and reaches the source point-shortage stop. Its 6,192-point example
reports the known single-rail MDL undefined-intermediate policy explicitly.
`tracking-demo --side-rails --max-steps 3` exercises fully initialized models and
an active turnout stack. On natural completion the tracker recognizes eligible
completed stacks. `recognition-demo` exercises all 20 source patterns: 16 are unique
matches and four return ambiguity code 999 because the catalogue contains duplicate
patterns. `toe-demo` exercises the twelve supported toe types using initialized
reference fixtures and reports the legacy missing-reference case explicitly.
Tracking invokes toe detection after recognition; tracker-created stack snapshots
have uninitialized member-line references, which the port reports as undefined
instead of producing a fabricated toe. `branch-demo` runs real inference on two
raw track corridors with a supplied toe fixture, accepting six main and five
branch steps. `bidirectional-demo` runs both travel directions from an interior
seed and assembles the vectors using source reversal/swapping rules. Neither demo
claims automatic recovery of undefined toe geometry. `switch-box-demo` evaluates
two planar patches beside a supplied toe, detects two boxes, labels 2,312 points
with class code 5 and consumes 1,200 random draws. Main tracking invokes this
postpass after toe detection. Sparse undefined plane outputs are reported explicitly.
`grade-demo` analyzes a supplied 17-point center vector into three sections and
six GWAD rows, preserving the source's curve formula, tag mutations and variable
parameter columns. Grade analysis is an explicit post-tracking action.

`pipeline-demo` runs raw bidirectional extraction with optional grade/GWAD actions
enabled by the demo. `--scenario turnout` runs raw side rails through stack
recognition; ambiguity and source gates remain visible. `--scenario components`
uses explicitly supplied reference/branch fixtures to exercise successful toe,
branch and box computation. `--scenario missing-references` retains the undefined
toe-reference result. The JSON reports stages, supplied state, limitations and
reproducibility metadata. `--output` creates a new diagnostic directory;
`--steps` and `--record-random` opt into detailed traces. Completed workflows with
known issues report `completed_with_limitations`, without implying source parity
or accuracy. Existing limitations stay open.

`utilities-demo` exercises eight source utility contracts, including empty bounds,
paired GMM/height filtering, literal scalar geometry and polygon boundaries with
absent caches. These contracts are source-tested; loop/cross scheduling remains
inactive.

`header-demo` exercises point/line/cache and neighbor-region state without input
data. The header audit maps 77 uncommented definitions and 19 named types,
including explicit policies for unknown and undefined native values.

`loop-cross-demo` supplies initialized rail references and 66 synthetic points.
Detection fits 60 candidates, appends one record and suppresses a repeat;
validation retains it. A separate sparse attempt reports the source's
uninitialized failed-fit output explicitly. Automatic calls remain inactive.

`legacy-io-demo` reads generated temporary PTS/CSV files and exercises all three
source rail-vector export formats. For real LAS/LAZ files, install the optional
decoder with `python -m pip install -e ".[las]"`. Then use
`python -m railway_mls inspect-cloud data/example.laz` to inspect loaded records.
LAS keeps alternating records; text retains all declared records. See the I/O
guide before supplying input formats, bounds and coordinate metadata.

`conversion-demo` exercises source ECEF/geodetic/UTM formulas, converted PTS,
progress, repeated buffers and nonfinite origin behavior. Conversion remains
explicit. `convert-cloud` takes caller-supplied direction and zone, plus an
explicit inverse hemisphere; see the coordinate guide before using real files.

`pcl-helpers-demo` exercises the source's two unused PointXYZ helpers with generated
data. It preserves float32 voxel centroids, double-centroid translation, metadata
and extent fallback, and reports empty/unsafe and equal-key sort limitations.
Neither helper is added to loading or tracking.

The installed `railway-mls` command supports the same nineteen demos, cloud inspection,
explicit file conversion and both audits. The audit inventories 265 definitions
(124 engine/UI, 77 header, 18 reader, 27 conversion and 19 support boundaries), hashes 41 project and 51 bundled
dependency files, plus 14 frozen PCL extracts and 19 selected PCL/Eigen hashes.
The completed computational scope review records 15 workflows and 29 active/inactive
call-site witnesses. No scoped definition remains pending. All 40 limitation
families remain open; whole-application equivalence and real MLS validation are pending.
Tests use `unittest`; when a C++ compiler is available they also compile temporary
extracts of the original
source for direct behavior comparisons. The stage guides linked above document
API contracts, preserved source quirks, undefined accesses and nonfinite arithmetic.
PCL helper comparisons additionally require the locally bundled Eigen headers;
their Python runtime and demo work without those headers or PCL libraries.

## What stays local

The root `.gitignore` excludes datasets and extracted data, dependency installation
bundles, the alternate project ZIP, compiled libraries/executables, build folders,
IDE caches and user settings, local credentials, and agent state. These files
remain on disk and are not part of the initial commit.

Keep future raw data under `data/` or `datasets/`, and generated results under
`outputs/` or `results/`. Common point-cloud, tabular, array and database formats
are also ignored by extension. Source files, dependency declarations and small
UI images are intentionally tracked. Do not use `git add -f` for local datasets.

The source tree contains its own pre-existing `.git` metadata. That history is
preserved locally and excluded from the outer repository; the outer repository
tracks the selected source files directly, without a submodule. Run this project's
Git commands from the repository root, because commands inside the nested source
repository refer to its separate history.

## Development direction

Continue from the integrated pipeline and completed utility contracts to the
remaining call-path/reconstruction review. Both unused native point helpers and explicit source
coordinate conversion is implemented; units/CRS still come from the caller.
Loop/cross components are implemented explicitly; their automatic scheduling
remains inactive as in the source.
Keep scope gaps separate from the source limitations already reproduced.
Include computational orchestration that currently lives in the GUI. Keep inactive source paths inactive
and preserve defaults, templates, ordering, precision and reproducible quirks.

Synthetic data can support implementation and component checks before database
integration. Real-data experiments and comparisons with the original implementation
will follow when inputs and reference results are available.
