# Integrated pipeline, diagnostics and source audit

The runner composes the existing source-first engine; it introduces no replacement
tracking equations, template scores or synthetic repairs. It shares one owning
cloud, grid and continuous random stream across initialization, directional and
branch passes, recognition, toes and boxes. Grade analysis and GWAD remain separate
opt-in actions. The original C++ files are unchanged.

## Run without input data

```bash
python -m railway_mls pipeline-demo
python -m railway_mls pipeline-demo --scenario turnout
python -m railway_mls pipeline-demo --scenario components --output outputs/components-001
python -m railway_mls pipeline-demo --scenario missing-references
python -m railway_mls pipeline-demo --max-steps 1 --steps --record-random
python -m railway_mls source-audit --repo .
```

The installed `railway-mls` command accepts the same arguments. The pipeline demo
enables grade and GWAD actions; `--skip-grades` skips both. Each scenario has an
explicit default direction, overridable with `--single-direction` or
`--bidirectional`. No input adapter, database or original application is required
for these demos. `source-audit` requires the original source checkout.

| Scenario | Inputs and default direction | Observed baseline result, seed 1 |
| --- | --- | --- |
| `straight` | Raw ground/web/head corridor, interior seed; bidirectional | Seven reverse and six forward accepted steps, assembled vectors, grades/GWAD; no supplied stacks/toes. Single-rail MDL policy is explicit (LIM-002). |
| `turnout` | Raw corridor with close side rails over part of its length; single direction | Eleven accepted steps and one completed seven-segment stack. Recognition returns type 999/score 0.5; toe detection is gated by score. No type or line reference is injected. |
| `components` | Two separated raw corridors, one supplied branch toe, one supplied six-segment side-membership stack with initialized references, dense box patches; single direction | Main and branch inference accept six/five steps. Recognition yields type 6, one additional toe is computed, and two boxes are detected. The additional sparse box candidate exposes LIM-025. A type-filtered empty layer is retained. |
| `missing-references` | Same supplied state as `components`, with stack reference lines absent; single direction | Toe fitting consumes its draws, then returns `undefined_source_reference_line` (LIM-001). No computed toe is appended; the supplied branch toe still drives its branch and two boxes. |

Component fixtures are spatially separated tests, not a physically joined railway
junction or evidence of automatic switch reconstruction. The manifest records
supplied toe inputs, stack signatures, initialized-reference counts and scenario
notes. Raw scenes can legitimately yield ambiguity, gated postpasses or source
limitations. Success does not mean every optional detection produced an object.

## In-memory API and ownership

```python
from railway_mls import (
    InitialLocation, PipelineOptions, Point3D, PointCloud,
    pipeline_summary, run_pipeline, write_pipeline_outputs,
)

cloud = PointCloud(xyz, intensity=intensity)  # owned float64 XYZ, shared labels
seed = InitialLocation(Point3D(seed_x, seed_y, seed_z), angle_degrees, mode)
result = run_pipeline(
    cloud, seed,
    options=PipelineOptions(
        bidirectional=True, grades=True, gwad=True, random_seed=1,
        coordinate_units="m", crs=known_crs,
    ),
)
report = pipeline_summary(result)
manifest = write_pipeline_outputs(result, "outputs/run-001")
```

`xyz`, intensity, seed and coordinate metadata are caller-provided here; this is
an input contract, not a data loader. `coordinate_units` and `crs` default to
`None`: they are recorded without inference or conversion. Native thresholds stay
unchanged. Empty clouds require explicit `bounds=Bounds3D(...)`.

`run_pipeline` creates a fresh engine and mutates the supplied cloud's class and
segment labels in source order. It does not clone, thin, reorder or reset the
input. Engine pools persist across that workflow's directions and branches, as
documented in [the orchestration guide](PYTHON_ORCHESTRATION.md). Results retain
live engine/layer references; call `pipeline_summary` to snapshot the current
state. A separate fresh PointCloud is needed to repeat from the same initial
labels. Do not reuse bounded engine passes as resumable runs (LIM-024).

`supplied_toes` and `supplied_stacks` explicitly seed engine pools for controlled
tests or caller-owned prior state. Toe values are copied; supplied stacks retain
their source-style mutable signature state. Every supplied stack membership must
refer to the owning pipeline cloud. Missing references remain absent.

`initial_location_from_picks(first, second)` ports the computational prefix of
`MainForm::ReceivedPoints`: XYZ midpoint, folded source angle narrowed to float32,
and direction mode, including coincident/axis cases. It excludes picking UI,
dialogs and subsequent initialization. Its output is an input seed; the fitted
second rail is established by initialization.

### Status and ordering

The stage log records grid, initial initialization, tracking/orchestration,
explicit grade action and GWAD action. Per-invocation results record model status
counts, completed/active stacks, recognition scores, toe outcomes, box outcomes,
branch seeds/filters and tags. Bidirectional reset seeds, initialization outcomes
and reversal diagnostics are preserved separately. New automatic stacks never
gain invented member lines (LIM-001).

The runner validates finite explicit seed XYZ/angle and integer mode, then stops
if its first initialization is unsuccessful. This is a new runner boundary:
the native GUI does not consistently guard initialization booleans. The literal
bidirectional forward-initialization behavior remains inside TrackingEngine,
including continuation after a defined false result. No fallback initialization
or source algorithm correction is inserted by the runner.

Grade/GWAD actions defer after an undefined, unsuccessful or step-limited workflow.
The API defaults both actions off. `gwad=True` still computes grades internally
even if `grades=False`; requesting both performs the native grade action and then
the export's repeated grade call, retaining tag side effects (LIM-028/LIM-030).
Export layer names are deterministic `layer-0`, `layer-1`, etc.; custom native UI
names can be supplied separately to `build_gwad_rows`.

- `completed`: requested workflow/actions finished with no collected limitation
  flags. It is not a numerical equivalence or accuracy assertion.
- `completed_with_limitations`: workflow/actions finished with findings or
  non-comparable component flags. These are inspectable partial/portable results.
- Other statuses retain the source/engine stop, or `initialization_failed`.
  `max_steps` bounds each invocation and defers dependent work.
- `reference_comparable` propagates known execution flags. It does not establish
  full-binary agreement, published performance or complete source coverage.

CLI exits: 0 for either completed status, 2 for `step_limit`, 1 for other pipeline
failures or source-audit drift. Argument/filesystem errors use argparse exit 2.
The earlier individual demos retain their existing exit behavior.

## Diagnostic artifacts, schema 1

Stdout contains strict JSON. `--output` creates a **new** directory containing:

- `manifest.json`: configuration, input seed, stage/result statuses, per-run
  findings and limitation IDs, source coverage, scenario context, runtime versions,
  input hashes, supplied-state signatures, random state/counts and artifact hashes.
- `vectors.csv`: final rail1/rail2/center XYZ, layer/vertex indices and point tags.
- `toes.csv`: global toe values, type, global used state and named geometry.
  Local branch used markers are retained separately in the manifest.
- `switch_boxes.csv`: global box centers, angles and optional reference heights.
  Full box vertices and layer associations are retained in the manifest.
- `grades.csv`: section endpoints and all scalar source grade/curve parameters.
- `gwad.tsv`, only when requested and reached: existing source row/text behavior,
  including variable columns, six-decimal finite formatting and repeated tagging.

Existing directories/files are rejected without overwriting. Files written before
an unexpected filesystem failure may remain in the newly created directory; the
manifest is written last. Diagnostic CSV uses Python's full float representation,
empty fields for unavailable values and `nan`/`inf` text for IEEE values. These are
new diagnostic schemas, not replacements for pending legacy TXT/vector exporters.
No complete raw cloud, intensity or label array is exported.

JSON represents unavailable native fields as ordinary `null`. For IEEE values it
adds a `nonfinite_values` array containing JSON-pointer paths and `NaN`,
`+Infinity` or `-Infinity`, while replacing the numeric position with `null`.
This preserves the distinction between an absent value and nonfinite arithmetic.
The algorithm's in-memory floating-point values remain untouched.

`--steps` includes ordered IDs, memberships and filter/model state; it can be large.
`--record-random` includes all raw draws. Input hashes cover canonical little-endian
XYZ/intensity/initial labels in row order; supplied stack signatures include
ordered membership hashes and geometry/caches. Hashing uses bounded row chunks.
These diagnostics support reproducing experiments but are not a serialized engine
checkpoint or a complete input-data archive. RNG metadata identifies supplied
generators; a seed/state is only asserted when known. Inputs/state and runtime are
still required for reproduction. MLS-scale runtime/memory remains unprofiled.

## Explicit coverage audit

[source_coverage.json](../src/railway_mls/source_coverage.json) is packaged in the
wheel. It records **124** out-of-class definitions in RailwayExtraction.cpp and
MainForm.cpp, with owner/name/overload, signature, source line, disposition,
Python target(s) and scope notes. All **41** top-level project .cpp/.h files and
**51** bundled Jama/etc .cpp/.h/.inl files have normalized-newline SHA-256 hashes.
Dependency hashes freeze the inputs; only reached numerical library behavior is
ported. State/header routines are mapped by the individual stage guides rather
than counted as separately audited definitions. This is a scoped lexical
inventory, not a C++ parser, exhaustive call graph or numerical coverage percentage.

`source-audit --repo PATH` checks file/dependency inventory and hashes, selected
definition IDs/signatures, explicit dispositions and resolvable Python targets.
It rejects drift instead of updating the baseline during a run. Regenerate only
after reviewing a change, using `python tools/update_source_coverage.py` from the
checkout; unknown definitions raise until explicitly classified. A successful
audit establishes inventory consistency, not correctness of every mapping or
parity with the Windows executable. `complete_reconstruction` remains false.

The standalone/partial utility gaps from the initial audit are now implemented:
paired GMM, previous-height erasure, bounds, cycling, inner angle, stored-vector
3D distance, endpoint-only boundaries and inactive intersection. See
[the utility guide](PYTHON_UTILITIES.md). No selected definition remains
`unused_helper_pending` or `ported_reached_behavior`.

The audit still leaves these gaps visible:

- Inactive `DetectLoopCross` and `ValidateLoopCross` workflows; they are recorded
  without enabling their commented-out scheduling. Their intersection helper,
  raw-count matcher and fixed-angle loop/cross fitter are standalone ports.
- Legacy reader/bounds adapters, subsampling, coordinate conversion and legacy
  point/vector export actions (LIM-014). Numerical UI actions are mapped separately
  from display/dialog callbacks; Windows Forms/VTK presentation is out of scope.

These are scope gaps, distinct from undefined-source behavior in implemented
components. The [limitations register](LIMITATIONS.md) remains open for the agreed
post-reconstruction review. Real inputs, a usable native reference and profiling
are still needed for evaluation.

## Validation

The integrated single/bidirectional runs are compared with direct existing-engine
composition for random consumption/state, labels, vectors/tags and grade/GWAD
results. Controlled state exercises recognition, toe refinement, branch inference,
box detection and the missing-reference path. Tests also check bounded/failed
stage deferral, draw-stream replay, pre-mutation hashes, JSON/CSV round trips,
nonfinite annotations, non-overwriting delivery and source-inventory drift.
ReceivedPoints' midpoint/angle/mode prefix is compared with actual compiled native
source across quadrants, axes and coincident picks. This adds component evidence;
it does not close LIM-011 or LIM-012.
