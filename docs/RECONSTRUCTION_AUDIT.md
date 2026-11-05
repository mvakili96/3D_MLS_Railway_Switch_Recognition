# Computational reconstruction audit

The agreed computational reconstruction of the frozen unpacked C++ project is
implemented and its scope/call-path review is complete. This includes active
algorithm paths, standalone uncommented helpers, numerical input boundaries,
legacy readers/exports and explicit coordinate conversion. Undefined native
results remain diagnosed rather than replaced by invented geometry. No original
source file or frozen baseline hash was changed.

This conclusion concerns implementation scope. Whole-application equivalence,
real MLS performance, published-result reproduction and the review of retained
limitations remain open. `complete_reconstruction: true` has that scoped meaning;
the reports separately retain `whole_application_parity_verified: false`,
`real_data_validation_complete: false` and `limitations_review_complete: false`.
It does not promise that every input completes every algorithm stage.

## Review evidence and checks

[reconstruction_audit.json](../src/railway_mls/reconstruction_audit.json) records
file decisions for all **41** project .cpp/.h files, **15** computational workflows,
their Python/test references, **29** active/inactive call-site witnesses and a
lexical token inventory for **265** selected definitions. Each definition retains
its disposition in [source_coverage.json](../src/railway_mls/source_coverage.json):

| Scope | Definitions |
| --- | ---: |
| RailwayExtraction.cpp and MainForm.cpp | 124 |
| LIDARPT.h uncommented inline definitions | 77 |
| Text/LAS readers, exports and PCL helpers | 18 |
| Converter, UTM functions and conversion form | 27 |
| PointXYZDouble, HashMap and numerical input forms | 19 |

The 19 named LIDARPT types are also mapped. Original source hashes cover all 41
project and 51 bundled numerical dependency files. Fourteen selected PCL extracts,
19 selected PCL/Eigen hashes and the original PCL notices are frozen separately.
Whole PCL/VTK/Boost/Jama libraries are not claimed ported.

```bash
python -m railway_mls reconstruction-audit --repo .
python -m railway_mls source-audit --repo .
python -m unittest discover -s tests -v
```

Both audits are read-only and reject source/resource/inventory drift, unavailable
Python targets, missing review test files, inconsistent limitation IDs, changed
lexical call counts and missing/unexpected reviewed call sites. They do not run
the tests. An installed wheel contains both manifests; auditing still requires
`--repo` pointing to the original checkout including tests and docs. Runtime
tracking and demos do not need the original C++/SDK files.

The scanner blanks comments/strings and bounds bodies with balanced braces. Its
tokens are inspectable call-site evidence, not a resolved C++ call graph. It does
not resolve overloads, macros, templates, indirect/virtual calls or conditional
runtime reachability, and external method names can collide with project names.
File hashes also detect changes outside the selected definition inventory.

## Findings implemented in this increment

**Source parameter updates.** `ExtractionParameters` models the source's ten
double input fields. `ExtractionConfig.with_source_parameters` retains grid/skip
settings and truncates the four point-count/intensity members toward zero before
storing them as integers. The source getter promotes them back to doubles.
Invalid native float-to-int conversions raise an explicit error. Direct Python
configuration construction keeps its existing interface.
Negative point-count thresholds also retain C++ unsigned comparison behavior:
for supported int-indexed clouds they trigger retries and never pass a strict
retry-success gate. Both window and head thresholds are compared against the
original tracking loop, including RNG consumption and partial side effects.

```python
from dataclasses import replace
from railway_mls import ExtractionConfig

config = ExtractionConfig()
inputs = replace(config.to_source_parameters(), min_track_points=5.9)
updated = config.with_source_parameters(inputs)  # min_track_points is 5
```

**Scalar point helpers.** `PointXYZDouble` exposes the original default/XYZ/copy
values, homogeneous coordinate 1, metadata copy, strict 1e-5 equality and exact
null test. Approximate scalar equality never replaces cloud row/pointer identity.
`Point3D` continues to model the separate tagged POINT3D type. C++ memory layout
and mutable union padding are outside the Python value interface.

**Seed dialog defaults.** `initial_location_dialog_defaults(location)` explicitly
models the active form overload: truncate and wrap the angle to 0..359, reset an
invalid mode to 1 and return primary XYZ/angle/mode only. For example, -45.9
becomes 315. Second coordinates remain unavailable. This adapter is optional;
direct seeds and reverse/branch calls keep their existing source math.

The scalar point methods, parameter getter/setter and active form normalization
are compared with compiled original bodies. Only PCL declarations/alignment,
Windows numeric controls and XYZ text assignments are replaced by documented
test shims. Undefined casts and uninitialized fields are never executed.

## Call-path decisions

| Path | Reviewed behavior and evidence |
| --- | --- |
| Loading and membership | Source record order/sampling, reader bounds, exact-key cache and per-file engine lifetime; no automatic downsampling or CRS conversion. Reader and foundation comparisons. |
| Seed initialization | World-coordinate picks and wrapper float narrowing; paired GMMs/fits, fixed seed windows, endpoint/gauge updates, saved heights/layers and failure side effects. Initialization and pipeline comparisons. |
| GMM, heads, fitting and MDL | Native RNG draws and custom EM; all fitter overloads, growing groups, candidate flags/ties and shared row labels. Dedicated native component comparisons. |
| Observation, gauge and Kalman | Directional sorting/side points, local gauge adjustment, support ordering, cache update order, bundled matrix/LU and covariance behavior. Dedicated native comparisons. |
| Sequential tracking | Window/head retries, strict gates, turnout snapshots, vector writes before stop checks, accepted segments and terminal postpasses. Original-loop comparisons plus raw synthetic integrations. |
| Recognition, toes and boxes | Literal templates/ambiguities, stack/score gates, toe type families and planes/box labeling. Initialized component fixtures; missing tracker reference lines remain LIM-001. |
| Branches and both directions | Source filter/layer order, Y-for-Z branch seed, global state, reverse-first UI scheduling, XYZ-only vector reversal and final tags. Controlled original-wrapper callbacks plus separate raw integration. |
| Grades and exports | Separate grade action; GWAD invokes grades explicitly. Source curve algebra, tags, row order and all vector formats. Native grade/export comparisons. |
| Conversions | Scalar projection formulas, units/constants, persistent buffers, callbacks, batch/menu bindings and source inverse-menu northern setting. Converter comparisons and generated file tests. |
| Standalone methods | Header/scalar utilities, paired wrappers, inactive loop/cross routines and two PCL helpers are callable independently. They remain absent from automatic source call sites. |

Windows Forms controls, progress, layer names, cameras, display colors, startup,
FPS and sample viewer geometry are explicitly excluded. VTK translates displayed
picks back to cloud/world coordinates before the numerical callback; Python picks
must already be in that coordinate system. Numeric Python inputs replace .NET
locale-sensitive text formatting/parsing. The unused XY form overload and two
block-commented rotation helpers do not enter the computational pipeline.

## Validation limits and next work

The full Python suite includes original C++ extracts, generated LAS/LAZ and text
fixtures, deterministic synthetic integrations and audit-drift checks. C++ checks
skip when the compiler/source is absent; PCL comparisons additionally need the
locally bundled Eigen headers, and LAS/LAZ tests require the optional codecs.
Those prerequisites were available for this review's validation. Scheduling
oracles use controlled callbacks; they do not execute the entire native pipeline.

Validation for this increment passed **532 test methods**, including **206**
methods using compiled source extracts, with no skips. A wheel installed outside
the repository passed **33 CLI checks**, the new public input adapters and all
four packaged resources. Ruff and whitespace checks passed. These are component,
integration and packaging results; they do not change the validation limits above.

All **40** [limitation families](LIMITATIONS.md) remain open, including the
numerical-boundary/review-evidence entry LIM-039 and the later interactive-viewer
entry LIM-040. The computational audit itself concluded with 39 entries;
the separate [desktop viewer](PYTHON_VIEWER.md) adds delivery scope. Its optional
fitted-reference recovery enables a real-data branch demo while preserving the
strict core and the open LIM-001 finding. The audit completes scope work
on LIM-015 but does not close its validation concerns or any other ID. Next,
review every limitation with the user and record decisions to retain, correct
separately, investigate or defer. Then select real MLS data, coordinate units/CRS
and experiment metrics. Any correction needs a separate behavior change while
preserving this source-based comparison baseline.

To refresh review artifacts after an explicitly reviewed change:

```bash
python tools/update_source_coverage.py
python tools/update_reconstruction_audit.py
python tools/update_source_coverage.py
python -m railway_mls reconstruction-audit --repo .
```

The review generator refuses a changed native baseline until its scope decisions
are explicitly revised. Regeneration itself is not a semantic review or test run.
