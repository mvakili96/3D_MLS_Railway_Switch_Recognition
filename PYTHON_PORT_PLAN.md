# Python reproduction: repository review and implementation brief

Reviewed on 2026-10-30. This is a source and paper review, not a running implementation or a claim of numerical equivalence. No legacy source was changed, and the Windows application was not built or executed.

## Recommendation

The agreed default is **source-first**: reproduce the existing C++ implementation's computational behavior in Python. Use the paper to explain the method and identify differences, while treating the active source as authoritative for formulas, parameters, templates, control flow and edge-case rules. Establish a source-to-Python specification, then implement and verify each computational stage. Preserve reproducible quirks in the baseline; record proposed corrections separately without applying them silently.

The user has explicitly designated the current unpacked code, as it stands, as the source of truth. Adopt it as the implementation baseline now. Questions about the historical evaluation snapshot are deferred to later experiments and do not block specification or implementation. Do not change the implementation to match the paper's prose or reported metrics.

Real input data is not needed to start the specification or implementation. Synthetic fixtures can test equations, geometry, model selection, tracking, and switch logic. Real clouds, initial conditions, and trusted outputs or annotations are needed later to establish empirical accuracy or legacy agreement.

## Persistent limitations and later review

Maintain [docs/LIMITATIONS.md](docs/LIMITATIONS.md) during every remaining increment.
It consolidates known source limitations, undefined/numerical paths, portability
choices and validation gaps under stable IDs. The user has requested a review of
all entries after full reconstruction. Explicit failure handling and passing tests
do not close an entry. Keep baseline behavior through implementation; record any
later behavior-changing correction as a separate agreed change. Completion review
must revisit every open ID, including LIM-001 (uninitialized toe reference lines).

## Implementation progress

The first Python foundation is implemented in `src/railway_mls/`. It includes the
source defaults, stable point identities/shared labels, geometry, paired seed-window
construction and ordered grid lookup. An installable package and synthetic demo
are available. The source mapping and behavioral contracts for this increment
are in [docs/PYTHON_FOUNDATION.md](docs/PYTHON_FOUNDATION.md).

The custom height GMM, source clustering and active paired rail-head extraction
are also implemented, including the required 100-attempt RANSAC overload, shared
label updates and retry accumulation. Their specification and portability decisions
are in [docs/PYTHON_GMM_HEADS.md](docs/PYTHON_GMM_HEADS.md).

The remaining line-fitting variants, source point growing, and greedy MDL selection
are implemented, including source RMS uncertainty, component
retention/label quirks, initialized-output reuse, and explicit missing-candidate
diagnostics. See [docs/PYTHON_FITTING_MDL.md](docs/PYTHON_FITTING_MDL.md).

Observation and side-point selection, support-derived observation vectors, and
both paired-line/gauge adjustments are implemented. The source's directional
stack cutoffs, signed-distance duplicate lookup, tie behavior, stored line state
and weighted-distance arithmetic are preserved. Undefined three-line ties and
nonfinite calculations are explicit. See
[docs/PYTHON_OBSERVATION_GAUGE.md](docs/PYTHON_OBSERVATION_GAUGE.md).

Full seed initialization and the three-coordinate Kalman prediction/update
components are implemented, including layer appends, initial filter states, fixed
noise constants, direct use of sigma and the bundled matrix inverse's pivot and
singular behavior. See
[docs/PYTHON_KALMAN_INITIALIZATION.md](docs/PYTHON_KALMAN_INITIALIZATION.md).

The sequential tracking loop is implemented, including prediction-window lookup,
separate point/head retry sequences, accumulated head references, MDL/observation
selection, Kalman observations, weighted gauge placement, stop ordering and
predicted turnout-stack snapshots. It retains source identity-covariance behavior
and exposes undefined/nonfinite paths. See
[docs/PYTHON_TRACKING.md](docs/PYTHON_TRACKING.md).

Stack signatures, all 20 source templates, source labels and template recognition
are implemented, including duplicate-pattern ambiguity, zero context weight,
short-stack/branch gates and natural-completion tracking integration. The inactive
raw-count matching helper remains standalone. See
[docs/PYTHON_RECOGNITION.md](docs/PYTHON_RECOGNITION.md).

Toe detection and endpoint refinement are implemented for all twelve supported
source types, with the score/length/main-track postpass gates, shared grouping
labels, support-bound cropping and source projections. The source saves turnout
stack snapshots before assigning member lines, so their later toe-reference reads
are undefined. Python reports that path after preceding grouping/fitting effects;
initialized-reference component fixtures are checked against C++. See
[docs/PYTHON_TOES.md](docs/PYTHON_TOES.md).

Branch scheduling and the UI's bidirectional orchestration are implemented,
including shared engine state, local toe copies, source type filters, Y-as-Z branch
seeds, center-vector tagging, reset-angle float32 narrowing and asymmetric vector
reversal/assembly. See [docs/PYTHON_ORCHESTRATION.md](docs/PYTHON_ORCHESTRATION.md).
LIM-001 remains open: unavailable automatic toe references are not replaced.

Switch-box detection and its plane-fitting dependencies are implemented, including
candidate geometry, the separate `pnpoly2` membership test, ordered two-pass RANSAC
and grouping, bundled symmetric JAMA eigen behavior, source output reuse, class-5
labels, appends and main-tracking integration. Sparse undefined plane reads remain
explicit. See [docs/PYTHON_SWITCH_BOXES.md](docs/PYTHON_SWITCH_BOXES.md).

Grade/curve analysis and both computational UI paths are implemented, including
X/Z division, source marker reuse, membership/report calculations, literal curve
algebra, IEEE results and GWAD 3D chainage/variable-field formatting. Marked-vector
reversal now matches native XYZ-only point assignment; mixed-tag insertion remains
explicitly non-comparable where native capacity can change tag transport. See
[docs/PYTHON_GRADES.md](docs/PYTHON_GRADES.md) and LIM-028–LIM-031.

Verification passed 324 test methods, including 108 methods comparing against
compiled extracts of the original C++. The complete original application has not
been built. Remaining computational inventory, data interfaces and complete
pipeline delivery/audit are pending. Their contracts and checks will accompany
each stage.

## Specification checklist

Turn this audit into an implementation-ready source-to-Python specification. Its concrete deliverables are:

1. A mapping from every active computational function and UI orchestration path to its Python module and entry point, with an explicit disposition for separately invoked and inactive routines.
2. Input/output and state contracts, including point identity, coordinate/angle conventions, configuration defaults, hard-coded thresholds, templates, ordering and random sampling.
3. A package layout and dependency-ordered implementation plan, starting with point/state models, geometry and spatial lookup before classification, fitting and tracking.
4. Acceptance checks for each stage using source-derived expected behavior and synthetic fixtures, with later C++ comparisons and real-data experiments recorded separately.

Use this checklist as each stage is implemented, without requiring input data or decisions about the historical evaluation. The first runnable foundation is now available; later stages still need their detailed behavioral contracts.

## What is in the repository

- The paper is `Multi-Railway_Track_and_Switch_Region_Recognition_Using_Mobile_Laser_Scanning_Data.pdf`, eight pages, by Jaewook Jung, Mohammadjavad Ghorbanalivakili, and Gunho Sohn. Methodology is primarily on PDF pages 2–5.
- The main unpacked project is `ADE code/PCLProject/PCLProject/PCLProject/`. Paths below are relative to that directory unless stated otherwise.
- This is **C++ plus C++/CLI Windows Forms**, rather than C. Native computation, managed UI, rendering, and file handling are coupled.
- `RailwayExtraction.cpp` contains about 5,300 lines. `LIDARPT.h` contains about 1,300 lines, including substantial inline geometry and state logic.
- `MainForm.cpp` contains computational orchestration as well as UI handlers. It must be included in the algorithm inventory.
- `ADE code/Installations/` bundles third-party dependencies. These are infrastructure, not additional original railway algorithms. The review distinguished this material from project-owned computation rather than treating every dependency file as code to translate.
- No project-owned README or automated test suite was found outside dependency/build material.

The legacy project references PCL 1.8, VTK 7.1, Boost, Eigen, FLANN, and libLAS/LASlib, with Windows paths and .NET/CLR configuration (`PCLProject.vcxproj:15–26,61–62`). The existing `CMakeLists.txt` does not establish that the application can build on Linux. Rebuilding the original is a possible later validation aid, not a prerequisite for writing the Python implementation.

### Source variants

`ADE code/VS2017/PCLProject.zip` contains another project version. Use the unpacked project as the primary source reference and retain the ZIP as historical evidence.

The unpacked version adds second seed coordinates, bidirectional tracking orchestration, vector reversal, progress fixes, and richer I/O. Its core MDL and template semantics are otherwise the same as the ZIP's. The archive therefore does not resolve the paper/source disagreements below. Avoid mixing versions without documenting it.

## Computational inventory

| Stage | Main source locations | Python behavior to account for |
| --- | --- | --- |
| Point representation and labels | `PointXYZDouble.h:14`; `LIDARPT.h:324,427,788` | Double precision XYZ, intensity, point identity, mutable class/segment labels, geometry conventions |
| Spatial lookup | `RailwayExtraction.cpp:154,3936` | XY grid, neighborhood selection, window membership, boundary behavior |
| Initialization | `RailwayExtraction.cpp:351`; `LIDARPT.h:814,945` | Explicit seed location, orientation and quadrant/mode; paired rail windows; initial GMM, line fits and gauge adjustment |
| Height classification | `RailwayExtraction.cpp:1042,1172,3323,3459` | Custom three-component height EM-GMM, head selection, previous-height filters and stopping rules |
| Line fitting and grouping | `RailwayExtraction.cpp:2942,3060,3158,3501` | Custom random sampling, fixed-angle alternatives, support sets, connected groups and height estimates |
| Multiple-rail model selection | `RailwayExtraction.cpp:1807,1976` | Competing one/two/three-line models, MDL residual and complexity definitions, ties |
| Observation selection | `RailwayExtraction.cpp:1302,2130,2210` | Main/secondary rail assignment, side occupancy and paired-rail gauge constraints |
| Sequential tracking | `RailwayExtraction.cpp:530,1255,4010,4070` | Prediction/update, orientation, adaptive window extension for sparse regions, termination and intermediate state |
| Segment stacks and switch recognition | `RailwayExtraction.cpp:900–1038,2001,2789`; `LIDARPT.h:1078,1157` | Single/multiple-track stacks, boundary segments, occupancy signatures, templates, context and ambiguity |
| Toes and branch tracks | `RailwayExtraction.cpp:251,2617` | Implemented: toe estimation, additional path seeds, branch direction, layer identity and local/global toe state |
| Bidirectional orchestration | `MainForm.cpp:101–137`; `RailwayExtraction.cpp:5272` | Implemented: reverse-direction run, asymmetric reversal/rail swapping, then original-direction run |
| Switch-box detection | `RailwayExtraction.cpp:2494,3912,4104,4133,4204,4242`; `LIDARPT.h:1056` | Implemented: toe-adjacent candidates, pnpoly2, plane fits/JAMA refit, grouping/gates, labels and engine/layer appends |
| Grade analysis and GWAD | `RailwayExtraction.cpp:5035–5189`; `LIDARPT.h:1254`; `MainForm.cpp:724–800,824–836` | Implemented: grade struct/division/parameters/reports, all-layer menu action, GWAD row computation and finite TSV formatting |
| Loop/cross-region routines | `RailwayExtraction.cpp:2342,2404` | Present in source, but initialization/tracking/validation calls are commented out at approximately lines 512, 827 and 1016 |
| File adapters and coordinate conversion | `LASFileReader.cpp`; `TXTFileReader.cpp`; `DataConvertor.cpp`; `UTM.cpp` | Input density, precision, output schemas and explicit coordinate metadata |

The implementation inventory should cover all source computation, including orchestration inside the UI. Preserve active behavior and explicitly account for separately invoked computational features such as grade analysis and switch-box detection. Record inactive routines without enabling them merely because they appear in the source or paper. Recreating Windows Forms is a separate product task; its computational orchestration must still be ported.

## Paper/source differences and source-first decisions

| Topic | Paper | Source observation | Required decision |
| --- | --- | --- | --- |
| Switch context | Context/unary balance uses weight 0.9, PDF p.5, Eq. 6 and following discussion | `RecognitionTurnoutType`, line 2800, sets `weight = 1`; line 2828 multiplies context by zero | Preserve weight 1 and its disabled context contribution in the baseline; a 0.9 variant is a separately documented experiment |
| Template taxonomy | 12 region templates, PDF pp.4–5 / Fig. 5; eight overall switch categories | `DefineTemplates`, line 2001, defines 20 entries, including extra "both" toe states and variants | Preserve all 20 source templates, their ordering, labels and tie behavior |
| Occupancy encoding | 24 bits: four segments × two rails × three subwindows | Lines 932–952 store 16 side-occupancy bits | Preserve the 16-bit source encoding and score normalization; equivalence analysis is explanatory, not a reason to substitute another encoding |
| MDL residual | Squared residual formulation, PDF p.4 | `ComputeCloseness`, lines 1980–1989, sums absolute distances; squared-distance alternative is commented out | Preserve the active absolute-distance objective and model-selection rules |
| MDL line hypotheses | RANSAC, PDF p.4 | Active calls in `SegmentTrackPtsByMDL` use fixed-angle fitting; free-angle RANSAC alternatives are commented out | Preserve fixed-angle fitting at these call sites and free-angle fitting where it is active elsewhere |
| Kalman uncertainty | Sequential prediction/update framework, PDF pp.3–4 | Lines 586–587 comment out propagation of updated covariance; measurement covariance uses `sigma` directly at line 4070 | Preserve the source recurrence and use of sigma; document conventional-filter corrections separately |
| Window dimensions | 1 m length and 0.20 m subwindows, PDF p.3 / Fig. 2 | Constructor window width is 0.42 m; side-window logic is distributed across other functions | Preserve actual source geometry, membership rules and defaults |

Encoding nuance: if all eight omitted main-window bits are always 1 for both observation and template, dropping them preserves unary ranking. The normalized scores satisfy `U24 = 1/3 + (2/3) * U16`. This still affects the relative influence of context. The paper's explicit split-right/open example reduces to the source's type-0 string under this transformation. Bit length alone is therefore not proof of incorrect recognition.

The paper leaves some implementation details unspecified, including numerical safeguards, portions of noise configuration, EM initialization, and recovery/termination behavior. Specify these from the active source. If the source itself is ambiguous or depends on unavailable runtime behavior, record the uncertainty and seek evidence rather than substituting a textbook algorithm.

### Potential defects and behavior requiring targeted tests

These are static-review findings, not runtime-verified failures:

- `LIDARPT.h:485`: axis-aligned line equations appear to use X where horizontal-line Y is expected, and Y where vertical-line X is expected.
- `RailwayExtraction.cpp:303`: branch seed XYZ is constructed as `(RightPt.x, RightPt.y, RightPt.y)`. Verify its actual downstream effect; initialization later refits heights.
- `RailwayExtraction.cpp:168–169`: grid assignment floors displacement before dividing by grid size; the default size of 1 masks differences from ordinary cell indexing.
- `SegmentTrackPtsByMDL` sends equal-score cases to the three-line fallback. Preserve and test this rule wherever behavior is well-defined; investigate invalid-model cases separately.
- GMM normalization and fit/support calculations contain sparse or degenerate cases that can divide by zero or consume invalid outputs. Characterize reference behavior where possible. Any finite-value guard or structured failure that changes observable behavior must be documented as a departure.
- Random fits use C `rand()`. A Python random seed alone does not guarantee the same samples or fitted models as C++. Compare fixed sample sequences or deterministic fixtures when testing individual fits.
- Point classification is mutated through shared point pointers. Preserve point identity with indices and shared label arrays in Python; independent copies can alter later processing.
- Angle/quadrant conventions are distributed across geometry and tracking. Test cardinal directions, all quadrants, reversed travel and left/right swapping before simplifying them.

Suspected bugs are investigation items, not authorization to change the baseline. Preserve well-defined, reproducible behavior even when it appears mathematically unusual. C++ undefined behavior has no portable equivalence target: document affected inputs and any necessary Python handling explicitly. Keep proposed fixes in a separate decision log and apply behavior-changing corrections only under an explicit scope change. Structural refactoring is appropriate when it preserves behavior.

## Data contract to prepare now

Start with an in-memory API that accepts XYZ and intensity, configuration, and a reproducible initial pose. Keep loading, database access, visualization and export outside the computational core. Record coordinate units and frame/CRS; preserve double precision internally.

Known legacy behavior matters when real data arrives:

- `LASFileReader.cpp:31–46,73–97` retains alternating input points. PTS reading retains all points. Format-dependent density can affect thresholds; make subsampling explicit.
- PTS uses a first-line count followed by `x y z intensity` (`TXTFileReader.cpp:37–52`). The conversion CSV reader also expects a count header, then comma-delimited fields (`:144–161`). Ordinary database CSV should have its own adapter.
- Loaded point class/segment fields are initialized rather than preserved from LAS metadata. Decide which source attributes the new API must retain.
- Existing exports round coordinates to three decimals (`TXTFileReader.cpp:191–267`). Keep full precision in diagnostic and numerical comparisons.
- Coordinate conversion is a separate workflow. The UTM/ECEF dialog has user-supplied zone handling and an unconditional hemisphere argument in an active path (`DataConverstionForm.cpp:231`); future adapters must specify zone and hemisphere explicitly.
- The bundled `ML_23_Line_2_with_track_points.csv` contains 462 extracted output rows: 154 center points and 308 rail points. It can inform export tests; it is not a raw MLS input or a complete reproduction benchmark.

Do not assume raw point clouds will already contain only rail points. The method classifies ground, web and head points inside local windows. Synthetic scenes should include those height populations and relevant noise/occlusion.

## Implementation sequence and completion criteria

| Phase | Deliverable | Evidence needed before progressing |
| --- | --- | --- |
| 1. Freeze the specification | Source-to-Python map with paper cross-references, full parameter inventory, frozen source baseline, discrepancy log, feature scope and validation plan | Every active computational path has a planned module; inactive features and UI orchestration are accounted for; the current unpacked source is authoritative |
| 2. Establish the core | Installable Python package, configuration, point/state models, geometry, spatial lookup and seed API | Analytic geometry, cell/window boundaries, precision, all direction modes, point identity and initialization tests |
| 3. Implement local inference | Source GMM behavior, head filtering, line fitting, grouping, MDL and observation selection | Fixtures derived from source behavior and independent calculations; compare reference support sets, parameters and model choices where available |
| 4. Implement tracking | Kalman recurrence, gauge constraints, adaptive windows, stopping, bidirectional orchestration and scheduling from supplied branch seeds | Straight and curved synthetic paths, elevation changes, sparse windows, gaps, reverse travel and supplied-seed branching; finite, inspectable intermediate outputs |
| 5. Implement recognition and source features | Stack transitions, all source templates, source scoring with context disabled by default, toe refinement, automatic toe-to-branch integration, and separately invoked source computation | All template entries, noisy occupancy, disabled-context behavior, ties/ambiguity, short stacks, detected-toe branch initialization and feature-specific checks |
| 6. Deliver a runnable synthetic pipeline | Public API, CLI, deterministic synthetic scene generator, result schemas, diagnostic export and usage documentation | A documented command runs every core stage without the database; tests verify the pipeline is doing meaningful extraction and recognition |
| 7. Validate with real data | Real-data adapters, matched configurations and seeds, stage comparisons, accuracy and performance report | Reference or annotated clouds; documented numerical tolerances; discrepancies investigated; measured runtime/memory |

Phases 1–6 can proceed without database integration. Maintain a small persistent progress document so work can continue across sessions. Each phase should produce reviewable artifacts and meaningful verification; repeated approval requests are unnecessary when decisions fall within the agreed specification.

Dependency selection belongs in phase 1. Prefer established numerical/spatial tools where their behavior matches the specification. Do not replace the custom EM, model selection or sampling logic with a library default without checking initialization, convergence, residuals and failure behavior. Avoid translating nested per-point C++ loops directly into Python where indexed or array operations can preserve the same mathematics.

### Validation levels

1. **Source behavior at component level:** small independently calculated results for the actual source formulas, thresholds, update rules and ties. Mathematical checks should expose unusual behavior without silently correcting it.
2. **Synthetic integration:** controlled straight, curved, turnout, noisy and occluded scenes; distinguish intended geometry from what the source actually produces and investigate differences.
3. **Legacy comparison:** the primary evidence for a faithful port. When a reference executable or isolated routines can be run, compare matched intermediate results using recorded configuration, precision, point order and random samples, with justified tolerances. Mark untested equivalence explicitly when the reference cannot run.
4. **Empirical reproduction:** use real annotated clouds to measure the paper's metrics. This is separate from passing synthetic tests.

The paper reports 99.46% rail recall and 97% region-recognition success on private MLS evaluation data (PDF pp.6–7), including 100 additional multi-track evaluation regions for recognition. Synthetic tests cannot establish those results. Even a complete Python implementation must label empirical reproduction as pending until suitable data is available.

## Suggested request to start the implementation

> Port the existing C++ railway tracking and switch-region recognition implementation to Python, prioritizing preservation of its computational behavior. Use `PYTHON_PORT_PLAN.md` as the starting audit and the unpacked `ADE code/PCLProject/PCLProject/PCLProject/` project as the baseline. Use the PDF for explanation and cross-reference; when it differs from active code, follow the code.
>
> First freeze the source baseline and turn the audit into a detailed implementation specification: map source functions and call paths to planned Python modules, cross-reference paper sections/equations, and inventory all parameters, hard-coded thresholds, templates and active/inactive behavior. Record paper/source differences without changing source semantics. Preserve reproducible quirks and keep proposed fixes separate. Account for bidirectional tracking in `MainForm.cpp`, switch-box detection, separately invoked grade analysis and inactive loop/cross routines. Do not activate commented-out behavior. Treat the current unpacked code as authoritative and defer questions about the historical evaluation to later experiments.
>
> Then implement the agreed pipeline in stages, with a reusable Python API, configuration, CLI, intermediate diagnostics and meaningful tests. Separate computation from file/database adapters and visualization while preserving algorithmically relevant I/O behavior such as subsampling. Do not require real data or database access yet: create deterministic synthetic point clouds with ground/web/head populations and known straight, curved, branching, noisy and occluded cases. Cover initialization, classification, line fitting, grouping, MDL, observation selection, Kalman tracking, gauge constraints, segment stacks, all 20 source templates and their scoring, toes, branches and both travel directions.
>
> Preserve coordinate precision, point identity, ordering where relevant, random-sampling behavior and source direction conventions. Do not silently substitute generic library defaults, fix suspected bugs or alter the method. Document any unavoidable handling of undefined behavior. Continue autonomously through routine implementation decisions; ask when unresolved source ambiguity or a proposed correction would materially change observable behavior. Keep the specification, discrepancy log and progress state current across sessions.
>
> Deliver an installable codebase, passing component and synthetic integration checks, a documented end-to-end synthetic example, and an honest validation report. Compare intermediate and final results against the C++ executable or isolated reference routines wherever practical; identify equivalence claims that remain unverified. Distinguish implementation completion, agreement with the frozen source baseline, and reproduction of published accuracy. Real-data validation and performance measurement will follow when I provide the data.

For a shorter first task, request only phase 1: “Turn this audit into the source-to-Python behavior specification, frozen baseline, discrepancy log, module design and acceptance criteria. Treat active C++ behavior as authoritative and do not implement yet.” A paper-aligned or corrected variant can be considered later as an explicitly separate change after the source-faithful baseline is established.
