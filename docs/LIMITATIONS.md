# Limitations register and post-reconstruction review

Last consolidated: **2026-11-01**. Baseline: the current unpacked C++ source.

This is the persistent register for known source limitations, Python portability
choices, and validation gaps. The user has requested that they be retained for
review after full reconstruction. All entries below remain **open for that review**.
An explicit failure status or passing regression test does not resolve an entry.
Documented source behavior remains in the baseline; corrections require a separate
agreed change and their own comparisons.

## Maintenance and review checkpoint

- Update this register during every remaining implementation increment. Append
  stable IDs; keep existing IDs when an issue is clarified or resolved.
- For new findings, record the source location or stage-guide evidence, trigger,
  impact, current handling, and the evidence needed for a later decision.
- Keep unresolved entries visible through merges. A source-faithful increment can
  be merged while its documented limitations remain open.
- After the agreed computational reconstruction is complete, review **every ID**
  with the user before declaring the limitations work finished. Record a decision
  to retain, correct separately, investigate further, or defer with a reason.
- A later correction must retain the original behavior as a comparison baseline
  and identify which outputs, sampling, ordering or stopping rules change.
- Real-data and published-result claims require their own evidence; component
  agreement alone does not close those validation entries.

## Source undefined behavior and numerical limitations

### LIM-001: Uninitialized toe reference lines

**Evidence:** `RailwayExtraction.cpp:709–713,801–802,2617–2774` and
[the toe guide](PYTHON_TOES.md). Tracking copies the predicted `nextSeg` into the
turnout stack before assigning its member lines. The native `Line3D` constructor
does not initialize those values. Later toe detection reads the stacked lines.

**Impact:** automatic toe geometry, and consequently later toe-based branch seeds,
cannot be assigned a portable result for these snapshots. Initialized-reference
fixtures validate the component formulas, not successful recovery of those lines.
The stacked distance-from-start is also uninitialized; future consumers must retain
that distinction from accepted-segment state.

**Current handling:** stacked member lines/distance remain `None`. If earlier
grouping/fitting succeeds, detection reports `undefined_source_reference_line`,
retaining label mutations, fit/crop diagnostics and consumed draws. An earlier
undefined fit is reported first. No successful toe is appended for this path.

**Post-reconstruction review:** examine snapshot/write order and required reference
geometry, reproduce affected cases with recorded inputs, and decide any separate
corrected behavior. Check toe positions, angles/modes, branch initialization and
engine/layer side effects together. **Status: open; review first.**

| ID and trigger | Impact/current baseline handling | Post-reconstruction review and evidence |
| --- | --- | --- |
| **LIM-002 — Missing MDL candidates.** Native scoring/length checks can read lines never initialized, commonly on single/two-line inputs. | Explicit source penalties can yield a usable Python selection with `source_uninitialized_intermediates`, but `reference_comparable=False`. Missing selected geometry produces a structured failure. | Audit all candidate-count/flag paths with recorded samples and matched stage outputs. Do not treat default single-rail demo success as proof of portable native execution. [Details](PYTHON_FITTING_MDL.md#missing-candidates-and-native-undefined-reads). |
| **LIM-003 — Failed fits consumed by callers.** Initialization, head extraction, tracking and toes can ignore a fit failure and access unavailable line/support data. | `undefined_source_*` results expose the failure; earlier point-label/grouping effects and raw draws are retained where defined. | Exercise too-few points, no valid sampled model, duplicate samples and empty support in each caller. Decide recovery behavior separately. [Fitting](PYTHON_FITTING_MDL.md), [heads](PYTHON_GMM_HEADS.md), [initialization](PYTHON_KALMAN_INITIALIZATION.md), [toes](PYTHON_TOES.md). |
| **LIM-004 — Three-candidate observation minimum ties.** No strict winner initializes the other local observation line. | Defined point vectors are retained; the unsafe discarded adjustment is skipped and the tracker reports the undefined observation path. | Review equal gauge-error cases and downstream fit behavior. Compare the defined native selection prefix separately from undefined later reads. [Details](PYTHON_OBSERVATION_GAUGE.md#undefined-and-nonfinite-cases). |
| **LIM-005 — GMM degeneracy, underflow and partial classification.** Zero/nonfinite parameters and all-zero/NaN density scores are possible. | No regularization is added. Last-match ties, classification-prefix returns, source stopping rules and explicit initialization empty-window behavior are preserved. Three components do not guarantee three semantic populations. | Evaluate sparse/flat/noisy windows and large height ranges; compare parameters, class prefixes and later extraction outcomes before considering safeguards. [Details](PYTHON_GMM_HEADS.md). |
| **LIM-006 — Degenerate gauge/projection arithmetic.** Zero basic gap, uniform weighted errors, empty support or explicit zero projection coefficients can yield NaN/Inf. | IEEE results and source return/append behavior are retained. Tracking stops before a subsequent unsafe nonfinite grid conversion. Nonfinite toe completion can still have native return true. | Review zero/near-zero denominators, weight normalization, projected versus final gauge and all prior side effects. Any fallback must be a documented variant. [Gauge](PYTHON_OBSERVATION_GAUGE.md), [tracking](PYTHON_TRACKING.md), [toes](PYTHON_TOES.md). |
| **LIM-007 — Kalman coordinate and inverse behavior.** Zero X/Y can divide by zero; covariance depends on absolute coordinates; bundled LU has unusual pivot and singular rules. | Source dense arithmetic is retained, including zero inverse with invalid flag, `1e-20` pivots and nonfinite propagation. Measurement noise uses sigma directly; the tracker keeps identity current covariance because posterior propagation is commented out. | Compare coordinate-frame shifts, singular/full covariance inputs and a separately proposed conventional recurrence against the preserved source equations. [Details](PYTHON_KALMAN_INITIALIZATION.md). |

## Portability, validation and delivery gaps

| ID and scope | Current evidence/handling | Required later review |
| --- | --- | --- |
| **LIM-008 — Equal-key sorting.** Native `std::sort` does not specify equal-height or equal-component-size order. | Python uses documented stable ties. Native exact-order comparisons use appropriate distinct-key fixtures; tied membership/height checks have narrower claims. | Compare the target compiler/runtime's permutations and resulting random-index selections, grouping choices and outputs. [Heads](PYTHON_GMM_HEADS.md#random-draws-and-ordering), [grouping](PYTHON_FITTING_MDL.md#grouping-and-point-identity). |
| **LIM-009 — Random state and floating-point environment.** Unknown earlier native draws, point order and arithmetic near thresholds affect model choices. | Owned MSVC recurrence/default seed and recorded raw streams are available. Component extracts receive identical draws; a seed alone does not recover an unknown original session. | Save complete configuration, cloud order and draw streams for matched experiments. Investigate threshold/tie discrepancies with explicit numerical tolerances and exact identity comparisons. [Details](PYTHON_GMM_HEADS.md#random-draws-and-ordering). |
| **LIM-010 — Python boundaries and native uninitialized fields.** Some invalid counts, ownership, dimensions or nonfinite query inputs have no supported native result. Default geometry fields and native output-reference interfaces need explicit contracts. | Python rejects invalid API inputs or exposes unavailable fields as `None`. Endpoint-only boundary caches now remain absent in initialization/tracking; native comparisons read only defined endpoints/tags. New scalar 3D distance requires an explicit stored vector, and returned utility vectors do not expose aliased native output buffers. | Audit every newly implemented consumer before reading absent fields or assuming native reference aliasing. Retain explicit boundary contracts and identify comparisons that exclude undefined native values. [Utilities](PYTHON_UTILITIES.md), [initialization](PYTHON_KALMAN_INITIALIZATION.md), [toes](PYTHON_TOES.md). |
| **LIM-011 — Whole-application equivalence.** Original Windows application has not been built/executed for full comparisons. | Current suite: 395 test methods, including 140 using compiled source extracts. Fifteen demos and a source audit are available. Integrated workflows match direct engine composition; inline point/line/neighbor/state contracts are source-tested. Grade/GWAD and switch-box cases use original routines/JAMA; orchestration callbacks isolate control flow. Unsafe native reads are excluded. | Complete call-path/reference review, then compare matched whole-path intermediate/final results where a usable reference exists. Keep component, controlled-callback and full-binary claims separate. |
| **LIM-012 — Real-data/published evaluation.** Suitable raw inputs, annotations and reference outputs are not yet supplied. | Synthetic fixtures validate specified behavior only; published accuracy/recall and robustness are not reproduced. Historical evaluation-snapshot questions are deferred and do not replace the chosen source baseline. | Run recorded real-data experiments, error analysis and metric calculations once data is available. [Validation plan](../PYTHON_PORT_PLAN.md#validation-levels). |
| **LIM-013 — Runtime and memory at MLS scale.** Local grouping can have quadratic scans; full step diagnostics retain reference arrays. | Sparse grid lookup and array operations are implemented; no representative real-cloud profiling has been completed. | Measure runtime/memory by stage and cloud/window size. Preserve neighbor/DFS/sample order when optimizing; assess optional diagnostic retention. [Grouping](PYTHON_FITTING_MDL.md), [tracking](PYTHON_TRACKING.md). |
| **LIM-014 — Data adapters, units and precision.** Legacy readers have format-dependent thinning; count-prefixed text/CSV, CRS conversion and rounded exports need explicit contracts. | Core accepts owned double-precision coordinates with shared labels. Database integration and matched legacy I/O are deferred; no implicit unit/frame conversion is added. | Verify LAS alternating-point loading versus PTS density, export precision, metadata initialization, zone/hemisphere handling and database schema. [Data contract](../PYTHON_PORT_PLAN.md#data-contract-to-prepare-now). |
| **LIM-015 — Remaining scope.** Inventory consistency is distinct from complete reconstruction. | Dispositions now cover 201 definitions: 124 engine/UI and 77 inline header definitions, plus 19 named header types. 41 project and 51 dependency files are hashed. Eight earlier utility gaps and inline geometry/state contracts are implemented or have explicit undefined-source/input policies. Inactive DetectLoopCross/ValidateLoopCross workflows and legacy I/O/conversion/export remain pending. Commented rotation functions stay excluded; dependency math is ported only where reached. | Complete agreed remaining workflows and call-path/reference review before declaring reconstruction complete. Keep implementation scope distinct from defects or undefined behavior. [Header audit](PYTHON_HEADER.md), [pipeline audit](PYTHON_PIPELINE.md#explicit-coverage-audit), [plan](../PYTHON_PORT_PLAN.md#computational-inventory). |

## Preserved source behavior and findings awaiting later decisions

These are recorded for investigation and possible separate variants. Their
well-defined behavior remains part of the source-first baseline.

| ID | Preserved behavior/evidence | Later decision or experiment |
| --- | --- | --- |
| **LIM-016 — Branch seed Y copied into Z.** | The branch path builds `(RightPt.x, RightPt.y, RightPt.y)` at `RailwayExtraction.cpp:303`. Implemented unchanged: original-wrapper comparisons check the seed, and raw integrations show subsequent initialization refits elevations. | Review its effect beyond the controlled fixture before proposing a correction after reconstruction. Passing the preserved behavior does not close the item. [Orchestration](PYTHON_ORCHESTRATION.md#branch-ordering-filters-and-side-effects). |
| **LIM-017 — Geometry, coordinate conventions and cached state.** | Axis angle/point coefficients use unusual X/Y terms; folded angles and separate modes are retained. Gauge/tracking can leave cached centers/angles distinct from endpoint-derived values. Seed Z, side-buffer width, reset behavior and the no-op `SwapWnd` follow source. | Measure downstream effects before changing coefficients, conventions or cache-refresh timing. Use all quadrants, cardinal directions and changed coordinate origins. [Foundation](PYTHON_FOUNDATION.md#source-quirks-retained), [gauge](PYTHON_OBSERVATION_GAUGE.md), [tracking](PYTHON_TRACKING.md). |
| **LIM-018 — Grid and polygon boundaries.** | Non-unit insertion floors displacement before division while queries divide before flooring; maximum-bound insertion/query differ. Windows use XY ray-crossing `pnpoly`; switch boxes use `pnpoly2`, requiring identical cross-product tags with fixed `1e-10` precision. Ordinary rectangle edges are excluded; all-zero degenerate tags qualify. Neighbor traversal order is retained. | Characterize non-unit cells, world bounds, boundary points, precision scaling and density effects. Keep both predicates distinct and retain order when comparing conventional alternatives. [Foundation](PYTHON_FOUNDATION.md#source-quirks-retained), [switch boxes](PYTHON_SWITCH_BOXES.md#candidate-geometry-and-membership). |
| **LIM-019 — Growing, fitting and MDL rules.** | First-component retention, later components >100, shared SegmentID 0, source quantiles/bounds and strict first-winner support are retained. Fixed-angle fits use N draws; two-point fits use 100/500 attempts with two draws each. MDL uses fixed-angle fits, absolute residuals and three-line tie fall-through, including invalid-but-initialized candidate cases. | Compare alternative retention/refitting/scoring only after the baseline review. Inspect thresholds, flags, identities and changed random consumption together. [Fitting/MDL](PYTHON_FITTING_MDL.md), [heads](PYTHON_GMM_HEADS.md), [toes](PYTHON_TOES.md). |
| **LIM-020 — Tracking, recognition and toe rules.** | Retry heads accumulate duplicates; retries restart at 2x base and use strict thresholds; lookup stays 5x5. Source appends can precede rejected-segment stops, active stacks are unflushed, and observations have direction-dependent choices. Recognition uses 16 side bits, 20 entries, zero context weight, duplicate 2/8 and 3/9 patterns, code 999 and its source labels. Toe support/type/gate and one-toe-per-call rules are retained. | Review actual effects on coverage, ambiguity, missing regions, toe/branch completeness and confidence thresholds. Preserve the current reference results for each proposed variation. [Tracking](PYTHON_TRACKING.md), [recognition](PYTHON_RECOGNITION.md), [toes](PYTHON_TOES.md). |
| **LIM-021 — Engine lifecycle and repeat processing.** | Reset initialization clears layers but retains global toes/stacks/switch boxes, labels and random state. Each pass copies all toes, resets only local used markers and creates layers before filtering. Failed/skipped candidates can leave empty layers and a changed current pointer; old stacks/toes can be processed again across directions or calls. | Measure duplicate branches/toes/boxes, layer associations and accumulated labels on matched inputs. Any clearing, deduplication or global used-marker write changes the baseline. [Engine state](PYTHON_ORCHESTRATION.md#persistent-state-and-initialization), [switch boxes](PYTHON_SWITCH_BOXES.md#constraints-labels-and-appends). |
| **LIM-022 — Bidirectional seed and vector asymmetries.** | The UI adds 180 without wrapping, permits reverse mode 0 and narrows reset-wrapper angles to float32. Native reversal moves XYZ but leaves tags at vector indices due to XYZ-only assignment, then swaps rail containers and pops only layer zero. Segment/boundary/toe/stack history is untouched. The forward initialization bool does not guard its tracking call. | Compare directional seams, rail/tag identities, histories and repeated recognition before proposing normalized angles, whole-layer reversal or guarded recovery. These details are ported and source-tested, not corrected. [Details](PYTHON_ORCHESTRATION.md#toe-tagging-and-reversal), LIM-031. |
| **LIM-023 — Toe tagging and empty-container accesses.** | Native `AddPointTofootOfPependicularOnRailVtx` uses unsigned `size()-1` on an empty vector; `ReserverVectors` indexes layer zero and pops without empty checks. Python returns explicit undefined-source statuses, retaining earlier mutations. On valid vectors, strict X AND Y insertion bounds reject axis-aligned projections and nearest-vertex tagging keeps first ties. | Characterize empty/failure workflows and toe placement, then decide any separate recovery/projection rule. Native comparisons execute only defined containers; no invented point repairs the source. [Details](PYTHON_ORCHESTRATION.md#undefined-paths-and-diagnostic-limits). |
| **LIM-024 — Tracking reentry and bounded workflows.** | A new native `Trackingrailway` call without initialization can enter with `m_bInit=false`, then read uninitialized local next heights or unavailable previous segments. The engine reports `undefined_source_tracking_reentry`. Engine step limits defer downstream work and require a fresh workflow; single-loop resume remains available through `SequentialTracker`. | Review session/restart semantics after reconstruction. Any resumable orchestration must define state/side effects separately from the native reentry path and preserve reproducible draws. [Details](PYTHON_ORCHESTRATION.md#undefined-paths-and-diagnostic-limits). |
| **LIM-025 — Plane outputs and switch-box metadata.** | Under four points, the void plane fitter leaves BOTH outputs untouched; with enough points and no winner it clears support but leaves coefficients untouched. Candidate-local coefficients can therefore remain uninitialized before `fabs(c)`. Python preserves valid first-fit reuse, reports missing coefficients without a fabricated box, and marks per-candidate continuation non-comparable. Final native switch-box `referHgt` is also uninitialized (`None` in Python). | Review sparse/no-model candidate behavior and downstream metadata consumers. A structured failure or a valid reuse fixture does not resolve the undefined native read. Full native detection tests only execute initialized coefficients. [Details](PYTHON_SWITCH_BOXES.md#plane-fitting-and-output-reuse). |
| **LIM-026 — Switch-box heuristics and repeated appends.** | Source uses XYZ span expansion 2.53, fixed candidate dimensions, strict 10 cm height band, two 100-trial fits at 0.008, XY grouping 0.1, `abs(c)>=0.999` and integer `1000/skip_rate`. Labels and centroid use the group BEFORE the second fit, even when its support differs. Global/layer appends retain duplicates and do not set toe used markers. | Evaluate false positives, density/skip-rate effects, surface slope, size and repeated processing on real inputs. Any dimensions, final-support substitution, threshold change or deduplication is a separate variant. [Details](PYTHON_SWITCH_BOXES.md#constraints-labels-and-appends). |
| **LIM-027 — Plane eigen degeneracy and numerical environment.** | Duplicate/collinear samples are not skipped. Empty eigen support gives the defined `x=0` plane; tied null spaces follow the bundled symmetric JAMA basis. Python ports those branches and row accumulation, while nonfinite scatter/eigen arithmetic stops before native invalid indexing/stalling. Floating-point/host math differences near ties remain possible. | Compare matched raw draws, basis/signs, support IDs and near-threshold cases across target runtimes. Investigate overflow/recovery separately; do not substitute regularized covariance or a generic eigensolver basis in the baseline. [Details](PYTHON_SWITCH_BOXES.md#plane-fitting-and-output-reuse). |
| **LIM-028 — Grade projection and marker mutation.** | Vertical division uses world X/Z rather than chainage/height, so constant-X/rotated tracks can split differently. The full action fixes threshold 1, preserves old grade markers and can overwrite switch tags at endpoints/interior splits. Python preserves left-first order using iterative spans; native recursion/resource limits are not reproduced. | Compare orientations, threshold units, repeated calls, long vectors and interactions with toe tags. Any rotation-invariant profile or combined marker scheme is a separate variant. [Details](PYTHON_GRADES.md#vertical-division-and-marker-state). |
| **LIM-029 — Grade/curve algebra and singular cases.** | Source radius is `chord² - 8*maxh + 4*maxh²`; straight sections get finite squared-chord radius and nonzero curve compensation. Negative radii/grades follow the source branch; zero XY arc or radius retains NaN/Inf. Python exposes diagnostic status without dropping sections or replacing values. | Review physical units, straight/curved/downhill profiles and singular recovery on matched data. Passing the literal expressions does not validate the engineering interpretation. [Details](PYTHON_GRADES.md#membership-endpoint-tags-and-formulas). |
| **LIM-030 — GWAD layout, chainage and formatting.** | Grade reports accumulate XY arc, while GWAD accumulates 3D length and invokes grade tagging again. Every matching section appends its pair of fields, so a seven-column header can have five/nine/more-field rows. Begin matches precede end matches with strict XYZ tolerance. Finite TSV uses six decimals; native nonfinite spelling/sign and Windows file newlines are not byte-equivalent claims. | Check downstream GWAD expectations, point tags, repeated coordinates, units, names/encoding and consumer parsing. Any normalized schema, chainage unification or export correction requires a separate change. [Details](PYTHON_GRADES.md#gwad-row-and-text-behavior). |
| **LIM-031 — Point assignment and capacity-dependent tag transport.** | Native point copy construction includes PtType; assignment copies only XYZ. Python matches grade endpoints, reversal, tagged line swaps, cache/point normalization and bound-output tags. Vector insertion/assignment can transport tags differently with allocation history. Segment duplication matches guaranteed sufficient-capacity assignment; ambiguous longer-source cases choose value transport and report non-comparability. Mixed-tail insertion/pass scope remains flagged. | Capture target-runtime capacity/operation history before reproducing marked copies/insertions. Do not treat Python value transport or a compiler growth policy as universally native. [Header state](PYTHON_HEADER.md#bounds-regions-and-assignment), [grades](PYTHON_GRADES.md#tagged-vector-interoperability-and-limitations). |

### LIM-032: Integrated runner boundaries, fixtures and diagnostic delivery

**Evidence:** `pipeline.py`, `artifacts.py`, `coverage.py` and
[the pipeline guide](PYTHON_PIPELINE.md). These are new Python interfaces around
the preserved native components, not original source file/database adapters.

**Trigger and handling:** the runner requires finite explicit seed coordinates
and angle, stops after unsuccessful first initialization and defers requested
grade/GWAD actions after incomplete/undefined/limited orchestration. The native
GUI does not uniformly guard initial booleans; literal engine forward-pass behavior
is retained. Grade/GWAD remain opt-in API actions. Component scenarios explicitly
supply stack references/toes; raw recognition may remain ambiguous or gated.
The runner never fills missing native lines to force successful reconstruction.

**Impact:** `completed_with_limitations` means requested actions executed, not
full parity or automatic extraction success. Source audit checks inventory/hash
consistency, not an exhaustive C++ call graph or proof of implementation accuracy.
New diagnostic CSV is full-precision text; JSON maps IEEE values to null with
separate typed JSON-pointer annotations, preserving ordinary unavailable nulls.
Existing output directories are rejected; a filesystem failure can leave partial
new files before the final manifest. Hashes/draw logs are provenance, not a complete
serialized input/archive or resumable engine checkpoint. Local branch used markers
and global toe state remain distinct.

**Post-reconstruction review:** confirm runner stop/defer policies and output
contracts with real consumers; compare matched source action sequences, fixture
versus automatic geometry, source snapshots, input identities, RNG and tag state.
Decide legacy adapters/schema/precision separately. Audit remaining scope under
LIM-015; retain LIM-001/LIM-002/LIM-025 and whole-reference/real-data gaps. No entry
is closed by these new interfaces or their tests. **Status: open.**

### LIM-033: Standalone utility arithmetic, cache and integer contracts

**Evidence:** `RailwayExtraction.cpp:1172–1251,3574–3620,3641–3710,3860–3875,4296–4307`,
`LIDARPT.h:324–443` and [the utility guide](PYTHON_UTILITIES.md).

**Trigger and impact:** empty paired GMM fits produce three NaN parameter triples
after 101 iterations; heads still come from the last component. Strict height
erasure retains equality and treats NaN bounds as deleting nothing, unlike a
rewritten inclusive keep predicate. Bounds retain empty sentinels and the first
equal signed zero. `ReCount` adds the total only once for negative current, so it
can return an invalid array index; negative totals have additional defined and
undefined branches. Inner-angle zero arms retain NaN. The inactive intersection
uses its literal A2-dependent Y rule even for noncanonical coefficients and keeps
IEEE parallel/coincident results. These quirks remain in the baseline.

**Current handling:** source bodies are ported as standalone APIs and tested on
defined native inputs. Reached int32 division by zero and signed overflow raise
explicit Python errors without inventing a native index. 3D distance requires
the stored unit vector; stale/nonunit/nonfinite values are preserved and missing
values are rejected. Endpoint-only boundaries keep all unavailable caches as
`None`, empty membership and source zero endpoint tags. Initialization/tracking
now use the same boundary helper. Explicit `Line3D` construction remains available
when a caller intentionally computes derived geometry. Python returns paired
parameter/head values rather than exposing aliased native output-reference buffers;
those aliasing arrangements are not part of the comparison claim (LIM-010).

**Post-reconstruction review:** characterize degenerate inputs, valid index
preconditions, comparison/IEEE behavior, marked boundary consumers and cache
initialization timing. Any conventional modulo, normalized direction, repaired
intersection, finite angle or different empty-data policy is a separate change.
Utility completion does not activate loop/cross scheduling, establish real-data
accuracy or close earlier limitations. **Status: open.**

### LIM-034: Inline legacy types, cache state and header coverage policies

**Evidence:** all uncommented inline definitions and named types in `LIDARPT.h`,
the packaged inventory and [the header guide](PYTHON_HEADER.md).

**Triggers:** `LiDPOINT::operator/` creates a default output and divides its
uninitialized XYZ, ignoring the initialized input coordinates. The POINT2D XY
constructor leaves its flag unknown, while default construction initializes it;
zero division returns the input value. `FLOOR_double` narrows to float32, truncates
negative values toward zero and rounds positive halves upward. Undefined
float-to-int conversion has no portable result. Coincident neighbor centers
retain nonfinite polygons. Point assignment, normalization and cached endpoint
operations preserve native destination tags, with vector-capacity uncertainty
under LIM-031.

**Current handling:** legacy undefined division and unsupported integer casts
raise explicit errors; unknown XYZ/flags/caches remain absent. Defined inline
operations preserve source state and scalar arithmetic. Explicit stored center
and unit-vector caches are distinct from computed conveniences; native refresh
call sites are recorded in initialization/tracking/head extraction. Member bounds
preserve last-equal zero signs and caller output tags. Segment copying retains
destination distance/neighbor regions and reports capacity-dependent tag cases.
Native comparisons cover defined fields only, excluding unknown caches, flags,
pointer layouts and output-reference aliasing. Signature schema 2 versions the
expanded state hashes; earlier stack fingerprints are not directly comparable.

**Post-reconstruction review:** examine intended legacy division, constructor
flags, integer rounding/ranges, degenerate regions, cache refresh sequences and
tagged value assignment with matched runtime/capacity histories. Any repaired
formula, initialized placeholder, new scheduling or different allocation policy
requires a separate change. All 77 inline definitions and 19 types now have
explicit dispositions, but lexical coverage is not full-binary equivalence.
Inactive workflows, I/O and real-data/profiling gaps remain. **Status: open.**

## Resolution history

No entries are closed. Record future decisions here with ID, date, evidence,
user-approved scope where behavior changes, associated commit/experiment, and
remaining validation. Keep the original finding and baseline behavior accessible.
