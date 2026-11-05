# Limitations register and post-reconstruction review

Last updated: **2026-11-04**. Baseline: the current unpacked C++ source.

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
| **LIM-011 — Whole-application equivalence.** Original Windows application has not been built/executed for full comparisons. | Current suite: 549 test methods, including 206 using compiled source extracts. Nineteen numerical demos, the separate desktop viewer, cloud inspection/conversion and both audits are available. PCL helper comparisons compile frozen original arithmetic/sort/filter bodies against bundled Eigen, with PointXYZ/field-trait shims and supplied native sort order. Parameter/point/seed boundaries and negative-count retry behavior are also source-tested. Codec, managed UI and whole-binary claims remain separate; unsafe reads/casts/overflow are excluded. | Computational call-path review is complete; compare matched whole-path intermediate/final results where a usable native reference exists. Keep component, controlled-callback/decoder/dependency and full-binary claims separate. |
| **LIM-012 — Real-data/published evaluation.** Vancouver raw LAS inputs are now available; evaluation annotations and matched reference outputs remain unverified. | On 2026-11-04 the local Vancouver collection was checked: 47 extracted LAS files, 1,671,344,500 point records, common 2016-05-23 creation date and TerraScan metadata. The commented C++ sample pose falls in `Thales_11_20Las/16.las`; Python initialization succeeds on a bounded 6 m by 6 m crop with the source's even-record selection. This establishes a usable real-data starting point, while full-scene tracing, published accuracy/recall and robustness remain untested. Historical evaluation-snapshot questions do not replace the chosen source baseline. | Start recorded real-data tracing from the matched tile/pose, identify evaluation annotations and reference outputs, then run error analysis and metric calculations. See the Vancouver checkpoint below and [validation plan](../PYTHON_PORT_PLAN.md#validation-levels). **Status: open.** |
| **LIM-013 — Runtime and memory at MLS scale.** Local grouping can have quadratic scans; full step diagnostics retain reference arrays. | Sparse grid lookup and array operations are implemented; no representative real-cloud profiling has been completed. | Measure runtime/memory by stage and cloud/window size. Preserve neighbor/DFS/sample order when optimizing; assess optional diagnostic retention. [Grouping](PYTHON_FITTING_MDL.md), [tracking](PYTHON_TRACKING.md). |
| **LIM-014 — Data adapters, units and precision.** Legacy readers have format-dependent thinning; count-prefixed text/CSV, CRS conversion and rounded exports need explicit contracts. | Legacy readers/vector exporters and explicit ECEF/geodetic/UTM conversion/exports are implemented. Source degree/radian, zone/hemisphere, ellipsoid/pi, sampling and three-decimal contracts are retained. Optional decoder equivalence, runtime/error policies, global geodetic accuracy and database integration remain unverified. Tracking performs no implicit frame conversion. | Review source-codec/format compatibility, sampling/density, rounded exports, bounds/labels, coordinate frames and database schema. [Coordinates](PYTHON_COORDINATES.md), LIM-037, [I/O guide](PYTHON_LEGACY_IO.md), LIM-036, [data contract](../PYTHON_PORT_PLAN.md#data-contract-to-prepare-now). |
| **LIM-015 — Reconstruction scope and validation.** Inventory consistency is distinct from numerical equivalence. | The [reconstruction audit](RECONSTRUCTION_AUDIT.md) records the completed computational scope review: 265 definitions, 19 named LIDARPT types, all 41 project files, 15 workflows and 29 active/inactive call-site witnesses. The original 41 project/51 dependency hashes and baseline ID remain unchanged; 14 PCL extracts/19 selected PCL/Eigen hashes stay separate. Commented scheduling/rotations remain excluded and whole dependency libraries are not claimed ported. | Implementation scope review is complete; whole-application/native and real-data validation remain open. Review every limitation with the user before selecting corrections. Neither completion metadata nor passing inventory checks close this ID or establish native/published-result equivalence. [Audit scope and evidence](RECONSTRUCTION_AUDIT.md). |

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
Automatic loop/cross scheduling remains inactive; I/O and real-data/profiling
gaps remain. **Status: open.**

### LIM-035: Inactive loop/cross projection, cache and integer phase contracts

**Evidence:** `RailwayExtraction.cpp:2342–2480`, fixed-angle loop/cross fitter,
commented calls at lines 512/827/1016 and [the loop/cross guide](PYTHON_LOOP_CROSS.md).

**Triggers and impact:** detection projects the fitter's output before checking
its Boolean. Empty/no-winner/insufficient support can therefore read uninitialized
endpoints and coefficients. A fully initialized short fit is projected even though
false, then discarded. Literal intersection, separately cached angle, NaN polygon
and zero-normal distance arithmetic remain visible. Successful chainage uses the
reference endpoint; deduplication checks only the last record's stored center and
coefficients, requiring mean distance strictly greater than one. An empty pool
can accept initialized nonfinite geometry; NaN duplicate distance suppresses it.
Validation casts absolute chainage difference to int32 **before** multiplying by
100, with possible undefined cast or signed overflow. It retains residues 0/1/24
of the truncated difference, chooses the first largest star neighborhood and
preserves that group's order, rather than sorting or finding a clique.

**Current handling:** both routines are standalone ports with no automatic
scheduling or label mutation. Unknown reached fields and unsafe int32 arithmetic
produce explicit `undefined_source_*` statuses, retaining the current pool and
actual prior draws. Singleton validation does not read chainage. Initialized
nonfinite projection retains the source append behavior with diagnostics.
Native failure comparisons stop before undefined endpoints; full comparisons
use initialized fit outputs and supplied references. Positive skip rate remains
the existing fitter's explicit API contract. These checks do not establish
automatic loop/cross recognition on real MLS data.

**Post-reconstruction review:** characterize failed fits, cache initialization,
nonfinite/parallel geometry, duplicate spacing, intended fractional phase behavior,
int32 ranges and group selection with real/reference inputs. Decide any Boolean
guard, corrected intersection/phase formula, geometry recovery or activation of
commented scheduling as a separate change. Existing source behavior remains the
baseline and all earlier IDs remain open. **Status: open.**

### LIM-036: Legacy reader bounds, codec, tokenization and vector export policies

**Evidence:** `TXTFileReader.cpp`, `LASFileReader.cpp`,
`RailwayExtraction.cpp:108–143`, `MainForm.cpp:149,172,443,577`, `HashMap.h` and
[the I/O guide](PYTHON_LEGACY_IO.md).

**Triggers and impact:** format-dependent density keeps LAS indices 0/2/4 while
PTS/CSV retain all declared rows. LAS uses unsampled header extrema. Text resets
only X/Y extrema; Z stays at its constructor zero. Source tokenization collapses
space/comma delimiters, C numeric prefixes can become zero, and fourth-field CSV
tags become intensity. Indexed export skips exactly one marker and can read
past center/right vectors after already writing a prefix. Three-decimal outputs
lose numerical precision; both unindexed rails receive tag 100. Cached filename
reentry retains existing engines/labels/layers, while direct PTS opening is fresh.

**Current handling:** literal reader bounds and read-only source indices remain
available. Explicit grid adaptation preserves source XY and encloses Z because
the native index ignores Z. Missing/overlong/NUL/nonfinite/out-of-range inputs are
rejected without invented coordinates. Optional laspy/lazrs decoding is separately
tested; libLAS codecs, format versions and malformed-file behavior are not proven
equivalent. The historical MIN/MAX macro inclusion chain remains unverified:
conditional macro shims are used, and equal signed-zero bounds have no matched
Windows parity claim. C-locale parsing and nonfinite/rounding text can differ by
CRT/runtime. Logical LF/ASCII text comparisons do not prove Windows CRLF bytes.
Reader periodic/final callbacks preserve their source cadence; grid/display
progress is separate. Python validates a new session grid before cache commit,
requires explicit overwrite and rejects undefined indexed output before file
creation. Defined prefixes are inspectable; multi-file writes are not atomic.
These file/input policies deliberately do not claim native failure side effects.

**Post-reconstruction review:** compare real LAS/LAZ and text files with the
matched Windows reader/runtime, decoder/backend versions, metadata and sampling;
inspect header bounds, source macros, line lengths/locale and round-trip loss.
Review indexed markers/vector assumptions, cache/error lifecycle and output
overwrite/partial-write requirements with consumers. The coordinate workflows
are now implemented separately under LIM-037, and standalone PCL helpers under LIM-038.
Establish units/CRS and database adapters
before real-data evaluation. No existing limitation is closed by I/O completion.
**Status: open.**

### LIM-037: Source coordinate units, constants, accumulated state and bindings

**Evidence:** `UTM.h/.cpp`, all `DataConvertor.cpp` bodies,
`MainForm.cpp:466–542`, `DataConverstionForm.cpp:50–248` and
[the coordinate guide](PYTHON_COORDINATES.md).

**Triggers and impact:** active UTM macros use double despite old float comments.
Forward UTM takes degrees despite a radian comment; inverse and raw geodetic
helpers use radians. UTM and ECEF use distinct pi/ellipsoid literals. Automatic
zones are not clamped, can yield 61 at +180, and are discarded by the native
converter even when different rows use different zones. Inverse hemisphere is
explicit at component level, but both source menu/batch bindings hardcode false;
the old standalone menus also hardcode zone 32. Origin/domain arithmetic can
produce initialized nonfinite results, and later automatic-zone casts can become
undefined. Conversion appends input and reprocesses old output instead of clearing
it. Callback totals remain 3N even when actual work exceeds that; converter and
writer emit only periodic events without a final event. Reverse batch aggregate
progress advances before conversion. Three-decimal files retain intensity while
omitting class/segment and frame metadata. Batch fallback treats non-.las/.pts
suffixes as legacy CSV, including .laz; only exactly .csv gets CSV output.

**Current handling:** literal formulas, expression order, mutable buffer state,
source bindings, callback cadence and initialized IEEE values are preserved.
Undefined automatic casts raise explicit errors with prior state retained.
Snapshots preserve labels/intensity; conversion never overwrites input XYZ or
silently enters tracking. Per-point zone diagnostics do not invent CRS metadata.
Finite core-cloud conversion is explicit. General CLI inverse hemisphere is
required, while separately named source bindings retain false. Python paths,
overwrite checks, ASCII/LF text and filesystem errors remain explicit boundaries;
managed dialogs, cancellation/reentry and progress-bar range exceptions are not
recreated. Full original numerical bodies are compiled for component tests;
managed batch execution, platform libm/rounding parity and global geodetic
accuracy are not established by those comparisons.

**Post-reconstruction review:** evaluate source projection accuracy/domain and
constants, frame/zone/hemisphere provenance, mixed-zone files, origin handling,
repeated-call intent and progress semantics with matched reference/real inputs.
Decide any buffer clear, normalized units/zones, different projection library,
guard/recovery, metadata schema or southern-batch correction as a separate
behavior change. Retain the source baseline and all earlier limitations.
**Status: open.**

### LIM-038: PCL float precision, voxel indices/ties and centroid metadata

**Evidence:** `LASFileReader.cpp:113–148`, the selected bundled PCL 1.8/Eigen
sources frozen in `pcl_reference.json` and [the helper guide](PYTHON_PCL_HELPERS.md).

**Triggers and impact:** both helper inputs are PointXYZ float32, unlike the
core's double-precision labeled points. Voxel size/inverse, bounds and centroid
accumulation round as float32; linear keys use checked native int32/int64 paths.
The safe large-extent branch copies the entire input, including invalid rows and
organization. Empty/all-invalid bounds retain sentinels and reach an unsafe
extent cast; intermediate extent, division or index arithmetic can overflow
before the advertised fallback. Equal-key std::sort order is unspecified and
can change float32 sums. Dense metadata with nonfinite values is contradictory;
Eigen scalar/SIMD NaN bound rules can change the reached path. The source's
unqualified floor overload and explicit float32 minimum-bound cast also matter
at large coordinate spans. Centering accumulates double XYZ but writes float32;
rotation is commented out. Empty centroids retain (1,0,0,0), and non-dense invalid
rows remain unchanged. Dense nonfinite rows execute off-diagonal 0*NaN/Inf.
Sensor pose is copied, not translated with the points.

**Current handling:** both helpers are standalone with explicit PointXYZ state
and core bridges; no labels/intensity are invented or transferred. Native unsafe
integer paths return diagnostic statuses without an output cloud. Dense
nonfinite voxel input has an explicit unsupported portable boundary. Initialized
centering IEEE values and empty metadata remain visible. Default voxel equal-key
order is stable input order; potentially sensitive groups are flagged as
non-comparable unless a valid native sorted-row permutation is supplied. This
replay is conditional on the caller's trace, not proof of the historical MSVC
sort. Exact finite bits, double matrix/centroid values and metadata are tested
against frozen source routines with the bundled Eigen. Point/trait shims, current
compiler overload rules, NaN payloads, allocator/runtime behavior and the complete
Eigen/Boost/stdlib include/ABI graph are outside whole-binary parity claims.
Selected dependencies are checked locally when present; frozen extracts/notices
ship without requiring the large ignored installation bundle. Native helper
comparison tests explicitly skip without a compiler or bundled Eigen.

**Post-reconstruction review:** compare the matched Windows PCL/Eigen/compiler
environment, tie permutations, coordinate ranges, floor overloads, density flags,
centroid narrowing and sensor-pose consumers. Profile real clouds separately.
Decide any stable-sort normalization, double centroid accumulation for voxels,
repaired integer fallback, empty filter result, metadata update or automatic
helper scheduling as a separate behavior change. Keep the original source and
all earlier limitation IDs open for the agreed review. **Status: open.**

### LIM-039: Numerical input adapters and reconstruction review evidence

**Evidence:** `RailwayExtraction.cpp:46–85`, integer member declarations at
`RailwayExtraction.h:81–84`, `PointXYZDouble.h:23–75`,
`InitialLocationForm.cpp:26–59`, the VTK picking paths in
`VTKWindowController.cpp`/`VMouseInteractorStyle.h`, and the
[reconstruction audit](RECONSTRUCTION_AUDIT.md).

**Triggers and handling:** the ten source extraction parameter fields are
doubles, but the engine stores point-count and intensity thresholds in four
signed integers. `ExtractionConfig.with_source_parameters` explicitly truncates
these values toward zero; nonfinite/out-of-int32 conversions are guarded instead
of executing undefined native casts. It retains the original config's grid size
and skip rate. Direct Python config construction remains an explicit typed input
interface; use the adapter when reproducing source parameter updates.

Negative point-count thresholds participate in unsigned `vector.size()`
comparisons in C++. For supported int-indexed clouds, their promoted values
exceed attainable counts: the initial shortage gate is true and all five retry
success gates are false. Python retains that exhaustion behavior; original-loop
tests compare both count thresholds, stops, labels and RNG use. It does not
reinterpret a negative threshold as disabling a check. Clouds requiring native
indices beyond signed-int range are outside the validated native input domain.

`initial_location_dialog_defaults` models the active form's integer angle wrap
and invalid-mode reset. Direct seeds do not pass through a dialog. It returns
only primary XYZ/angle/mode and leaves second coordinates unavailable, as the
native getter does not assign them. Python numeric values avoid .NET locale and
text-format round trips. Screen picking/camera behavior is excluded; Python picks
must already be in the input cloud's world coordinates. VTK's display translation
is not automatically applied to those inputs.

`PointXYZDouble` scalar equality ignores metadata and uses strict 1e-5 XYZ
tolerance; exact null, default metadata and pointer-copy values are available.
Scalar equality does not replace point-pointer/cloud-row identity. Native union
layout, alignment, mutation of homogeneous padding and null-pointer dereferences
are outside the Python value interface.

The packaged review records 15 computational workflows, explicit file scope,
29 lexical call-site witnesses and 265 function token inventories. The scanner
blanks comments/strings but does not resolve overloads, templates, preprocessing,
indirect dispatch or runtime reachability. External names can collide with
project method names. Checks reject inventory/resource/call-token drift and
missing evidence references; they do not rerun tests or certify every mapping.
Original-wrapper scheduling tests retain controlled callbacks, and whole Windows
execution and real MLS measurements remain unverified.

**Post-reconstruction review:** decide whether GUI text/locale and screen picking
need a separate interface, compare matched Windows input conversion behavior if
needed, and review all existing limitations before choosing corrections. The
completed computational scope review does not close any limitation.
**Status: open.**

### LIM-040: Interactive main-track viewer, crop scope and desktop lifecycle

**Evidence:** `viewer_model.py`, `viewer.py`, `tests/test_viewer.py` and
[the viewer guide](PYTHON_VIEWER.md). This is a new desktop presentation layer
over the existing source-based computational core.

The viewer streams a bounded display preview and loads an explicit XY crop for
inference. Both preserve original zero-based even-record selection; the preview
alone may be coarser. Crop inference retains selected records in order,
original float64 world coordinates, intensity and the full LAS header grid origin.
A point budget rejects an oversized crop rather than introducing extra thinning.
Crop boundaries can cause earlier point shortages than full-file tracking.
The displayed trace must not be interpreted as the entire scene or a benchmark.

The default radius is now 8 m; it can be set up to 30 m. On a point-shortage
stop, the final attempted windows provide a crop/file-boundary coverage hint
without overriding the source stop code. **Restart with larger region** doubles
the radius and replays from the saved preview pose with confirmation edits
restored. It creates a fresh grid, labels and RNG and repeats the source
initialization order. In-place cloud expansion/point-index remapping is not
implemented. Other algorithm stop conditions are not bypassed.

At full-tile scale, exactly two even-index records in `16.las` decode one float64
ULP below the header minimum Y (`5458208.5540000005`). The viewer omits these
records, reports the count and keeps header/grid origins and every retained XYZ
unchanged. Only discrepancies within four ULPs of a header bound are permitted;
larger discrepancies raise a metadata error. This is an explicit viewer input
policy, with unit coverage for both cases. No native-equivalence claim is made
for the omission. The computational reader and grid are unchanged; the raw file
is unmodified. Review exact decoder/header rounding if whole-reader equivalence
becomes necessary.

Recorded real-data traces from the same source pose and RNG seed 1 reach
3 steps/2.87 m at radius 3, 8 steps/7.82 m at radius 8 and 19 steps/18.62 m
at radius 30. The full-tile run retains 27,716,650 inference records and ends
with `insufficient_window_points` after search lengths 1–6 are exhausted near
the LAS edge. These runs extend through multi-line windows; they do not validate
turnout recognition or establish continuity across adjacent files. The first
two steps of 3/8 m runs and first seven of 8/30 m runs match exactly.

The initial UI exposes one main-track pass. Branch and bidirectional algorithms
remain available in the computational API, but their live orchestration, original
Windows menus/layer management, all-file stitching and exports are not recreated
in this first interface. Existing source toe/fit/MDL limitations remain visible.
The paired windows and paths use computed results, not canned animation.

Two display picks are reconstructed from the rendered depth buffer, restored to
world coordinates and resolved to actual preview records before the original
midpoint/angle/mode prefix. Sparse preview selection and picking precision can
differ from the original VTK interaction. A preset retains the source's commented
Vancouver pose. Preview initialization precedes confirmation/reinitialization
on the same MSVC RNG stream. Pause/Step reuse a single SequentialTracker; pauses
take effect at iteration boundaries. Reset deliberately begins a new session and
clears working labels. Display centering affects rendering only; no CRS conversion
or change to coordinate-dependent Kalman formulas is introduced.

GUI changes execute on the main thread and serialized workers perform loading/
inference. Close cancels loading at chunk boundaries; an already-running filter
iteration finishes before its worker exits. Open3D 0.19 is an optional dependency;
a working graphical desktop is required. Linux checks X11/XWayland access before
native window creation. Headless/SSH/GPU-driver environments need separate checks.

**Post-reconstruction review:** assess real picking precision, longer/denser
regions, UI responsiveness, desktop/driver portability, source stop diagnostics,
crop-to-full-scene continuity and live multi-pass/branch lifecycle before extending
the scope. Preserve the computational baseline while changing display/session
behavior. This interface does not establish paper accuracy or full Windows-app
equivalence. **Status: open.**

## Vancouver data checkpoint — 2026-11-04

The local, ignored `Vancouver/` collection contains 47 extracted LAS 1.2 files
(point format 1), with XYZ, intensity, return fields, classification and GPS time.
Their LAS files total 46,797,659,943 bytes (43.584 GiB), excluding RAR archives and
sidecars. Point totals count stored records across tiles, not unique physical
returns. All header-declared point payloads fit their file sizes.

Both commented source XY seeds (`MainForm.cpp:383` and
`RailwayExtraction.cpp:214`) fall within `Thales_11_20Las/16.las`. A supplied
`delete.txt.config` records an original path under
`E:\Theles Project\Vancouver data\Thales_11_20Las\`. All LAS headers report
TerraScan and a creation date of 2016-05-23; that is a file creation date, not
independent proof of the acquisition date or exact published evaluation subset.

Four tiles (`19`, `24`, `26`, `56`) explicitly declare NAD83 / UTM zone 10N
(EPSG:26910) in embedded GeoTIFF metadata; the other 43 omit CRS declarations.
Shared coordinates are consistent with one collection, but carrying the same
CRS to untagged tiles remains a documented inference. No coordinate conversion
was performed. The existing `ML_23_Line_2_with_track_points.csv` export has
different stored coordinate bounds and cannot yet be linked to these LAS tiles.

Point checks sampled 577,536 records across the 47 files, with no sampled
nonfinite XYZ, out-of-header-bounds coordinates or invalid return fields.
Sampled classifications were all code 1; evaluation ground-truth labels have
not been identified, and the auxiliary `.cls` files have not been interpreted.
This is not an exhaustive scan for corruption or dataset-quality defects.

A separate bounded check retained 1,462,180 original even-index records from a
6 m by 6 m crop of `16.las`. The original sample pose
`(513194.41, 5458237.85, 6.61)`, angle 355 and mode 4, with source defaults and
MSVC RNG seed 1, produced Python initialization status `ok`, one saved segment
and 400 draws. The returned gauge is constrained by the configured 1.5 value;
it is not an independent gauge-accuracy measurement. Full-scene tracking,
switch/branch success, runtime/memory and agreement with a matched native run
remain unverified. Subsequent viewer traces cover 19 accepted steps/18.62 m
through `16.las` to its file edge, with the explicit two-record header-rounding
policy in LIM-040. This extends the initial bounded check; adjacent-file tracing
and native/full-scene validation remain open.

The read-only working script and detailed evidence are kept locally under
`outputs/vancouver-verification/verify_dataset.py` and `verification.json`.
The raw data/archives were not modified. All 40 limitation IDs remain open;
availability of this collection does not close LIM-012 or other validation IDs.

## Resolution history

No entries are closed. Record future decisions here with ID, date, evidence,
user-approved scope where behavior changes, associated commit/experiment, and
remaining validation. Keep the original finding and baseline behavior accessible.
