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
| **LIM-010 — Python boundaries and native uninitialized fields.** Some invalid counts, ownership, dimensions or nonfinite query inputs have no supported native result. Default geometry/visualization fields also differ in initialization. | Python rejects invalid API inputs or exposes unavailable fields as `None`. Visualization edges have initialized Python caches while native comparisons cover their endpoints. | Audit every newly implemented consumer before reading those fields. Retain explicit boundary contracts and identify which comparisons exclude undefined native values. [Foundation](PYTHON_FOUNDATION.md), [initialization](PYTHON_KALMAN_INITIALIZATION.md), [toes](PYTHON_TOES.md). |
| **LIM-011 — Whole-application equivalence.** Original Windows application has not been built/executed for full comparisons. | At consolidation: 222 test methods, including 73 against compiled source extracts; eight demos and packaging checks pass. Extracts use documented shims/diagnostics and exclude unsafe native cases. | Complete function/call-path coverage, then compare matched whole-path intermediate/final results where a usable reference exists. Keep component and full-binary claims separate. |
| **LIM-012 — Real-data/published evaluation.** Suitable raw inputs, annotations and reference outputs are not yet supplied. | Synthetic fixtures validate specified behavior only; published accuracy/recall and robustness are not reproduced. Historical evaluation-snapshot questions are deferred and do not replace the chosen source baseline. | Run recorded real-data experiments, error analysis and metric calculations once data is available. [Validation plan](../PYTHON_PORT_PLAN.md#validation-levels). |
| **LIM-013 — Runtime and memory at MLS scale.** Local grouping can have quadratic scans; full step diagnostics retain reference arrays. | Sparse grid lookup and array operations are implemented; no representative real-cloud profiling has been completed. | Measure runtime/memory by stage and cloud/window size. Preserve neighbor/DFS/sample order when optimizing; assess optional diagnostic retention. [Grouping](PYTHON_FITTING_MDL.md), [tracking](PYTHON_TRACKING.md). |
| **LIM-014 — Data adapters, units and precision.** Legacy readers have format-dependent thinning; count-prefixed text/CSV, CRS conversion and rounded exports need explicit contracts. | Core accepts owned double-precision coordinates with shared labels. Database integration and matched legacy I/O are deferred; no implicit unit/frame conversion is added. | Verify LAS alternating-point loading versus PTS density, export precision, metadata initialization, zone/hemisphere handling and database schema. [Data contract](../PYTHON_PORT_PLAN.md#data-contract-to-prepare-now). |
| **LIM-015 — Remaining scope.** Branch/bidirectional orchestration, switch-box detection, grade/curve analysis, remaining standalone routines and final pipeline delivery are incomplete. | Tracked in the implementation inventory. Inactive loop/cross calls remain inactive; available standalone helpers do not imply a complete workflow. | Finish the agreed inventory and integrated synthetic/diagnostic/export checks. Keep unimplemented scope distinct from a defect in a completed component. [Plan](../PYTHON_PORT_PLAN.md#computational-inventory). |

## Preserved source behavior and findings awaiting later decisions

These are recorded for investigation and possible separate variants. Their
well-defined behavior remains part of the source-first baseline.

| ID | Preserved behavior/evidence | Later decision or experiment |
| --- | --- | --- |
| **LIM-016 — Branch seed Y copied into Z.** | The audited branch path builds `(RightPt.x, RightPt.y, RightPt.y)` at `RailwayExtraction.cpp:303`. Branch orchestration is not ported yet. | Preserve and characterize it when porting, including subsequent initialization height refits. Review any correction after reconstruction. [Audit](../PYTHON_PORT_PLAN.md#potential-defects-and-behavior-requiring-targeted-tests). |
| **LIM-017 — Geometry, coordinate conventions and cached state.** | Axis angle/point coefficients use unusual X/Y terms; folded angles and separate modes are retained. Gauge/tracking can leave cached centers/angles distinct from endpoint-derived values. Seed Z, side-buffer width, reset behavior and the no-op `SwapWnd` follow source. | Measure downstream effects before changing coefficients, conventions or cache-refresh timing. Use all quadrants, cardinal directions and changed coordinate origins. [Foundation](PYTHON_FOUNDATION.md#source-quirks-retained), [gauge](PYTHON_OBSERVATION_GAUGE.md), [tracking](PYTHON_TRACKING.md). |
| **LIM-018 — Grid and polygon boundaries.** | Non-unit insertion floors displacement before division while queries divide before flooring; maximum-bound insertion/query differ. Membership is XY-only with source edge asymmetry and neighbor order. | Characterize non-unit cells, world bounds, boundary points and density effects. Retain exact order when comparing a conventional grid or polygon variant. [Details](PYTHON_FOUNDATION.md#source-quirks-retained). |
| **LIM-019 — Growing, fitting and MDL rules.** | First-component retention, later components >100, shared SegmentID 0, source quantiles/bounds and strict first-winner support are retained. Fixed-angle fits use N draws; two-point fits use 100/500 attempts with two draws each. MDL uses fixed-angle fits, absolute residuals and three-line tie fall-through, including invalid-but-initialized candidate cases. | Compare alternative retention/refitting/scoring only after the baseline review. Inspect thresholds, flags, identities and changed random consumption together. [Fitting/MDL](PYTHON_FITTING_MDL.md), [heads](PYTHON_GMM_HEADS.md), [toes](PYTHON_TOES.md). |
| **LIM-020 — Tracking, recognition and toe rules.** | Retry heads accumulate duplicates; retries restart at 2x base and use strict thresholds; lookup stays 5x5. Source appends can precede rejected-segment stops, active stacks are unflushed, and observations have direction-dependent choices. Recognition uses 16 side bits, 20 entries, zero context weight, duplicate 2/8 and 3/9 patterns, code 999 and its source labels. Toe support/type/gate and one-toe-per-call rules are retained. | Review actual effects on coverage, ambiguity, missing regions, toe/branch completeness and confidence thresholds. Preserve the current reference results for each proposed variation. [Tracking](PYTHON_TRACKING.md), [recognition](PYTHON_RECOGNITION.md), [toes](PYTHON_TOES.md). |

## Resolution history

No entries are closed. Record future decisions here with ID, date, evidence,
user-approved scope where behavior changes, associated commit/experiment, and
remaining validation. Keep the original finding and baseline behavior accessible.
