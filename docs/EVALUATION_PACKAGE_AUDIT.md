# Evaluation package audit — 2026-11-14

**Both Vancouver rail-head ground truth and an implemented evaluation program
are present.** The essential reference/model inputs for paths 1–9 are available.
Exact published-result reproduction, execution of the Windows tool, and the
complete mapping of the 100 additional switch regions remain unverified.

**Subsequent implementation:** the [Python rail evaluator](PYTHON_EVALUATION.md)
now reconstructs the computational evaluation command, with original-source
comparisons and a full 24-part Vancouver run. Historical pass/fail agreement and
full-scene Python prediction evaluation remain unresolved. This package audit
records the discovery evidence preceding that implementation.

The supplied `evaluation data/` tree contains 7,657 files, approximately 36.4 GB.
It includes Vancouver and Hyundai/Honam datasets, separate algorithm/evaluation
applications, vendor dependencies, duplicates and historical results. It remains
local and ignored by Git. None of its source, point clouds or workbooks was edited.

## Confirmed components

| Component | Location under `evaluation data/` | Verified evidence |
| --- | --- | --- |
| Vancouver reference labels | `DataForPaper/Vancouver For Evaluation_20180306/Manually Edited/` | 29 LAS: 24 numbered annotation parts covering paths 1–9 and five aggregated head-only files. Full scans show classes 0, 1 and 3; the evaluator explicitly defines class 3 as rail-head reference points. |
| Saved original model outputs | `DataForPaper/Vancouver For Evaluation_20180306/RailVector/` | All 18 left/right files for paths 1–9 are present. Each has a declared segment count followed by six CSV coordinates per segment. All rows parse, match their declared counts and are finite. These are predictions/model outputs, not ground-truth rail vectors. |
| Native evaluator source | `RailwayExtractor_20180306_evaluation/Source/Viewer/ViewerDoc.cpp` | Active loaders and `OnHeadptsandtrackEvaluation`, at line 5094, read classified LAS and two rail-vector files, assign point memberships and compute error/count summaries. The menu is present in `Viewer.rc:90–95`. |
| Native evaluator executables | `RailwayExtractor_20180306_evaluation/bin/Viewer.exe` and `Viewerd.exe` | Both are Windows x86 PE executables, with PE timestamps in March 2018. Their presence and static imports were checked; neither was executed. |
| Published-style historical results | `DataForPaper/Evaluation.xlsx` | Vancouver rows match the paper's Table I values, including approximately 1.58 km main tracks and mean per-path recall 99.4633%, displayed as 99.46%. |
| Saved point-level outcomes | `DataForPaper/Vancouver/TPFN/Path1_TPFN.txt` through `Path9_TPFN.txt` | All nine files exist. Their code-1/code-2 counts match the older workbook's per-path success/failure counts exactly. These are evaluated outcomes, not input annotations. |
| Additional setup/switch notes | `DataForPaper/Grade_param.xlsx` and `Templates.xlsx` | Initial poses, template definitions, per-path switch ledgers and a 22-entry sheet with a `Ground True` column are present. |

The reference LAS set has a second byte-identical copy under
`DataForPaper/Vancouver/Reference/Manually Edited/`. All 18 saved vector files
also have identical counterparts under `DataForPaper/Vancouver/Path N/`.
These copies must not be treated as independent examples.

The 24 annotation parts contain 4,710,223 class-3 records when counted by part;
this sum is not a claim of globally unique points across overlapping crops.
All five aggregated head-only LAS contain class 3 exclusively. Missing head-only
aggregates for paths 4, 5, 8 and 9 are not missing labels: their full numbered
annotation LAS contain class-3 points.

## Relationship to our existing Vancouver data

All 18,360 saved rail-segment endpoints fall inside the existing 47 extracted
raw LAS header footprints, in their original coordinates. The maximum supplied
vector easting is 514487.290, west of the extracted-file gap beginning at
514489.610. The archived-only tiles 57–59 therefore are not needed to cover
these supplied vector endpoints. Header rectangles alone do not certify exact
point occupancy or every segment interior.

A check of 1,424 sampled manual reference points against the existing 4,684,730
raw-point display sample found nearest 3D distances with per-path medians around
4–5 cm. Sampling explains some separation; these are alignment checks against
a sparse display sample, not annotation accuracy or model-error measurements.
The generated full/parking-zone overlay is local at
`outputs/evaluation-audit/vancouver_reference_overlay.png`. The layout agrees
visually with the paper's Figure 7 and identifies the five parking tracks and
two connectors in the same coordinate space.

The package also contains 47 `Vancouver/LiDAR/subsample/` tiles. Their point
counts are approximately 18–33% of our same-named original tiles, with varying
ratios. They are a different input/preprocessing version. Confirm which input
stage and settings produced the stored models before comparing Python results
to published values. Do not combine the Honam data into the Vancouver cases.

## Evaluation protocol and version gaps

The files are sufficient to start a nine-path rail evaluation, but selecting a
single authoritative reproduction protocol requires the following decisions.

1. **Thresholds differ.** The active C++ source uses horizontal distance
   `0.12 m` and absolute vertical distance `0.05 m`, after a `0.3 m` candidate
   polygon assignment. `Evaluation_20180306.xlsx` states 10 cm/5 cm. The paper
   describes 12 cm-wide buffers. Threshold distance, full buffer width and
   vertical gating must be distinguished rather than equated.
2. **The current source's FP/FN names are swapped relative to conventional
   ground-truth-based classification.** A class-3 reference point failing the
   distance test increments `nFP`; a non-head point passing increments `nFN`.
   Consequently its printed precision/recall names need interpretation. Saved
   TPFN files use 1/2 for reference successes/failures, while this source uses
   `ClassCodeRe` 1/3 for those cases. Do not reuse one code map for both inputs.
3. **Signed vertical errors differ from the saved results.** The active source
   accumulates absolute vertical residuals for its mean, but the older workbook
   and paper report negative vertical means. Its standard-deviation pass then
   uses signed residuals around that absolute mean. This source, unchanged,
   cannot directly generate those reported signed means.
4. **Scoring scope requires review.** The source assigns each point to the first
   eligible left/right segment polygon and does not score reference positives
   outside every candidate polygon. The part-wise annotation-positive sum
   exceeds the saved outcomes' denominator by 10,463 records. Exact point
   identity, crop overlap and excluded regions must be reconciled before
   interpreting that difference as failures or coverage loss.
5. **Aggregation differs from pooled accuracy.** The older workbook's 99.46%
   headline is the mean of nine per-path recalls. Pooling its recorded counts
   produces approximately 99.7347%. Use the intended denominator explicitly.
6. **Some saved model lengths differ.** Averaging the two supplied rail lengths
   matches the older workbook to millimetre scale for most paths; path 4 differs
   by about −0.716 m and path 9 by +0.602 m. Available center-line files do not
   remove those differences. This requires artifact/version matching for exact
   reproduction, not an assumption that the files are missing.
7. **The later workbook is unfinished.** Its path-1/2 count sections are filled;
   path-3–9 count sections are blank/zero with cached `#DIV/0!` results. The older
   complete workbook and saved outcomes remain the better initial cross-check.

The evaluator is stateful: vector loaders append, membership arrays persist and
point segment IDs are mutated. A fresh case/session must be defined for repeatable
batch evaluation. A Python implementation should preserve a documented legacy
comparison separately from any corrected metric definitions.

## Additional switch-region evaluation

`Templates.xlsx` Sheet2 contains 22 numbered ground-truth type entries associated
with Vancouver path groups, consistent with the nine-path scene's 22 multi-track
regions. `Grade_param.xlsx` supplies further per-path and Honam ledgers.

These are useful switch labels, but no unambiguous manifest was identified that
maps the paper's **100 additional** regions to input clips/coordinates, reference
types and experiment variants. Larger LAS files such as `Data_multitrack*.las`
may contain additional regions; absence of an index is not proof that their raw
points are absent. The precise 100-case mapping still needs confirmation.

Both 2018 source copies currently set unary/context weight to `1` (commented
alternative `0.9`), while the paper's main contextual experiment uses 0.9. This
configuration difference affects switch-result reproduction. Templates and type
IDs must also be mapped explicitly before computing recognition accuracy.

## Running the software and remaining work

The release binary imports VC80/MFC80 x86 runtime DLLs; the debug binary imports
their debug counterparts. The package includes `bin/install_first/vcredist_x86.exe`
and its application/vendor DLLs. A compatible Windows runtime environment is
still needed to execute it; native loading, the installer and dynamic plugins
were not tested on this Linux host. A Windows rebuild additionally requires a
compatible MFC toolchain and the bundled SDK/library setup. No missing-data claim
is inferred from Windows system DLLs being outside the package.

Our existing Python code follows the frozen ADE/PCL snapshot. The newly found
2018 MFC/LiDAS application is a separate reference, not an automatic replacement
for that baseline. Its ground-truth evaluation workflow is not yet implemented
in `railway_mls`, and the current interactive demonstration produces only a
small local trace, not predictions for all nine complete paths.

The practical next step is to reproduce the supplied historical outcomes using
the bundled original models and references, establish the exact metric protocol,
and then evaluate Python predictions over a matched scope. The core nine-path
reference data need not be searched for further. The 100-region case index,
published-run settings/preprocessing and exact evaluation-version correspondence
remain unresolved. **LIM-012 remains open; no limitation family is closed.**

## Inspectable local evidence

`outputs/evaluation-audit/` contains the inventory, full LAS label profiles,
source hashes, workbook rows, vector checks, saved-outcome count reconciliation,
runtime import closure and the alignment plot. Reproducible checks are in
`profile_package.py`, `audit_runtime.py` and `crosscheck.py`. Workbooks were read
as XML without modifying formulas/caches. All 199 LAS headers fit their declared
minimum point payload; this is not a full corruption/checksum validation.
