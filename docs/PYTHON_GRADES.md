# Grade, curve and GWAD calculations

This increment ports the separately invoked grade-analysis feature, its report
calculations and the computational GWAD export loop. It follows the unpacked C++
formulas and tag mutations, including their unusual geometry. It does not add
grade analysis to the automatic tracking loop.

## Run and use

```bash
.venv/bin/python -m railway_mls grade-demo
```

The fixture is a supplied 17-point center vector with a horizontal curve, three
elevation regimes and two switch tags. Source analysis marks indices 0, 6, 10 and
16, produces three sections/four report rows, and builds six GWAD rows. This is a
post-tracking vector example, not a raw-cloud accuracy benchmark.

```python
from railway_mls import compute_grade, build_gwad_rows, format_gwad
from railway_mls.synthetic import grade_scene

layer = grade_scene()
analysis = compute_grade(layer)  # Mutates center-vector PtType values, not XYZ.
print(analysis.status, len(analysis.sections))
export = build_gwad_rows([layer], ["Main line"])
text = format_gwad(export)       # Produces text; does not write a file.
```

`engine.compute_grades()` reproduces the grade menu's ordered call over all engine
layers, including empty layers. `compute_layer_grades(layers)` provides the same
operation without an engine. `GradeAnalysis` retains the live layer, division
diagnostics, section snapshots and report rows. There is no new global grade pool,
random sampling, cloud reclassification or automatic recomputation.

## Source mapping

| Source | Python | Contract |
| --- | --- | --- |
| `LIDARPT.h:1254–1289`, `GradeSect` | `models.GradeSection` | Zero-initialized fields, value memberships and reset |
| `RailwayExtraction.cpp:5035–5077`, `ComputeGrade` | `grades.compute_grade` | Fixed threshold 1, marker mutation, parameters and report rows |
| `RailwayExtraction.cpp:5079–5091`, `DetermineGradeSection` | `determine_grade_sections` | Empty return; endpoints marked 1 before simplification |
| `RailwayExtraction.cpp:5093–5134`, `DouglasPeuckerVertical` | `douglas_peucker_vertical` | X/Z projection, first strict maximum, strict threshold and left-first recursion order |
| `RailwayExtraction.cpp:5136–5189`, `ComputeGradeParameters` | `compute_grade_parameters` | Clear outputs, shared boundaries, XY arc/chord, height, source radius/degree/compensation |
| `MainForm.cpp:824–836`, grade menu | `compute_layer_grades`, `TrackingEngine.compute_grades` | Ordered calls for all layers, no selected-layer filter |
| `MainForm.cpp:724–800`, GWAD export computation | `build_gwad_rows`, `format_gwad` | Compute grades again, 3D chainage, marker selection, all endpoint matches, fixed-six-decimal TSV |

The public API also exposes `GradeDivision`, `GradeSplit`, `GradeParameters`,
`GradeReportRow`, `GWADRow` and `GWADExport`. `compute_grade_parameters(...,
sections=existing_list)` clears and fills the supplied list as the native output
reference does. It does not add grade tags; that belongs to the division call.

## Vertical division and marker state

The source swaps Y/Z in a temporary vector and measures XY line distances there:
the effective simplification plane is **world X versus Z**, not chainage versus
height or full 3D. For constant-X paths, arbitrary vertical deviations can receive
zero X/Z line distance. Rotation of the same physical profile can therefore change
sectioning. The line uses the existing endpoint-coefficient rules.

Endpoints become type **1**, even when they previously had switch type **2**.
Other old type-1 markers are never cleared. Interior split points are also set to
1 without protecting switch tags. Every span keeps the first strictly greatest
distance, with initial maximum `-1e100`; NaNs fail the comparison. Split only when
the selected index is >0 and its distance is **strictly greater** than threshold.
The left span precedes the right. Python uses an iterative span stack and
vectorized distances to preserve that order without copying the full vector on
every recursion or reaching Python's recursion limit.

The standalone division helper accepts the native threshold argument. `ComputeGrade`
always uses **1** in input coordinate units. A later larger threshold does not
remove markers written by an earlier call. `GradeDivision` distinguishes all
marked indices, newly changed indices and split visitation diagnostics.

## Membership, endpoint tags and formulas

Only type-1 vertices split sections. A boundary vertex ends its previous section
and starts the next. Leading/trailing unmarked runs are retained when they contain
at least two vertices. A single point yields no section; the full operation still
marks it as an endpoint. Type-2 vertices inside a section remain members.

`POINT3D` copy construction copies its tag, while its assignment operator copies
only XYZ. Native grade-section endpoint objects start at tag 0, so **section
begin/end tags stay 0**, while member vertices retain their tags. Python represents
that distinction explicitly; coordinate equality still ignores tags.

For each section, the source computes:

```text
chord = XY distance(begin, end)
arc = sequential sum of XY distances between consecutive members
maxh = maximum XY distance of members[:-1] to the infinite chord line
height = end.z - begin.z
grade = height / arc * 100
radius = chord*chord - 8*maxh + 4*maxh*maxh
degree_of_curve = 360*arc / (2*pi*radius)
delta = 0.04 * degree_of_curve
grade_with_curve = grade + delta if delta > 0 else grade - delta
```

The radius expression is retained literally. In particular, a straight section
with `maxh=0` has radius `chord²` and a nonzero curve correction. Negative radii,
negative grades and the compensation branch are not normalized. Zero arc/radius
and nonfinite inputs retain defined IEEE NaN/Inf values. Their diagnostic status
is `nonfinite_grade_parameters`; sections and report rows are not dropped or
replaced. Such results are checked against C++, including matching NaNs.

Report rows reproduce the source's first/boundary/last calculations with full
precision. Distances accumulate **XY arcs**, before/after grades use missing
values at the two ends, and differences follow the native signs. Native console
printing rounds to two decimals; the API retains the underlying values.

## GWAD row and text behavior

GWAD computation invokes the full grade calculation again on each layer. It
accumulates **3D** distances along every consecutive center vertex, even vertices
that do not produce rows. Only resulting tags 1 and 2 are emitted as `GR` and
`SW`. Each supplied layer name prefixes its numeric chainage as `name+distance`.

For a GR point, every section is checked in order with `POINT3D::IsEqual`'s strict
`1e-5` XYZ tolerance. Begin match wins over end match within a section. Begin adds
`(blank, grade)`; end adds `(grade, blank)`. There is no early exit after a match.
Consequently ordinary internal boundaries can append four parameter values;
coincident geometry can match more sections. SW rows, and GR rows without a
matching section, add no parameter values.

The header has seven columns, but **rows can have five, seven, nine or more**.
The formatter preserves this source layout, trailing empty fields, fixed six
decimals and a final newline. It does not silently normalize to two parameters.
Supplied names must cover all layers before any mutation occurs. Nonfinite values
use Python's `nan`/`inf` text; native NaN spelling/sign and file newline translation
are platform-dependent. Finite in-memory TSV is compared to the original UI loop.
Database/file adapters and byte-level Windows export compatibility remain separate.

## Tagged-vector interoperability and limitations

This increment's comparisons exposed the effect of XYZ-only assignment on the
existing reversal helper: native `std::reverse` reverses XYZ while **tags stay at
their original vector indices**. Subsequent rail-container swap moves whole tag
sequences; only the main layer loses its three tails. Python reversal now matches
that behavior, with a marked-vector C++ regression check.

Native vector insertion has a further capacity-dependent behavior: with spare
capacity, assignments can leave shifted coordinates carrying destination tags;
reallocation can copy-construct tags with their points. Python preserves value
transport on insertion. If a shifted tail contains mixed tags, `VectorTag` now
returns `reference_comparable=False`, propagated to the tracking pass. Uniform
tail tags do not introduce this difference. The target-runtime capacity policy is
unresolved; no guessed Windows capacity model is introduced.

[LIM-028–LIM-031](LIMITATIONS.md) retain the projection/marker, curve/numerical,
GWAD-layout and point-assignment findings for review after reconstruction. The
existing toe/MDL/switch-box limitations also remain open. Passing tests does not
close these review items.

## Verification and remaining work

The suite passes **324 test methods**, including **108** using original C++
extracts. This increment adds 30 methods, including 12 native comparison methods.
It compiles the original grade struct/four functions and both computational UI
loops. Diagnostics only copy existing split, offset and report values; GWAD tests
replace dialogs/names/file streams with supplied names and an in-memory stream.

Checks cover empty/singleton inputs, ties/thresholds/subspans, old/switch markers,
membership snapshots, large origins, world-axis dependence, positive/negative/zero
radius, IEEE outputs, endpoint tags, report signs/distances, GWAD names/precision/
variable fields and marked reversal. Raw tracking integration verifies explicit
analysis, preserved XYZ/cloud labels/random state, and export rows afterward.

Next audit the remaining computational inventory and complete the integrated
synthetic pipeline and input/diagnostic/export interfaces. Published accuracy,
real-data performance and whole Windows-binary equivalence remain unverified.
The complete limitations review remains a later required checkpoint.
