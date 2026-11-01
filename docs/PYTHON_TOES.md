# Toe detection and endpoint refinement

This increment ports `DetectToePnts` and its post-recognition caller. It preserves
the twelve supported types, side-point choices, grouping, fitting, endpoint crop,
distance comparisons, projections and append behavior. Branch scheduling, switch
boxes and bidirectional orchestration remain later work.

**Baseline limitation:** the source tracker pushes `nextSeg` into a turnout stack
before assigning its member lines. `Line3D` has an empty constructor, so those
stored references are uninitialized. `DetectToePnts` later reads them. Python's
stack snapshots retain `member_line=None` and report
`undefined_source_reference_line` when grouping/fitting have succeeded up to that
read. Earlier undefined fit paths have their own diagnostic. The detection formulas
are validated on explicitly initialized references; this does not establish valid
automatic toes from the native tracker's undefined snapshots.

This remains open as **LIM-001** in the persistent
[limitations register](LIMITATIONS.md#lim-001-uninitialized-toe-reference-lines),
scheduled for review after full reconstruction.

## Run and use

```bash
.venv/bin/python -m railway_mls toe-demo
.venv/bin/python -m railway_mls toe-demo --type 6
.venv/bin/python -m unittest discover -s tests -v
```

The default component example processes all twelve supported source types, with
60 selected side points per case and initialized reference lines. It produces
twelve toes and consumes 12,000 raw MSVC random draws. A separate missing-reference
example reports the undefined path after its own 1,000 draws. These fixtures enter
at the side-membership stage; they are not automatically tracked raw MLS regions.
All eight CLI demos remain available.

```python
from railway_mls import MsvcRand, TrackLayer, detect_stack_toes, detect_toe_points
from railway_mls.synthetic import toe_scene

scene = toe_scene(type_id=0)
result = detect_toe_points(
    0, scene.stack.track_segments, config=scene.config, rng=MsvcRand(1),
)
if result.success:
    center = result.toe.center

layer = TrackLayer()
batch = detect_stack_toes(
    [scene.stack], config=scene.config, rng=MsvcRand(1),
    cloud=scene.cloud, layer=layer,
)
```

Use one owned RNG across a real initialization/tracking/recognition/toe run. The
separate streams above demonstrate independent calls. `toe_scene` deliberately
supplies initialized references and four distinct side-point banks; its stored
recognition tags correspond to the requested source template.

## Source mapping

Paths below are relative to `ADE code/PCLProject/PCLProject/PCLProject/`.

| Source | Python | Preserved behavior |
| --- | --- | --- |
| `LIDARPT.h:1169–1182` | `models.ToePoint` | Type -1, used false; uninitialized point/angle/mode fields represented by None |
| `RailwayExtraction.cpp:2617–2774` | `detect_toe_points` | Four type branches, grouping, 500-attempt fit, crop, endpoint choice and projection |
| `LIDARPT.h:612–653` | `Line3D.set_ending_points` | Endpoint XY replacement only; retained Z, coefficients, angle and cached center |
| `RailwayExtraction.cpp:3729–3751` | `perpendicular_foot` | Stored coefficients and mean endpoint Z, including defined IEEE divisions |
| `RailwayExtraction.cpp:990–1005` | `detect_stack_toes` | Main-track, stack-length and score gates; append to engine and layer lists |
| `RailwayExtraction.cpp:709–713,801–802` | Existing tracking snapshots | Source ordering that leaves stacked member lines uninitialized |

## Component contract and source branches

`detect_toe_points(type_id, segments, *, config=None, rng=None, toe=None,
cloud=None)` returns a `ToeDetection`. If supplied, `toe` is mutated in source
order. The result holds a value snapshot plus source segment indices, selected
side, reference segment, concatenated candidates, grouping result, fit, cropped
line, chosen endpoint and status. Reference lines themselves are not mutated.

| Types | Side memberships | Input segments | Reference segment | Endpoint rule |
| --- | --- | --- | --- | --- |
| 0, 1, 12, 13 | track2 right | first two | first | nearest |
| 4, 5 | track1 left | first two | first | nearest |
| 6, 7, 14, 15 | track2 right | last two | last | farthest |
| 10, 11 | track1 left | last two | last | farthest |

The distance reference is that side's selected rail line **begin** point. Both
nearest and farthest comparisons are strict; an exact tie chooses the cropped
line's **end**. Left/right-labelled template variants keep the same source side
selection within each row above. Types labelled "both" still produce one toe per
call, as the source does.

Types 2,3,8,9,16,17,18,19 and unknown/999 values return the defined native false
result before changing the output or consuming randomness. Supported direct calls
need at least two segments; the separate batch applies its stricter >4 gate.

The routine concatenates the first selected membership, then the second. It
runs the existing source point growing with **hardcoded XY threshold 0.03**,
independent of `config.grouping_distance` and skip rate. Grouping keeps its source
first-component bias, >100 retention rule for later components, pointer identity
and shared SegmentID changes. Class codes are unchanged. Discarded components can
still have visited IDs set to zero. Repeated references are not new point identities.

The selected component enters the **500-attempt** fitter using
`config.max_distance_for_ransac`. It retains the source support threshold, first
winning model, descending 5%-index height, sigma and all-input endpoint bounds.
Toe refinement then clips endpoint XY to the bounds of the **returned support**
using `SetEndingPnts`; it does not refit. The signed-slope branches remain intact.
Endpoint Z, angle, coefficients and the pre-crop cached center remain unchanged.
The result keeps both raw fit and cropped-line diagnostics for inspection.

Finally, the chosen point is projected onto track1 and track2 using their stored
line coefficients. `right` always comes from track1 and `left` from track2. Each
projection's Z is the mean of that reference line's begin/end heights, not the
fitted side-point height. The toe center is their full XYZ midpoint. Track1's angle
is recomputed **without refreshing its coefficients**, and track1's endpoint
direction supplies the toe mode. A previously true `used` marker is not reset.

## Postpass and append behavior

`detect_stack_toes(stacks, *, config=None, rng=None, cloud=None, layer=None,
toes=None, main_track=True)` consumes classified completed stacks. It applies:

- Main-track gating: disabled calls do not classify or modify points/toes.
- More than four stack segments.
- Skip only when **score < 0.9**. Equality passes. The exact native comparison also
  allows a NaN score through; no extra confidence or finiteness rule is added.
- The component's supported-type check. A high-scoring ambiguity code 999 remains
  unsupported and appends no toe.

Fresh successful outputs have `used=False`. Caller-provided engine storage through
`toes=` is appended without clearing or deduplication; a supplied layer receives
separate value copies in `layer.toe_points`. Existing used markers remain intact.
`ToeBatchResult.toes` snapshots the full engine pool, `new_toes` contains only this
call's additions, and entries retain each stack's gate/detection diagnostic.

`SequentialTracker` runs this postpass immediately after recognition on natural
completion, using the same RNG and configuration. Step limits defer both
postpasses. Non-main calls retain their gates. Undefined/nonfinite loop exits and
empty-layer calls do not invoke the postpass. The active unflushed stack remains
excluded. Results expose `toe_detection`; undefined toe paths also make
`TrackingResult.reference_comparable=False`. Branch and switch-box routines are
not invoked by this increment.

## Undefined, nonfinite and boundary cases

`ToeDetection.success` means finite status `ok`. Its `source_return` is separate:
false for unsupported types, None for undefined native paths, and true for defined
completion, including nonfinite output. The principal statuses are:

- `undefined_source_short_stack`: supported type cannot access the requested pair.
- `undefined_source_toe_fit`: grouping/fitting does not initialize a usable line;
  source code ignores the fitter's return and proceeds to read it.
- `undefined_source_reference_line`: required stacked member line is uninitialized.
  Grouping labels, fit diagnostics, crop state and consumed draws are retained.
- `nonfinite_toe_geometry`: initialized projection arithmetic produces NaN/Inf.
  The native return is still true, so batch append behavior is preserved.

An output supplied by the caller retains any fields the source has not yet written
when an undefined path is reported; its updated type alone does not establish valid
new geometry. Empty selections require no fabricated owner cloud. Providing the
owning cloud allows full empty grouping/fit diagnostics. Mixed membership owners
are rejected at the Python boundary.

The shared projection helper now preserves native IEEE division for explicit zero
coefficients. Finite geometry remains source-tested; no coefficient repair or
replacement reference is introduced. Reference snapshots from the tracker still
contain None, as required by the source ordering. Successful initialized-reference
fixtures therefore establish component behavior, not automatic recovery of those
missing source references.

## Verification and next increment

The suite passes **222 test methods**, including **73** comparing compiled original
C++ extracts. Nine new native comparison methods cover all twelve supported types,
all four geometric modes, horizontal/vertical/steep lines, nearest/farthest rules,
exact endpoint ties, support cropping/cached state, duplicate identities, component
retention, cached reference coefficients, nonfinite projections, rejected types,
score/length/main-track gates and appends with existing used markers. Point labels,
support IDs, refined line state, output geometry and raw-draw counts are compared.

The harness compiles the actual original toe struct and complete detection function,
plus the exact postpass through the point before switch-box detection. Diagnostic
assignments copy existing vectors, line state and sigma without changing branches
or arithmetic. Only initialized, defined inputs are used as native oracles.
Integration tests use a shared global cloud with a completed initialized-reference
fixture from another region, verify actual toe appends after tracking, and check
the deferred and undefined-reference paths. The original source files remain
unchanged; full Windows-binary and empirical accuracy claims remain unverified.

Next implement branch seed selection and tracking orchestration, followed by
bidirectional runs, switch-box detection and remaining source features. Preserve
the documented missing-reference limitation when consuming source toe results.
