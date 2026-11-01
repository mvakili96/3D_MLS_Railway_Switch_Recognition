# Branch scheduling and bidirectional orchestration

This increment implements the source engine state around sequential tracking:
main-first execution, toe-to-branch initialization, center-vector tagging, and the
two-direction workflow from `MainForm.cpp`. It preserves the current unpacked C++
and leaves the original files unchanged. Automatic toe references remain subject
to [LIM-001](LIMITATIONS.md#lim-001-uninitialized-toe-reference-lines).

## Run and use

```bash
.venv/bin/python -m railway_mls branch-demo
.venv/bin/python -m railway_mls bidirectional-demo
.venv/bin/python -m railway_mls branch-demo --max-steps 1
```

`branch-demo` uses raw ground/web/head populations in two separated rail corridors
and one **supplied toe geometry fixture**. It exercises actual initialization and
tracking on both corridors: the default seed accepts six main steps and five
branch steps, saving seven/six segments in two layers and consuming 29,600 draws.
It does not simulate an automatically recognized physical junction. A separate
integration check passes a toe produced by the initialized-reference component
into actual branch initialization/tracking.

`bidirectional-demo` uses a 12 m raw corridor with an interior seed. It runs both
directions, reverses/swaps vectors, and appends the forward run to the main layer.
JSON contains the actual seeds, pass/branch statuses, accepted-step counts, local
and global used markers, vector coordinates/types and random-draw count. Ten
stage demos are available; the existing eight remain valid.

```python
from railway_mls import GridIndex, MsvcRand, TrackingEngine
from railway_mls.synthetic import branch_scene

scene = branch_scene()
engine = TrackingEngine(
    GridIndex(scene.cloud, scene.config.grid_size), config=scene.config, rng=MsvcRand(1)
)
initialized = engine.initialize(scene.seed)
if initialized.success:
    engine.toe_points.append(scene.toe)  # Explicit supplied fixture, not a recovered toe.
    result = engine.run_pass()
    print(result.status, len(engine.layers), result.branches[0].tracking.status)
```

For the UI-equivalent workflow, initialize an engine once and call
`engine.run_bidirectional()`. It captures the fitted original location, including
the second rail position, before resetting layers for the reverse run. Do not
run a separate main pass first. All invocations share the engine RNG, owning cloud
and mutable labels. Database/file adapters remain outside this API.

## Source mapping

| Original source | Python entry point | Contract |
| --- | --- | --- |
| `RailwayExtraction.cpp:234–249`, `OnRailextractionSetinit(int, POINT3D, float)` | `TrackingEngine.initialize(reset_layers=True)` | Clear layers, create a current layer, initialize, then retain that layer even on failure; narrow the wrapper angle to float32 |
| `RailwayExtraction.cpp:351–517`, `Setinitiallocation` | `TrackingEngine.initialize(reset_layers=False)` and existing `initialize_tracks` | Append to the current layer; reset engine distance/init flag; update heights/location at source assignment points |
| `RailwayExtraction.cpp:251–336`, `OnRailextractionTrackingRailway` | `TrackingEngine.run_pass` | Main direction 1, snapshot global toes, branch filter/seed/init, direction 0 branch calls, final layer-center tags |
| `RailwayExtraction.cpp:5192–5241` | `tag_rail_vector` | First exact XY match, strict interior projection, then first nearest XY vertex; source mean-endpoint projection Z |
| `RailwayExtraction.cpp:5272–5293`, `ReserverVectors` | `reverse_layer_vectors`, `TrackingEngine.reverse_vectors` | Reverse all layers, swap rails, select layer zero and pop its three vector tails |
| `MainForm.cpp:101–137`, tracking handler | `reverse_seed`, `TrackingEngine.run_bidirectional` | Save original location, reverse seed/run, vector reversal, direct original seed/run; source expressions and unchecked defined initialization failures |

GUI rendering, progress callbacks and layer-list presentation are excluded from
computation. The active switch-box postpass is still pending in the next source
feature increment; the engine does not claim that computation is complete.

## Persistent state and initialization

`TrackingEngine` owns ordered `layers`, `current_layer`, `initial_location`,
`initial_heights`, `distance_from_start`, `completed_stacks`, and `toe_points`.
Its grid/config must agree. Current-layer lookup uses identity, avoiding dataclass
comparisons of shared NumPy selections.

Reset initialization clears layers, not the global toe pool, completed stacks,
cloud labels or RNG. It retains a new empty layer on a defined failure. Direct
initialization appends to the current layer. A failed empty-head initialization
resets distance/init state but retains previous engine heights/location; native
height assignments occur only after the empty-head check. Failed fitted-line use
is reported before undefined geometry is consumed. The engine retains the last
initialization status, so a subsequent tracking call cannot present an undefined
initialization as a defined empty pass. A successful branch
initialization overwrites the engine location just as a main initialization does.

The reset wrapper takes a native `float` angle before calling the `double`
initializer. Python narrows only that wrapper argument using float32, then keeps
double precision throughout computation. Direct branch/forward initialization
retains its double angle. This is relevant when reversing a fitted angle.

## Branch ordering, filters and side effects

1. Invoke the main tracker on the current layer with direction **1**, sharing
   global stacks/toes and distance. A source empty-segment return still proceeds
   to the wrapper's branch loop.
2. After the main postpasses, copy the entire global toe pool by value and reset
   **local** `used` markers. There is no position/type deduplication. Branches
   process only this snapshot; later pool additions do not extend the loop.
3. Create and append a new layer **before** testing each toe type. Skip types
   `2,3,6,7,8,9,10,11,14,15,16,17,18,19`, retaining their empty layers and leaving
   their local used markers false. This exclusion list is independent of the
   twelve types supported by toe detection. Unknown types are not excluded.
4. For eligible toes, construct XYZ as **`(right.x, right.y, right.y)`** and use
   stored toe angle/mode. Do not replace the copied Y with `right.z`. Actual raw
   initialization refits elevations in the integration fixture.
5. If initialization returns true, invoke branch tracking with direction **0**
   and `main_track=False`. Branch tracking resets distance and disables its own
   recognition/toe postpasses. A defined false initialization skips tracking but
   still marks the local toe used. Neither case writes used back into the global
   pool or the layer's saved toe values. No recursive branch scheduling is added.
6. Leave the current-layer pointer at the last allocated layer, even if skipped.
   Finally tag each layer's own toes on that layer's center vector with type 2.

Results contain `TrackingInvocation`, `BranchTracking`, `LayerToeTag` and
`TrackingPass` diagnostics. Layer references remain live: later bidirectional
assembly mutates the same layer objects returned by earlier steps.

## Toe tagging and reversal

Tagging compares XY with exact equality and changes only the first matching
vertex's type, preserving its Z. Otherwise it visits segments in vector order,
projects onto their XY coefficients and inserts the first foot strictly between
both endpoint X bounds **and** both endpoint Y bounds. Projection Z is the mean
of the endpoint heights. Horizontal/vertical segments therefore fall through to
nearest-vertex tagging; nearest ties keep the first vertex. No spline, tolerance,
endpoint extension or nearest-segment replacement is added.

Reversal reverses all three vectors on every layer and swaps rail 1/2 contents,
retaining list identity. It then sets current layer to layer zero and pops one
point from each of its rail1, rail2 and center tails, in that order. Branch layers
are not popped. Segments, boundaries, predicted points, toes, stack order and
labels are not geometrically reversed or cleared. These asymmetries are native
behavior, not an opportunity to rebuild a cleaner merged trajectory.

The UI reverse seed is the fitted second rail position, with mode
`(original.mode + 2) % 4`. Mode **0** is retained. Its expression
`angle + 180 % 360` adds 180 without wrapping the result, after which the reset
wrapper narrows to float32. After reversal, the original captured seed is passed
directly to initialization on the main layer and its main tracking call again
uses direction 1. Source main/branch direction flags control observation choice;
they do not themselves reverse travel geometry.

The forward initialization bool is not a guard in the UI: a defined false result
still leads to tracking from whatever segment history remains. Python retains
that behavior. Undefined fit reads stop the workflow instead of fabricating state.

## Undefined paths and diagnostic limits

- Missing eligible toe right point/angle/mode reports
  `undefined_source_branch_seed`, retaining the new layer and earlier effects.
- A nonfinite branch pose reports `nonfinite_branch_seed` before unsafe grid
  conversion, retaining the source distance/init reset.
- Empty native tagging vectors underflow unsigned `size()-1` and access invalid
  vertices. Python reports `undefined_source_empty_vector` without a fabricated
  point. Missing layer-toe centers are likewise explicit.
- No layers or empty first-layer vectors make native reversal invalid. Python
  retains preceding reversals, swaps and successful pops, returning
  `undefined_source_reverse_layer` or `undefined_source_vector_pop`.
- Reentering tracking without another initialization can read uninitialized local
  next-height variables and unavailable previous segments. The engine reports
  `undefined_source_tracking_reentry`; valid source workflow invocations each
  follow initialization.
- `max_steps` is a diagnostic per-invocation bound. Main limits defer snapshotting,
  branch limits defer used writes and all final tags, and reverse limits defer
  vector reversal/forward execution. A bounded engine workflow is not resumable;
  use a fresh engine for a rerun, or the existing `SequentialTracker` API when
  inspecting/resuming a single tracking loop. Negative limits fail before work.
- Source-created stack references remain unavailable (LIM-001). An undefined toe
  produces no fabricated branch seed; the baseline's explicit handling and
  `reference_comparable=False` remain visible even when orchestration completes.

All findings are retained in [the limitations register](LIMITATIONS.md), including
new lifecycle, bidirectional, container and reentry entries LIM-021–LIM-024. None
is closed by this increment.

## Verification and remaining scope

The complete suite passes **259 test methods**, including **83** using compiled
original C++ extracts. This increment adds 37 methods, including 10 native
comparisons. Native vector tagging/reversal use actual complete source helpers.
The original reset/tracking wrappers and the UI computation body are compiled
with controlled initialization/tracking callbacks to verify scheduling order,
all type gates, unknown types, used-marker copies, layer/pointer state, float32
wrapper arguments, failed initialization and original-seed capture. Those
callbacks isolate control flow; they do not establish whole-application numerical
equivalence. Existing extracts validate the numerical stages separately.

Raw synthetic integrations exercise real main/branch inference, both directions,
center-vector assembly, shared labels/RNG, a detected component toe, actual
postpass-to-scheduler transfer with initialized references, deferred work and
undefined references. Single-candidate MDL paths remain explicitly non-comparable.

Next implement switch-box detection, then remaining source features and final
pipeline delivery/audit. Real-data accuracy/performance and the full native
application remain unverified. The post-reconstruction limitations review remains
a required later checkpoint.
