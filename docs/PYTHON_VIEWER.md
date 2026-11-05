# Interactive LAS tracking viewer

The optional Open3D desktop viewer displays a real LAS/LAZ cloud, accepts two
picked positions, previews the fitted paired windows and runs the existing
tracker with Play, Pause, Step and Reset controls. The default local input is
`Vancouver/Thales_11_20Las/16.las`, where the source's sample pose was verified.
It follows the main path inside a bounded crop, then returns to detected switch
points and traces eligible alternate paths. Switch tracing is enabled by default
and uses an explicit fitted-reference recovery described below. The original
reverse/forward UI workflow and Windows menus remain outside the viewer's scope.

## Start the demo

Use the project environment from a graphical desktop terminal:

```bash
source .venv/bin/activate
python -m pip install -e ".[viewer]"
python -m railway_mls view
```

The `railway-mls view` entry point works as well. Viewer dependencies are optional:
the existing computational CLI, API and tests do not import Open3D. The tested
backend is Open3D 0.19 with Python 3.12 on Linux; the viewer extra pins that backend
minor version. LAS/LAZ decoding uses the existing optional laspy/lazrs dependencies.
A usable OpenGL desktop is required. On Linux, an inaccessible X11/XWayland
display produces a CLI error; run from a terminal in the graphical session.

1. Wait for the cloud preview. The header gives the original record count and
   the number of displayed points.
2. Click **Use Vancouver source example**. This loads a crop around the original
   commented pose `(513194.41, 5458237.85, 6.61)`, angle 355, mode 4, and previews
   initialized windows. The preset is available only for files containing it.
3. Review the green/red windows. Expand **Edit position / direction** if needed,
   then click **Confirm start**.
4. Click **Play**, **Pause** or **Step**. Each step runs an actual source-based
   prediction/extraction/fitting/update iteration. Green/red paths show the two
   rails, gold shows the main centerline, purple shows branch centerlines, and
   cyan marks the loaded inference crop. After the main path finishes, Play
   automatically returns the windows to eligible switch points. Earlier paths
   remain visible. Status/details show the active path and totals for each path.
5. Use **Reset / pick again** for a fresh run. **Top view** and **3D view** change
   the camera. Standard mouse dragging/wheel controls orbit, pan and zoom.
6. After a crop-edge stop, click **Restart with larger region**, then **Confirm
   start** and **Play** again. This doubles the radius up to 30 m and replays from
   the saved initial pose, restoring your confirmation edits. The panel scrolls
   if controls or detailed stop messages extend below the window.

For manual initialization, click **Pick two points**, then Ctrl-left-click two
visible cloud points to define the initial midpoint and direction. The original
`initial_location_from_picks` prefix computes the pose. Picks are restored to
world coordinates and snapped to actual displayed records. Try a different
position or edit the fitted seed if the original initializer cannot fit rails.

## Files and size controls

```bash
python -m railway_mls view Vancouver/Thales_11_20Las/16.las
python -m railway_mls view --radius 30
python -m railway_mls view /path/to/scene.laz --radius 8 --max-tracking-points 10000000
python -m railway_mls view --preview-points 250000 --seed 1
python -m railway_mls view --main-only
```

**Open LAS / LAZ** selects another local file. Without a file argument, the
Vancouver preset path is used if present; otherwise the empty viewer provides
the open-file button. The dataset remains ignored by Git.

The default crop radius is 30 m, yielding a 60 m by 60 m region around the initial
seed. Radius can be adjusted from 2 to 30 m before choosing a seed; larger regions
need more memory and computation. A 30 m radius around the source example covers
all of `16.las`, but other files and poses may need different coverage.
This loads 27.7 million inference points on the example tile and takes more
startup time and memory than the earlier 8 m default. Small crops can end before
a switch stack closes, leaving no source-eligible branch starting point.
The default guard permits up to 30,000,000 inference
points. If a region exceeds the budget, reduce its radius or explicitly increase
`--max-tracking-points`. The viewer reports an error instead of thinning inference.

The default full-file preview has at most 150,000 points. This is display-only
sampling, independent of the algorithm cloud. Inference keeps every original
zero-based even LAS record inside the crop, subject to the explicit header-boundary
policy below, preserving the source reader's order, world XYZ and intensity.
It does not use the displayed sample as its input.
The grid uses the original full-file bounds, preserving the source grid origin.
Only the rendering coordinates are centered to avoid float32 precision loss.
No geodetic/UTM transformation is applied, and no CSV pairing is needed.

Full-tile loading exposes two even-index records decoded one float64 ULP below
the declared minimum Y. The viewer omits these records and reports their count
in its details. It permits only discrepancies within four ULPs of the header
bounds; larger discrepancies fail with a metadata error. Header/grid origins and
retained coordinates remain unchanged. This is a viewer input policy documented
in LIM-040; the original reader and computational grid are unchanged.

## Session behavior and scope

`viewer_model.load_preview` and `load_tracking_crop` stream bounded chunks.
`ViewerSession` owns the cropped cloud, grid, original engine and a retained
`SequentialTracker` for each path. Preview initialization and confirm/reinitialization consume
the same RNG stream in source UI order. Pause resumes that same tracker; it does
not repeat initialization or restart the random sequence. Reset clears working
labels and creates a new RNG/session. No LAS file is edited.

The UI submits work to one worker and returns small geometry snapshots to the
main thread. Pause takes effect after the current iteration, so dense windows
may finish computing before the controls report a paused state. Close stops
loading at chunk boundaries and allows a running iteration to finish safely.
Play/Step do not repeatedly call bounded `TrackingEngine.run_pass`, which cannot
resume a partial orchestrated workflow.

### Returning to switches and tracing branches

The original `OnRailextractionTrackingRailway` first runs direction 1, recognizes
completed switch stacks, detects toes and switch boxes, then snapshots the toe
pool. It initializes eligible branch layers from those toes and runs direction 0
with `main_track=False`. The viewer now schedules these events incrementally so
Play/Pause/Step can cross the handoff without restarting the main loop.
The type filters, layer allocation before filtering, copied Y-as-Z seed expression,
shared cloud/labels/RNG, local used flags and final center-vector tagging follow
the source-based engine. Branches process only the main pass's toe snapshot;
they do not recursively discover and schedule new branches.

The source snapshot contains a separate obstacle: it copies switch segments
before filling their member lines (LIM-001). With **Trace other switch paths**
enabled, the viewer supplies those missing lines from the same accepted step's
saved fitted rails, before the toe postpass. It consumes no additional fitting
draws, leaves rejected/nonfinite steps and uninitialized stack distances alone,
and retains the original toe formulas and eligibility thresholds. The interface
reports that fitted references are in use. This is an explicit demo recovery;
its geometry is not a portable result of the native uninitialized reads.

Uncheck **Trace other switch paths** before choosing a seed, or use `--main-only`,
to disable both the recovery and branch scheduling. The computational
`SequentialTracker`, toe helpers and `TrackingEngine.run_pass` remain unchanged
and retain the strict source behavior. No branch pose is hardcoded or supplied
as a fixture to the Vancouver viewer.

Source stopping conditions and undefined-output diagnostics are retained.
Reaching the cyan crop can produce point-shortage stops before the original
scene ends. A trace can also carry documented MDL comparison limitations, and
the optional reference recovery remains tracked under LIM-001. Full-scene
completion, Windows interaction equivalence, paper metrics and annotation
matching require further validation. See [LIM-040](LIMITATIONS.md) and the existing
source entries. Active stacks are not flushed artificially at a boundary; an
incomplete switch region is reported in the viewer.

On a point-shortage stop, the viewer checks the final attempted windows against
the loaded region and LAS header bounds. It reports a crop-edge or file-edge
coverage hint alongside the original stop code. Other algorithm stops retain
their own diagnostics. A boundary hint describes coverage; it cannot guarantee
that usable rail points exist beyond the boundary. Larger-region replay creates
a fresh cloud, grid, labels and RNG and repeats preview/confirmation in the source
order. Pause/Play continues the existing session.

### Observed trace length on the example tile

These earlier main-only runs use the unchanged source pose and RNG seed 1
(`--main-only` reproduces that viewer mode):

| Crop radius | Inference points | Accepted steps | Traced distance | Available coverage |
| --- | ---: | ---: | ---: | --- |
| 3 m, initial viewer default | 1,462,180 | 3 | 2.87 m | Crop edge |
| 8 m, previous default | 8,006,302 | 8 | 7.82 m | Crop edge |
| 30 m, current default/full tile | 27,716,650 | 19 | 18.62 m | LAS tile edge |

All three runs end with `insufficient_window_points` after search lengths 1–6
are tried. Their unaffected prefixes match exactly: the first two steps for
3/8 m, and the first seven for 8/30 m. Larger regions therefore extend this demo
through the multi-line windows. Loading adjacent tiles and validating turnout
recognition remain separate work. The full-tile count includes the two reported
header-rounding omissions. These observations establish demo coverage; native
application equivalence and recognition accuracy remain open.

With switch tracing enabled on the full example tile, actual point-cloud
inference recognizes an eight-segment type-0 stack at score 0.9375, computes one
eligible toe and traces a second path. A later five-segment type-18 stack also
scores 0.9375; the original toe routine does not support that type.

| Path | Accepted steps | Traced distance | End condition |
| --- | ---: | ---: | --- |
| Main path | 19 | 18.62 m | Insufficient window points at tile edge |
| Branch 1 | 16 | 15.73 m | Insufficient window points at tile edge |

The 35 accepted steps use actual tracking calculations. The branch endpoint is
about 2.45 m from the main trace, demonstrating a distinct path. These results
depend on the documented reference recovery and do not establish switch accuracy
or native whole-application parity.

## Verification

```bash
python -m unittest tests.test_viewer tests.test_viewer_branches -v
python -m unittest discover -s tests -v
python -m railway_mls reconstruction-audit --repo .
```

Model/loader tests cover global record parity across odd chunk sizes, crop order,
metadata/coordinates, preview budgets, cancellation, memory-budget failures,
initialization/confirmation RNG order, pause/step continuation, reset, terminal
postpasses, coverage hints, replayed confirmation edits, header-boundary handling
and CLI validation. Branch tests compare the incremental workflow with the
source-based engine's blocking scheduler, including raw inference, exact vectors,
labels, RNG, toe snapshots/type gates, failed/undefined seeds and final tagging.
They also check optional recovery and the unchanged strict baseline.
They use small generated LAS and synthetic
algorithm fixtures. GUI imports are excluded from the numerical core.
The full suite passed 560 tests, including 28 viewer loader/session/CLI checks
and 206 compiled-native comparisons. No tests were skipped. Eleven viewer tests
were added for branch scheduling and recovery; native comparisons continue to
cover the strict computational core.

Desktop acceptance uses the real Vancouver file and native Open3D rendering,
including depth-based picking, seed confirmation, Play/Pause/Step and closure.
Local run evidence and screenshots are saved under `outputs/interactive-viewer/`
when running the desktop check. These files are ignored and are not a substitute
for testing another user's desktop/driver environment.
The initial 3 m Vancouver desktop check displayed 149,820 preview points, initialized a crop
with 1,462,180 source-selected records, resolved a depth-buffer pick back to an
actual world-coordinate record, confirmed the seed, advanced two accepted steps
through Play/Pause/Step and closed normally. Pause retained the RNG state.

The longer-region desktop check played to each natural stop and exercised the
new control twice: radius 8 produced 8 steps/7.82 m, radius 16 produced
16 steps/15.71 m, and radius 30 produced 19 steps/18.62 m. Both restarts restored
the original preview and confirmed poses and reproduced the first two steps
exactly. The final run reported a file-edge hint, two header-rounding omissions
and the existing unavailable-toe-geometry diagnostic. It disabled further region
expansion and closed normally. Evidence is recorded locally in
`outputs/interactive-viewer/extension-gui.json`, `extended-viewer.png` and
`extend_desktop_check.py`; numerical traces are in `trace-radius-*.json`.

The branch desktop check uses the default full-tile region, previews/confirms the
source pose, plays to the main stop, pauses before the switch handoff, verifies
that the RNG does not advance while paused, then steps the branch initialization
and resumes Play. It retains the main geometry, renders both paths, reaches
19/16 accepted steps and closes normally. Evidence is local in `branch-gui.json`,
`branch-viewer.png` and `branch_desktop_check.py`. Thirteen captured segments use
the explicit fitted-reference policy; no toe fixture is inserted into this run.

The GUI thread and point-picking implementation follows the documented
[Open3D application/threading API](https://www.open3d.org/docs/0.19.0/python_api/open3d.visualization.gui.Application.html)
and [SceneWidget callbacks](https://www.open3d.org/docs/0.19.0/python_api/open3d.visualization.gui.SceneWidget.html).
