# Kalman components and complete seed initialization

This increment implements `Setinitiallocation`, `KalmanPredict`, `KalmanUpdate`,
the bundled matrix operations used by the filter, and the first filter states
prepared at entry to `Trackingrailway`. The active C++ source remains the reference.
The [tracking](PYTHON_TRACKING.md) and [recognition](PYTHON_RECOGNITION.md)
increments now integrate these components.

## Run and use

```bash
.venv/bin/python -m railway_mls initialization-demo
.venv/bin/python -m unittest discover -s tests -v
```

The demo supplies 1,098 raw synthetic ground/web/head points in two oblique rail
neighborhoods. Initialization assigns window members, classifies heights, fits
heads, adjusts the pair and saves one segment with eight boundary edges. Seed 1
consumes 400 raw MSVC random draws, selects 177/180 window members and 118/120
last-component candidates. These are the source GMM's actual selections; the
example does not substitute its known synthetic population labels.

It then demonstrates one prediction/update for each initialized rail, using a
controlled observation offset and sigma 0.02. Those observations are explicitly
synthetic; the example does not run next-window inference or trace a rail path.
All five existing CLI demos remain available.

```python
from railway_mls import (
    GridIndex, MsvcRand, Point3D, direction_mode, initial_kalman_states,
    initialize_tracks, kalman_predict, kalman_update,
)
from railway_mls.synthetic import initialization_scene

scene = initialization_scene()
initialized = initialize_tracks(
    scene.seed, GridIndex(scene.cloud, scene.config.grid_size),
    config=scene.config, rng=MsvcRand(1),
)
if not initialized.success:
    raise RuntimeError(initialized.status)

segment = initialized.segment
first = segment.track1.member_line
mode = direction_mode(first.end, first.begin)
current1, current2 = initial_kalman_states(segment)
predicted = kalman_predict(
    current1, mode=mode, angle=segment.track1.orientation,
    step_length=scene.config.track_window_length,
)
observation = predicted.state.point + Point3D(.01, -.02, .005)
updated = kalman_update(predicted.state, observation, sigma=.02)
```

Use one owned RNG across initialization and subsequent fits. Do not reseed at
each stage. Callers may supply an existing `TrackLayer` through `layer=`;
initialization appends to its vectors, boundaries and segments without clearing it.
`config.grid_size` must match the supplied `GridIndex`.

## Source mapping

Paths are relative to `ADE code/PCLProject/PCLProject/PCLProject/`.

| Source | Python | Preserved behavior |
| --- | --- | --- |
| `RailwayExtraction.cpp:351–515` | `initialization.initialize_tracks` | Entire seed initialization, labels inactive, updated seed and layer appends |
| `RailwayExtraction.cpp:561–580` | `initialization.initial_kalman_states` | Adjusted rail centers with independent identity covariances |
| `RailwayExtraction.cpp:4010–4051` | `kalman.kalman_predict` | Coordinate-dependent diagonal transition and fixed Q |
| `RailwayExtraction.cpp:4070–4089` | `kalman.kalman_update` | Direct sigma, dense H operations and original covariance update |
| `etc/matrix.inl:174–200,503–664` | Private filter matrix helpers | Sequential multiplication and source LU/inverse behavior |
| `RailwayExtraction.cpp:4296–4307` | `TrackLayer.updated_boundaries` | Four endpoint edges per updated window |

## Initialization contract

`initialize_tracks(seed, grid, *, config=None, rng=None, layer=None)` returns
`InitializationResult`. It includes the segment, target layer, classifications,
ordered head selections, pre-adjustment initial heights, both fits, gauge
diagnostics, updated location and status. A supplied `seed.second_point` is not
used to position the second window; the native function derives it from the first
seed, gauge and direction and overwrites the stored second point on success.

Initialization follows this order:

1. Construct the source paired windows, with the second initial center at Z=0.
2. Gather the shared ordered 5x5 grid neighborhood about the pair midpoint and
   assign both memberships using the initial polygons.
3. Run the source three-component height GMM and take **only the last component**
   in each window. This is distinct from later `ExtractTrkPts`, which starts with
   two components and applies height filters/refinement.
4. If either last component is empty, return the source's no-head failure. Otherwise
   sort heads by descending height and store the 15%-index initial heights.
5. Call the 100-attempt fitter twice with **hardcoded threshold 0.04**, independent
   of `config.max_distance_for_ransac`. Order endpoints using the input mode.
6. Apply the basic gauge adjustment with the configured gauge and global window
   length. No +0.02 bias or weighted adjustment is used here.
7. Append adjusted centers and pair midpoint to the layer. Set each window's
   stored orientation, center and member line, then regenerate its vertices using
   its own adjusted endpoint direction. Set vertex Z to its line's begin Z.
8. Refresh segment gauge, retain distance-from-start zero, append eight boundary
   edges and the segment, and return the updated two-point initial location.

The updated location uses the first line's angle and the **second line's mode**,
because that is the final value of the reused native `mode` variable. The initial
tracker mode is derived separately from the first line.

Memberships are **not reassigned** after the windows move. Their point references
remain those selected from the initial polygons. The 100-attempt native fit does
not assign line `MemPts`; Python leaves `member_line.members=None`. Classification
and initialization leave class/segment labels unchanged. Existing axis coefficient
quirks can move adjusted centers far from the original points; no correction or
reclassification is introduced.

Boundary edges preserve source endpoint order, including closing each polygon.
The native visualization edges have uninitialized cached coefficients; Python's
`Line3D` constructor initializes its usual geometry fields. Only edge endpoints are
part of this source comparison; those caches are not used by initialization.

## Filter equations and numerical behavior

`KalmanState(point, covariance=np.eye(3))` owns a read-only float64 covariance of
shape `(3,3)`. Inputs need not be symmetric or positive definite: the source does
not impose those checks. Output states and diagnostic matrices do not alias the
caller's arrays. Nonfinite source arithmetic is retained.

For angle in source degrees, let `dx=abs(step_length*cos(angle))` and
`dy=abs(step_length*sin(angle))`. The transition is diagonal:

```text
Axx = 1 ± (1/current.x)*dx
Ayy = 1 ± (1/current.y)*dy
Azz = 1
```

Signs for modes 1,2,3,4 are `(+,+)`, `(-,+)`, `(-,-)`, `(+,-)`; any other mode
uses the source's final else branch, mode-4 signs. Negative lengths still use
absolute displacement. Prediction uses the actual matrix product `A*point`,
without replacing it by coordinate addition. `Q=diag(.005,.005,.01)`, and
`nextP=A*currentP*A.T+Q`.

This transition changes covariance with the absolute coordinate values. At zero
X/Y the source divides by zero; even a zero displacement can produce `Inf*0` and
NaN. Dense matrix multiplication includes zero terms, so `0*NaN` can spread
nonfinite values. Python reports `nonfinite_prediction` without changing the
equations or moving the origin.

Update constructs `H=I`, **`R=diag(sigma,sigma,sigma)`**, and uses:

```text
K = (predP*H.T) * inverse(H*predP*H.T + R)
hatPoint = predPoint + K*(observation - H*predPoint)
hatP = (I - K*H)*predP
```

Sigma is not squared. The covariance is not replaced with a Joseph form or
symmetrized. The result exposes gain, innovation, measurement noise, inverse valid
flag, zero-pivot count, state and status.

The original `etc/matrix.inl` inverse uses LU arithmetic with several relevant
rules: its merit calculation uses `vv[i]` rather than `vv[j]`; pivot ties select
the last candidate; a zero pivot is replaced by `1e-20`; an all-zero row returns
an initialized zero inverse with a false valid flag. `KalmanUpdate` ignores that
flag and continues. Python retains those values and reports
`source_invalid_inverse` or `source_zero_pivot`. A nonfinite result reports
`nonfinite_update`. No pseudoinverse, library regularization or new singular-case
fallback is used. The small matrix products accumulate k terms in source order.

These diagnostics distinguish finite success from the native function merely
finishing. A singular update can expose an unchanged, finite state while reporting
the source invalid-inverse flag.

## Failure results and scope

- `InitializationResult.success` means status `ok` and finite adjusted geometry.
- `source_return` exposes the native bool separately. An empty head component
  gives `False`; successful native completion gives `True`, including a defined
  nonfinite gauge result. In that latter case Python preserves layer appends and
  updated geometry, but reports `nonfinite_initial_geometry`.
- Empty windows are a defined native no-head path. The empty source GMM reaches
  101 iterations with NaN parameters and returns empty clusters. The general GMM
  API continues to reject empty inputs; initialization represents this exact empty
  outcome explicitly and returns `no_head_points` without fits or layer appends.
- The source ignores both fit return values and can read an uninitialized line.
  Python attempts both fits, retains their diagnostics and initial heights, and
  returns `undefined_source_initial_line` without publishing invented geometry.
  `source_return=None` denotes that undefined native path.
- Equal-height sort ties keep the existing documented stable Python policy.
  Geometry comparisons use distinct-height fixtures to avoid claiming equivalence
  with a compiler-specific tied permutation.

`initial_kalman_states` prepares the first tracking states only. Preserve a further
source detail when integrating the loop: assignments from `hatP` to `curP` are
commented out in `Trackingrailway`. Its current covariances therefore stay at their
initial identity values; a conventional posterior-covariance recurrence would
change that active tracker. Standalone repeated filter calls in the tests validate
the component equations, without changing this future orchestration requirement.

## Verification and next increment

At completion of this increment, the suite passed **143 test methods**, including **48** comparing compiled original
C++ extracts. New checks cover initialization in modes 0–4, changed dimensions,
gauge and random seed, exact point memberships, cached line state, layer vectors,
boundary endpoints, label preservation, empty/no-head returns and nonfinite native
true-return behavior. Filter comparisons include all source direction branches,
large/zero coordinates, full and asymmetric covariances, direct sigma, singular
and nonfinite inputs, pivot diagnostics and repeated component calls.

The reference harness compiles the original filter functions with the **actual
bundled matrix/vector methods they use**. Unused legacy helper methods with
unrelated compile errors are excluded. Mechanical const qualifiers permit GCC to
bind temporary matrix arguments; inverse diagnostics copy its flag and count the
existing zero-pivot substitutions. Arithmetic and pivot branches remain source
code. Native no-model/uninitialized-line cases are not executed as an oracle.
The original source files remain unchanged; the full Windows application is not built.

The [sequential tracking increment](PYTHON_TRACKING.md) now integrates prediction
windows, retries, inference, gauge placement, stopping and stacks, preserving the
source covariance orchestration and inactive loop/cross calls. Templates and stack
classification are available in the [recognition increment](PYTHON_RECOGNITION.md).
Toes, branch/bidirectional orchestration and real-data validation remain later work.
