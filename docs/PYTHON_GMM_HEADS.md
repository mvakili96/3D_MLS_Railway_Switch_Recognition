# Height GMM and rail-head extraction

This increment ports the source's height-only EM, classification, and the active
paired rail-head routine. The current unpacked C++ remains authoritative. No
library GMM, variance floor, intensity filter, or paper-based correction replaces
its behavior.

## Run it

Use the editable installation described in the [README](../README.md). From the
repository root:

```bash
.venv/bin/python -m railway_mls heads-demo
.venv/bin/python -m unittest discover -s tests -v
```

The demo uses the same 1,460-point synthetic scene as the geometry demo. Each
window contains 180 points. Extraction selects **60 head points per rail**, writes
class code 3 to 120 cloud rows, and returns heights `(0.2, 0.2)`. Its JSON also
reports GMM parameters, component counts, iterations, stop reasons and raw random
draw count. `heads-demo --seed 1792` selects another explicit CRT seed.

The two previous rail heights are supplied from the synthetic scene's known head
height. This is a local extraction example, not completed initialization or tracing.

## Public API

```python
from railway_mls import GridIndex, MsvcRand, extract_track_points, seed_windows
from railway_mls.synthetic import straight_scene

scene = straight_scene()
segment = seed_windows(scene.seed, scene.config)
GridIndex(scene.cloud, scene.config.grid_size).assign_segment_members(segment)

random = MsvcRand(seed=1)
result = extract_track_points(segment, current_heights=(0.2, 0.2), rng=random)
if result.success:
    first_heads, second_heads = result.heads
    next_heights = result.next_heights
```

Keep the same `random` object for subsequent fits/extractions. On a window-extension
retry, pass `existing_heads=previous_result.heads`; the source appends without
clearing these vectors. Duplicates are preserved, and accumulated points participate
in the returned height percentile. The tracker will own this orchestration later.

Lower-level operations are available independently:

- `fit_height_gmm(heights, n_components=3)` returns `HeightGMMFit`: ordered
  parameters, iteration count, stop reasons and a `finite` diagnostic. It accepts
  a nonempty finite height vector. Three or more components are supported; the
  source's hardcoded checks of the first three means remain in place.
- `cluster_by_gmm(selection, parameters)` returns ordered component selections
  and `failed_position`, retaining the source's partial prefix on classification
  failure. It does not mutate point labels.
- `classify_window(selection)` combines fitting and classification.
  `classify_segment(segment)` does this for both windows, matching
  `ComputeGMMParaAndCluster`. Its `.heads` is the last component; `.head_candidates`
  concatenates the last component and then the penultimate one.
- `filter_height_band(selection, current_height)` keeps the inclusive source band.
- `fitting.fit_rail_head_line(selection, threshold=0.04, rng=...)` implements only
  the required 100-attempt RANSAC overload. The 500-attempt uncertainty-producing
  overload, fixed-angle fitting, connectivity grouping and MDL are available in
  the subsequent [fitting/MDL increment](PYTHON_FITTING_MDL.md).

The returned point selections reference their original cloud. Only the active
`extract_track_points` operation writes class code 3; intensity and segment IDs
are unchanged. Window memberships and previous output selections are not modified.

## Source map and fixed thresholds

Paths are relative to `ADE code/PCLProject/PCLProject/PCLProject/`.

| Source | Python | Rules preserved |
| --- | --- | --- |
| `RailwayExtraction.cpp:3323–3445` | `gmm.fit_height_gmm` | Range-spaced initial means, sigma 1, equal weights, source update and exit order |
| `RailwayExtraction.cpp:3459–3491` | `gmm.cluster_by_gmm` | Unnormalized Gaussian scores, last-match ties, max-element NaN behavior and partial return |
| `RailwayExtraction.cpp:1172–1189` | `gmm.classify_segment` | Last-component head selection; no point-label mutation |
| `RailwayExtraction.cpp:3060–3141` | `fitting.fit_rail_head_line` | 100 attempts, modulo draws, strict support, first best, all-input bounds and 15% descending height |
| `LIDARPT.h:650–692` | Endpoint construction in `fitting` | `SetEndingPnts3D`, including signed slope comparison |
| `RailwayExtraction.cpp:3886–3899` | Z refinement in `heads` | Only projected Z residual is thresholded, with the source unit vector |
| `RailwayExtraction.cpp:1042–1155` | `heads.extract_track_points` | Band, upper-two candidates, 4 cm refinement, labels, height fallback and append behavior |

| Quantity | Source value / condition |
| --- | --- |
| Pre-iteration upper-mean separation | Stop below `0.05` |
| Initial component standard deviations | `1` |
| First-mean convergence | Stop below `0.00001`, without assigning pending parameters |
| Post-update adjacent-mean separation | Stop below `0.005` for pairs 0/1 or 1/2 |
| EM iteration cap | Stop when iteration count is **greater than** 100 (up to 101 steps) |
| Previous-height band | `[previous - 2.5*0.1, previous + 0.1]`, inclusive |
| Required RANSAC attempts | `100`, with two raw draws per attempt |
| Candidate line support | XY distance **strictly less than** `0.04` |
| Refined rail heads | Projected Z residual **strictly less than** `0.04` |
| Line elevation and next rail height | Index `int(N*0.15)` of descending heights |
| Head class code | `3` |

These are source constants, not new tuning defaults inferred from the paper.

## Important source behavior

The EM routine can stop before any iteration when initial upper means are close.
When the first mean converges, it returns the old parameters rather than accepting
the pending M-step. Components are never sorted or renamed. Sequential cumulative
sums preserve the source's sample accumulation order; no convergence likelihood
criterion is added.

Classification chooses the last component on equal scores, including all-zero
underflow scores. If the source's maximum is NaN, classification stops at that
point and retains only the already processed prefix. Nonfinite fitted parameters
are exposed, not silently regularized. With the demo data, the source stops after
one EM step and produces component counts `[60, 0, 120]`; the later geometric
height refinement selects the 60 actual head-height points. Three fitted components
do not guarantee three independently separated semantic populations.

RANSAC consumes two draws even when both sampled indices are equal; that attempt
is skipped rather than retried. Support ties keep the first winner. Endpoints use
all candidate bounds rather than just inlier bounds. Final refinement tests Z only,
so a candidate outside the winning line's XY support can still become a head.
The commented intensity constraints remain inactive.

After both line fits, extracted heads are appended to supplied existing heads,
then all returned references receive class code 3. If either refined list is empty,
both previous heights are retained and neither list is height-sorted. Otherwise
both lists are sorted by descending height and their 15% indices determine next
heights. No configurable minimum head-count threshold is imposed here; the source
tracker checks that separately.

## Random draws and ordering

`sampling.MsvcRand` owns the Microsoft CRT recurrence and uses default seed 1.
The repository targets Windows/MSVC and has no project-owned `srand` call. An
explicit stream avoids silently substituting Linux's C generator or NumPy's RNG.
The generator is checked against the ten values in Microsoft's published
[rand example](https://learn.microsoft.com/en-us/cpp/c-runtime-library/reference/rand?view=msvc-170).
The recurrence is visible in the
[Microsoft UCRT source, mirrored here](https://github.com/huangqinjin/ucrt/blob/master/stdlib/rand.cpp).
This reproduces the selected generator, not unknown earlier draws in an original
application session. Pass a shared instance or replay recorded raw draws when
comparing a sequence of operations.

`RecordedRand` supports replay, including duplicate indices after modulo reduction.
Tests feed the same raw stream to Python and the compiled original source. The
original `rand()` call is redirected only at the test-harness boundary.

C++ `std::sort` leaves equal-height ordering unspecified. Python explicitly uses
stable descending order for those ties. Tests verify exact head order for distinct
heights and membership/multiplicity plus heights for equal-height cases. This is a
documented portability choice; equivalence to a particular compiler's equal-key
permutation is not claimed. It can matter to later randomized stages and should
be considered in future reference experiments.

## Failure results and validation scope

`TrackHeadResult.status` distinguishes:

- `ok`: heads and next heights were produced with complete classifications.
- `no_head_points`: the source's defined empty-refinement path, preserving both
  previous heights and any class updates made to the nonempty side.
- `partial_clustering`: output was produced from the classification prefix, as
  permitted by the source's early classification return; diagnostics identify it.
- `undefined_source_line_fit`: a prerequisite produced too few candidates or no
  valid sampled model. The source continues with uninitialized line/empty-support
  accesses. Python returns `next_heights=None` and diagnostics, without fabricating
  a fitted line, modifying labels or presenting a successful extraction.

Standalone GMM rejects empty/nonfinite input and component counts below three.
The extraction wrapper handles empty bands through the explicit failure result.
These API boundaries and invalid downstream paths are not claimed as matching a
portable C++ result. Degenerate but defined IEEE paths, such as coincident XY
endpoints giving NaN unit vectors and therefore empty refinement, are retained.

The full suite passes **58 test methods**, including the prior foundation checks.
Sixteen methods compare against compiled original source extracts, eight of them
covering this increment. Checks include parameters, memberships, head identities,
labels, heights, retries, supplied raw draws, source NaN behavior and edge thresholds.
The demo and Python lint checks pass. Complete Windows-binary equivalence, runtime
on real MLS clouds and published accuracy remain later experimental work.

The remaining line-fitting variants, grouping and MDL are now implemented in the
[fitting/MDL increment](PYTHON_FITTING_MDL.md). Observation selection and paired-line
adjustment are implemented in the [observation/gauge increment](PYTHON_OBSERVATION_GAUGE.md).
Full initialization and filter components are available in the
[Kalman/initialization increment](PYTHON_KALMAN_INITIALIZATION.md).
The [sequential tracking increment](PYTHON_TRACKING.md) integrates these components.
The full suite currently passes 171 test methods. The next work is recognition.
