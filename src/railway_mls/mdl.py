"""The source's greedy one/two/three-line MDL selection and diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
from math import log, log10
from operator import index
from typing import Sequence

import numpy as np

from .config import ExtractionConfig
from .fitting import LineFitResult, fit_fixed_angle
from .geometry import Line3D, distance_2d
from .grouping import PointGrouping, group_points
from .points import PointSelection
from .sampling import MsvcRand, RandSource


@dataclass(frozen=True)
class MDLHypothesis:
    line_count: int
    eligible: bool
    raw_closeness: float | None
    scaled_closeness: float
    complexity: float
    score: float


@dataclass(frozen=True)
class MDLResult:
    lines: tuple[Line3D, ...]
    selected_count: int | None
    candidates: tuple[Line3D | None, ...]
    hypotheses: tuple[MDLHypothesis, ...]
    attempts: tuple[tuple[str, LineFitResult], ...]
    groupings: tuple[PointGrouping, ...]
    uninitialized_candidates: tuple[int, ...]
    status: str

    @property
    def success(self) -> bool:
        """A usable initialized selection, not a claim to emulate undefined reads."""
        return bool(self.lines)

    @property
    def reference_comparable(self) -> bool:
        return self.status == "ok"


def compute_closeness(points: PointSelection, lines: Sequence[Line3D]) -> float:
    """Sum minimum ABSOLUTE XY distances, including the source's 1e100 sentinel."""
    if not len(points):
        return 0.0
    xyz = points.xyz
    minimum = np.full(len(points), 1e100, dtype=np.float64)
    for line in lines:
        with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
            distance = np.abs(
                (line.a * xyz[:, 0] + line.b * xyz[:, 1] + line.c)
                / np.sqrt(line.a * line.a + line.b * line.b)
            )
        take = distance < minimum
        minimum[take] = distance[take]
    return float(np.cumsum(np.abs(minimum))[-1])


def choose_mdl_model(scores: Sequence[float]) -> int:
    """Strict comparisons; every tie/NaN fall-through chooses three, as in C++."""
    if len(scores) != 3:
        raise ValueError("provide the source's three hypothesis scores")
    first, second, third = scores
    if first < second and first < third:
        return 1
    if second < first and second < third:
        return 2
    return 3


def _remaining(points: PointSelection, *groups: PointSelection) -> PointSelection:
    removed = np.concatenate([group.indices for group in groups])
    # Source pointer equality removes all references to an included row, not all
    # geometrically equal points. Preserve order and other duplicate references.
    return points.where(~np.isin(points.indices, removed))


def segment_track_points_mdl(
    points: PointSelection,
    mode: int,
    angle: float,
    *,
    config: ExtractionConfig | None = None,
    rng: RandSource | None = None,
) -> MDLResult:
    """SegmentTrackPtsByMDL, retaining source flags, scores, and membership order.

    Missing candidates have no fabricated geometry. For hypotheses explicitly
    invalidated by the source, use its assigned 1e100 penalty without evaluating
    uninitialized lines. Such selections are flagged and are not claimed to be
    portable native execution results. Fully initialized cases are source-tested.
    """
    settings = config if config is not None else ExtractionConfig()
    rate = index(settings.skip_rate)
    if rate <= 0:
        raise ValueError("skip_rate must be positive")
    minimum_points = 50 // rate
    random = rng if rng is not None else MsvcRand()
    candidates: list[Line3D | None] = [None, None, None]
    flags = [False, False, False]
    attempts = []
    groupings = []
    segments = [points.cloud.select([]) for _ in range(3)]

    def failure() -> MDLResult:
        missing = tuple(i + 1 for i, line in enumerate(candidates) if line is None)
        return MDLResult(
            (),
            None,
            tuple(candidates),
            (),
            tuple(attempts),
            tuple(groupings),
            missing,
            "undefined_source_fit",
        )

    first = fit_fixed_angle(points, angle, settings.max_distance_for_ransac, rng=random)
    attempts.append(("first", first))
    if not first.success:
        return failure()
    candidates[0] = first.line
    first.line.order_endpoints(mode)
    segments[0] = first.support
    first.line.members = segments[0]
    flags[0] = True

    for candidate_index in (1, 2):
        remaining = _remaining(points, *segments[:candidate_index])
        if len(remaining) <= minimum_points:
            continue
        stage = "second" if candidate_index == 1 else "third"
        fit = fit_fixed_angle(remaining, angle, settings.max_distance_for_ransac, rng=random)
        attempts.append((stage + "_raw", fit))
        if not fit.success:
            return failure()
        candidates[candidate_index] = fit.line
        grouping = group_points(fit.support, settings.grouping_distance)
        groupings.append(grouping)
        segments[candidate_index] = grouping.selected
        if len(grouping.selected) > minimum_points:
            refined = fit_fixed_angle(
                grouping.selected, angle, settings.max_distance_for_ransac, rng=random, previous=fit
            )
            attempts.append((stage + "_refined", refined))
            if not refined.success:
                return failure()
            candidates[candidate_index] = refined.line
            refined.line.order_endpoints(mode)
            # Source uses the grown component, not the refined support vector.
            refined.line.members = grouping.selected
            flags[candidate_index] = True

    for number in (1, 2):
        line = candidates[number]
        if line is not None and distance_2d(line.begin, line.end) < 0.2:
            flags[number] = False

    hypotheses = []
    for count in (1, 2, 3):
        prefix = candidates[:count]
        raw = (
            compute_closeness(points, prefix) if all(line is not None for line in prefix) else None
        )
        effective = raw if flags[count - 1] else 1e100
        # A valid third model may include an invalid-but-initialized second line;
        # its closeness overwrites the earlier false-second 1e100 assignment.
        scaled = effective * rate / (2.0 * log(2.0))
        complexity = log10(float(2**count)) / log10(2.0)
        score = settings.mdl_weight * scaled + (1 - settings.mdl_weight) * complexity
        hypotheses.append(MDLHypothesis(count, flags[count - 1], raw, scaled, complexity, score))
    chosen = choose_mdl_model([hypothesis.score for hypothesis in hypotheses])
    selected = candidates[:chosen]
    missing = tuple(i + 1 for i, line in enumerate(candidates) if line is None)
    if any(line is None for line in selected):
        lines = ()
        status = "undefined_source_selected_line"
    else:
        lines = tuple(selected)
        status = "source_uninitialized_intermediates" if missing else "ok"
    return MDLResult(
        lines,
        chosen,
        tuple(candidates),
        tuple(hypotheses),
        tuple(attempts),
        tuple(groupings),
        missing,
        status,
    )
