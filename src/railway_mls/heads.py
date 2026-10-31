"""Active paired rail-head extraction from RailwayExtraction.cpp:1042–1155."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

import numpy as np

from .fitting import RailHeadLineFit, descending_height, fit_rail_head_line
from .gmm import WindowClassification, classify_window
from .models import TrackSegment
from .points import PointSelection
from .sampling import MsvcRand, RandSource


@dataclass(frozen=True)
class HeadWindowResult:
    band_points: PointSelection
    classification: WindowClassification | None
    line_fit: RailHeadLineFit | None
    new_heads: PointSelection


@dataclass(frozen=True)
class TrackHeadResult:
    heads: tuple[PointSelection, PointSelection]
    next_heights: tuple[float, float] | None
    windows: tuple[HeadWindowResult, HeadWindowResult]
    status: str

    @property
    def success(self) -> bool:
        return self.status == "ok"


def filter_height_band(points: PointSelection, current_height: float) -> PointSelection:
    """The source keeps inclusive [previous - 2.5*0.1, previous + 0.1]."""
    if not isfinite(current_height):
        raise ValueError("previous rail height must be finite")
    z = points.xyz[:, 2]
    return points.where((z <= current_height + 0.1) & (z >= current_height - 2.5 * 0.1))


def _refine_candidates(candidates: PointSelection, fit: RailHeadLineFit) -> PointSelection:
    assert fit.line is not None
    line = fit.line
    midpoint, unit = line.center.as_array(), line.unit_vector.as_array()
    difference = midpoint - candidates.xyz
    # DistanceBetweenPointAndLine3DHgt uses only the projected Z residual.
    with np.errstate(invalid="ignore"):
        product = (
            unit[0] * difference[:, 0] + unit[1] * difference[:, 1] + unit[2] * difference[:, 2]
        )
        dz = difference[:, 2] - product * unit[2]
        height_distance = np.sqrt(dz * dz)
    return candidates.where(height_distance < 0.04)


def extract_track_points(
    segment: TrackSegment,
    current_heights: tuple[float, float],
    *,
    rng: RandSource | None = None,
    existing_heads: tuple[PointSelection, PointSelection] | None = None,
) -> TrackHeadResult:
    """ExtractTrkPts, including label mutation and retry accumulation.

    Pass one owned RNG across calls to retain the process's draw sequence. Supplying
    existing_heads reproduces the source's append-without-clear retry behavior,
    including duplicate point references. A failed prerequisite that leads to C++
    uninitialized behavior is explicit: next_heights=None, not an invented fit or
    previous-height fallback. The defined no-heads path does keep both old heights.
    """
    if len(current_heights) != 2 or not all(isfinite(h) for h in current_heights):
        raise ValueError("provide two finite previous rail heights")
    members = segment.track1.members, segment.track2.members
    if any(points is None for points in members):
        raise ValueError("assign both window memberships before extracting heads")
    previous = existing_heads
    if previous is None:
        previous = tuple(points.cloud.select([]) for points in members)
    if len(previous) != 2 or any(
        old.cloud is not points.cloud for old, points in zip(previous, members)
    ):
        raise ValueError("existing heads must reference the corresponding window's owning cloud")
    random = rng if rng is not None else MsvcRand()
    bands = tuple(
        filter_height_band(points, height) for points, height in zip(members, current_heights)
    )
    classifications = tuple(classify_window(points) if len(points) else None for points in bands)
    candidates = tuple(
        classification.head_candidates if classification is not None else points.cloud.select([])
        for classification, points in zip(classifications, members)
    )
    fits = tuple(fit_rail_head_line(points, 0.04, rng=random) for points in candidates)
    new_heads = tuple(
        _refine_candidates(points, fit) if fit.success else points.cloud.select([])
        for points, fit in zip(candidates, fits)
    )
    windows = tuple(
        HeadWindowResult(*values) for values in zip(bands, classifications, fits, new_heads)
    )
    if any(not fit.success for fit in fits):
        return TrackHeadResult(previous, None, windows, "undefined_source_line_fit")

    heads = tuple(
        old.cloud.select(np.concatenate((old.indices, new.indices)))
        for old, new in zip(previous, new_heads)
    )
    for points in heads:
        points.set_class_code(3)
    if any(not len(points) for points in heads):
        return TrackHeadResult(heads, tuple(current_heights), windows, "no_head_points")

    ordered = tuple(descending_height(points) for points in heads)
    heights = tuple(float(points.xyz[int(len(points) * 0.15), 2]) for points in ordered)
    status = (
        "ok"
        if all(result.clustering.complete for result in classifications)
        else "partial_clustering"
    )
    return TrackHeadResult(ordered, heights, windows, status)
