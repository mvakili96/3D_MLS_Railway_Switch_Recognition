"""Standalone inactive DetectLoopCross/ValidateLoopCross, with literal source rules."""

from __future__ import annotations

from copy import copy
from dataclasses import dataclass, replace
from math import isfinite, sqrt

import numpy as np

from .config import ExtractionConfig
from .fitting import LineFitResult, fit_loop_cross_line
from .geometry import (
    BoundaryLine,
    Line3D,
    distance_2d,
    intersection_point_3d,
    points_in_convex_polygon,
)
from .models import LoopCross, NeighborRegions, TrackSegment
from .points import Point3D, PointSelection
from .sampling import MsvcRand, RandSource


def _snapshot(value: LoopCross) -> LoopCross:
    return replace(value, line=copy(value.line))


def _coefficients_available(line) -> bool:
    return line is not None and all(
        getattr(line, name, None) is not None for name in ("a", "b", "c")
    )


def _distance_xy(point, line) -> float:
    # Literal source scalar division, including initialized zero-normal IEEE cases.
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        return abs(
            float(
                np.divide(
                    line.a * point.x + line.b * point.y + line.c,
                    sqrt(line.a * line.a + line.b * line.b),
                )
            )
        )


@dataclass(frozen=True)
class LoopCrossDetection:
    regions: NeighborRegions | None
    region_points: tuple[PointSelection, PointSelection] | None
    polygon_points: PointSelection | None
    height_points: PointSelection | None
    candidates: PointSelection | None
    fit: LineFitResult | None
    projected_line: Line3D | None
    candidate: LoopCross | None
    duplicate_distances: tuple[float, float] | None
    appended: bool
    loop_crosses: tuple[LoopCross, ...]
    status: str

    @property
    def reference_comparable(self) -> bool:
        return not self.status.startswith(("undefined_source", "nonfinite"))


def detect_loop_cross(
    segment: TrackSegment,
    points: PointSelection,
    ground_height: float | None,
    track_height: float | None,
    reference_line: Line3D | BoundaryLine | None,
    *,
    config: ExtractionConfig | None = None,
    rng: RandSource | None = None,
    loop_crosses: list[LoopCross] | None = None,
) -> LoopCrossDetection:
    """Detect only supplied ordered points; append to the explicit caller pool.

    Native Seg is passed by value: region generation never changes the caller's
    segment. Track height is deliberately unused. No grid lookup, label mutation,
    validation postpass, engine scheduling or replacement rail geometry is added.
    """
    settings = config if config is not None else ExtractionConfig()
    random = rng if rng is not None else MsvcRand()
    pool = loop_crosses if loop_crosses is not None else []
    regions = region_points = polygon_points = height_points = candidates = None
    fit = projected = candidate = distances = None
    appended = False

    def finish(status):
        return LoopCrossDetection(
            regions,
            region_points,
            polygon_points,
            height_points,
            candidates,
            fit,
            projected,
            candidate,
            distances,
            appended,
            tuple(_snapshot(value) for value in pool),
            status,
        )

    if segment.track1.turnout or segment.track2.turnout:
        return finish("turnout_skipped")
    local = copy(segment)
    regions = local.generate_neighbor_regions(
        settings.track_window_length + 0.1, 0.50, settings.track_window_width
    )
    # Source pnpoly2 retains its IEEE behavior even for nonfinite regions.
    with np.errstate(invalid="ignore", over="ignore"):
        region_points = tuple(
            points.where(points_in_convex_polygon(vertices, points.xyz[:, :2]))
            for vertices in (regions.middle_left, regions.middle_right)
        )
    polygon_points = points.cloud.select(np.concatenate([group.indices for group in region_points]))
    if ground_height is None and len(polygon_points):
        return finish("undefined_source_ground_height")
    if len(polygon_points):
        z = polygon_points.xyz[:, 2]
        # Use the two native rejection comparisons; NaN bounds reject nothing.
        height_points = polygon_points.where(
            ~(z > ground_height - 0.1) & ~(z < ground_height - 2 * 0.1)
        )
    else:
        height_points = points.cloud.select([])
    rows = []
    for row in height_points.indices:
        # Both distances are evaluated before either source rejection branch.
        if not all(
            _coefficients_available(window.member_line)
            for window in (segment.track1, segment.track2)
        ):
            candidates = points.cloud.select(rows)
            return finish("undefined_source_loop_track_line")
        point = points.cloud.point(int(row))
        first = _distance_xy(point, segment.track1.member_line)
        second = _distance_xy(point, segment.track2.member_line)
        if first < 0.3 or second < 0.3:
            continue
        rows.append(int(row))
    candidates = points.cloud.select(rows)
    if reference_line is None or reference_line.angle is None:
        return finish("undefined_source_loop_reference_angle")
    angle = reference_line.angle + 90
    if len(candidates):
        # Positive skip_rate remains the existing fitter's public input contract.
        fit = fit_loop_cross_line(candidates, angle, 0.07, skip_rate=settings.skip_rate, rng=random)
    else:
        # Empty native fit returns before reading skip_rate or consuming draws.
        fit = LineFitResult(None, candidates, 0, "too_few_points")
    if fit.line is None:
        # Source reads endpoints and coefficients BEFORE testing the false flag.
        return finish("undefined_source_loop_fit_line")
    if not all(
        _coefficients_available(window.member_line) for window in (segment.track1, segment.track2)
    ):
        return finish("undefined_source_loop_track_line")
    projected = copy(fit.line)
    coefficients = projected.a, projected.b, projected.c
    for attr, window in (("begin", segment.track1), ("end", segment.track2)):
        rail = window.member_line
        intersection = intersection_point_3d(*coefficients, rail.a, rail.b, rail.c)
        old = getattr(projected, attr)
        setattr(projected, attr, Point3D(intersection.x, intersection.y, old.z, old.point_type))
    # Native does not refresh Ang or direction after endpoint projection.
    projected.update_coefficients()
    projected.update_center()
    if not fit.success:
        return finish("fit_rejected")
    if segment.distance_from_start is None:
        return finish("undefined_source_loop_chainage")
    if reference_line.end is None:
        return finish("undefined_source_loop_reference_endpoint")
    candidate = LoopCross(
        copy(projected),
        segment.distance_from_start - distance_2d(projected.center, reference_line.end),
    )
    if pool:
        last = pool[-1].line
        if not _coefficients_available(last):
            return finish("undefined_source_previous_loop_line")
        center = last._center if isinstance(last, Line3D) else last.center
        first = _distance_xy(projected.center, last)
        if center is None:
            return finish("undefined_source_previous_loop_center")
        second = _distance_xy(center, projected)
        distances = first, second
        # Native > 1, not the negation of <= 1: NaN does not append.
        if not (first + second) / 2 > 1:
            return finish("duplicate_suppressed")
    pool.append(_snapshot(candidate))
    appended = True
    finite = np.isfinite(
        [
            *projected.begin.as_array(),
            *projected.end.as_array(),
            *projected.center.as_array(),
            projected.a,
            projected.b,
            projected.c,
            candidate.distance_from_start,
        ]
    ).all()
    return finish("accepted" if finite else "nonfinite_loop_cross_geometry")


@dataclass(frozen=True)
class LoopCrossValidation:
    groups: tuple[tuple[int, ...], ...]
    selected_indices: tuple[int, ...]
    winner_index: int | None
    failed_pair: tuple[int, int] | None
    loop_crosses: tuple[LoopCross, ...]
    status: str

    @property
    def reference_comparable(self) -> bool:
        return self.status in ("ok", "empty")


def validate_loop_cross(loop_crosses: list[LoopCross]) -> LoopCrossValidation:
    """Source integer phase test and first largest star neighborhood, in place.

    On an undefined cast/product the original list is left intact: native replaces
    its vector only after every pair and winner have been evaluated.
    """
    groups, selected, winner, failed = [], (), None, None

    def finish(status):
        return LoopCrossValidation(
            tuple(groups),
            selected,
            winner,
            failed,
            tuple(_snapshot(value) for value in loop_crosses),
            status,
        )

    if not loop_crosses:
        return finish("empty")
    for i, first in enumerate(loop_crosses):
        group = [i]
        for j, second in enumerate(loop_crosses):
            if i == j:
                continue
            failed = i, j
            if first.distance_from_start is None or second.distance_from_start is None:
                return finish("undefined_source_loop_chainage")
            distance = abs(second.distance_from_start - first.distance_from_start)
            if not isfinite(distance) or not 0 <= distance < (1 << 31):
                return finish("undefined_source_loop_distance_cast")
            truncated = int(distance)
            product = truncated * 100
            if product > (1 << 31) - 1:
                return finish("undefined_source_loop_distance_product")
            residual = (product % 2500) / 100
            if residual < 2 or residual > 23:
                group.append(j)
        groups.append(tuple(group))
    failed = None
    maximum = 0
    for i, group in enumerate(groups):
        if len(group) > maximum:
            selected, winner, maximum = group, i, len(group)
    retained = [_snapshot(loop_crosses[i]) for i in selected]
    loop_crosses[:] = retained
    return finish("ok")
