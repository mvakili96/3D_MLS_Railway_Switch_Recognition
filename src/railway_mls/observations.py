"""Source observation/side-point selection and support-derived state vectors."""

from __future__ import annotations

from copy import copy
from dataclasses import dataclass
from math import isfinite
from operator import index
from typing import Sequence

import numpy as np

from .config import ExtractionConfig
from .fitting import descending_height
from .gauge import GaugeAdjustment, adjust_two_lines
from .geometry import Line3D, distance_to_line_2d
from .points import Point3D, PointCloud, PointSelection


@dataclass(frozen=True)
class ObservationSelection:
    observations: tuple[PointSelection, PointSelection]
    left_points: tuple[PointSelection, PointSelection]
    right_points: tuple[PointSelection, PointSelection]
    selected_lines: tuple[Line3D | None, Line3D | None]
    selected_indices: tuple[int | None, int | None]
    ordered_indices: tuple[tuple[int, ...], tuple[int, ...]]
    nearest_ranks: tuple[int | None, int | None]
    target_distance: float
    local_adjustment: GaugeAdjustment | None
    status: str

    @property
    def success(self) -> bool:
        return self.status == "ok"


@dataclass(frozen=True)
class SupportObservation:
    point: Point3D | None
    mean: Point3D | None
    support: PointSelection
    status: str

    @property
    def success(self) -> bool:
        return self.point is not None and self.status == "ok"


def observation_from_support(points: PointSelection) -> SupportObservation:
    """Trackingrailway's mean XY plus descending 15%-index Z, before KalmanUpdate.

    The mean uses the incoming support order; sorted support is returned for the
    source's following operations. Empty input produces no state vector, matching
    the tracker's break before filling obsPt (without a fabricated zero point).
    """
    if not len(points):
        return SupportObservation(None, None, points, "no_support")
    mean = Point3D(*(np.cumsum(points.xyz, axis=0)[-1] / len(points)))
    ordered = descending_height(points)
    point = Point3D(mean.x, mean.y, float(ordered.xyz[int(len(ordered) * 0.15), 2]))
    status = "ok" if np.isfinite(point.as_array()).all() else "nonfinite_observation"
    return SupportObservation(point, mean, ordered, status)


def _signed_order(lines: Sequence[Line3D], guide: Line3D) -> tuple[tuple[int, ...], int]:
    direction = guide.end - guide.begin
    values = []
    for line in lines:
        offset = line.center - guide.begin
        distance = distance_to_line_2d(line.center, guide)
        if direction.x * offset.y - direction.y * offset.x > 0:
            distance = -distance
        if not isfinite(distance):
            raise ValueError("source signed ordering requires finite guide distances")
        values.append(distance)
    ordered_values = sorted(values)
    # The source rescans from the beginning for EVERY sorted value. Equal values
    # repeat the first matching line; they do not produce a stable permutation.
    ordered_indices = tuple(
        next(i for i, value in enumerate(values) if value == ordered) for ordered in ordered_values
    )
    absolute = [abs(value) for value in ordered_values]
    nearest = next(i for i, value in enumerate(absolute) if value == min(absolute))
    return ordered_indices, nearest


def determine_observation(
    lines1: Sequence[Line3D],
    lines2: Sequence[Line3D],
    guide1: Line3D,
    guide2: Line3D,
    *,
    track_mode: int = 0,
    stack_size: int = 0,
    line_mode: int = 1,
    config: ExtractionConfig | None = None,
    cloud: PointCloud | None = None,
) -> ObservationSelection:
    """DetermineObservation, preserving directional asymmetries and exact ties.

    The two candidate lists contain one to three initialized lines. stack_size is
    the source's trkStk.trkSegs.size(). All memberships refer to one owning cloud;
    provide cloud explicitly if every line has empty/unassigned members.
    """
    settings = config if config is not None else ExtractionConfig()
    count = index(stack_size)
    if count < 0:
        raise ValueError("stack_size must be nonnegative")
    candidates = tuple(lines1), tuple(lines2)
    if any(not 1 <= len(lines) <= 3 for lines in candidates):
        raise ValueError("each rail window requires one to three initialized candidate lines")
    memberships = [
        line.members for lines in candidates for line in lines if line.members is not None
    ]
    owner = cloud if cloud is not None else (memberships[0].cloud if memberships else None)
    if owner is None or any(points.cloud is not owner for points in memberships):
        raise ValueError("provide one owning cloud for all candidate memberships")
    empty = owner.select([])
    observations = [empty, empty]
    left, right = [empty, empty], [empty, empty]
    selected = [None, None]
    selected_indices = [None, None]
    ordered = tuple(tuple(range(len(lines))) for lines in candidates)
    nearest = (None, None)
    target = settings.track_gauge + 0.02

    def result(status: str, adjustment: GaugeAdjustment | None = None) -> ObservationSelection:
        if status == "ok" and any(not len(points) for points in observations):
            status = "empty_observation_members"
        return ObservationSelection(
            tuple(observations),
            tuple(left),
            tuple(right),
            tuple(selected),
            tuple(selected_indices),
            ordered,
            nearest,
            target,
            adjustment,
            status,
        )

    def members(line: Line3D) -> PointSelection:
        return line.members if line.members is not None else empty

    def pick(rail: int, rank: int) -> None:
        original = ordered[rail][rank]
        selected_indices[rail] = original
        selected[rail] = candidates[rail][original]
        observations[rail] = members(selected[rail])
        number = len(candidates[rail])
        if rank != 0:
            left[rail] = members(candidates[rail][ordered[rail][0]])
        if rank != number - 1:
            right[rail] = members(candidates[rail][ordered[rail][-1]])

    def match(rail: int, anchor: Line3D) -> bool:
        rails = [candidates[rail][position] for position in ordered[rail]]
        if len(rails) == 1:
            pick(rail, 0)
            return True
        errors = [abs(target - distance_to_line_2d(line.center, anchor)) for line in rails]
        if len(rails) == 2:
            pick(rail, 0 if errors[0] < errors[1] else 1)
            return True
        for rank in range(3):
            if all(errors[rank] < errors[other] for other in range(3) if other != rank):
                pick(rail, rank)
                return True
        return False  # Source leaves the other obsLn uninitialized for this tie.

    # This return precedes sorting and the discarded local gauge adjustment.
    if len(candidates[0]) == len(candidates[1]) == 1:
        pick(0, 0)
        pick(1, 0)
        return result("ok")
    first_order, first_nearest = _signed_order(candidates[0], guide1)
    second_order, second_nearest = _signed_order(candidates[1], guide2)
    ordered, nearest = (first_order, second_order), (first_nearest, second_nearest)
    size1, size2 = len(candidates[0]), len(candidates[1])
    if track_mode == 0:
        if size1 == 1:
            pick(0, 0)
            matched = match(1, selected[0])
        elif size1 == 2 and size2 == 1:
            pick(1, 0)
            matched = match(0, selected[1])
        else:
            pick(0, nearest[0] if count < 2 else size1 - 1)
            matched = match(1, selected[0])
    else:
        if size2 == 1:
            pick(1, 0)
            matched = match(0, selected[1])
        elif size2 == 2 and size1 == 1:
            pick(0, 0)
            matched = match(1, selected[0])
        else:
            pick(1, nearest[1] if size2 == 3 or count <= 2 else 0)
            matched = match(0, selected[1])
    if not matched:
        return result("unresolved_gauge_tie")
    # Source adjusts local line copies at the end, but only point vectors are
    # returned. Preserve this computation as diagnostics, without changing models
    # or treating the locally adjusted lines as caller-visible observations.
    local = adjust_two_lines(
        copy(selected[0]),
        copy(selected[1]),
        line_mode=line_mode,
        target_distance=target,
        config=settings,
    )
    return result("ok", local)
