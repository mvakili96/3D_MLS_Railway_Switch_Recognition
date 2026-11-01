"""DetectSwitchBox: toe-adjacent regions, two plane fits, grouping and appends."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace
from math import cos, pi, sin
from operator import index

import numpy as np

from .config import ExtractionConfig
from .geometry import Line3D, points_in_convex_polygon
from .grouping import PointGrouping, group_points
from .models import SwitchBox, ToePoint, TrackLayer
from .planes import PlaneFitResult, fit_plane_ransac
from .points import Point3D, PointSelection
from .sampling import MsvcRand, RandSource
from .spatial import GridIndex


@dataclass(frozen=True)
class SwitchBoxCandidate:
    toe_index: int
    side: str
    region: SwitchBox


@dataclass(frozen=True)
class SwitchBoxCandidates:
    candidates: tuple[SwitchBoxCandidate, ...]
    skipped_toes: tuple[int, ...]
    failed_toe: int | None
    status: str


def switch_box_candidates(toes: Sequence[ToePoint]) -> SwitchBoxCandidates:
    """Build ALL candidate pairs before assigning/evaluating any memberships."""
    candidates, skipped = [], []
    for i, toe in enumerate(toes):
        if toe.used:
            skipped.append(i)
            continue
        if toe.right is None or toe.left is None:
            return SwitchBoxCandidates(
                tuple(candidates), tuple(skipped), i, "undefined_source_switch_box_toe"
            )
        middle = (toe.right + toe.left) / 2
        span = Line3D(middle - (middle - toe.right) * 2.53, middle - (middle - toe.left) * 2.53)
        angle = span.angle
        reference_height = (span.begin.z + span.end.z) / 2
        radians = angle * pi / 180
        offset = Point3D(0.5 * cos(radians), 0.5 * sin(radians), 0)
        for side, center in (("begin", span.begin), ("end", span.end)):
            line = Line3D(center + offset, center - offset)
            region = SwitchBox(center, angle, line.buffer_vertices(1.0), reference_height)
            candidates.append(SwitchBoxCandidate(i, side, region))
    return SwitchBoxCandidates(tuple(candidates), tuple(skipped), None, "ok")


@dataclass(frozen=True)
class SwitchBoxDetection:
    candidate_index: int
    region_points: PointSelection
    height_points: PointSelection
    first_fit: PlaneFitResult | None
    grouping: PointGrouping | None
    second_fit: PlaneFitResult | None
    box: SwitchBox | None
    status: str


@dataclass(frozen=True)
class SwitchBoxBatch:
    candidates: SwitchBoxCandidates
    entries: tuple[SwitchBoxDetection, ...]
    boxes: tuple[SwitchBox, ...]
    new_boxes: tuple[SwitchBox, ...]
    main_track: bool
    status: str

    @property
    def reference_comparable(self) -> bool:
        return self.status in ("completed", "main_track_disabled") and all(
            not entry.status.startswith(("undefined_source", "nonfinite")) for entry in self.entries
        )


def detect_switch_boxes(
    toes: Sequence[ToePoint],
    grid: GridIndex,
    *,
    config: ExtractionConfig | None = None,
    rng: RandSource | None = None,
    layer: TrackLayer | None = None,
    boxes: list[SwitchBox] | None = None,
    main_track: bool = True,
) -> SwitchBoxBatch:
    """Native main-track postpass, including shared labels and append-without-clear.

    Undefined candidate plane reads are reported individually; other candidates
    can still be inspected. Invalid geometry or nonfinite eigen arithmetic stops
    before the source's unsafe conversion/indexing. Earlier effects are retained.
    """
    pool = boxes if boxes is not None else []
    count_before = len(pool)
    candidates, entries = SwitchBoxCandidates((), (), None, "ok"), []

    def finish(status: str) -> SwitchBoxBatch:
        return SwitchBoxBatch(
            candidates,
            tuple(entries),
            tuple(replace(box) for box in pool),
            tuple(replace(box) for box in pool[count_before:]),
            main_track,
            status,
        )

    if not main_track:
        return finish("main_track_disabled")
    settings = config if config is not None else ExtractionConfig()
    if settings.grid_size != grid.grid_size:
        raise ValueError("switch-box configuration must match the grid size")
    skip_rate = index(settings.skip_rate)
    if skip_rate <= 0:
        raise ValueError("switch-box skip_rate must be a positive integer")
    minimum = 1000 // skip_rate
    candidates = switch_box_candidates(tuple(replace(toe) for toe in toes))
    if candidates.status != "ok":
        return finish(candidates.status)
    random = rng if rng is not None else MsvcRand()
    empty = grid.cloud.select([])
    for i, candidate in enumerate(candidates.candidates):
        region = candidate.region
        points = height_points = empty
        first_fit = grouping = second_fit = box = None

        def record(status: str) -> None:
            entries.append(
                SwitchBoxDetection(
                    i, points, height_points, first_fit, grouping, second_fit, box, status
                )
            )

        if not np.isfinite(
            [
                *region.center.as_array(),
                region.angle,
                *(v for point in region.vertices for v in point.as_array()),
            ]
        ).all():
            record("nonfinite_switch_box_candidate")
            return finish("nonfinite_switch_box_candidate")
        neighbors = grid.neighbor_points(region.center.x, region.center.y, neighborhood_size=3)
        points = neighbors.where(points_in_convex_polygon(region.vertices, neighbors.xyz[:, :2]))
        height_points = points.where(
            (points.xyz[:, 2] < region.reference_height + 0.10)
            & (points.xyz[:, 2] > region.reference_height - 0.00)
        )
        first_fit = fit_plane_ransac(height_points, 0.008, rng=random)
        if first_fit.status.startswith("nonfinite"):
            record(first_fit.status)
            return finish(first_fit.status)
        grouping = group_points(first_fit.support, threshold=0.1)
        second_fit = fit_plane_ransac(
            grouping.selected, 0.008, rng=random, plane=first_fit.plane, support=first_fit.support
        )
        if second_fit.status.startswith("nonfinite"):
            record(second_fit.status)
            return finish(second_fit.status)
        if second_fit.plane is None:
            record("undefined_source_switch_box_plane")
            continue
        if abs(second_fit.plane.c) < 0.999:
            record("not_horizontal")
            continue
        if len(grouping.selected) < minimum:
            record("too_few_group_points")
            continue
        # Source uses the pre-second-fit group for labels AND center, not its support.
        selected = grouping.selected
        selected.set_class_code(5)
        center = Point3D(0, 0, 0)
        for xyz in selected.xyz:
            center = center + Point3D(*xyz)
        center = center / len(selected)
        box = SwitchBox(center, region.angle)
        box.generate_region(0.7)
        box.vertices = tuple(replace(vertex, z=center.z) for vertex in box.vertices)
        pool.append(replace(box))
        if layer is not None:
            layer.switch_boxes.append(replace(box))
        record("accepted")
    status = (
        "partial_undefined_source"
        if any(entry.status.startswith("undefined_source") for entry in entries)
        else "completed"
    )
    return finish(status)
