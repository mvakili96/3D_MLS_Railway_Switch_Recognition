"""DetectToePnts and its post-recognition gates, preserving native side effects."""

from __future__ import annotations

from collections.abc import Sequence
from copy import copy
from dataclasses import dataclass, replace

import numpy as np

from .config import ExtractionConfig
from .fitting import LineFitResult, fit_line_ransac
from .geometry import Line3D, direction_mode, distance_2d, perpendicular_foot, source_angle
from .grouping import PointGrouping, group_points
from .models import ToePoint, TrackLayer, TrackSegment, TrackStack
from .points import Point3D, PointCloud, PointSelection
from .sampling import MsvcRand, RandSource


_START_RIGHT = (0, 1, 12, 13)
_START_LEFT = (4, 5)
_END_RIGHT = (6, 7, 14, 15)
_END_LEFT = (10, 11)
SUPPORTED_TOE_TYPES = tuple(sorted((*_START_RIGHT, *_START_LEFT, *_END_RIGHT, *_END_LEFT)))


@dataclass(frozen=True)
class ToeDetection:
    toe: ToePoint
    source_segments: tuple[int, ...]
    side: str | None
    reference_segment: int | None
    candidates: PointSelection | None
    grouping: PointGrouping | None
    fit: LineFitResult | None
    refined_line: Line3D | None
    selected_endpoint: str | None
    selected_point: Point3D | None
    status: str

    @property
    def success(self) -> bool:
        return self.status == "ok"

    @property
    def source_return(self) -> bool | None:
        if self.status.startswith("undefined_source"):
            return None
        return self.status != "unsupported_type"

    @property
    def reference_comparable(self) -> bool:
        return not self.status.startswith("undefined_source")


def detect_toe_points(
    type_id: int,
    segments: Sequence[TrackSegment],
    *,
    config: ExtractionConfig | None = None,
    rng: RandSource | None = None,
    toe: ToePoint | None = None,
    cloud: PointCloud | None = None,
) -> ToeDetection:
    """DetectToePnts, including source grouping, 500-attempt fit and endpoint crop.

    A caller-provided toe is mutated in source order. Input member lines are copied
    before recomputing track1's angle; their stored coefficients remain authoritative
    for projection. A missing reference is reported after preceding grouping/fitting.
    """
    output = toe if toe is not None else ToePoint()
    indices, side, reference = (), None, None
    candidates, grouping, fit, refined, endpoint, selected = None, None, None, None, None, None

    def finish(status: str) -> ToeDetection:
        return ToeDetection(
            replace(output),
            indices,
            side,
            reference,
            candidates,
            grouping,
            fit,
            refined,
            endpoint,
            selected,
            status,
        )

    if type_id not in SUPPORTED_TOE_TYPES:
        return finish("unsupported_type")
    output.track_type = type_id
    if len(segments) < 2:
        return finish("undefined_source_short_stack")
    start = type_id in (*_START_RIGHT, *_START_LEFT)
    right = type_id in (*_START_RIGHT, *_END_RIGHT)
    indices = (0, 1) if start else (len(segments) - 2, len(segments) - 1)
    reference = 0 if start else len(segments) - 1
    side = "track2_right" if right else "track1_left"
    selections = []
    for i in indices:
        points = segments[i].track2.right_members if right else segments[i].track1.left_members
        if points is not None:
            selections.append(points)
    owner = cloud if cloud is not None else selections[0].cloud if selections else None
    if owner is None:
        return finish("undefined_source_toe_fit")  # Empty inputs; no owner to attach diagnostics.
    if any(points.cloud is not owner for points in selections):
        raise ValueError("toe side memberships must share one owning cloud")
    candidates = owner.select(
        np.concatenate([points.indices for points in selections]) if selections else []
    )
    grouping = group_points(candidates, threshold=0.03)  # Hardcoded independently of config.
    settings = config if config is not None else ExtractionConfig()
    fit = fit_line_ransac(
        grouping.selected,
        settings.max_distance_for_ransac,
        rng=rng if rng is not None else MsvcRand(),
    )
    if not fit.success:
        return finish("undefined_source_toe_fit")
    refined = copy(fit.line)
    xy = fit.support.xyz[:, :2]
    minimum, maximum = np.minimum(xy.min(axis=0), 1e100), np.maximum(xy.max(axis=0), -1e100)
    refined.set_ending_points(minimum[0], maximum[0], minimum[1], maximum[1])
    first = segments[reference].track1.member_line
    second = segments[reference].track2.member_line
    # Source first reads the relevant reference BeginPt for endpoint selection.
    relevant = second if right else first
    if relevant is None:
        return finish("undefined_source_reference_line")
    distance1 = distance_2d(relevant.begin, refined.begin)
    distance2 = distance_2d(relevant.begin, refined.end)
    choose_begin = distance1 < distance2 if start else distance1 > distance2
    endpoint = "begin" if choose_begin else "end"
    selected = refined.begin if choose_begin else refined.end
    if first is None or second is None:
        return finish("undefined_source_reference_line")
    # GetAngle ONLY. Updating parameters here would replace cached source coefficients.
    angle = source_angle(first.begin, first.end)
    output.right = perpendicular_foot(selected, first)
    output.left = perpendicular_foot(selected, second)
    output.center = (output.right + output.left) / 2
    output.mode = direction_mode(first.end, first.begin)
    output.angle = angle
    finite = np.isfinite(
        [*output.right.as_array(), *output.left.as_array(), *output.center.as_array(), output.angle]
    ).all()
    return finish("ok" if finite else "nonfinite_toe_geometry")


@dataclass(frozen=True)
class StackToeDetection:
    index: int
    segment_count: int
    score: float
    detection: ToeDetection | None
    status: str


@dataclass(frozen=True)
class ToeBatchResult:
    entries: tuple[StackToeDetection, ...]
    toes: tuple[ToePoint, ...]
    new_toes: tuple[ToePoint, ...]
    main_track: bool

    @property
    def reference_comparable(self) -> bool:
        return all(
            entry.detection is None or entry.detection.reference_comparable
            for entry in self.entries
        )


def detect_stack_toes(
    stacks: Sequence[TrackStack],
    *,
    config: ExtractionConfig | None = None,
    rng: RandSource | None = None,
    cloud: PointCloud | None = None,
    layer: TrackLayer | None = None,
    toes: list[ToePoint] | None = None,
    main_track: bool = True,
) -> ToeBatchResult:
    """Source toe postpass: >4 segments, score not <.9, append without deduplication."""
    pool = toes if toes is not None else []
    count_before = len(pool)
    random = rng if rng is not None else MsvcRand()
    entries = []
    for i, stack in enumerate(stacks):
        count, score = len(stack.track_segments), stack.binary.score
        if not main_track or count <= 4 or score < 0.9:
            status = (
                "main_track_disabled"
                if not main_track
                else "too_short"
                if count <= 4
                else "score_below_threshold"
            )
            entries.append(StackToeDetection(i, count, score, None, status))
            continue
        detection = detect_toe_points(
            stack.binary.type, stack.track_segments, config=config, rng=random, cloud=cloud
        )
        entries.append(StackToeDetection(i, count, score, detection, detection.status))
        if detection.source_return is True:
            pool.append(replace(detection.toe))
            if layer is not None:
                layer.toe_points.append(replace(detection.toe))
    return ToeBatchResult(
        tuple(entries),
        tuple(replace(toe) for toe in pool),
        tuple(replace(toe) for toe in pool[count_before:]),
        main_track,
    )
