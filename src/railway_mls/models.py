"""The first implemented subset of LIDARPT.h's window/segment/layer state."""

from __future__ import annotations

from dataclasses import dataclass, field
from collections.abc import Iterable
from math import cos, fabs, pi, sin
from operator import index

import numpy as np

from .config import ExtractionConfig
from .geometry import Line3D, distance_2d, points_in_polygon
from .points import Point3D, PointSelection


@dataclass(frozen=True)
class InitialLocation:
    point: Point3D
    angle: float
    mode: int
    second_point: Point3D | None = None


@dataclass
class GMMParameters:
    mu: float = 0.0
    sigma: float = 0.0
    alpha: float = 0.0


@dataclass
class TrackWindow:
    center: Point3D
    orientation: float
    vertices: tuple[Point3D, ...] = ()
    left_vertices: tuple[Point3D, ...] = ()
    right_vertices: tuple[Point3D, ...] = ()
    main_vertices: tuple[Point3D, ...] = ()
    members: PointSelection | None = None
    left_members: PointSelection | None = None
    right_members: PointSelection | None = None
    member_line: Line3D | None = None
    turnout: bool = False
    left_tag: int = 0
    right_tag: int = 0

    def reset(self) -> None:
        """ResetTrackWindow leaves side polygons/members and pose untouched."""
        self.vertices = ()
        self.members = None
        self.turnout = False
        self.left_tag = 0
        self.right_tag = 0

    def generate_vertices(self, length: float, width: float, mode: int) -> None:
        angle = self.orientation * pi / 180
        dx, dy = fabs(length / 2 * cos(angle)), fabs(length / 2 * sin(angle))
        if mode == 1:
            offset = Point3D(dx, dy, 0)
        elif mode == 2:
            offset = Point3D(-dx, dy, 0)
        elif mode == 3:
            offset = Point3D(-dx, -dy, 0)
        else:
            offset = Point3D(dx, -dy, 0)
        line = Line3D(self.center - offset, self.center + offset)
        self.vertices = line.buffer_vertices(width / 2)
        # The source only writes vertices here, not MemLn or ori.

    def assign_members(self, candidates: PointSelection) -> PointSelection:
        """SetTrackWindowMemPnts preserves candidate order and ignores height."""
        if not self.vertices:
            raise ValueError("generate window vertices before assigning members")
        self.members = candidates.where(points_in_polygon(self.vertices, candidates.xyz[:, :2]))
        return self.members

    def member_bounds(self) -> tuple[Point3D, Point3D]:
        """Return (maximum, minimum), including the source's empty sentinels."""
        if self.members is None or not len(self.members):
            return Point3D(-1e100, -1e100, -1e100), Point3D(1e100, 1e100, 1e100)
        xyz = self.members.xyz
        return Point3D(*np.max(xyz, axis=0)), Point3D(*np.min(xyz, axis=0))

    def generate_side_vertices(self, line: Line3D, center_width: float, side_width: float) -> None:
        """GetSideVtx: even the middle polygon is buffered by side_width/2."""
        normal = Point3D(line.a, line.b, 0).normalized()
        offset = normal * (center_width / 2 + side_width / 2)
        left = Line3D(line.begin - offset, line.end - offset)
        middle = Line3D(line.begin, line.end)
        right = Line3D(line.begin + offset, line.end + offset)
        self.left_vertices = left.buffer_vertices(side_width / 2)
        self.main_vertices = middle.buffer_vertices(side_width / 2)
        self.right_vertices = right.buffer_vertices(side_width / 2)


@dataclass
class TrackSegment:
    track1: TrackWindow
    track2: TrackWindow
    gauge: float = 1.5
    distance_from_start: float | None = None

    @property
    def center(self) -> Point3D:
        return (self.track1.center + self.track2.center) / 2

    def update_gauge(self) -> float:
        self.gauge = distance_2d(self.track1.center, self.track2.center)
        return self.gauge

    def swap_windows(self, mode: int) -> None:
        """Source SwapWnd compares track1.x to itself; it is always a no-op."""
        return None


@dataclass
class TrackLayer:
    """Source layer geometry and shared point references."""

    number: int | None = None
    track_segments: list[TrackSegment] = field(default_factory=list)
    rail1_vector: list[Point3D] = field(default_factory=list)
    rail2_vector: list[Point3D] = field(default_factory=list)
    center_vector: list[Point3D] = field(default_factory=list)
    predicted_points: list[Point3D] = field(default_factory=list)
    updated_boundaries: list[Line3D] = field(default_factory=list)
    turnout_boundaries: list[Line3D] = field(default_factory=list)


@dataclass
class BinaryString:
    """LIDARPT.h binary state; labels are independent of Type and bit assignment."""

    type: int = -1
    score: float = 0.0
    bits: tuple[int, ...] = ()
    position_label: int = -1
    direction_label: int = -1
    turnout_label: int = -1
    toe_label: int = -1

    def __post_init__(self) -> None:
        self.bits = tuple(index(bit) for bit in self.bits)

    def set_bits(self, bits: Iterable[int]) -> None:
        """SetBinaryString overwrites its 16 integers, leaving metadata intact."""
        values = tuple(index(bit) for bit in bits)
        if len(values) != 16:
            raise ValueError("SetBinaryString requires exactly 16 integers")
        self.bits = values

    def set_labels(self, type_id: int) -> None:
        """SetLables, including the source's else labels for unknown/999 types.

        This method does not assign self.type, just as the source does not.
        """
        self.position_label = 0 if type_id in (0, 1, 2, 3, 4, 5, 12, 13, 16, 17) else 1
        self.direction_label = 0 if type_id in (1, 3, 5, 7, 9, 11, 12, 15, 16, 19) else 1
        self.turnout_label = 0 if type_id in (2, 3, 6, 7, 10, 11, 14, 15, 16, 17) else 1
        self.toe_label = (
            0
            if type_id in (0, 1, 6, 7)
            else 1
            if type_id in (4, 5, 10, 11)
            else 2
            if type_id in (12, 13, 14, 15)
            else -1
        )


@dataclass
class TrackStack:
    """Predicted turnout segments and the source's independently stored signature."""

    track_segments: list[TrackSegment] = field(default_factory=list)
    binary: BinaryString = field(default_factory=BinaryString)


def seed_windows(seed: InitialLocation, config: ExtractionConfig | None = None) -> TrackSegment:
    """Port only Setinitiallocation's initial paired-window construction.

    This does not fit GMMs/lines or perform the full C++ initialization. The second
    window initially has z=0, as in the source, even if the first seed has elevation.
    Modes other than 1/2/3 follow the source's else branch (including GUI mode 0).
    """
    config = config if config is not None else ExtractionConfig()
    angle = seed.angle * pi / 180
    dx = config.track_gauge * fabs(sin(angle))
    dy = config.track_gauge * fabs(cos(angle))
    if seed.mode == 1:
        x, y = seed.point.x - dx, seed.point.y + dy
    elif seed.mode == 2:
        x, y = seed.point.x - dx, seed.point.y - dy
    elif seed.mode == 3:
        x, y = seed.point.x + dx, seed.point.y - dy
    else:
        x, y = seed.point.x + dx, seed.point.y + dy
    segment = TrackSegment(
        TrackWindow(seed.point, seed.angle),
        TrackWindow(Point3D(x, y, 0), seed.angle),
        gauge=config.track_gauge,
        distance_from_start=0.0,
    )
    segment.swap_windows(seed.mode)
    for window in (segment.track1, segment.track2):
        window.generate_vertices(config.track_window_length, config.track_window_width, seed.mode)
    return segment
