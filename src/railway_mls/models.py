"""Source window, segment and layer state, including standalone header routines."""

from __future__ import annotations

from dataclasses import dataclass, field
from copy import copy
from collections.abc import Iterable
from math import cos, fabs, pi, sin, sqrt
from operator import index

import numpy as np

from .config import ExtractionConfig
from .geometry import BoundaryLine, Line3D, distance_2d, points_in_polygon
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

    def assign_from(self, other: GMMParameters) -> None:
        self.mu, self.sigma, self.alpha = other.mu, other.sigma, other.alpha


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
        self.members = self.members.cloud.select([]) if self.members is not None else None
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

    def member_bounds(
        self, maximum: Point3D | None = None, minimum: Point3D | None = None
    ) -> tuple[Point3D, Point3D]:
        """Return (maximum, minimum), including the source's empty sentinels."""
        high, low = Point3D(-1e100, -1e100, -1e100), Point3D(1e100, 1e100, 1e100)
        if self.members is not None and len(self.members):
            xyz = self.members.xyz[::-1]
            axes = np.arange(3)
            # min(new_value, old_value) keeps the LAST equal zero sign.
            high = Point3D(*np.maximum(xyz[np.argmax(xyz, axis=0), axes], -1e100))
            low = Point3D(*np.minimum(xyz[np.argmin(xyz, axis=0), axes], 1e100))
        return (
            (maximum.assigned_xyz(high) if maximum is not None else high),
            (minimum.assigned_xyz(low) if minimum is not None else low),
        )

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


@dataclass(frozen=True)
class NeighborRegions:
    middle: tuple[Point3D, ...] = ()
    middle_left: tuple[Point3D, ...] = ()
    middle_right: tuple[Point3D, ...] = ()
    left: tuple[Point3D, ...] = ()
    right: tuple[Point3D, ...] = ()

    @property
    def status(self) -> str:
        return (
            "ok"
            if all(
                np.isfinite(point.as_array()).all()
                for polygon in (
                    self.middle,
                    self.middle_left,
                    self.middle_right,
                    self.left,
                    self.right,
                )
                for point in polygon
            )
            else "nonfinite_neighbor_vertices"
        )


@dataclass(frozen=True)
class StateAssignment:
    status: str = "ok"
    reference_comparable: bool = True


def _assigned_vertices(destination, source):
    # A destination long enough guarantees sufficient native capacity. If it is
    # shorter, allocation history can select value copying or XYZ-only assignment.
    if len(source) <= len(destination):
        return tuple(old.assigned_xyz(new) for old, new in zip(destination, source)), True
    comparable = all(old.point_type == new.point_type for old, new in zip(destination, source))
    return tuple(source), comparable


def _assigned_line(destination, source):
    if source is None:
        return None
    result = copy(source)
    for attr in ("begin", "end", "_center", "_unit_vector"):
        new = getattr(source, attr)
        old = getattr(destination, attr) if destination is not None else None
        if new is not None:
            setattr(
                result, attr, Point3D(new.x, new.y, new.z, old.point_type if old is not None else 0)
            )
    return result


@dataclass
class TrackSegment:
    track1: TrackWindow
    track2: TrackWindow
    gauge: float = 1.5
    distance_from_start: float | None = None
    neighbor_regions: NeighborRegions = field(default_factory=NeighborRegions)
    stored_center: Point3D | None = None

    @property
    def center(self) -> Point3D:
        return (self.track1.center + self.track2.center) / 2

    def update_gauge(self) -> float:
        self.gauge = distance_2d(self.track1.center, self.track2.center)
        return self.gauge

    def set_gauge(self, distance: float) -> None:
        self.gauge = distance

    def update_center(self) -> Point3D:
        value = self.center
        self.stored_center = Point3D(
            value.x,
            value.y,
            value.z,
            self.stored_center.point_type if self.stored_center is not None else 0,
        )
        return self.stored_center

    def generate_neighbor_regions(
        self, length: float, width: float, window_width: float
    ) -> NeighborRegions:
        """GetNeighborVtxBND; do not activate its inactive loop/cross caller."""
        first, second = self.track1.center, self.track2.center
        center = (first + second) / 2

        def endpoint(point, offset):
            dx, dy = point.x - center.x, point.y - center.y
            radius = sqrt(dx * dx + dy * dy)
            with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
                return Point3D(
                    center.x + float(np.divide((radius + offset) * dx, radius)),
                    center.y + float(np.divide((radius + offset) * dy, radius)),
                    0,
                )

        def region(begin, end):
            return Line3D(begin, end).buffer_vertices(length / 2)

        self.neighbor_regions = NeighborRegions(
            region(endpoint(first, -width), endpoint(second, -width)),
            region(endpoint(second, -width), endpoint(second, -window_width / 2)),
            region(endpoint(first, -width), endpoint(first, -window_width / 2)),
            region(endpoint(second, window_width / 2), endpoint(second, 4 * window_width / 2)),
            region(endpoint(first, window_width / 2), endpoint(first, 4 * window_width / 2)),
        )
        return self.neighbor_regions

    def duplicate_from(self, source: TrackSegment) -> StateAssignment:
        """DuplicatePolyPair: preserve destination distance and neighbor regions."""
        if self is source:
            return StateAssignment()
        comparable = True
        for old, new in ((self.track1, source.track1), (self.track2, source.track2)):
            # Window assignment is field-wise; copy selectors but keep cloud rows shared.
            for attr in ("vertices", "left_vertices", "right_vertices", "main_vertices"):
                values, known = _assigned_vertices(getattr(old, attr), getattr(new, attr))
                setattr(old, attr, values)
                comparable = comparable and known
            old.center = old.center.assigned_xyz(new.center)
            for attr in (
                "orientation",
                "members",
                "left_members",
                "right_members",
                "turnout",
                "left_tag",
                "right_tag",
            ):
                setattr(old, attr, getattr(new, attr))
            old.member_line = _assigned_line(old.member_line, new.member_line)
        self.gauge = source.gauge
        self.stored_center = (
            self.stored_center.assigned_xyz(source.stored_center)
            if self.stored_center is not None and source.stored_center is not None
            else Point3D(source.stored_center.x, source.stored_center.y, source.stored_center.z)
            if source.stored_center is not None
            else None
        )
        return StateAssignment("ok" if comparable else "capacity_dependent_tags", comparable)

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
    updated_boundaries: list[BoundaryLine] = field(default_factory=list)
    turnout_boundaries: list[BoundaryLine] = field(default_factory=list)
    toe_points: list[ToePoint] = field(default_factory=list)
    switch_boxes: list[SwitchBox] = field(default_factory=list)
    loop_crosses: list[LoopCross] = field(default_factory=list)
    visual_loop_crosses: list[LoopCross] = field(default_factory=list)
    loop_boundaries: list[BoundaryLine] = field(default_factory=list)
    stacks: list[TrackStack] = field(default_factory=list)


@dataclass
class LoopCross:
    line: Line3D | BoundaryLine | None = None
    distance_from_start: float | None = None


@dataclass
class SwitchBox:
    """SwitchBox fields; final native referHgt is uninitialized and remains None."""

    center: Point3D
    angle: float
    vertices: tuple[Point3D, ...] = ()
    reference_height: float | None = None

    def generate_region(self, size: float) -> None:
        radius = size / 2 / cos(45 * pi / 180)
        self.vertices = tuple(
            Point3D(
                self.center.x + radius * cos((self.angle + i * 90 + 45) * pi / 180),
                self.center.y + radius * sin((self.angle + i * 90 + 45) * pi / 180),
                0,
            )
            for i in range(4)
        )


@dataclass
class GradeSection:
    """GradeSect defaults/reset; membership contains value points, not cloud IDs."""

    begin: Point3D = field(default_factory=lambda: Point3D(0, 0, 0))
    end: Point3D = field(default_factory=lambda: Point3D(0, 0, 0))
    member_vertices: list[Point3D] = field(default_factory=list)
    arc: float = 0.0
    chord: float = 0.0
    height: float = 0.0
    grade: float = 0.0
    radius: float = 0.0
    degree_of_curve: float = 0.0
    grade_with_curve: float = 0.0

    def reset(self) -> None:
        # Native SetPoint3D resets both XYZ and PtType.
        self.begin = Point3D(0, 0, 0)
        self.end = Point3D(0, 0, 0)
        self.member_vertices.clear()
        self.arc = self.chord = self.height = self.grade = 0.0
        self.radius = self.degree_of_curve = self.grade_with_curve = 0.0


@dataclass
class ToePoint:
    """ToePt defaults; native uninitialized geometry/angle/mode are explicit None."""

    center: Point3D | None = None
    left: Point3D | None = None
    right: Point3D | None = None
    track_type: int = -1
    used: bool = False
    angle: float | None = None
    mode: int | None = None


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
    segment.update_center()
    return segment
