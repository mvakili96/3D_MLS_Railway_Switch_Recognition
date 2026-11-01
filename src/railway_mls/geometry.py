"""Source geometry from LIDARPT.h and RailwayExtraction.cpp.

Angles are the source's folded degrees, not compass bearings. Preserved quirks
are intentional; see docs/PYTHON_FOUNDATION.md before changing these formulas.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import acos, fabs, isfinite, pi, sqrt, tan

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .points import Point3D, PointSelection


def direction_mode(next_point: Point3D, previous_point: Point3D) -> int:
    dx, dy = next_point.x - previous_point.x, next_point.y - previous_point.y
    if dx >= 0 and dy >= 0:
        return 1
    if dx < 0 and dy >= 0:
        return 2
    if dx < 0 and dy < 0:
        return 3
    return 4


def source_angle(begin: Point3D, end: Point3D) -> float:
    """Line3D::GetAngle / RailwayExtraction::GetAngle, including coincident=0."""
    dx, dy = end.x - begin.x, end.y - begin.y
    length = sqrt(dx * dx + dy * dy)
    if length == 0:
        return 0.0  # C++ obtains NaN, then explicitly replaces it with zero.
    ratio = fabs(dx / length)
    angle = acos(ratio) if ratio <= 1 else float("nan")
    if (dy >= 0 and dx < 0) or (dy < 0 and dx >= 0):
        angle = pi - angle
    if not isfinite(angle):
        angle = 0.0
    return angle * 180 / pi


def line_parameters(begin: Point3D, end: Point3D) -> tuple[float, float, float]:
    dx, dy = end.x - begin.x, end.y - begin.y
    if dx == 0:
        return 1.0, 0.0, -begin.x
    if dy == 0:
        return 0.0, -1.0, begin.y
    a = dy / dx
    return a, -1.0, begin.y - a * begin.x


def line_parameters_from_angle(angle: float, x: float, y: float) -> tuple[float, float, float]:
    """Preserve Get2DLineParameterWithAngleAndPt's unusual 0/90 degree cases."""
    if angle == 0:
        return 0.0, -1.0, float(x)
    if angle == 90:
        return -1.0, 0.0, float(y)
    a = tan(angle * pi / 180)
    return a, -1.0, y - a * x


@dataclass
class Line3D:
    begin: Point3D
    end: Point3D
    a: float = field(init=False)
    b: float = field(init=False)
    c: float = field(init=False)
    angle: float = field(init=False)
    members: PointSelection | None = field(default=None, repr=False, compare=False)
    _center: Point3D | None = field(default=None, init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        self.update_parameters()

    def update_parameters(self) -> None:
        """Equivalent to explicitly calling GetLineParameter then GetAngle."""
        self.a, self.b, self.c = line_parameters(self.begin, self.end)
        self.angle = source_angle(self.begin, self.end)

    @property
    def center(self) -> Point3D:
        return self._center if self._center is not None else (self.begin + self.end) / 2

    @center.setter
    def center(self, point: Point3D) -> None:
        self._center = point

    def update_center(self) -> Point3D:
        """GetCenterPt: explicitly refresh the source's separately stored center."""
        self._center = (self.begin + self.end) / 2
        return self._center

    def set_ending_points(self, min_x: float, max_x: float, min_y: float, max_y: float) -> None:
        """SetEndingPnts: retain endpoint Z, coefficients, angle and cached center."""
        a, b, c = self.a, self.b, self.c
        if a == 0 and b == 0:
            return
        if a == 0:
            bx, by, ex, ey = min_x, -c / b, max_x, -c / b
        elif b == 0:
            bx, by, ex, ey = -c / a, min_y, -c / a, max_y
        elif -a / b < 1:  # Signed slope, as in the source.
            bx, by, ex, ey = min_x, -a / b * min_x - c / b, max_x, -a / b * max_x - c / b
        else:
            bx, by, ex, ey = -b / a * min_y - c / a, min_y, -b / a * max_y - c / a, max_y
        self.begin = Point3D(bx, by, self.begin.z, self.begin.point_type)
        self.end = Point3D(ex, ey, self.end.z, self.end.point_type)

    @property
    def unit_vector(self) -> Point3D:
        return (self.end - self.begin).normalized()

    def order_endpoints(self, mode: int) -> None:
        """SetEndingByMode, including else-as-mode-4 and Y priority on ties."""
        if abs(self.begin.x - self.end.x) > abs(self.begin.y - self.end.y):
            swap = self.begin.x < self.end.x if mode in (2, 3) else self.begin.x > self.end.x
        else:
            swap = self.begin.y > self.end.y if mode in (1, 2) else self.begin.y < self.end.y
        if swap:
            self.begin, self.end = self.end, self.begin

    def order_endpoints_by_reference(self, reference: Point3D) -> None:
        """SetEndByReferencePt: closer XY endpoint is begin; ties do not swap."""
        if distance_2d(self.begin, reference) > distance_2d(self.end, reference):
            self.begin, self.end = self.end, self.begin

    def topology(self, x: float, y: float) -> int:
        """GetTopologyUsingCrossProduct: right side is +1 for an eastward line."""
        value = (x - self.end.x) * (self.end.y - self.begin.y) - (y - self.end.y) * (
            self.end.x - self.begin.x
        )
        return int(value > 0) - int(value < 0)

    def buffer_vertices(self, buffer_size: float) -> tuple[Point3D, ...]:
        """GetBufferVtxOfLine, preserving vertex order and zero vertex heights."""
        a, b, c = self.a, self.b, self.c
        if a != 0 and b != 0:
            y_intercept = -c / b
            c1 = buffer_size * sqrt(a * a + b * b) - b * y_intercept
            c2 = -buffer_size * sqrt(a * a + b * b) - b * y_intercept
            normal_a, normal_b = 1 / a, -b
            normal_c1 = -(normal_a * self.begin.x + normal_b * self.begin.y)
            normal_c2 = -(normal_a * self.end.x + normal_b * self.end.y)
            vertices = []
            for side_c, end_c in (
                (c1, normal_c1),
                (c1, normal_c2),
                (c2, normal_c2),
                (c2, normal_c1),
            ):
                x = -(side_c + end_c) / (a + normal_a)
                y = -(a * x + side_c) / b
                vertices.append(Point3D(x, y, 0))
            return tuple(vertices)
        if a == 0:
            return (
                Point3D(self.begin.x, c - buffer_size, 0),
                Point3D(self.end.x, c - buffer_size, 0),
                Point3D(self.end.x, c + buffer_size, 0),
                Point3D(self.begin.x, c + buffer_size, 0),
            )
        return (
            Point3D(-c - buffer_size, self.begin.y, 0),
            Point3D(-c - buffer_size, self.end.y, 0),
            Point3D(-c + buffer_size, self.end.y, 0),
            Point3D(-c + buffer_size, self.begin.y, 0),
        )


def points_in_polygon(vertices: tuple[Point3D, ...], xy: ArrayLike) -> NDArray[np.bool_]:
    """Vectorized source pnpoly: XY only, strict crossing and asymmetric edges.

    No tolerance, boundary inflation, bounding-box clipping or point reordering
    is introduced. A point on a polygon edge is handled by the original predicate.
    """
    values = np.asarray(xy, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 2:
        raise ValueError("xy must have shape (N, 2)")
    inside = np.zeros(len(values), dtype=np.bool_)
    if not vertices:
        return inside
    x, y = values[:, 0], values[:, 1]
    previous = vertices[-1]
    for current in vertices:
        eligible = (current.y > y) != (previous.y > y)
        # C++ && short-circuits horizontal edges. Do not divide on those rows.
        rows = np.flatnonzero(eligible)
        crossing_x = (previous.x - current.x) * (y[rows] - current.y) / (
            previous.y - current.y
        ) + current.x
        inside[rows] ^= x[rows] < crossing_x
        previous = current
    return inside


def polygon_area(vertices: tuple[Point3D, ...]) -> float:
    area = 0.0
    for number, point in enumerate(vertices):
        following = vertices[(number + 1) % len(vertices)]
        area += (following.x - point.x) * (point.y + following.y)
    return 0.5 * fabs(area)


def distance_2d(first: Point3D, second: Point3D) -> float:
    dx, dy = first.x - second.x, first.y - second.y
    return sqrt(dx * dx + dy * dy)


def distance_3d(first: Point3D, second: Point3D) -> float:
    dx, dy, dz = first.x - second.x, first.y - second.y, first.z - second.z
    return sqrt(dx * dx + dy * dy + dz * dz)


def distance_to_line_2d(point: Point3D, line: Line3D) -> float:
    return fabs(
        (line.a * point.x + line.b * point.y + line.c) / sqrt(line.a * line.a + line.b * line.b)
    )


def perpendicular_foot(point: Point3D, line: Line3D) -> Point3D:
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        if line.a != 0 and line.b != 0:
            perpendicular_a = float(np.divide(-1.0, line.a))
            perpendicular_c = point.y - perpendicular_a * point.x
            x = float(np.divide(perpendicular_c - line.c, line.a - perpendicular_a))
            y = line.a * x + line.c
        elif line.a == 0:
            x, y = point.x, float(np.divide(-line.c, line.b))
        else:
            x, y = float(np.divide(-line.c, line.a)), point.y
    return Point3D(x, y, (line.begin.z + line.end.z) / 2)
