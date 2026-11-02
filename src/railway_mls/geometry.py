"""Source geometry from LIDARPT.h and RailwayExtraction.cpp.

Angles are the source's folded degrees, not compass bearings. Preserved quirks
are intentional; see docs/PYTHON_FOUNDATION.md before changing these formulas.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from collections.abc import Sequence
from math import acos, fabs, isfinite, pi, sqrt, tan
from operator import index

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


def inner_angle(start: Point3D, middle: Point3D, end: Point3D) -> float:
    """ComputeInnerAngle: normalized XY arms, clamped dot product, IEEE NaNs."""
    x1, y1 = start.x - middle.x, start.y - middle.y
    x2, y2 = end.x - middle.x, end.y - middle.y
    length1, length2 = sqrt(x1 * x1 + y1 * y1), sqrt(x2 * x2 + y2 * y2)
    with np.errstate(divide="ignore", invalid="ignore"):
        x1, y1 = float(np.divide(x1, length1)), float(np.divide(y1, length1))
        x2, y2 = float(np.divide(x2, length2)), float(np.divide(y2, length2))
    product = x1 * x2 + y1 * y2
    if product < -1:
        product = -1.0
    if product > 1:
        product = 1.0
    return acos(product) * 180 / pi


def recount(total: int, current: int) -> int:
    """ReCount's int32 contract; negative indices add total only once.

    Undefined native integer division/overflow raise instead of wrapping Python
    integers or manufacturing a usable polygon index.
    """
    total, current = index(total), index(current)
    minimum, maximum = -(1 << 31), (1 << 31) - 1
    if not all(minimum <= value <= maximum for value in (total, current)):
        raise OverflowError("ReCount arguments must fit the source signed 32-bit int")
    if current >= total:
        if total == 0:
            raise ZeroDivisionError("undefined source ReCount integer division by zero")
        quotient = abs(current) // abs(total)
        if (current < 0) != (total < 0):
            quotient = -quotient
        value = current - total * quotient
    elif current < 0:
        value = total + current
    else:
        value = current
    if not minimum <= value <= maximum:
        raise OverflowError("undefined source ReCount signed integer overflow")
    return value


def min_max_xy(points: PointSelection) -> tuple[float, float, float, float]:
    """GetMinMaxXY: (minX, maxX, minY, maxY), including empty sentinels."""
    if not len(points):
        return 1e100, -1e100, 1e100, -1e100
    xy = points.xyz[:, :2]
    # argmin/argmax retain the FIRST equal value, including its zero sign.
    lowest, highest = np.argmin(xy, axis=0), np.argmax(xy, axis=0)
    return (
        min(1e100, float(xy[lowest[0], 0])),
        max(-1e100, float(xy[highest[0], 0])),
        min(1e100, float(xy[lowest[1], 1])),
        max(-1e100, float(xy[highest[1], 1])),
    )


def intersection_point_3d(
    a1: float, b1: float, c1: float, a2: float, b2: float, c2: float
) -> Point3D:
    """Inactive ComputeIntersectionPoint3D helper, including its literal Y rule.

    This is not a general corrected line solver. Parallel/coincident lines keep
    IEEE results; Z is always zero, and the source's A2 == 1 branch is retained.
    """
    a1, b1, c1, a2, b2, c2 = map(float, (a1, b1, c1, a2, b2, c2))
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        x = float(np.divide(b1 * c2 - b2 * c1, a1 * b2 - b1 * a2))
    y = a2 * x + c2 if a2 != 1 else a1 * x + c1
    return Point3D(x, y, 0)


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


@dataclass(frozen=True)
class BoundaryLine:
    """GetLnsFromVtx initializes endpoints only; caches remain explicitly absent.

    Native MemPts is an initialized empty vector. The default POINT3D endpoint
    tags stay zero because its assignment operator copies XYZ only.
    """

    begin: Point3D
    end: Point3D
    a: None = field(default=None, init=False)
    b: None = field(default=None, init=False)
    c: None = field(default=None, init=False)
    angle: None = field(default=None, init=False)
    center: None = field(default=None, init=False)
    unit_vector: None = field(default=None, init=False)
    d3d: None = field(default=None, init=False)
    members: tuple = field(default=(), init=False)


def lines_from_vertices(vertices: Sequence[Point3D]) -> tuple[BoundaryLine, ...]:
    """GetLnsFromVtx's replacement output, with cyclic edges in source order."""
    count = len(vertices)
    result = []
    for i, vertex in enumerate(vertices):
        following = vertices[recount(count, i + 1)]
        result.append(
            BoundaryLine(
                Point3D(vertex.x, vertex.y, vertex.z),
                Point3D(following.x, following.y, following.z),
            )
        )
    return tuple(result)


def distance_to_line_3d(
    point: Point3D, line: Line3D | BoundaryLine, *, unit_vector: Point3D
) -> float:
    """DistanceBetweenPointAndLine3D, using the explicitly supplied stored vector.

    A stale/nonunit vector is neither normalized nor replaced from endpoints.
    The midpoint is recomputed; a stale stored center is ignored, as in C++.
    """
    if unit_vector is None:
        raise ValueError("source line's stored unit vector is uninitialized")
    middle = (line.begin + line.end) / 2
    x, y, z = middle.x - point.x, middle.y - point.y, middle.z - point.z
    product = unit_vector.x * x + unit_vector.y * y + unit_vector.z * z
    dx, dy, dz = (
        x - product * unit_vector.x,
        y - product * unit_vector.y,
        z - product * unit_vector.z,
    )
    return sqrt(dx * dx + dy * dy + dz * dz)


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


def points_in_convex_polygon(vertices: tuple[Point3D, ...], xy: ArrayLike) -> NDArray[np.bool_]:
    """Source pnpoly2: ALL edge cross-product tags must agree, including zeros.

    The fixed 1e-10 test excludes ordinary rectangle edges, but all-zero tags
    qualify. This predicate is distinct from the window ray-crossing pnpoly.
    """
    values = np.asarray(xy, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 2:
        raise ValueError("xy must have shape (N, 2)")
    if not vertices:
        raise ValueError("native pnpoly2 has no defined result for an empty polygon")
    tags = []
    for i, first in enumerate(vertices):
        second = vertices[(i + 1) % len(vertices)]
        cross = (second.x - first.x) * (values[:, 1] - first.y) - (second.y - first.y) * (
            values[:, 0] - first.x
        )
        tags.append(np.where(np.abs(cross) < 1e-10, 0, np.where(cross < 0, -1, 1)))
    return np.all(np.asarray(tags) == tags[0], axis=0)


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
