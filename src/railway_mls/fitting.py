"""Source RANSAC and fixed-angle line-fitting variants."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from operator import index

import numpy as np

from .geometry import Line3D, distance_2d, line_parameters_from_angle, min_max_xy
from .points import Point3D, PointSelection
from .sampling import MsvcRand, RandSource


@dataclass(frozen=True)
class RailHeadLineFit:
    line: Line3D | None
    support: PointSelection
    iterations: int
    status: str

    @property
    def success(self) -> bool:
        return self.line is not None


@dataclass(frozen=True)
class LineFitResult:
    line: Line3D | None
    support: PointSelection
    iterations: int
    status: str
    sigma: float | None = None

    @property
    def success(self) -> bool:
        return self.status in ("ok", "ok_existing_outputs")


def _bounds(points: PointSelection) -> tuple[np.ndarray, np.ndarray]:
    min_x, max_x, min_y, max_y = min_max_xy(points)
    return np.array((min_x, min_y)), np.array((max_x, max_y))


def _bounded_line(
    coefficients: tuple[float, float, float],
    minimum: np.ndarray,
    maximum: np.ndarray,
    height: float,
) -> Line3D:
    """SetEndingPnts/SetEndingPnts3D, followed by source parameter recomputation."""
    a, b, c = coefficients
    if a == 0:
        begin, end = Point3D(minimum[0], -c / b, height), Point3D(maximum[0], -c / b, height)
    elif b == 0:
        begin, end = Point3D(-c / a, minimum[1], height), Point3D(-c / a, maximum[1], height)
    elif -a / b < 1:  # Signed comparison, intentionally not absolute slope.
        begin = Point3D(minimum[0], -a / b * minimum[0] - c / b, height)
        end = Point3D(maximum[0], -a / b * maximum[0] - c / b, height)
    else:
        begin = Point3D(-b / a * minimum[1] - c / a, minimum[1], height)
        end = Point3D(-b / a * maximum[1] - c / a, maximum[1], height)
    line = Line3D(begin, end)
    line.update_center()
    return line


def _distances(xyz: np.ndarray, coefficients: tuple[float, float, float]) -> np.ndarray:
    a, b, c = coefficients
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        return np.abs((a * xyz[:, 0] + b * xyz[:, 1] + c) / sqrt(a * a + b * b))


def descending_height(points: PointSelection) -> PointSelection:
    """HigherZValue ordering; equal heights retain input order in this port.

    C++ std::sort does not specify equal-key order. The chosen Python tie order is
    explicit, not a claim to duplicate every compiler's equal-height permutation.
    """
    return points.cloud.select(points.indices[np.argsort(-points.xyz[:, 2], kind="stable")])


def fit_rail_head_line(
    points: PointSelection, threshold: float = 0.04, *, rng: RandSource | None = None
) -> RailHeadLineFit:
    """LineExtractionByRansac(pnts, ThDist, line), at source line 3060.

    Exactly 100 attempts, two modulo-reduced draws per attempt, skip equal sample
    indices, strict support threshold, first winning model on support-count ties.
    Endpoint bounds use ALL input points. Flat elevation uses floor(0.15*N) in
    descending support heights. No least-squares refinement is introduced.
    """
    empty = points.cloud.select([])
    if len(points) < 2:
        return RailHeadLineFit(None, empty, 0, "too_few_points")
    random = rng if rng is not None else MsvcRand()
    xyz = points.xyz
    minimum, maximum = _bounds(points)
    best_line = None
    best_support = empty
    maximum_count = 0
    for _ in range(100):
        first, second = random.rand() % len(points), random.rand() % len(points)
        if first == second:
            continue
        line = Line3D(Point3D(*xyz[first]), Point3D(*xyz[second]))
        distance = np.abs(
            (line.a * xyz[:, 0] + line.b * xyz[:, 1] + line.c)
            / sqrt(line.a * line.a + line.b * line.b)
        )
        support = distance < threshold
        count = int(np.count_nonzero(support))
        if count > maximum_count:
            maximum_count = count
            best_line = line
            best_support = points.where(support)

    if best_line is None:
        # The source proceeds with an uninitialized line/empty vector here.
        return RailHeadLineFit(None, empty, 100, "undefined_source_no_model")
    support = descending_height(best_support)
    height = float(support.xyz[int(len(support) * 0.15), 2])
    line = _bounded_line((best_line.a, best_line.b, best_line.c), minimum, maximum, height)
    return RailHeadLineFit(line, support, 100, "ok")


def fit_line_ransac(
    points: PointSelection, threshold: float = 0.06, *, rng: RandSource | None = None
) -> LineFitResult:
    """The 500-attempt overload with 5% support elevation and RMS XY sigma."""
    empty = points.cloud.select([])
    if len(points) < 2:
        return LineFitResult(None, empty, 0, "too_few_points")
    random = rng if rng is not None else MsvcRand()
    xyz = points.xyz
    minimum, maximum = _bounds(points)
    best = None
    support = empty
    maximum_count = 0
    for _ in range(500):
        first, second = random.rand() % len(points), random.rand() % len(points)
        if first == second:
            continue
        line = Line3D(Point3D(*xyz[first]), Point3D(*xyz[second]))
        mask = _distances(xyz, (line.a, line.b, line.c)) < threshold
        count = int(np.count_nonzero(mask))
        if count > maximum_count:
            maximum_count = count
            best, support = line, points.where(mask)
    if best is None:
        return LineFitResult(None, support, 500, "undefined_source_no_model")
    if len(support) < 2:
        return LineFitResult(None, support, 500, "insufficient_support")
    support = descending_height(support)
    height = float(support.xyz[int(len(support) * 0.05), 2])
    line = _bounded_line((best.a, best.b, best.c), minimum, maximum, height)
    distances = _distances(support.xyz, (line.a, line.b, line.c))
    sigma = sqrt(float(np.cumsum(distances * distances)[-1]) / len(support))
    return LineFitResult(line, support, 500, "ok", sigma)


def _fit_fixed(
    points: PointSelection,
    angle: float,
    threshold: float,
    random: RandSource,
    previous: LineFitResult | None,
    loop_minimum: int | None,
) -> LineFitResult:
    if previous is not None and previous.support.cloud is not points.cloud:
        raise ValueError("previous support must reference the same owning cloud")
    support = previous.support if previous is not None else points.cloud.select([])
    if not len(points):
        # The source returns false before clearing existing support or drawing.
        return LineFitResult(None, support, 0, "too_few_points")
    coefficients = None
    maximum_count = 0
    xyz = points.xyz
    for _ in range(len(points)):
        row = random.rand() % len(points)
        candidate = line_parameters_from_angle(angle, float(xyz[row, 0]), float(xyz[row, 1]))
        distances = _distances(xyz, candidate)
        mask = distances < threshold
        count = int(np.count_nonzero(mask))
        if loop_minimum is not None and count * 1.4 < np.count_nonzero(distances < threshold * 2):
            continue
        if count <= maximum_count:
            continue
        maximum_count = count
        coefficients, support = candidate, points.where(mask)

    retained = coefficients is None
    if retained and previous is not None and previous.line is not None:
        coefficients = previous.line.a, previous.line.b, previous.line.c
    if loop_minimum is not None and len(support) < loop_minimum:
        return LineFitResult(None, support, len(points), "insufficient_support")
    if coefficients is None or not len(support):
        return LineFitResult(None, support, len(points), "undefined_source_no_model")
    minimum, maximum = _bounds(support)  # Fixed-angle fits use support bounds.
    support = descending_height(support)
    height = float(support.xyz[int(len(support) * 0.05), 2])
    line = _bounded_line(coefficients, minimum, maximum, height)
    if retained and previous is not None and previous.line is not None:
        line.members = previous.line.members
    if loop_minimum is not None and distance_2d(line.begin, line.end) < 0.5:
        return LineFitResult(line, support, len(points), "line_too_short")
    return LineFitResult(line, support, len(points), "ok_existing_outputs" if retained else "ok")


def fit_fixed_angle(
    points: PointSelection,
    angle: float,
    threshold: float = 0.06,
    *,
    rng: RandSource | None = None,
    previous: LineFitResult | None = None,
) -> LineFitResult:
    """LineExtractionWithFixedAngle: N draws, 5% elevation, first best on ties.

    previous supplies the initialized output references used during source refits.
    If no new model wins, those outputs remain available, exactly as in C++.
    """
    return _fit_fixed(
        points, angle, threshold, rng if rng is not None else MsvcRand(), previous, None
    )


def fit_loop_cross_line(
    points: PointSelection,
    angle: float,
    threshold: float = 0.06,
    *,
    skip_rate: int = 1,
    rng: RandSource | None = None,
    previous: LineFitResult | None = None,
) -> LineFitResult:
    """Dormant loop/cross fitter, exposed alone without activating its workflow."""
    rate = index(skip_rate)
    if rate <= 0:
        raise ValueError("skip_rate must be positive")
    minimum = max(20 // rate, 15)
    return _fit_fixed(
        points, angle, threshold, rng if rng is not None else MsvcRand(), previous, minimum
    )
