"""The 100-iteration line-fit overload required by rail-head extraction."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

import numpy as np

from .geometry import Line3D
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
    minimum = np.minimum(xyz[:, :2].min(axis=0), 1e100)
    maximum = np.maximum(xyz[:, :2].max(axis=0), -1e100)
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
    a, b, c = best_line.a, best_line.b, best_line.c
    if a == 0:
        begin, end = Point3D(minimum[0], -c / b, height), Point3D(maximum[0], -c / b, height)
    elif b == 0:
        begin, end = Point3D(-c / a, minimum[1], height), Point3D(-c / a, maximum[1], height)
    elif -a / b < 1:  # Signed comparison in SetEndingPnts3D, not abs(slope).
        begin = Point3D(minimum[0], -a / b * minimum[0] - c / b, height)
        end = Point3D(maximum[0], -a / b * maximum[0] - c / b, height)
    else:
        begin = Point3D(-b / a * minimum[1] - c / a, minimum[1], height)
        end = Point3D(-b / a * maximum[1] - c / a, maximum[1], height)
    return RailHeadLineFit(Line3D(begin, end), support, 100, "ok")
