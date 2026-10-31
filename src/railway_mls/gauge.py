"""Source paired-line adjustments, including their cached and IEEE behavior."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi

import numpy as np

from .config import ExtractionConfig
from .geometry import (
    Line3D,
    direction_mode,
    distance_2d,
    line_parameters_from_angle,
    perpendicular_foot,
)
from .points import Point3D, PointSelection


@dataclass(frozen=True)
class GaugeAdjustment:
    lines: tuple[Line3D, Line3D]
    common_angle: float
    initial_gap: float
    scale: float
    weighted_distance: float | None
    support_distances: tuple[float, ...]
    normalized_errors: tuple[float, ...]
    status: str

    @property
    def success(self) -> bool:
        return self.status == "ok"


def _divide(numerator: float, denominator: float) -> float:
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        return float(np.divide(numerator, denominator))


def _modes(first: Line3D, second: Line3D, modes: tuple[int, int] | None) -> tuple[int, int]:
    return (
        modes
        if modes is not None
        else (direction_mode(first.end, first.begin), direction_mode(second.end, second.begin))
    )


def _set_coefficients(line: Line3D, angle: float, point: Point3D) -> None:
    line.angle = angle
    if isfinite(angle):
        line.a, line.b, line.c = line_parameters_from_angle(angle, point.x, point.y)
    else:
        # C++ tan on a nonfinite angle propagates a domain NaN, not a Python exception.
        line.a, line.b, line.c = float("nan"), -1.0, float("nan")


def _common_projection(first: Line3D, second: Line3D, angle: float) -> tuple[Point3D, Point3D]:
    point1, point2 = first.update_center(), second.update_center()
    _set_coefficients(first, angle, point1)
    _set_coefficients(second, angle, point2)
    return point1, perpendicular_foot(point1, second)


def _extend(first: Point3D, second: Point3D, scale: float) -> tuple[Point3D, Point3D]:
    # ExtendEndingPts scales XYZ, although the scale is computed from XY data.
    center = (first + second) / 2
    return center - (center - first) * scale, center - (center - second) * scale


def _place(line: Line3D, center: Point3D, angle: float, length: float) -> None:
    _set_coefficients(line, angle, center)
    line.center = center
    with np.errstate(invalid="ignore", over="ignore"):
        radians = angle * pi / 180
        dx, dy = float(length / 2 * np.cos(radians)), float(length / 2 * np.sin(radians))
    line.begin = Point3D(center.x + dx, center.y + dy, center.z, line.begin.point_type)
    line.end = Point3D(center.x - dx, center.y - dy, center.z, line.end.point_type)
    # The source does NOT recompute coefficients or angle after setting endpoints.


def _finite(lines: tuple[Line3D, Line3D]) -> bool:
    for line in lines:
        values = (
            *line.begin.as_array(),
            *line.end.as_array(),
            *line.center.as_array(),
            line.a,
            line.b,
            line.c,
            line.angle,
        )
        if not all(isfinite(value) for value in values):
            return False
    return True


def adjust_two_lines(
    first: Line3D,
    second: Line3D,
    *,
    modes: tuple[int, int] | None = None,
    line_mode: int = 1,
    target_distance: float | None = None,
    config: ExtractionConfig | None = None,
) -> GaugeAdjustment:
    """AdjustTwoLines mutates the two lines; no point labels or memberships change."""
    settings = config if config is not None else ExtractionConfig()
    target = settings.track_gauge if target_distance is None else target_distance
    mode1, mode2 = _modes(first, second, modes)
    angle1, angle2 = first.angle, second.angle
    if mode1 == mode2 or ((mode1 in (1, 2)) == (mode2 in (1, 2))):
        angle = (angle1 + angle2) / 2
    elif mode1 in (1, 2):
        angle = (angle1 + (angle2 - 180)) / 2
    else:
        angle = ((angle1 - 180) + angle2) / 2
    if angle < 0:
        angle += 180
    point1, point2 = _common_projection(first, second, angle)
    gap = distance_2d(point1, point2)
    scale = _divide(target, gap)
    center1, center2 = _extend(point1, point2, scale)
    for line, center in ((first, center1), (second, center2)):
        _place(line, center, angle, settings.track_window_length)
        line.order_endpoints(line_mode)
    lines = first, second
    return GaugeAdjustment(
        lines, angle, gap, scale, None, (), (), "ok" if _finite(lines) else "nonfinite_adjustment"
    )


def adjust_two_lines_with_observation(
    first: Line3D,
    second: Line3D,
    support1: PointSelection,
    support2: PointSelection,
    *,
    references: tuple[Point3D, Point3D],
    modes: tuple[int, int] | None = None,
    line_mode: int = 1,
    target_distance: float | None = None,
    config: ExtractionConfig | None = None,
) -> GaugeAdjustment:
    """AdjustTwoLinesWithObservation, including exact-zero weights and raw NaNs.

    The source's line_mode parameter is unused here: reference points order the
    endpoints. Uniform support errors and empty support retain IEEE NaN behavior;
    no epsilon, ordinary average or target-gauge fallback is substituted.
    """
    settings = config if config is not None else ExtractionConfig()
    target = settings.track_gauge if target_distance is None else target_distance
    mode1, mode2 = _modes(first, second, modes)
    angle1, angle2 = first.angle, second.angle
    angle = (angle1 + angle2) / 2
    if (mode1, mode2) in ((1, 4), (4, 1), (2, 3), (3, 2)):
        difference = abs(angle1 - angle2)
        if abs(180 - difference) < difference:
            angle = (angle1 + angle2 + 180) / 2
    if angle < 0:
        angle += 180
    if angle >= 180:
        angle -= 180
    point1, point2 = _common_projection(first, second, angle)
    gap = distance_2d(point1, point2)
    distance_groups = []
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        for points, other in ((support1, second), (support2, first)):
            xyz = points.xyz
            distance_groups.append(
                np.abs(
                    (other.a * xyz[:, 0] + other.b * xyz[:, 1] + other.c)
                    / np.sqrt(other.a * other.a + other.b * other.b)
                )
            )
        distances = np.concatenate(distance_groups)
        errors = np.abs(target - distances)
        maximum, minimum = -1e100, 1e100
        for error in errors:
            if maximum < error:
                maximum = float(error)
            if minimum > error:
                minimum = float(error)
        normalized = (errors - minimum) / (maximum - minimum)
        weights = np.ones(len(distances), dtype=np.float64)
        contributions = distances.copy()
        nonzero = normalized != 0
        weights[nonzero] = 1 / normalized[nonzero]
        contributions[nonzero] = distances[nonzero] / normalized[nonzero]
        total = float(np.cumsum(contributions)[-1]) if len(contributions) else 0.0
        weight = float(np.cumsum(weights)[-1]) if len(weights) else 0.0
    weighted = _divide(total, weight)
    scale = _divide(target, weighted)
    center1, center2 = _extend(point1, point2, scale)
    for line, center, reference in (
        (first, center1, references[0]),
        (second, center2, references[1]),
    ):
        _place(line, center, angle, settings.track_window_length)
        line.order_endpoints_by_reference(reference)
    lines = first, second
    status = (
        "nonfinite_weighted_distance"
        if not isfinite(weighted)
        else ("ok" if _finite(lines) else "nonfinite_adjustment")
    )
    return GaugeAdjustment(
        lines,
        angle,
        gap,
        scale,
        weighted,
        tuple(map(float, distances)),
        tuple(map(float, normalized)),
        status,
    )
