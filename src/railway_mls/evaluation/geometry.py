"""Literal geometry from the 2018 evaluator, independently of the 2017 tracker."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class EvaluationLine:
    begin: np.ndarray
    end: np.ndarray
    a: float
    b: float
    c: float
    polygon: np.ndarray


def prepare_line(endpoints, buffer=0.3):
    begin, end = np.asarray(endpoints, dtype=np.float64)
    dx, dy = end[:2] - begin[:2]
    if dx == 0:
        a, b, c = np.float64(1), np.float64(0), -begin[0]
    elif dy == 0:
        a, b, c = np.float64(0), np.float64(-1), begin[1]
    else:
        a, b = dy / dx, np.float64(-1)
        c = begin[1] - a * begin[0]
    with np.errstate(all="ignore"):
        if a != 0 and b != 0:
            px, py = np.float64(0), -c / b
            c1 = buffer * np.sqrt(a*a + b*b) - a*px - b*py
            c2 = -1*buffer * np.sqrt(a*a + b*b) - a*px - b*py
            oa, ob = 1/a, -1*b
            oc1 = -1*(oa*begin[0] + ob*begin[1])
            oc2 = -1*(oa*end[0] + ob*end[1])
            polygon = []
            for pc, oc in ((c1, oc1), (c1, oc2), (c2, oc2), (c2, oc1)):
                x = -1*(pc + oc)/(a + oa)
                y = -1*(a*x + pc)/b
                polygon.append((x, y))
        elif a == 0:
            polygon = [(begin[0], c-buffer), (end[0], c-buffer),
                       (end[0], c+buffer), (begin[0], c+buffer)]
        else:
            polygon = [(-1*c-buffer, begin[1]), (-1*c-buffer, end[1]),
                       (-1*c+buffer, end[1]), (-1*c+buffer, begin[1])]
    return EvaluationLine(begin, end, a, b, c, np.asarray(polygon))


def polygon_membership(xyz, polygon):
    """pnpoly2: all edge cross-product categories must be identical (including 0)."""
    xyz = np.asarray(xyz)
    first = None
    result = np.ones(len(xyz), dtype=bool)
    for i in range(len(polygon)):
        p, q = polygon[i], polygon[(i+1) % len(polygon)]
        cross = (q[0]-p[0])*(xyz[:, 1]-p[1]) - (q[1]-p[1])*(xyz[:, 0]-p[0])
        # NaN follows the source's else/else branch, categorizing it as +1.
        category = np.where(np.abs(cross) < 1e-10, 0, np.where(cross < 0, -1, 1))
        if first is None:
            first = category
        else:
            result &= category == first
    return result


def residuals(xyz, line):
    """Unsigned XY distance and native signed, unsigned-chainage vertical residual."""
    p = np.asarray(xyz, dtype=np.float64)
    a, b, c = line.a, line.b, line.c
    with np.errstate(all="ignore"):
        horizontal = np.abs((a*p[:, 0] + b*p[:, 1] + c) / np.sqrt(a*a + b*b))
        if a != 0 and b != 0:
            pa = -1/a
            pc = p[:, 1] - pa*p[:, 0]
            fx = (pc-c)/(a-pa)
            fy = a*fx + c
        elif a == 0:
            fx, fy = p[:, 0], np.full(len(p), -c/b)
        else:
            fx, fy = np.full(len(p), -c/a), p[:, 1]
        dx, dy = line.begin[:2]-line.end[:2]
        length = np.sqrt(dx*dx + dy*dy)
        dx, dy = line.begin[0]-fx, line.begin[1]-fy
        chainage = np.sqrt(dx*dx + dy*dy)
        height = line.end[2] - line.begin[2]
        vertical = (p[:, 2]-line.begin[2] if height == 0
                    else p[:, 2] - (line.begin[2] + chainage*height/length))
    return horizontal, vertical


def assign_points(xyz, lines, *, use_index=True):
    """Stable first membership. X sorting is only a broad phase; pnpoly2 decides."""
    xyz = np.asarray(xyz, dtype=np.float64)
    assigned = np.full(len(xyz), -1, dtype=np.int64)
    order = np.argsort(xyz[:, 0], kind="stable") if use_index else None
    sorted_x = xyz[order, 0] if use_index else None
    for segment_id, line in enumerate(lines):
        polygon = line.polygon
        relative = polygon - polygon[0]
        area2 = np.sum(relative[:, 0]*np.roll(relative[:, 1], -1)
                       - relative[:, 1]*np.roll(relative[:, 0], -1))
        # Degenerate polygons can accept collinear points outside their AABB.
        indexed = use_index and np.all(np.isfinite(polygon)) and abs(area2) > 4e-10
        if indexed:
            lower = np.nextafter(polygon.min(axis=0), -np.inf)
            upper = np.nextafter(polygon.max(axis=0), np.inf)
            lo, hi = np.searchsorted(sorted_x, [lower[0], upper[0]], side="left")
            candidates = order[lo:hi]
            candidates = candidates[(xyz[candidates, 1] >= lower[1])
                                    & (xyz[candidates, 1] <= upper[1])]
        else:
            candidates = np.arange(len(xyz))
        candidates = candidates[assigned[candidates] == -1]
        if len(candidates):
            candidates = candidates[polygon_membership(xyz[candidates], polygon)]
            assigned[candidates] = segment_id
    return assigned
