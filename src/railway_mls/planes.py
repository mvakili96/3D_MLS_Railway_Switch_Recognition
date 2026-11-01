"""Source three-point planes, bundled symmetric JAMA refit and 100-trial RANSAC."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot

import numpy as np

from .points import Point3D, PointSelection
from .sampling import MsvcRand, RandSource


@dataclass(frozen=True)
class PlaneParameters:
    a: float
    b: float
    c: float
    d: float

    def as_array(self) -> np.ndarray:
        return np.array((self.a, self.b, self.c, self.d), dtype=np.float64)


def plane_from_three_points(first: Point3D, second: Point3D, third: Point3D) -> PlaneParameters:
    """GetPlaneParameters3p: negative cross product, without normalization."""
    ba, cb = second - first, third - second
    x = ba.y * cb.z - cb.y * ba.z
    y = -1 * (ba.x * cb.z - cb.x * ba.z)
    z = ba.x * cb.y - cb.x * ba.y
    return PlaneParameters(-x, -y, -z, second.x * x + second.y * y + second.z * z)


def plane_distances(points: PointSelection, plane: PlaneParameters) -> np.ndarray:
    """DistanceBetweenPointAndPlane, preserving zero-normal IEEE NaNs."""
    xyz, (a, b, c, d) = points.xyz, plane.as_array()
    with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
        return np.abs(
            (xyz[:, 0] * a + xyz[:, 1] * b + xyz[:, 2] * c + d) / np.sqrt(a * a + b * b + c * c)
        )


def _symmetric_eigen(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
    """The reached 3x3 symmetric tred2/tql2 paths in bundled Jama/jama_eig.h.

    Keep its sign choices, strict eigenvalue sorting, zero-scatter axis and
    repeated-eigenvalue basis. No generic eigensolver basis is substituted.
    None identifies nonfinite arithmetic before native invalid indexing/stalling.
    """
    v = matrix.copy()
    d, e = v[2].copy(), np.zeros(3)
    with np.errstate(over="ignore", divide="ignore", invalid="ignore", under="ignore"):
        for i in (2, 1):
            scale, h = 0.0, 0.0
            for k in range(i):
                scale += abs(d[k])
            if scale == 0:
                e[i] = d[i - 1]
                for j in range(i):
                    d[j] = v[i - 1, j]
                    v[i, j] = v[j, i] = 0.0
            else:
                for k in range(i):
                    d[k] /= scale
                    h += d[k] * d[k]
                f, g = d[i - 1], np.sqrt(h)
                if f > 0:
                    g = -g
                e[i] = scale * g
                h -= f * g
                d[i - 1] = f - g
                e[:i] = 0.0
                for j in range(i):
                    f = d[j]
                    v[j, i] = f
                    g = e[j] + v[j, j] * f
                    for k in range(j + 1, i):
                        g += v[k, j] * d[k]
                        e[k] += v[k, j] * f
                    e[j] = g
                f = 0.0
                for j in range(i):
                    e[j] /= h
                    f += e[j] * d[j]
                hh = f / (h + h)
                for j in range(i):
                    e[j] -= hh * d[j]
                for j in range(i):
                    f, g = d[j], e[j]
                    for k in range(j, i):
                        v[k, j] -= f * e[k] + g * d[k]
                    d[j] = v[i - 1, j]
                    v[i, j] = 0.0
            d[i] = h
        for i in range(2):
            v[2, i] = v[i, i]
            v[i, i] = 1.0
            h = d[i + 1]
            if h != 0:
                for k in range(i + 1):
                    d[k] = v[k, i + 1] / h
                for j in range(i + 1):
                    g = 0.0
                    for k in range(i + 1):
                        g += v[k, i + 1] * v[k, j]
                    for k in range(i + 1):
                        v[k, j] -= g * d[k]
            for k in range(i + 1):
                v[k, i + 1] = 0.0
        for j in range(3):
            d[j], v[2, j] = v[2, j], 0.0
        v[2, 2] = 1.0
        e[0] = 0.0
        e[:2], e[2] = e[1:].copy(), 0.0
        f, scale, eps = 0.0, 0.0, 2.0**-52
        for low in range(3):
            if not np.isfinite(v).all() or not np.isfinite([*d, *e]).all():
                return None
            scale = max(scale, abs(d[low]) + abs(e[low]))
            high = low
            while high < 3 and not abs(e[high]) <= eps * scale:
                high += 1
            if high == 3:
                return None
            if high > low:
                while True:
                    g = d[low]
                    p = (d[low + 1] - g) / (2.0 * e[low])
                    r = hypot(p, 1.0)
                    if p < 0:
                        r = -r
                    d[low], d[low + 1] = e[low] / (p + r), e[low] * (p + r)
                    dl1 = d[low + 1]
                    h = g - d[low]
                    for i in range(low + 2, 3):
                        d[i] -= h
                    f += h
                    p, c, c2, c3 = d[high], 1.0, 1.0, 1.0
                    el1, s, s2 = e[low + 1], 0.0, 0.0
                    for i in range(high - 1, low - 1, -1):
                        c3, c2, s2 = c2, c, s
                        g, h = c * e[i], c * p
                        r = hypot(p, e[i])
                        e[i + 1] = s * r
                        s, c = e[i] / r, p / r
                        p = c * d[i] - s * g
                        d[i + 1] = h + s * (c * g + s * d[i])
                        for k in range(3):
                            h = v[k, i + 1]
                            v[k, i + 1] = s * v[k, i] + c * h
                            v[k, i] = c * v[k, i] - s * h
                    p = -s * s2 * c3 * el1 * e[low] / dl1
                    e[low], d[low] = s * p, c * p
                    if not np.isfinite(v).all() or not np.isfinite([*d, *e]).all():
                        return None
                    if not abs(e[low]) > eps * scale:
                        break
            d[low] += f
            e[low] = 0.0
        for i in range(2):
            k, p = i, d[i]
            for j in range(i + 1, 3):
                if d[j] < p:
                    k, p = j, d[j]
            if k != i:
                d[k], d[i] = d[i], p
                for j in range(3):
                    v[j, i], v[j, k] = v[j, k], v[j, i]
    return d, v


@dataclass(frozen=True)
class PlaneEigenFit:
    plane: PlaneParameters | None
    center: Point3D
    scatter: np.ndarray
    eigenvalues: np.ndarray | None
    status: str


def plane_from_eigen(points: PointSelection) -> PlaneEigenFit:
    """GetPlaneParametersEigen, including its defined empty/collinear results."""
    xyz = points.xyz
    cx = cy = cz = 0.0
    with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
        for i, (x, y, z) in enumerate(xyz):
            cx = (i * cx + x) / (i + 1)
            cy = (i * cy + y) / (i + 1)
            cz = (i * cz + z) / (i + 1)
        center = Point3D(cx, cy, cz)
        scatter = np.zeros((3, 3))
        if len(points):
            delta = xyz - center.as_array()
            # cumsum retains source row accumulation order, unlike a pairwise sum.
            for i in range(3):
                for j in range(i, 3):
                    value = np.cumsum(delta[:, i] * delta[:, j])[-1]
                    scatter[i, j] = scatter[j, i] = value
    if not np.isfinite(scatter).all() or not np.isfinite(center.as_array()).all():
        return PlaneEigenFit(None, center, scatter, None, "nonfinite_plane_scatter")
    eigen = _symmetric_eigen(scatter)
    if eigen is None:
        return PlaneEigenFit(None, center, scatter, None, "nonfinite_plane_eigen")
    values, vectors = eigen
    a, b, c = vectors[:, 0]
    plane = PlaneParameters(a, b, c, -1 * (a * cx + b * cy + c * cz))
    return PlaneEigenFit(plane, center, scatter, values, "ok")


@dataclass(frozen=True)
class PlaneFitResult:
    plane: PlaneParameters | None
    support: PointSelection
    attempts: int
    sampled_counts: tuple[int, ...]
    refined_counts: tuple[int, ...]
    best_iteration: int | None
    parameters_written: bool
    support_cleared: bool
    status: str

    @property
    def reference_comparable(self) -> bool:
        return self.plane is not None and not self.status.startswith("nonfinite")


def fit_plane_ransac(
    points: PointSelection,
    threshold: float = 0.008,
    *,
    rng: RandSource | None = None,
    plane: PlaneParameters | None = None,
    support: PointSelection | None = None,
) -> PlaneFitResult:
    """ExtractPlaneByRANSAC: three draws/trial, refit every model, first winner.

    Under four inputs return BEFORE clearing support or touching parameters.
    With enough inputs support is cleared, but the old plane survives no winner.
    Repeated/collinear samples are not skipped: even empty support is refitted.
    """
    previous = support if support is not None else points.cloud.select([])
    if previous.cloud is not points.cloud:
        raise ValueError("plane support and input must share one owning cloud")
    sampled, refined, best, written = [], [], None, False

    def finish(status: str) -> PlaneFitResult:
        return PlaneFitResult(
            plane,
            previous,
            len(sampled),
            tuple(sampled),
            tuple(refined),
            best,
            written,
            len(points) >= 4,
            status,
        )

    if len(points) < 4:
        return finish("too_few_points")
    previous = points.cloud.select([])
    random = rng if rng is not None else MsvcRand()
    xyz, maximum = points.xyz, 0
    for i in range(100):
        indices = [random.rand() % len(points) for _ in range(3)]
        with np.errstate(over="ignore", invalid="ignore"):
            sampled_plane = plane_from_three_points(*(Point3D(*xyz[j]) for j in indices))
        selected = points.where(plane_distances(points, sampled_plane) < threshold)
        sampled.append(len(selected))
        fitted = plane_from_eigen(selected)
        if fitted.plane is None:
            return finish(fitted.status)
        selected = points.where(plane_distances(points, fitted.plane) < threshold)
        refined.append(len(selected))
        if len(selected) > maximum:
            maximum, previous, plane = len(selected), selected, fitted.plane
            best, written = i, True
    return finish("ok" if written else "no_winning_model")
