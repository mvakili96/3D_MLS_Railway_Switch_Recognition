"""The custom height-only EM and clustering in RailwayExtraction.cpp:3323–3491.

This intentionally is not a regularized/library Gaussian mixture. Component order,
early exits, last-component selection and last-match classification ties all matter.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import pi, sqrt
from operator import index
from typing import Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .models import GMMParameters, TrackSegment
from .points import PointSelection


@dataclass(frozen=True)
class HeightGMMFit:
    parameters: tuple[GMMParameters, ...]
    iterations: int
    stop_reasons: tuple[str, ...]

    @property
    def finite(self) -> bool:
        return all(np.isfinite((p.mu, p.sigma, p.alpha)).all() for p in self.parameters)


@dataclass(frozen=True)
class GMMClustering:
    clusters: tuple[PointSelection, ...]
    failed_position: int | None = None

    @property
    def complete(self) -> bool:
        return self.failed_position is None


@dataclass(frozen=True)
class WindowClassification:
    points: PointSelection
    fit: HeightGMMFit
    clustering: GMMClustering

    @property
    def heads(self) -> PointSelection:
        """ComputeGMMParaAndCluster/initialization select the last component."""
        return self.clustering.clusters[-1]

    @property
    def head_candidates(self) -> PointSelection:
        """ExtractTrkPts concatenates last component THEN penultimate component."""
        clusters = self.clustering.clusters
        return self.points.cloud.select(
            np.concatenate((clusters[-1].indices, clusters[-2].indices))
        )


def _density(
    heights: NDArray[np.float64],
    mean: NDArray[np.float64],
    sigma: NDArray[np.float64],
    alpha: NDArray[np.float64],
) -> NDArray[np.float64]:
    difference = heights[:, None] - mean
    return (
        1
        / (sigma * sqrt(2 * pi))
        * np.exp((-1 * difference) * difference / ((2 * sigma) * sigma))
        * alpha
    )


def fit_height_gmm(heights: ArrayLike, n_components: int = 3) -> HeightGMMFit:
    """ComputeGMMfeature, including its nonstandard convergence/update sequence.

    At least three components are required: the source unconditionally checks
    means[0:3]. Empty/nonfinite input is reported explicitly at the API boundary.
    Finite inputs can still produce NaN parameters; no variance floor, log-space
    recovery, component sorting, reinitialization or library EM is substituted.
    """
    values = np.asarray(heights, dtype=np.float64)
    if values.ndim != 1 or not len(values) or not np.all(np.isfinite(values)):
        raise ValueError("height GMM requires a nonempty finite one-dimensional array")
    count = index(n_components)
    if count < 3:
        raise ValueError("the source GMM accesses three components; n_components must be >= 3")
    minimum = min(1e100, float(values.min()))
    maximum = max(-1e100, float(values.max()))
    with np.errstate(divide="ignore", invalid="ignore", over="ignore", under="ignore"):
        spacing = (maximum - minimum) / count
        mean = np.array([minimum + i * spacing for i in range(count)], dtype=np.float64)
        sigma = np.ones(count, dtype=np.float64)
        alpha = np.full(count, 1.0 / count, dtype=np.float64)
        iterations = 0
        while True:
            reasons = []
            if np.any(sigma == 0):
                reasons.append("zero_sigma")
            if abs(mean[-1] - mean[-2]) < 0.05:
                reasons.append("upper_means_close")
            if reasons:
                break

            probability = _density(values, mean, sigma, alpha)
            # Sequential cumulative sums retain the source's accumulation order.
            total = np.cumsum(probability, axis=1)[:, -1]
            probability = probability / total[:, None]
            mass = np.cumsum(probability, axis=0)[-1]
            weighted_height = np.cumsum(probability * values[:, None], axis=0)[-1]
            next_alpha = mass / len(values)
            next_mean = weighted_height / mass
            residual = values[:, None] - next_mean
            weighted_variance = np.cumsum((probability * residual) * residual, axis=0)[-1]
            next_sigma = np.sqrt(weighted_variance / mass)

            if abs(mean[0] - next_mean[0]) < 0.00001:
                # The source exits without assigning the newly computed parameters.
                reasons.append("first_mean_converged")
            else:
                mean, sigma, alpha = next_mean, next_sigma, next_alpha
            iterations += 1
            if abs(mean[0] - mean[1]) < 0.005 or abs(mean[1] - mean[2]) < 0.005:
                reasons.append("adjacent_means_close")
            if iterations > 100:
                reasons.append("iteration_limit")
            if reasons:
                break

    parameters = tuple(
        GMMParameters(float(m), float(s), float(a)) for m, s, a in zip(mean, sigma, alpha)
    )
    return HeightGMMFit(parameters, iterations, tuple(reasons))


def cluster_by_gmm(points: PointSelection, parameters: Sequence[GMMParameters]) -> GMMClustering:
    """ClusteringByGMM, with last-match ties and source max_element NaN behavior.

    The C++ routine returns immediately if a point has no matching maximum. The
    result preserves the already assigned prefix and reports its failed position.
    No point after that position is assigned, even if it would be classifiable.
    """
    if not parameters:
        raise ValueError("at least one GMM component is required")
    mean = np.array([p.mu for p in parameters], dtype=np.float64)
    sigma = np.array([p.sigma for p in parameters], dtype=np.float64)
    alpha = np.array([p.alpha for p in parameters], dtype=np.float64)
    with np.errstate(divide="ignore", invalid="ignore", over="ignore", under="ignore"):
        probability = _density(points.xyz[:, 2], mean, sigma, alpha)
    maximum = probability[:, 0].copy()
    for component in range(1, len(parameters)):
        take = maximum < probability[:, component]
        maximum[take] = probability[take, component]
    labels = np.full(len(points), -1, dtype=np.intp)
    for component in range(len(parameters)):
        labels[probability[:, component] == maximum] = component
    invalid = np.flatnonzero(labels < 0)
    failed = int(invalid[0]) if len(invalid) else None
    if failed is not None:
        labels[failed:] = -1
    clusters = tuple(points.where(labels == component) for component in range(len(parameters)))
    return GMMClustering(clusters, failed)


def classify_window(points: PointSelection) -> WindowClassification:
    fit = fit_height_gmm(points.xyz[:, 2])
    return WindowClassification(points, fit, cluster_by_gmm(points, fit.parameters))


def classify_segment(segment: TrackSegment) -> tuple[WindowClassification, WindowClassification]:
    """ComputeGMMParaAndCluster; no point labels are changed by this operation."""
    first, second = segment.track1.members, segment.track2.members
    if first is None or second is None:
        raise ValueError("assign both window memberships before classification")
    return classify_window(first), classify_window(second)
