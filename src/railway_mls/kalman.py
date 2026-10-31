"""The source's three-coordinate Kalman recurrence and bundled LU arithmetic."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, pi, sin

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .points import Point3D


def _matrix(values: ArrayLike) -> NDArray[np.float64]:
    matrix = np.array(values, dtype=np.float64, copy=True)
    if matrix.shape != (3, 3):
        raise ValueError("Kalman matrices must have shape (3, 3)")
    matrix.flags.writeable = False
    return matrix


@dataclass(frozen=True)
class KalmanState:
    point: Point3D
    covariance: NDArray[np.float64] = field(default_factory=lambda: np.eye(3))

    def __post_init__(self) -> None:
        if not isinstance(self.point, Point3D):
            raise TypeError("Kalman state point must be a Point3D")
        object.__setattr__(self, "covariance", _matrix(self.covariance))

    @property
    def finite(self) -> bool:
        return bool(np.isfinite(self.point.as_array()).all() and np.isfinite(self.covariance).all())


@dataclass(frozen=True)
class KalmanPrediction:
    state: KalmanState
    transition: NDArray[np.float64]
    process_noise: NDArray[np.float64]
    status: str

    @property
    def success(self) -> bool:
        return self.status == "ok"


@dataclass(frozen=True)
class KalmanUpdate:
    state: KalmanState
    gain: NDArray[np.float64]
    innovation: Point3D
    measurement_noise: NDArray[np.float64]
    inverse_valid: bool
    zero_pivots: int
    status: str

    @property
    def success(self) -> bool:
        return self.status == "ok"


def _multiply(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """matrix.inl's zero-initialized, sequential k accumulation, including 0*NaN.

    These matrices are at most 3x3. Avoid BLAS reassociation or skipping zeros:
    both can change the source's nonfinite propagation and pivot decisions.
    """
    result = np.zeros((first.shape[0], second.shape[1]), dtype=np.float64)
    for k in range(first.shape[1]):
        result += first[:, k, None] * second[None, k, :]
    return result


def _source_inverse(matrix: np.ndarray) -> tuple[np.ndarray, bool, int]:
    """etc/matrix.inl:503–664, including vv[i], last pivot ties and TINY.

    An all-zero row returns an initialized ZERO inverse with its valid flag false.
    KalmanUpdate ignores that flag in C++; retain the matrix and report it. A zero
    pivot after elimination is replaced with 1e-20, as in the bundled helper.
    """
    lu = matrix.copy()
    inverse = np.zeros((3, 3), dtype=np.float64)
    scale = np.zeros(3, dtype=np.float64)
    indices = np.zeros(3, dtype=np.intp)
    for row in range(3):
        big = 0.0
        for column in range(3):
            # Argument order also preserves std::max's behavior with NaNs.
            big = max(abs(lu[row, column]), big)
        if big == 0:
            return inverse, False, 0
        scale[row] = 1 / big
    last_pivot = 0
    zero_pivots = 0
    for column in range(3):
        for row in range(column):
            value = lu[row, column]
            for k in range(row):
                value -= lu[row, k] * lu[k, column]
            lu[row, column] = value
        big = 0.0
        for row in range(column, 3):
            value = lu[row, column]
            for k in range(column):
                value -= lu[row, k] * lu[k, column]
            lu[row, column] = value
            merit = scale[column] * abs(value)  # Source uses vv[i], not vv[j].
            if merit >= big:
                big, last_pivot = merit, row
        if column != last_pivot:
            lu[[column, last_pivot]] = lu[[last_pivot, column]]
            scale[last_pivot] = scale[column]
        indices[column] = last_pivot
        if lu[column, column] == 0:
            lu[column, column] = 1e-20
            zero_pivots += 1
        # The source tests j != n-1 AFTER its row loop, where j == n. The
        # resulting final-column reciprocal has no following rows to modify.
        reciprocal = 1 / lu[column, column]
        for row in range(column + 1, 3):
            lu[row, column] *= reciprocal
    for column in range(3):
        back = np.zeros(3, dtype=np.float64)
        back[column] = 1
        first_nonzero = 0
        for row in range(3):
            pivot = indices[row]
            value = back[pivot]
            back[pivot] = back[row]
            if first_nonzero != 0:
                for k in range(first_nonzero - 1, row):
                    value -= lu[row, k] * back[k]
            elif value != 0:
                first_nonzero = row + 1
            back[row] = value
        for row in range(2, -1, -1):
            value = back[row]
            for k in range(row + 1, 3):
                value -= lu[row, k] * back[k]
            back[row] = value / lu[row, row]
        inverse[:, column] = back
    return inverse, True, zero_pivots


def kalman_predict(
    current: KalmanState, *, mode: int, angle: float, step_length: float
) -> KalmanPrediction:
    """RailwayExtraction::KalmanPredict; angle is in source folded degrees."""
    radians = angle * pi / 180
    # Preserve native trig domain NaNs without Python math's domain exception.
    cosine = cos(radians) if np.isfinite(radians) else float("nan")
    sine = sin(radians) if np.isfinite(radians) else float("nan")
    dx, dy = abs(step_length * cosine), abs(step_length * sine)
    sign_x = -1 if mode in (2, 3) else 1
    sign_y = 1 if mode in (1, 2) else -1
    transition = np.zeros((3, 3), dtype=np.float64)
    process = np.diag([0.005, 0.005, 0.01])
    with np.errstate(divide="ignore", invalid="ignore", over="ignore", under="ignore"):
        transition[0, 0] = 1 + sign_x * (np.divide(1.0, current.point.x) * dx)
        transition[1, 1] = 1 + sign_y * (np.divide(1.0, current.point.y) * dy)
        transition[2, 2] = 1
        point = _multiply(transition, current.point.as_array()[:, None])[:, 0]
        covariance = _multiply(_multiply(transition, current.covariance), transition.T) + process
    state = KalmanState(Point3D(*point), covariance)
    return KalmanPrediction(
        state,
        _matrix(transition),
        _matrix(process),
        "ok" if state.finite else "nonfinite_prediction",
    )


def kalman_update(predicted: KalmanState, observation: Point3D, sigma: float) -> KalmanUpdate:
    """RailwayExtraction::KalmanUpdate; R's diagonal is sigma, NOT sigma squared."""
    identity = np.eye(3)
    measurement = np.zeros((3, 3), dtype=np.float64)
    np.fill_diagonal(measurement, sigma)
    with np.errstate(divide="ignore", invalid="ignore", over="ignore", under="ignore"):
        pht = _multiply(predicted.covariance, identity.T)
        hpht = _multiply(_multiply(identity, predicted.covariance), identity.T)
        inverse, valid, zero_pivots = _source_inverse(hpht + measurement)
        gain = _multiply(pht, inverse)
        innovation = observation.as_array()[:, None] - _multiply(
            identity, predicted.point.as_array()[:, None]
        )
        point = predicted.point.as_array()[:, None] + _multiply(gain, innovation)
        covariance = _multiply(identity - _multiply(gain, identity), predicted.covariance)
    state = KalmanState(Point3D(*point[:, 0]), covariance)
    status = (
        "nonfinite_update"
        if not state.finite
        else "source_invalid_inverse"
        if not valid
        else "source_zero_pivot"
        if zero_pivots
        else "ok"
    )
    return KalmanUpdate(
        state,
        _matrix(gain),
        Point3D(*innovation[:, 0]),
        _matrix(measurement),
        valid,
        zero_pivots,
        status,
    )
