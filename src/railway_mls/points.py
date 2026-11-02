"""Double precision point storage with stable identities and shared labels."""

from __future__ import annotations

from dataclasses import dataclass, replace
from copy import copy
from math import sqrt
from operator import index

import numpy as np
from numpy.typing import ArrayLike, NDArray

POINT_EQUAL_TOLERANCE = 1e-5


@dataclass(frozen=True)
class Point3D:
    """A value point corresponding to POINT3D; coordinates must be explicit.

    The C++ default constructor leaves XYZ uninitialized. Python deliberately
    requires coordinates instead of manufacturing a value for uninitialized data.
    Arithmetic creates a new point with point_type=0, as in the source operators.
    """

    x: float
    y: float
    z: float
    point_type: int = 0

    def __post_init__(self) -> None:
        for name in ("x", "y", "z"):
            object.__setattr__(self, name, float(getattr(self, name)))

    def __add__(self, other: Point3D) -> Point3D:
        return Point3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Point3D) -> Point3D:
        return Point3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> Point3D:
        return Point3D(self.x * scalar, self.y * scalar, self.z * scalar)

    def __truediv__(self, scalar: float) -> Point3D:
        # Keep IEEE NaN/inf on zero division, rather than substituting a direction.
        with np.errstate(divide="ignore", invalid="ignore"):
            values = np.divide(self.as_array(), scalar)
        return Point3D(*values)

    def as_array(self) -> NDArray[np.float64]:
        return np.array((self.x, self.y, self.z), dtype=np.float64)

    def is_equal(self, other: Point3D) -> bool:
        """Source IsEqual: strict absolute tolerance, independent of PtType."""
        return (
            abs(self.x - other.x) < POINT_EQUAL_TOLERANCE
            and abs(self.y - other.y) < POINT_EQUAL_TOLERANCE
            and abs(self.z - other.z) < POINT_EQUAL_TOLERANCE
        )

    def dot(self, other: Point3D) -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: Point3D) -> Point3D:
        return Point3D(
            self.y * other.z - other.y * self.z,
            other.x * self.z - self.x * other.z,
            self.x * other.y - other.x * self.y,
        )

    def normalized(self) -> Point3D:
        value = self / sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
        return replace(value, point_type=self.point_type)

    def assigned_xyz(self, other: Point3D) -> Point3D:
        """Value equivalent of native assignment: retain this destination's tag."""
        return replace(self, x=other.x, y=other.y, z=other.z)

    def set_coordinates(self, x: float, y: float, z: float) -> Point3D:
        """SetPoint3D resets the tag; frozen Python points return a new value."""
        return Point3D(x, y, z)

    def changed_sign(self) -> Point3D:
        return replace(self, x=-self.x, y=-self.y, z=-self.z)


@dataclass
class Point2D:
    """POINT2D: the XY constructor leaves flg unknown; default construction does not."""

    x: float | None = None
    y: float | None = None
    flag: bool | None = None

    def __post_init__(self):
        if self.x is None and self.y is None:
            self.x, self.y, self.flag = 0.0, 0.0, False
        elif self.x is None or self.y is None:
            raise ValueError("supply both POINT2D coordinates")
        else:
            self.x, self.y = float(self.x), float(self.y)

    @staticmethod
    def _arithmetic(x, y):
        result = Point2D()
        result.x, result.y = float(x), float(y)
        return result

    def __add__(self, other):
        return self._arithmetic(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return self._arithmetic(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return self._arithmetic(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar):
        return copy(self) if scalar == 0 else self._arithmetic(self.x / scalar, self.y / scalar)

    def is_equal(self, other):
        return abs(self.x - other.x) < 1e-5 and abs(self.y - other.y) < 1e-5

    def is_exactly_equal(self, other):
        return self.x == other.x and self.y == other.y


@dataclass
class LegacyLiDPoint:
    """Unused LiDPOINT state. Its broken division has no portable numeric result."""

    x: float | None = None
    y: float | None = None
    z: float | None = None
    segment_id: int = -1
    class_code: int = 0
    intensity: int = 0
    alpha: float = 0.0
    height_over_ground: float = 0.0
    delta_z: float = 0.0
    deleted: bool = False

    def __post_init__(self):
        if any(v is not None for v in (self.x, self.y, self.z)):
            self.set_xyz(self.x, self.y, self.z)
        self.intensity = index(self.intensity) & 0xFFFF

    def coordinates(self):
        if any(v is None for v in (self.x, self.y, self.z)):
            raise ValueError("undefined source LiDPOINT coordinates")
        return self.x, self.y, self.z

    def set_xyz(self, x, y, z):
        if any(v is None for v in (x, y, z)):
            raise ValueError("supply all LiDPOINT coordinates")
        self.x, self.y, self.z = float(x), float(y), float(z)

    def set_from(self, other):
        self.set_xyz(*other.coordinates())

    def is_equal(self, other):
        return all(abs(a - b) < 1e-5 for a, b in zip(self.coordinates(), other.coordinates()))

    def __add__(self, other):
        return LegacyLiDPoint(*(a + b for a, b in zip(self.coordinates(), other.coordinates())))

    def __sub__(self, other):
        return LegacyLiDPoint(*(a - b for a, b in zip(self.coordinates(), other.coordinates())))

    def __truediv__(self, scalar):
        raise ValueError(
            "undefined_source_lidpoint_division: native divides uninitialized output XYZ"
        )


def _integer_vector(values: ArrayLike, length: int, name: str) -> NDArray[np.int32]:
    raw = np.asarray(values)
    if raw.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},)")
    if not np.issubdtype(raw.dtype, np.number) or np.iscomplexobj(raw):
        raise ValueError(f"{name} must contain integers")
    limits = np.iinfo(np.int32)
    if (
        not np.all(np.isfinite(raw))
        or np.any(raw != np.trunc(raw))
        or np.any(raw < limits.min)
        or np.any(raw > limits.max)
    ):
        raise ValueError(f"{name} must contain signed 32-bit integers")
    return np.array(raw, dtype=np.int32, copy=True)


class PointCloud:
    """Own XYZ/intensity/labels; a row index is the equivalent of a point pointer.

    Coordinates are copied to float64, kept in input order, and made read-only so
    an existing grid cannot become stale. Equal coordinates are never deduplicated.
    Class codes and segment IDs are shared mutable int32 arrays. Selection label
    changes write back to these arrays, not to independent point copies.
    """

    def __init__(
        self,
        xyz: ArrayLike,
        intensity: ArrayLike | None = None,
        *,
        class_codes: ArrayLike | None = None,
        segment_ids: ArrayLike | None = None,
    ) -> None:
        self._xyz = np.array(xyz, dtype=np.float64, order="C", copy=True)
        if self._xyz.ndim != 2 or self._xyz.shape[1] != 3:
            raise ValueError("xyz must have shape (N, 3); use an explicit (0, 3) empty array")
        if not np.all(np.isfinite(self._xyz)):
            raise ValueError("xyz must contain finite coordinates")
        self._xyz.flags.writeable = False
        size = len(self._xyz)
        self.intensity = (
            np.zeros(size, dtype=np.int32)
            if intensity is None
            else _integer_vector(intensity, size, "intensity")
        )
        self.class_codes = (
            np.zeros(size, dtype=np.int32)
            if class_codes is None
            else _integer_vector(class_codes, size, "class_codes")
        )
        self.segment_ids = (
            np.full(size, -1, dtype=np.int32)
            if segment_ids is None
            else _integer_vector(segment_ids, size, "segment_ids")
        )

    @property
    def xyz(self) -> NDArray[np.float64]:
        result = self._xyz.view()
        result.flags.writeable = False
        return result

    def __len__(self) -> int:
        return len(self._xyz)

    def point(self, row: int) -> Point3D:
        row = index(row)
        if row < 0 or row >= len(self):
            raise IndexError("point row is outside the cloud")
        return Point3D(*self._xyz[row])

    def select(self, indices: ArrayLike | None = None) -> PointSelection:
        if indices is None:
            indices = np.arange(len(self), dtype=np.intp)
        return PointSelection(self, indices)


@dataclass(frozen=True, eq=False)
class PointSelection:
    """Ordered references into one cloud, including repeated indices if supplied.

    xyz returns gathered coordinates. Set labels with set_class_code/set_segment_id
    or directly through cloud.class_codes[indices]/cloud.segment_ids[indices].
    """

    cloud: PointCloud
    indices: NDArray[np.intp]

    def __post_init__(self) -> None:
        raw = np.asarray(self.indices)
        if raw.ndim != 1 or (raw.size and not np.issubdtype(raw.dtype, np.integer)):
            raise ValueError("indices must be a one-dimensional integer array")
        if raw.size and (np.any(raw < 0) or np.any(raw >= len(self.cloud))):
            raise IndexError("point selection contains an out-of-range row")
        rows = np.array(raw, dtype=np.intp, copy=True)
        rows.flags.writeable = False
        object.__setattr__(self, "indices", rows)

    def __len__(self) -> int:
        return len(self.indices)

    @property
    def xyz(self) -> NDArray[np.float64]:
        return self.cloud.xyz[self.indices]

    def where(self, mask: ArrayLike) -> PointSelection:
        mask = np.asarray(mask)
        if mask.dtype != np.bool_ or mask.shape != (len(self),):
            raise ValueError("selection mask must be a boolean vector of matching length")
        return self.cloud.select(self.indices[mask])

    def set_class_code(self, value: int) -> None:
        self.cloud.class_codes[self.indices] = _integer_vector([value], 1, "class code")[0]

    def set_segment_id(self, value: int) -> None:
        self.cloud.segment_ids[self.indices] = _integer_vector([value], 1, "segment ID")[0]
