"""Standalone LASFileReader PointXYZ helpers from bundled PCL 1.8.0.

Derived numerical subset: retain the notices in PCL_LICENSE.txt. The reached
native bodies and dependency hashes are frozen in pcl_reference.json. Helpers
stay outside loading/tracking, as in the original application.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import floor, isfinite
from operator import index

import numpy as np

from .points import PointCloud


@dataclass(frozen=True)
class PclHeader:
    sequence: int = 0
    stamp: int = 0
    frame_id: str = ""

    def __post_init__(self):
        if not 0 <= index(self.sequence) < 1 << 32 or not 0 <= index(self.stamp) < 1 << 64:
            raise ValueError("PCL header sequence/stamp must fit uint32/uint64")


class PclXYZCloud:
    """Owned float32 PointXYZ values; labels/intensity are not part of this type.

    Width/height default to native default-cloud metadata (0/0), independently
    of point-vector length. Density is explicit and is not inferred from data.
    XYZ and sensor arrays are copied/read-only. No implicit core conversion.
    """

    def __init__(
        self,
        xyz,
        *,
        is_dense=True,
        width=0,
        height=0,
        header=None,
        sensor_origin=(0, 0, 0, 0),
        sensor_orientation=(0, 0, 0, 1),
    ):
        with np.errstate(over="ignore", invalid="ignore"):
            values = np.array(xyz, dtype=np.float32, copy=True, order="C")
        if values.ndim != 2 or values.shape[1] != 3:
            raise ValueError("PointXYZ must have shape (N,3); use explicit (0,3) for empty input")
        self.xyz = values
        self.xyz.flags.writeable = False
        self.width, self.height = index(width), index(height)
        if not 0 <= self.width < 1 << 32 or not 0 <= self.height < 1 << 32:
            raise ValueError("PCL width/height must fit uint32")
        self.is_dense = bool(is_dense)
        self.header = header if header is not None else PclHeader()
        if not isinstance(self.header, PclHeader):
            raise TypeError("supply a PclHeader")
        for name, vector in (
            ("sensor_origin", sensor_origin),
            ("sensor_orientation", sensor_orientation),
        ):
            values = np.array(vector, dtype=np.float32, copy=True)
            if values.shape != (4,):
                raise ValueError(name + " must contain four values (quaternion order XYZW)")
            values.flags.writeable = False
            setattr(self, name, values)

    def __len__(self):
        return len(self.xyz)

    @classmethod
    def from_core(cls, cloud: PointCloud) -> PclXYZCloud:
        """Explicit narrowing; PointXYZ carries no algorithm labels or intensity."""
        if not isinstance(cloud, PointCloud):
            raise TypeError("supply a PointCloud")
        return cls(cloud.xyz, width=len(cloud), height=1)

    def as_core(self) -> PointCloud:
        """Explicit new core cloud with default labels and the core finite guard."""
        return PointCloud(self.xyz)

    def copied(self, xyz=None, **metadata) -> PclXYZCloud:
        values = {
            "is_dense": self.is_dense,
            "width": self.width,
            "height": self.height,
            "header": self.header,
            "sensor_origin": self.sensor_origin,
            "sensor_orientation": self.sensor_orientation,
        }
        values.update(metadata)
        return PclXYZCloud(self.xyz if xyz is None else xyz, **values)


@dataclass(frozen=True)
class VoxelDownsample:
    cloud: PclXYZCloud | None
    status: str
    minimum: tuple[float, ...] | None
    maximum: tuple[float, ...] | None
    extent_counts: tuple[int, ...] | None
    minimum_voxel: tuple[int, ...] | None
    maximum_voxel: tuple[int, ...] | None
    divisions: tuple[int, ...] | None
    sorted_rows: tuple[int, ...]
    groups: tuple[tuple[int, ...], ...]
    sort_sensitive_voxels: tuple[int, ...]
    native_order_supplied: bool
    failed_row: int | None

    @property
    def reference_comparable(self):
        return (
            self.cloud is not None
            and not self.status.startswith("nonfinite")
            and (not self.sort_sensitive_voxels or self.native_order_supplied)
        )


def decrease_density(cloud: PclXYZCloud | None, *, native_sorted_rows=None) -> VoxelDownsample:
    """Fixed 0.1f VoxelGrid<PointXYZ>; retain float32 and source integer guards.

    Default equal-key order is stable input order. A supplied native std::sort
    row permutation enables matched-order float32 accumulation. Unsafe native
    casts/overflow return no output; the source's defined extent fallback copies
    the complete original cloud, including invalid non-dense rows/organization.
    """
    minimum = maximum = counts = low = high = divisions = None
    order, groups, sensitive = (), (), ()
    failed = None
    supplied = native_sorted_rows is not None

    def finish(status, output=None):
        return VoxelDownsample(
            output,
            status,
            minimum,
            maximum,
            counts,
            low,
            high,
            divisions,
            order,
            groups,
            sensitive,
            supplied,
            failed,
        )

    if cloud is None:
        return finish("undefined_source_null_pcl_cloud")
    if not isinstance(cloud, PclXYZCloud):
        raise TypeError("use PclXYZCloud, or explicitly narrow with PclXYZCloud.from_core")
    if len(cloud) > (1 << 31) - 1:
        return finish("unsupported_source_pcl_index_count")
    finite = np.isfinite(cloud.xyz).all(axis=1)
    if cloud.is_dense and not finite.all():
        # Dense promises finite values. Eigen scalar/SIMD NaN bounds can differ;
        # do not invent a portable bounds/fallback outcome for contradictory data.
        return finish("unsupported_source_dense_nonfinite")
    rows = np.arange(len(cloud), dtype=np.int64) if cloud.is_dense else np.flatnonzero(finite)
    smallest = np.full(3, np.finfo(np.float32).max, dtype=np.float32)
    largest = -smallest.copy()
    for row in rows:
        smallest = np.minimum(smallest, cloud.xyz[row])
        largest = np.maximum(largest, cloud.xyz[row])
    minimum, maximum = tuple(map(float, smallest)), tuple(map(float, largest))
    inverse = np.float32(np.float32(1.0) / np.float32(0.1))
    with np.errstate(over="ignore", invalid="ignore"):
        spans = np.float32(np.float32(largest - smallest) * inverse)
    sizes = []
    for value in spans:
        if not isfinite(float(value)) or not -(1 << 63) <= float(value) < 1 << 63:
            return finish("undefined_source_voxel_extent_cast")
        dimension = int(value) + 1
        if not -(1 << 63) <= dimension < 1 << 63:
            return finish("undefined_source_voxel_extent_increment")
        sizes.append(dimension)
    counts = tuple(sizes)
    product = sizes[0] * sizes[1]
    if not -(1 << 63) <= product < 1 << 63:
        return finish("undefined_source_voxel_extent_product")
    product *= sizes[2]
    if not -(1 << 63) <= product < 1 << 63:
        return finish("undefined_source_voxel_extent_product")
    if product > (1 << 31) - 1:
        return finish("extent_fallback", cloud.copied())
    with np.errstate(over="ignore", invalid="ignore"):
        lower, upper = np.float32(smallest * inverse), np.float32(largest * inverse)
    endpoints = []
    for values in (lower, upper):
        result = []
        for value in values:
            if not isfinite(float(value)) or not -(1 << 31) <= float(value) < 1 << 31:
                return finish("undefined_source_voxel_bound_cast")
            result.append(floor(float(value)))
        endpoints.append(tuple(result))
    low, high = endpoints
    dims = []
    for a, b in zip(low, high):
        difference = b - a
        if not -(1 << 31) <= difference < (1 << 31) - 1:
            return finish("undefined_source_voxel_division_count")
        dims.append(difference + 1)
    divisions = tuple(dims)
    xy = dims[0] * dims[1]
    if not -(1 << 31) <= xy < 1 << 31:
        return finish("undefined_source_voxel_division_product")
    keys = {}
    for row in rows:
        failed = int(row)
        values = []
        for axis in range(3):
            scaled = np.float32(cloud.xyz[row, axis] * inverse)
            # Unqualified native floor is double; min_b is explicitly float32.
            value = floor(float(scaled)) - float(np.float32(low[axis]))
            if not -(1 << 31) <= value < 1 << 31:
                return finish("undefined_source_voxel_relative_cast")
            values.append(int(value))
        key = 0
        for value, multiplier in zip(values, (1, dims[0], xy)):
            term = value * multiplier
            if not -(1 << 31) <= term < 1 << 31:
                return finish("undefined_source_voxel_index_product")
            key += term
            if not -(1 << 31) <= key < 1 << 31:
                return finish("undefined_source_voxel_index_sum")
        keys[int(row)] = key & 0xFFFFFFFF
    failed = None
    if supplied:
        order = tuple(map(index, native_sorted_rows))
        if sorted(order) != sorted(keys) or any(
            keys[a] > keys[b] for a, b in zip(order, order[1:])
        ):
            raise ValueError(
                "native_sorted_rows must contain each eligible row once in voxel-key order"
            )
    else:
        order = tuple(sorted(keys, key=keys.get))
    grouped = []
    for row in order:
        if not grouped or keys[row] != keys[grouped[-1][0]]:
            grouped.append([])
        grouped[-1].append(row)
    groups = tuple(map(tuple, grouped))
    sensitive = tuple(
        i
        for i, group in enumerate(groups)
        if len(group) > 2 and not np.all(cloud.xyz[list(group)] == cloud.xyz[group[0]])
    )
    output = []
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        for group in groups:
            total = np.zeros(3, dtype=np.float32)
            for row in group:
                total += cloud.xyz[row]
            output.append(total / np.float32(len(group)))
    xyz = np.array(output, dtype=np.float32).reshape(-1, 3)
    status = "ok_with_native_sort_ties" if sensitive and not supplied else "ok"
    if not np.isfinite(xyz).all():
        status = "nonfinite_voxel_centroid"
    return finish(status, cloud.copied(xyz, width=len(xyz), height=1, is_dense=True))


@dataclass(frozen=True)
class PclCentering:
    cloud: PclXYZCloud | None
    centroid: tuple[float, ...] | None
    centroid_count: int
    matrix: tuple[tuple[float, ...], ...] | None
    status: str

    @property
    def reference_comparable(self):
        return self.status in ("ok", "empty")


def center_point_cloud(cloud: PclXYZCloud | None) -> PclCentering:
    """Source Transform: double centroid, double identity affine, float32 output.

    Literal four-term products retain 0*NaN/Inf behavior; invalid non-dense rows
    are copied unchanged. Rotation stays commented out. Empty input retains the
    wrapper's initial Vector4d::Identity centroid (1,0,0,0).
    """
    if cloud is None:
        return PclCentering(None, None, 0, None, "undefined_source_null_pcl_cloud")
    if not isinstance(cloud, PclXYZCloud):
        raise TypeError("use a PclXYZCloud")
    centroid = np.array((1, 0, 0, 0), dtype=np.float64)
    count = 0
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        if len(cloud):
            centroid[:] = 0
            for point in cloud.xyz:
                if not cloud.is_dense and not np.isfinite(point).all():
                    continue
                centroid[:3] += point.astype(np.float64)
                count += 1
            centroid /= np.float64(count)
            centroid[3] = 1
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, 3] = -centroid[:3]
        xyz = cloud.xyz.copy()
        for i, point in enumerate(cloud.xyz):
            if not cloud.is_dense and not np.isfinite(point).all():
                continue
            x, y, z = map(float, point)
            for axis in range(3):
                xyz[i, axis] = (
                    float(matrix[axis, 0]) * x
                    + float(matrix[axis, 1]) * y
                    + float(matrix[axis, 2]) * z
                    + float(matrix[axis, 3])
                )
    finite = np.isfinite(xyz).all() and np.isfinite(centroid).all()
    status = "empty" if not len(cloud) else "ok" if finite else "nonfinite_pcl_centering"
    return PclCentering(
        cloud.copied(xyz),
        tuple(map(float, centroid)),
        count,
        tuple(tuple(map(float, row)) for row in matrix),
        status,
    )
