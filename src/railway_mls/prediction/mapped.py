"""Mapped implementations of the existing cloud/selection/grid interfaces."""

import json
from contextlib import contextmanager
from copy import copy
from operator import index
from pathlib import Path

import numpy as np

from ..points import PointCloud, PointSelection, Point3D
from ..spatial import Bounds3D, GridIndex
from .resources import (MemoryBudget, ResourceLimitError, check_disk, close_mapping,
                        release_pages)


def _map(cache, key):
    entry = cache.metadata["files"][key]
    return np.memmap(cache.path/entry["path"], dtype=entry["dtype"], mode="r", shape=tuple(entry["shape"]))


class MappedPointSelection(PointSelection):
    @property
    def xyz(self):
        return self.cloud.read_xyz(self.indices)

    def set_class_code(self, value):
        self.cloud.require_open()
        self.cloud.memory.require(location="class label update", reclaim=self.cloud.release_pages)
        super().set_class_code(value)

    def set_segment_id(self, value):
        self.cloud.require_open()
        self.cloud.memory.require(location="segment label update", reclaim=self.cloud.release_pages)
        super().set_segment_id(value)


class MappedPointCloud(PointCloud):
    """Read-only coordinate/intensity snapshot plus fresh run-owned mutable labels.

    The base constructor is deliberately bypassed: it copies complete arrays.
    All cached points were validated while building the immutable snapshot.
    """

    def __init__(self, cache, run_directory, memory, *, private_labels=False):
        self.cache, self.memory = cache, memory
        self._private_labels = private_labels
        self._closed, self._arrays = False, []
        try:
            for attribute, key in (("_xyz", "xyz"), ("intensity", "intensity"), ("_records", "records")):
                array = _map(cache, key)
                self._arrays.append((array, False))
                setattr(self, attribute, array)
            for name in ("class_codes", "segment_ids"):
                array = np.memmap(
                    Path(run_directory)/(name+".bin"), dtype="<i4",
                    mode="c" if private_labels else "r+", shape=(len(self),),
                )
                self._arrays.append((array, True))
                setattr(self, name, array)
        except BaseException:
            self.close()
            raise

    def require_open(self):
        if self._closed:
            raise RuntimeError("mapped cloud is closed")

    @property
    def xyz(self):
        self.require_open()
        return super().xyz

    def select(self, indices=None):
        self.require_open()
        if indices is not None:
            indices = np.asarray(indices)
            if indices.ndim != 1:
                raise ValueError("indices must be a one-dimensional integer array")
        size = len(self) if indices is None else len(indices)
        self.memory.require(size*24, location="point selection", reclaim=self.release_pages)
        return MappedPointSelection(
            self, np.arange(len(self), dtype=np.intp) if indices is None else indices)

    def read_xyz(self, rows):
        self.require_open()
        self.memory.require(len(rows)*48, location="coordinate gather", reclaim=self.release_pages)
        try:
            result = self._xyz[rows]
            self.memory.require(location="coordinate gather complete", reclaim=self.release_input_pages)
            return result
        finally:
            self.release_input_pages()

    def point(self, row):
        row = index(row)
        if row < 0 or row >= len(self):
            raise IndexError("point row is outside the cloud")
        return Point3D(*self.read_xyz(np.array([row], dtype=np.int64))[0])

    def source_records(self, rows):
        """Resolve retained row IDs to original manifest tile ordinal and LAS record."""
        self.require_open()
        rows = np.asarray(rows)
        if rows.ndim != 1 or (rows.size and (not np.issubdtype(rows.dtype, np.integer)
                                            or np.any(rows < 0) or np.any(rows >= len(self)))):
            raise IndexError("invalid retained point IDs")
        self.memory.require(len(rows)*32, location="source identity lookup", reclaim=self.release_pages)
        rows = rows.astype(np.int64)
        ends = np.array([tile["row_end"] for tile in self.cache.metadata["tiles"]], dtype=np.int64)
        tiles, records = np.searchsorted(ends, rows, side="right"), self._records[rows].copy()
        self.release_input_pages()
        return tiles, records

    def release_input_pages(self):
        if not self._closed:
            for array, writable in self._arrays:
                if not writable:
                    release_pages(array)

    def release_pages(self):
        if not self._closed:
            for array, writable in self._arrays:
                # Discarding a private label page would silently reset its changes.
                if not writable or not self._private_labels:
                    release_pages(array, writable=writable)

    def close(self):
        if not self._closed:
            for array, writable in self._arrays:
                close_mapping(array, writable=writable and not self._private_labels)
            self._closed = True


class MappedGridIndex(GridIndex):
    """Stable sparse cell runs; reuse native-compatible bounds/query arithmetic."""

    def __init__(self, cache, cloud):
        self.cloud, self.memory = cloud, cloud.memory
        bounds, grid = cache.metadata["bounds"], cache.metadata["grid"]
        self.bounds = Bounds3D(Point3D(*bounds["minimum"]), Point3D(*bounds["maximum"]))
        self.grid_size = grid["grid_size"]
        self.num_x, self.num_y, self.num_cells = grid["num_x"], grid["num_y"], grid["num_cells"]
        self._arrays, self._closed = [], False
        try:
            for name in ("cell_ids", "offsets", "members"):
                array = _map(cache, name)
                self._arrays.append(array)
                setattr(self, "_"+name, array)
        except BaseException:
            self.close()
            raise

    @property
    def occupied_cells(self):
        return len(self._cell_ids)

    def members(self, cell):
        self.cloud.require_open()
        if self._closed:
            raise RuntimeError("mapped grid is closed")
        cell = index(cell)
        if cell < 0 or cell >= self.num_cells:
            raise IndexError("cell is outside the grid")
        position = int(np.searchsorted(self._cell_ids, cell))
        if position == len(self._cell_ids) or self._cell_ids[position] != cell:
            return self.cloud.select([])
        first, end = map(int, self._offsets[position:position+2])
        self.memory.require((end-first)*24, location="cell members", reclaim=self.cloud.release_pages)
        try:
            result = self.cloud.select(self._members[first:end])
        finally:
            for array in self._arrays:
                release_pages(array)
        return result

    def neighbor_points(self, x, y, neighborhood_size=5):
        groups = [self.members(cell).indices for cell in self.neighbor_indices(x, y, neighborhood_size)
                  if cell >= 0]
        size = sum(len(group) for group in groups)
        self.memory.require(size*24, location="ordered neighborhood", reclaim=self.cloud.release_pages)
        return self.cloud.select(np.concatenate(groups) if groups else [])

    def close(self):
        if not self._closed:
            for array in self._arrays:
                close_mapping(array)
            self._closed = True


class CachedScene:
    """One fresh label state and grid view. Use as a context manager."""

    def __init__(self, cache, directory):
        self.cache, self.path = cache, Path(directory)
        self.cloud = self.grid = None
        if self.path.exists():
            raise FileExistsError(self.path)
        count = cache.metadata["point_count"]
        memory = MemoryBudget(cache.memory_bytes)
        memory.require(4*1024*1024, location="fresh label state")
        state_bytes = count*8+4096
        cache_bytes = (sum(entry["bytes"] for entry in cache.metadata["files"].values())
                       + (cache.path/"cache.json").stat().st_size)
        if cache_bytes+state_bytes > cache.disk_bytes:
            raise ResourceLimitError("cache plus fresh labels exceed cache/state disk budget")
        check_disk(self.path, state_bytes, cache.disk_bytes)
        self.path.mkdir(parents=True, exist_ok=False)
        try:
            for name, value in (("class_codes", 0), ("segment_ids", -1)):
                with (self.path/(name+".bin")).open("xb") as stream:
                    for first in range(0, count, 250_000):
                        memory.require(1_000_000, location="label initialization")
                        np.full(min(250_000, count-first), value, dtype="<i4").tofile(stream)
            self.cloud = MappedPointCloud(cache, self.path, memory)
            self.grid = MappedGridIndex(cache, self.cloud)
            (self.path/"state.json").write_text(json.dumps({
                "schema_version": 1, "kind": "fresh_prediction_labels", "status": "ready",
                "cache_manifest_sha256": cache.metadata["manifest_sha256"],
                "point_count": count, "initial_class_code": 0, "initial_segment_id": -1,
                "tracking_executed_at_creation": False}, indent=2)+"\n")
        except BaseException as error:
            self.close()
            for name in ("class_codes.bin", "segment_ids.bin"):
                (self.path/name).unlink(missing_ok=True)
            (self.path/"failure.json").write_text(json.dumps({
                "status": getattr(error, "status", "state_creation_failed"), "error": str(error)})+"\n")
            raise

    def close(self):
        if self.grid is not None:
            self.grid.close()
        if self.cloud is not None:
            self.cloud.close()

    @contextmanager
    def private_run(self):
        """Open a copy-on-write label state over this scene's scratch files."""
        self.cloud.require_open()
        scene = copy(self)
        scene.cloud = MappedPointCloud(
            self.cache, self.path, MemoryBudget(self.cache.memory_bytes), private_labels=True
        )
        scene.grid = None
        with scene:
            scene.grid = MappedGridIndex(self.cache, scene.cloud)
            yield scene

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
