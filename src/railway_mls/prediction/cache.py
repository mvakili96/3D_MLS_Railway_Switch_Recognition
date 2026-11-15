"""Immutable multi-tile point storage and stable external grid construction."""

from hashlib import sha256
from contextlib import ExitStack
import json
from math import ceil, isfinite
from operator import index
from pathlib import Path

import numpy as np

from .catalog import catalog_scene, _hash_file, _stat_identity, write_json_new
from .manifest import ADE_BASELINE, InferenceSpec, _read_json
from .resources import (MemoryBudget, ResourceLimitError, SceneCacheError, check_disk,
                        close_mapping, release_pages)


FORMAT_VERSION = 1
FILES = {
    "xyz": ("xyz.bin", "<f8", 3),
    "intensity": ("intensity.bin", "<i4", 1),
    "records": ("records.bin", "<i8", 1),
    "cell_ids": ("cell_ids.bin", "<i8", 1),
    "offsets": ("offsets.bin", "<i8", 1),
    "members": ("members.bin", "<i8", 1),
}


def _digest(value):
    return sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def compatibility_settings(spec):
    """Only settings affecting cached bytes/lookup; seeds and fitting parameters do not."""
    if not isinstance(spec, InferenceSpec):
        raise TypeError("expected an annotation-free InferenceSpec")
    if (spec.source_baseline != ADE_BASELINE or spec.record_selection != "per_file_even_zero_based"
            or spec.bounds_policy != "fixed_header_union" or spec.coordinate_units != "m"
            or spec.header_roundoff not in ("omit_outside_within_four_ulps", "reject_outside")):
        raise SceneCacheError("unsupported source, selection, coordinate or bounds policy")
    return {"format_version": FORMAT_VERSION, "source_baseline": spec.source_baseline,
            "tiles": [{"path": t.path.relative_to(spec.data_root).as_posix(), "sha256": t.sha256}
                      for t in spec.tiles],
            "record_selection": spec.record_selection, "bounds_policy": spec.bounds_policy,
            "header_roundoff": spec.header_roundoff, "coordinate_units": spec.coordinate_units,
            "grid_size": spec.config.grid_size,
            "decode_policy": "int32_xyz_times_float64_scale_plus_offset_v1"}


def grid_dimensions(bounds, grid_size):
    if not isfinite(grid_size) or grid_size <= 0:
        raise SceneCacheError("grid size must be positive and finite")
    low, high = bounds["minimum"], bounds["maximum"]
    spans = [(high[i]-low[i])/grid_size for i in (0, 1)]
    if not all(isfinite(span) for span in spans):
        raise SceneCacheError("grid extent cannot be represented by int64 cell IDs")
    nx, ny = (max(1, ceil(span)) for span in spans)
    if nx*ny > np.iinfo(np.int64).max:
        raise SceneCacheError("grid cell IDs exceed the int64 cache layout")
    return nx, ny


def insertion_cells(xyz, bounds, grid_size, nx, ny):
    """Literal GridIndex insertion: floor displacement BEFORE division and truncation."""
    low = bounds["minimum"]
    with np.errstate(all="ignore"):
        columns = np.minimum(nx-1, np.floor(xyz[:, 0]-low[0])/grid_size).astype(np.int64)
        rows = np.minimum(ny-1, np.floor(xyz[:, 1]-low[1])/grid_size).astype(np.int64)
    return rows*nx+columns


def _decoded_chunks(path, metadata, chunk_size, policy):
    import laspy
    low, high = np.array(metadata["bounds"]["minimum"]), np.array(metadata["bounds"]["maximum"])
    tolerance = 4*np.spacing(np.maximum(1, np.maximum(np.abs(low), np.abs(high))))
    scales, offsets = metadata["scales"], metadata["offsets"]
    first = 0
    with laspy.open(path) as reader:
        for chunk in reader.chunk_iterator(chunk_size):
            chosen = chunk[first % 2::2]
            ids = np.arange(first+first % 2, first+len(chunk), 2, dtype=np.int64)
            xyz = np.column_stack([np.asarray(getattr(chosen, axis), dtype=np.float64)*scales[i]+offsets[i]
                                   for i, axis in enumerate(("X", "Y", "Z"))])
            if not np.all(np.isfinite(xyz)):
                raise SceneCacheError(f"{path}: nonfinite decoded coordinates")
            inside = np.all((xyz >= low) & (xyz <= high), axis=1)
            omitted = int(np.count_nonzero(~inside))
            if omitted:
                if policy == "reject_outside":
                    raise SceneCacheError(f"{path}: decoded point outside declared tile bounds")
                if np.any(xyz < low-tolerance) or np.any(xyz > high+tolerance):
                    raise SceneCacheError(f"{path}: decoded point exceeds four-ULP header policy")
            yield (xyz[inside], np.asarray(chosen.intensity, dtype=np.int32)[inside],
                   ids[inside], omitted)
            first += len(chunk)
        if first != metadata["point_count"]:
            raise SceneCacheError(f"{path}: decoded record count differs from its header")


def _scatter_members(path, count, cell_ids, counts, chunk_size, memory, progress):
    offsets = np.empty(len(cell_ids)+1, dtype=np.int64)
    offsets[0] = 0
    np.cumsum(counts, out=offsets[1:])
    offsets.astype("<i8", copy=False).tofile(path/"offsets.bin")
    cell_ids.astype("<i8", copy=False).tofile(path/"cell_ids.bin")
    cursor = offsets[:-1].copy()
    members = np.memmap(path/"members.bin", dtype="<i8", mode="w+", shape=(count,))
    try:
        with (path/"cells.tmp").open("rb") as stream:
            first = 0
            while first < count:
                memory.require(chunk_size*64, location="index scatter",
                               reclaim=lambda: release_pages(members, writable=True))
                cells = np.fromfile(stream, dtype="<i8", count=chunk_size)
                if not len(cells):
                    raise SceneCacheError("incomplete temporary cell ledger")
                order = np.argsort(cells, kind="stable")
                ordered = cells[order]
                boundaries = np.r_[0, np.flatnonzero(np.diff(ordered))+1, len(order)]
                positions = np.searchsorted(cell_ids, ordered[boundaries[:-1]])
                for begin, end, position in zip(boundaries[:-1], boundaries[1:], positions):
                    start = int(cursor[position])
                    members[start:start+end-begin] = first+order[begin:end]
                    cursor[position] += end-begin
                first += len(cells)
                # Release dirty mappings after bounded blocks; source ordering is
                # entirely in the stable arrays, independent of paging decisions.
                release_pages(members, writable=True)
                memory.require(location="index scatter complete")
                if progress and (first == count or first % (chunk_size*20) == 0):
                    progress(f"Indexed {first:,}/{count:,} retained points")
        if not np.array_equal(cursor, offsets[1:]):
            raise SceneCacheError("cell membership counts do not reconcile")
    finally:
        close_mapping(members, writable=True)
    (path/"cells.tmp").unlink()


def build_scene_cache(spec, output, *, chunk_size=250_000, progress=None):
    """Build a NEW immutable cache. No evaluation inputs are available to this API."""
    settings = compatibility_settings(spec)
    chunk_size = index(chunk_size)
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    path = Path(output)
    if path.exists():
        raise FileExistsError(path)
    memory = MemoryBudget(spec.memory_bytes)
    memory.require(chunk_size*192, location="LAS decode preflight")
    source_stats = [_stat_identity(t.path.stat()) for t in spec.tiles]
    catalog = catalog_scene([t.path for t in spec.tiles], root=spec.data_root, progress=progress)
    if catalog["status"] != "ok":
        raise SceneCacheError(f"input catalog failed: {catalog['issues']}")
    for expected, actual, stat in zip(spec.tiles, catalog["tiles"], source_stats):
        if expected.sha256 != actual["sha256"]:
            raise SceneCacheError(f"input SHA256 mismatch: {expected.path}")
        if _stat_identity(expected.path.stat()) != stat:
            raise SceneCacheError(f"input changed during cataloging: {expected.path}")
    bounds = catalog["footprints"]["header_bounds_union"]
    nx, ny = grid_dimensions(bounds, spec.config.grid_size)
    upper = catalog["totals"]["source_even_record_count_before_boundary_policy"]
    # Worst case: 44 bytes/point permanent data, 16 bytes/occupied cell,
    # 8 bytes temporary cells and 8 bytes for one fresh pair of label arrays.
    disk_estimate = upper*76+1_048_576+len(spec.tiles)*4096
    check_disk(path, disk_estimate, spec.cache_disk_bytes)
    path.mkdir(parents=True, exist_ok=False)
    total, counts, ledger = 0, {}, []
    try:
        with ExitStack() as stack:
            streams = [stack.enter_context((path/name).open("xb"))
                       for name in ("xyz.bin", "intensity.bin", "records.bin", "cells.tmp")]
            for tile, metadata, stat in zip(spec.tiles, catalog["tiles"], source_stats):
                before = tile.path.stat()
                if _stat_identity(before) != stat:
                    raise SceneCacheError(f"input changed before decoding: {tile.path}")
                begin, omissions = total, 0
                for xyz, intensity, records, omitted in _decoded_chunks(
                        tile.path, metadata, chunk_size, spec.header_roundoff):
                    memory.require(chunk_size*128, location="LAS decode")
                    cells = insertion_cells(xyz, bounds, spec.config.grid_size, nx, ny)
                    unique, frequencies = np.unique(cells, return_counts=True)
                    memory.require(len(unique)*160, location="sparse cell counts")
                    for cell, frequency in zip(unique, frequencies):
                        key = int(cell)
                        counts[key] = counts.get(key, 0)+int(frequency)
                    for stream, (array, dtype) in zip(streams, ((xyz, "<f8"), (intensity, "<i4"),
                                                               (records, "<i8"), (cells, "<i8"))):
                        array.astype(dtype, copy=False).tofile(stream)
                    total += len(xyz)
                    omissions += omitted
                    memory.require(location="LAS decode complete")
                if _stat_identity(before) != _stat_identity(tile.path.stat()):
                    raise SceneCacheError(f"input changed during decoding: {tile.path}")
                ledger.append({**metadata, "row_start": begin, "row_end": total,
                               "retained_count": total-begin, "omitted_header_roundoff": omissions})
                if progress:
                    progress(f"Decoded {metadata['path']}: {total-begin:,} retained, {omissions} omitted")
        if not total:
            raise SceneCacheError("no points retained by the declared input policy")
        memory.require(len(counts)*48, location="sparse index arrays")
        cells = np.array(sorted(counts), dtype=np.int64)
        frequencies = np.array([counts[int(c)] for c in cells], dtype=np.int64)
        del counts
        _scatter_members(path, total, cells, frequencies, chunk_size, memory, progress)
        files = {}
        for key, (name, dtype, width) in FILES.items():
            rows = total if key in ("xyz", "intensity", "records", "members") else len(cells)+(key == "offsets")
            files[key] = {"path": name, "dtype": dtype,
                          "shape": [rows, width] if width > 1 else [rows],
                          "bytes": (path/name).stat().st_size, "sha256": _hash_file(path/name)}
        metadata = {"schema_version": FORMAT_VERSION, "kind": "prediction_scene_cache", "status": "complete",
                    "compatibility": settings, "compatibility_fingerprint": _digest(settings),
                    "source_scene_fingerprint": catalog["scene_fingerprint"], "point_count": total,
                    "tiles": ledger, "bounds": bounds,
                    "grid": {"grid_size": spec.config.grid_size, "num_x": nx, "num_y": ny,
                             "num_cells": nx*ny, "occupied_cells": len(cells)},
                    "files": files, "budgets": {"memory_bytes": spec.memory_bytes,
                                                  "cache_disk_bytes": spec.cache_disk_bytes},
                    "build": {"chunk_size": chunk_size, "estimated_peak_disk_bytes": disk_estimate,
                              "observed_peak_rss_bytes": memory.observed_peak,
                              "omitted_header_roundoff": sum(t["omitted_header_roundoff"] for t in ledger)},
                    "tracking_executed": False}
        metadata["manifest_sha256"] = _digest(metadata)
        write_json_new(path/"cache.json", metadata)
        return SceneCache(path, metadata, spec.memory_bytes, spec.cache_disk_bytes)
    except BaseException as error:
        # Only our newly created payloads are removed. Never touch source files.
        for name in (*[v[0] for v in FILES.values()], "cells.tmp", "cache.json"):
            (path/name).unlink(missing_ok=True)
        write_json_new(path/"failure.json", {"status": getattr(error, "status", "build_failed"), "error": str(error)})
        raise


class SceneCache:
    def __init__(self, path, metadata, memory_bytes, disk_bytes):
        self.path, self.metadata = Path(path), metadata
        self.memory_bytes, self.disk_bytes = memory_bytes, disk_bytes

    def new_run(self, directory):
        from .mapped import CachedScene
        return CachedScene(self, directory)


def open_scene_cache(path, *, inference=None):
    """Verify cached bytes; matching frozen inputs can be reused without rereading LAS."""
    path = Path(path)
    metadata = _read_json(path/"cache.json")
    if (not isinstance(metadata, dict) or metadata.get("kind") != "prediction_scene_cache"
            or type(metadata.get("schema_version")) is not int or metadata["schema_version"] != FORMAT_VERSION
            or metadata.get("status") != "complete"):
        raise SceneCacheError("not a complete supported scene cache")
    digest = metadata.pop("manifest_sha256", None)
    if digest != _digest(metadata):
        raise SceneCacheError("cache manifest SHA256 mismatch")
    metadata["manifest_sha256"] = digest
    if metadata["compatibility_fingerprint"] != _digest(metadata["compatibility"]):
        raise SceneCacheError("cache compatibility fingerprint is inconsistent")
    if inference is not None and _digest(compatibility_settings(inference)) != metadata["compatibility_fingerprint"]:
        raise SceneCacheError("scene cache is incompatible with the inference definition")
    budget = inference.memory_bytes if inference is not None else metadata["budgets"]["memory_bytes"]
    disk = inference.cache_disk_bytes if inference is not None else metadata["budgets"]["cache_disk_bytes"]
    MemoryBudget(budget).require(4*1024*1024, location="cache verification")
    if set(metadata["files"]) != set(FILES):
        raise SceneCacheError("cache payload table is incomplete")
    for key, (name, dtype, width) in FILES.items():
        entry = metadata["files"][key]
        n = metadata["point_count"] if key in ("xyz", "intensity", "records", "members") else metadata["grid"]["occupied_cells"]+(key == "offsets")
        shape = [n, width] if width > 1 else [n]
        expected_bytes = int(np.prod(shape, dtype=object))*np.dtype(dtype).itemsize
        if (entry["path"] != name or entry["dtype"] != dtype or entry["shape"] != shape
                or entry["bytes"] != expected_bytes or (path/name).stat().st_size != expected_bytes):
            raise SceneCacheError(f"cache payload layout/length mismatch: {name}")
        if _hash_file(path/name) != entry["sha256"]:
            raise SceneCacheError(f"cache payload SHA256 mismatch: {name}")
    return SceneCache(path, metadata, budget, disk)
