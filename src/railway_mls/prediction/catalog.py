"""Deterministic, read-only LAS inventories for prediction experiments.

This stage reads headers and hashes bytes; it never decodes a cloud for tracking,
imports classifications, extracts archives or changes source sampling.
"""

from hashlib import sha256
import json
import math
import os
from pathlib import Path
import re


class CatalogError(ValueError):
    pass


def _natural_key(path):
    return [((1, int(part)) if part.isdecimal() else (0, part.casefold()))
            for part in re.split(r"(\d+)", path.name)] + [(2, path.name)]


def _stat_identity(stat):
    return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns


def _hash_file(path):
    digest = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4*1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def discover_tiles(sources):
    """Preserve explicit source order; naturally sort direct LAS/LAZ directory entries."""
    result = []
    for source in sources:
        path = Path(source).resolve()
        if path.is_dir():
            children = sorted((child for child in path.iterdir()
                               if child.is_file() and child.suffix.casefold() in (".las", ".laz")),
                              key=_natural_key)
            if not children:
                raise CatalogError(f"{path}: no extracted LAS/LAZ files in this directory")
            result.extend(child.resolve() for child in children)
        elif path.suffix.casefold() in (".las", ".laz"):
            result.append(path)
        else:
            raise CatalogError(f"{path}: supply an extracted LAS/LAZ file or directory")
    if not result:
        raise CatalogError("at least one LAS/LAZ source is required")
    seen = set()
    for path in result:
        # Also catch hard links, which would otherwise inventory one file twice.
        identity = (path.stat().st_dev, path.stat().st_ino) if path.is_file() else str(path)
        if identity in seen:
            raise CatalogError(f"{path}: duplicate source file")
        seen.add(identity)
    return tuple(result)


def read_tile_metadata(path):
    """Record metadata and a complete file hash without reading point arrays."""
    try:
        import laspy
    except ImportError as error:
        raise ImportError("LAS catalogs require: python -m pip install -e '.[las]'") from error
    path = Path(path)
    before = path.stat()
    try:
        reader = laspy.open(path)
    except laspy.errors.LaspyException as error:
        raise CatalogError(f"{path}: invalid LAS header: {error}") from error
    with reader:
        header = reader.header
        count = int(header.point_count)
        scales, offsets = list(map(float, header.scales)), list(map(float, header.offsets))
        minimum, maximum = list(map(float, header.mins)), list(map(float, header.maxs))
        if (not all(math.isfinite(v) for v in scales+offsets+minimum+maximum)
                or any(s <= 0 for s in scales) or any(a > b for a, b in zip(minimum, maximum))):
            raise CatalogError(f"{path}: invalid/nonfinite LAS scales, offsets or bounds")
        if count <= 0:
            raise CatalogError(f"{path}: empty cloud cannot be a prediction input")
        compressed = bool(header.are_points_compressed)
        payload_end = int(header.offset_to_point_data) + count*header.point_format.size
        if not compressed and before.st_size < payload_end:
            raise CatalogError(f"{path}: truncated LAS point payload")
        record = {
            "bytes": before.st_size, "point_count": count,
            "source_even_record_count_before_boundary_policy": (count+1)//2,
            "las_version": str(header.version), "point_format": header.point_format.id,
            "record_bytes": header.point_format.size,
            "point_data_offset": int(header.offset_to_point_data), "compressed": compressed,
            "scales": scales, "offsets": offsets,
            "bounds": {"minimum": minimum, "maximum": maximum},
            "legacy_layout": header.version.major == 1 and header.version.minor <= 3
                             and header.point_format.id <= 5,
        }
    record["sha256"] = _hash_file(path)
    if _stat_identity(before) != _stat_identity(path.stat()):
        raise CatalogError(f"{path}: input changed while cataloging")
    return record


def _axis_gaps(tiles, axis):
    intervals = sorted((tile["bounds"]["minimum"][axis], tile["bounds"]["maximum"][axis])
                       for tile in tiles)
    gaps = []
    if intervals:
        end = intervals[0][1]
        for lower, upper in intervals[1:]:
            if lower > end:
                gaps.append({"minimum": end, "maximum": lower, "width": lower-end})
            end = max(end, upper)
    return gaps


def footprint_summary(tiles):
    """Header rectangles only. Axis gaps and rectangle components are distinct."""
    components, unseen = [], set(range(len(tiles)))
    while unseen:
        first = min(unseen)
        unseen.remove(first)
        pending, component = [first], []
        while pending:
            current = pending.pop()
            component.append(current)
            a = tiles[current]["bounds"]
            for index in sorted(unseen):
                b = tiles[index]["bounds"]
                if all(a["minimum"][axis] <= b["maximum"][axis]
                       and b["minimum"][axis] <= a["maximum"][axis] for axis in (0, 1)):
                    unseen.remove(index)
                    pending.append(index)
        components.append([tiles[i]["path"] for i in sorted(component)])
    bounds = ({"minimum": [min(t["bounds"]["minimum"][i] for t in tiles) for i in range(3)],
               "maximum": [max(t["bounds"]["maximum"][i] for t in tiles) for i in range(3)]}
              if tiles else None)
    return {"header_bounds_union": bounds, "xy_rectangle_components": components,
            "x_axis_gaps": _axis_gaps(tiles, 0), "y_axis_gaps": _axis_gaps(tiles, 1),
            "point_occupancy_verified": False, "rail_continuity_verified": False,
            "interpretation": "Header rectangles and axis projections only; connectivity does not "
                              "establish point occupancy, rail continuity or study-area completeness."}


def catalog_scene(sources, *, root=None, progress=None):
    """Inventory extracted files only, preserving an explicit ordered tile ledger."""
    root = Path.cwd().resolve() if root is None else Path(root).resolve()
    paths = discover_tiles(sources)
    tiles, issues = [], []
    for order, path in enumerate(paths):
        relative = Path(os.path.relpath(path, root)).as_posix()
        if progress:
            progress(f"[{order+1}/{len(paths)}] Cataloging {relative}")
        try:
            tile = {"order": order, "path": relative, **read_tile_metadata(path)}
            tiles.append(tile)
            if not tile["legacy_layout"]:
                issues.append({"path": relative, "status": "unsupported_prediction_layout",
                               "message": "Source prediction profile requires LAS <=1.3, formats 0–5"})
        except (OSError, ValueError) as error:
            issues.append({"path": relative, "status": "input_error", "message": str(error)})
    # This identifies ordered file contents independently of the machine's root path.
    fingerprint = sha256(json.dumps(tiles, sort_keys=True, separators=(",", ":"),
                                   allow_nan=False).encode()).hexdigest()
    return {"schema_version": 1, "kind": "prediction_scene_catalog",
            "status": "ok" if not issues else "incomplete", "root": str(root),
            "ordered_inputs": [Path(os.path.relpath(p, root)).as_posix() for p in paths],
            "order_policy": "explicit source order; natural filename order within each directory",
            "scene_fingerprint": fingerprint, "tiles": tiles, "issues": issues,
            "totals": {"requested_files": len(paths), "cataloged_files": len(tiles),
                       "point_count": sum(t["point_count"] for t in tiles),
                       "source_even_record_count_before_boundary_policy":
                           sum(t["source_even_record_count_before_boundary_policy"] for t in tiles),
                       "bytes": sum(t["bytes"] for t in tiles)},
            "footprints": footprint_summary(tiles),
            "validation_scope": "Header, byte-length and SHA256 checks; point records not decoded. "
                                "No archives extracted, coordinates converted or input data modified."}


def write_json_new(path, payload):
    """Write only a new file, keeping JSON deterministic and finite."""
    text = json.dumps(payload, indent=2, allow_nan=False)+"\n"
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(text)
