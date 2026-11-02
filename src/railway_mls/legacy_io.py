"""Source PTS/CSV/LAS adapters and rail-vector formats, separate from the core.

LAS decoding is an optional laspy boundary; native record selection and field
assignment are retained. No CRS conversion, label import or density repair.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass
from math import isfinite
from operator import index
from pathlib import Path
import re
from typing import TYPE_CHECKING

import numpy as np

from .models import TrackLayer
from .config import ExtractionConfig
from .points import Point3D, PointCloud
from .sampling import MsvcRand, RandSource
from .spatial import Bounds3D, GridIndex

ProgressCallback = Callable[[int, int], None]
if TYPE_CHECKING:
    from .orchestration import TrackingEngine


class LegacyIOError(ValueError):
    """Explicit unsupported/undefined source input, with the first reached row."""

    def __init__(self, status: str, row: int | None = None):
        self.status, self.row = status, row
        super().__init__(status + (f" at record {row}" if row is not None else ""))


@dataclass(frozen=True)
class ReaderBounds:
    """Literal reader outputs, including text Z=0 and empty XY sentinels."""

    minimum: Point3D = Point3D(1e100, 1e100, 0)
    maximum: Point3D = Point3D(-1e100, -1e100, 0)

    def include_xy(self, point: Point3D) -> ReaderBounds:
        # Conditional MIN/MAX shims: the second operand wins equal comparisons.
        lo, hi = self.minimum, self.maximum
        return ReaderBounds(
            Point3D(lo.x if lo.x < point.x else point.x, lo.y if lo.y < point.y else point.y, lo.z),
            Point3D(hi.x if hi.x > point.x else point.x, hi.y if hi.y > point.y else point.y, hi.z),
        )


@dataclass(frozen=True)
class LegacyPointCloud:
    cloud: PointCloud
    reader_bounds: ReaderBounds
    source_format: str
    declared_count: int
    input_count: int
    record_indices: np.ndarray

    def grid_bounds(self) -> Bounds3D:
        """Preserve source XY; explicitly enclose Z for Python's validated grid.

        Native grid construction uses only X/Y. Text reader Z is initialized to
        zero, not measured. Reported reader bounds themselves are never changed.
        """
        if not len(self.cloud):
            raise LegacyIOError("empty_legacy_cloud_requires_explicit_grid_bounds")
        low, high = self.reader_bounds.minimum, self.reader_bounds.maximum
        return Bounds3D(
            Point3D(low.x, low.y, min(low.z, float(self.cloud.xyz[:, 2].min()))),
            Point3D(high.x, high.y, max(high.z, float(self.cloud.xyz[:, 2].max()))),
        )

    def make_grid(self, grid_size: float = 1.0) -> GridIndex:
        return GridIndex(self.cloud, grid_size, bounds=self.grid_bounds())


_INTEGER = re.compile(r"\s*([+-]?\d+)", re.ASCII)
_FLOAT = re.compile(
    r"\s*([+-]?(?:0[xX](?:[0-9a-fA-F]+(?:\.[0-9a-fA-F]*)?|\.[0-9a-fA-F]+)[pP][+-]?\d+"
    r"|(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?|inf(?:inity)?|nan(?:\([^)]*\))?))",
    re.ASCII | re.IGNORECASE,
)


def _atoi(token: str) -> int:
    match = _INTEGER.match(token)
    value = int(match[1]) if match else 0
    if not -(1 << 31) <= value < (1 << 31):
        raise LegacyIOError("undefined_source_text_integer_range")
    return value


def _atof(token: str) -> float:
    match = _FLOAT.match(token)
    if not match:
        return 0.0
    value = match[1]
    if "nan" in value.lower():
        return float("nan")
    try:
        return float.fromhex(value) if "0x" in value.lower() else float(value)
    except OverflowError:
        return float("-inf" if value.startswith("-") else "inf")


def _line(stream, row):
    text = stream.readline(201)
    if not text:
        raise LegacyIOError("undefined_source_missing_text_record", row)
    if text.endswith("\n"):
        text = text[:-1].removesuffix("\r")  # Native Windows text-mode CRLF.
    if len(text) > 199:
        raise LegacyIOError("unsupported_source_text_line_length", row)
    if "\0" in text:
        raise LegacyIOError("unsupported_source_text_embedded_nul", row)
    return text


def _result(xyz, intensity, bounds, format, declared, count, ids):
    rows = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    if not np.all(np.isfinite(rows)):
        raise LegacyIOError("nonfinite_legacy_coordinates")
    cloud = PointCloud(rows, intensity)
    record_indices = np.asarray(ids, dtype=np.int64)
    record_indices.flags.writeable = False
    return LegacyPointCloud(cloud, bounds, format, declared, count, record_indices)


class LegacyTextReader:
    """Count-prefixed TXTFileReader, retaining its loaded-data identity/state."""

    def __init__(self):
        # Native PointXYZDouble constructor initializes both points to XYZ=0.
        self.bounds = ReaderBounds(Point3D(0, 0, 0), Point3D(0, 0, 0))
        self.result: LegacyPointCloud | None = None

    def loaded_data(self) -> PointCloud | None:
        return self.result.cloud if self.result is not None else None

    def read(
        self, path, *, format: str = "pts", callback: ProgressCallback | None = None
    ) -> LegacyPointCloud:
        if format not in ("pts", "csv"):
            raise ValueError("legacy text format must be 'pts' or 'csv'")
        delimiter = " " if format == "pts" else ","
        xyz, intensity = [], []
        with Path(path).open(encoding="ascii", newline="\n") as stream:
            old = self.bounds
            self.bounds = ReaderBounds(
                Point3D(1e100, 1e100, old.minimum.z), Point3D(-1e100, -1e100, old.maximum.z)
            )
            declared = _atoi(_line(stream, -1))
            for row in range(max(0, declared)):
                # strtok collapses repeated delimiters and skips empty fields.
                tokens = [part for part in _line(stream, row).split(delimiter) if part]
                if len(tokens) < 4:
                    raise LegacyIOError("undefined_source_missing_text_field", row)
                point = Point3D(*map(_atof, tokens[:3]))
                if not all(isfinite(v) for v in point.as_array()):
                    raise LegacyIOError("nonfinite_legacy_coordinates", row)
                xyz.append(point.as_array())
                intensity.append(_atoi(tokens[3]))
                self.bounds = self.bounds.include_xy(point)
                if callback is not None and (row + 1) % 10 == 0:
                    callback(row + 1, declared)
        result = _result(xyz, intensity, self.bounds, format, declared, len(xyz), range(len(xyz)))
        self.result = result
        if callback is not None:
            callback(declared, declared)
        return result


def _laspy():
    try:
        import laspy
    except ImportError as error:
        raise ImportError('LAS/LAZ reading requires: python -m pip install -e ".[las]"') from error
    return laspy


class LegacyLasReader:
    """LASFileReader field/record rules using chunked optional LAS/LAZ decoding."""

    def __init__(self):
        self.result: LegacyPointCloud | None = None

    def loaded_data(self) -> PointCloud | None:
        return self.result.cloud if self.result is not None else None

    def read_bounds(self, path) -> ReaderBounds:
        with _laspy().open(path) as stream:
            return ReaderBounds(Point3D(*stream.header.mins), Point3D(*stream.header.maxs))

    def read(
        self, path, *, callback: ProgressCallback | None = None, chunk_size: int = 1_000_000
    ) -> LegacyPointCloud:
        size = index(chunk_size)
        if size <= 0:
            raise ValueError("chunk_size must be positive")
        xyz, intensity, ids, count = [], [], [], 0
        with _laspy().open(path) as stream:
            declared = int(stream.header.point_count)
            bounds = ReaderBounds(Point3D(*stream.header.mins), Point3D(*stream.header.maxs))
            for chunk in stream.chunk_iterator(size):
                record_ids = np.arange(count, count + len(chunk), dtype=np.int64)
                keep = record_ids % 2 == 0
                selected = chunk[keep]
                xyz.append(np.column_stack((selected.x, selected.y, selected.z)))
                intensity.append(np.asarray(selected.intensity, dtype=np.int32))
                ids.append(record_ids[keep])
                if callback is not None:
                    for current in range((count // 10 + 1) * 10, count + len(chunk) + 1, 10):
                        callback(current, declared)
                count += len(chunk)
        if count != declared:
            raise LegacyIOError("unsupported_las_record_count_mismatch")
        result = _result(
            np.vstack(xyz) if xyz else [],
            np.concatenate(intensity) if intensity else [],
            bounds,
            "las",
            declared,
            count,
            np.concatenate(ids) if ids else [],
        )
        self.result = result
        if callback is not None:
            callback(declared, declared)
        return result


def load_point_cloud(
    path,
    *,
    format: str | None = None,
    callback: ProgressCallback | None = None,
    chunk_size: int = 1_000_000,
) -> LegacyPointCloud:
    """Explicit legacy formats; ordinary CSV is not inferred to have this schema."""
    selected = format if format is not None else Path(path).suffix.removeprefix(".")
    if selected in ("las", "laz"):
        return LegacyLasReader().read(path, callback=callback, chunk_size=chunk_size)
    if selected in ("pts", "csv"):
        return LegacyTextReader().read(path, format=selected, callback=callback)
    raise ValueError("select a legacy pts/csv/las/laz format")


@dataclass(frozen=True)
class LegacyFileEntry:
    loaded: LegacyPointCloud
    engine: TrackingEngine


class LegacyFileSession:
    """MainForm's numeric file switching/cache, with explicit Python ownership.

    Exact filename strings key the map; lookup inserts a null miss as HashMap::get
    does. Cached engines retain labels/layers and all engines share one RNG.
    Reader callbacks exclude the separate native grid/display progress UI.
    """

    def __init__(self, *, config: ExtractionConfig | None = None, rng: RandSource | None = None):
        self.config = config if config is not None else ExtractionConfig()
        self.rng = rng if rng is not None else MsvcRand()
        self.files: dict[str, LegacyFileEntry | None] = {}
        self.current: LegacyFileEntry | None = None

    def _entry(self, loaded):
        from .orchestration import TrackingEngine

        self.current = LegacyFileEntry(
            loaded,
            TrackingEngine(
                loaded.make_grid(self.config.grid_size), config=self.config, rng=self.rng
            ),
        )
        return self.current

    def load_file(
        self, path, *, callback: ProgressCallback | None = None, chunk_size: int = 1_000_000
    ) -> LegacyFileEntry | None:
        key = str(path)
        cached = self.files.setdefault(key, None)
        if cached is not None:
            self.current = cached
            return cached
        if Path(path).suffix not in (".las", ".laz", ".pts"):
            return None
        loaded = load_point_cloud(path, callback=callback, chunk_size=chunk_size)
        entry = self._entry(loaded)
        self.files[key] = entry
        return entry

    def open_pts(self, path) -> LegacyFileEntry:
        """Direct PTS menu creates a fresh engine without registering a cache key."""
        return self._entry(LegacyTextReader().read(path))


@dataclass(frozen=True)
class LegacyRailExport:
    text: str
    declared_count: int
    rows_written: int
    failed_index: int | None
    status: str

    @property
    def reference_comparable(self):
        return self.status == "ok"


def format_rail_vector(
    center: Sequence[Point3D],
    rail1: Sequence[Point3D] | None = None,
    rail2: Sequence[Point3D] | None = None,
    *,
    indexed: bool = False,
) -> LegacyRailExport:
    """Three source save variants; undefined indexed access returns its prefix."""
    paired = rail1 is not None and rail2 is not None
    if (rail1 is None) != (rail2 is None) or (indexed and not paired):
        raise ValueError("supply both rails for paired/indexed export")
    declared = (
        len(rail1) * 3 if indexed else len(center) + (len(rail1) + len(rail2) if paired else 0)
    )
    lines, rows, nonfinite = [str(declared)], 0, False

    def row(point, tag, i=None):
        nonlocal rows, nonfinite
        nonfinite = nonfinite or not all(isfinite(v) for v in point.as_array())
        value = f"{point.x:.3f},{point.y:.3f},{point.z:.3f},{index(tag)}"
        lines.append(value if i is None else value + f",{i}")
        rows += 1

    def finish(status="ok", failed=None):
        if status == "ok" and nonfinite:
            status = "nonfinite_legacy_export"
        return LegacyRailExport("\n".join(lines) + "\n", declared, rows, failed, status)

    if not indexed:
        for point in center:
            row(point, point.point_type)
        if paired:
            for rail in (rail1, rail2):
                for point in rail:
                    row(point, 100)
        return finish()
    offset = 0
    for i, left in enumerate(rail1):
        row(left, 100, i)
        if i + offset >= len(center):
            return finish("undefined_source_indexed_center", i)
        if center[i + offset].point_type == 2:
            offset += 1  # Skip exactly one marker, not all consecutive markers.
        if i + offset >= len(center):
            return finish("undefined_source_indexed_center", i)
        row(center[i + offset], center[i + offset].point_type, i)
        if i >= len(rail2):
            return finish("undefined_source_indexed_rail2", i)
        row(rail2[i], 200, i)
    return finish()


def write_rail_vector(
    path, center, rail1=None, rail2=None, *, indexed=False, overwrite=False
) -> LegacyRailExport:
    result = format_rail_vector(center, rail1, rail2, indexed=indexed)
    if result.status.startswith("undefined_source"):
        raise LegacyIOError(result.status, result.failed_index)
    with Path(path).open("w" if overwrite else "x", encoding="ascii", newline="\n") as stream:
        stream.write(result.text)
    return result


def write_layer_vectors(
    path, layer: TrackLayer, *, indexed=False, overwrite=False
) -> tuple[Path, ...]:
    """Explicit layer from the UI action; paired file suffix follows MainForm."""
    base = Path(path)
    if not base.suffix:
        raise ValueError("layer export requires a filename extension")
    paired = base.with_name(base.stem + "_with_track_points.csv")
    center, left, right = layer.center_vector, layer.rail1_vector, layer.rail2_vector
    if indexed:
        write_rail_vector(paired, center, left, right, indexed=True, overwrite=overwrite)
        return (paired,)
    write_rail_vector(base, center, overwrite=overwrite)
    write_rail_vector(paired, center, left, right, overwrite=overwrite)
    return base, paired
