"""Stateful source coordinate conversion and explicit legacy file workflows."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, replace
from math import isfinite
from operator import index
from pathlib import Path

import numpy as np

from .coordinates import (
    CONVERTER_PI,
    ConverterEllipsoid,
    _int32,
    ecef_to_geodetic,
    geodetic_to_ecef,
    lat_lon_to_utm_xy,
    utm_xy_to_lat_lon,
)
from .legacy_io import LegacyPointCloud, ProgressCallback, load_point_cloud
from .points import PointCloud


@dataclass
class ConvertedPoint:
    x: float
    y: float
    z: float
    class_code: int = 0
    segment_id: int = -1
    intensity: int = 0

    def as_array(self):
        return np.array((self.x, self.y, self.z), dtype=np.float64)


@dataclass(frozen=True)
class CoordinateConversion:
    points: tuple[ConvertedPoint, ...]
    direction: str
    zone_argument: int
    south_hemi: bool | None
    zones: tuple[int, ...]
    status: str

    @property
    def reference_comparable(self):
        return self.status == "ok"

    def as_cloud(self) -> PointCloud:
        """An explicit owning core cloud; its existing finite-coordinate guard applies."""
        return PointCloud(
            np.array([p.as_array() for p in self.points]).reshape(-1, 3),
            [p.intensity for p in self.points],
            class_codes=[p.class_code for p in self.points],
            segment_ids=[p.segment_id for p in self.points],
        )


class CoordinateConverter:
    """DataConvertor: append inputs, then reconvert the ENTIRE persistent buffer.

    Input arrays/labels are read at each append. Result records are value copies;
    the public converted_points list retains reached mutations on callback errors
    or undefined automatic-zone casts. No input coordinates are overwritten.
    """

    def __init__(self, point_cloud: PointCloud):
        if not isinstance(point_cloud, PointCloud):
            raise TypeError("supply an owning PointCloud")
        self.point_cloud = point_cloud
        self.converted_points: list[ConvertedPoint] = []
        self.ellipsoid = ConverterEllipsoid()

    def _progress(self, callback):
        total, progress = len(self.point_cloud) * 3, 0

        def tick():
            nonlocal progress
            progress += 1
            if callback is not None and progress % 10 == 0:
                callback(progress, total)

        return tick

    def _append(self, tick):
        row = 0
        while row < len(self.point_cloud):
            cloud = self.point_cloud
            self.converted_points.append(
                ConvertedPoint(
                    *map(float, cloud.xyz[row]),
                    int(cloud.class_codes[row]),
                    int(cloud.segment_ids[row]),
                    int(cloud.intensity[row]),
                )
            )
            row += 1
            tick()

    def _finish(self, direction, zone, south_hemi, zones):
        points = tuple(replace(p) for p in self.converted_points)
        finite = all(isfinite(v) for p in points for v in (p.x, p.y, p.z))
        return CoordinateConversion(
            points,
            direction,
            zone,
            south_hemi,
            tuple(zones),
            "ok" if finite else "nonfinite_conversion",
        )

    def convert_ecef_to_utm(
        self, zone: int, *, callback: ProgressCallback | None = None
    ) -> CoordinateConversion:
        zone = _int32(zone)
        tick = self._progress(callback)
        self._append(tick)
        row = 0
        while row < len(self.converted_points):
            point = self.converted_points[row]
            lat, lon, height = ecef_to_geodetic(point.x, point.y, point.z, ellipsoid=self.ellipsoid)
            point.x, point.y, point.z = lat * 180 / CONVERTER_PI, lon * 180 / CONVERTER_PI, height
            row += 1
            tick()
        zones, row = [], 0
        while row < len(self.converted_points):
            point = self.converted_points[row]
            used, point.x, point.y = lat_lon_to_utm_xy(point.x, point.y, zone)
            zones.append(used)  # Source ignores the return; exposed only as diagnostics.
            row += 1
            tick()
        return self._finish("ecef_to_utm", zone, None, zones)

    def convert_utm_to_ecef(
        self, zone: int, south_hemi: bool, *, callback: ProgressCallback | None = None
    ) -> CoordinateConversion:
        zone = _int32(zone)
        tick = self._progress(callback)
        self._append(tick)
        row = 0
        while row < len(self.converted_points):
            point = self.converted_points[row]
            point.x, point.y = utm_xy_to_lat_lon(point.x, point.y, zone, south_hemi)
            row += 1
            tick()
        row = 0
        while row < len(self.converted_points):
            point = self.converted_points[row]
            point.x, point.y, point.z = geodetic_to_ecef(
                point.x, point.y, point.z, ellipsoid=self.ellipsoid
            )
            row += 1
            tick()
        return self._finish(
            "utm_to_ecef", zone, bool(south_hemi), [zone] * len(self.converted_points)
        )


@dataclass(frozen=True)
class ConvertedExport:
    text: str
    declared_count: int
    rows_written: int
    status: str

    @property
    def reference_comparable(self):
        return self.status == "ok"


def _emit_converted_points(points, delimiter, callback, stream=None):
    if not isinstance(delimiter, str) or "\0" in delimiter:
        raise ValueError("delimiter must be a string without embedded NUL")
    delimiter.encode("ascii")
    declared, rows, nonfinite = len(points), 0, False
    lines = []

    def emit(text):
        lines.append(text)
        if stream is not None:
            stream.write(text)
            stream.flush()  # Source endl flushes before the callback.

    emit(str(declared) + "\n")
    while rows < len(points):
        p = points[rows]
        nonfinite = nonfinite or not all(isfinite(v) for v in (p.x, p.y, p.z))
        emit(
            delimiter.join((f"{p.x:.3f}", f"{p.y:.3f}", f"{p.z:.3f}", str(index(p.intensity))))
            + "\n"
        )
        rows += 1
        if callback is not None and rows % 10 == 0:
            callback(rows, declared)
    return ConvertedExport(
        "".join(lines), declared, rows, "nonfinite_coordinate_export" if nonfinite else "ok"
    )


def format_converted_points(
    points: Sequence[ConvertedPoint], *, delimiter=" ", callback: ProgressCallback | None = None
) -> ConvertedExport:
    """SaveConvertedPoints: count, fixed three decimals, intensity; no final callback."""
    return _emit_converted_points(points, delimiter, callback)


def write_converted_points(
    path,
    points: Sequence[ConvertedPoint],
    *,
    delimiter=" ",
    callback: ProgressCallback | None = None,
    overwrite=False,
) -> ConvertedExport:
    with Path(path).open("w" if overwrite else "x", encoding="ascii", newline="\n") as stream:
        return _emit_converted_points(points, delimiter, callback, stream)


@dataclass(frozen=True)
class LegacyConversionFile:
    source: Path
    destination: Path | None
    loaded: LegacyPointCloud | None
    conversion: CoordinateConversion | None
    export: ConvertedExport | None
    status: str


def source_menu_conversion(
    path, direction: str, *, destination=None, overwrite=False
) -> LegacyConversionFile:
    """MainForm's standalone menu actions: zone 32, inverse hemisphere false.

    Explicit optional output replaces the independent save dialog; a supplied
    input replaces the open dialog. Managed cancellation/reentry is not simulated.
    """
    if direction not in ("ecef_to_utm", "utm_to_ecef"):
        raise ValueError("select ecef_to_utm or utm_to_ecef")
    loaded = load_point_cloud(path, format="las" if direction == "ecef_to_utm" else "pts")
    converter = CoordinateConverter(loaded.cloud)
    result = (
        converter.convert_ecef_to_utm(32)
        if direction == "ecef_to_utm"
        else converter.convert_utm_to_ecef(32, False)
    )
    out = Path(destination) if destination is not None else None
    exported = (
        write_converted_points(out, converter.converted_points, overwrite=overwrite)
        if out is not None
        else None
    )
    return LegacyConversionFile(Path(path), out, loaded, result, exported, result.status)


def convert_legacy_files(
    paths,
    destination,
    direction: str,
    zone: int,
    *,
    callback: ProgressCallback | None = None,
    all_files_callback: ProgressCallback | None = None,
    overwrite=False,
) -> tuple[LegacyConversionFile, ...]:
    """DataConverstionForm's ordered batch actions with explicit Python paths.

    Its UTM-to-ECEF action hardcodes southhemi=false. .las uses LAS, .pts uses
    PTS, ALL other suffixes use legacy CSV; only exactly .csv produces CSV output.
    Missing sources are skipped while remaining in the nominal progress total.
    """
    if direction not in ("ecef_to_utm", "utm_to_ecef"):
        raise ValueError("select ecef_to_utm or utm_to_ecef")
    zone = _int32(zone)
    paths, destination = tuple(map(Path, paths)), Path(destination)
    if not destination.is_dir():
        raise NotADirectoryError(destination)
    total, progress, results = 3 * len(paths), 0, []

    def step():
        nonlocal progress
        progress += 1
        if all_files_callback is not None:
            all_files_callback(progress, total)

    for path in paths:
        if not path.is_file():
            results.append(
                LegacyConversionFile(path, None, None, None, None, "missing_source_skipped")
            )
            continue
        suffix = path.suffix
        source_format = "las" if suffix == ".las" else "pts" if suffix == ".pts" else "csv"
        output_csv = suffix == ".csv"
        out = destination / (
            path.stem
            + ("_utm" if direction == "ecef_to_utm" else "_ecef")
            + (".csv" if output_csv else ".pts")
        )
        loaded = load_point_cloud(path, format=source_format, callback=callback)
        converter = CoordinateConverter(loaded.cloud)
        step()
        if direction == "ecef_to_utm":
            converted = converter.convert_ecef_to_utm(zone, callback=callback)
            step()
        else:
            step()  # Native advances before, rather than after, conversion here.
            converted = converter.convert_utm_to_ecef(zone, False, callback=callback)
        exported = write_converted_points(
            out,
            converter.converted_points,
            delimiter="," if output_csv else " ",
            callback=callback,
            overwrite=overwrite,
        )
        step()
        results.append(
            LegacyConversionFile(path, out, loaded, converted, exported, converted.status)
        )
    return tuple(results)
