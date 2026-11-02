"""Defaults from RailwayExtraction.cpp:24–37, without algorithm corrections."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from math import isfinite


def source_int32(value: float) -> int:
    """Defined C++ double-to-int assignment: truncate, then require int32 range."""
    value = float(value)
    if not isfinite(value):
        raise ValueError("undefined source double-to-int conversion")
    result = int(value)
    if not -(2**31) <= result < 2**31:
        raise ValueError("undefined source double-to-int conversion")
    return result


@dataclass(frozen=True)
class ExtractionParameters:
    """The source's ten *double* input fields; no uninitialized defaults.

    These are distinct from the engine's four integer members. Text formatting,
    .NET locale parsing and the Windows dialog itself are outside this adapter.
    """

    track_window_length: float
    track_window_width: float
    track_gauge: float
    min_track_points: float
    min_points: float
    max_distance_for_ransac: float
    grouping_distance: float
    mdl_weight: float
    min_track_intensity: float
    max_track_intensity: float


@dataclass(frozen=True)
class ExtractionConfig:
    """The ten GUI parameters plus the source's grid size and skip rate.

    Distances use the input coordinate units (metres in the original application).
    No coordinate conversion, intensity filtering, or subsampling is implicit.
    Other stage-specific constants will be ported with their owning algorithms.
    """

    grid_size: float = 1.0
    skip_rate: int = 1
    track_window_width: float = 0.42
    track_window_length: float = 1.0
    max_distance_for_ransac: float = 0.06
    track_gauge: float = 1.5
    min_track_points: int = 5
    min_points: int = 20
    mdl_weight: float = 0.4
    grouping_distance: float = 0.15
    min_track_intensity: int = 0
    max_track_intensity: int = 1500

    def to_dict(self) -> dict[str, float | int]:
        return asdict(self)

    def with_source_parameters(self, parameters: ExtractionParameters) -> ExtractionConfig:
        """updateExtractionParameters; preserve grid/skip and narrow four fields.

        This explicit immutable update replaces the native setter. Direct Python
        configuration construction is unchanged; use this boundary for source
        dialog/parameter values. Intensity thresholds remain inactive in tracking.
        """
        values = {name: float(value) for name, value in asdict(parameters).items()}
        for name in ("min_track_points", "min_points", "min_track_intensity", "max_track_intensity"):
            values[name] = source_int32(values[name])
        return replace(self, **values)

    def to_source_parameters(self) -> ExtractionParameters:
        """getExtractionParameters: promote the engine's integer fields to double."""
        return ExtractionParameters(
            **{name: float(getattr(self, name)) for name in ExtractionParameters.__dataclass_fields__}
        )
