"""Defaults from RailwayExtraction.cpp:24–37, without algorithm corrections."""

from dataclasses import asdict, dataclass


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
