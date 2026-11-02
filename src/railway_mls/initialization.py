"""Full Setinitiallocation, with explicit results for undefined native fit paths."""

from __future__ import annotations

from dataclasses import dataclass

from .config import ExtractionConfig
from .fitting import RailHeadLineFit, descending_height, fit_rail_head_line
from .gauge import GaugeAdjustment, adjust_two_lines
from .geometry import direction_mode, lines_from_vertices
from .gmm import WindowClassification, classify_segment
from .kalman import KalmanState
from .models import InitialLocation, TrackLayer, TrackSegment, seed_windows
from .points import Point3D, PointSelection
from .sampling import MsvcRand, RandSource
from .spatial import GridIndex


@dataclass(frozen=True)
class InitializationResult:
    segment: TrackSegment
    layer: TrackLayer
    classifications: tuple[WindowClassification, ...]
    heads: tuple[PointSelection, PointSelection]
    initial_heights: tuple[float, float] | None
    fits: tuple[RailHeadLineFit, ...]
    adjustment: GaugeAdjustment | None
    location: InitialLocation | None
    status: str

    @property
    def success(self) -> bool:
        return self.status == "ok"

    @property
    def source_return(self) -> bool | None:
        """Native bool, separate from finite success; None denotes undefined fit use."""
        if self.status == "undefined_source_initial_line":
            return None
        return self.status != "no_head_points"


def initial_kalman_states(segment: TrackSegment) -> tuple[KalmanState, KalmanState]:
    """Trackingrailway's first-iteration XYZ and identity covariance for each rail.

    The tracker uses adjusted window centers, not the seed input or line ends.
    Later covariance assignment remains the sequential tracker's responsibility.
    """
    return KalmanState(segment.track1.center), KalmanState(segment.track2.center)


def initialize_tracks(
    seed: InitialLocation,
    grid: GridIndex,
    *,
    config: ExtractionConfig | None = None,
    rng: RandSource | None = None,
    layer: TrackLayer | None = None,
) -> InitializationResult:
    """Setinitiallocation, including shared ordered grid lookup and layer appends.

    The seed's optional second_point is overwritten on success, as in the source.
    Window memberships are assigned once, BEFORE fitted windows replace vertices.
    Initialization does not call ExtractTrkPts, MDL or the weighted gauge routine.
    """
    settings = config if config is not None else ExtractionConfig()
    if grid.grid_size != settings.grid_size:
        raise ValueError("initialization configuration must match the grid size")
    target_layer = layer if layer is not None else TrackLayer()
    random = rng if rng is not None else MsvcRand()
    segment = seed_windows(seed, settings)
    empty = grid.cloud.select([])
    heads = (empty, empty)
    classifications = ()
    fits = ()
    heights = None

    def result(
        status: str,
        adjustment: GaugeAdjustment | None = None,
        location: InitialLocation | None = None,
    ) -> InitializationResult:
        return InitializationResult(
            segment,
            target_layer,
            classifications,
            heads,
            heights,
            fits,
            adjustment,
            location,
            status,
        )

    grid.assign_segment_members(segment, neighborhood_size=5)
    classifications = classify_segment(segment)
    heads = tuple(classification.heads for classification in classifications)
    if any(not len(points) for points in heads):
        # Native Setinitiallocation returns false, including defined partial
        # clusters returned early by ClusteringByGMM.
        return result("no_head_points")
    heads = tuple(descending_height(points) for points in heads)
    heights = tuple(float(points.xyz[int(len(points) * 0.15), 2]) for points in heads)
    # Hardcoded source threshold; settings.max_distance_for_ransac belongs to
    # later tracking. Both calls occur even if the first native fit returns false.
    fits = tuple(fit_rail_head_line(points, threshold=0.04, rng=random) for points in heads)
    if any(not fit.success for fit in fits):
        return result("undefined_source_initial_line")
    lines = tuple(fit.line for fit in fits)
    for line in lines:
        line.order_endpoints(seed.mode)
    adjustment = adjust_two_lines(*lines, line_mode=seed.mode, config=settings)
    # Preserve even defined nonfinite output: the source still appends this segment
    # and returns true. Report its numerical status without inventing replacements.
    for points, line in (
        (target_layer.rail1_vector, lines[0]),
        (target_layer.rail2_vector, lines[1]),
    ):
        points.append(line.center)
    target_layer.center_vector.append((lines[0].center + lines[1].center) / 2)
    modes = []
    for window, line in zip((segment.track1, segment.track2), lines):
        window.orientation, window.center, window.member_line = line.angle, line.center, line
        mode = direction_mode(line.end, line.begin)
        modes.append(mode)
        window.generate_vertices(settings.track_window_length, settings.track_window_width, mode)
        window.vertices = tuple(
            Point3D(point.x, point.y, line.begin.z, point.point_type) for point in window.vertices
        )
    segment.update_gauge()
    segment.distance_from_start = 0.0
    for window in (segment.track1, segment.track2):
        target_layer.updated_boundaries.extend(lines_from_vertices(window.vertices))
    target_layer.track_segments.append(segment)
    location = InitialLocation(
        segment.track1.center, lines[0].angle, modes[1], segment.track2.center
    )
    return result(
        "ok" if adjustment.success else "nonfinite_initial_geometry", adjustment, location
    )
