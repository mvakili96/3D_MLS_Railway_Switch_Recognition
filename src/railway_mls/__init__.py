"""Source-faithful Python foundations for railway recognition from 3D MLS."""

from .config import ExtractionConfig
from .geometry import Line3D, direction_mode
from .fitting import LineFitResult, fit_fixed_angle, fit_line_ransac
from .gmm import classify_segment, classify_window, cluster_by_gmm, fit_height_gmm
from .gauge import GaugeAdjustment, adjust_two_lines, adjust_two_lines_with_observation
from .heads import TrackHeadResult, extract_track_points
from .grouping import PointGrouping, group_points
from .mdl import MDLResult, segment_track_points_mdl
from .models import (
    GMMParameters,
    InitialLocation,
    TrackLayer,
    TrackSegment,
    TrackWindow,
    seed_windows,
)
from .observations import (
    ObservationSelection,
    SupportObservation,
    determine_observation,
    observation_from_support,
)
from .points import Point3D, PointCloud, PointSelection
from .sampling import MsvcRand, RecordedRand
from .spatial import Bounds3D, GridIndex

__version__ = "0.1.0"

__all__ = [
    "Bounds3D",
    "ExtractionConfig",
    "GMMParameters",
    "GaugeAdjustment",
    "GridIndex",
    "InitialLocation",
    "Line3D",
    "LineFitResult",
    "MDLResult",
    "MsvcRand",
    "ObservationSelection",
    "Point3D",
    "PointCloud",
    "PointGrouping",
    "PointSelection",
    "RecordedRand",
    "SupportObservation",
    "TrackHeadResult",
    "TrackLayer",
    "TrackSegment",
    "TrackWindow",
    "adjust_two_lines",
    "adjust_two_lines_with_observation",
    "classify_segment",
    "classify_window",
    "cluster_by_gmm",
    "direction_mode",
    "determine_observation",
    "extract_track_points",
    "fit_height_gmm",
    "fit_fixed_angle",
    "fit_line_ransac",
    "group_points",
    "observation_from_support",
    "segment_track_points_mdl",
    "seed_windows",
]
