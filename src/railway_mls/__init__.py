"""Source-faithful Python foundations for railway recognition from 3D MLS."""

from .config import ExtractionConfig
from .geometry import Line3D, direction_mode
from .fitting import LineFitResult, fit_fixed_angle, fit_line_ransac
from .gmm import classify_segment, classify_window, cluster_by_gmm, fit_height_gmm
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
from .points import Point3D, PointCloud, PointSelection
from .sampling import MsvcRand, RecordedRand
from .spatial import Bounds3D, GridIndex

__version__ = "0.1.0"

__all__ = [
    "Bounds3D",
    "ExtractionConfig",
    "GMMParameters",
    "GridIndex",
    "InitialLocation",
    "Line3D",
    "LineFitResult",
    "MDLResult",
    "MsvcRand",
    "Point3D",
    "PointCloud",
    "PointGrouping",
    "PointSelection",
    "RecordedRand",
    "TrackHeadResult",
    "TrackLayer",
    "TrackSegment",
    "TrackWindow",
    "classify_segment",
    "classify_window",
    "cluster_by_gmm",
    "direction_mode",
    "extract_track_points",
    "fit_height_gmm",
    "fit_fixed_angle",
    "fit_line_ransac",
    "group_points",
    "segment_track_points_mdl",
    "seed_windows",
]
