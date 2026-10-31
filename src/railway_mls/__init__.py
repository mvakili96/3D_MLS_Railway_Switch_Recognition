"""Source-faithful Python foundations for railway recognition from 3D MLS."""

from .config import ExtractionConfig
from .geometry import Line3D, direction_mode
from .gmm import classify_segment, classify_window, cluster_by_gmm, fit_height_gmm
from .heads import TrackHeadResult, extract_track_points
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
    "MsvcRand",
    "Point3D",
    "PointCloud",
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
    "seed_windows",
]
