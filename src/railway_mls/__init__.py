"""Source-faithful Python foundations for railway recognition from 3D MLS."""

from .config import ExtractionConfig
from .geometry import Line3D, direction_mode
from .fitting import LineFitResult, fit_fixed_angle, fit_line_ransac
from .gmm import classify_segment, classify_window, cluster_by_gmm, fit_height_gmm
from .gauge import GaugeAdjustment, adjust_two_lines, adjust_two_lines_with_observation
from .heads import TrackHeadResult, extract_track_points
from .grouping import PointGrouping, group_points
from .initialization import InitializationResult, initial_kalman_states, initialize_tracks
from .kalman import KalmanPrediction, KalmanState, KalmanUpdate, kalman_predict, kalman_update
from .mdl import MDLResult, segment_track_points_mdl
from .models import (
    BinaryString,
    GMMParameters,
    InitialLocation,
    TrackLayer,
    TrackSegment,
    TrackStack,
    TrackWindow,
    ToePoint,
    SwitchBox,
    GradeSection,
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
from .recognition import (
    StackRecognition,
    StackRecognitionBatch,
    TemplateCatalogue,
    TemplateRecognition,
    define_templates,
    recognize_stacks,
    recognize_turnout,
    stack_signature,
    template_matching,
)
from .tracking import (
    SequentialTracker,
    TrackingResult,
    TrackingStep,
    WindowAttempt,
    generate_tracking_segment,
    track_railway,
)
from .toes import (
    ToeBatchResult,
    ToeDetection,
    StackToeDetection,
    detect_stack_toes,
    detect_toe_points,
)
from .orchestration import (
    BidirectionalTracking,
    BranchTracking,
    LayerToeTag,
    TrackingEngine,
    TrackingInvocation,
    TrackingPass,
    VectorReversal,
    VectorTag,
    reverse_layer_vectors,
    reverse_seed,
    tag_rail_vector,
)
from .planes import (
    PlaneParameters,
    PlaneEigenFit,
    PlaneFitResult,
    plane_from_three_points,
    plane_from_eigen,
    plane_distances,
    fit_plane_ransac,
)
from .switch_boxes import (
    SwitchBoxCandidate,
    SwitchBoxCandidates,
    SwitchBoxDetection,
    SwitchBoxBatch,
    switch_box_candidates,
    detect_switch_boxes,
)
from .grades import (
    GradeAnalysis, GradeDivision, GradeParameters, GradeReportRow, GradeSplit,
    GWADExport, GWADRow, build_gwad_rows, compute_grade, compute_grade_parameters,
    compute_layer_grades, determine_grade_sections, douglas_peucker_vertical, format_gwad,
)
from .pipeline import (
    PipelineFinding, PipelineOptions, PipelineResult, PipelineStage,
    initial_location_from_picks, run_pipeline,
)
from .artifacts import pipeline_summary, write_pipeline_outputs
from .coverage import check_source_coverage, coverage_summary, source_coverage

__version__ = "0.1.0"

__all__ = [
    "PipelineFinding", "PipelineOptions", "PipelineResult", "PipelineStage",
    "initial_location_from_picks", "run_pipeline", "pipeline_summary", "write_pipeline_outputs",
    "check_source_coverage", "coverage_summary", "source_coverage",
    "GradeSection", "GradeAnalysis", "GradeDivision", "GradeParameters", "GradeReportRow",
    "GradeSplit", "GWADExport", "GWADRow", "build_gwad_rows", "compute_grade",
    "compute_grade_parameters", "compute_layer_grades", "determine_grade_sections",
    "douglas_peucker_vertical", "format_gwad",
    "BidirectionalTracking",
    "BranchTracking",
    "BinaryString",
    "Bounds3D",
    "ExtractionConfig",
    "GMMParameters",
    "GaugeAdjustment",
    "GridIndex",
    "InitialLocation",
    "InitializationResult",
    "KalmanPrediction",
    "KalmanState",
    "KalmanUpdate",
    "Line3D",
    "LineFitResult",
    "LayerToeTag",
    "MDLResult",
    "MsvcRand",
    "ObservationSelection",
    "Point3D",
    "PointCloud",
    "PointGrouping",
    "PointSelection",
    "PlaneParameters",
    "PlaneEigenFit",
    "PlaneFitResult",
    "RecordedRand",
    "SupportObservation",
    "SequentialTracker",
    "StackRecognition",
    "StackRecognitionBatch",
    "TemplateCatalogue",
    "TemplateRecognition",
    "ToePoint",
    "ToeBatchResult",
    "ToeDetection",
    "StackToeDetection",
    "SwitchBox",
    "SwitchBoxCandidate",
    "SwitchBoxCandidates",
    "SwitchBoxDetection",
    "SwitchBoxBatch",
    "TrackHeadResult",
    "TrackLayer",
    "TrackSegment",
    "TrackStack",
    "TrackWindow",
    "TrackingResult",
    "TrackingEngine",
    "TrackingInvocation",
    "TrackingPass",
    "TrackingStep",
    "WindowAttempt",
    "VectorReversal",
    "VectorTag",
    "adjust_two_lines",
    "adjust_two_lines_with_observation",
    "classify_segment",
    "classify_window",
    "cluster_by_gmm",
    "direction_mode",
    "define_templates",
    "detect_stack_toes",
    "detect_toe_points",
    "determine_observation",
    "extract_track_points",
    "fit_height_gmm",
    "fit_fixed_angle",
    "fit_line_ransac",
    "fit_plane_ransac",
    "group_points",
    "generate_tracking_segment",
    "initial_kalman_states",
    "initialize_tracks",
    "kalman_predict",
    "kalman_update",
    "observation_from_support",
    "plane_from_three_points",
    "plane_from_eigen",
    "plane_distances",
    "recognize_stacks",
    "recognize_turnout",
    "reverse_layer_vectors",
    "reverse_seed",
    "segment_track_points_mdl",
    "seed_windows",
    "track_railway",
    "stack_signature",
    "template_matching",
    "tag_rail_vector",
    "switch_box_candidates",
    "detect_switch_boxes",
]
