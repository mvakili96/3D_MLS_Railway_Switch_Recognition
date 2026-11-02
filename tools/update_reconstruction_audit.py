"""Freeze the reviewed computational scope and lexical call-site evidence.

Run after update_source_coverage.py, then rerun it to refresh the review reference.
These decisions apply only to the explicitly frozen baseline below. This tool
does not perform a semantic review, execute tests or certify native parity.
"""

from collections import Counter
import json
from pathlib import Path

from railway_mls.coverage import SOURCE_DIRECTORY, cpp_function_calls, source_coverage


BASELINE = "b04978b12f533a86f2fd24527e5dc4b4779a082461a7050142cb8e372b6c09f0"
ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / SOURCE_DIRECTORY
coverage = source_coverage()
if coverage["baseline_id"] != BASELINE:
    raise ValueError("A changed source baseline requires a new explicit reconstruction review")


def files(stems, outcome, note):
    return {"files": [item["path"] for item in coverage["files"]
                      if Path(item["path"]).stem in stems.split()], "outcome": outcome, "note": note}


file_reviews = [
    files("RailwayExtraction", "computational_methods_mapped",
          "All 79 definitions reviewed: defaults, bounds/grid, initialization, tracking/retries, postpasses, branches, fitting, grades and inactive helpers. Rendering/progress/print-only methods are excluded; uninitialized source reads retain diagnostics."),
    files("LIDARPT", "inline_state_and_geometry_mapped",
          "All 77 uncommented inline definitions and 19 named types mapped. Constructors, tag assignment, cached geometry and snapshot semantics are explicit. Two block-commented rotations remain excluded."),
    files("MainForm", "computational_ui_actions_mapped",
          "All 45 out-of-class definitions reviewed. File/cache lifecycle, picked seed, reverse-first two-pass tracking, grades, exports and coordinate menu bindings are mapped. Generated controls, dialogs, layer names, progress and rendering are presentation."),
    files("PointXYZDouble", "scalar_point_boundary_mapped",
          "Default/XYZ/pointer-copy, homogeneous coordinate, metadata, strict tolerance equality and exact null are now explicit. Core cloud row identity remains separate; native PCL registration/alignment are storage infrastructure."),
    files("TXTFileReader LASFileReader", "readers_exports_and_unused_helpers_mapped",
          "All 18 methods mapped. Original order, alternate LAS records, text reader bounds, cache identity and all vector formats retained. Optional LAS decoding is a declared library boundary. DecreaseDensity/Transform are standalone and remain absent from loading/tracking."),
    files("DataConvertor UTM DataConverstionForm", "conversion_paths_mapped",
          "All 27 definitions mapped, including UTM free functions, unit conventions, accumulating buffers, callback variants, exports, batch actions and the source inverse-menu northern binding. File dialogs/progress are presentation."),
    files("ExtractionParameterForm InitialLocationForm", "numeric_boundary_and_presentation_separated",
          "All 11 cpp definitions classified. Ten numeric doubles feed four engine int casts. Active seed defaults truncate/wrap angle and normalize mode; inactive XY overload remains presentation. Numeric Python inputs replace locale-sensitive text round trips. No dialog behavior is inserted into direct engine calls."),
    files("HashMap SharedData SharedResources", "explicit_session_ownership",
          "HashMap get inserts a null exact-string key and put replaces its engine; LegacyFileSession preserves those effects. SharedData's singleton/cloud/file map and SharedResources filename list are application ownership. The display center of mass is not an algorithm input or implicit cloud transform."),
    files("VTKWindowController VTKWindowControl VMouseInteractorStyle typedef1", "presentation_excluded",
          "Reviewed controller, duplicate mouse styles, event forwarding, cameras and axes. Display-world coordinates are translated back by adding the display centroid before the six-coordinate ReceivedPoints callback. Python picks already use input/world coordinates. Screen projection, double-click state, sentinel bugs and unused event-position Z reads are UI behavior; no VTK cloud transform enters tracking."),
    files("Viewer", "viewer_demo_excluded",
          "Unused sample PCL viewer generates an elliptical cylinder for display and sets RGB/camera properties. It is independent of railway tracking and Python railway test fixtures."),
    files("Main", "application_entry_and_fps_excluded",
          "Windows Forms startup and FPS display callback only. Python CLI replaces application startup; render-time reciprocal is presentation."),
    files("ExportOptionsForm", "generated_dialog_excluded",
          "Generated path/format dialog and resource lifetime. Numeric rail/GWAD formats are implemented by legacy_io/grades; the Windows selection UI is excluded."),
]


def workflow(id, definitions, python, tests, note):
    return {"id": id, "definitions": definitions.split(), "python": python.split(),
            "tests": ["tests/" + path for path in tests.split()], "note": note}


workflows = [
    workflow("configuration_and_state",
             "RailwayExtraction::RailwayExtraction#1 RailwayExtraction::updateExtractionParameters#1 RailwayExtraction::getExtractionParameters#1 PointXYZDouble::isNull#1",
             "railway_mls.config:ExtractionConfig railway_mls.points:PointXYZDouble",
             "test_foundation.py test_header.py test_reconstruction.py",
             "Default configuration, live field types, explicit immutable source-parameter updates, point/tag identity, constructor/cache state and undefined values reviewed. Four source double-to-int assignments are now compared directly. Negative count thresholds retain native unsigned retry comparisons."),
    workflow("loading_and_file_lifecycle", "PCLProject::MainForm::loadFile#1 HashMap::get#1 TXTFileReader::ReadData#1 LASFileReader::ReadData#1",
             "railway_mls.legacy_io:LegacyFileSession railway_mls.legacy_io:load_point_cloud",
             "test_legacy_io.py", "PTS/CSV tokenization, LAS sampling/bounds/callbacks, case-sensitive cache keys, null misses, direct fresh PTS loading and persistent engine state reviewed. Native reader compares decoded LAS shims; optional generated LAS/LAZ tests check the codec adapter separately."),
    workflow("bounds_grid_and_membership", "RailwayExtraction::generateGrid#1 RailwayExtraction::GetNeighborIdx#1 TrackWindow::SetTrackWindowMemPnts#1",
             "railway_mls.spatial:GridIndex railway_mls.models:TrackWindow.assign_members",
             "test_foundation.py test_cpp_parity.py", "Source XY indexing, cell/point traversal order, explicit text/header bounds and membership polygon behavior reviewed. Validated finite Python inputs are an explicit boundary."),
    workflow("seed_and_initialization", "PCLProject::MainForm::ReceivedPoints#1 RailwayExtraction::OnRailextractionSetinit#2 RailwayExtraction::Setinitiallocation#1 PCLProject::InitialLocationForm::updateInputs#2",
             "railway_mls.pipeline:initial_location_from_picks railway_mls.orchestration:initial_location_dialog_defaults railway_mls.initialization:initialize_tracks",
             "test_kalman_initialization.py test_pipeline.py test_reconstruction.py", "World-coordinate picks, float wrapper narrowing, mode/angle directions, reset/append layers, failed-init side effects, both GMMs/fits, initial heights and saved seed reviewed. Dialog normalization is available explicitly and not applied to direct seeds."),
    workflow("gmm_and_rail_heads", "RailwayExtraction::ComputeGMMfeature#1 RailwayExtraction::ClusteringByGMM#1 RailwayExtraction::ExtractTrkPts#1",
             "railway_mls.gmm:fit_height_gmm railway_mls.heads:extract_track_points",
             "test_gmm_heads.py test_gmm_cpp_parity.py", "Custom three-component EM, initialized RNG draws, early means-close stop, height filtering/refinement, head order, shared labels and retry-retained head buffers reviewed."),
    workflow("fitting_grouping_and_mdl", "RailwayExtraction::LineExtractionByRansac#1 RailwayExtraction::LineExtractionByRansac#2 RailwayExtraction::SegmentTrackPtsByMDL#1 RailwayExtraction::PointGrowing#1",
             "railway_mls.fitting:fit_line_ransac railway_mls.fitting:fit_rail_head_line railway_mls.mdl:segment_track_points_mdl",
             "test_fitting_mdl.py test_fitting_mdl_cpp.py", "100/500-trial overloads, fixed-angle fits, growing components, pointer identity, candidate flags, strict ties, RNG consumption and undefined intermediate policies reviewed. MDL with missing candidates remains an explicit limitation."),
    workflow("observations_gauge_and_filter", "RailwayExtraction::DetermineObservation#1 RailwayExtraction::AdjustTwoLines#1 RailwayExtraction::AdjustTwoLinesWithObservation#1 RailwayExtraction::KalmanPredict#1 RailwayExtraction::KalmanUpdate#1",
             "railway_mls.observations:determine_observation railway_mls.gauge:adjust_two_lines_with_observation railway_mls.kalman:kalman_update",
             "test_observation_gauge.py test_kalman_initialization.py", "Directional sorting/ties and side memberships, gauge formulas/cache update order, support mean/height, source LU and covariance behavior reviewed. NumPy inverse is not substituted for bundled matrix semantics."),
    workflow("tracking_retries_and_stops", "RailwayExtraction::Trackingrailway#1 RailwayExtraction::GenerateTrkSegAssignMemPts#1 TrackSegment::DuplicatePolyPair#1",
             "railway_mls.tracking:SequentialTracker",
             "test_tracking.py test_pipeline.py", "Raw integration and original loop comparisons cover growing windows, strict count gates, previous/current snapshots, stack append/flush order, vectors written before stopping, gauge/trajectory stops and terminal postpasses. Print/progress/render tail is excluded. Diagnostic limits defer later stages."),
    workflow("recognition_and_toes", "RailwayExtraction::DefineTemplates#1 RailwayExtraction::RecognitionTurnoutType#1 RailwayExtraction::DetectToePnts#1",
             "railway_mls.recognition:recognize_stacks railway_mls.toes:detect_toe_points",
             "test_recognition.py test_toes.py", "20 literal templates, duplicate-pattern ambiguity, stack-length/score gates, 12 toe families, unsupported cases and stack/reference initialization reviewed. Tracker-created missing reference lines remain undefined (LIM-001); initialized component fixtures do not cure that path."),
    workflow("planes_and_switch_boxes", "RailwayExtraction::ExtractPlaneByRANSAC#1 RailwayExtraction::DetectSwitchBox#1",
             "railway_mls.planes:fit_plane_ransac railway_mls.switch_boxes:detect_switch_boxes",
             "test_switch_boxes.py", "Candidate region order, point grouping, original JAMA eigen solver, plane constraints, switch box geometry and class-5 labels reviewed. Original detection comparisons execute initialized fits; failed native fits remain diagnosed."),
    workflow("branch_and_bidirectional_assembly", "RailwayExtraction::OnRailextractionTrackingRailway#1 RailwayExtraction::ReserverVectors#1 PCLProject::MainForm::railwayTrackingToolStripMenuItem_Click#1",
             "railway_mls.orchestration:TrackingEngine.run_pass railway_mls.orchestration:TrackingEngine.run_bidirectional",
             "test_orchestration.py test_pipeline.py", "Toe-copy/used state, new-layer-before-type-filter, branch seed Y-for-Z quirk, global stack/toe/box state, reverse-first GUI schedule, XYZ-only reversals and center tagging reviewed. Original wrapper comparisons use controlled fit/track callbacks; raw synthetic integration is separate evidence, not whole-native equivalence."),
    workflow("grades_gwad_and_vector_exports", "RailwayExtraction::ComputeGrade#1 PCLProject::MainForm::exportGWADToolStripMenuItem_Click#1 TXTFileReader::saveRailVectorWithIndex#1",
             "railway_mls.grades:compute_layer_grades railway_mls.grades:build_gwad_rows railway_mls.legacy_io:format_rail_vector",
             "test_grades.py test_legacy_io.py", "Grade is a separate action, GWAD explicitly computes grades, source curve algebra/XYZ tag behavior, missing native outputs, export row ordering/markers and three-decimal formats reviewed. UI dialog/name selection is external."),
    workflow("coordinate_conversion_actions", "DataConvertor::ConvertECEFToUTM#1 DataConvertor::ConvertUTMToECEF#1 UTM::LatLonToUTMXY#1 PCLProject::MainForm::uTMPTSToECEFPTSToolStripMenuItem_Click#1 PCLProject::DataConverstionForm::utmConvertButton_Click#1",
             "railway_mls.conversion:CoordinateConverter railway_mls.conversion:convert_legacy_files",
             "test_coordinates.py", "Scalar projections, source float aliases/constants/units, repeated buffers, callbacks, metadata, menus and batch selection/export reviewed. Explicit conversion stays outside core tracking and preserves the source menu's northern inverse binding."),
    workflow("standalone_header_utilities_and_loop_cross", "RailwayExtraction::ComputeGMMParaAndCluster#1 RailwayExtraction::DeleteSegPntsCmpWithPreHgt#1 RailwayExtraction::DetectLoopCross#1 RailwayExtraction::ValidateLoopCross#1 Line3D::GetUnitVec#1",
             "railway_mls.gmm:classify_segment railway_mls.loop_cross:detect_loop_cross railway_mls.loop_cross:validate_loop_cross",
             "test_header.py test_source_utilities.py test_loop_cross.py", "Uncalled wrappers, scalar math, mutable/cache-sensitive header operations, loop/cross projection and integer-phase validation reviewed. Standalone ports are available; inactive source scheduling and commented rotations remain inactive."),
    workflow("standalone_pcl_helpers", "LASFileReader::DecreaseDensity#1 LASFileReader::Transform#1",
             "railway_mls.pcl_helpers:decrease_density railway_mls.pcl_helpers:center_point_cloud",
             "test_pcl_helpers.py", "Literal 0.1f voxel grid, double centroid plus float32 affine output, metadata, density/nonfinite/empty cases, integer-overflow guards, float floor overload and equal-key replay reviewed. Selected PCL extracts and bundled Eigen comparisons need local headers only for tests; helpers remain uncalled by readers/tracking."),
]


def calls(caller, names="", inactive=""):
    return {"caller": caller, "must_call": names.split(), "must_not_call": inactive.split()}


call_sites = [
    calls("RailwayExtraction::RailwayExtraction#1", "DefineTemplates"),
    calls("PCLProject::MainForm::loadFile#1", "get put ReadData generateGrid",
          "DecreaseDensity Transform ConvertECEFToUTM ConvertUTMToECEF"),
    calls("PCLProject::MainForm::openPTSFileToolStripMenuItem_Click#1", "ReadData generateGrid",
          "DecreaseDensity Transform"),
    calls("PCLProject::MainForm::ReceivedPoints#1", "GetAngle OnRailextractionSetinit updateInputs getLocation"),
    calls("PCLProject::MainForm::setParametersToolStripMenuItem_Click#1", "getExtractionParameters updateValues getValues updateExtractionParameters"),
    calls("RailwayExtraction::OnRailextractionSetinit#2", "Setinitiallocation"),
    calls("RailwayExtraction::Setinitiallocation#1", "ComputeGMMfeature ClusteringByGMM LineExtractionByRansac AdjustTwoLines GetLnsFromVtx",
          "DetectLoopCross DetectLoopCrossDemo"),
    calls("RailwayExtraction::Trackingrailway#1", "KalmanPredict GenerateTrkSegAssignMemPts ExtractTrkPts SegmentTrackPtsByMDL DetermineObservation LineExtractionByRansac KalmanUpdate AdjustTwoLinesWithObservation DuplicatePolyPair DefineTemplates RecognitionTurnoutType DetectToePnts DetectSwitchBox",
          "DetectLoopCross DetectLoopCrossDemo ValidateLoopCross ValidateTurnoutType TemplateMatching ComputeGrade DeleteSegPntsCmpWithPreHgt"),
    calls("RailwayExtraction::ExtractTrkPts#1", "ComputeGMMfeature ClusteringByGMM LineExtractionByRansac DistanceBetweenPointAndLine3DHgt",
          "DeleteSegPntsCmpWithPreHgt"),
    calls("RailwayExtraction::SegmentTrackPtsByMDL#1", "LineExtractionWithFixedAngle SegmentPoints ComputeCloseness"),
    calls("RailwayExtraction::DetermineObservation#1", "DistanceBetweenPointAndLine2D AdjustTwoLines"),
    calls("RailwayExtraction::OnRailextractionTrackingRailway#1", "Trackingrailway Setinitiallocation AddPointTofootOfPependicularOnRailVtx"),
    calls("PCLProject::MainForm::railwayTrackingToolStripMenuItem_Click#1", "OnRailextractionSetinit OnRailextractionTrackingRailway ReserverVectors Setinitiallocation",
          "ComputeGrade DetectLoopCross"),
    calls("RailwayExtraction::DetectToePnts#1", "SegmentPoints LineExtractionByRansac ComputeFootOfPerpendicular"),
    calls("RailwayExtraction::DetectSwitchBox#1", "GetBufferVtxOfLine ExtractPlaneByRANSAC SegmentPoints GenerateRgnsUsingCenterPt"),
    calls("RailwayExtraction::ExtractPlaneByRANSAC#1", "GetPlaneParameters3p GetPlaneParametersEigen"),
    calls("RailwayExtraction::ComputeGMMParaAndCluster#1", "ComputeGMMfeature ClusteringByGMM"),
    calls("RailwayExtraction::DetectLoopCross#1", "LineExtractionWithFixedAngleForLoopCross ComputeIntersectionPoint3D"),
    calls("PCLProject::MainForm::computeGradeToolStripMenuItem_Click#1", "GetLayers ComputeGrade"),
    calls("PCLProject::MainForm::exportGWADToolStripMenuItem_Click#1", "ComputeGrade DistanceBetweenPoints3D IsEqual"),
    calls("RailwayExtraction::ComputeGrade#1", "DetermineGradeSection ComputeGradeParameters"),
    calls("PCLProject::MainForm::saveRailVectorAtIndex#1", "saveRailVector"),
    calls("PCLProject::MainForm::saveRailVectorWithIndexAtIndex#1", "saveRailVectorWithIndex"),
    calls("DataConvertor::ConvertECEFToUTM#1", "Ecef_to_geodetic LatLonToUTMXY"),
    calls("DataConvertor::ConvertUTMToECEF#1", "UTMXYToLatLon Geodetic_to_ecef"),
    calls("PCLProject::MainForm::eCEFLASToUTMToolStripMenuItem_Click#1", "ConvertECEFToUTM SaveConvertedPoints"),
    calls("PCLProject::MainForm::uTMPTSToECEFPTSToolStripMenuItem_Click#1", "ConvertUTMToECEF SaveConvertedPoints"),
    calls("PCLProject::DataConverstionForm::utmConvertButton_Click#1", "ConvertECEFToUTM SaveConvertedPoints"),
    calls("PCLProject::DataConverstionForm::ecefConvertButton_Click#1", "ConvertUTMToECEF SaveConvertedPoints"),
]

names = {item["name"] for file in coverage["definition_files"] for item in file["definitions"]}
call_inventory = []
for file in coverage["definition_files"]:
    text = (SOURCE / file["path"]).read_text()
    for definition in file["definitions"]:
        tokens = cpp_function_calls(text, definition)
        call_inventory.append({"definition": definition["id"],
                               "project_named_call_tokens": dict(Counter(name for name in tokens if name in names))})

review = {
    "schema_version": 1,
    "baseline_id": BASELINE,
    "review_status": "complete",
    "scope": "Computational reconstruction of the frozen unpacked project, including standalone uncommented algorithm helpers and numerical input/output boundaries; presentation and whole dependency libraries are excluded.",
    "evidence_limits": [
        "Review and inventory consistency are not numerical parity proofs or a percentage of algorithm correctness.",
        "Lexical tokens exclude comments/strings but do not resolve overloads, templates, macros, pointer calls or runtime reachability; matching external method names can collide.",
        "Original C++ extracts use documented shims, instrumentation and supported initialized paths. Scheduling comparisons use controlled callbacks; the original Windows application has not been built or run.",
        "Synthetic and generated file tests establish implementation behavior, not real MLS accuracy or published performance.",
        "All limitation IDs remain open. Source undefined behavior, sorting/RNG/platform boundaries and diagnostic policies are retained for the separate post-reconstruction review.",
    ],
    "findings": [
        {"id": "AUD-001", "resolution": "implemented", "note": "Explicit source parameter adapter now preserves four double-to-int assignments and ten-field getter mapping. Tracking retains unsigned count comparisons for negative thresholds; original-loop tests compare retries, stops, side effects and random draws."},
        {"id": "AUD-002", "resolution": "implemented", "note": "Scalar PointXYZDouble default/XYZ/pointer-copy, equality, exact null and homogeneous coordinate are explicit; row identity is unchanged."},
        {"id": "AUD-003", "resolution": "implemented", "note": "Active seed dialog normalization is available as an explicit adapter; direct numeric seeds bypass presentation."},
        {"id": "AUD-004", "resolution": "documented_and_checked", "note": "19 previously file-only support definitions are now inventoried, including HashMap and both numerical forms. All 41 source files have reviewed scope notes."},
        {"id": "AUD-005", "resolution": "documented_and_checked", "note": "15 workflows, active/inactive call-site witnesses and lexical token inventory record the call-path review without claiming whole-binary parity."},
    ],
    "file_reviews": file_reviews,
    "workflows": workflows,
    "call_sites": call_sites,
    "call_inventory": call_inventory,
    "open_limitations": [f"LIM-{number:03d}" for number in range(1, 40)],
    "next_work": ["Review every limitation with the user before choosing corrections.",
                  "Agree real MLS inputs, coordinate units/CRS, sampling and experiment metrics.",
                  "Validate real-data outcomes and, if available, the matched original Windows executable."],
}
destination = ROOT / "src/railway_mls/reconstruction_audit.json"
destination.write_text(json.dumps(review, indent=2) + "\n")
print(f"Wrote {len(workflows)} workflow reviews, {len(call_sites)} call-site witnesses and {len(call_inventory)} lexical function records.")
