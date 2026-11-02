"""Rebuild the explicit source inventory after reviewing source or port changes.

Run from the repository root. Unknown definitions fail rather than being assigned
an invented implementation status. The original source is read, never modified.
"""

from collections import Counter
from hashlib import sha256
import json
from pathlib import Path

from railway_mls.coverage import SOURCE_DIRECTORY, cpp_definitions, source_text_hash


def rule(disposition, python=(), note=""):
    return {"disposition": disposition, "python": list(python), "note": note}


ENGINE = {}


def mapped(
    names, target, note="Computational behavior is implemented; stage guides define parity limits."
):
    for name in names.split():
        ENGINE[name] = rule("ported", (target,), note)


mapped(
    "RailwayExtraction",
    "railway_mls.config:ExtractionConfig",
    "Source defaults plus explicit Python engine ownership; native rendering/lifetimes are excluded.",
)
mapped(
    "updateExtractionParameters getExtractionParameters",
    "railway_mls.config:ExtractionConfig",
    "Immutable configuration replaces the native parameter getter/setter interface.",
)
mapped(
    "updateMaxMin",
    "railway_mls.spatial:Bounds3D",
    "Explicit bounds replace the two overloads; the zero-argument source stub remains a stub.",
)
mapped("generateGrid GetNeighborIdx", "railway_mls.spatial:GridIndex")
mapped(
    "OnRailextractionSetinit",
    "railway_mls.orchestration:TrackingEngine.initialize",
    "Explicit-seed overload only; no-argument uninitialized sample is not enabled.",
)
mapped(
    "OnRailextractionTrackingRailway ReserverVectors", "railway_mls.orchestration:TrackingEngine"
)
mapped("Setinitiallocation", "railway_mls.initialization:initialize_tracks")
mapped("Trackingrailway", "railway_mls.tracking:SequentialTracker")
mapped("ExtractTrkPts", "railway_mls.heads:extract_track_points")
mapped("GenerateTrkSegAssignMemPts", "railway_mls.tracking:generate_tracking_segment")
mapped("DetermineObservation", "railway_mls.observations:determine_observation")
mapped("SegmentTrackPtsByMDL ComputeCloseness", "railway_mls.mdl:segment_track_points_mdl")
mapped("DefineTemplates RecognitionTurnoutType", "railway_mls.recognition:recognize_turnout")
mapped("AdjustTwoLines", "railway_mls.gauge:adjust_two_lines")
mapped("AdjustTwoLinesWithObservation", "railway_mls.gauge:adjust_two_lines_with_observation")
mapped("DetectSwitchBox", "railway_mls.switch_boxes:detect_switch_boxes")
mapped("DetectToePnts", "railway_mls.toes:detect_toe_points")
mapped(
    "LineExtractionByRansac",
    "railway_mls.fitting:fit_line_ransac",
    "Both overloads are ported: 500-trial fit_line_ransac and 100-trial fit_rail_head_line; do not interchange them.",
)
mapped("LineExtractionWithFixedAngle", "railway_mls.fitting:fit_fixed_angle")
mapped("ComputeGMMfeature ClusteringByGMM", "railway_mls.gmm:classify_window")
mapped("SegmentPoints PointGrowing", "railway_mls.grouping:group_points")
mapped(
    "GetMinMaxXY",
    "railway_mls.geometry:min_max_xy",
    "Standalone source-order bounds, empty sentinels and first-tie zero signs; fitting delegates to the same helper.",
)
mapped(
    "ReCount",
    "railway_mls.geometry:recount",
    "Literal int32 index cycling, including one-add negative branch; undefined integer division/overflow raise explicitly.",
)
mapped("ComputeArea", "railway_mls.geometry:polygon_area")
mapped("ComputeFootOfPerpendicular", "railway_mls.geometry:perpendicular_foot")
mapped("DistanceBetweenPoints2D", "railway_mls.geometry:distance_2d")
mapped("DistanceBetweenPoints3D", "railway_mls.geometry:distance_3d")
mapped("DistanceBetweenPointAndLine2D", "railway_mls.geometry:distance_to_line_2d")
mapped("DistanceBetweenPointAndLine3DHgt", "railway_mls.heads:_refine_candidates")
mapped("DistanceBetweenPointAndPlane", "railway_mls.planes:plane_distances")
mapped("GetDirectionMode", "railway_mls.geometry:direction_mode")
mapped("GetAngle", "railway_mls.geometry:source_angle")
mapped(
    "KalmanPredict KalmanUpdate",
    "railway_mls.kalman:kalman_update",
    "Prediction and update are separate public functions; bundled inverse semantics and tracker covariance behavior are retained.",
)
mapped("GetPntsInsidePolygon", "railway_mls.geometry:points_in_convex_polygon")
mapped("ExtractPlaneByRANSAC", "railway_mls.planes:fit_plane_ransac")
mapped("GetPlaneParameters3p", "railway_mls.planes:plane_from_three_points")
mapped("GetPlaneParametersEigen", "railway_mls.planes:plane_from_eigen")
mapped(
    "GetLnsFromVtx",
    "railway_mls.geometry:lines_from_vertices",
    "Cyclic endpoint-only BoundaryLine output, zero endpoint tags and absent native caches. Initialization/tracking use this same helper.",
)
mapped(
    "GetLayers",
    "railway_mls.orchestration:TrackingEngine",
    "Ordered live layer references are exposed as engine.layers.",
)
mapped(
    "ComputeGrade DetermineGradeSection DouglasPeuckerVertical ComputeGradeParameters",
    "railway_mls.grades:compute_grade",
    "Each native routine has its own public counterpart in grades; XYZ-only endpoint assignment and source curve algebra are retained.",
)
mapped("AddPointTofootOfPependicularOnRailVtx", "railway_mls.orchestration:tag_rail_vector")
for name, target, note in (
    (
        "ComputeGMMParaAndCluster",
        "railway_mls.gmm:classify_segment",
        "Both three-component fits precede both clustering calls; empty/partial results and replacement heads are retained. Python returns values instead of exposing aliased native output-reference buffers.",
    ),
    (
        "DeleteSegPntsCmpWithPreHgt",
        "railway_mls.heads:filter_segment_by_previous_heights",
        "Four ordered strict erasure loops mutate memberships only; NaN bounds erase nothing. This is not added to active head extraction.",
    ),
    (
        "ComputeInnerAngle",
        "railway_mls.geometry:inner_angle",
        "Normalized XY arms and clamped dot product retain source IEEE degeneracy and ignore Z.",
    ),
    (
        "DistanceBetweenPointAndLine3D",
        "railway_mls.geometry:distance_to_line_3d",
        "The stored unit vector must be supplied explicitly, including stale/nonunit values; no automatic normalization or invented cache is used.",
    ),
):
    ENGINE[name] = rule("unused_helper_ported", (target,), note)
for name in ("DetectLoopCross", "ValidateLoopCross"):
    ENGINE[name] = rule(
        "inactive_pending",
        note="Loop/cross workflow calls are commented out. Workflow reconstruction remains pending; do not enable implicitly.",
    )
ENGINE["ComputeIntersectionPoint3D"] = rule(
    "inactive_ported",
    ("railway_mls.geometry:intersection_point_3d",),
    "Standalone literal XY intersection with source A2==1 Y branch, Z=0 and IEEE division; inactive loop/cross scheduling remains unchanged.",
)
ENGINE["TemplateMatching"] = rule(
    "inactive_ported",
    ("railway_mls.recognition:template_matching",),
    "Standalone raw-count matcher exists; active tracking uses RecognitionTurnoutType instead.",
)
ENGINE["LineExtractionWithFixedAngleForLoopCross"] = rule(
    "inactive_ported",
    ("railway_mls.fitting:fit_loop_cross_line",),
    "Standalone fitter is implemented; loop/cross scheduling remains inactive and pending.",
)
for name in ("updateMaxMinFromReader", "readMaxMinFromReader", "readMaxMinFromTXTReader"):
    ENGINE[name] = rule(
        "io_pending",
        note="Legacy reader bounds/data adapters remain pending. Explicit GridIndex bounds are available without those adapters.",
    )
for name in (
    "PrintTurnoutType",
    "PrintVtx",
    "PrintPts",
    "PrintLines",
    "updateView",
    "~RailwayExtraction",
):
    ENGINE[name] = rule(
        "presentation_or_lifetime",
        note="Console/rendering or native lifetime infrastructure; numerical diagnostics/geometry are exposed by Python without recreating Windows/PCL/VTK UI.",
    )
ENGINE["setInitialLocation"] = rule(
    "source_stub",
    note="Empty source method; not an implementation gap in the active explicit-seed API.",
)


UI = {}
UI_PORTS = {
    "GenerateGrid": "railway_mls.spatial:GridIndex",
    "setInitalLocationToolStripMenuItem_Click": "railway_mls.orchestration:TrackingEngine.initialize",
    "railwayTrackingToolStripMenuItem_Click": "railway_mls.orchestration:TrackingEngine.run_bidirectional",
    "GetAngle": "railway_mls.geometry:source_angle",
    "ReceivedPoints": "railway_mls.pipeline:initial_location_from_picks",
    "setParametersToolStripMenuItem_Click": "railway_mls.config:ExtractionConfig",
    "computeGradeToolStripMenuItem_Click": "railway_mls.grades:compute_layer_grades",
    "exportGWADToolStripMenuItem_Click": "railway_mls.grades:build_gwad_rows",
}
for name, target in UI_PORTS.items():
    UI[name] = rule(
        "ported_computation",
        (target,),
        "Computational action is ported; dialogs, callbacks and editable form controls are presentation. GWAD text uses format_gwad; file-dialog I/O is excluded.",
    )
for name in (
    "openToolStripMenuItem_Click",
    "openPTSFileToolStripMenuItem_Click",
    "LoadPointCloud",
    "loadFile",
    "eCEFLASToUTMToolStripMenuItem_Click",
    "uTMPTSToECEFPTSToolStripMenuItem_Click",
    "dataConversionToolStripMenuItem_Click",
    "saveRailVectorToolStripMenuItem_Click",
    "saveRailVectorAtIndex",
    "saveRailVectorWithIndexAtIndex",
    "savePointsToolStripMenuItem_Click",
    "SaveVectorToolStripMenuItem_Click",
    "saveRailVectorWithIndexToolStripMenuItem_Click",
):
    UI[name] = rule(
        "io_or_conversion_pending",
        note="Legacy loading, coordinate conversion or export workflow. New diagnostic artifacts are not a substitute for this legacy schema/precision contract.",
    )
for name in (
    "exitToolStripMenuItem_Click",
    "showPointCloud",
    "toolStripCameraTop_Click",
    "toolStripCameraDown_Click",
    "toolStripCameraLeft_Click",
    "toolStripCameraRight_Click",
    "toolStripCameraFront_Click",
    "toolStripCameraBack_Click",
    "toolStripCameraIsometric_Click",
    "VTKMouseMove",
    "InitLocationPick",
    "selectInitialLocationToolStripMenuItem_Click",
    "listFiles",
    "updateTrackedLayers",
    "filesTreeView_NodeMouseDoubleClick",
    "setPointCloudColorToolStripMenuItem_Click",
    "exportOptionsToolStripMenuItem_Click",
    "setBackgroundColorToolStripMenuItem_Click",
    "toolIntensity_Click",
    "layersTreeView_EnabledChanged",
    "layersTreeView_AfterCheck",
    "renameToolStripMenuItem_Click",
    "layersTreeView_NodeMouseClick",
    "UpdateProgress",
):
    UI[name] = rule(
        "presentation",
        note="Windows Forms/VTK interaction, layer naming/display, application lifetime or progress; not recreated by the numerical pipeline.",
    )


root = Path(__file__).resolve().parents[1]
source = root / SOURCE_DIRECTORY
files = []
for path in sorted(source.iterdir()):
    if not path.is_file() or path.suffix not in (".cpp", ".h"):
        continue
    if path.stem in (
        "LASFileReader",
        "TXTFileReader",
        "DataConvertor",
        "UTM",
        "DataConverstionForm",
    ):
        disposition = "io_or_conversion_pending"
    elif path.stem in ("RailwayExtraction", "MainForm", "LIDARPT", "PointXYZDouble"):
        disposition = "audited_core_or_state"
    else:
        disposition = "presentation_or_infrastructure"
    files.append(
        {
            "path": path.name,
            "sha256": source_text_hash(path.read_text()),
            "disposition": disposition,
        }
    )
dependencies = []
for directory in ("Jama", "etc"):
    for path in sorted((source / directory).rglob("*")):
        if path.is_file() and path.suffix in (".cpp", ".h", ".inl"):
            dependencies.append(
                {
                    "path": path.relative_to(source).as_posix(),
                    "sha256": source_text_hash(path.read_text()),
                    "disposition": "dependency_only_reached_math_ported",
                }
            )
definition_files = []
for filename, owner, rules in (
    ("RailwayExtraction.cpp", "RailwayExtraction", ENGINE),
    ("MainForm.cpp", "PCLProject::MainForm", UI),
):
    entries = []
    for definition in cpp_definitions((source / filename).read_text(), owner):
        entry = dict(definition)
        if entry["name"] not in rules:
            raise ValueError(f"Review unclassified definition: {entry['id']}")
        entry.update(rules[entry["name"]])
        if entry["id"] == "RailwayExtraction::OnRailextractionSetinit#1":
            entry.update(
                rule(
                    "unsafe_inactive_sample",
                    note="No-argument sample uses an uninitialized point and is not the active form action. No fabricated seed is supplied.",
                )
            )
        if entry["id"] == "RailwayExtraction::updateMaxMin#3":
            entry.update(
                rule("source_stub", note="Zero-argument bounds method is empty in source.")
            )
        entries.append(entry)
    definition_files.append({"path": filename, "owner": owner, "definitions": entries})
baseline = sha256(
    "\n".join(f"{f['path']}:{f['sha256']}" for f in (*files, *dependencies)).encode()
).hexdigest()
audit = {
    "schema_version": 1,
    "baseline_id": baseline,
    "definition_scope": [
        "Every out-of-class definition in RailwayExtraction.cpp and MainForm.cpp",
        "All top-level project .cpp/.h files have file-level dispositions",
        "LIDARPT.h state/geometry is mapped by stage guides; header members are not individually counted",
        "Jama/etc are dependencies: only reached numerical behavior is ported, not whole libraries",
    ],
    "files": files,
    "bundled_dependencies": dependencies,
    "definition_files": definition_files,
    "complete_reconstruction": False,
}
destination = root / "src/railway_mls/source_coverage.json"
destination.write_text(json.dumps(audit, indent=2) + "\n")
print(
    f"Wrote {sum(len(f['definitions']) for f in definition_files)} explicit definition dispositions and {len(files)} file hashes."
)
print(f"Also hashed {len(dependencies)} bundled dependency files; only reached math is ported.")
print(dict(Counter(e["disposition"] for f in definition_files for e in f["definitions"])))
