"""Rebuild the explicit source inventory after reviewing source or port changes.

Run from the repository root. Unknown definitions fail rather than being assigned
an invented implementation status. The original source is read, never modified.
"""

from collections import Counter
from hashlib import sha256
import json
from pathlib import Path

from railway_mls.coverage import SOURCE_DIRECTORY, cpp_definitions, cpp_free_definitions, source_text_hash
from header_coverage import header_entries


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
for name, target in (("DetectLoopCross", "detect_loop_cross"), ("ValidateLoopCross", "validate_loop_cross")):
    ENGINE[name] = rule(
        "inactive_ported", ("railway_mls.loop_cross:" + target,),
        "Standalone literal source component, with explicit undefined-output/int32 guards. Original initialization/tracking/validation call sites stay commented out; no automatic scheduling is enabled.",
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
    "Standalone fitter and detection/validation are implemented; automatic loop/cross scheduling remains inactive.",
)
for name in ("updateMaxMinFromReader", "readMaxMinFromReader", "readMaxMinFromTXTReader"):
    ENGINE[name] = rule(
        "io_ported", ("railway_mls.legacy_io:ReaderBounds", "railway_mls.legacy_io:LegacyPointCloud.grid_bounds"),
        "Literal reader XYZ bounds are retained; Python grid adaptation preserves XY and explicitly encloses Z, which native grid lookup ignores. Text reader Z remains zero by construction.",
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
for name, target in (
    ("openToolStripMenuItem_Click", "LegacyFileSession.load_file"),
    ("openPTSFileToolStripMenuItem_Click", "LegacyFileSession.open_pts"),
    ("loadFile", "LegacyFileSession.load_file"),
    ("saveRailVectorToolStripMenuItem_Click", "write_layer_vectors"),
    ("saveRailVectorAtIndex", "write_layer_vectors"),
    ("saveRailVectorWithIndexAtIndex", "write_layer_vectors"),
    ("SaveVectorToolStripMenuItem_Click", "write_layer_vectors"),
    ("saveRailVectorWithIndexToolStripMenuItem_Click", "write_layer_vectors"),
):
    UI[name] = rule("io_ported", ("railway_mls.legacy_io:" + target,),
                    "Reader/cache/grid or legacy export action ported with explicit path/layer inputs; dialogs/display and selected UI controls are excluded. Indexed undefined reads and Python file/input policies remain explicit.")
UI["LoadPointCloud"] = rule("ported_computation", ("railway_mls.spatial:GridIndex",),
                            "Source body only invokes GenerateGrid; it is not an additional file reader.")
UI["savePointsToolStripMenuItem_Click"] = rule("source_stub", note="Source menu handler is empty; no invented point-file exporter.")
for name in ("eCEFLASToUTMToolStripMenuItem_Click", "uTMPTSToECEFPTSToolStripMenuItem_Click"):
    UI[name] = rule("conversion_ported", ("railway_mls.conversion:source_menu_conversion",),
                    "Explicit paths replace dialogs; source menu actions retain zone 32 and inverse hemisphere false, reader selection, no-callback conversion and rounded PTS output. Managed cancellation/reentry is excluded.")
UI["dataConversionToolStripMenuItem_Click"] = rule("presentation", note="Only launches the conversion dialog. Its numerical actions are mapped in DataConverstionForm.cpp.")

CONVERTER = {
    "DataConvertor": rule("conversion_ported", ("railway_mls.conversion:CoordinateConverter", "railway_mls.coordinates:ConverterEllipsoid"), "Source constants, shared input ownership and initially empty persistent output buffer."),
    "~DataConvertor": rule("native_lifetime", note="Empty source destructor; Python owns converted records."),
    "ConvertECEFToUTM": rule("conversion_ported", ("railway_mls.conversion:CoordinateConverter.convert_ecef_to_utm",), "Both overloads append input and reprocess the entire buffer; native phase order, PI units and periodic-only callback cadence are preserved."),
    "ConvertUTMToECEF": rule("conversion_ported", ("railway_mls.conversion:CoordinateConverter.convert_utm_to_ecef",), "Both overloads preserve explicit hemisphere, radian intermediate values, repeated buffer conversion and periodic-only callbacks."),
    "Ecef_to_geodetic": rule("conversion_ported", ("railway_mls.coordinates:ecef_to_geodetic",), "Literal two-branch inverse and constants; origin/domain IEEE NaN is retained."),
    "Geodetic_to_ecef": rule("conversion_ported", ("railway_mls.coordinates:geodetic_to_ecef",), "Source radian inputs, constants and scalar expression order."),
    "SaveConvertedPoints": rule("io_ported", ("railway_mls.conversion:format_converted_points", "railway_mls.conversion:write_converted_points"), "All three overloads: count header, fixed three decimals, intensity, delimiter and periodic-only progress. Python requires explicit overwrite and surfaces nonfinite output."),
}
UTM = {name: rule("conversion_ported", ("railway_mls.coordinates:" + target,),
                   "Literal double-precision source formulas and pi/ellipsoid constants; actual forward input degrees, inverse radians, source zone behavior and initialized IEEE results are preserved. Unsafe automatic-zone casts are explicit guards.")
       for name, target in (
           ("DegToRad", "deg_to_rad"), ("RadToDeg", "rad_to_deg"),
           ("ArcLengthOfMeridian", "arc_length_of_meridian"), ("UTMCentralMeridian", "utm_central_meridian"),
           ("FootpointLatitude", "footpoint_latitude"), ("MapLatLonToXY", "map_lat_lon_to_xy"),
           ("MapXYToLatLon", "map_xy_to_lat_lon"), ("LatLonToUTMXY", "lat_lon_to_utm_xy"),
           ("UTMXYToLatLon", "utm_xy_to_lat_lon"))}
CONVERSION_UI = {}
for name in ("utmConvertButton_Click", "ecefConvertButton_Click"):
    CONVERSION_UI[name] = rule("conversion_ported", ("railway_mls.conversion:convert_legacy_files",),
                               "Ordered source batch dispatch/naming, missing-file skips, reverse hemisphere false and asymmetric aggregate progress timing. Explicit paths replace controls; Python file/error/overwrite policies remain explicit.")
for name in ("sourceButton_Click", "destinationButton_Click", "UpdateProgress", "UpdateAllFilesProgress", "CheckErrors"):
    CONVERSION_UI[name] = rule("presentation", note="File/folder selection, progress-bar rendering or nonempty-control validation. Callers provide explicit paths/zone; numerical workflows expose callbacks without recreating Windows Forms limits.")

READERS = {
    "TXTFileReader": rule("io_ported", ("railway_mls.legacy_io:LegacyTextReader",), "Reader initialization and owned loaded-data state."),
    "~TXTFileReader": rule("native_lifetime", note="Empty source destructor; Python owns reader/cloud references."),
    "ReadDataCSV": rule("io_ported", ("railway_mls.legacy_io:LegacyTextReader.read",), "Count-prefixed strtok CSV, fourth field intensity, ordered records and source callback cadence."),
    "saveRailVector": rule("io_ported", ("railway_mls.legacy_io:format_rail_vector", "railway_mls.legacy_io:write_rail_vector"), "Both overloads: fixed three decimals, center tags, center/rail1/rail2 order and tag 100 on both rails."),
    "saveRailVectorWithIndex": rule("io_ported", ("railway_mls.legacy_io:format_rail_vector",), "Three rows per left record, one marker skip, 100/200 rail tags, partial-prefix diagnostics on undefined indexing."),
    "LASFileReader": rule("io_ported", ("railway_mls.legacy_io:LegacyLasReader",), "Owned loaded-data state; optional decoder boundary."),
    "~LASFileReader": rule("native_lifetime", note="Native shared-pointer reset; Python owns reader/cloud references."),
    "minPoint": rule("io_ported", ("railway_mls.legacy_io:LegacyLasReader.read_bounds",), "Original LAS header minima, not sampled-cloud minima."),
    "maxPoint": rule("io_ported", ("railway_mls.legacy_io:LegacyLasReader.read_bounds",), "Original LAS header maxima, not sampled-cloud maxima."),
}
for name in ("DecreaseDensity", "Transform"):
    READERS[name] = rule("native_filter_pending", note="Unused wrappers around PCL float32 voxel-centroid filtering or centroid translation. They are not called by either reader and are reserved for a separate dependency-level numerical port.")
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
    if path.stem == "HashMap":
        disposition = "io_cache_ported"
    elif path.stem == "TXTFileReader":
        disposition = "io_ported"
    elif path.stem == "LASFileReader":
        disposition = "reader_io_ported_native_filters_pending"
    elif path.stem in (
        "DataConvertor",
        "UTM",
        "DataConverstionForm",
    ):
        disposition = "coordinate_conversion_ported"
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
    ("TXTFileReader.cpp", "TXTFileReader", READERS),
    ("LASFileReader.cpp", "LASFileReader", READERS),
    ("DataConvertor.cpp", "DataConvertor", CONVERTER),
    ("UTM.cpp", "UTM", UTM),
    ("DataConverstionForm.cpp", "PCLProject::DataConverstionForm", CONVERSION_UI),
):
    entries = []
    inventory = cpp_free_definitions if filename == "UTM.cpp" else cpp_definitions
    for definition in inventory((source / filename).read_text(), owner):
        entry = dict(definition)
        if owner in ("TXTFileReader", "LASFileReader") and entry["name"] in ("ReadData", "LoadedData"):
            target = "LegacyTextReader" if owner == "TXTFileReader" else "LegacyLasReader"
            entry.update(rule("io_ported", ("railway_mls.legacy_io:" + target + (".read" if entry["name"] == "ReadData" else ".loaded_data"),),
                              "Ordered records and loaded-data identity preserved; LAS keeps zero-based even records. Optional codec equivalence, malformed inputs and CRT/runtime differences remain explicit limitations."))
        elif entry["name"] not in rules:
            raise ValueError(f"Review unclassified definition: {entry['id']}")
        else:
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
    kind = ("legacy_io" if owner in ("TXTFileReader", "LASFileReader") else
            "coordinate_conversion" if filename in ("DataConvertor.cpp", "UTM.cpp", "DataConverstionForm.cpp") else "engine_ui")
    item = {"path": filename, "owner": owner, "kind": kind, "definitions": entries}
    if filename == "UTM.cpp":
        item["definition_style"] = "free_functions"
    definition_files.append(item)
inline_entries, header_types = header_entries((source / "LIDARPT.h").read_text())
definition_files.append({"path": "LIDARPT.h", "kind": "inline_header", "definitions": inline_entries})
baseline = sha256(
    "\n".join(f"{f['path']}:{f['sha256']}" for f in (*files, *dependencies)).encode()
).hexdigest()
audit = {
    "schema_version": 1,
    "baseline_id": baseline,
    "definition_scope": [
        "Every out-of-class definition in RailwayExtraction.cpp and MainForm.cpp",
        "All 18 out-of-class definitions in TXTFileReader.cpp and LASFileReader.cpp, including unused native-filter gaps",
        "All 27 definitions in DataConvertor.cpp, UTM.cpp and DataConverstionForm.cpp, including free projection functions and computational UI actions",
        "All top-level project .cpp/.h files have file-level dispositions",
        "All 77 active inline definitions and 19 named top-level types in LIDARPT.h have explicit dispositions; commented Rotation3D/InverseRotation3D remain excluded",
        "Jama/etc are dependencies: only reached numerical behavior is ported, not whole libraries",
    ],
    "files": files,
    "bundled_dependencies": dependencies,
    "definition_files": definition_files,
    "header_types": header_types,
    "excluded_header_definitions": [
        {"source": "LIDARPT.h", "name": name, "reason": "definition is inside a block comment"}
        for name in ("Rotation3D", "InverseRotation3D")
    ],
    "complete_reconstruction": False,
}
destination = root / "src/railway_mls/source_coverage.json"
destination.write_text(json.dumps(audit, indent=2) + "\n")
print(
    f"Wrote {sum(len(f['definitions']) for f in definition_files)} explicit definition dispositions and {len(files)} file hashes."
)
print(f"Also hashed {len(dependencies)} bundled dependency files; only reached math is ported.")
print(dict(Counter(e["disposition"] for f in definition_files for e in f["definitions"])))
