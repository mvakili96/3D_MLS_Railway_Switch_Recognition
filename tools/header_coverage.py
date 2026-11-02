"""Explicit dispositions for every active inline definition/type in LIDARPT.h."""

from railway_mls.coverage import cpp_header_inventory


METHODS = {
    "LIDARPT": {
        "bIsNan": "geometry:source_is_nan",
        "FLOOR_double": "geometry:source_float_round",
        "pnpoly": "geometry:points_in_polygon",
        "pnpoly2": "geometry:points_in_convex_polygon",
        "HigherZValue": "fitting:higher_z_value",
        "LargePts": "grouping:larger_group",
    },
    "LiDPOINT": {
        "LiDPOINT": "points:LegacyLiDPoint",
        "~LiDPOINT": "points:LegacyLiDPoint",
        "SetXYZ": "points:LegacyLiDPoint.set_xyz",
        "IsEqual": "points:LegacyLiDPoint.is_equal",
        "operator-": "points:LegacyLiDPoint.__sub__",
        "operator+": "points:LegacyLiDPoint.__add__",
        "operator/": "points:LegacyLiDPoint.__truediv__",
    },
    "POINT2D": {
        "POINT2D": "points:Point2D",
        "IsEqual": "points:Point2D.is_equal",
        "isEqual": "points:Point2D.is_exactly_equal",
        "operator*": "points:Point2D.__mul__",
        "operator/": "points:Point2D.__truediv__",
        "operator-": "points:Point2D.__sub__",
        "operator+": "points:Point2D.__add__",
    },
    "POINT3D": {
        "POINT3D": "points:Point3D",
        "SetPoint3D": "points:Point3D.set_coordinates",
        "ChangeSign": "points:Point3D.changed_sign",
        "Normalize": "points:Point3D.normalized",
        "operator=": "points:Point3D.assigned_xyz",
        "operator-": "points:Point3D.__sub__",
        "operator+": "points:Point3D.__add__",
        "operator*": "points:Point3D.__mul__",
        "operator/": "points:Point3D.__truediv__",
        "IsEqual": "points:Point3D.is_equal",
        "DotProduct": "points:Point3D.dot",
        "CrossProduct": "points:Point3D.cross",
    },
    "Line3D": {
        "Line3D": "geometry:Line3D",
        "GetAngle": "geometry:Line3D.update_angle",
        "GetLineParameter": "geometry:Line3D.update_coefficients",
        "Get2DLineParameterWithAngleAndPt": "geometry:Line3D.set_parameters_from_angle_at",
        "GetCenterPt": "geometry:Line3D.update_center",
        "SwapEnding": "geometry:Line3D.swap_endpoints",
        "SetEndingByMode": "geometry:Line3D.order_endpoints",
        "SetEndByReferencePt": "geometry:Line3D.order_endpoints_by_reference",
        "ExtendEndingPts": "geometry:Line3D.extend_endpoints",
        "IsVertex": "geometry:Line3D.is_vertex",
        "SwapVertex": "geometry:Line3D.swap_endpoints",
        "GetTopologyUsingCrossProduct": "geometry:Line3D.topology",
        "SetEndingPnts": "geometry:Line3D.set_ending_points",
        "SetEndingPnts3D": "geometry:Line3D.set_ending_points_3d",
        "SetEndingPntsByCenterAndLength": "geometry:Line3D.set_ending_points_by_center_and_length",
        "GetUnitVec": "geometry:Line3D.update_unit_vector",
        "GetBufferVtxOfLine": "geometry:Line3D.buffer_vertices",
    },
    "GMMPara": {"GMMPara": "models:GMMParameters", "operator=": "models:GMMParameters.assign_from"},
    "TrackWindow": {
        "TrackWindow": "models:TrackWindow",
        "ResetTrackWindow": "models:TrackWindow.reset",
        "GenerateVtxUsingCenterAndAngle": "models:TrackWindow.generate_vertices",
        "SetTrackWindowMemPnts": "models:TrackWindow.assign_members",
        "GetMaxMinOfMemPnts": "models:TrackWindow.member_bounds",
        "GetSideVtx": "models:TrackWindow.generate_side_vertices",
    },
    "TrackSegment": {
        "TrackSegment": "models:TrackSegment",
        "SetTrkGauge": "models:TrackSegment.set_gauge",
        "GetNeighborVtxBND": "models:TrackSegment.generate_neighbor_regions",
        "GetCenterOfTwoTracks": "models:TrackSegment.update_center",
        "GetDistanceBetweenTwoCenter": "models:TrackSegment.update_gauge",
        "DuplicatePolyPair": "models:TrackSegment.duplicate_from",
        "SwapWnd": "models:TrackSegment.swap_windows",
    },
    "SwitchBox": {"GenerateRgnsUsingCenterPt": "models:SwitchBox.generate_region"},
    "BinaryString": {
        "BinaryString": "models:BinaryString",
        "SetBinaryString": "models:BinaryString.set_bits",
        "SetLables": "models:BinaryString.set_labels",
    },
    "TrackStack": {"TrackStack": "models:TrackStack"},
    "ToePt": {"ToePt": "models:ToePoint"},
    "PlnPara": {"operator=": "planes:PlaneParameters"},
    "GradeSect": {
        "GradeSect": "models:GradeSection",
        "ResetGradeSect": "models:GradeSection.reset",
    },
}

TYPES = {
    "LiDPOINT": (
        "points:LegacyLiDPoint",
        "Unused legacy point metadata/defaults; XYZ unknown until assigned; native division is undefined and guarded.",
    ),
    "POINT2D": (
        "points:Point2D",
        "Default flag false; XY-constructor flag None; zero division returns a value copy and preserves flag.",
    ),
    "POINT3D": (
        "points:Point3D",
        "Frozen value API requires explicit XYZ. Assignment/normalization/sign preserve tags; arithmetic resets tags. Native union layout is not emulated.",
    ),
    "Line3D": (
        "geometry:Line3D",
        "Explicit-endpoint initialized API plus BoundaryLine for missing caches; stored unit cache is distinct from derived direction; D3D stays None.",
    ),
    "GMMPara": ("models:GMMParameters", "Three zero-default scalar fields and assignment."),
    "Grid": (
        "spatial:GridIndex",
        "Array/sparse backend retains reached indexing/member order; unused/uninitialized per-cell metadata is not manufactured.",
    ),
    "TrackWindow": (
        "models:TrackWindow",
        "Explicit center/orientation boundary; ordered main/side polygons, selections, line state, reset and tags.",
    ),
    "TrackSegment": (
        "models:TrackSegment",
        "Gauge, distance, stored/computed centers and five neighbor polygons; duplicate preserves destination distance/regions and flags capacity-dependent tag transport.",
    ),
    "SwitchBox": (
        "models:SwitchBox",
        "Center/angle required; region, optional native uninitialized reference height.",
    ),
    "BinaryString": (
        "models:BinaryString",
        "All scalar defaults, bits, source type/position/direction/toe labels.",
    ),
    "TrackStack": ("models:TrackStack", "Ordered segment values and independent binary state."),
    "ToePt": (
        "models:ToePoint",
        "Type -1/used false; unavailable geometry/angle/mode explicitly None.",
    ),
    "LoopCross": (
        "models:LoopCross",
        "Unavailable line/distance None; inactive workflow remains pending and unscheduled.",
    ),
    "PlnPara": (
        "planes:PlaneParameters",
        "Explicit coefficients, immutable value-copy API; uninitialized native output is handled by optional plane results.",
    ),
    "PtSegment": (
        "grouping:PointGrouping",
        "Array-backed groups/selectors and shared SegmentID labels; no invented standalone uninitialized SegmentID.",
    ),
    "TrackLayer": (
        "models:TrackLayer",
        "All native containers represented, including empty loop/visual-loop/boundary/stack pools and unknown number.",
    ),
    "InitialLocation": (
        "models:InitialLocation",
        "Explicit seed coordinates/angle/mode and optional paired second point; no guessed default coordinates.",
    ),
    "GradeSect": ("models:GradeSection", "All endpoint/tag/membership/scalar defaults and reset."),
    "ExtractionParameters": (
        "config:ExtractionConfig",
        "Ten UI parameters mapped to core configuration; defaults come from source engine, not the uninitialized native aggregate.",
    ),
}


def header_entries(text):
    definitions, types = cpp_header_inventory(text)
    entries = []
    for item in definitions:
        owner, name = item["owner"], item["name"]
        target = METHODS[owner][name]  # Unknown additions require an explicit review.
        if item["id"] == "LiDPOINT::SetXYZ#2":
            target = "points:LegacyLiDPoint.set_from"
        entry = {
            **item,
            "python": ["railway_mls." + target],
            "disposition": "ported_inline",
            "note": TYPES[owner][1]
            if owner in TYPES
            else "Standalone predicate/comparator; preserve literal arithmetic and source ordering policies.",
        }
        if item["id"] in ("POINT3D::POINT3D#1", "Line3D::Line3D#1", "TrackWindow::TrackWindow#1"):
            entry["disposition"] = "explicit_input_policy"
        if owner == "LiDPOINT" and name == "operator/":
            entry["disposition"] = "undefined_source_guard"
        if owner == "LiDPOINT" and name.startswith("~"):
            entry["disposition"] = "native_lifetime"
        entries.append(entry)
    type_entries = [
        {
            **item,
            "source": "LIDARPT.h",
            "python": ["railway_mls." + TYPES[item["name"]][0]],
            "note": TYPES[item["name"]][1],
        }
        for item in types
    ]
    return entries, type_entries
