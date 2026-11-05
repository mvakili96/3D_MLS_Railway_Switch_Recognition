"""Command-line examples for implemented Python stages."""

import argparse
from dataclasses import asdict
import json
import sys
from math import isfinite
from .viewer_model import DEFAULT_CROP_RADIUS, DEFAULT_TRACKING_POINT_LIMIT, MAX_CROP_RADIUS

import numpy as np

from .heads import extract_track_points
from .config import ExtractionConfig
from .fitting import fit_line_ransac
from .gauge import adjust_two_lines_with_observation
from .geometry import (
    Line3D, direction_mode, distance_to_line_3d, inner_angle, intersection_point_3d,
    lines_from_vertices, min_max_xy, recount,
)
from .gmm import classify_segment
from .initialization import initial_kalman_states, initialize_tracks
from .kalman import kalman_predict, kalman_update
from .mdl import segment_track_points_mdl
from .models import BinaryString, TrackSegment, TrackWindow, seed_windows
from .orchestration import TrackingEngine
from .switch_boxes import detect_switch_boxes
from .grades import build_gwad_rows, compute_grade, format_gwad
from .observations import determine_observation, observation_from_support
from .points import Point3D, PointCloud
from .sampling import MsvcRand
from .recognition import define_templates, recognize_stacks, recognize_turnout
from .spatial import GridIndex
from .synthetic import (
    initialization_scene,
    paired_head_cloud,
    parallel_head_cloud,
    straight_scene,
    tracking_scene,
    signature_stack,
    toe_scene,
    branch_scene,
    bidirectional_scene,
    switch_box_scene,
    grade_scene,
)
from .tracking import track_railway
from .toes import SUPPORTED_TOE_TYPES, detect_toe_points
from .pipeline import PipelineOptions, run_pipeline
from .artifacts import pipeline_summary, strict_json_value, write_pipeline_outputs
from .heads import filter_segment_by_previous_heights
from .coverage import check_source_coverage, coverage_summary
from .reconstruction import reconstruction_summary
from .synthetic import pipeline_scene
from .points import LegacyLiDPoint, Point2D
from .geometry import source_float_round
from .models import TrackLayer
from .loop_cross import detect_loop_cross, validate_loop_cross
from .synthetic import loop_cross_scene
from .legacy_io import LegacyTextReader, format_rail_vector, load_point_cloud
from .conversion import CoordinateConverter, format_converted_points, write_converted_points
from .coordinates import CONVERTER_PI, geodetic_to_ecef
from .pcl_helpers import PclHeader, PclXYZCloud, decrease_density, center_point_cloud


def _pcl_cloud_json(cloud) -> dict:
    return {"xyz": cloud.xyz.tolist(), "width": cloud.width, "height": cloud.height,
            "is_dense": cloud.is_dense, "header": asdict(cloud.header),
            "sensor_origin": cloud.sensor_origin.tolist(),
            "sensor_orientation_xyzw": cloud.sensor_orientation.tolist()}


def _pcl_helpers_demo() -> dict:
    cloud = PclXYZCloud([[.01,.02,.03],[.03,.04,.05],[.21,.02,.03],
                         [.23,.04,.05],[.01,.22,.03],[.03,.24,.05]],
                        width=6, height=1, header=PclHeader(7,12345,"synthetic"),
                        sensor_origin=(10,20,30,1))
    voxel = decrease_density(cloud)
    centered = center_point_cloud(cloud)
    fallback = decrease_density(PclXYZCloud([[0,0,0],[1000,1000,1000]]))
    empty = PclXYZCloud(np.empty((0,3)))
    empty_voxel, empty_center = decrease_density(empty), center_point_cloud(empty)
    tied = decrease_density(PclXYZCloud([[.09,0,0],*[[1e-9,0,0]]*31]))
    return strict_json_value({
        "status": "ok", "data": "generated PointXYZ helper fixtures",
        "automatic_processing_active": False, "input_count": len(cloud),
        "voxel": {"status": voxel.status, "output": _pcl_cloud_json(voxel.cloud),
                  "groups": voxel.groups, "reference_comparable": voxel.reference_comparable},
        "centering": {"status": centered.status, "centroid": centered.centroid,
                      "count": centered.centroid_count, "matrix": centered.matrix,
                      "output": _pcl_cloud_json(centered.cloud)},
        "extent_fallback": {"status": fallback.status, "output_count": len(fallback.cloud)},
        "empty_voxel": {"status": empty_voxel.status, "output_available": empty_voxel.cloud is not None},
        "empty_center": {"status": empty_center.status, "centroid": empty_center.centroid},
        "native_sort_ties": {"status": tied.status, "sort_sensitive_voxels": tied.sort_sensitive_voxels,
                             "reference_comparable": tied.reference_comparable},
    })


def _conversion_demo() -> dict:
    cloud = PointCloud([geodetic_to_ecef((52 + i * .01) * CONVERTER_PI / 180,
                                        (9 + i * .001) * CONVERTER_PI / 180, 100 + i)
                        for i in range(12)], np.arange(12) * 100,
                       class_codes=[7] * 12, segment_ids=[9] * 12)
    events, export_events = [], []
    converter = CoordinateConverter(cloud)
    projected = converter.convert_ecef_to_utm(32, callback=lambda *v: events.append(v))
    reverse = CoordinateConverter(projected.as_cloud()).convert_utm_to_ecef(32, False)
    export = format_converted_points(projected.points, callback=lambda *v: export_events.append(v))
    repeat = CoordinateConverter(PointCloud(cloud.xyz[:4]))
    repeat.convert_ecef_to_utm(32)
    repeat_events = []
    repeated = repeat.convert_ecef_to_utm(32, callback=lambda *v: repeat_events.append(v))
    origin = CoordinateConverter(PointCloud([[0, 0, 0]])).convert_ecef_to_utm(32)
    return strict_json_value({
        "status": projected.status, "data": "generated geodetic/ECEF component fixture",
        "input_count": len(cloud), "zone": 32, "inverse_south_hemi": False,
        "first_utm_point": asdict(projected.points[0]),
        "roundtrip_max_abs_xyz_error": float(np.max(np.abs(reverse.as_cloud().xyz - cloud.xyz))),
        "conversion_progress": events, "export_progress": export_events,
        "converted_pts": export.text,
        "repeated_call": {"input_count": 4, "buffer_count": len(repeated.points),
                          "status": repeated.status, "progress": repeat_events},
        "origin": {"status": origin.status, "point": asdict(origin.points[0])},
        "source_batch_inverse_south_hemi": False,
        "input_labels_unchanged": bool(np.all(cloud.class_codes == 7) and np.all(cloud.segment_ids == 9)),
    })


def _loaded_cloud_json(loaded) -> dict:
    return {
        "status": "ok", "source_format": loaded.source_format,
        "declared_count": loaded.declared_count, "input_count": loaded.input_count,
        "loaded_count": len(loaded.cloud), "reader_bounds": asdict(loaded.reader_bounds),
        "first_record_indices": loaded.record_indices[:20].tolist(),
        "class_codes": np.unique(loaded.cloud.class_codes).tolist(),
        "segment_ids": np.unique(loaded.cloud.segment_ids).tolist(),
        "coordinate_units": None, "crs": None,
    }


def _legacy_io_demo() -> dict:
    from pathlib import Path
    from tempfile import TemporaryDirectory
    # Deliberately synthetic file fixtures; all temporary files are cleaned up.
    with TemporaryDirectory(prefix="railway-mls-io-demo-") as directory:
        path = Path(directory) / "points.pts"
        path.write_text("12\n" + "".join(f"{500000 + i * .125:.9f} {6000000 + i * .25:.9f} {100 + i * .1:.9f} {i}\n" for i in range(12)))
        events = []
        loaded = LegacyTextReader().read(path, callback=lambda *value: events.append(value))
        center = [Point3D(0, 0, 0, 2), Point3D(1, 0, 0, 7), Point3D(2, 0, 0, 9)]
        left = [Point3D(1, 1, 0), Point3D(2, 1, 0)]
        right = [Point3D(1, -1, 0), Point3D(2, -1, 0)]
        exports = {"center": format_rail_vector(center),
                   "paired": format_rail_vector(center, left, right),
                   "indexed": format_rail_vector(center, left, right, indexed=True)}
        csv = path.with_suffix(".csv")
        csv.write_text(exports["paired"].text)
        csv_loaded = load_point_cloud(csv)
        undefined = format_rail_vector([], left, right, indexed=True)
        return {"status": "ok", "data": "generated temporary legacy text/vector fixtures",
                "pts": _loaded_cloud_json(loaded), "progress": events,
                "grid_bounds": asdict(loaded.grid_bounds()),
                "exports": {name: asdict(value) for name, value in exports.items()},
                "csv": _loaded_cloud_json(csv_loaded),
                "csv_intensity": csv_loaded.cloud.intensity.tolist(),
                "undefined_indexed_export": asdict(undefined)}


def _loop_cross_demo(seed: int) -> dict:
    scene = loop_cross_scene()
    random = MsvcRand(seed)
    pool = []
    first = detect_loop_cross(scene.segment, scene.cloud.select(), scene.ground_height,
                              scene.track_height, scene.reference_line,
                              config=scene.config, rng=random, loop_crosses=pool)
    repeated = detect_loop_cross(scene.segment, scene.cloud.select(), scene.ground_height,
                                 scene.track_height, scene.reference_line,
                                 config=scene.config, rng=random, loop_crosses=pool)
    validated = validate_loop_cross(pool)
    sparse_random = MsvcRand(seed)
    sparse = detect_loop_cross(scene.segment, scene.cloud.select([*range(7), *range(30, 37)]),
                               scene.ground_height, scene.track_height, scene.reference_line,
                               config=scene.config, rng=sparse_random)
    return _finite_json({
        "stage": "standalone source loop/cross detection and validation",
        "data": "synthetic inner strips with explicitly initialized rail references",
        "automatic_scheduling_active": False,
        "status": first.status,
        "input_points": len(scene.cloud),
        "region_counts": [len(group) for group in first.region_points],
        "polygon_count": len(first.polygon_points),
        "height_count": len(first.height_points),
        "candidate_ids": first.candidates.indices.tolist(),
        "fit": {"status": first.fit.status, "support_count": len(first.fit.support),
                "iterations": first.fit.iterations},
        "projected_line": asdict(first.projected_line),
        "distance_from_start": first.candidate.distance_from_start,
        "repeat": {"status": repeated.status, "appended": repeated.appended,
                   "duplicate_distances": repeated.duplicate_distances},
        "validation": {"status": validated.status, "groups": validated.groups,
                       "selected_indices": validated.selected_indices},
        "loop_count": len(pool),
        "random_draws": random.draw_count,
        "sparse_fit": {"status": sparse.status, "fit_status": sparse.fit.status,
                       "appended": sparse.appended, "random_draws": sparse_random.draw_count},
        "labels_unchanged": bool(np.all(scene.cloud.class_codes == 9)
                                 and np.all(scene.cloud.segment_ids == 7)),
    })


def _header_demo() -> dict:
    try:
        LegacyLiDPoint(1, 2, 3) / 2
    except ValueError as error:
        division_status = str(error).partition(":")[0]
    line = Line3D(Point3D(0, 0, 2, 7), Point3D(2, 1, 4, 9))
    expanded = line.extend_endpoints(2)
    line.swap_endpoints()
    unit = line.update_unit_vector()
    segment = TrackSegment(TrackWindow(Point3D(1000, 2000, 1), 35),
                           TrackWindow(Point3D(1001.2, 1999.1, 9), 35))
    regions = segment.generate_neighbor_regions(1.1, 0.5, 0.42)
    source = TrackSegment(TrackWindow(Point3D(20, 21, 22), 35),
                          TrackWindow(Point3D(23, 24, 25), 35))
    source.update_center()
    segment.distance_from_start = 99
    assignment = segment.duplicate_from(source)
    point = Point2D(3, 4)
    return strict_json_value({
        "status": "ok", "rounding": [source_float_round(v) for v in (-1.8, 0.5, 1.49999999)],
        "point2d": asdict(point), "zero_division_copy": asdict(point / 0),
        "normalized_marked_point": asdict(Point3D(3, 4, 0, 17).normalized()),
        "legacy_default": asdict(LegacyLiDPoint()),
        "legacy_division_status": division_status,
        "expanded_line": asdict(expanded), "endpoint_tags_after_swap": [line.begin.point_type, line.end.point_type],
        "stored_unit_vector": asdict(unit), "neighbor_status": regions.status,
        "neighbor_regions": asdict(regions), "duplicate": asdict(assignment),
        "retained_distance": segment.distance_from_start,
        "inactive_pools_empty": not TrackLayer().loop_crosses,
        "source_coverage": coverage_summary(), "random_draws": 0,
    })


def _utilities_demo() -> dict:
    heights = (-0.1, 0.0, 0.1, 0.2, 0.3)
    xyz = [(i * 0.01, rail * 1.5, height) for rail in range(2)
           for height in heights for i in range(6)]
    cloud = PointCloud(xyz, class_codes=[9] * len(xyz), segment_ids=[7] * len(xyz))
    segment = TrackSegment(TrackWindow(Point3D(0, 0, 0.2), 0, members=cloud.select(range(30))),
                           TrackWindow(Point3D(0, 1.5, 0.2), 0, members=cloud.select(range(30, 60))))
    filtered = filter_segment_by_previous_heights(segment, (0.2, 0.2), (0, 0), 0, 0)
    classifications = classify_segment(segment)
    vertices = (Point3D(0, 0, 2, 7), Point3D(2, 0, 3, 9), Point3D(1, 1, 4, 11))
    line = Line3D(Point3D(0, 0, 0), Point3D(2, 0, 0))
    return strict_json_value({
        "status": "ok", "filtered_counts": [len(points) for points in filtered],
        "gmm": [{"fit": asdict(result.fit), "head_ids": result.heads.indices.tolist(),
                 "cluster_counts": [len(cluster) for cluster in result.clustering.clusters]}
                for result in classifications],
        "labels_unchanged": bool(np.all(cloud.class_codes == 9) and np.all(cloud.segment_ids == 7)),
        "bounds": min_max_xy(cloud.select()), "empty_bounds": min_max_xy(cloud.select([])),
        "inner_angle": inner_angle(Point3D(1, 0, 4), Point3D(0, 0, 1), Point3D(0, 1, 9)),
        "degenerate_inner_angle": inner_angle(Point3D(0, 0, 0), Point3D(0, 0, 0), Point3D(1, 0, 0)),
        "distance_3d": distance_to_line_3d(Point3D(1, 3, 4), line, unit_vector=Point3D(1, 0, 0)),
        "recount": [{"total": total, "current": current, "result": recount(total, current)}
                    for total, current in ((4, 5), (4, -1), (4, -9), (0, -1))],
        "boundaries": [asdict(edge) for edge in lines_from_vertices(vertices)],
        "inactive_intersection": asdict(intersection_point_3d(2, -1, 1, 1, 0, -3)),
        "random_draws": 0,
    })


def _toe_json(toe):
    return {
        "track_type": toe.track_type,
        "used": toe.used,
        "center": toe.center.as_array().tolist() if toe.center is not None else None,
        "left": toe.left.as_array().tolist() if toe.left is not None else None,
        "right": toe.right.as_array().tolist() if toe.right is not None else None,
        "angle": toe.angle,
        "mode": toe.mode,
    }


def _box_json(box):
    return {
        "center": box.center.as_array().tolist(),
        "angle": box.angle,
        "reference_height": box.reference_height,
        "vertices": [point.as_array().tolist() for point in box.vertices],
    }


def _grade_demo() -> dict:
    layer = grade_scene()
    analysis = compute_grade(layer)
    export = build_gwad_rows([layer], ["synthetic-main"])
    return _finite_json({
        "stage": "source grade and curve analysis on a supplied center vector",
        "status": analysis.status,
        "input_vertices": len(layer.center_vector),
        "grade_vertex_indices": analysis.division.marked_indices,
        "new_grade_vertex_indices": analysis.division.new_indices,
        "section_count": len(analysis.sections),
        "sections": [
            {"begin": s.begin.as_array().tolist(), "end": s.end.as_array().tolist(),
             "member_count": len(s.member_vertices), "arc": s.arc, "chord": s.chord,
             "height": s.height, "grade": s.grade, "radius": s.radius,
             "degree_of_curve": s.degree_of_curve, "grade_with_curve": s.grade_with_curve}
            for s in analysis.sections
        ],
        "report": [
            {"point": row.point.as_array().tolist(), "distance_from_start": row.distance_from_start,
             "grade_begin": row.grade_begin, "grade_end": row.grade_end,
             "grade_difference": row.grade_difference}
            for row in analysis.report
        ],
        "gwad_rows": [
            {"point": row.point.as_array().tolist(), "layer_name": row.layer_name,
             "chainage": row.chainage, "element": row.element, "parameters": row.parameters}
            for row in export.rows
        ],
        "gwad_tsv": format_gwad(export),
        "grade_analysis_implemented": True,
    })


def _switch_box_demo(seed: int) -> dict:
    scene = switch_box_scene()
    random = MsvcRand(seed)
    result = detect_switch_boxes(scene.toes, GridIndex(scene.cloud), config=scene.config, rng=random)
    return _finite_json({
        "stage": "source switch-box detection from planar patches and a supplied toe",
        "status": result.status,
        "input_points": len(scene.cloud),
        "candidate_count": len(result.candidates.candidates),
        "detected_count": len(result.new_boxes),
        "random_seed": seed,
        "random_draws": random.draw_count,
        "reference_comparable": result.reference_comparable,
        "cases": [
            {
                "status": entry.status,
                "region_points": len(entry.region_points),
                "height_points": len(entry.height_points),
                "group_points": len(entry.grouping.selected) if entry.grouping is not None else 0,
                "plane": entry.second_fit.plane.as_array().tolist()
                    if entry.second_fit is not None and entry.second_fit.plane is not None else None,
                "box": _box_json(entry.box) if entry.box is not None else None,
            }
            for entry in result.entries
        ],
        "class_5_points": int(np.count_nonzero(scene.cloud.class_codes == 5)),
        "switch_box_detection_implemented": True,
        "switch_box_detection_run": True,
    })


def _toe_demo(seed: int, type_id: int | None = None) -> dict:
    random = MsvcRand(seed)
    cases = []
    for current_type in SUPPORTED_TOE_TYPES if type_id is None else (type_id,):
        scene = toe_scene(current_type)
        result = detect_toe_points(
            current_type, scene.stack.track_segments, config=scene.config, rng=random
        )
        cases.append(
            {
                "status": result.status,
                "source_return": result.source_return,
                "source_segments": result.source_segments,
                "side": result.side,
                "reference_segment": result.reference_segment,
                "selected_endpoint": result.selected_endpoint,
                "candidate_count": len(result.candidates) if result.candidates is not None else 0,
                "group_count": len(result.grouping.selected) if result.grouping is not None else 0,
                "support_count": len(result.fit.support) if result.fit is not None else 0,
                "toe": _toe_json(result.toe),
            }
        )
    missing = toe_scene(initialized_references=False)
    missing_random = MsvcRand(seed)
    missing_result = detect_toe_points(0, missing.stack.track_segments, rng=missing_random)
    return _finite_json(
        {
            "stage": "source toe detection and endpoint refinement with initialized references",
            "status": "ok" if all(case["status"] == "ok" for case in cases) else "failed",
            "detected_count": sum(case["source_return"] is True for case in cases),
            "random_seed": seed,
            "random_draws": random.draw_count,
            "cases": cases,
            "uninitialized_reference_example": {
                "status": missing_result.status,
                "source_return": missing_result.source_return,
                "random_draws": missing_random.draw_count,
            },
            "toe_detection_implemented": True,
            "branch_orchestration_implemented": True,
            "branch_orchestration_run": False,
            "switch_box_detection_implemented": True,
            "switch_box_detection_run": False,
        }
    )


def _finite_json(value):
    if isinstance(value, float) and not isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _finite_json(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_finite_json(item) for item in value]
    return value


def _pass_json(result):
    def invocation(item):
        return {
            "status": item.status,
            "layer_index": item.layer_index,
            "direction": item.direction,
            "main_track": item.main_track,
            "accepted_steps": item.result.accepted_steps if item.result is not None else 0,
            "switch_box_status": (item.result.switch_box_detection.status
                                  if item.result is not None
                                  and item.result.switch_box_detection is not None else None),
        }

    return {
        "status": result.status,
        "reference_comparable": result.reference_comparable,
        "main": invocation(result.main),
        "branches": [
            {
                "toe_index": branch.toe_index,
                "track_type": branch.toe.track_type,
                "status": branch.status,
                "layer_index": branch.layer_index,
                "local_used": branch.toe.used,
                "seed": {
                    "point": branch.seed.point.as_array().tolist(),
                    "angle": branch.seed.angle,
                    "mode": branch.seed.mode,
                } if branch.seed is not None else None,
                "initialization_status": (
                    branch.initialization.status if branch.initialization is not None else None
                ),
                "tracking": invocation(branch.tracking) if branch.tracking is not None else None,
            }
            for branch in result.branches
        ],
        "center_tags": [asdict(tag) for tag in result.tags],
    }


def _orchestration_demo(seed: int, bidirectional: bool, max_steps: int | None) -> dict:
    scene = bidirectional_scene() if bidirectional else branch_scene()
    engine = TrackingEngine(GridIndex(scene.cloud), config=scene.config, rng=MsvcRand(seed))
    initialized = engine.initialize(scene.seed)
    payload = {
        "stage": "source bidirectional orchestration" if bidirectional else "source branch scheduling",
        "status": initialized.status,
        "input_points": len(scene.cloud),
        "random_seed": seed,
        "toe_input": "automatic postpass only" if bidirectional else "supplied toe geometry fixture",
        "branch_orchestration_implemented": True,
        "bidirectional_orchestration_implemented": True,
        "switch_box_detection_implemented": True,
    }
    if initialized.success:
        if bidirectional:
            result = engine.run_bidirectional(max_steps=max_steps)
            payload.update(
                status=result.status,
                reverse_seed={
                    "point": result.reverse_location.point.as_array().tolist(),
                    "angle": result.reverse_location.angle,
                    "mode": result.reverse_location.mode,
                },
                reverse_pass=_pass_json(result.reverse_pass) if result.reverse_pass else None,
                reversal_status=result.reversal.status if result.reversal else None,
                forward_pass=_pass_json(result.forward_pass) if result.forward_pass else None,
                reference_comparable=result.reference_comparable,
            )
        else:
            engine.toe_points.append(scene.toe)
            result = engine.run_pass(max_steps=max_steps)
            payload.update(
                status=result.status, tracking_pass=_pass_json(result),
                reference_comparable=result.reference_comparable,
            )
    payload.update(
        random_draws=engine.rng.draw_count,
        layer_count=len(engine.layers),
        current_layer_index=engine.current_layer_index,
        global_toe_used=[toe.used for toe in engine.toe_points],
        switch_box_count=len(engine.switch_boxes),
        layers=[
            {
                "saved_segments": len(layer.track_segments),
                "rail1_vector": [point.as_array().tolist() for point in layer.rail1_vector],
                "rail2_vector": [point.as_array().tolist() for point in layer.rail2_vector],
                "center_vector": [point.as_array().tolist() for point in layer.center_vector],
                "center_point_types": [point.point_type for point in layer.center_vector],
                "toe_count": len(layer.toe_points),
                "switch_box_count": len(layer.switch_boxes),
            }
            for layer in engine.layers
        ],
    )
    return _finite_json(payload)


def _recognition_demo() -> dict:
    catalogue = define_templates()
    stacks = [signature_stack(template.bits) for template in catalogue.all]
    result = recognize_stacks(stacks)
    noisy = list(catalogue.all[0].bits)
    noisy[0] = 1 - noisy[0]
    noise_result = recognize_turnout(BinaryString(bits=noisy))
    return {
        "stage": "controlled stack signatures and source template recognition",
        "template_count": len(catalogue.all),
        "matched_count": sum(entry.status == "matched" for entry in result.entries),
        "ambiguous_count": sum(entry.status == "ambiguous" for entry in result.entries),
        "context_weight": 0,
        "cases": [
            {
                "input_template_type": template.type,
                "recognized_type": entry.binary.type,
                "score": entry.binary.score,
                "status": entry.status,
                "matching_template_types": [
                    entry.match.template_types[i] for i in entry.match.best_indices
                ],
                "labels": {
                    "position": entry.binary.position_label,
                    "direction": entry.binary.direction_label,
                    "turnout": entry.binary.turnout_label,
                    "toe": entry.binary.toe_label,
                },
            }
            for template, entry in zip(catalogue.all, result.entries)
        ],
        "one_bit_noise_example": {
            "recognized_type": noise_result.binary.type,
            "score": noise_result.binary.score,
            "status": noise_result.status,
        },
        "recognition_implemented": True,
        "toe_detection_implemented": True,
        "toe_detection_run": False,
        "branch_orchestration_implemented": True,
        "branch_orchestration_run": False,
    }


def _tracking_demo(seed: int, max_steps: int | None, direction: int, side_rails: bool) -> dict:
    scene = tracking_scene(side_rails=side_rails)
    grid = GridIndex(scene.cloud, scene.config.grid_size)
    random = MsvcRand(seed)
    initialized = initialize_tracks(scene.seed, grid, config=scene.config, rng=random)
    payload = {
        "stage": "raw synthetic initialization and sequential tracking",
        "status": initialized.status,
        "input_points": len(scene.cloud),
        "random_seed": seed,
        "tracking_implemented": True,
        "recognition_implemented": True,
        "toe_detection_implemented": True,
        "branch_orchestration_implemented": True,
        "branch_orchestration_run": False,
        "switch_box_detection_implemented": True,
    }
    if not initialized.success:
        return payload
    result = track_railway(
        initialized,
        grid,
        config=scene.config,
        rng=random,
        track_direction=direction,
        max_steps=max_steps,
    )
    payload.update(
        status=result.status,
        accepted_steps=result.accepted_steps,
        saved_segments=len(result.layer.track_segments),
        distance_from_start=result.distance_from_start,
        rail1_vector=[point.as_array().tolist() for point in result.layer.rail1_vector],
        completed_stack_sizes=[len(stack.track_segments) for stack in result.completed_stacks],
        active_stack_size=len(result.active_stack.track_segments),
        reference_comparable=result.reference_comparable,
        random_draws=random.draw_count,
        switch_box_postpass_ran=result.switch_box_detection is not None,
        switch_box_status=(result.switch_box_detection.status
                           if result.switch_box_detection is not None else None),
        switch_box_count=(len(result.switch_box_detection.new_boxes)
                         if result.switch_box_detection is not None else 0),
        recognition_postpass_ran=result.recognition is not None,
        stack_recognition=[
            {
                "index": entry.index,
                "status": entry.status,
                "type": entry.binary.type,
                "score": entry.binary.score,
            }
            for entry in result.recognition.entries
        ]
        if result.recognition is not None
        else [],
        toe_detection_postpass_ran=result.toe_detection is not None,
        toes=[_toe_json(toe) for toe in result.toe_detection.toes]
        if result.toe_detection is not None
        else [],
        toe_diagnostics=[
            {
                "index": entry.index,
                "status": entry.status,
                "source_return": entry.detection.source_return
                if entry.detection is not None
                else None,
            }
            for entry in result.toe_detection.entries
        ]
        if result.toe_detection is not None
        else [],
        steps=[
            {
                "number": step.number,
                "status": step.status,
                "mode": step.mode,
                "attempts": [
                    {
                        "phase": attempt.phase,
                        "length": attempt.length,
                        "member_counts": [
                            len(window.members)
                            for window in (attempt.segment.track1, attempt.segment.track2)
                        ],
                        "head_counts": [len(points) for points in attempt.extraction.heads]
                        if attempt.extraction
                        else None,
                    }
                    for attempt in step.attempts
                ],
                "selected_model_counts": [model.selected_count for model in step.models],
                "model_statuses": [model.status for model in step.models],
                "candidate_gauge": step.candidate.gauge if step.candidate is not None else None,
            }
            for step in result.steps
        ],
    )
    return _finite_json(payload)


def _initialization_demo(seed: int) -> dict:
    scene = initialization_scene()
    random = MsvcRand(seed)
    result = initialize_tracks(
        scene.seed, GridIndex(scene.cloud, scene.config.grid_size), config=scene.config, rng=random
    )
    payload = {
        "stage": "full seed initialization and standalone Kalman example",
        "status": result.status,
        "input_points": len(scene.cloud),
        "window_point_counts": [
            len(window.members) for window in (result.segment.track1, result.segment.track2)
        ],
        "head_point_counts": [len(points) for points in result.heads],
        "saved_segments": len(result.layer.track_segments),
        "updated_boundaries": len(result.layer.updated_boundaries),
        "random_seed": seed,
        "random_draws": random.draw_count,
        "tracking_implemented": False,
    }
    if not result.success:
        return payload
    payload.update(
        initial_heights=result.initial_heights,
        initialized_centers=[
            window.center.as_array().tolist()
            for window in (result.segment.track1, result.segment.track2)
        ],
        initialized_gauge=result.segment.gauge,
        updated_location_mode=result.location.mode,
    )
    first_line = result.segment.track1.member_line
    mode = direction_mode(first_line.end, first_line.begin)
    examples = []
    for state, window in zip(
        initial_kalman_states(result.segment), (result.segment.track1, result.segment.track2)
    ):
        predicted = kalman_predict(
            state, mode=mode, angle=window.orientation, step_length=scene.config.track_window_length
        )
        # This controlled observation demonstrates the filter components; the
        # next increment supplies observations through the full tracking loop.
        observation = predicted.state.point + Point3D(0.01, -0.02, 0.005)
        updated = kalman_update(predicted.state, observation, sigma=0.02)
        examples.append(
            {
                "prediction_status": predicted.status,
                "update_status": updated.status,
                "current": state.point.as_array().tolist(),
                "predicted": predicted.state.point.as_array().tolist(),
                "observation": observation.as_array().tolist(),
                "updated": updated.state.point.as_array().tolist(),
                "updated_covariance": updated.state.covariance.tolist(),
            }
        )
    payload.update(
        kalman_example=examples, observation_source="controlled synthetic offsets from prediction"
    )
    return payload


def _observation_demo(seed: int) -> dict:
    config = ExtractionConfig()
    cloud = paired_head_cloud(config)
    random = MsvcRand(seed)
    shift = config.track_gauge / np.sqrt(2)
    guides = (
        Line3D(Point3D(0, 0, 0.2), Point3D(1, 1, 0.2)),
        Line3D(Point3D(-shift, shift, 0.21), Point3D(1 - shift, 1 + shift, 0.21)),
    )
    candidates = tuple(
        segment_track_points_mdl(
            cloud.select(np.arange(start, start + len(cloud) // 2)),
            mode=1,
            angle=45,
            config=config,
            rng=random,
        )
        for start in (0, len(cloud) // 2)
    )
    payload = {
        "stage": "MDL, observation selection, fitting and paired gauge adjustment",
        "status": "candidate_selection_failed",
        "input_points": len(cloud),
        "candidate_counts": [len(result.lines) for result in candidates],
        "candidate_statuses": [result.status for result in candidates],
        "random_seed": seed,
        "tracking_implemented": False,
    }
    if not all(result.success for result in candidates):
        payload["random_draws"] = random.draw_count
        return payload
    observations = determine_observation(
        candidates[0].lines, candidates[1].lines, *guides, config=config, stack_size=1
    )
    payload.update(
        status=observations.status,
        observation_counts=[len(points) for points in observations.observations],
        selected_indices=observations.selected_indices,
        side_counts={
            "left": [len(points) for points in observations.left_points],
            "right": [len(points) for points in observations.right_points],
        },
    )
    if not observations.success:
        payload["random_draws"] = random.draw_count
        return payload
    fits = tuple(
        fit_line_ransac(points, threshold=config.max_distance_for_ransac, rng=random)
        for points in observations.observations
    )
    if not all(fit.success for fit in fits):
        payload.update(
            status="line_fit_failed",
            fit_statuses=[fit.status for fit in fits],
            random_draws=random.draw_count,
        )
        return payload
    states = tuple(observation_from_support(fit.support) for fit in fits)
    # Standalone component demonstration: full tracking applies this adjustment
    # after its Kalman update, which is a subsequent implementation stage.
    adjusted = adjust_two_lines_with_observation(
        fits[0].line,
        fits[1].line,
        states[0].support,
        states[1].support,
        references=(guides[0].begin, guides[1].begin),
        config=config,
    )
    payload.update(
        status=adjusted.status,
        observation_vectors=[state.point.as_array().tolist() for state in states],
        fit_sigma=[fit.sigma for fit in fits],
        gauge_adjustment={
            "status": adjusted.status,
            "common_angle": adjusted.common_angle,
            "initial_gap": adjusted.initial_gap,
            "weighted_distance": adjusted.weighted_distance,
            "scale": adjusted.scale,
            "lines": [
                {"begin": line.begin.as_array().tolist(), "end": line.end.as_array().tolist()}
                for line in adjusted.lines
            ],
        },
        random_draws=random.draw_count,
    )
    return payload


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="3D MLS railway Python foundation")
    commands = parser.add_subparsers(dest="command", required=True)
    view_parser = commands.add_parser("view", help="open the interactive LAS/LAZ tracking viewer")
    view_parser.add_argument("path", nargs="?", help="LAS/LAZ file; defaults to Vancouver 16.las when available")
    view_parser.add_argument("--radius", type=float, default=DEFAULT_CROP_RADIUS, help="initial XY tracking crop radius in metres (default: %(default)s)")
    view_parser.add_argument("--preview-points", type=int, default=150_000, help="maximum display points; tracking is sampled separately")
    view_parser.add_argument("--max-tracking-points", type=int, default=DEFAULT_TRACKING_POINT_LIMIT, help="crop memory guard; inference is never thinned to fit (default: %(default)s)")
    view_parser.add_argument("--seed", type=int, default=1, help="initial Microsoft CRT random seed")
    commands.add_parser("pcl-helpers-demo", help="exercise source float32 voxel and double-centroid PointXYZ helpers")
    commands.add_parser("conversion-demo", help="exercise source coordinate formulas, buffer state and converted PTS output")
    convert_parser = commands.add_parser("convert-cloud", help="explicit legacy ECEF/UTM conversion with source formulas")
    convert_parser.add_argument("source")
    convert_parser.add_argument("destination")
    convert_parser.add_argument("--direction", choices=("ecef-to-utm", "utm-to-ecef"), required=True)
    convert_parser.add_argument("--zone", type=int, required=True)
    convert_parser.add_argument("--hemisphere", choices=("north", "south"), help="required for inverse conversion")
    convert_parser.add_argument("--format", choices=("pts", "csv", "las", "laz"))
    convert_parser.add_argument("--delimiter", choices=("space", "comma"), default="space")
    convert_parser.add_argument("--overwrite", action="store_true")
    commands.add_parser("legacy-io-demo", help="exercise legacy text readers and all three vector export formats")
    inspect_parser = commands.add_parser("inspect-cloud", help="read a legacy point cloud and report source sampling/bounds")
    inspect_parser.add_argument("path")
    inspect_parser.add_argument("--format", choices=("pts", "csv", "las", "laz"))
    inspect_parser.add_argument("--chunk-size", type=int, default=1_000_000)
    loop_parser = commands.add_parser("loop-cross-demo", help="exercise inactive source loop/cross components with supplied references")
    loop_parser.add_argument("--seed", type=int, default=1, help="Microsoft CRT seed (default: 1)")
    commands.add_parser("header-demo", help="exercise source inline geometry, tags and state contracts")
    commands.add_parser("utilities-demo", help="exercise standalone source utilities and explicit boundary cache state")
    audit_parser = commands.add_parser("source-audit", help="check frozen native source and explicit coverage dispositions")
    audit_parser.add_argument("--repo", default=".", help="original source repository root (default: current directory)")
    review_parser = commands.add_parser("reconstruction-audit", help="check reviewed computational scope, call sites and open limitation IDs")
    review_parser.add_argument("--repo", default=".", help="source checkout including tests/docs (default: current directory)")
    pipeline_parser = commands.add_parser("pipeline-demo", help="run the integrated synthetic workflow and report limitations")
    pipeline_parser.add_argument("--scenario", choices=("straight", "turnout", "components", "missing-references"), default="straight")
    pipeline_parser.add_argument("--seed", type=int, default=1)
    pipeline_parser.add_argument("--max-steps", type=int, help="diagnostic limit per invocation; later stages defer")
    directions = pipeline_parser.add_mutually_exclusive_group()
    directions.add_argument("--bidirectional", dest="bidirectional", action="store_true")
    directions.add_argument("--single-direction", dest="bidirectional", action="store_false")
    pipeline_parser.set_defaults(bidirectional=None)
    pipeline_parser.add_argument("--skip-grades", action="store_true", help="skip both grade and GWAD actions")
    pipeline_parser.add_argument("--record-random", action="store_true", help="include the raw random stream")
    pipeline_parser.add_argument("--steps", action="store_true", help="include detailed steps, memberships and filter state")
    pipeline_parser.add_argument("--output", help="create a new diagnostic output directory")
    commands.add_parser("grade-demo", help="analyze source grades/curves and build legacy GWAD rows")
    commands.add_parser(
        "recognition-demo", help="classify controlled stack signatures with all 20 source templates"
    )
    toe_parser = commands.add_parser(
        "toe-demo", help="detect and refine toe points for the twelve supported source types"
    )
    toe_parser.add_argument("--seed", type=int, default=1, help="Microsoft CRT seed (default: 1)")
    toe_parser.add_argument(
        "--type",
        type=int,
        choices=SUPPORTED_TOE_TYPES,
        help="supported source type; default runs all twelve",
    )
    switch_parser = commands.add_parser(
        "switch-box-demo", help="detect switch boxes in dense planar patches beside a supplied toe"
    )
    switch_parser.add_argument("--seed", type=int, default=1)
    for name, help_text in (
        ("branch-demo", "run main and branch tracking on raw corridors with a supplied toe fixture"),
        ("bidirectional-demo", "run the source UI's two-direction tracking and vector assembly"),
    ):
        orchestration_parser = commands.add_parser(name, help=help_text)
        orchestration_parser.add_argument("--seed", type=int, default=1)
        orchestration_parser.add_argument("--max-steps", type=int, help="limit each tracking invocation")
    commands.add_parser("demo", help="construct paired windows and select synthetic points")
    heads_parser = commands.add_parser(
        "heads-demo", help="run height GMM and paired rail-head extraction"
    )
    heads_parser.add_argument(
        "--seed", type=int, default=1, help="Microsoft CRT random seed (default: 1)"
    )
    mdl_parser = commands.add_parser(
        "mdl-demo", help="fit, group and select one/two/three-line models"
    )
    mdl_parser.add_argument("--seed", type=int, default=1, help="Microsoft CRT seed (default: 1)")
    mdl_parser.add_argument(
        "--weight", type=float, default=0.4, help="source MDL weight (default: 0.4)"
    )
    observation_parser = commands.add_parser(
        "observation-demo", help="select paired observations, fit supports and adjust gauge"
    )
    observation_parser.add_argument(
        "--seed", type=int, default=1, help="Microsoft CRT seed (default: 1)"
    )
    initialization_parser = commands.add_parser(
        "initialization-demo",
        help="initialize paired rails from raw synthetic points and exercise Kalman components",
    )
    initialization_parser.add_argument(
        "--seed", type=int, default=1, help="Microsoft CRT seed (default: 1)"
    )
    tracking_parser = commands.add_parser(
        "tracking-demo", help="initialize and sequentially trace raw synthetic rail neighborhoods"
    )
    tracking_parser.add_argument(
        "--seed", type=int, default=1, help="Microsoft CRT seed (default: 1)"
    )
    tracking_parser.add_argument(
        "--max-steps", type=int, help="optional diagnostic limit; default runs to a source stop"
    )
    tracking_parser.add_argument(
        "--direction",
        type=int,
        choices=(0, 1),
        default=0,
        help="source observation direction; does not reverse seed geometry",
    )
    tracking_parser.add_argument(
        "--side-rails",
        action="store_true",
        help="include close secondary rails to exercise turnout stacks",
    )
    arguments = parser.parse_args(argv)
    if arguments.command == "view":
        if not np.isfinite(arguments.radius) or not 2 <= arguments.radius <= MAX_CROP_RADIUS:
            parser.error(f"--radius must be between 2 and {MAX_CROP_RADIUS:g} metres")
        if min(arguments.preview_points, arguments.max_tracking_points) <= 0:
            parser.error("viewer point limits must be positive")
        from .viewer import launch_viewer

        try:
            launch_viewer(arguments.path, radius=arguments.radius,
                          preview_points=arguments.preview_points,
                          max_tracking_points=arguments.max_tracking_points,
                          random_seed=arguments.seed)
        except (ImportError, RuntimeError, OSError, ValueError) as error:
            print(f"Viewer: {error}", file=sys.stderr)
            return 1
        return 0
    if arguments.command == "pcl-helpers-demo":
        print(json.dumps(_pcl_helpers_demo(), indent=2, allow_nan=False))
        return 0
    if arguments.command == "conversion-demo":
        print(json.dumps(_conversion_demo(), indent=2, allow_nan=False))
        return 0
    if arguments.command == "convert-cloud":
        if arguments.direction == "utm-to-ecef" and arguments.hemisphere is None:
            parser.error("inverse conversion requires --hemisphere north or south")
        if arguments.direction == "ecef-to-utm" and arguments.hemisphere is not None:
            parser.error("--hemisphere applies only to inverse conversion")
        try:
            loaded = load_point_cloud(arguments.source, format=arguments.format)
            converter = CoordinateConverter(loaded.cloud)
            result = (converter.convert_ecef_to_utm(arguments.zone) if arguments.direction == "ecef-to-utm"
                      else converter.convert_utm_to_ecef(arguments.zone, arguments.hemisphere == "south"))
            exported = write_converted_points(arguments.destination, converter.converted_points,
                                              delimiter="," if arguments.delimiter == "comma" else " ",
                                              overwrite=arguments.overwrite)
        except (OSError, ValueError, ImportError) as error:
            print(json.dumps({"status": getattr(error, "status", "conversion_failed"), "error": str(error)}, allow_nan=False))
            return 1
        print(json.dumps({"status": result.status, "direction": result.direction,
                          "input_count": loaded.input_count, "converted_count": len(result.points),
                          "zone_argument": result.zone_argument, "used_zones": sorted(set(result.zones)),
                          "south_hemi": result.south_hemi, "output": arguments.destination,
                          "export_status": exported.status}, indent=2, allow_nan=False))
        return 0 if result.reference_comparable else 1
    if arguments.command == "legacy-io-demo":
        print(json.dumps(_legacy_io_demo(), indent=2, allow_nan=False))
        return 0
    if arguments.command == "inspect-cloud":
        try:
            loaded = load_point_cloud(arguments.path, format=arguments.format, chunk_size=arguments.chunk_size)
        except (OSError, ValueError, ImportError) as error:
            print(json.dumps({"status": getattr(error, "status", "load_failed"), "error": str(error)}, allow_nan=False))
            return 1
        print(json.dumps(_loaded_cloud_json(loaded), indent=2, allow_nan=False))
        return 0
    if arguments.command == "loop-cross-demo":
        print(json.dumps(_loop_cross_demo(arguments.seed), indent=2, allow_nan=False))
        return 0
    if arguments.command == "header-demo":
        print(json.dumps(_header_demo(), indent=2, allow_nan=False))
        return 0
    if arguments.command == "utilities-demo":
        print(json.dumps(_utilities_demo(), indent=2, allow_nan=False))
        return 0
    if arguments.command == "source-audit":
        errors = check_source_coverage(arguments.repo)
        print(json.dumps({"status": "ok" if not errors else "drift", "errors": errors,
                          **coverage_summary()}, indent=2, allow_nan=False))
        return 1 if errors else 0
    if arguments.command == "reconstruction-audit":
        errors = check_source_coverage(arguments.repo)
        print(json.dumps({"status": "ok" if not errors else "drift", "errors": errors,
                          **reconstruction_summary(), "source_coverage": coverage_summary()},
                         indent=2, allow_nan=False))
        return 1 if errors else 0
    if arguments.command == "pipeline-demo":
        if arguments.max_steps is not None and arguments.max_steps < 0:
            parser.error("--max-steps must be nonnegative")
        scene = pipeline_scene(arguments.scenario)
        options = PipelineOptions(
            bidirectional=scene.bidirectional if arguments.bidirectional is None else arguments.bidirectional,
            grades=not arguments.skip_grades, gwad=not arguments.skip_grades,
            max_steps=arguments.max_steps, random_seed=arguments.seed,
            record_random=arguments.record_random, coordinate_units="m", crs="synthetic local coordinates")
        result = run_pipeline(scene.cloud, scene.seed, config=scene.config, options=options,
                              supplied_toes=scene.supplied_toes, supplied_stacks=scene.supplied_stacks)
        context = {"synthetic_scenario": {"name": arguments.scenario, "notes": scene.notes}}
        output = pipeline_summary(result, include_steps=arguments.steps, context=context)
        if arguments.output is not None:
            try:
                output["manifest"] = str(write_pipeline_outputs(result, arguments.output,
                                                                 include_steps=arguments.steps, context=context))
            except OSError as error:
                parser.error(str(error))
        print(json.dumps(output, indent=2, allow_nan=False))
        return 0 if result.completed else 2 if result.status == "step_limit" else 1
    if arguments.command == "grade-demo":
        result = _grade_demo()
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result["status"] == "ok" else 1
    if arguments.command == "switch-box-demo":
        result = _switch_box_demo(arguments.seed)
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result["status"] == "completed" and result["detected_count"] == 2 else 1
    if arguments.command in ("branch-demo", "bidirectional-demo"):
        if arguments.max_steps is not None and arguments.max_steps < 0:
            parser.error("--max-steps must be nonnegative")
        result = _orchestration_demo(
            arguments.seed, arguments.command == "bidirectional-demo", arguments.max_steps
        )
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result["status"] in {"completed", "step_limit"} else 1
    if arguments.command == "toe-demo":
        result = _toe_demo(arguments.seed, arguments.type)
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result["status"] == "ok" else 1
    if arguments.command == "recognition-demo":
        print(json.dumps(_recognition_demo(), indent=2, allow_nan=False))
        return 0
    if arguments.command == "tracking-demo":
        if arguments.max_steps is not None and arguments.max_steps < 0:
            parser.error("--max-steps must be nonnegative")
        result = _tracking_demo(
            arguments.seed, arguments.max_steps, arguments.direction, arguments.side_rails
        )
        print(json.dumps(result, indent=2, allow_nan=False))
        return (
            0
            if result["status"]
            in {
                "insufficient_window_points",
                "insufficient_head_points",
                "gauge_difference",
                "trajectory_end",
                "step_limit",
            }
            else 1
        )
    if arguments.command == "initialization-demo":
        result = _initialization_demo(arguments.seed)
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result["status"] == "ok" else 1
    if arguments.command == "observation-demo":
        result = _observation_demo(arguments.seed)
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result["status"] == "ok" else 1
    if arguments.command == "mdl-demo":
        if not isfinite(arguments.weight):
            parser.error("--weight must be finite for JSON output")
        cloud = parallel_head_cloud()
        random = MsvcRand(arguments.seed)
        result = segment_track_points_mdl(
            cloud.select(),
            mode=1,
            angle=45,
            config=ExtractionConfig(mdl_weight=arguments.weight),
            rng=random,
        )
        print(
            json.dumps(
                {
                    "stage": "line fitting, source grouping and MDL selection",
                    "status": result.status,
                    "input_points": len(cloud),
                    "selected_count": result.selected_count,
                    "hypotheses": [asdict(hypothesis) for hypothesis in result.hypotheses],
                    "selected_lines": [
                        {
                            "begin": line.begin.as_array().tolist(),
                            "end": line.end.as_array().tolist(),
                            "member_count": len(line.members) if line.members is not None else 0,
                        }
                        for line in result.lines
                    ],
                    "source_uninitialized_candidates": result.uninitialized_candidates,
                    "reference_comparable": result.reference_comparable,
                    "visited_segment_points": int(np.count_nonzero(cloud.segment_ids == 0)),
                    "random_seed": arguments.seed,
                    "random_draws": random.draw_count,
                    "tracking_implemented": False,
                },
                indent=2,
                allow_nan=False,
            )
        )
        return 0 if result.success else 1
    scene = straight_scene()
    segment = seed_windows(scene.seed, scene.config)
    grid = GridIndex(scene.cloud, scene.config.grid_size)
    first, second = grid.assign_segment_members(segment)
    if arguments.command == "heads-demo":
        random = MsvcRand(arguments.seed)
        previous = (scene.seed.point.z, scene.seed.point.z)
        result = extract_track_points(segment, previous, rng=random)
        print(
            json.dumps(
                {
                    "stage": "height GMM and rail-head extraction",
                    "status": result.status,
                    "input_points": len(scene.cloud),
                    "window_point_counts": [len(first), len(second)],
                    "head_point_counts": [len(points) for points in result.heads],
                    "previous_heights": previous,
                    "next_heights": result.next_heights,
                    "class_3_points": int(np.count_nonzero(scene.cloud.class_codes == 3)),
                    "gmm": [
                        {
                            "iterations": window.classification.fit.iterations,
                            "stop_reasons": window.classification.fit.stop_reasons,
                            "parameters": [asdict(p) for p in window.classification.fit.parameters],
                            "cluster_counts": [
                                len(points) for points in window.classification.clustering.clusters
                            ],
                        }
                        for window in result.windows
                        if window.classification is not None
                    ],
                    "random_seed": arguments.seed,
                    "random_draws": random.draw_count,
                    "tracking_implemented": False,
                },
                indent=2,
                allow_nan=False,
            )
        )
        return 0 if result.success else 1
    print(
        json.dumps(
            {
                "stage": "foundation: window construction and spatial membership",
                "input_points": len(scene.cloud),
                "grid_shape": [grid.num_x, grid.num_y],
                "occupied_cells": grid.occupied_cells,
                "window_point_counts": [len(first), len(second)],
                "window_vertices": [
                    [point.as_array().tolist() for point in window.vertices]
                    for window in (segment.track1, segment.track2)
                ],
                "coordinate_dtype": str(scene.cloud.xyz.dtype),
                "tracking_implemented": False,
            },
            indent=2,
        )
    )
    return 0
