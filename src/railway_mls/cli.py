"""Command-line examples for implemented Python stages."""

import argparse
from dataclasses import asdict
import json
from math import isfinite

import numpy as np

from .heads import extract_track_points
from .config import ExtractionConfig
from .fitting import fit_line_ransac
from .gauge import adjust_two_lines_with_observation
from .geometry import Line3D, direction_mode
from .initialization import initial_kalman_states, initialize_tracks
from .kalman import kalman_predict, kalman_update
from .mdl import segment_track_points_mdl
from .models import BinaryString, seed_windows
from .orchestration import TrackingEngine
from .switch_boxes import detect_switch_boxes
from .grades import build_gwad_rows, compute_grade, format_gwad
from .observations import determine_observation, observation_from_support
from .points import Point3D
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
