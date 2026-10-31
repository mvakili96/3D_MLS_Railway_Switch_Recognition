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
from .models import seed_windows
from .observations import determine_observation, observation_from_support
from .points import Point3D
from .sampling import MsvcRand
from .spatial import GridIndex
from .synthetic import initialization_scene, paired_head_cloud, parallel_head_cloud, straight_scene


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
    arguments = parser.parse_args(argv)
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
