"""Command-line examples for implemented Python stages."""

import argparse
from dataclasses import asdict
import json

import numpy as np

from .heads import extract_track_points
from .models import seed_windows
from .sampling import MsvcRand
from .spatial import GridIndex
from .synthetic import straight_scene


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
    arguments = parser.parse_args(argv)
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
