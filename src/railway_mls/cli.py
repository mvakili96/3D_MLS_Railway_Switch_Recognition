"""Command-line entry point for the current foundation demo."""

import argparse
import json

from .models import seed_windows
from .spatial import GridIndex
from .synthetic import straight_scene


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="3D MLS railway Python foundation")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("demo", help="construct paired windows and select synthetic points")
    parser.parse_args(argv)
    scene = straight_scene()
    segment = seed_windows(scene.seed, scene.config)
    grid = GridIndex(scene.cloud, scene.config.grid_size)
    first, second = grid.assign_segment_members(segment)
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
