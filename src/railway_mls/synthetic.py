"""Small deterministic scenes for exercising the foundation, not accuracy claims."""

from dataclasses import dataclass

import numpy as np

from .config import ExtractionConfig
from .models import InitialLocation
from .points import Point3D, PointCloud


@dataclass(frozen=True)
class SyntheticScene:
    cloud: PointCloud
    seed: InitialLocation
    config: ExtractionConfig


def straight_scene(config: ExtractionConfig | None = None) -> SyntheticScene:
    """Two parallel rail neighborhoods with ground/web/head heights and clutter.

    Points have default source labels. Heights are generated explicitly; no rail
    classification, GMM, or tracking is performed by this generator.
    """
    config = config if config is not None else ExtractionConfig()
    center = Point3D(1000, 2000, 0.2)
    points = []
    for rail_y in (center.y, center.y + config.track_gauge):
        for along in np.linspace(-2, 2, 81):
            for across in (-0.08, 0.0, 0.08):
                for height in (0.0, 0.1, 0.2):
                    points.append((center.x + along, rail_y + across, height))
    points.extend(((center.x, center.y - 2, 1), (center.x, center.y + 4, 1)))
    cloud = PointCloud(points, intensity=np.full(len(points), 100, dtype=np.int32))
    return SyntheticScene(cloud, InitialLocation(center, angle=0, mode=1), config)
