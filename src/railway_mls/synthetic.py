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


def parallel_head_cloud() -> PointCloud:
    """270 oblique head candidates in three rails (100/90/80), for MDL alone.

    Distinct heights make source sort comparisons unambiguous. This fixture enters
    at the local head-candidate stage; it does not perform GMM or full tracing.
    """
    points = []
    for rail, count in enumerate((100, 90, 80)):
        for position in range(count):
            x = position * 0.01
            points.append((x, x + rail * 0.3, 0.2 + len(points) * 1e-6))
    return PointCloud(points)


def paired_head_cloud(config: ExtractionConfig | None = None) -> PointCloud:
    """Two 270-point oblique candidate neighborhoods for observation/gauge demos.

    Each has three rails (100/90/80 points). Small deterministic transverse noise
    exercises the weighted gauge arithmetic without its uniform-error NaNs. This
    enters after head extraction; it is not a raw-cloud tracking scene.
    """
    settings = config if config is not None else ExtractionConfig()
    first = parallel_head_cloud().xyz.copy()
    first[:, 1] += 0.004 * np.sin(np.arange(len(first)) * 0.71)
    shift = settings.track_gauge / np.sqrt(2)
    second = first + np.array([-shift, shift, 0.01])
    return PointCloud(np.vstack((first, second)))


def initialization_scene(
    *,
    angle: float = 35.0,
    mode: int = 1,
    config: ExtractionConfig | None = None,
) -> SyntheticScene:
    """Oblique raw ground/web/head neighborhoods with unique heights for parity.

    Points are supplied for initialization, without preassigned classes or heads.
    Unique heights avoid native std::sort's unspecified equal-key permutation.
    """
    settings = config if config is not None else ExtractionConfig()
    seed = InitialLocation(Point3D(1000, 2000, 0.2), angle, mode)
    radians = angle * np.pi / 180
    dx, dy = abs(np.cos(radians)), abs(np.sin(radians))
    sx, sy = (-1 if mode in (2, 3) else 1), (1 if mode in (1, 2) else -1)
    along = np.array([sx * dx, sy * dy])
    across = np.array([-sy * dy, sx * dx])
    points = []
    for rail in range(2):
        center = seed.point.as_array()[:2] + rail * settings.track_gauge * across
        for position in np.linspace(-1.5, 1.5, 61):
            for lateral in (-0.08, 0.0, 0.08):
                xy = center + position * along + lateral * across
                for height in (0.0, 0.1, 0.2):
                    points.append((*xy, height + len(points) * 1e-7))
    return SyntheticScene(PointCloud(points), seed, settings)
