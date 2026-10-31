"""Small deterministic scenes for exercising the foundation, not accuracy claims."""

from dataclasses import dataclass

import numpy as np

from .config import ExtractionConfig
from .models import InitialLocation, TrackSegment, TrackStack, TrackWindow
from .points import Point3D, PointCloud


@dataclass(frozen=True)
class SyntheticScene:
    cloud: PointCloud
    seed: InitialLocation
    config: ExtractionConfig


def signature_stack(bits, segment_count: int = 6) -> TrackStack:
    """Construct a tag-only recognition fixture, not a raw-cloud tracking result.

    The four rows go into first/second/penultimate/last segments. Interior tags
    are zero and are ignored by source signature extraction.
    """
    from operator import index

    values = tuple(index(bit) for bit in bits)
    count = index(segment_count)
    if len(values) != 16 or count <= 4:
        raise ValueError("signature fixtures require 16 integers and at least five segments")
    rows = [[0] * 4 for _ in range(count)]
    for source, destination in enumerate((0, 1, count - 2, count - 1)):
        rows[destination] = values[source * 4 : source * 4 + 4]
    segments = []
    for i, (left2, right2, left1, right1) in enumerate(rows):
        segments.append(
            TrackSegment(
                TrackWindow(Point3D(i, 0, 0.2), 0, turnout=True, left_tag=left1, right_tag=right1),
                TrackWindow(
                    Point3D(i, 1.5, 0.2), 0, turnout=True, left_tag=left2, right_tag=right2
                ),
            )
        )
    return TrackStack(segments)


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


def tracking_scene(
    *,
    length: float = 8.0,
    angle: float = 35.0,
    mode: int = 1,
    side_rails: bool = False,
    gaps: tuple[tuple[float, float], ...] = (),
    curvature: float = 0.0,
    grade: float = 0.0,
    config: ExtractionConfig | None = None,
) -> SyntheticScene:
    """Raw noisy rail neighborhoods for sequential tracing, with optional gaps.

    Ground/web/head heights and small transverse noise are generated directly.
    Optional close secondary rails exercise fully initialized three-line MDL
    comparisons; the default is a single rail per neighborhood.
    """
    settings = config if config is not None else ExtractionConfig()
    seed = InitialLocation(Point3D(1000, 2000, 0.2), angle, mode)
    radians = angle * np.pi / 180
    sx, sy = (-1 if mode in (2, 3) else 1), (1 if mode in (1, 2) else -1)
    along = np.array([sx * abs(np.cos(radians)), sy * abs(np.sin(radians))])
    across = np.array([-along[1], along[0]])
    points = []
    offsets = ((0.0, 200), (-0.14, 120), (0.14, 110)) if side_rails else ((0.0, 120),)
    for rail in range(2):
        for offset, density in offsets:
            for position in np.arange(-0.6, length, 1 / density):
                if any(begin <= position <= end for begin, end in gaps):
                    continue
                lateral = offset + 0.003 * np.sin(position * 31 + rail * 0.7)
                if curvature == 0:
                    center_offset, normal = position * along, across
                else:
                    turn = curvature * position
                    center_offset = (
                        np.sin(turn) / curvature * along + (1 - np.cos(turn)) / curvature * across
                    )
                    normal = -np.sin(turn) * along + np.cos(turn) * across
                xy = (
                    seed.point.as_array()[:2]
                    + center_offset
                    + (rail * settings.track_gauge + lateral) * normal
                )
                for height in (0.0, 0.1, 0.2):
                    points.append((*xy, height + grade * position + len(points) * 1e-8))
    return SyntheticScene(PointCloud(points), seed, settings)
