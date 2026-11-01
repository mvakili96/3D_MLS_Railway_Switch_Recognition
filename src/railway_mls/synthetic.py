"""Small deterministic scenes for exercising the foundation, not accuracy claims."""

from dataclasses import dataclass

import numpy as np

from .config import ExtractionConfig
from .models import BinaryString, InitialLocation, ToePoint, TrackSegment, TrackStack, TrackWindow
from .geometry import Line3D
from .points import Point3D, PointCloud


@dataclass(frozen=True)
class SyntheticScene:
    cloud: PointCloud
    seed: InitialLocation
    config: ExtractionConfig


@dataclass(frozen=True)
class SyntheticToeScene:
    cloud: PointCloud
    stack: TrackStack
    config: ExtractionConfig


@dataclass(frozen=True)
class SyntheticBranchScene:
    cloud: PointCloud
    seed: InitialLocation
    config: ExtractionConfig
    toe: ToePoint


@dataclass(frozen=True)
class SyntheticSwitchBoxScene:
    cloud: PointCloud
    toes: tuple[ToePoint, ...]
    config: ExtractionConfig


def switch_box_scene(
    *,
    angle: float = 35.0,
    points_per_side: int = 34,
    slope: float = 0.0,
    config: ExtractionConfig | None = None,
) -> SyntheticSwitchBoxScene:
    """Dense planar patches beside an explicit toe, with height-band distractors."""
    from .switch_boxes import switch_box_candidates

    settings = config if config is not None else ExtractionConfig()
    radians = angle * np.pi / 180
    across = Point3D(-np.sin(radians), np.cos(radians), 0)
    right = Point3D(1000, 2000, 0.2)
    left = right + across * settings.track_gauge
    toe = ToePoint((left + right) / 2, left, right, track_type=0, angle=angle, mode=1)
    points = []
    for candidate in switch_box_candidates([toe]).candidates:
        center = candidate.region.center
        reference = candidate.region.reference_height
        for u in np.linspace(-0.3, 0.3, points_per_side):
            for v in np.linspace(-0.3, 0.3, points_per_side):
                points.append((center.x + u, center.y + v,
                               reference + 0.05 + slope * u + 0.0002 * np.sin(len(points) * 0.71)))
        points.extend([(center.x, center.y, reference), (center.x, center.y, reference + 0.10)])
    cloud = PointCloud(points, class_codes=[9] * len(points), segment_ids=[7] * len(points))
    return SyntheticSwitchBoxScene(cloud, (toe,), settings)


def branch_scene(config: ExtractionConfig | None = None) -> SyntheticBranchScene:
    """Two raw rail corridors and one supplied toe fixture for scheduling.

    This isolates branch orchestration from undefined automatic toe references;
    the corridors are separated, not a simulated recognized physical junction.
    """
    main = tracking_scene(length=6.0, config=config)
    branch = tracking_scene(length=5.0, angle=50.0, config=main.config)
    offset = Point3D(15.0, 10.0, 0.0)
    first = branch.seed.point + offset
    radians = branch.seed.angle * np.pi / 180
    second = first + Point3D(-np.sin(radians), np.cos(radians), 0) * main.config.track_gauge
    toe = ToePoint(
        center=(first + second) / 2, left=second, right=first,
        track_type=0, angle=branch.seed.angle, mode=branch.seed.mode,
    )
    cloud = PointCloud(np.vstack((main.cloud.xyz, branch.cloud.xyz + offset.as_array())))
    return SyntheticBranchScene(cloud, main.seed, main.config, toe)


def bidirectional_scene(config: ExtractionConfig | None = None) -> SyntheticScene:
    """Raw rail corridor with a seed in its interior for both travel directions."""
    scene = tracking_scene(length=12.0, config=config)
    radians = scene.seed.angle * np.pi / 180
    seed = InitialLocation(
        scene.seed.point + Point3D(np.cos(radians), np.sin(radians), 0) * 6,
        scene.seed.angle, scene.seed.mode,
    )
    return SyntheticScene(scene.cloud, seed, scene.config)


def toe_scene(
    type_id: int = 0,
    *,
    angle: float = 35.0,
    mode: int = 1,
    initialized_references: bool = True,
    config: ExtractionConfig | None = None,
) -> SyntheticToeScene:
    """Side-point/ref-line toe fixture, not an automatically tracked stack.

    Four distinct 60-point banks cover start/end and track2-right/track1-left.
    References can be omitted to exercise the source's uninitialized-stack path.
    """
    from .recognition import define_templates

    settings = config if config is not None else ExtractionConfig()
    origin = np.array([1000.0, 2000.0])
    radians = angle * np.pi / 180
    sx, sy = (-1 if mode in (2, 3) else 1), (1 if mode in (1, 2) else -1)
    along = np.array([sx * abs(np.cos(radians)), sy * abs(np.sin(radians))])
    across = np.array([-along[1], along[0]])
    points = []
    for start, lateral in ((0.0, 1.7), (0.0, -0.2), (4.0, 1.7), (4.0, -0.2)):
        for i in range(60):
            xy = origin + (start + 0.01 * (i + 1)) * along + lateral * across
            points.append((*xy, 10.0 + len(points) * 0.001))
    cloud = PointCloud(points, class_codes=[9] * len(points), segment_ids=[7] * len(points))
    empty = cloud.select([])
    segments = []
    for i in range(6):
        windows = []
        for rail, (low_z, high_z) in enumerate(
            ((0.2 + i * 0.01, 0.3 + i * 0.01), (0.4 + i * 0.02, 0.6 + i * 0.02))
        ):
            center = origin + i * along + rail * settings.track_gauge * across
            line = Line3D(
                Point3D(*(center - 0.5 * along), low_z), Point3D(*(center + 0.5 * along), high_z)
            )
            line.update_center()
            windows.append(
                TrackWindow(
                    line.center,
                    line.angle,
                    members=empty,
                    left_members=empty,
                    right_members=empty,
                    member_line=line if initialized_references else None,
                    turnout=True,
                )
            )
        segments.append(TrackSegment(*windows, gauge=settings.track_gauge))
    for segment_index, begin in ((0, 0), (1, 30), (4, 120), (5, 150)):
        segments[segment_index].track2.right_members = cloud.select(np.arange(begin, begin + 30))
        segments[segment_index].track1.left_members = cloud.select(
            np.arange(begin + 60, begin + 90)
        )
    catalogue = define_templates().all
    binary = BinaryString(
        type=type_id, score=1.0, bits=catalogue[type_id].bits if 0 <= type_id < 20 else ()
    )
    binary.set_labels(type_id)
    if binary.bits:
        for row, destination in enumerate((0, 1, 4, 5)):
            segment = segments[destination]
            (
                segment.track2.left_tag,
                segment.track2.right_tag,
                segment.track1.left_tag,
                segment.track1.right_tag,
            ) = binary.bits[row * 4 : row * 4 + 4]
    return SyntheticToeScene(cloud, TrackStack(segments, binary), settings)


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
