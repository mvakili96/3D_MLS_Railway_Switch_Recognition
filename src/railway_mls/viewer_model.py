"""Bounded LAS loading and resumable main/branch tracing, independent of the GUI.

Rendering may use fewer points; inference uses the source reader's zero-based
even records in file order inside the explicitly selected XY crop. A reported
few-ULP header-boundary omission preserves the strict computational grid bounds.
The retained world coordinates and full-file grid origin are unchanged.
"""

from __future__ import annotations

from copy import copy
from dataclasses import dataclass, replace
from math import ceil, isfinite
from pathlib import Path
from typing import Callable

import numpy as np

from .config import ExtractionConfig
from .models import InitialLocation, TrackLayer
from .orchestration import (
    SKIPPED_BRANCH_TYPES, LayerToeTag, TrackingEngine, VectorTag,
    initial_location_dialog_defaults, tag_rail_vector,
)
from .points import Point3D, PointCloud
from .sampling import MsvcRand
from .spatial import Bounds3D, GridIndex
from .tracking import SequentialTracker


VANCOUVER_SEED = InitialLocation(Point3D(513194.41, 5458237.85, 6.61), 355.0, 4)
DEFAULT_LAS = Path("Vancouver/Thales_11_20Las/16.las")
DEFAULT_CROP_RADIUS = 30.0
MAX_CROP_RADIUS = 30.0
DEFAULT_TRACKING_POINT_LIMIT = 30_000_000
Progress = Callable[[str], None]


class ViewerDataError(ValueError):
    pass


def _laspy():
    try:
        import laspy
    except ImportError as error:
        raise ViewerDataError('Install the viewer extras: python -m pip install -e ".[viewer]"') from error
    return laspy


@dataclass(frozen=True)
class LasInfo:
    path: Path
    point_count: int
    bounds: Bounds3D

    @property
    def origin(self):
        return (self.bounds.minimum.as_array() + self.bounds.maximum.as_array()) / 2

    def contains(self, point: Point3D):
        return bool(np.all(point.as_array() >= self.bounds.minimum.as_array())
                    and np.all(point.as_array() <= self.bounds.maximum.as_array()))


@dataclass(frozen=True)
class CloudPreview:
    info: LasInfo
    xyz: np.ndarray
    stride: int


@dataclass(frozen=True)
class TrackingCrop:
    info: LasInfo
    cloud: PointCloud
    center: Point3D
    radius: float
    omitted_header_roundoff: int = 0


def _info(path, header):
    if not header.point_count:
        raise ViewerDataError("This LAS file is empty.")
    return LasInfo(Path(path).resolve(), int(header.point_count),
                   Bounds3D(Point3D(*header.mins), Point3D(*header.maxs)))


def _check_cancel(cancel):
    if cancel is not None and cancel():
        raise ViewerDataError("Loading cancelled.")


def load_preview(path, *, max_points=150_000, chunk_size=250_000,
                 progress: Progress | None = None, cancel=None) -> CloudPreview:
    """Stream a deterministic display sample; never load the full file as XYZ."""
    if max_points <= 0 or chunk_size <= 0:
        raise ValueError("point and chunk limits must be positive")
    with _laspy().open(path) as stream:
        info = _info(path, stream.header)
        stride = 2 * max(1, ceil(info.point_count / (2 * max_points)))
        groups, first = [], 0
        for points in stream.chunk_iterator(chunk_size):
            _check_cancel(cancel)
            start = (-first) % stride
            chosen = points[start::stride]
            if len(chosen):
                groups.append(np.column_stack((chosen.x, chosen.y, chosen.z)))
            first += len(points)
            if progress:
                progress(f"Reading preview… {first / info.point_count:.0%}")
    xyz = np.concatenate(groups)
    if not np.isfinite(xyz).all():
        raise ViewerDataError("The display sample contains nonfinite coordinates.")
    xyz.flags.writeable = False
    return CloudPreview(info, xyz, stride)


def load_tracking_crop(path, center: Point3D, *, radius=DEFAULT_CROP_RADIUS,
                       max_points=DEFAULT_TRACKING_POINT_LIMIT,
                       chunk_size=250_000, progress: Progress | None = None,
                       cancel=None) -> TrackingCrop:
    """Read all source-selected records in a crop; fail rather than thin inference."""
    if not isfinite(radius) or radius <= 0 or max_points <= 0 or chunk_size <= 0:
        raise ValueError("crop radius and point/chunk limits must be positive")
    if not np.isfinite(center.as_array()).all():
        raise ViewerDataError("Choose a finite starting position.")
    xyz, intensity, first, kept, omitted = [], [], 0, 0, 0
    with _laspy().open(path) as stream:
        info = _info(path, stream.header)
        if not info.contains(center):
            raise ViewerDataError("The starting position is outside this file's bounds.")
        for points in stream.chunk_iterator(chunk_size):
            _check_cancel(cancel)
            # Preserve parity across odd-sized chunks and filter AFTER source sampling.
            even = points[first % 2::2]
            inside = ((np.abs(np.asarray(even.x) - center.x) <= radius)
                      & (np.abs(np.asarray(even.y) - center.y) <= radius))
            chosen = even[inside]
            if len(chosen):
                values = np.column_stack((chosen.x, chosen.y, chosen.z))
                if not np.isfinite(values).all():
                    raise ViewerDataError("The tracking region contains nonfinite coordinates.")
                low, high = info.bounds.minimum.as_array(), info.bounds.maximum.as_array()
                valid = np.all((values >= low) & (values <= high), axis=1)
                if not np.all(valid):
                    tolerance = 4 * np.spacing(np.maximum(1, np.maximum(np.abs(low), np.abs(high))))
                    if np.any(values < low - tolerance) or np.any(values > high + tolerance):
                        raise ViewerDataError(
                            "Decoded LAS records exceed the declared file bounds. "
                            "Check the file's metadata before tracking this region."
                        )
                    # Keep the header grid origin and every retained XYZ unchanged.
                    # Only a few-ULP header/decode boundary discrepancies are omitted.
                    omitted += int(np.count_nonzero(~valid))
                    chosen = chosen[valid]
                    values = values[valid]
            kept += len(chosen)
            if kept > max_points:
                raise ViewerDataError(
                    f"This region exceeds {max_points:,} tracking points. "
                    "Reduce the crop radius, or raise --max-tracking-points."
                )
            if len(chosen):
                xyz.append(values)
                intensity.append(np.asarray(chosen.intensity).copy())
            first += len(points)
            if progress:
                progress(f"Loading tracking region… {first / info.point_count:.0%}")
    if not kept:
        raise ViewerDataError("There are no source-selected points around that position.")
    return TrackingCrop(info, PointCloud(np.concatenate(xyz), np.concatenate(intensity)),
                        center, radius, omitted)


def _xyz(points):
    return np.array([point.as_array() for point in points], dtype=np.float64).reshape(-1, 3)


def search_boundary(crop: TrackingCrop, vertices: np.ndarray) -> str | None:
    """Whether search geometry exceeds artificial crop edges or LAS header bounds.

    This describes available coverage; it does not override the source stop code
    or prove that points exist just beyond the boundary.
    """
    if not len(vertices) or not np.isfinite(vertices[:, :2]).all():
        return None
    low = crop.center.as_array()[:2] - crop.radius
    high = crop.center.as_array()[:2] + crop.radius
    file_low = crop.info.bounds.minimum.as_array()[:2]
    file_high = crop.info.bounds.maximum.as_array()[:2]
    if (np.any((vertices[:, :2] < low - 1e-8) & (low > file_low + 1e-8))
            or np.any((vertices[:, :2] > high + 1e-8) & (high < file_high - 1e-8))):
        return "crop"
    if np.any(vertices[:, :2] < file_low) or np.any(vertices[:, :2] > file_high):
        return "file"
    return None


@dataclass(frozen=True)
class ViewerPath:
    name: str
    paths: tuple[np.ndarray, ...]
    accepted_steps: int
    distance: float
    status: str


@dataclass(frozen=True)
class ViewerFrame:
    status: str
    phase: str
    accepted_steps: int
    windows: tuple[np.ndarray, ...]
    paths: tuple[np.ndarray, ...]
    pose: InitialLocation | None
    distance: float
    notes: tuple[str, ...] = ()
    boundary: str | None = None
    traces: tuple[ViewerPath, ...] = ()
    active_path: str = "Main path"
    total_steps: int = 0


class ViewerSession:
    """One retained tracker per path and shared RNG across the complete session.

    Picking previews initialization, then confirmation performs the native second
    initialization on the SAME random stream. Reset explicitly starts a new run.
    Full-file bounds keep the source grid origin even though inference is cropped.
    Optional branch tracing supplies missing stack references from the same
    accepted segment's fitted rails. This explicit demo policy does not change
    the strict core. Native branch type gates, order and shared RNG are retained.
    Bidirectional UI scheduling remains separate.
    """

    def __init__(self, crop: TrackingCrop, *, config=None, random_seed=1, trace_branches=True):
        self.crop = crop
        self.config = config or ExtractionConfig()
        self.random_seed = random_seed
        self.trace_branches = bool(trace_branches)
        self.grid = GridIndex(crop.cloud, self.config.grid_size, bounds=crop.info.bounds)
        self.reset()

    def reset(self):
        self.crop.cloud.class_codes.fill(0)
        self.crop.cloud.segment_ids.fill(-1)
        self.engine = TrackingEngine(self.grid, config=self.config, rng=MsvcRand(self.random_seed))
        self.tracker = None
        self.trackers = []
        self.branch_attempts = []
        self.tags = []
        self.recovered_references = 0
        self._toes = []
        self._toe_cursor = 0
        self._branch_toe = None
        self._scheduling = False
        self._last_stop = "ready"
        self.initialized = None
        self.preview_seed = self.confirmed_seed = None
        self.phase = "empty"

    def preview(self, pose: InitialLocation):
        if self.phase != "empty":
            raise ViewerDataError("Reset before previewing a different seed.")
        self.initialized = self.engine.initialize(pose)
        self.preview_seed = pose
        self.phase = "preview" if self.initialized.success else "failed"
        return self.frame(self.initialized.status)

    def confirm(self, pose: InitialLocation):
        if self.phase != "preview":
            raise ViewerDataError("Preview a valid seed before confirming it.")
        point = pose.point
        if (abs(point.x - self.crop.center.x) > self.crop.radius
                or abs(point.y - self.crop.center.y) > self.crop.radius):
            raise ViewerDataError("The edited seed is outside the loaded region. Reset and pick again.")
        self.initialized = self.engine.initialize(pose)
        self.confirmed_seed = pose
        self.phase = "ready" if self.initialized.success else "failed"
        if self.initialized.success:
            self._start_tracker(main_track=True)
        return self.frame(self.initialized.status)

    def _start_tracker(self, *, main_track):
        self.tracker = SequentialTracker(
            self.engine.current_layer, self.grid, self.initialized.initial_heights,
            config=self.config, rng=self.engine.rng,
            track_direction=1 if main_track else 0, main_track=main_track,
            completed_stacks=self.engine.completed_stacks, toe_points=self.engine.toe_points,
            switch_boxes=self.engine.switch_boxes,
        )
        self.trackers.append(self.tracker)

    def _save_switch_references(self):
        """Viewer-only recovery of LIM-001; never fit again or consume extra draws.

        The core copied the turnout snapshot before writing member lines. Use
        the fitted lines of that same accepted step, before any toe postpass runs.
        Rejected segments and the native uninitialized stack distance are untouched.
        """
        step = self.tracker.steps[-1]
        stack = self.tracker.active_stack.track_segments
        if not step.accepted or step.status != "accepted" or not stack:
            return
        changed = False
        for saved, accepted in zip((stack[-1].track1, stack[-1].track2),
                                   (step.candidate.track1, step.candidate.track2)):
            line = accepted.member_line
            if (saved.member_line is None and line is not None and np.isfinite(
                [*line.begin.as_array(), *line.end.as_array(), line.a, line.b, line.c]
            ).all()):
                saved.member_line = copy(line)
                changed = True
        self.recovered_references += int(changed)

    def step(self):
        if self.phase not in ("ready", "tracking") or self.tracker is None:
            raise ViewerDataError("Confirm the starting windows before tracking.")
        if self._scheduling:
            return self._next_branch()
        result = self.tracker.run(max_steps=1)
        self.engine.distance_from_start = result.distance_from_start
        if result.steps:
            self.engine._initialization_pending = False
        if self.trace_branches and self.tracker.main_track:
            self._save_switch_references()
        self.phase = "tracking"
        if result.status != "step_limit":
            self._last_stop = result.status
            if not self.trace_branches or self.engine._halt(result.status):
                self.phase = "stopped"
            else:
                if self.tracker.main_track:
                    # Snapshot only after recognition/toes/boxes; used writes stay local.
                    self._toes = [replace(toe, used=False) for toe in self.engine.toe_points]
                else:
                    self._branch_toe.used = True
                    i, _ = self.branch_attempts[-1]
                    self.branch_attempts[-1] = (i, result.status)
                self._scheduling = True
                if self._toe_cursor < len(self._toes):
                    return self.frame("branches_pending")
                return self._finish_paths()
        return self.frame(result.status)

    def _next_branch(self):
        """One resumable scheduling event, following OnRailextractionTrackingRailway."""
        if self._toe_cursor >= len(self._toes):
            return self._finish_paths()
        i = self._toe_cursor
        toe = self._toes[i]
        self._toe_cursor += 1
        # Native allocation occurs before the type gate, including skipped toes.
        self.engine.current_layer = TrackLayer()
        self.engine.layers.append(self.engine.current_layer)
        if toe.track_type in SKIPPED_BRANCH_TYPES:
            self.branch_attempts.append((i, "skipped_type"))
            return self.frame("branch_skipped")
        if toe.right is None or toe.angle is None or toe.mode is None:
            status = "undefined_source_branch_seed"
        else:
            # Preserve the original scheduler's copied Y-as-Z expression.
            seed = InitialLocation(Point3D(toe.right.x, toe.right.y, toe.right.y),
                                   toe.angle, toe.mode)
            status = ("ready" if np.isfinite([*seed.point.as_array(), seed.angle]).all()
                      else "nonfinite_branch_seed")
        if status != "ready":
            if status == "nonfinite_branch_seed":
                self.engine.distance_from_start = 0.0
                self.engine._initialization_pending = True
            self.branch_attempts.append((i, status))
            self.phase = "stopped"
            return self.frame(status)
        self.initialized = self.engine.initialize(seed, reset_layers=False)
        if not self.initialized.success:
            status = (self.initialized.status if self.initialized.source_return is None
                      else "initialization_failed")
            self.branch_attempts.append((i, status))
            if self.engine._halt(status):
                self.phase = "stopped"
            else:
                toe.used = True
            return self.frame(status)
        self._branch_toe = toe
        self.branch_attempts.append((i, "tracking"))
        self._scheduling = False
        self._start_tracker(main_track=False)
        return self.frame("branch_initialized")

    def _finish_paths(self):
        self._scheduling = False
        self.phase = "stopped"
        for n, layer in enumerate(self.engine.layers):
            for i, toe in enumerate(layer.toe_points):
                tag = (tag_rail_vector(layer.center_vector, toe.center) if toe.center is not None
                       else VectorTag("undefined_source_toe_center", reference_comparable=False))
                self.tags.append(LayerToeTag(n, i, tag))
                if tag.status.startswith("undefined_source"):
                    return self.frame(tag.status)
        return self.frame(self._last_stop)

    def frame(self, status="ready"):
        layer = self.tracker.layer if self.tracker else self.engine.current_layer
        segment = self.tracker.current if self.tracker else (
            self.initialized.segment if self.initialized and self.initialized.success else None
        )
        windows = tuple(_xyz(w.vertices) for w in (segment.track1, segment.track2)) if segment else ()
        paths = tuple(_xyz(p) for p in (layer.rail1_vector, layer.rail2_vector,
                                       layer.center_vector)) if layer else ()
        pose = (initial_location_dialog_defaults(self.engine.initial_location)
                if self.engine.initial_location is not None else None)
        notes = ([f"{self.crop.omitted_header_roundoff} LAS header-rounding records omitted."]
                 if self.crop.omitted_header_roundoff else [])
        boundary = None
        if self.recovered_references:
            notes.append("Switch references use saved fitted rails (demo recovery).")
        if (self.phase == "stopped" and self.trace_branches and self.tracker
                and self.tracker.main_track and len(self.tracker.active_stack.track_segments) > 4):
            notes.append("A switch region is incomplete at this boundary.")
        if self.tracker and self.tracker.steps:
            last = self.tracker.steps[-1]
            if (self.phase == "stopped" and last.attempts
                    and status in ("insufficient_window_points", "insufficient_head_points",
                                   "no_support_points")):
                searched = last.attempts[-1].segment
                vertices = np.concatenate([_xyz(w.vertices) for w in
                                           (searched.track1, searched.track2)])
                boundary = search_boundary(self.crop, vertices)
            if any(t.toe_detection is not None and any(
                r.status.startswith("undefined_source") for r in t.toe_detection.entries
            ) for t in self.trackers):
                notes.append("Some source toe geometry is unavailable.")
        if any(not model.reference_comparable for t in self.trackers for step in t.steps
               for model in step.models):
            notes.append("Trace includes documented source MDL limitations.")
        if any(status == "initialization_failed" for _, status in self.branch_attempts):
            notes.append("A switch branch could not be initialized.")
        traces = tuple(ViewerPath(
            "Main path" if i == 0 else f"Branch {i}",
            tuple(_xyz(p) for p in (t.layer.rail1_vector, t.layer.rail2_vector, t.layer.center_vector)),
            sum(s.accepted for s in t.steps), t.distance, t.status,
        ) for i, t in enumerate(self.trackers))
        return ViewerFrame(
            status, self.phase,
            sum(step.accepted for step in self.tracker.steps) if self.tracker else 0,
            windows, paths, pose, self.tracker.distance if self.tracker else 0.0,
            tuple(dict.fromkeys(notes)), boundary, traces,
            traces[-1].name if traces else "Main path", sum(t.accepted_steps for t in traces),
        )
