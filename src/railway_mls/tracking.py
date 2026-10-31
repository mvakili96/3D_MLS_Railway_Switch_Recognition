"""Trackingrailway's sequential loop, up to the separate recognition postpass."""

from __future__ import annotations

from copy import copy
from dataclasses import dataclass, replace
from math import isfinite
from operator import index

import numpy as np

from .config import ExtractionConfig
from .fitting import LineFitResult, fit_line_ransac
from .gauge import GaugeAdjustment, adjust_two_lines_with_observation
from .geometry import Line3D, direction_mode, distance_2d, line_parameters
from .heads import TrackHeadResult, extract_track_points
from .initialization import InitializationResult
from .kalman import KalmanPrediction, KalmanState, KalmanUpdate, kalman_predict, kalman_update
from .mdl import MDLResult, segment_track_points_mdl
from .models import TrackLayer, TrackSegment, TrackStack, TrackWindow
from .observations import (
    ObservationSelection,
    SupportObservation,
    determine_observation,
    observation_from_support,
)
from .points import Point3D
from .sampling import MsvcRand, RandSource
from .spatial import GridIndex


def _snapshot(segment: TrackSegment) -> TrackSegment:
    # C++ structs/vectors copy by value; pointed-to cloud rows remain shared.
    windows = [
        replace(window, member_line=copy(window.member_line))
        for window in (segment.track1, segment.track2)
    ]
    return replace(segment, track1=windows[0], track2=windows[1])


def generate_tracking_segment(
    current: TrackSegment,
    predictions: tuple[KalmanPrediction, KalmanPrediction],
    grid: GridIndex,
    *,
    length: float,
    width: float,
    mode: int,
) -> TrackSegment:
    """GenerateTrkSegAssignMemPts, with one shared ordered 5x5 neighborhood."""
    if any(not np.isfinite(prediction.state.point.as_array()).all() for prediction in predictions):
        raise ValueError("nonfinite predicted coordinates cannot be converted to grid indices")
    empty = grid.cloud.select([])
    windows = [
        TrackWindow(
            prediction.state.point, window.orientation, left_members=empty, right_members=empty
        )
        for prediction, window in zip(predictions, (current.track1, current.track2))
    ]
    segment = TrackSegment(*windows)
    for window in windows:
        window.generate_vertices(length, width, mode)
    segment.update_gauge()
    grid.assign_segment_members(segment, neighborhood_size=5)
    return segment


@dataclass(frozen=True)
class WindowAttempt:
    phase: str
    length: float
    predictions: tuple[KalmanPrediction, KalmanPrediction]
    segment: TrackSegment
    extraction: TrackHeadResult | None = None


@dataclass(frozen=True)
class TrackingStep:
    number: int
    mode: int
    current_states: tuple[KalmanState, KalmanState]
    attempts: tuple[WindowAttempt, ...]
    models: tuple[MDLResult, ...]
    selection: ObservationSelection | None
    fits: tuple[LineFitResult, ...]
    observations: tuple[SupportObservation, ...]
    updates: tuple[KalmanUpdate, ...]
    adjustment: GaugeAdjustment | None
    candidate: TrackSegment | None
    accepted: bool
    status: str


@dataclass(frozen=True)
class TrackingResult:
    layer: TrackLayer
    steps: tuple[TrackingStep, ...]
    completed_stacks: tuple[TrackStack, ...]
    active_stack: TrackStack
    distance_from_start: float
    next_heights: tuple[float, float]
    status: str

    @property
    def reference_comparable(self) -> bool:
        """False if a port policy bypassed undefined native intermediate access."""
        return not self.status.startswith(("undefined_source", "nonfinite")) and all(
            model.reference_comparable for step in self.steps for model in step.models
        )

    @property
    def accepted_steps(self) -> int:
        return sum(step.accepted for step in self.steps)


class SequentialTracker:
    """One source tracking invocation, with optional inspectable step limits.

    Reuse this object to continue after run(max_steps=...), preserving the RNG,
    active turnout stack, current heights and first-iteration state. The source's
    current covariances stay identity: its posterior-assignment lines are inactive.
    Recognition, toe detection and GUI callbacks are outside this loop stage.
    """

    def __init__(
        self,
        layer: TrackLayer,
        grid: GridIndex,
        initial_heights: tuple[float, float],
        *,
        config: ExtractionConfig | None = None,
        rng: RandSource | None = None,
        track_direction: int = 0,
        main_track: bool = True,
        distance_from_start: float = 0.0,
        completed_stacks: list[TrackStack] | None = None,
    ) -> None:
        self.config = config if config is not None else ExtractionConfig()
        if self.config.grid_size != grid.grid_size:
            raise ValueError("tracking configuration must match the grid size")
        if len(initial_heights) != 2 or not all(isfinite(value) for value in initial_heights):
            raise ValueError("provide two finite initial rail heights")
        if layer.track_segments:
            last = layer.track_segments[-1]
            if any(window.member_line is None for window in (last.track1, last.track2)):
                raise ValueError("initialize both member lines before sequential tracking")
            if any(
                window.members is not None and window.members.cloud is not grid.cloud
                for window in (last.track1, last.track2)
            ):
                raise ValueError("tracking layer and grid must share one owning cloud")
        self.layer, self.grid = layer, grid
        self.rng = rng if rng is not None else MsvcRand()
        self.track_direction = track_direction
        self.distance = distance_from_start if main_track else 0.0
        self.heights = tuple(initial_heights)
        self.completed_stacks = completed_stacks if completed_stacks is not None else []
        self.active_stack = TrackStack()
        self.steps: list[TrackingStep] = []
        self.status = "ready" if layer.track_segments else "no_segments"
        self.current = _snapshot(layer.track_segments[-1]) if layer.track_segments else None

    def run(self, *, max_steps: int | None = None) -> TrackingResult:
        limit = index(max_steps) if max_steps is not None else None
        if limit is not None and limit < 0:
            raise ValueError("max_steps must be nonnegative")
        remaining = limit
        while self.status == "ready" and (remaining is None or remaining > 0):
            step = self._step()
            self.steps.append(step)
            if remaining is not None:
                remaining -= 1
            if not step.accepted or step.status != "accepted":
                self.status = step.status
        status = "step_limit" if self.status == "ready" else self.status
        return TrackingResult(
            self.layer,
            tuple(self.steps),
            tuple(TrackStack(list(stack.track_segments)) for stack in self.completed_stacks),
            TrackStack(list(self.active_stack.track_segments)),
            self.distance,
            self.heights,
            status,
        )

    def _step(self) -> TrackingStep:
        settings, current = self.config, self.current
        first, second = current.track1, current.track2
        if not self.steps:
            mode = direction_mode(first.member_line.end, first.member_line.begin)
        else:
            mode = direction_mode(
                self.layer.track_segments[-1].track1.center,
                self.layer.track_segments[-2].track1.center,
            )
        states = KalmanState(first.center), KalmanState(second.center)
        attempts, models, fits, observations, updates = [], (), (), (), ()
        selection, adjustment, candidate = None, None, None

        def finish(status: str, accepted: bool = False) -> TrackingStep:
            return TrackingStep(
                len(self.steps) + 1,
                mode,
                states,
                tuple(attempts),
                models,
                selection,
                fits,
                observations,
                updates,
                adjustment,
                candidate,
                accepted,
                status,
            )

        def predict(length: float, phase: str) -> bool:
            nonlocal candidate
            predictions = tuple(
                kalman_predict(state, mode=mode, angle=window.orientation, step_length=length)
                for state, window in zip(states, (first, second))
            )
            if any(
                not np.isfinite(prediction.state.point.as_array()).all()
                for prediction in predictions
            ):
                return False
            candidate = generate_tracking_segment(
                current,
                predictions,
                self.grid,
                length=length,
                width=settings.track_window_width,
                mode=mode,
            )
            attempts.append(WindowAttempt(phase, length, predictions, _snapshot(candidate)))
            return True

        base = settings.track_window_length
        if not predict(base, "initial"):
            return finish("nonfinite_prediction")
        if any(
            len(window.members) < settings.min_points
            for window in (candidate.track1, candidate.track2)
        ):
            for multiplier in range(2, 7):
                if not predict(base + (multiplier - 1) * base, "point_retry"):
                    return finish("nonfinite_prediction")
                if all(
                    len(window.members) > settings.min_points
                    for window in (candidate.track1, candidate.track2)
                ):
                    break
            else:
                return finish("insufficient_window_points")

        extraction = extract_track_points(candidate, self.heights, rng=self.rng)
        attempts[-1] = replace(attempts[-1], extraction=extraction)
        if extraction.next_heights is None:
            return finish("undefined_source_head_extraction")
        if any(len(points) < settings.min_track_points for points in extraction.heads):
            for multiplier in range(2, 7):
                if not predict(base + (multiplier - 1) * base, "head_retry"):
                    return finish("nonfinite_prediction")
                extraction = extract_track_points(
                    candidate, self.heights, rng=self.rng, existing_heads=extraction.heads
                )
                attempts[-1] = replace(attempts[-1], extraction=extraction)
                if extraction.next_heights is None:
                    return finish("undefined_source_head_extraction")
                if all(len(points) > settings.min_track_points for points in extraction.heads):
                    break
            else:
                return finish("insufficient_head_points")
        models = tuple(
            segment_track_points_mdl(
                points, mode, window.orientation, config=settings, rng=self.rng
            )
            for points, window in zip(extraction.heads, (first, second))
        )
        if any(not model.success for model in models):
            return finish("undefined_source_mdl")
        turnout = any(len(model.lines) > 1 for model in models)
        for window, model in zip((candidate.track1, candidate.track2), models):
            window.turnout = len(model.lines) > 1
        if turnout:
            selection = determine_observation(
                models[0].lines,
                models[1].lines,
                first.member_line,
                second.member_line,
                track_mode=self.track_direction,
                stack_size=len(self.active_stack.track_segments),
                line_mode=mode,
                config=settings,
                cloud=self.grid.cloud,
            )
            if selection.status == "unresolved_gauge_tie":
                return finish("undefined_source_observation_tie")
            fit_points = selection.observations
        else:
            fit_points = extraction.heads
        fits = tuple(
            fit_line_ransac(points, settings.max_distance_for_ransac, rng=self.rng)
            for points in fit_points
        )
        if any(not fit.success for fit in fits):
            return finish("undefined_source_observation_fit")
        if turnout:
            empty = self.grid.cloud.select([])
            for rail, (window, fit, previous) in enumerate(
                zip((candidate.track1, candidate.track2), fits, (first, second))
            ):
                fit.line.order_endpoints_by_reference(previous.center)
                window.left_members, window.right_members = (
                    selection.left_points[rail],
                    selection.right_points[rail],
                )
                window.left_tag, window.right_tag = (
                    int(bool(window.left_members)),
                    int(bool(window.right_members)),
                )
                window.members, window.turnout = empty, True
            self.active_stack.track_segments.append(_snapshot(candidate))
        else:
            if len(self.active_stack.track_segments) > 2:
                self.completed_stacks.append(TrackStack(list(self.active_stack.track_segments)))
            self.active_stack.track_segments.clear()
        observations = tuple(observation_from_support(fit.support) for fit in fits)
        if any(observation.point is None for observation in observations):
            return finish("no_support_points")
        predictions = attempts[-1].predictions
        updates = tuple(
            kalman_update(prediction.state, observation.point, fit.sigma)
            for prediction, observation, fit in zip(predictions, observations, fits)
        )
        self.heights = tuple(update.state.point.z for update in updates)
        lines = []
        for state, update, previous in zip(states, updates, (first, second)):
            line = Line3D(state.point, update.state.point)
            line.order_endpoints_by_reference(previous.center)
            lines.append(line)
        adjustment = adjust_two_lines_with_observation(
            *lines,
            observations[0].support,
            observations[1].support,
            references=(first.center, second.center),
            line_mode=mode,
            config=settings,
        )
        for line, previous in zip(lines, (first, second)):
            line.begin = previous.center
            # GetLineParameter ONLY: source keeps angle and cached gauge center.
            line.a, line.b, line.c = line_parameters(line.begin, line.end)
        midpoint_begin, midpoint_end = (
            (lines[0].begin + lines[1].begin) / 2,
            (lines[0].end + lines[1].end) / 2,
        )
        self.layer.rail1_vector.append(lines[0].end)
        self.layer.rail2_vector.append(lines[1].end)
        self.layer.center_vector.append(midpoint_end)
        self.distance += distance_2d(midpoint_begin, midpoint_end)
        for window, line in zip((candidate.track1, candidate.track2), lines):
            window.orientation, window.center, window.member_line = line.angle, line.end, line
            window.generate_vertices(
                base, settings.track_window_width, direction_mode(line.end, line.begin)
            )
        candidate.update_gauge()
        candidate.distance_from_start = self.distance
        if abs(candidate.gauge - settings.track_gauge) > 0.15:
            return finish("gauge_difference")
        if distance_2d(first.center, candidate.track1.center) < 0.3:
            return finish("trajectory_end")
        for window in (candidate.track1, candidate.track2):
            window.vertices = tuple(
                Point3D(point.x, point.y, window.member_line.begin.z) for point in window.vertices
            )
            boundaries = (
                self.layer.turnout_boundaries if window.turnout else self.layer.updated_boundaries
            )
            boundaries.extend(
                Line3D(begin, window.vertices[(i + 1) % len(window.vertices)])
                for i, begin in enumerate(window.vertices)
            )
        self.layer.track_segments.append(_snapshot(candidate))
        # DuplicatePolyPair does not copy distFromSp; preserve the previous value.
        old_distance = current.distance_from_start
        self.current = _snapshot(candidate)
        self.current.distance_from_start = old_distance
        finite = all(
            np.isfinite(window.center.as_array()).all()
            for window in (candidate.track1, candidate.track2)
        )
        return finish("accepted" if finite else "nonfinite_tracking_geometry", accepted=True)


def track_railway(
    initialized: InitializationResult,
    grid: GridIndex,
    *,
    config: ExtractionConfig | None = None,
    rng: RandSource | None = None,
    track_direction: int = 0,
    main_track: bool = True,
    max_steps: int | None = None,
) -> TrackingResult:
    """Run the sequential stage after initialization; no recognition postpass yet."""
    if initialized.source_return is not True or initialized.initial_heights is None:
        raise ValueError("successful source initialization is required before tracking")
    tracker = SequentialTracker(
        initialized.layer,
        grid,
        initialized.initial_heights,
        config=config,
        rng=rng,
        track_direction=track_direction,
        main_track=main_track,
    )
    return tracker.run(max_steps=max_steps)
