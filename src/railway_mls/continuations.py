"""Experimental continuation of separate rail pairs at a selected switch."""

from copy import copy
from dataclasses import dataclass
from itertools import product

import numpy as np

from .config import ExtractionConfig
from .geometry import Line3D, direction_mode
from .models import TrackLayer, TrackSegment, TrackWindow
from .observations import ObservationSelection
from .points import Point3D
from .tracking import SequentialTracker, TrackingStep


def _layer_from_windows(first: TrackWindow, second: TrackWindow) -> TrackLayer:
    segment = TrackSegment(first, second, distance_from_start=0.0)
    segment.update_gauge()
    segment.update_center()
    return TrackLayer(
        track_segments=[segment],
        rail1_vector=[first.center],
        rail2_vector=[second.center],
        center_vector=[segment.center],
    )


def fitted_window_layer(segment: TrackSegment) -> TrackLayer:
    """Keep the accepted pose while detaching its references to the prefix cloud."""
    windows = []
    for fitted in (segment.track1, segment.track2):
        line = copy(fitted.member_line)
        line.members = None
        windows.append(TrackWindow(
            fitted.center, fitted.orientation, vertices=fitted.vertices, member_line=line
        ))
    return _layer_from_windows(*windows)


@dataclass(frozen=True)
class RailPair:
    indices: tuple[int, int]
    begins: tuple[Point3D, Point3D]
    ends: tuple[Point3D, Point3D]
    support_counts: tuple[int, int]
    gauge: float
    shared_length: float
    continuity_error: float

    def seed_layer(self, config: ExtractionConfig) -> TrackLayer:
        windows = []
        for begin, end in zip(self.begins, self.ends):
            line = Line3D(begin, end)
            window = TrackWindow(line.center, line.angle, member_line=line)
            window.generate_vertices(
                config.track_window_length, config.track_window_width, direction_mode(end, begin)
            )
            windows.append(window)
        return _layer_from_windows(*windows)


def pair_rail_lines(
    lines: tuple[tuple[Line3D, ...], tuple[Line3D, ...]],
    previous: tuple[Point3D, Point3D],
    predicted: tuple[Point3D, Point3D],
    config: ExtractionConfig,
) -> tuple[RailPair, ...]:
    """Pair supported model lines in ordered rail windows at a common station."""
    predicted_xy = np.array([(point.x, point.y) for point in predicted])
    previous_xy = np.array([(point.x, point.y) for point in previous])
    origin = predicted_xy.mean(axis=0)
    forward = origin - previous_xy.mean(axis=0)
    advance = np.linalg.norm(forward)
    if not np.isfinite(advance) or advance == 0:
        return ()
    forward /= advance
    across = np.array([-forward[1], forward[0]])
    expected = (predicted_xy - origin) @ across
    if expected[1] < expected[0]:
        across = -across
        expected = -expected

    pairs = []
    for (i, first), (j, second) in product(enumerate(lines[0]), enumerate(lines[1])):
        if not first.members or not second.members:
            continue
        endpoints = np.array(
            [[line.begin.as_array(), line.end.as_array()] for line in (first, second)]
        )
        if not np.isfinite(endpoints).all():
            continue
        stations = (endpoints[:, :, :2] - origin) @ forward
        order = stations.argsort(axis=1)
        stations = np.take_along_axis(stations, order, axis=1)
        endpoints = np.take_along_axis(endpoints, order[:, :, None], axis=1)
        start, end = max(stations[:, 0]), min(stations[:, 1])
        # Require a common measured span as long as the source MDL minimum
        # for a secondary line. Interpolating within it keeps both rail heights.
        if end - start < 0.2:
            continue
        fractions = (np.array([start, end]) - stations[:, :1]) / np.diff(stations, axis=1)
        clipped = endpoints[:, :1] + fractions[:, :, None] * np.diff(endpoints, axis=1)
        offsets = (clipped[:, :, :2] - origin) @ across
        gauges = offsets[1] - offsets[0]
        if np.any(abs(gauges - config.track_gauge) > 0.15):
            continue
        pairs.append(
            RailPair(
                (i, j),
                (Point3D(*clipped[0, 0]), Point3D(*clipped[1, 0])),
                (Point3D(*clipped[0, 1]), Point3D(*clipped[1, 1])),
                (len(first.members), len(second.members)),
                float(gauges.mean()),
                float(end - start),
                float(max(abs(offsets.mean(axis=1) - expected))),
            )
        )
    return tuple(pairs)


def pairs_at_step(step: TrackingStep, config: ExtractionConfig) -> tuple[RailPair, ...]:
    if len(step.models) != 2 or not step.attempts:
        return ()
    return pair_rail_lines(
        tuple(model.lines for model in step.models),
        tuple(state.point for state in step.current_states),
        tuple(prediction.state.point for prediction in step.attempts[-1].predictions),
        config,
    )


class ContinuationTracker(SequentialTracker):
    """Choose observations using this continuation's own predicted rail positions."""

    def _select_turnout_observation(self, models, attempt, mode):
        lines = tuple(model.lines for model in models)
        pairs = pair_rail_lines(
            lines,
            (self.current.track1.center, self.current.track2.center),
            tuple(prediction.state.point for prediction in attempt.predictions),
            self.config,
        )
        if not pairs:
            return None
        pair = min(pairs, key=lambda item: (item.continuity_error, item.indices))
        first, second = (rail[index] for rail, index in zip(lines, pair.indices))
        empty = self.grid.cloud.select([])
        return ObservationSelection(
            observations=(first.members, second.members),
            left_points=(empty, empty),
            right_points=(empty, empty),
            selected_lines=(first, second),
            selected_indices=pair.indices,
            ordered_indices=tuple(tuple(range(len(rail))) for rail in lines),
            nearest_ranks=(None, None),
            target_distance=self.config.track_gauge,
            local_adjustment=None,
            status="ok",
        )
