"""Source engine state, toe-to-branch scheduling and the UI's two-pass workflow."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import isfinite
from operator import index

import numpy as np

from .config import ExtractionConfig
from .geometry import Line3D, distance_2d, perpendicular_foot
from .initialization import InitializationResult, initialize_tracks
from .models import InitialLocation, SwitchBox, ToePoint, TrackLayer, TrackStack
from .points import Point3D
from .sampling import MsvcRand, RandSource
from .spatial import GridIndex
from .tracking import SequentialTracker, TrackingResult


# This is the scheduler's exclusion list, independent of DetectToePnts support.
SKIPPED_BRANCH_TYPES = frozenset((2, 3, 6, 7, 8, 9, 10, 11, 14, 15, 16, 17, 18, 19))


@dataclass(frozen=True)
class VectorTag:
    status: str
    index: int | None = None


def tag_rail_vector(points: list[Point3D], point: Point3D, point_type: int = 2) -> VectorTag:
    """AddPointTofootOfPependicularOnRailVtx, mutating one vector in place.

    XY equality ignores Z; insertion requires strict interior in BOTH axes.
    Otherwise the first nearest XY vertex is tagged. Empty native vectors cause
    unsigned size()-1 underflow and an invalid access; report that explicitly.
    """
    if not points:
        return VectorTag("undefined_source_empty_vector")
    for i, vertex in enumerate(points):
        if vertex.x == point.x and vertex.y == point.y:
            points[i] = replace(vertex, point_type=point_type)
            return VectorTag("existing_vertex", i)
    for i in range(len(points) - 1):
        first, second = points[i : i + 2]
        foot = perpendicular_foot(point, Line3D(first, second))
        if min(first.x, second.x) < foot.x < max(first.x, second.x) and min(
            first.y, second.y
        ) < foot.y < max(first.y, second.y):
            points.insert(i + 1, replace(foot, point_type=point_type))
            return VectorTag("inserted_projection", i + 1)
    nearest, minimum = None, 1e100
    for i, vertex in enumerate(points):
        distance = distance_2d(vertex, point)
        if distance < minimum:
            nearest, minimum = i, distance
    if nearest is None:
        return VectorTag("unchanged")
    points[nearest] = replace(points[nearest], point_type=point_type)
    return VectorTag("nearest_vertex", nearest)


@dataclass(frozen=True)
class VectorReversal:
    status: str
    popped_points: tuple[Point3D, ...] = ()


def reverse_layer_vectors(layers: list[TrackLayer]) -> VectorReversal:
    """ReserverVectors: reverse/swap every layer, then pop layer zero's tails.

    Segment history, boundaries, toes and stacks are deliberately untouched.
    Earlier mutations survive an undefined empty-vector pop.
    """
    for layer in layers:
        layer.rail1_vector.reverse()
        layer.rail2_vector.reverse()
        layer.center_vector.reverse()
        layer.rail1_vector[:], layer.rail2_vector[:] = layer.rail2_vector[:], layer.rail1_vector[:]
    if not layers:
        return VectorReversal("undefined_source_reverse_layer")
    popped = []
    for vector in (layers[0].rail1_vector, layers[0].rail2_vector, layers[0].center_vector):
        if not vector:
            return VectorReversal("undefined_source_vector_pop", tuple(popped))
        popped.append(vector.pop())
    return VectorReversal("ok", tuple(popped))


def reverse_seed(location: InitialLocation) -> InitialLocation:
    """MainForm's literal mode/angle expressions; the angle is not wrapped."""
    if location.second_point is None:
        raise ValueError("bidirectional tracking requires the initialized second rail position")
    mode = index(location.mode) + 2
    # C++ remainder keeps the dividend's sign, unlike Python's negative modulo.
    mode = mode % 4 if mode >= 0 else -(-mode % 4)
    return InitialLocation(location.second_point, location.angle + 180, mode)


@dataclass(frozen=True)
class TrackingInvocation:
    layer_index: int
    direction: int
    main_track: bool
    result: TrackingResult | None
    status: str

    @property
    def reference_comparable(self) -> bool:
        return not self.status.startswith(("undefined_source", "nonfinite")) and (
            self.result is None or self.result.reference_comparable
        )


@dataclass(frozen=True)
class BranchTracking:
    toe_index: int
    toe: ToePoint
    layer_index: int | None
    seed: InitialLocation | None
    initialization: InitializationResult | None
    tracking: TrackingInvocation | None
    status: str


@dataclass(frozen=True)
class LayerToeTag:
    layer_index: int
    toe_index: int
    result: VectorTag


@dataclass(frozen=True)
class TrackingPass:
    main: TrackingInvocation
    branches: tuple[BranchTracking, ...]
    tags: tuple[LayerToeTag, ...]
    status: str

    @property
    def reference_comparable(self) -> bool:
        return (
            self.status == "completed"
            and self.main.reference_comparable
            and all(
                branch.initialization is None
                or (
                    branch.initialization.source_return is not None
                    and not branch.initialization.status.startswith("nonfinite")
                )
                for branch in self.branches
            )
            and all(
                branch.tracking is None or branch.tracking.reference_comparable
                for branch in self.branches
            )
        )


@dataclass(frozen=True)
class BidirectionalTracking:
    original_location: InitialLocation
    reverse_location: InitialLocation
    reverse_initialization: InitializationResult
    reverse_pass: TrackingPass | None
    reversal: VectorReversal | None
    forward_initialization: InitializationResult | None
    forward_pass: TrackingPass | None
    status: str

    @property
    def reference_comparable(self) -> bool:
        return (
            self.status == "completed"
            and self.reverse_initialization.status == "ok"
            and self.reverse_pass is not None
            and self.reverse_pass.reference_comparable
            and self.forward_initialization is not None
            and self.forward_initialization.status == "ok"
            and self.forward_pass is not None
            and self.forward_pass.reference_comparable
        )


class TrackingEngine:
    """Shared RailwayExtraction state across main/branch/directional invocations.

    The grid, labels, RNG, stacks and toe pool persist. Reset initialization clears
    layers only; branch initialization overwrites the engine's current location.
    Results retain live layer references, as do the lower-level tracking results.
    Diagnostic max_steps bounds each invocation and stops dependent orchestration;
    rerun a bounded workflow from a fresh engine rather than restarting a pass.
    """

    def __init__(
        self,
        grid: GridIndex,
        *,
        config: ExtractionConfig | None = None,
        rng: RandSource | None = None,
    ) -> None:
        self.config = config if config is not None else ExtractionConfig()
        if self.config.grid_size != grid.grid_size:
            raise ValueError("engine configuration must match the grid size")
        self.grid = grid
        self.rng = rng if rng is not None else MsvcRand()
        self.layers: list[TrackLayer] = []
        self.current_layer: TrackLayer | None = None
        self.initial_location: InitialLocation | None = None
        self.initial_heights: tuple[float, float] | None = None
        self.initialization_status: str | None = None
        self.distance_from_start = 0.0
        self.completed_stacks: list[TrackStack] = []
        self.toe_points: list[ToePoint] = []
        self.switch_boxes: list[SwitchBox] = []
        self._initialization_pending = True

    @property
    def current_layer_index(self) -> int:
        # Layer equality can descend into NumPy arrays; source uses pointer identity.
        return next(i for i, layer in enumerate(self.layers) if layer is self.current_layer)

    def initialize(
        self, seed: InitialLocation, *, reset_layers: bool = True
    ) -> InitializationResult:
        """OnRailextractionSetinit or Setinitiallocation on the current layer.

        The reset wrapper retains its new layer even when initialization fails.
        Its float angle argument narrows to float32, unlike direct initialization.
        Initial heights/location change only at their source assignment points.
        """
        if reset_layers:
            with np.errstate(over="ignore", invalid="ignore"):
                seed = replace(seed, angle=float(np.float32(seed.angle)))
            self.layers.clear()
            self.current_layer = TrackLayer()
        elif self.current_layer is None:
            raise ValueError("create a current layer before appending initialization")
        self.distance_from_start = 0.0
        self._initialization_pending = True
        result = initialize_tracks(
            seed, self.grid, config=self.config, rng=self.rng, layer=self.current_layer
        )
        self.initialization_status = result.status
        if result.initial_heights is not None:
            self.initial_heights = result.initial_heights
        if result.location is not None:
            self.initial_location = result.location
        if reset_layers:
            self.layers.append(self.current_layer)
        return result

    def _track_current(
        self, direction: int, main_track: bool, max_steps: int | None
    ) -> TrackingInvocation:
        layer_index = self.current_layer_index
        if self.initialization_status and self.initialization_status.startswith("undefined_source"):
            return TrackingInvocation(
                layer_index, direction, main_track, None, self.initialization_status
            )
        if not self.current_layer.track_segments:
            return TrackingInvocation(layer_index, direction, main_track, None, "no_segments")
        if not self._initialization_pending:
            return TrackingInvocation(
                layer_index, direction, main_track, None, "undefined_source_tracking_reentry"
            )
        if self.initial_heights is None:
            return TrackingInvocation(
                layer_index, direction, main_track, None, "undefined_source_initial_heights"
            )
        if not all(isfinite(value) for value in self.initial_heights):
            return TrackingInvocation(
                layer_index, direction, main_track, None, "nonfinite_initial_heights"
            )
        tracker = SequentialTracker(
            self.current_layer,
            self.grid,
            self.initial_heights,
            config=self.config,
            rng=self.rng,
            track_direction=direction,
            main_track=main_track,
            distance_from_start=self.distance_from_start,
            completed_stacks=self.completed_stacks,
            toe_points=self.toe_points,
            switch_boxes=self.switch_boxes,
        )
        result = tracker.run(max_steps=max_steps)
        if result.steps:
            self._initialization_pending = False
        self.distance_from_start = result.distance_from_start
        return TrackingInvocation(layer_index, direction, main_track, result, result.status)

    @staticmethod
    def _halt(status: str) -> bool:
        return status == "step_limit" or status.startswith(("undefined_source", "nonfinite"))

    def run_pass(self, *, max_steps: int | None = None) -> TrackingPass:
        """OnRailextractionTrackingRailway, including its final center-vector tags."""
        if self.current_layer is None:
            raise ValueError("initialize an engine layer before running tracking")
        if max_steps is not None:
            max_steps = index(max_steps)
            if max_steps < 0:
                raise ValueError("max_steps must be nonnegative")
        main = self._track_current(1, True, max_steps)
        branches, tags = [], []

        def finish(status: str) -> TrackingPass:
            return TrackingPass(main, tuple(branches), tuple(tags), status)

        if self._halt(main.status):
            return finish(main.status)
        # Native vector<ToePt> copy: used changes remain local to this invocation.
        toes = [replace(toe, used=False) for toe in self.toe_points]
        for i, toe in enumerate(toes):
            if toe.used:
                branches.append(BranchTracking(i, toe, None, None, None, None, "already_used"))
                continue
            self.current_layer = TrackLayer()
            self.layers.append(self.current_layer)
            layer_index = self.current_layer_index
            if toe.track_type in SKIPPED_BRANCH_TYPES:
                branches.append(
                    BranchTracking(i, toe, layer_index, None, None, None, "skipped_type")
                )
                continue
            if toe.right is None or toe.angle is None or toe.mode is None:
                branches.append(
                    BranchTracking(
                        i, toe, layer_index, None, None, None, "undefined_source_branch_seed"
                    )
                )
                return finish("undefined_source_branch_seed")
            seed = InitialLocation(
                Point3D(toe.right.x, toe.right.y, toe.right.y), toe.angle, toe.mode
            )
            if not all(isfinite(value) for value in (*seed.point.as_array(), seed.angle)):
                # Setinitiallocation resets these before its unsafe grid lookup.
                self.distance_from_start = 0.0
                self._initialization_pending = True
                branches.append(
                    BranchTracking(i, toe, layer_index, seed, None, None, "nonfinite_branch_seed")
                )
                return finish("nonfinite_branch_seed")
            initialized = self.initialize(seed, reset_layers=False)
            if initialized.source_return is None:
                branches.append(
                    BranchTracking(i, toe, layer_index, seed, initialized, None, initialized.status)
                )
                return finish(initialized.status)
            tracking = (
                self._track_current(0, False, max_steps) if initialized.source_return else None
            )
            # In native code this write follows Trackingrailway, even after a
            # defined initialization failure. A diagnostic pause has not returned.
            halted = tracking is not None and self._halt(tracking.status)
            if not halted:
                toe.used = True
            status = tracking.status if tracking is not None else "initialization_failed"
            branches.append(
                BranchTracking(i, toe, layer_index, seed, initialized, tracking, status)
            )
            if halted:
                return finish(status)
        for n, layer in enumerate(self.layers):
            for i, toe in enumerate(layer.toe_points):
                if toe.center is None:
                    tags.append(LayerToeTag(n, i, VectorTag("undefined_source_toe_center")))
                    return finish("undefined_source_toe_center")
                tag = tag_rail_vector(layer.center_vector, toe.center)
                tags.append(LayerToeTag(n, i, tag))
                if tag.status.startswith("undefined_source"):
                    return finish(tag.status)
        return finish("completed")

    def reverse_vectors(self) -> VectorReversal:
        result = reverse_layer_vectors(self.layers)
        if self.layers:
            self.current_layer = self.layers[0]
        return result

    def run_bidirectional(self, *, max_steps: int | None = None) -> BidirectionalTracking:
        """MainForm's reverse run, vector reversal and original-direction run.

        Start from initialize(...). The original fitted location is captured before
        branch initialization can overwrite the engine location. Both passes use
        the source wrapper's fixed direction flags and one persistent random stream.
        """
        if self.initial_location is None:
            raise ValueError("initialize a complete paired location before bidirectional tracking")
        if max_steps is not None:
            max_steps = index(max_steps)
            if max_steps < 0:
                raise ValueError("max_steps must be nonnegative")
        original = self.initial_location
        reverse_location = reverse_seed(original)
        reverse_initialization = self.initialize(reverse_location)
        reverse_pass = None
        reversal, forward_initialization, forward_pass = None, None, None

        def finish(status: str) -> BidirectionalTracking:
            return BidirectionalTracking(
                original,
                reverse_location,
                reverse_initialization,
                reverse_pass,
                reversal,
                forward_initialization,
                forward_pass,
                status,
            )

        if reverse_initialization.source_return is None:
            return finish(reverse_initialization.status)
        reverse_pass = self.run_pass(max_steps=max_steps)
        if reverse_pass.status != "completed":
            return finish(reverse_pass.status)
        reversal = self.reverse_vectors()
        if reversal.status != "ok":
            return finish(reversal.status)
        forward_initialization = self.initialize(original, reset_layers=False)
        if forward_initialization.source_return is None:
            return finish(forward_initialization.status)
        forward_pass = self.run_pass(max_steps=max_steps)
        return finish(forward_pass.status)
