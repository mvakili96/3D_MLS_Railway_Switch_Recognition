"""Rail-pair discovery and geometric evidence for independent continuations."""

from dataclasses import dataclass
from itertools import pairwise
from math import floor, hypot

from .config import ExtractionConfig
from .continuations import RailPair, pairs_at_step
from .geometry import distance_2d
from .points import Point3D
from .tracking import TrackingStep

RailCenters = tuple[Point3D, Point3D]

# A single offset window can be a switch blade or a transient fit. Require a
# run of supported windows before adding a route or declaring a rejoin.
CONFIRM_WINDOWS = 3
CONFIRM_LIMIT = 8
REJOIN_WINDOWS = 3


@dataclass(frozen=True)
class BranchStart:
    parent: str
    step: int
    elapsed_steps: int
    pair: RailPair
    random_state: int


def accepted_rails(step: TrackingStep) -> RailCenters | None:
    if step.accepted and step.status == "accepted":
        return step.candidate.track1.center, step.candidate.track2.center
    return None


def outgoing_starts(
    step: TrackingStep, config: ExtractionConfig, parent: str, random_state: int,
    *, preceding_steps: int = 0,
) -> list[BranchStart]:
    """Read local paired fits before their membership references leave scope."""
    if all(len(model.lines) < 2 for model in step.models):
        return []
    pairs = pairs_at_step(step, config)
    if len(pairs) < 2:
        return []
    selected = step.selection.selected_indices if step.selection is not None else None
    primary = next((pair for pair in pairs if pair.indices == selected), None)
    fitted = accepted_rails(step)

    # Use the chosen pair first to suppress nearly coincident model variants.
    # After a rejected step there is no committed continuation to suppress.
    kept, centers = [], []
    if fitted is not None:
        rails = fitted if primary is None else tuple(
            (a + b) * 0.5 for a, b in zip(primary.begins, primary.ends)
        )
        centers.append((rails[0] + rails[1]) * 0.5)
    for pair in pairs:
        if fitted is not None and pair is primary:
            continue
        begin = (pair.begins[0] + pair.begins[1]) * 0.5
        end = (pair.ends[0] + pair.ends[1]) * 0.5
        center = (begin + end) * 0.5
        tangent = end - begin
        length = hypot(tangent.x, tangent.y)
        if any(
            abs((point.x - center.x)*tangent.y - (point.y - center.y)*tangent.x)
            <= config.max_distance_for_ransac * length
            for point in centers
        ):
            continue
        centers.append(center)
        kept.append(BranchStart(parent, step.number, preceding_steps + step.number, pair, random_state))
    return kept


@dataclass(frozen=True)
class _Section:
    route: str
    index: int
    count: int
    begins: RailCenters
    ends: RailCenters
    center: Point3D
    tangent_x: float
    tangent_y: float
    length: float


@dataclass(frozen=True)
class TraceMatch:
    route: str
    station: float
    direction: int
    has_continuation: bool
    distance_xy: float


class TraceCoverage:
    """Index accepted rail sections for same-track tests in either direction."""

    def __init__(self, config: ExtractionConfig):
        self._cells: dict[tuple[int, int], list[_Section]] = {}
        self._cell_size = config.track_window_length
        self.xy_tolerance = config.track_window_width * 0.5
        self.z_tolerance = 2 * config.max_distance_for_ransac
        self._min_alignment = 1 / hypot(1, config.track_window_width / config.track_window_length)

    def add(self, route: str, positions: list[RailCenters]) -> None:
        for index, (begins, ends) in enumerate(pairwise(positions)):
            start, end = ((rails[0] + rails[1]) * 0.5 for rails in (begins, ends))
            length = distance_2d(start, end)
            if length == 0:
                continue
            dx, dy = (end.x - start.x) / length, (end.y - start.y) / length
            section = _Section(route, index, len(positions) - 1, begins, ends, start, dx, dy, length)
            margin, cell = self.xy_tolerance, self._cell_size
            x = range(floor((min(start.x, end.x) - margin) / cell),
                      floor((max(start.x, end.x) + margin) / cell) + 1)
            y = range(floor((min(start.y, end.y) - margin) / cell),
                      floor((max(start.y, end.y) + margin) / cell) + 1)
            for i in x:
                for j in y:
                    self._cells.setdefault((i, j), []).append(section)

    def match(self, begins: RailCenters, ends: RailCenters) -> TraceMatch | None:
        start, center = ((rails[0] + rails[1]) * 0.5 for rails in (begins, ends))
        travel = center - start
        length = hypot(travel.x, travel.y)
        if length == 0:
            return None
        cell = floor(center.x / self._cell_size), floor(center.y / self._cell_size)
        matches = []
        for section in self._cells.get(cell, ()):
            alignment = (travel.x * section.tangent_x + travel.y * section.tangent_y) / length
            if abs(alignment) < self._min_alignment:
                continue
            difference = center - section.center
            fraction = (
                difference.x * section.tangent_x + difference.y * section.tangent_y
            ) / section.length
            if not -1e-8 <= fraction <= 1 + 1e-8:
                continue
            fraction = min(max(fraction, 0), 1)
            predicted = tuple(a + (b - a) * fraction for a, b in zip(section.begins, section.ends))
            for ordered in (predicted, predicted[::-1]):
                if max(abs(a.z - b.z) for a, b in zip(ends, ordered)) > self.z_tolerance:
                    continue
                error = max(distance_2d(a, b) for a, b in zip(ends, ordered))
                if error <= self.xy_tolerance:
                    forward = alignment > 0
                    ahead = section.index < section.count - 1 if forward else section.index > 0
                    matches.append(TraceMatch(
                        section.route, section.index + fraction, 1 if forward else -1, ahead, error
                    ))
                    break
        return min(matches, key=lambda match: (not match.has_continuation, match.distance_xy),
                   default=None)


class BranchProgress:
    """Confirm novel geometry, then detect a sustained return to covered rails."""

    def __init__(self):
        self.confirmed_at: int | None = None
        self.rejoin_at: int | None = None
        self.rejoin: TraceMatch | None = None
        self._novel = 0
        self._overlap = 0
        self._previous_match: TraceMatch | None = None

    def observe(self, step: int, match: TraceMatch | None) -> str | None:
        if self.confirmed_at is None:
            self._novel = self._novel + 1 if match is None else 0
            if self._novel >= CONFIRM_WINDOWS:
                self.confirmed_at = step
            elif step >= CONFIRM_LIMIT:
                return "unconfirmed"
            return None
        if match is None or not match.has_continuation:
            self._overlap = 0
            self._previous_match = None
            self.rejoin = None
            return None
        previous = self._previous_match
        advancing = (
            previous is not None
            and (previous.route, previous.direction) == (match.route, match.direction)
            and (match.station - previous.station) * match.direction > 1e-8
        )
        self._overlap = self._overlap + 1 if advancing else 1
        if self._overlap == 1:
            self.rejoin = match
        self._previous_match = match
        if self._overlap >= REJOIN_WINDOWS:
            self.rejoin_at = step - REJOIN_WINDOWS + 1
            return "rejoined"
        return None
