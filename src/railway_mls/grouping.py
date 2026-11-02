"""Source XY point growing, retention rules, and shared SegmentID mutation."""

from dataclasses import dataclass

import numpy as np

from .points import PointSelection


def larger_group(first: PointSelection, second: PointSelection) -> bool:
    """LargePts comparator: strict member-count comparison, including ties."""
    return len(first) > len(second)


@dataclass(frozen=True)
class PointGrouping:
    selected: PointSelection
    components: tuple[PointSelection, ...]
    retained: tuple[PointSelection, ...]
    stopped_at_limit: bool = False


def group_points(points: PointSelection, threshold: float = 0.15) -> PointGrouping:
    """SegmentPoints/PointGrowing, preserving source DFS and first-component bias.

    Every visited pointer receives SegmentID=0 (the source never increments it).
    Keep the first component, plus later components strictly larger than 100.
    Return the largest retained component, with stable ties as a documented port
    convention. Iterative frames avoid Python's recursion limit without changing
    seed selection, input scan order, or recursive depth-first visitation order.
    """
    cloud, indices = points.cloud, points.indices
    if not len(points):
        return PointGrouping(cloud.select([]), (), ())
    cloud.segment_ids[indices] = -1
    components = []
    retained = []
    limited = False
    while True:
        unvisited = np.flatnonzero(cloud.segment_ids[indices] < 0)
        if not len(unvisited):
            break
        seed = int(indices[unvisited[0]])
        cloud.segment_ids[seed] = 0
        members = [seed]
        stack = [(seed, 0)]
        while stack:
            current, cursor = stack[-1]
            pending = np.flatnonzero(cloud.segment_ids[indices[cursor:]] < 0) + cursor
            if not len(pending):
                stack.pop()
                continue
            with np.errstate(over="ignore", invalid="ignore"):
                difference = cloud.xyz[indices[pending], :2] - cloud.xyz[current, :2]
                distance = np.sqrt(
                    difference[:, 0] * difference[:, 0] + difference[:, 1] * difference[:, 1]
                )
            neighbors = np.flatnonzero(distance < threshold)
            if not len(neighbors):
                stack.pop()
                continue
            position = int(pending[neighbors[0]])
            child = int(indices[position])
            stack[-1] = (current, position + 1)
            cloud.segment_ids[child] = 0
            members.append(child)
            stack.append((child, 0))
        component = cloud.select(members)
        components.append(component)
        if len(component) > 100 or not retained:
            retained.append(component)
        if len(retained) > 1000:
            limited = True
            break
    retained.sort(key=len, reverse=True)
    return PointGrouping(retained[0], tuple(components), tuple(retained), limited)
