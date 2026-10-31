"""A sparse implementation of the source's XY grid and ordered neighborhoods."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, floor, isfinite
from operator import index

import numpy as np

from .models import TrackSegment
from .points import Point3D, PointCloud, PointSelection


@dataclass(frozen=True)
class Bounds3D:
    minimum: Point3D
    maximum: Point3D

    def __post_init__(self) -> None:
        low, high = self.minimum.as_array(), self.maximum.as_array()
        if not np.all(np.isfinite((low, high))) or np.any(low > high):
            raise ValueError("bounds must be finite and minimum must not exceed maximum")

    @classmethod
    def from_cloud(cls, cloud: PointCloud) -> Bounds3D:
        if not len(cloud):
            raise ValueError("an empty cloud requires explicit grid bounds")
        return cls(Point3D(*cloud.xyz.min(axis=0)), Point3D(*cloud.xyz.max(axis=0)))


class GridIndex:
    """generateGrid/GetNeighborIdx, including the non-unit insertion quirk.

    Only occupied cells consume storage. Cell numbering, members, traversal order,
    and out-of-bounds -1 sentinels match the dense C++ grid. No thinning is applied.
    """

    def __init__(
        self, cloud: PointCloud, grid_size: float = 1.0, *, bounds: Bounds3D | None = None
    ) -> None:
        self.cloud = cloud
        self.grid_size = float(grid_size)
        if not isfinite(self.grid_size) or self.grid_size <= 0:
            raise ValueError("grid_size must be finite and positive")
        self.bounds = bounds if bounds is not None else Bounds3D.from_cloud(cloud)
        low, high = self.bounds.minimum.as_array(), self.bounds.maximum.as_array()
        if np.any(cloud.xyz < low) or np.any(cloud.xyz > high):
            raise ValueError("grid bounds must contain all cloud coordinates")
        self.num_x = max(1, ceil((high[0] - low[0]) / self.grid_size))
        self.num_y = max(1, ceil((high[1] - low[1]) / self.grid_size))
        self.num_cells = self.num_x * self.num_y
        buckets: dict[int, list[int]] = {}
        # The source floors displacement BEFORE division, then converts to int
        # (truncation). This intentionally differs from the query formula.
        for row, (x, y, _) in enumerate(cloud.xyz):
            column = int(min(self.num_x - 1, floor(x - low[0]) / self.grid_size))
            line = int(min(self.num_y - 1, floor(y - low[1]) / self.grid_size))
            cell = line * self.num_x + column
            buckets.setdefault(cell, []).append(row)
        self._cells = {cell: cloud.select(rows) for cell, rows in buckets.items()}

    @property
    def occupied_cells(self) -> int:
        return len(self._cells)

    def members(self, cell: int) -> PointSelection:
        cell = index(cell)
        if cell < 0 or cell >= self.num_cells:
            raise IndexError("cell is outside the grid")
        selection = self._cells.get(cell)
        return selection if selection is not None else self.cloud.select([])

    def cell_bounds(self, cell: int) -> tuple[float, float, float, float]:
        """Return min_x, max_x, min_y, max_y, without shrinking the last cell."""
        cell = index(cell)
        if cell < 0 or cell >= self.num_cells:
            raise IndexError("cell is outside the grid")
        y, x = divmod(cell, self.num_x)
        low, size = self.bounds.minimum, self.grid_size
        return low.x + x * size, low.x + (x + 1) * size, low.y + y * size, low.y + (y + 1) * size

    def neighbor_indices(self, x: float, y: float, neighborhood_size: int = 5) -> tuple[int, ...]:
        """GetNeighborIdx: X outer/Y inner; retain -1 for every invalid cell.

        Maximum-bound queries are not clamped to the last cell, unlike insertion.
        Even sizes retain the source's asymmetric offset; zero returns no cells.
        """
        size = index(neighborhood_size)
        if size < 0:
            raise ValueError("neighborhood_size must be nonnegative")
        column = floor((x - self.bounds.minimum.x) / self.grid_size)
        line = floor((y - self.bounds.minimum.y) / self.grid_size)
        half = size // 2
        cells = []
        for i in range(size):
            for j in range(size):
                r, c = column + i - half, line + j - half
                cells.append(
                    c * self.num_x + r if 0 <= r < self.num_x and 0 <= c < self.num_y else -1
                )
        return tuple(cells)

    def neighbor_points(self, x: float, y: float, neighborhood_size: int = 5) -> PointSelection:
        groups = [
            self._cells[cell].indices
            for cell in self.neighbor_indices(x, y, neighborhood_size)
            if cell in self._cells
        ]
        return self.cloud.select(np.concatenate(groups) if groups else [])

    def assign_segment_members(
        self, segment: TrackSegment, neighborhood_size: int = 5
    ) -> tuple[PointSelection, PointSelection]:
        """The member-assignment part of GenerateTrkSegAssignMemPts only."""
        center = segment.center
        candidates = self.neighbor_points(center.x, center.y, neighborhood_size)
        return segment.track1.assign_members(candidates), segment.track2.assign_members(candidates)
