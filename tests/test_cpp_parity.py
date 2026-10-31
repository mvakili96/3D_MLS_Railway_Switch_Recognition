"""Behavior comparisons with executable extracts of the original C++ source."""

import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls import (
    Bounds3D,
    ExtractionConfig,
    GridIndex,
    InitialLocation,
    Line3D,
    Point3D,
    PointCloud,
)
from railway_mls.geometry import (
    line_parameters_from_angle,
    perpendicular_foot,
    points_in_polygon,
    source_angle,
)
from railway_mls.models import TrackWindow, seed_windows
from tests.cpp_reference import CppReference


def flatten(points):
    return [value for point in points for value in (point.x, point.y, point.z)]


class CppParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def test_windows_all_modes_cardinal_and_oblique_angles(self):
        for mode in (0, 1, 2, 3, 4):
            for angle in (0, 30, 45, 90, 135, 180, 280):
                with self.subTest(mode=mode, angle=angle):
                    center = Point3D(12.3, -7.2, 4)
                    window = TrackWindow(center, angle)
                    window.generate_vertices(1, 0.42, mode)
                    expected = self.reference.run(
                        "window", mode, angle, *flatten([center]), 1, 0.42
                    )
                    assert_allclose(flatten(window.vertices), expected, rtol=0, atol=2e-12)

    def test_angles_and_ordering(self):
        origin = Point3D(0, 0, 1)
        for x, y in [(0, 0), (0, -1), (-1, 0), (0, 1), (2, 1), (-2, -1), (1, -1)]:
            end = Point3D(x, y, 4)
            with self.subTest(x=x, y=y):
                self.assertAlmostEqual(
                    source_angle(origin, end),
                    self.reference.run("angle", *flatten([origin, end]))[0],
                )
                for mode in (0, 1, 2, 3, 4):
                    line = Line3D(origin, end)
                    line.order_endpoints(mode)
                    assert_allclose(
                        flatten([line.begin, line.end]),
                        self.reference.run("order", mode, *flatten([origin, end])),
                        rtol=0,
                        atol=0,
                    )

    def test_angle_parameters_including_source_axis_quirks(self):
        for angle in (0, 30, 90, 135, 180, 280):
            with self.subTest(angle=angle):
                assert_allclose(
                    line_parameters_from_angle(angle, 7, 11),
                    self.reference.run("params", angle, 7, 11),
                    rtol=1e-14,
                    atol=1e-14,
                )

    def test_grid_assignment_with_fractional_sizes_and_explicit_header_bounds(self):
        random = np.random.default_rng(19)
        xyz = random.uniform([10, -2, 0], [14, 2, 1], size=(40, 3))
        xyz = np.concatenate([xyz, [[10, -2, 0], [14, 2, 1]]])
        bounds = Bounds3D(Point3D(10, -2, 0), Point3D(14, 2, 1))
        for size in (0.3, 0.5, 1.0, 1.5, 2.0):
            with self.subTest(size=size):
                grid = GridIndex(PointCloud(xyz), size, bounds=bounds)
                cell_ids = np.full(len(xyz), -1)
                for cell in range(grid.num_cells):
                    cell_ids[grid.members(cell).indices] = cell
                expected = self.reference.run(
                    "grid", size, *flatten([bounds.minimum, bounds.maximum]), len(xyz), *xyz.ravel()
                )
                assert_array_equal([grid.num_x, grid.num_y, *cell_ids], expected)

    def test_neighbor_indices_at_borders_and_outside(self):
        bounds = Bounds3D(Point3D(10, -2, 0), Point3D(14, 2, 1))
        cloud = PointCloud(np.empty((0, 3)))
        for size in (0.5, 1.0, 1.5):
            grid = GridIndex(cloud, size, bounds=bounds)
            for x, y in [(10, -2), (14, 2), (12.1, 0.3), (9.9, -2.1), (15, 3)]:
                for count in (0, 1, 2, 3, 5):
                    with self.subTest(size=size, x=x, y=y, count=count):
                        expected = self.reference.run(
                            "neighbor", 10, -2, size, grid.num_x, grid.num_y, x, y, count
                        )
                        assert_array_equal(grid.neighbor_indices(x, y, count), expected)

    def test_polygon_boundaries_and_ordered_membership(self):
        vertices = tuple(Point3D(x, y, 0) for x, y in [(0, 0), (2, 0), (2, 1), (0, 1)])
        xy = np.array([[0, 0], [2, 0], [2, 1], [0, 1], [1, 0], [1, 1], [1, 0.5], [-0.1, 0.5]])
        for polygon in (vertices, tuple(reversed(vertices))):
            expected = self.reference.run(
                "polygon",
                4,
                len(xy),
                *[v for point in polygon for v in (point.x, point.y)],
                *xy.ravel(),
            )
            assert_array_equal(points_in_polygon(polygon, xy), expected)

    def test_seed_placement_and_noop_window_swap(self):
        seed_point = Point3D(12.3, -7.2, 4)
        config = ExtractionConfig()
        for mode in (0, 1, 2, 3, 4):
            for angle in (0, 30, 90, 135, 280):
                with self.subTest(mode=mode, angle=angle):
                    segment = seed_windows(InitialLocation(seed_point, angle, mode), config)
                    actual = flatten([segment.track1.center, segment.track2.center])
                    actual += flatten(segment.track1.vertices) + flatten(segment.track2.vertices)
                    expected = self.reference.run(
                        "seed",
                        mode,
                        angle,
                        *flatten([seed_point]),
                        config.track_gauge,
                        config.track_window_length,
                        config.track_window_width,
                    )
                    assert_allclose(actual, expected, rtol=0, atol=2e-12)

    def test_side_windows_and_perpendicular_foot(self):
        for end in (Point3D(4, 2, 3), Point3D(2, 4, 5), Point3D(4, 4, 5)):
            begin = Point3D(2, 2, 1)
            line = Line3D(begin, end)
            window = TrackWindow(begin, 0)
            window.generate_side_vertices(line, 0.4, 0.2)
            actual = flatten(window.left_vertices + window.main_vertices + window.right_vertices)
            expected = self.reference.run("side", *flatten([begin, end]), 0.4, 0.2)
            assert_allclose(actual, expected, rtol=0, atol=2e-12)
            point = Point3D(7, -1, 9)
            assert_allclose(
                flatten([perpendicular_foot(point, line)]),
                self.reference.run("foot", *flatten([point, begin, end])),
                atol=2e-12,
            )


if __name__ == "__main__":
    unittest.main()
