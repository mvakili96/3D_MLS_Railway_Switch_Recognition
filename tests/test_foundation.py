import json
import math
import subprocess
import sys
import unittest

import numpy as np
from numpy.testing import assert_array_equal

from railway_mls import (
    Bounds3D,
    ExtractionConfig,
    GridIndex,
    InitialLocation,
    Line3D,
    Point3D,
    PointCloud,
    TrackLayer,
    TrackWindow,
    seed_windows,
)
from railway_mls.geometry import (
    direction_mode,
    distance_2d,
    distance_3d,
    distance_to_line_2d,
    line_parameters_from_angle,
    perpendicular_foot,
    points_in_polygon,
    polygon_area,
    source_angle,
)
from railway_mls.synthetic import straight_scene


class PointTests(unittest.TestCase):
    def test_defaults_precision_and_input_ownership(self):
        xyz = np.array([[513195.62000001, 5458237.77000001, 6.562]], dtype=np.float64)
        cloud = PointCloud(xyz)
        assert_array_equal(cloud.xyz, xyz)
        self.assertEqual(cloud.xyz.dtype, np.float64)
        assert_array_equal(cloud.class_codes, [0])
        assert_array_equal(cloud.segment_ids, [-1])
        assert_array_equal(cloud.intensity, [0])
        xyz[0, 0] = 0
        self.assertEqual(cloud.xyz[0, 0], 513195.62000001)
        with self.assertRaises(ValueError):
            cloud.xyz[0, 0] = 2

    def test_overlapping_selections_share_labels_but_not_coordinate_identity(self):
        cloud = PointCloud([[1, 2, 3], [1, 2, 3], [4, 5, 6]])
        first, second = cloud.select([2, 0, 2]), cloud.select([0, 1])
        first.set_class_code(5)
        assert_array_equal(cloud.class_codes, [5, 0, 5])
        second.set_segment_id(9)
        assert_array_equal(cloud.segment_ids, [9, 9, -1])
        self.assertEqual(len(cloud), 3)  # Equal coordinates do not merge pointers.
        assert_array_equal(first.indices, [2, 0, 2])
        with self.assertRaises(IndexError):
            cloud.select([-1])
        with self.assertRaises(ValueError):
            cloud.select([0.5])

    def test_explicit_empty_and_invalid_clouds(self):
        empty = PointCloud(np.empty((0, 3)))
        self.assertEqual(empty.select().xyz.shape, (0, 3))
        for xyz in ([], [[1, 2]], [[1, 2, float("nan")]]):
            with self.subTest(xyz=xyz), self.assertRaises(ValueError):
                PointCloud(xyz)
        with self.assertRaises(ValueError):
            PointCloud([[1, 2, 3]], intensity=[1.5])
        with self.assertRaises(ValueError):
            PointCloud([[1, 2, 3]], intensity=[2**40])

    def test_point_arithmetic_and_source_equality(self):
        point = Point3D(0, 0, 0, point_type=2)
        self.assertTrue(point.is_equal(Point3D(0.000009, 0, 0)))
        self.assertFalse(point.is_equal(Point3D(0.00001, 0, 0)))
        self.assertEqual((point + Point3D(1, 2, 3)).point_type, 0)
        self.assertEqual(Point3D(1, 0, 0).cross(Point3D(0, 1, 0)), Point3D(0, 0, 1))
        self.assertTrue(np.all(np.isnan(point.normalized().as_array())))


class GeometryTests(unittest.TestCase):
    def test_folded_angles_and_modes(self):
        start = Point3D(0, 0, 0)
        cases = [
            ((1, 0), 0, 1),
            ((0, 1), 90, 1),
            ((-1, 0), 180, 2),
            ((0, -1), 180 - 90, 4),
            ((1, 1), 45, 1),
            ((-1, 1), 135, 2),
            ((-1, -1), 45, 3),
            ((1, -1), 135, 4),
            ((0, 0), 0, 1),
        ]
        for (x, y), angle, mode in cases:
            end = Point3D(x, y, 10)
            with self.subTest(x=x, y=y):
                self.assertAlmostEqual(source_angle(start, end), angle)
                self.assertEqual(direction_mode(end, start), mode)

    def test_axis_angle_parameter_quirks_are_preserved(self):
        self.assertEqual(line_parameters_from_angle(0, 7, 11), (0, -1, 7))
        self.assertEqual(line_parameters_from_angle(90, 7, 11), (-1, 0, 11))
        line = Line3D(Point3D(7, 11, 2), Point3D(8, 11, 4))
        self.assertEqual((line.a, line.b, line.c), (0, -1, 11))

    def test_polygon_boundary_contract_and_height_independence(self):
        vertices = tuple(Point3D(x, y, 0) for x, y in [(0, 0), (2, 0), (2, 1), (0, 1)])
        mask = points_in_polygon(vertices, [[0, 0], [2, 0], [0, 1], [1, 0], [1, 1], [1, 0.5]])
        assert_array_equal(mask, [True, False, False, True, False, True])
        self.assertEqual(polygon_area(vertices), 2)
        self.assertEqual(polygon_area(tuple(reversed(vertices))), 2)
        window = TrackWindow(Point3D(1, 0.5, 100), 0, vertices=vertices)
        cloud = PointCloud([[1, 0.5, -100], [1, 0.5, 100], [3, 0.5, 0]])
        assert_array_equal(window.assign_members(cloud.select()).indices, [0, 1])

    def test_window_geometry_in_every_mode(self):
        for mode in (0, 1, 2, 3, 4):
            with self.subTest(mode=mode):
                window = TrackWindow(Point3D(10, 20, 3), 30)
                window.generate_vertices(2, 0.4, mode)
                self.assertAlmostEqual(polygon_area(window.vertices), 0.8)
                assert_array_equal([p.z for p in window.vertices], [0, 0, 0, 0])
                self.assertTrue(points_in_polygon(window.vertices, [[10, 20]])[0])
                self.assertIsNone(window.member_line)

    def test_distances_and_projection_height(self):
        origin, point = Point3D(0, 0, 0), Point3D(3, 4, 12)
        self.assertEqual(distance_2d(origin, point), 5)
        self.assertEqual(distance_3d(origin, point), 13)
        horizontal = Line3D(Point3D(0, 2, 4), Point3D(2, 2, 8))
        self.assertEqual(distance_to_line_2d(point, horizontal), 2)
        self.assertEqual(perpendicular_foot(point, horizontal), Point3D(3, 2, 6))
        diagonal = Line3D(Point3D(0, 0, 2), Point3D(2, 2, 4))
        self.assertEqual(perpendicular_foot(Point3D(2, 0, 100), diagonal), Point3D(1, 1, 3))

    def test_side_window_width_and_reset_follow_source(self):
        window = TrackWindow(Point3D(1, 2, 0), 0)
        line = Line3D(Point3D(0, 2, 0), Point3D(2, 2, 0))
        window.generate_side_vertices(line, center_width=0.4, side_width=0.2)
        self.assertAlmostEqual(polygon_area(window.main_vertices), 0.4)
        self.assertTrue(points_in_polygon(window.left_vertices, [[1, 2.3]])[0])
        self.assertTrue(points_in_polygon(window.right_vertices, [[1, 1.7]])[0])
        before = window.left_vertices
        window.vertices = window.main_vertices
        window.turnout, window.left_tag = True, 1
        window.reset()
        self.assertEqual(window.left_vertices, before)
        self.assertFalse(window.turnout)
        self.assertEqual(window.vertices, ())

    def test_seed_geometry_is_not_full_initialization(self):
        config = ExtractionConfig()
        for mode, signs in [(1, (-1, 1)), (2, (-1, -1)), (3, (1, -1)), (4, (1, 1))]:
            segment = seed_windows(InitialLocation(Point3D(10, 20, 5), 30, mode), config)
            self.assertAlmostEqual(segment.track2.center.x, 10 + signs[0] * 0.75)
            self.assertAlmostEqual(
                segment.track2.center.y, 20 + signs[1] * 1.5 * math.cos(math.pi / 6)
            )
            self.assertEqual(segment.track2.center.z, 0)
            self.assertEqual(segment.track1.center.z, 5)
            self.assertIsNone(segment.track1.members)
            self.assertIsNone(segment.track1.member_line)
            before = segment.track1, segment.track2
            segment.swap_windows(mode)
            self.assertEqual((segment.track1, segment.track2), before)
            self.assertAlmostEqual(segment.update_gauge(), config.track_gauge)

    def test_layer_storage_is_independent(self):
        first, second = TrackLayer(), TrackLayer()
        first.center_vector.append(Point3D(0, 0, 0))
        self.assertEqual(second.center_vector, [])


class SpatialTests(unittest.TestCase):
    def setUp(self):
        self.bounds = Bounds3D(Point3D(0, 0, 0), Point3D(3, 3, 1))

    def test_neighbor_order_and_sentinels(self):
        grid = GridIndex(PointCloud([[0, 0, 0], [3, 3, 1]]), bounds=self.bounds)
        self.assertEqual(grid.neighbor_indices(1.5, 1.5, 3), (0, 3, 6, 1, 4, 7, 2, 5, 8))
        self.assertEqual(grid.neighbor_indices(0, 0, 3), (-1, -1, -1, -1, 0, 3, -1, 1, 4))
        self.assertEqual(grid.neighbor_indices(1, 1, 2), (0, 3, 1, 4))
        self.assertEqual(grid.neighbor_indices(1, 1, 0), ())

    def test_membership_preserves_cell_and_input_order(self):
        cloud = PointCloud([[1.1, 0.1, 0], [0.1, 1.1, 0], [0.2, 0.2, 0], [0.3, 0.3, 0]])
        grid = GridIndex(cloud, bounds=self.bounds)
        assert_array_equal(grid.neighbor_points(1, 1, 3).indices, [2, 3, 1, 0])
        grid.members(0).set_class_code(8)
        assert_array_equal(cloud.class_codes, [0, 0, 8, 8])
        with self.assertRaises(IndexError):
            grid.members(-1)

    def test_nonunit_insertion_is_intentionally_not_floor_of_quotient(self):
        cloud = PointCloud([[0, 0, 0], [0.75, 0, 0], [1, 0, 0]])
        grid = GridIndex(cloud, grid_size=0.5)
        assert_array_equal(grid.members(0).indices, [0, 1])
        self.assertEqual(grid.neighbor_indices(0.75, 0, 1), (1,))
        self.assertEqual(len(grid.members(1)), 1)  # Exact maximum clamps to last cell.

    def test_maximum_insertion_and_query_use_different_rules(self):
        cloud = PointCloud([[0, 0, 0], [3, 3, 1]])
        grid = GridIndex(cloud, bounds=self.bounds)
        assert_array_equal(grid.members(8).indices, [1])
        self.assertEqual(grid.neighbor_indices(3, 3, 1), (-1,))
        self.assertEqual(grid.cell_bounds(8), (2, 3, 2, 3))

    def test_empty_sparse_and_invalid_grids(self):
        empty = PointCloud(np.empty((0, 3)))
        with self.assertRaises(ValueError):
            GridIndex(empty)
        huge = GridIndex(empty, bounds=Bounds3D(Point3D(0, 0, 0), Point3D(1e6, 1e6, 0)))
        self.assertEqual(huge.num_cells, 10**12)
        self.assertEqual(huge.occupied_cells, 0)
        self.assertEqual(len(huge.neighbor_points(0, 0)), 0)
        for size in (0, -1, float("nan")):
            with self.subTest(size=size), self.assertRaises(ValueError):
                GridIndex(empty, size, bounds=self.bounds)
        with self.assertRaises(ValueError):
            GridIndex(PointCloud([[4, 0, 0]]), bounds=self.bounds)


class IntegrationTests(unittest.TestCase):
    def test_constructor_defaults(self):
        self.assertEqual(
            ExtractionConfig().to_dict(),
            {
                "grid_size": 1.0,
                "skip_rate": 1,
                "track_window_width": 0.42,
                "track_window_length": 1.0,
                "max_distance_for_ransac": 0.06,
                "track_gauge": 1.5,
                "min_track_points": 5,
                "min_points": 20,
                "mdl_weight": 0.4,
                "grouping_distance": 0.15,
                "min_track_intensity": 0,
                "max_track_intensity": 1500,
            },
        )

    def test_synthetic_selection_matches_full_cloud_filter(self):
        scene = straight_scene()
        segment = seed_windows(scene.seed, scene.config)
        selected = GridIndex(scene.cloud).assign_segment_members(segment)
        for window, members in zip((segment.track1, segment.track2), selected):
            exhaustive = scene.cloud.select().where(
                points_in_polygon(window.vertices, scene.cloud.xyz[:, :2])
            )
            assert_array_equal(np.sort(members.indices), exhaustive.indices)
            self.assertEqual(len(members), 180)
            assert_array_equal(np.unique(members.xyz[:, 2]), [0, 0.1, 0.2])
        selected[0].set_segment_id(3)
        assert_array_equal(scene.cloud.segment_ids[selected[0].indices], 3)
        assert_array_equal(scene.cloud.segment_ids[selected[1].indices], -1)

    def test_cli_runs_without_input_data(self):
        completed = subprocess.run(
            [sys.executable, "-m", "railway_mls", "demo"],
            text=True,
            capture_output=True,
            check=True,
        )
        result = json.loads(completed.stdout)
        self.assertEqual(result["window_point_counts"], [180, 180])
        self.assertEqual(result["coordinate_dtype"], "float64")
        self.assertFalse(result["tracking_implemented"])


if __name__ == "__main__":
    unittest.main()
