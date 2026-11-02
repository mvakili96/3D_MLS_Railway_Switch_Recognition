from dataclasses import asdict
import json
from math import copysign
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls import (
    BoundaryLine,
    GridIndex,
    Line3D,
    PipelineOptions,
    Point3D,
    PointCloud,
    TrackSegment,
    TrackWindow,
    classify_segment,
    distance_to_line_3d,
    filter_segment_by_previous_heights,
    initialize_tracks,
    inner_angle,
    intersection_point_3d,
    lines_from_vertices,
    min_max_xy,
    recount,
    run_pipeline,
)
from railway_mls.coverage import check_source_coverage, coverage_summary
from railway_mls.fitting import _bounds
from railway_mls.gmm import classify_window, fit_height_gmm
from railway_mls.synthetic import initialization_scene, tracking_scene
from tests.cpp_reference import CppReference


def segment_fixture(cloud, first, second):
    return TrackSegment(
        TrackWindow(Point3D(1, 2, 3), 35, members=cloud.select(first)),
        TrackWindow(Point3D(4, 5, 6), 35, members=cloud.select(second)),
    )


def cloud_values(cloud):
    values = [len(cloud)]
    for i, point in enumerate(cloud.xyz):
        values.extend(
            (*point, int(cloud.class_codes[i]), int(cloud.segment_ids[i]), int(cloud.intensity[i]))
        )
    return values


def selection_values(points):
    return [len(points), *map(int, points.indices)]


def read_selection(values, cursor):
    count = int(values[cursor])
    return values[cursor + 1 : cursor + 1 + count], cursor + 1 + count


class SourceUtilityCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def compare_segment_gmm(self, heights, first, second):
        cloud = PointCloud(
            np.column_stack((np.arange(len(heights)), np.zeros(len(heights)), heights)),
            intensity=np.arange(len(heights)),
            class_codes=[9] * len(heights),
            segment_ids=[7] * len(heights),
        )
        segment = segment_fixture(cloud, first, second)
        expected = self.reference.run(
            "utility_segment_gmm",
            *cloud_values(cloud),
            *selection_values(segment.track1.members),
            *selection_values(segment.track2.members),
        )
        actual = classify_segment(segment)
        cursor = 0
        for result in actual:
            self.assertEqual(expected[cursor], 3)  # Native prepopulated output is cleared.
            assert_allclose(
                [[p.mu, p.sigma, p.alpha] for p in result.fit.parameters],
                np.array(expected[cursor + 1 : cursor + 10]).reshape(3, 3),
                rtol=1e-10,
                atol=1e-12,
                equal_nan=True,
            )
            cursor += 10
        self.assertEqual(
            [result.fit.iterations for result in actual], expected[cursor : cursor + 2]
        )
        cursor += 2
        for result in actual:
            indices, cursor = read_selection(expected, cursor)
            assert_array_equal(result.heads.indices, indices)
            self.assertIs(result.points.cloud, cloud)
        self.assertEqual(expected[cursor], len(cloud))
        labels = np.array(expected[cursor + 1 : cursor + 1 + 3 * len(cloud)]).reshape(-1, 3)
        assert_array_equal(cloud.class_codes, labels[:, 0])
        assert_array_equal(cloud.segment_ids, labels[:, 1])
        assert_array_equal(cloud.intensity, labels[:, 2])
        self.assertEqual(expected[-1], 0)
        return actual

    def test_paired_gmm_replaces_parameters_and_heads_with_ordered_duplicate_memberships(self):
        heights = np.tile([0.0, 0.1, 0.2], 30)
        first = [*range(0, 45), 2, 2, 1]
        second = list(reversed(range(45, 90)))
        self.compare_segment_gmm(heights, first, second)

    def test_paired_gmm_both_empty_and_one_empty_match_native_ieee_outputs(self):
        heights = np.tile([0.0, 0.1, 0.2], 6)
        for first, second in (([], []), ([], list(range(18))), (list(range(18)), [])):
            with self.subTest(first=bool(first), second=bool(second)):
                actual = self.compare_segment_gmm(heights, first, second)
                for result in actual:
                    if not len(result.points):
                        self.assertFalse(result.fit.finite)
                        self.assertEqual(result.fit.iterations, 101)
                        self.assertTrue(result.clustering.complete)

    def test_paired_gmm_degenerate_fits_keep_empty_or_tied_last_heads(self):
        for heights in ([0.2] * 12, [-1e50, 1e50], [0, 100]):
            with self.subTest(heights=heights[:3]):
                self.compare_segment_gmm(heights, list(range(len(heights))), [0, 0])

    def compare_height_filter(self, track, ground, upper, lower):
        heights = [-0.1, 0.0, 0.1, 0.2, 0.3, 1.0]
        cloud = PointCloud(
            [[i, i * 0.2, z] for i, z in enumerate(heights)],
            intensity=np.arange(6),
            class_codes=[9] * 6,
            segment_ids=[7] * 6,
        )
        segment = segment_fixture(cloud, [5, 0, 1, 2, 2, 3, 4], [4, 3, 2, 1, 0, 5])
        inputs = (
            *cloud_values(cloud),
            *selection_values(segment.track1.members),
            *selection_values(segment.track2.members),
            *track,
            *ground,
            upper,
            lower,
        )
        expected = self.reference.run("utility_height_filter", *inputs)
        result = filter_segment_by_previous_heights(segment, track, ground, upper, lower)
        cursor = 0
        for points in result:
            indices, cursor = read_selection(expected, cursor)
            assert_array_equal(points.indices, indices)
        self.assertEqual(expected[cursor], len(cloud))
        self.assertEqual(expected[-1], 0)
        assert_array_equal(cloud.class_codes, 9)
        assert_array_equal(cloud.segment_ids, 7)
        assert_array_equal(cloud.intensity, np.arange(6))
        return result

    def test_height_erasure_retains_both_equal_limits_order_and_duplicate_ids(self):
        result = self.compare_height_filter((0.2, 0.2), (0, 0), 0, 0)
        assert_array_equal(result[0].indices, [1, 2, 2, 3])
        assert_array_equal(result[1].indices, [3, 2, 1])

    def test_height_erasure_keeps_nan_comparison_semantics_and_negative_thresholds(self):
        for track, ground, upper, lower in (
            ((float("nan"), 0.2), (0, float("nan")), 0, 0),
            ((0.2, 0.2), (0, 0), float("nan"), float("nan")),
            ((float("inf"), -float("inf")), (-float("inf"), float("inf")), 0, 0),
            ((0.1, 0.1), (0.1, 0.1), -0.2, -0.2),
        ):
            with self.subTest(track=track, ground=ground, upper=upper, lower=lower):
                self.compare_height_filter(track, ground, upper, lower)

    def test_xy_bounds_empty_sentinels_large_coordinates_order_and_duplicate_ids(self):
        for xyz, ids in (
            ([], []),
            ([[3, -4, 9], [-2, 6, 0]], [1, 0, 1]),
            ([[2e100, -2e100, 0], [3e100, -3e100, 0]], [0, 1]),
        ):
            with self.subTest(xyz=xyz):
                cloud = PointCloud(np.array(xyz, dtype=float).reshape(-1, 3))
                points = cloud.select(ids)
                expected = self.reference.run(
                    "utility_bounds", *cloud_values(cloud), *selection_values(points)
                )
                assert_allclose(min_max_xy(points), expected, rtol=0, atol=0)

    def test_xy_bounds_keep_first_equal_zero_sign_like_strict_source_comparisons(self):
        cloud = PointCloud([[-0.0, 0.0, 0], [0.0, -0.0, 1]])
        for ids in ([0, 1], [1, 0], [0, 1, 0]):
            points = cloud.select(ids)
            expected = self.reference.run(
                "utility_bounds", *cloud_values(cloud), *selection_values(points)
            )
            actual = min_max_xy(points)
            self.assertEqual([copysign(1, x) for x in actual], [copysign(1, x) for x in expected])

    def test_recount_matches_native_single_add_negative_and_truncated_division(self):
        cases = [(4, current) for current in (-13, -9, -4, -1, 0, 3, 4, 5, 12)]
        cases += [(-4, current) for current in (-10, -4, -3, -1, 0, 1, 3, 4, 9)]
        cases += [(0, -1), (0, -(1 << 31)), (1, (1 << 31) - 1), (-(1 << 31), -(1 << 31))]
        for total, current in cases:
            with self.subTest(total=total, current=current):
                self.assertEqual(
                    recount(total, current),
                    self.reference.run("utility_recount", total, current)[0],
                )
        self.assertEqual(recount(4, -9), -5)

    def test_polygon_edges_close_in_order_zero_tags_and_replace_populated_destination(self):
        for points in (
            (),
            (Point3D(1, 2, 3, 7),),
            (Point3D(0, 0, 1, 7), Point3D(2, 0, 2, 9), Point3D(1, 1, 3, 11)),
        ):
            inputs = [len(points), *(v for p in points for v in (*p.as_array(), p.point_type))]
            expected = self.reference.run("utility_boundaries", *inputs)
            actual = lines_from_vertices(points)
            self.assertEqual(len(actual), expected[0])
            flattened = [len(actual)]
            for line in actual:
                flattened.extend(
                    (
                        *line.begin.as_array(),
                        line.begin.point_type,
                        *line.end.as_array(),
                        line.end.point_type,
                        len(line.members),
                    )
                )
            assert_allclose(flattened, expected, rtol=0, atol=0)

    def test_inner_angle_matches_source_arms_ignore_z_and_clamp_near_parallel(self):
        middle = Point3D(0, 0, 1)
        cases = (
            (Point3D(1, 0, -100), Point3D(0, 1, 200)),
            (Point3D(8, 1, 0), Point3D(8, 1, 0)),
            (Point3D(1, 1, 0), Point3D(-1, -1, 9)),
            (Point3D(-3, 2, 0), Point3D(7, -1, 0)),
        )
        for first, last in cases:
            expected = self.reference.run(
                "utility_angle", *first.as_array(), *middle.as_array(), *last.as_array()
            )
            assert_allclose(inner_angle(first, middle, last), expected[0], rtol=0, atol=0)

    def test_inner_angle_preserves_zero_arm_nan_and_large_arm_ieee_results(self):
        middle = Point3D(0, 0, 0)
        for first, last in (
            (middle, Point3D(1, 0, 0)),
            (middle, middle),
            (Point3D(1e308, 1e308, 0), Point3D(1e308, 1e308, 0)),
        ):
            expected = self.reference.run(
                "utility_angle", *first.as_array(), *middle.as_array(), *last.as_array()
            )
            assert_allclose(
                inner_angle(first, middle, last), expected[0], equal_nan=True, rtol=0, atol=0
            )

    def test_distance_uses_explicit_stored_vector_even_when_stale_nonunit_or_zero(self):
        line = Line3D(Point3D(0, 0, 0), Point3D(2, 0, 0))
        line.center = Point3D(999, 888, 777)  # Native helper ignores this cache.
        point = Point3D(4, 3, 2)
        for vector in (Point3D(1, 0, 0), Point3D(0, 1, 0), Point3D(2, 0, 0), Point3D(0, 0, 0)):
            expected = self.reference.run(
                "utility_distance3d",
                *point.as_array(),
                *line.begin.as_array(),
                *line.end.as_array(),
                *vector.as_array(),
            )
            assert_allclose(
                distance_to_line_3d(point, line, unit_vector=vector), expected[0], rtol=0, atol=0
            )

    def test_distance_preserves_nonfinite_stored_vector_and_coincident_endpoints(self):
        line = Line3D(Point3D(2, 3, 4), Point3D(2, 3, 4))
        point = Point3D(1, 2, 3)
        for vector in (Point3D(1, 0, 0), Point3D(float("nan"), 0, 0), line.unit_vector):
            expected = self.reference.run(
                "utility_distance3d",
                *point.as_array(),
                *line.begin.as_array(),
                *line.end.as_array(),
                *vector.as_array(),
            )
            assert_allclose(
                distance_to_line_3d(point, line, unit_vector=vector), expected[0], equal_nan=True
            )

    def test_inactive_intersection_keeps_both_y_branches_and_noncanonical_coefficient_rule(self):
        for values in ((1, -1, 0, 0, -1, 2), (2, -1, 1, 1, 0, -3), (1, 2, 3, 4, 5, 6)):
            actual = intersection_point_3d(*values)
            expected = self.reference.run("utility_intersection", *values)
            assert_allclose([*actual.as_array(), actual.point_type], expected, rtol=0, atol=0)
        self.assertEqual(intersection_point_3d(1, 2, 3, 4, 5, 6), Point3D(1, 10, 0))

    def test_inactive_intersection_keeps_parallel_and_coincident_ieee_values(self):
        for values in ((1, -1, 0, 2, -2, 1), (1, -1, 0, 1, -1, 0), (0, 0, 0, 0, 0, 0)):
            actual = intersection_point_3d(*values)
            expected = self.reference.run("utility_intersection", *values)
            assert_allclose([*actual.as_array(), actual.point_type], expected, equal_nan=True)


class SourceUtilityContractTests(unittest.TestCase):
    def test_integer_undefined_cases_raise_without_executing_native_division_or_overflow(self):
        for total, current in ((0, 0), (0, 1)):
            with self.assertRaisesRegex(ZeroDivisionError, "undefined source"):
                recount(total, current)
        with self.assertRaisesRegex(OverflowError, "undefined source"):
            recount(-2, -(1 << 31))
        with self.assertRaises(OverflowError):
            recount(1 << 31, 0)
        with self.assertRaises(TypeError):
            recount(4, 1.5)

    def test_boundary_caches_are_absent_while_explicitly_initialized_lines_remain_available(self):
        vertices = (Point3D(0, 0, 0, 9), Point3D(1, 1, 2, 7))
        boundary = lines_from_vertices(vertices)[0]
        self.assertIsInstance(boundary, BoundaryLine)
        self.assertEqual((boundary.begin.point_type, boundary.end.point_type), (0, 0))
        self.assertTrue(
            all(
                getattr(boundary, field) is None
                for field in ("a", "b", "c", "angle", "center", "unit_vector", "d3d")
            )
        )
        self.assertEqual(boundary.members, ())
        self.assertEqual(vertices[0].point_type, 9)
        with self.assertRaisesRegex(ValueError, "uninitialized"):
            distance_to_line_3d(Point3D(2, 0, 0), boundary, unit_vector=boundary.unit_vector)
        initialized = Line3D(boundary.begin, boundary.end)
        self.assertTrue(np.isfinite(initialized.unit_vector.as_array()).all())

    def test_initialization_and_tracking_use_the_endpoint_only_boundary_contract(self):
        scene = initialization_scene()
        initialized = initialize_tracks(scene.seed, GridIndex(scene.cloud))
        self.assertEqual(len(initialized.layer.updated_boundaries), 8)
        self.assertTrue(
            all(
                isinstance(line, BoundaryLine) and line.a is None
                for line in initialized.layer.updated_boundaries
            )
        )
        scene = tracking_scene(length=2)
        result = run_pipeline(scene.cloud, scene.seed, options=PipelineOptions(bidirectional=False))
        self.assertTrue(result.completed)
        self.assertGreater(len(result.engine.layers[0].updated_boundaries), 8)
        self.assertTrue(
            all(
                isinstance(line, BoundaryLine) and line.unit_vector is None
                for line in result.engine.layers[0].updated_boundaries
            )
        )

    def test_bounds_adapter_preserves_empty_native_sentinels_for_fit_callers(self):
        cloud = PointCloud(np.empty((0, 3)))
        self.assertEqual(min_max_xy(cloud.select()), (1e100, -1e100, 1e100, -1e100))
        minimum, maximum = _bounds(cloud.select())
        assert_array_equal(minimum, [1e100, 1e100])
        assert_array_equal(maximum, [-1e100, -1e100])

    def test_paired_empty_gmm_does_not_change_single_window_public_validation(self):
        cloud = PointCloud(np.empty((0, 3)))
        segment = segment_fixture(cloud, [], [])
        first, second = classify_segment(segment)
        self.assertEqual(len(first.fit.parameters), 3)
        self.assertEqual(second.clustering.failed_position, None)
        with self.assertRaises(ValueError):
            fit_height_gmm([])
        with self.assertRaises(ValueError):
            classify_window(cloud.select())

    def test_height_erasure_leaves_original_selection_side_members_and_fitted_lines_untouched(self):
        cloud = PointCloud([[0, 0, 0], [1, 1, 0.2], [2, 2, 1.0]], class_codes=[9] * 3)
        segment = segment_fixture(cloud, [2, 1, 1, 0], [0, 1, 2])
        original = segment.track1.members
        side = cloud.select([2, 2])
        segment.track1.left_members = side
        line = Line3D(Point3D(0, 0, 0), Point3D(1, 1, 0))
        segment.track1.member_line = line
        result = filter_segment_by_previous_heights(segment, (0.2, 0.2), (0, 0), 0, 0)
        assert_array_equal(original.indices, [2, 1, 1, 0])
        assert_array_equal(result[0].indices, [1, 1, 0])
        self.assertIs(segment.track1.left_members, side)
        self.assertIs(segment.track1.member_line, line)
        self.assertEqual(segment.track1.center, Point3D(1, 2, 3))
        assert_array_equal(cloud.class_codes, 9)

    def test_unassigned_memberships_fail_before_erasure(self):
        cloud = PointCloud([[0, 0, 1]])
        segment = segment_fixture(cloud, [0], [0])
        original = segment.track1.members
        segment.track2.members = None
        with self.assertRaises(ValueError):
            filter_segment_by_previous_heights(segment, (0, 0), (0, 0), 0, 0)
        self.assertIs(segment.track1.members, original)
        with self.assertRaises(ValueError):
            classify_segment(segment)

    def test_coverage_has_no_remaining_standalone_or_partial_utility_gaps(self):
        summary = coverage_summary()
        self.assertEqual(check_source_coverage(Path(__file__).resolve().parents[1]), ())
        self.assertEqual(summary["partially_ported_definitions"], [])
        self.assertFalse(
            any("unused_helper" in item["disposition"] for item in summary["pending_definitions"])
        )
        self.assertEqual(
            {
                entry["name"]
                for entry in summary["pending_definitions"]
                if entry["disposition"] == "inactive_pending"
            },
            {"DetectLoopCross", "ValidateLoopCross"},
        )
        self.assertFalse(summary["complete_reconstruction"])

    def test_utilities_demo_reports_real_outputs_and_distinguishes_nan_from_absent_caches(self):
        process = subprocess.run(
            [sys.executable, "-m", "railway_mls", "utilities-demo"],
            capture_output=True,
            text=True,
            check=True,
        )
        result = json.loads(process.stdout)
        self.assertEqual(result["filtered_counts"], [18, 18])
        self.assertEqual(result["distance_3d"], 5)
        self.assertTrue(result["labels_unchanged"])
        self.assertIsNone(result["boundaries"][0]["a"])
        self.assertIn(
            {"path": "/degenerate_inner_angle", "value": "NaN"}, result["nonfinite_values"]
        )
        self.assertEqual(result["boundaries"][0]["begin"]["point_type"], 0)
        self.assertEqual(result["random_draws"], 0)
        self.assertEqual(asdict(lines_from_vertices([Point3D(0, 0, 0)])[0])["members"], ())


if __name__ == "__main__":
    unittest.main()
