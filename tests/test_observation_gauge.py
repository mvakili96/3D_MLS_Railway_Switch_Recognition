from copy import copy
from itertools import product
import json
import subprocess
import sys
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls.config import ExtractionConfig
from railway_mls.gauge import adjust_two_lines, adjust_two_lines_with_observation
from railway_mls.geometry import Line3D, distance_2d, distance_to_line_2d
from railway_mls.observations import determine_observation, observation_from_support
from railway_mls.points import Point3D, PointCloud
from tests.cpp_reference import CppReference


def state(line):
    return [
        *line.begin.as_array(),
        *line.end.as_array(),
        *line.center.as_array(),
        line.angle,
        line.a,
        line.b,
        line.c,
    ]


def selection(points):
    return [0] if points is None else [len(points), *points.indices]


def descriptor(line):
    return [*state(line), *selection(line.members)]


def rail(y, cloud=None, row=0, reverse=False):
    begin, end = Point3D(0, y, 0.2), Point3D(2, y, 0.2)
    line = Line3D(end, begin) if reverse else Line3D(begin, end)
    line.update_center()
    if cloud is not None:
        line.members = cloud.select([row, row, (row + 1) % len(cloud)])
    return line


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.cloud = PointCloud(
            [(i, i * 0.1, i * 0.01) for i in range(9)], class_codes=[9] * 9, segment_ids=[7] * 9
        )

    def test_single_single_returns_members_without_sorting_or_adjustment(self):
        first, second = rail(0, self.cloud), rail(1.5, self.cloud, 3)
        bad_guide = Line3D(Point3D(0, 0, 0), Point3D(0, 0, 0))
        result = determine_observation([first], [second], bad_guide, bad_guide)
        self.assertTrue(result.success)
        self.assertIs(result.observations[0], first.members)
        self.assertIsNone(result.local_adjustment)
        self.assertEqual(result.nearest_ranks, (None, None))

    def test_equal_signed_values_repeat_first_line_and_preserve_duplicate_points(self):
        first = [rail(0, self.cloud), rail(0, self.cloud, 3)]
        result = determine_observation(first, [rail(1.5, self.cloud, 6)], rail(0), rail(1.5))
        self.assertEqual(result.ordered_indices[0], (0, 0))
        self.assertEqual(result.selected_indices[0], 0)
        assert_array_equal(result.observations[0].indices, [0, 0, 1])
        assert_array_equal(result.left_points[0].indices, [0, 0, 1])

    def test_two_candidates_break_gauge_tie_toward_second_rank(self):
        result = determine_observation(
            [rail(0, self.cloud)],
            [rail(0.875, self.cloud, 3), rail(1.125, self.cloud, 6)],
            rail(0),
            rail(1),
            config=ExtractionConfig(track_gauge=0.98),
        )
        self.assertTrue(result.success)
        self.assertEqual(result.target_distance, 1)
        self.assertEqual(result.selected_indices, (0, 0))  # lower Y is second signed rank

    def test_three_candidates_leave_a_minimum_tie_unresolved(self):
        result = determine_observation(
            [rail(0, self.cloud)],
            [rail(0.875, self.cloud, 3), rail(1.125, self.cloud, 6), rail(2, self.cloud, 8)],
            rail(0),
            rail(1),
            config=ExtractionConfig(track_gauge=0.98),
        )
        self.assertFalse(result.success)
        self.assertEqual(result.status, "unresolved_gauge_tie")
        self.assertIsNone(result.selected_lines[1])
        self.assertEqual(len(result.observations[1]), 0)
        self.assertIsNone(result.local_adjustment)

    def test_stack_cutoffs_differ_by_direction(self):
        first = [rail(y, self.cloud, i) for i, y in enumerate((-0.2, 0.03))]
        second = [rail(y, self.cloud, i + 3) for i, y in enumerate((1.4, 1.53))]
        forward1 = determine_observation(first, second, rail(0), rail(1.5), stack_size=1)
        forward2 = determine_observation(first, second, rail(0), rail(1.5), stack_size=2)
        reverse2 = determine_observation(
            first, second, rail(0), rail(1.5), stack_size=2, track_mode=1
        )
        reverse3 = determine_observation(
            first, second, rail(0), rail(1.5), stack_size=3, track_mode=1
        )
        self.assertEqual(forward1.selected_indices[0], 1)
        self.assertEqual(forward2.selected_indices[0], 0)
        self.assertEqual(reverse2.selected_indices[1], 1)
        self.assertEqual(reverse3.selected_indices[1], 1)  # rank zero happens to be nearest
        # Move guide so rank one is nearest: reverse changes only AFTER size two.
        reverse2 = determine_observation(
            first, second, rail(0), rail(1.4), stack_size=2, track_mode=1
        )
        reverse3 = determine_observation(
            first, second, rail(0), rail(1.4), stack_size=3, track_mode=1
        )
        self.assertEqual(reverse2.selected_indices[1], 0)
        self.assertEqual(reverse3.selected_indices[1], 1)

    def test_three_line_reverse_anchor_always_nearest_and_outer_sides_only(self):
        second = [rail(y, self.cloud, i + 3) for i, y in enumerate((1.1, 1.6, 2))]
        result = determine_observation(
            [rail(0, self.cloud)], second, rail(0), rail(1.1), stack_size=50, track_mode=1
        )
        self.assertEqual(result.selected_indices[1], 0)
        assert_array_equal(result.left_points[1].indices, second[2].members.indices)
        self.assertEqual(len(result.right_points[1]), 0)

    def test_stored_centers_control_ranking_and_local_adjustment_is_discarded(self):
        first = [rail(0, self.cloud), rail(-0.3, self.cloud, 3)]
        first[0].center = Point3D(1, 9, 0.2)
        before = [state(line) for line in first]
        result = determine_observation(first, [rail(1.5, self.cloud, 6)], rail(0), rail(1.5))
        self.assertEqual(result.ordered_indices[0], (0, 1))
        for line, original in zip(first, before):
            assert_array_equal(state(line), original)
        self.assertIsNotNone(result.local_adjustment)
        self.assertIsNot(result.local_adjustment.lines[0], result.selected_lines[0])
        assert_array_equal(self.cloud.class_codes, [9] * 9)
        assert_array_equal(self.cloud.segment_ids, [7] * 9)

    def test_invalid_candidates_ownership_and_empty_members_are_explicit(self):
        with self.assertRaises(ValueError):
            determine_observation([], [rail(1.5)], rail(0), rail(1.5), cloud=self.cloud)
        with self.assertRaises(ValueError):
            determine_observation([rail(0)], [rail(1.5)], rail(0), rail(1.5))
        with self.assertRaises(ValueError):
            determine_observation(
                [rail(0, self.cloud)], [rail(1.5, PointCloud([[0, 0, 0]]))], rail(0), rail(1.5)
            )
        result = determine_observation([rail(0)], [rail(1.5)], rail(0), rail(1.5), cloud=self.cloud)
        self.assertEqual(result.status, "empty_observation_members")


class GaugeTests(unittest.TestCase):
    def test_weighted_source_formula_does_not_force_final_gap_to_target(self):
        cloud = PointCloud([[0, 0, 0], [0, -0.25, 0], [0, -0.5, 0]])
        first, second = rail(0), rail(2)
        result = adjust_two_lines_with_observation(
            first,
            second,
            cloud.select(),
            cloud.select([]),
            references=(Point3D(0, 0, 0), Point3D(0, 2, 0)),
            target_distance=1,
        )
        self.assertTrue(result.success)
        assert_allclose(result.support_distances, [1, 1.25, 1.5])
        assert_allclose(result.normalized_errors, [0, 0.5, 1])
        self.assertEqual(result.weighted_distance, 1.25)
        self.assertEqual(result.scale, 0.8)
        self.assertAlmostEqual(distance_2d(first.center, second.center), 0.8)
        self.assertAlmostEqual(first.center.y, 0.1)
        self.assertEqual(first.c, 1)  # The angle-zero coefficient bug is preserved.
        self.assertNotEqual(distance_to_line_2d(first.center, first), 0)

    def test_uniform_errors_and_empty_support_retain_nan(self):
        cloud = PointCloud([[0, 0, 0], [1, 0, 0]])
        for support in (cloud.select(), cloud.select([])):
            result = adjust_two_lines_with_observation(
                rail(0),
                rail(2),
                support,
                cloud.select([]),
                references=(Point3D(0, 0, 0), Point3D(0, 2, 0)),
            )
            self.assertEqual(result.status, "nonfinite_weighted_distance")
            self.assertTrue(np.isnan(result.weighted_distance))
            self.assertTrue(np.isnan(result.lines[0].begin.as_array()).all())

    def test_basic_zero_gap_is_exposed_and_uses_configured_length(self):
        result = adjust_two_lines(rail(1), rail(1))
        self.assertEqual(result.status, "nonfinite_adjustment")
        self.assertTrue(np.isinf(result.scale))
        result = adjust_two_lines(rail(0), rail(2), config=ExtractionConfig(track_window_length=3))
        self.assertTrue(result.success)
        for line in result.lines:
            self.assertAlmostEqual(distance_2d(line.begin, line.end), 3)

    def test_cached_center_refresh_and_reference_order_are_explicit(self):
        line = rail(0)
        line.end = Point3D(4, 0, 0)
        self.assertEqual(line.center.x, 1)
        self.assertEqual(line.update_center().x, 2)
        parameters = (line.angle, line.a, line.b, line.c)
        line.order_endpoints_by_reference(Point3D(10, 0, 100))
        self.assertEqual(line.begin.x, 4)
        self.assertEqual((line.angle, line.a, line.b, line.c), parameters)

    def test_support_observation_uses_input_order_mean_and_descending_15_percent_z(self):
        cloud = PointCloud([(i, -i, i * 0.01) for i in range(20)])
        result = observation_from_support(cloud.select())
        self.assertTrue(result.success)
        assert_allclose(result.point.as_array(), [9.5, -9.5, 0.16])
        assert_allclose(result.mean.as_array(), [9.5, -9.5, 0.095])
        assert_array_equal(result.support.indices, np.arange(19, -1, -1))
        assert_array_equal(cloud.select().indices, np.arange(20))
        self.assertEqual(observation_from_support(cloud.select([])).status, "no_support")


class ObservationGaugeCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def compare_observation(
        self, first, second, guides, cloud, *, track_mode=0, stack_size=0, config=None, prefix=False
    ):
        config = config or ExtractionConfig()
        inputs = [
            len(cloud),
            config.track_gauge,
            config.track_window_length,
            track_mode,
            stack_size,
            1,
            *cloud.xyz.ravel(),
            *descriptor(guides[0]),
            *descriptor(guides[1]),
            len(first),
        ]
        for line in first:
            inputs.extend(descriptor(line))
        inputs.append(len(second))
        for line in second:
            inputs.extend(descriptor(line))
        native = self.reference.run("observe_prefix" if prefix else "observe", *inputs)
        result = determine_observation(
            first,
            second,
            *guides,
            track_mode=track_mode,
            stack_size=stack_size,
            config=config,
            cloud=cloud,
        )
        cursor = 0
        for points in (
            *result.observations,
            result.left_points[0],
            result.right_points[0],
            result.left_points[1],
            result.right_points[1],
        ):
            length = int(native[cursor])
            assert_array_equal(points.indices, native[cursor + 1 : cursor + 1 + length])
            cursor += length + 1
        self.assertEqual(cursor, len(native))
        return result

    def test_all_candidate_counts_directions_and_stack_boundaries_match_original(self):
        cloud = PointCloud([(i, i * 0.2, i * 0.01) for i in range(9)])
        for n1, n2, mode, stack in product(range(1, 4), range(1, 4), (0, 1), (0, 1, 2, 3)):
            with self.subTest(n1=n1, n2=n2, mode=mode, stack=stack):
                first = [rail(y, cloud, i) for i, y in enumerate((-0.23, 0.04, 0.39)[:n1])][::-1]
                second = [rail(y, cloud, i + 3) for i, y in enumerate((1.29, 1.57, 1.93)[:n2])]
                self.compare_observation(
                    first, second, (rail(0), rail(1.5)), cloud, track_mode=mode, stack_size=stack
                )

    def test_reversed_guides_and_stored_centers_match_original(self):
        cloud = PointCloud([(i, i * 0.2, i * 0.01) for i in range(9)])
        for mode, stack in product((0, 1, 7), (1, 2, 3)):
            first = [rail(y, cloud, i) for i, y in enumerate((-0.3, 0.04, 0.4))]
            second = [rail(y, cloud, i + 3) for i, y in enumerate((1.3, 1.57, 1.9))]
            first[0].center = Point3D(1, 0.07, 22)
            self.compare_observation(
                first,
                second,
                (rail(0, reverse=True), rail(1.5, reverse=True)),
                cloud,
                track_mode=mode,
                stack_size=stack,
            )

    def test_oblique_and_vertical_guides_at_large_coordinates_match_original(self):
        cloud = PointCloud([(i, i * 0.2, i * 0.01) for i in range(9)])
        origin = np.array([600000.0, 4800000.0, 125.0])
        for angle, mode, stack in product((30, 90, 140), (0, 1), (1, 2, 3)):
            with self.subTest(angle=angle, mode=mode, stack=stack):
                radians = angle * np.pi / 180
                along = np.array([np.cos(radians), np.sin(radians), 0])
                across = np.array([-np.sin(radians), np.cos(radians), 0])

                def make(offset, row=None):
                    center = origin + offset * across
                    line = Line3D(Point3D(*(center - along)), Point3D(*(center + along)))
                    line.update_center()
                    if row is not None:
                        line.members = cloud.select([row])
                    return line

                first = [make(y, i) for i, y in enumerate((-0.3, 0.02, 0.4))]
                second = [make(y, i + 3) for i, y in enumerate((1.3, 1.57, 1.9))]
                self.compare_observation(
                    first, second, (make(0), make(1.5)), cloud, track_mode=mode, stack_size=stack
                )

    def test_duplicate_signed_values_and_two_way_ties_match_original(self):
        cloud = PointCloud([(i, i, 0) for i in range(9)])
        self.compare_observation(
            [rail(0, cloud), rail(0, cloud, 3)], [rail(1.5, cloud, 6)], (rail(0), rail(1.5)), cloud
        )
        self.compare_observation(
            [rail(0, cloud)],
            [rail(0.875, cloud, 3), rail(1.125, cloud, 6)],
            (rail(0), rail(1)),
            cloud,
            config=ExtractionConfig(track_gauge=0.98),
        )

    def test_unresolved_triple_tie_matches_defined_native_selection_prefix(self):
        cloud = PointCloud([(i, i, 0) for i in range(9)])
        result = self.compare_observation(
            [rail(0, cloud)],
            [rail(0.875, cloud, 3), rail(1.125, cloud, 6), rail(2, cloud, 8)],
            (rail(0), rail(1)),
            cloud,
            config=ExtractionConfig(track_gauge=0.98),
            prefix=True,
        )
        self.assertEqual(result.status, "unresolved_gauge_tie")

    def compare_gauge(
        self,
        first,
        second,
        cloud,
        *,
        modes=(1, 1),
        weighted=False,
        supports=None,
        length=1,
        target=1.5,
        line_mode=1,
    ):
        references = (Point3D(-5, -3, 0), Point3D(8, 6, 0))
        inputs = [
            len(cloud),
            length,
            target,
            *modes,
            line_mode,
            *cloud.xyz.ravel(),
            *descriptor(first),
            *descriptor(second),
        ]
        config = ExtractionConfig(track_window_length=length)
        if weighted:
            supports = (
                supports
                if supports is not None
                else (cloud.select([0, 1, 2]), cloud.select([3, 4, 5]))
            )
            inputs.extend(
                [
                    *references[0].as_array(),
                    *references[1].as_array(),
                    *selection(supports[0]),
                    *selection(supports[1]),
                ]
            )
        native = self.reference.run("gauge_weighted" if weighted else "gauge_basic", *inputs)
        kwargs = dict(modes=modes, line_mode=line_mode, target_distance=target, config=config)
        result = (
            adjust_two_lines_with_observation(
                first, second, *supports, references=references, **kwargs
            )
            if weighted
            else adjust_two_lines(first, second, **kwargs)
        )
        expected = [
            *descriptor(first),
            *descriptor(second),
            result.common_angle,
            result.initial_gap,
            result.scale,
            result.weighted_distance if weighted else float("nan"),
            len(result.normalized_errors),
            *result.normalized_errors,
            len(result.support_distances),
            *result.support_distances,
        ]
        assert_allclose(expected, native, rtol=1e-11, atol=1e-11, equal_nan=True)
        return result

    def test_basic_all_mode_pairs_and_angle_wraps_match_original_stored_state(self):
        cloud = PointCloud(
            [
                [0, -0.1, 0.2],
                [1, 0.2, 0.21],
                [2, 0.6, 0.22],
                [0, 1.4, 0.23],
                [1, 2.1, 0.24],
                [2, 2.5, 0.25],
            ]
        )
        for m1, m2, angles in product(
            range(1, 5), range(1, 5), ((15, 25), (175, 5), (5, 175), (0, 0), (90, 90), (200, 200))
        ):
            with self.subTest(modes=(m1, m2), angles=angles):
                first, second = rail(0, cloud), rail(2, cloud, 3)
                first.angle, second.angle = angles
                self.compare_gauge(first, second, cloud, modes=(m1, m2), length=2.3, line_mode=m2)

    def test_weighted_all_mode_pairs_and_angle_wraps_match_original(self):
        cloud = PointCloud(
            [
                [0, -0.1, 0.2],
                [1, 0.2, 0.21],
                [2, 0.6, 0.22],
                [0, 1.4, 0.23],
                [1, 2.1, 0.24],
                [2, 2.5, 0.25],
            ]
        )
        for m1, m2, angles in product(
            range(1, 5), range(1, 5), ((15, 25), (175, 5), (5, 175), (0, 0), (90, 90))
        ):
            with self.subTest(modes=(m1, m2), angles=angles):
                first, second = rail(0, cloud), rail(2, cloud, 3)
                first.angle, second.angle = angles
                self.compare_gauge(
                    first, second, cloud, modes=(m1, m2), weighted=True, line_mode=m1
                )

    def test_weighted_nan_and_basic_zero_gap_match_original(self):
        cloud = PointCloud([[0, 0, 0]] * 6)
        self.compare_gauge(rail(1), rail(1), cloud)
        for supports in (
            (cloud.select([0, 1]), cloud.select([])),
            (cloud.select([]), cloud.select([])),
        ):
            self.compare_gauge(rail(0), rail(2), cloud, weighted=True, supports=supports)

    def test_basic_and_weighted_scale_elevation_match_original(self):
        cloud = PointCloud(
            [
                [0, -0.1, 0.2],
                [1, 0.2, 0.21],
                [2, 0.6, 0.22],
                [0, 1.4, 0.23],
                [1, 2.1, 0.24],
                [2, 2.5, 0.25],
            ]
        )
        first = Line3D(Point3D(2, 3, 4), Point3D(4, 5, 6))
        second = Line3D(Point3D(1, 6, 8), Point3D(3, 8, 10))
        self.compare_gauge(copy(first), copy(second), cloud, target=1.7)
        self.compare_gauge(first, second, cloud, target=1.7, weighted=True)


class ObservationDemoTests(unittest.TestCase):
    def test_demo_selects_fits_observes_and_adjusts_both_rails(self):
        completed = subprocess.run(
            [sys.executable, "-m", "railway_mls", "observation-demo"],
            capture_output=True,
            text=True,
            check=True,
        )
        payload = json.loads(completed.stdout)
        self.assertEqual(payload["status"], "ok")
        self.assertEqual(payload["candidate_counts"], [3, 3])
        self.assertEqual(payload["observation_counts"], [100, 100])
        self.assertEqual(payload["gauge_adjustment"]["status"], "ok")
        self.assertFalse(payload["tracking_implemented"])


if __name__ == "__main__":
    unittest.main()
