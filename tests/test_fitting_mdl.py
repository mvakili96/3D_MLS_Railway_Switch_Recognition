import math
import json
import subprocess
import sys
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls.config import ExtractionConfig
from railway_mls.fitting import (
    LineFitResult,
    fit_fixed_angle,
    fit_line_ransac,
    fit_loop_cross_line,
    fit_rail_head_line,
)
from railway_mls.geometry import Line3D
from railway_mls.grouping import group_points
from railway_mls.mdl import choose_mdl_model, compute_closeness, segment_track_points_mdl
from railway_mls.points import Point3D, PointCloud
from railway_mls.sampling import MsvcRand, RecordedRand
from tests.line_fixtures import component_cloud, invalid_second_valid_third_cloud, parallel_cloud


class LineFittingTests(unittest.TestCase):
    def test_500_trial_height_quantile_all_input_bounds_and_sigma(self):
        xyz = [(i * 0.05, 0, i * 0.001) for i in range(20)] + [(100, 10, 99)]
        points = PointCloud(xyz).select()
        random = RecordedRand([0, 1] * 500)
        result = fit_line_ransac(points, rng=random)
        self.assertTrue(result.success)
        self.assertEqual(result.iterations, 500)
        assert_allclose(result.line.begin.as_array(), [0, 0, 0.018], rtol=0, atol=1e-15)
        assert_allclose(result.line.end.as_array(), [100, 0, 0.018], rtol=0, atol=1e-15)
        self.assertEqual(result.sigma, 0)
        self.assertEqual(random.draw_count, 1000)
        old = fit_rail_head_line(points, rng=RecordedRand([0, 1] * 100))
        self.assertEqual(old.line.begin.z, 0.016)  # The overloads intentionally differ.

    def test_sigma_uses_mean_squared_xy_distance_without_degrees_of_freedom(self):
        cloud = PointCloud([[0, 0, 0.2], [1, 0, 0.21], [0.5, 0.03, 0.22]])
        result = fit_line_ransac(cloud.select(), rng=RecordedRand([0, 1] * 500))
        self.assertAlmostEqual(result.sigma, math.sqrt(0.03 * 0.03 / 3))
        self.assertEqual(result.line.begin.z, 0.22)
        assert_array_equal(result.support.indices, [2, 1, 0])

    def test_500_attempts_consume_duplicate_draws_and_do_not_fabricate_a_model(self):
        cloud = PointCloud([[0, 0, 0.2], [1, 0, 0.21]])
        random = RecordedRand([0, 0] * 500)
        result = fit_line_ransac(cloud.select(), rng=random)
        self.assertFalse(result.success)
        self.assertEqual(result.status, "undefined_source_no_model")
        self.assertEqual(random.draw_count, 1000)
        random = RecordedRand([])
        self.assertEqual(fit_line_ransac(cloud.select([0]), rng=random).status, "too_few_points")
        self.assertEqual(random.draw_count, 0)

    def test_fixed_angle_has_n_draws_five_percent_height_and_support_bounds(self):
        xyz = [(i * 0.05, 0, i * 0.001) for i in range(20)] + [(100, 10, 99)]
        cloud = PointCloud(xyz)
        random = RecordedRand([0] * len(cloud))
        result = fit_fixed_angle(cloud.select(), 0, rng=random)
        self.assertTrue(result.success)
        self.assertEqual(random.draw_count, len(cloud))
        assert_allclose(result.line.end.as_array(), [0.95, 0, 0.018], rtol=0, atol=1e-15)
        self.assertIsNone(result.sigma)
        self.assertEqual(len(result.support), 20)

    def test_fixed_angle_zero_and_ninety_preserve_axis_coefficient_quirks(self):
        cloud = PointCloud([[10, 0, 0.2], [11, 0, 0.21]])
        failed = fit_fixed_angle(cloud.select(), 0, rng=RecordedRand([0, 1]))
        self.assertEqual(failed.status, "undefined_source_no_model")
        cloud = PointCloud([[5, 5, 0.2], [5, 6, 0.21]])
        fit = fit_fixed_angle(cloud.select(), 90, rng=RecordedRand([0, 0]))
        self.assertTrue(fit.success)
        self.assertEqual(fit.line.begin.x, 5)

    def test_initialized_outputs_are_retained_when_no_new_fixed_model_wins(self):
        cloud = PointCloud([[10, 0, 0.2], [11, 0, 0.21]])
        old_line = Line3D(cloud.point(0), cloud.point(1), members=cloud.select())
        previous = LineFitResult(old_line, cloud.select(), 2, "ok")
        result = fit_fixed_angle(cloud.select(), 0, previous=previous, rng=RecordedRand([0, 1]))
        self.assertTrue(result.success)
        self.assertEqual(result.status, "ok_existing_outputs")
        self.assertEqual(result.line.begin, Point3D(10, 0, 0.21))
        self.assertIs(result.line.members, old_line.members)
        assert_array_equal(result.support.indices, [1, 0])

    def test_fixed_angle_ties_keep_the_first_model_and_equality_is_not_support(self):
        cloud = PointCloud([[0, 0, 0.2], [0.2, 0.2, 0.21], [0, 0.06, 0.22]])
        result = fit_fixed_angle(cloud.select(), 0, rng=RecordedRand([0, 1, 0]))
        assert_array_equal(result.support.indices, [0])

    def test_loop_cross_variant_is_standalone_and_keeps_its_source_gates(self):
        cloud = PointCloud([(i * 0.03, 0, 0.2 + i * 0.001) for i in range(20)])
        valid = fit_loop_cross_line(cloud.select(), 0, rng=RecordedRand([0] * 20))
        self.assertTrue(valid.success)
        self.assertAlmostEqual(valid.line.begin.z, 0.218)
        few = fit_loop_cross_line(cloud.select(np.arange(19)), 0, rng=RecordedRand([0] * 19))
        self.assertFalse(few.success)
        self.assertEqual(few.status, "insufficient_support")
        downsampled = fit_loop_cross_line(
            cloud.select(np.arange(19)), 0, skip_rate=2, rng=RecordedRand([0] * 19)
        )
        self.assertTrue(downsampled.success)
        short = PointCloud([(i * 0.001, 0, 0.2 + i * 0.001) for i in range(20)])
        result = fit_loop_cross_line(short.select(), 0, rng=RecordedRand([0] * 20))
        self.assertEqual(result.status, "line_too_short")
        self.assertIsNotNone(result.line)

    def test_loop_contrast_rejects_a_weak_inner_band(self):
        inner = [(i * 0.03, 0, 0.2 + i * 0.001) for i in range(20)]
        outer = [(i * 0.03, 0.09, 0.3 + i * 0.001) for i in range(10)]
        cloud = PointCloud(inner + outer)
        result = fit_loop_cross_line(cloud.select(), 0, rng=RecordedRand([0] * len(cloud)))
        self.assertFalse(result.success)  # 20*1.4 < 30.
        self.assertEqual(result.status, "insufficient_support")


class GroupingTests(unittest.TestCase):
    def test_depth_first_order_xy_only_and_all_visited_ids_zero(self):
        cloud = PointCloud(
            [[0, 0, 0], [0.1, 0, 100], [-0.1, 0, -100], [0.2, 0, 0]],
            class_codes=[9] * 4,
            segment_ids=[7] * 4,
        )
        grouped = group_points(cloud.select(), 0.15)
        assert_array_equal(grouped.selected.indices, [0, 1, 3, 2])
        assert_array_equal(cloud.segment_ids, [0] * 4)
        assert_array_equal(cloud.class_codes, [9] * 4)

    def test_first_component_survives_and_later_hundred_is_discarded(self):
        cloud = component_cloud((2, 100))
        result = group_points(cloud.select(), 0.015)
        self.assertEqual([len(group) for group in result.components], [2, 100])
        self.assertEqual([len(group) for group in result.retained], [2])
        assert_array_equal(result.selected.indices, [0, 1])
        assert_array_equal(cloud.segment_ids, 0)

    def test_later_101_component_is_retained_and_largest_selected(self):
        cloud = component_cloud((2, 101))
        result = group_points(cloud.select(), 0.015)
        self.assertEqual([len(group) for group in result.retained], [101, 2])
        assert_array_equal(result.selected.indices, np.arange(2, 103))

    def test_pointer_duplicates_are_visited_once_and_unselected_rows_are_untouched(self):
        cloud = component_cloud((4,))
        result = group_points(cloud.select([0, 1, 0, 2]), 0.015)
        assert_array_equal(result.selected.indices, [0, 1, 2])
        assert_array_equal(cloud.segment_ids, [0, 0, 0, 7])

    def test_strict_distance_boundary_and_empty_input(self):
        cloud = PointCloud([[0, 0, 0], [0.15, 0, 0]], segment_ids=[7, 7])
        self.assertEqual(len(group_points(cloud.select(), 0.15).selected), 1)
        self.assertEqual(len(group_points(cloud.select(), np.nextafter(0.15, np.inf)).selected), 2)
        before = cloud.segment_ids.copy()
        result = group_points(cloud.select([]), 0.15)
        self.assertEqual(len(result.selected), 0)
        assert_array_equal(cloud.segment_ids, before)

    def test_stable_component_size_ties_are_an_explicit_port_choice(self):
        cloud = component_cloud((101, 101))
        result = group_points(cloud.select(), 0.015)
        assert_array_equal(result.selected.indices, np.arange(101))

    def test_long_chain_uses_equivalent_iterative_growth(self):
        cloud = PointCloud([(i * 0.01, 0, 0.2) for i in range(1100)])
        result = group_points(cloud.select(), 0.011)
        assert_array_equal(result.selected.indices, np.arange(1100))


class MDLTests(unittest.TestCase):
    def test_absolute_distances_and_source_tie_fallback(self):
        cloud = PointCloud([[0, 2, 0], [1, 3, 0]])
        line = Line3D(Point3D(0, 0, 0), Point3D(1, 0, 0))
        self.assertEqual(compute_closeness(cloud.select(), [line]), 5)  # Not 4+9.
        self.assertEqual(compute_closeness(cloud.select(), []), 2e100)
        self.assertEqual(compute_closeness(cloud.select([]), []), 0)
        self.assertEqual(choose_mdl_model((1, 2, 3)), 1)
        self.assertEqual(choose_mdl_model((2, 1, 3)), 2)
        self.assertEqual(choose_mdl_model((1, 1, 100)), 3)
        self.assertEqual(choose_mdl_model((float("nan"), 0, 10)), 3)

    def test_same_initialized_candidates_can_select_one_two_or_three(self):
        for weight, expected in ((0, 1), (0.05, 2), (0.4, 3)):
            with self.subTest(weight=weight):
                cloud = parallel_cloud()
                random = MsvcRand(1)
                result = segment_track_points_mdl(
                    cloud.select(), 1, 45, config=ExtractionConfig(mdl_weight=weight), rng=random
                )
                self.assertTrue(result.success)
                self.assertTrue(result.reference_comparable)
                self.assertEqual(result.selected_count, expected)
                self.assertEqual([h.eligible for h in result.hypotheses], [True] * 3)
                self.assertEqual(
                    [len(line.members) for line in result.lines], [100, 90, 80][:expected]
                )
                self.assertEqual(random.draw_count, 690)
                assert_array_equal(cloud.segment_ids[:100], -1)
                assert_array_equal(cloud.segment_ids[100:], 0)
                assert_array_equal(cloud.class_codes, 0)

    def test_strict_point_gate_skip_scaling_and_no_first_length_gate(self):
        for count, second_attempted in ((50, False), (51, True)):
            result = segment_track_points_mdl(parallel_cloud((100, count)).select(), 1, 45)
            self.assertEqual(
                any(name == "second_raw" for name, _ in result.attempts), second_attempted
            )
        result = segment_track_points_mdl(
            parallel_cloud((100, 40)).select(), 1, 45, config=ExtractionConfig(skip_rate=2)
        )
        self.assertTrue(result.hypotheses[1].eligible)
        self.assertAlmostEqual(
            result.hypotheses[0].scaled_closeness,
            result.hypotheses[0].raw_closeness * 2 / (2 * math.log(2)),
        )
        short = PointCloud([[0, 0, 0.2], [0.1, 0.1, 0.21]])
        result = segment_track_points_mdl(short.select(), 1, 45)
        self.assertTrue(result.hypotheses[0].eligible)
        self.assertTrue(result.success)
        self.assertFalse(result.reference_comparable)

    def test_third_score_overwrites_false_second_penalty_and_can_include_raw_second(self):
        result = segment_track_points_mdl(invalid_second_valid_third_cloud().select(), 1, 45)
        self.assertTrue(result.reference_comparable)
        self.assertEqual(result.selected_count, 3)
        self.assertEqual([h.eligible for h in result.hypotheses], [True, False, True])
        self.assertGreater(result.hypotheses[1].scaled_closeness, 1e99)
        self.assertLess(result.hypotheses[2].raw_closeness, 1e-10)
        self.assertIsNone(result.lines[1].members)
        self.assertEqual(len(result.lines[2].members), 90)

    def test_missing_geometry_is_flagged_and_never_manufactured(self):
        result = segment_track_points_mdl(parallel_cloud((100,)).select(), 1, 45)
        self.assertEqual(result.selected_count, 1)
        self.assertEqual(result.status, "source_uninitialized_intermediates")
        self.assertEqual(result.uninitialized_candidates, (2, 3))
        self.assertIsNone(result.candidates[1])
        self.assertIsNone(result.hypotheses[1].raw_closeness)
        self.assertFalse(result.reference_comparable)
        forced = segment_track_points_mdl(
            parallel_cloud((100,)).select(), 1, 45, config=ExtractionConfig(mdl_weight=-1)
        )
        self.assertEqual(forced.selected_count, 3)
        self.assertEqual(forced.status, "undefined_source_selected_line")
        self.assertEqual(forced.lines, ())
        self.assertFalse(forced.success)

    def test_failed_initial_fit_and_invalid_skip_rate_are_explicit(self):
        cloud = PointCloud([[10, 0, 0.2], [11, 0, 0.21]])
        result = segment_track_points_mdl(cloud.select(), 1, 0, rng=RecordedRand([0, 1]))
        self.assertEqual(result.status, "undefined_source_fit")
        self.assertIsNone(result.selected_count)
        self.assertEqual(result.lines, ())
        with self.assertRaises(ValueError):
            segment_track_points_mdl(cloud.select(), 1, 45, config=ExtractionConfig(skip_rate=0))

    def test_second_and_third_short_lines_are_invalidated(self):
        cloud = parallel_cloud((100, 60))
        xyz = cloud.xyz.copy()
        xyz[100:, 0] *= 0.1
        xyz[100:, 1] = xyz[100:, 0] + 0.3
        result = segment_track_points_mdl(PointCloud(xyz).select(), 1, 45)
        self.assertFalse(result.hypotheses[1].eligible)
        self.assertEqual(result.selected_count, 1)

    def test_duplicate_references_remain_in_first_support_without_coordinate_dedup(self):
        cloud = parallel_cloud()
        points = cloud.select(np.concatenate((np.arange(len(cloud)), [0, 0, 1])))
        result = segment_track_points_mdl(points, 1, 45)
        self.assertTrue(result.reference_comparable)
        self.assertEqual(len(result.candidates[0].members), 103)
        self.assertEqual(list(result.candidates[0].members.indices).count(0), 3)

    def test_mdl_demo_runs_all_three_selection_counts_without_data_files(self):
        for weight, count in ((0, 1), (0.05, 2), (0.4, 3)):
            with self.subTest(weight=weight):
                completed = subprocess.run(
                    [sys.executable, "-m", "railway_mls", "mdl-demo", "--weight", str(weight)],
                    text=True,
                    capture_output=True,
                    check=True,
                )
                result = json.loads(completed.stdout)
                self.assertEqual(result["status"], "ok")
                self.assertEqual(result["selected_count"], count)
                self.assertTrue(result["reference_comparable"])
                self.assertEqual(result["random_draws"], 690)
                self.assertEqual(result["visited_segment_points"], 170)


if __name__ == "__main__":
    unittest.main()
