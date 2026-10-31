import math
import json
import subprocess
import sys
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls.fitting import fit_rail_head_line
from railway_mls.gmm import (
    HeightGMMFit,
    WindowClassification,
    classify_segment,
    classify_window,
    cluster_by_gmm,
    fit_height_gmm,
)
from railway_mls.heads import extract_track_points, filter_height_band
from railway_mls.models import GMMParameters, TrackSegment, TrackWindow, seed_windows
from railway_mls.points import Point3D, PointCloud
from railway_mls.sampling import MsvcRand, RecordedRand
from railway_mls.spatial import GridIndex
from railway_mls.synthetic import straight_scene


def paired_segment(first, second, *, initial_label=9):
    xyz = np.concatenate((np.asarray(first), np.asarray(second)), axis=0)
    cloud = PointCloud(
        xyz,
        intensity=np.arange(len(xyz)) * 1000 - 500,
        class_codes=np.full(len(xyz), initial_label),
    )
    segment = TrackSegment(
        TrackWindow(Point3D(0, 0, 0.2), 0, members=cloud.select(np.arange(len(first)))),
        TrackWindow(Point3D(0, 1.5, 0.2), 0, members=cloud.select(np.arange(len(first), len(xyz)))),
    )
    return cloud, segment


def rail_points(y, *, coincident_xy=False):
    rows = []
    for sample in range(30):
        for height, jitter in ((0.0, 3e-6), (0.1, 7e-6), (0.2, 1e-5)):
            rows.append((0 if coincident_xy else sample * 0.02, y, height + sample * jitter))
    rows.extend(((0.2, y + 0.2, 0.28), (0.2, y, 0.6), (0.2, y, -0.5)))
    return rows


def raw_draws(count=400, seed=1):
    random = MsvcRand(seed)
    return [random.rand() for _ in range(count)]


class GMMTests(unittest.TestCase):
    def test_constant_heights_return_initial_parameters_without_an_em_iteration(self):
        fit = fit_height_gmm([0.2] * 5)
        self.assertEqual(fit.iterations, 0)
        self.assertEqual(fit.stop_reasons, ("upper_means_close",))
        for parameter in fit.parameters:
            self.assertEqual((parameter.mu, parameter.sigma, parameter.alpha), (0.2, 1.0, 1 / 3))

    def test_small_span_uses_source_initialization_and_upper_mean_exit(self):
        fit = fit_height_gmm([0, 0.06, 0.12])
        self.assertEqual(fit.iterations, 0)
        assert_allclose([p.mu for p in fit.parameters], [0, 0.04, 0.08], atol=0, rtol=0)
        self.assertEqual([p.sigma for p in fit.parameters], [1, 1, 1])

    def test_convergence_keeps_old_parameters_instead_of_the_pending_m_step(self):
        fit = fit_height_gmm([0, 100])
        self.assertEqual(fit.iterations, 1)
        self.assertEqual(fit.stop_reasons, ("first_mean_converged",))
        assert_allclose([p.mu for p in fit.parameters], [0, 100 / 3, 200 / 3], atol=0, rtol=0)
        assert_array_equal([p.sigma for p in fit.parameters], [1, 1, 1])
        assert_array_equal([p.alpha for p in fit.parameters], [1 / 3] * 3)

    def test_nonfinite_fit_is_exposed_without_a_variance_floor_or_reinitialization(self):
        fit = fit_height_gmm([-1e50, 1e50])
        self.assertFalse(fit.finite)
        self.assertEqual(fit.iterations, 101)
        self.assertEqual(fit.stop_reasons, ("iteration_limit",))
        self.assertTrue(all(math.isnan(p.mu) for p in fit.parameters))

    def test_invalid_inputs_are_explicit(self):
        for heights in ([], [[0, 1]], [0, float("nan")]):
            with self.subTest(heights=heights), self.assertRaises(ValueError):
                fit_height_gmm(heights)
        with self.assertRaises(ValueError):
            fit_height_gmm([0, 1], n_components=2)

    def test_ties_and_underflow_go_to_the_last_component(self):
        cloud = PointCloud([[0, 0, 0], [1, 0, 1e4]])
        parameters = [GMMParameters(0, 1, 1 / 3)] * 3
        result = cluster_by_gmm(cloud.select([1, 0, 1]), parameters)
        self.assertTrue(result.complete)
        self.assertEqual([len(points) for points in result.clusters], [0, 0, 3])
        assert_array_equal(result.clusters[-1].indices, [1, 0, 1])
        assert_array_equal(cloud.class_codes, [0, 0])

    def test_source_maximum_and_partial_failure_nan_semantics(self):
        cloud = PointCloud([[0, 0, 0], [1, 0, 100], [2, 0, 0]])
        first_nan = cluster_by_gmm(
            cloud.select(), [GMMParameters(float("nan"), 1, 1), GMMParameters(0, 1, 1)]
        )
        self.assertEqual(first_nan.failed_position, 0)
        self.assertEqual([len(group) for group in first_nan.clusters], [0, 0])
        second_nan = cluster_by_gmm(
            cloud.select(), [GMMParameters(0, 1, 1), GMMParameters(float("nan"), 1, 1)]
        )
        self.assertTrue(second_nan.complete)
        assert_array_equal(second_nan.clusters[0].indices, [0, 1, 2])
        partial = cluster_by_gmm(
            cloud.select(), [GMMParameters(0, 1, float("inf")), GMMParameters(0, 1, 1)]
        )
        self.assertEqual(partial.failed_position, 1)
        assert_array_equal(partial.clusters[0].indices, [0])
        self.assertEqual(len(partial.clusters[1]), 0)

    def test_component_order_is_not_sorted_and_candidate_order_is_last_then_penultimate(self):
        cloud = PointCloud([[0, 0, 0.2], [1, 0, 0], [2, 0, 0.1]])
        parameters = tuple(GMMParameters(mean, 0.001, 1 / 3) for mean in (0.2, 0, 0.1))
        groups = cluster_by_gmm(cloud.select(), parameters)
        result = WindowClassification(cloud.select(), HeightGMMFit(parameters, 0, ()), groups)
        assert_array_equal(result.heads.indices, [2])  # Last component, not maximum mean.
        assert_array_equal(result.head_candidates.indices, [2, 1])

    def test_segment_classification_is_separate_from_label_mutation(self):
        cloud, segment = paired_segment(rail_points(0), rail_points(1.5))
        first, second = classify_segment(segment)
        self.assertGreater(len(first.heads), 0)
        self.assertGreater(len(second.heads), 0)
        assert_array_equal(cloud.class_codes, 9)
        self.assertTrue(classify_window(segment.track1.members).fit.finite)


class RailHeadLineTests(unittest.TestCase):
    def test_microsoft_published_random_sequence(self):
        # Microsoft Learn's rand() example uses srand(1792).
        random = MsvcRand(1792)
        self.assertEqual(
            [random.rand() for _ in range(10)],
            [5890, 1279, 19497, 1207, 11420, 3377, 15317, 29489, 9716, 23323],
        )
        self.assertEqual(random.draw_count, 10)

    def test_ransac_height_percentile_and_all_input_endpoint_bounds(self):
        xyz = [(i * 0.05, 0, i * 0.001) for i in range(20)] + [(100, 10, 99)]
        points = PointCloud(xyz).select()
        random = RecordedRand([0, 1] * 100)
        result = fit_rail_head_line(points, rng=random)
        self.assertTrue(result.success)
        self.assertEqual(result.line.begin, Point3D(0, 0, 0.016))
        self.assertEqual(result.line.end, Point3D(100, 0, 0.016))
        self.assertEqual(len(result.support), 20)
        self.assertEqual(random.draw_count, 200)

    def test_support_ties_retain_first_winner_and_threshold_is_strict(self):
        cloud = PointCloud(
            [[0, 0, 0.2], [1, 0, 0.21], [0, 0.2, 0.22], [1, 0.2, 0.23], [0.5, 0.04, 0.24]]
        )
        result = fit_rail_head_line(cloud.select(), rng=RecordedRand([0, 1] + [2, 3] * 99))
        assert_array_equal(np.sort(result.support.indices), [0, 1])
        self.assertEqual(result.line.begin.y, 0)

    def test_too_few_points_and_no_model_do_not_fabricate_a_line(self):
        cloud = PointCloud([[0, 0, 0.2], [1, 0, 0.2]])
        random = RecordedRand([])
        self.assertEqual(fit_rail_head_line(cloud.select([0]), rng=random).status, "too_few_points")
        self.assertEqual(random.draw_count, 0)
        random = RecordedRand([0, 0] * 100)
        fit = fit_rail_head_line(cloud.select(), rng=random)
        self.assertFalse(fit.success)
        self.assertEqual(fit.status, "undefined_source_no_model")
        self.assertEqual(random.draw_count, 200)
        with self.assertRaises(RuntimeError):
            fit_rail_head_line(cloud.select(), rng=RecordedRand([0]))


class HeadExtractionTests(unittest.TestCase):
    def test_height_band_is_inclusive_and_keeps_input_order(self):
        current = 0.2
        upper, lower = current + 0.1, current - 2.5 * 0.1
        cloud = PointCloud(
            [
                [0, 0, np.nextafter(upper, np.inf)],
                [0, 0, lower],
                [0, 0, upper],
                [0, 0, np.nextafter(lower, -np.inf)],
            ]
        )
        assert_array_equal(filter_height_band(cloud.select([2, 3, 0, 1]), current).indices, [2, 1])

    def test_full_head_extraction_labels_percentile_and_inactive_intensity_filter(self):
        first, second = rail_points(0), rail_points(1.5)
        cloud, segment = paired_segment(first, second)
        random = MsvcRand()
        result = extract_track_points(segment, (0.2, 0.2), rng=random)
        self.assertTrue(result.success)
        self.assertEqual([len(points) for points in result.heads], [30, 30])
        assert_allclose(result.next_heights, [0.20025, 0.20025], rtol=0, atol=1e-15)
        expected = np.full(len(cloud), 9, dtype=np.int32)
        head_rows = np.arange(2, 90, 3)
        expected[head_rows] = expected[head_rows + len(first)] = 3
        assert_array_equal(cloud.class_codes, expected)
        assert_array_equal(cloud.segment_ids, -1)
        self.assertTrue(np.any(cloud.intensity[result.heads[0].indices] > 1500))
        self.assertEqual(random.draw_count, 400)
        self.assertEqual(len(result.windows[0].band_points), 91)

    def test_retry_appends_duplicates_and_retains_previous_heads(self):
        first, second = rail_points(0), rail_points(1.5)
        cloud, segment = paired_segment(first, second)
        existing = cloud.select([2, 2, 0]), cloud.select([len(first) + 2])
        result = extract_track_points(segment, (0.2, 0.2), existing_heads=existing)
        self.assertEqual([len(points) for points in result.heads], [33, 31])
        self.assertEqual(list(result.heads[0].indices).count(2), 3)
        self.assertIn(0, result.heads[0].indices)
        self.assertEqual(cloud.class_codes[0], 3)

    def test_defined_empty_refinement_keeps_both_old_heights_and_partial_labels(self):
        # Coincident XY gives a zero unit vector and NaN Z residuals in the source.
        first = [(0, 0, z) for z in np.tile([0, 0.1, 0.2], 20)]
        cloud, segment = paired_segment(first, rail_points(1.5))
        result = extract_track_points(segment, (0.2, 0.2))
        self.assertEqual(result.status, "no_head_points")
        self.assertEqual(result.next_heights, (0.2, 0.2))
        self.assertEqual(len(result.heads[0]), 0)
        self.assertGreater(len(result.heads[1]), 0)
        assert_array_equal(cloud.class_codes[: len(first)], 9)
        self.assertTrue(np.all(cloud.class_codes[result.heads[1].indices] == 3))

    def test_empty_band_is_an_explicit_undefined_source_prerequisite_failure(self):
        cloud, segment = paired_segment(rail_points(0), rail_points(1.5))
        result = extract_track_points(segment, (20, 0.2))
        self.assertEqual(result.status, "undefined_source_line_fit")
        self.assertIsNone(result.next_heights)
        self.assertIsNone(result.windows[0].classification)
        assert_array_equal(cloud.class_codes, 9)

    def test_synthetic_pipeline_extracts_only_head_heights(self):
        scene = straight_scene()
        segment = seed_windows(scene.seed, scene.config)
        GridIndex(scene.cloud).assign_segment_members(segment)
        result = extract_track_points(segment, (0.2, 0.2))
        self.assertEqual([len(points) for points in result.heads], [60, 60])
        for heads in result.heads:
            assert_array_equal(heads.xyz[:, 2], 0.2)

    def test_height_refinement_does_not_add_an_xy_support_filter(self):
        first = rail_points(0) + [(0.25, 0.2, 0.20011)]
        cloud, segment = paired_segment(first, rail_points(1.5))
        result = extract_track_points(segment, (0.2, 0.2))
        self.assertIn(len(first) - 1, result.heads[0].indices)
        self.assertEqual(cloud.class_codes[len(first) - 1], 3)
        self.assertNotIn(len(first) - 1, result.windows[0].line_fit.support.indices)

    def test_shared_rng_and_retry_outputs_continue_across_calls(self):
        cloud, segment = paired_segment(rail_points(0), rail_points(1.5))
        random = MsvcRand()
        first = extract_track_points(segment, (0.2, 0.2), rng=random)
        second = extract_track_points(segment, (0.2, 0.2), rng=random, existing_heads=first.heads)
        self.assertEqual(random.draw_count, 800)
        self.assertEqual([len(points) for points in second.heads], [60, 60])
        self.assertEqual(len(np.unique(second.heads[0].indices)), 30)

    def test_heads_demo_runs_without_a_dataset(self):
        completed = subprocess.run(
            [sys.executable, "-m", "railway_mls", "heads-demo", "--seed", "1792"],
            text=True,
            capture_output=True,
            check=True,
        )
        result = json.loads(completed.stdout)
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["head_point_counts"], [60, 60])
        self.assertEqual(result["class_3_points"], 120)
        self.assertEqual(result["random_draws"], 400)
        self.assertFalse(result["tracking_implemented"])


if __name__ == "__main__":
    unittest.main()
