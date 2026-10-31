import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls.fitting import fit_rail_head_line
from railway_mls.gmm import classify_window, cluster_by_gmm, fit_height_gmm
from railway_mls.heads import extract_track_points
from railway_mls.models import GMMParameters
from railway_mls.points import PointCloud
from railway_mls.sampling import RecordedRand
from tests.cpp_reference import CppReference
from tests.test_gmm_heads import paired_segment, rail_points, raw_draws


def read_group(values, position):
    count = int(values[position])
    return values[position + 1 : position + 1 + count], position + 1 + count


class GMMCppParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def test_gmm_parameters_and_ordered_memberships_match_source(self):
        random = np.random.default_rng(21)
        samples = [
            np.full(12, 0.2),
            np.array([0, 0.06, 0.12]),
            np.array([0, 100]),
            np.tile([0, 0.1, 0.2], 20),
            np.concatenate([random.normal(mean, 0.01, 30) for mean in (0, 0.1, 0.2)]),
            np.concatenate([random.normal(mean, 0.08, 25) for mean in (-1, 0, 1)]),
            np.array([-1e50, 1e50]),
        ]
        for sample in samples:
            with self.subTest(span=float(np.ptp(sample)), count=len(sample)):
                points = PointCloud(
                    np.column_stack((np.arange(len(sample)), np.zeros(len(sample)), sample))
                ).select()
                actual = classify_window(points)
                expected = self.reference.run("gmm", len(sample), 3, *sample)
                parameters = [[p.mu, p.sigma, p.alpha] for p in actual.fit.parameters]
                assert_allclose(
                    parameters,
                    np.array(expected[:9]).reshape(3, 3),
                    rtol=1e-10,
                    atol=1e-12,
                    equal_nan=True,
                )
                cursor = 9
                for cluster in actual.clustering.clusters:
                    group, cursor = read_group(expected, cursor)
                    assert_array_equal(cluster.indices, group)
                self.assertEqual(cursor, len(expected))

    def test_source_hardcoded_first_three_checks_with_four_components(self):
        sample = np.array([-0.5, -0.49, -0.1, -0.09, 0.5, 0.51, 0.9, 0.91])
        actual = fit_height_gmm(sample, n_components=4)
        expected = self.reference.run("gmm", len(sample), 4, *sample)
        assert_allclose(
            [[p.mu, p.sigma, p.alpha] for p in actual.parameters],
            np.array(expected[:12]).reshape(4, 3),
            rtol=1e-10,
            atol=1e-12,
        )

    def test_classifier_ties_underflow_and_partial_nan_behavior(self):
        cases = [
            ([0, 0.1, 1e4], [GMMParameters(0, 1, 1 / 3)] * 3),
            ([0.2, 0, 0.1], [GMMParameters(mu, 0.001, 1 / 3) for mu in (0.2, 0, 0.1)]),
            ([0, 100, 0], [GMMParameters(0, 1, float("inf")), GMMParameters(0, 1, 1)]),
            ([0, 1], [GMMParameters(float("nan"), 1, 1), GMMParameters(0, 1, 1)]),
            ([0, 1], [GMMParameters(0, 1, 1), GMMParameters(float("nan"), 1, 1)]),
        ]
        for heights, parameters in cases:
            with self.subTest(heights=heights, parameters=parameters):
                points = PointCloud([[0, 0, z] for z in heights]).select()
                actual = cluster_by_gmm(points, parameters)
                expected = self.reference.run(
                    "cluster",
                    len(heights),
                    len(parameters),
                    *heights,
                    *[value for p in parameters for value in (p.mu, p.sigma, p.alpha)],
                )
                cursor = len(parameters) * 3
                for cluster in actual.clusters:
                    group, cursor = read_group(expected, cursor)
                    assert_array_equal(cluster.indices, group)

    def test_100_trial_ransac_matches_given_the_same_raw_draws(self):
        draws = raw_draws(200, 7)
        cases = [
            [(i * 0.05, 0, i * 0.001) for i in range(20)] + [(100, 10, 99)],
            [(i * 0.05, -2 * i * 0.05, 0.2 + i * 0.001) for i in range(20)] + [(10, 10, 0.5)],
            [(2, i * 0.05, 0.2 + i * 0.001) for i in range(20)],
        ]
        for xyz in cases:
            with self.subTest(xyz=xyz[:2]):
                cloud = PointCloud(xyz)
                random = RecordedRand(draws)
                actual = fit_rail_head_line(cloud.select(), rng=random)
                expected = self.reference.run(
                    "ransac", len(xyz), 0.04, *np.array(xyz).ravel(), len(draws), *draws
                )
                self.assertTrue(actual.success)
                self.assertEqual(expected[0], 1)
                assert_allclose(
                    [*actual.line.begin.as_array(), *actual.line.end.as_array()],
                    expected[1:7],
                    rtol=0,
                    atol=1e-12,
                )
                self.assertEqual(random.draw_count, expected[7])

    def compare_heads(self, first, second, *, existing_rows=((), ()), tied_heights=False):
        cloud, segment = paired_segment(first, second)
        draws = raw_draws()
        old = tuple(cloud.select(rows) for rows in existing_rows)
        # The C++ protocol reads integer fields with >> int, not floating literals.
        inputs = []
        for row in range(len(cloud)):
            inputs.extend([*cloud.xyz[row], int(cloud.class_codes[row]), int(cloud.intensity[row])])
        expected = self.reference.run(
            "heads",
            len(first),
            len(second),
            0.2,
            0.2,
            *inputs,
            len(old[0]),
            *old[0].indices,
            len(old[1]),
            *old[1].indices,
            len(draws),
            *draws,
        )
        random = RecordedRand(draws)
        actual = extract_track_points(segment, (0.2, 0.2), rng=random, existing_heads=old)
        assert_allclose(actual.next_heights, expected[:2], rtol=0, atol=1e-12)
        cursor = 2
        for group in actual.heads:
            ids, cursor = read_group(expected, cursor)
            if tied_heights:
                # std::sort has unspecified ordering for equal-height distinct IDs.
                assert_array_equal(np.sort(group.indices), np.sort(ids))
            else:
                assert_array_equal(group.indices, ids)
        assert_array_equal(cloud.class_codes, expected[cursor : cursor + len(cloud)])
        self.assertEqual(random.draw_count, expected[cursor + len(cloud)])
        return actual

    def test_complete_extraction_heights_order_and_shared_labels(self):
        actual = self.compare_heads(rail_points(0), rail_points(1.5))
        self.assertEqual(actual.status, "ok")

    def test_retry_accumulation_matches_source(self):
        first, second = rail_points(0), rail_points(1.5)
        self.compare_heads(first, second, existing_rows=((2, 2, 0), (len(first) + 2,)))

    def test_tied_height_membership_without_claiming_std_sort_tie_order(self):
        first = [(i * 0.02, 0, 0.2) for i in range(30)]
        second = [(i * 0.02, 1.5, 0.2) for i in range(30)]
        actual = self.compare_heads(first, second, tied_heights=True)
        self.assertEqual([len(points) for points in actual.heads], [30, 30])

    def test_defined_no_head_path_returns_both_previous_heights(self):
        first = [(0, 0, z) for z in np.tile([0, 0.1, 0.2], 20)]
        actual = self.compare_heads(first, rail_points(1.5))
        self.assertEqual(actual.status, "no_head_points")


if __name__ == "__main__":
    unittest.main()
