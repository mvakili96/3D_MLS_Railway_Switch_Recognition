import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls.config import ExtractionConfig
from railway_mls.fitting import LineFitResult, fit_fixed_angle, fit_line_ransac, fit_loop_cross_line
from railway_mls.geometry import Line3D
from railway_mls.grouping import group_points
from railway_mls.mdl import segment_track_points_mdl
from railway_mls.points import PointCloud
from railway_mls.sampling import RecordedRand
from tests.cpp_reference import CppReference
from tests.line_fixtures import (
    component_cloud,
    draws,
    invalid_second_valid_third_cloud,
    parallel_cloud,
)


def read_group(values, position):
    count = int(values[position])
    return values[position + 1 : position + 1 + count], position + 1 + count


def endpoints(line):
    return [*line.begin.as_array(), *line.end.as_array()]


class FittingMDLCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def test_500_trial_models_support_sigma_and_random_consumption(self):
        for slope in (0.0, 0.2, -2.0):
            with self.subTest(slope=slope):
                xyz = [
                    (i * 0.05, slope * i * 0.05 + 0.007 * np.sin(i), 0.2 + i * 0.001)
                    for i in range(30)
                ] + [(100, 10, 99)]
                cloud = PointCloud(xyz)
                stream = draws(1000, 7)
                expected = self.reference.run(
                    "ransac500", len(cloud), 0.06, *cloud.xyz.ravel(), len(stream), *stream
                )
                random = RecordedRand(stream)
                actual = fit_line_ransac(cloud.select(), rng=random)
                self.assertEqual(expected[0], 1)
                self.assertTrue(actual.success)
                assert_allclose(endpoints(actual.line), expected[1:7], rtol=0, atol=2e-12)
                self.assertAlmostEqual(actual.sigma, expected[7], places=12)
                group, cursor = read_group(expected, 8)
                assert_array_equal(actual.support.indices, group)
                self.assertEqual(random.draw_count, expected[cursor])

    def compare_fixed(
        self, cloud, angle, *, stream=None, loop=False, skip_rate=1, selection=None, previous=None
    ):
        points = cloud.select(selection)
        stream = [0] * len(points) if stream is None else stream
        prior = [0]
        if previous is not None:
            prior = [1, *endpoints(previous.line), len(previous.support), *previous.support.indices]
        expected = self.reference.run(
            "loop_fixed" if loop else "fixed",
            len(cloud),
            angle,
            0.06,
            skip_rate,
            *cloud.xyz.ravel(),
            len(points),
            *points.indices,
            *prior,
            len(stream),
            *stream,
        )
        random = RecordedRand(stream)
        if loop:
            actual = fit_loop_cross_line(
                points, angle, rng=random, skip_rate=skip_rate, previous=previous
            )
        else:
            actual = fit_fixed_angle(points, angle, rng=random, previous=previous)
        self.assertEqual(actual.success, bool(expected[0]))
        cursor = 1
        if actual.success:
            assert_allclose(
                endpoints(actual.line), expected[cursor : cursor + 6], rtol=0, atol=2e-12
            )
            cursor += 6
        group, cursor = read_group(expected, cursor)
        assert_array_equal(actual.support.indices, group)
        if actual.success:
            members, cursor = read_group(expected, cursor)
            actual_members = actual.line.members.indices if actual.line.members is not None else []
            assert_array_equal(actual_members, members)
        self.assertEqual(random.draw_count, expected[cursor])
        return actual

    def test_fixed_angle_models_bounds_support_and_height_match_source(self):
        for angle in (0, 45, 90, 135):
            with self.subTest(angle=angle):
                if angle == 90:
                    xyz = [(0, i * 0.05, 0.2 + i * 0.001) for i in range(30)]
                else:
                    slope = np.tan(angle * np.pi / 180)
                    xyz = [(i * 0.05, slope * i * 0.05, 0.2 + i * 0.001) for i in range(30)]
                self.compare_fixed(PointCloud(xyz), angle)

    def test_fixed_angle_keeps_preexisting_outputs_when_no_new_model_wins(self):
        cloud = PointCloud([[10, 0, 0.2], [11, 0, 0.21]])
        line = Line3D(cloud.point(0), cloud.point(1), members=cloud.select())
        previous = LineFitResult(line, cloud.select(), 2, "ok")
        self.compare_fixed(cloud, 0, previous=previous)
        # Empty input returns false without clearing the supplied support vector.
        self.compare_fixed(cloud, 0, previous=previous, selection=[])

    def test_dormant_loop_cross_fitter_gates_match_without_activating_workflow(self):
        for count, spacing, skip in ((20, 0.03, 1), (19, 0.03, 1), (19, 0.03, 2), (20, 0.001, 1)):
            with self.subTest(count=count, spacing=spacing, skip=skip):
                xyz = [(i * spacing, 0, 0.2 + i * 0.001) for i in range(count)]
                self.compare_fixed(PointCloud(xyz), 0, loop=True, skip_rate=skip)
        weak = PointCloud(
            [(i * 0.03, 0, 0.2 + i * 0.001) for i in range(20)]
            + [(i * 0.03, 0.09, 0.3 + i * 0.001) for i in range(10)]
        )
        self.compare_fixed(weak, 0, loop=True)

    def compare_group(self, cloud, threshold, indices=None):
        points = cloud.select(indices)
        rows = []
        for row in range(len(cloud)):
            rows.extend([*cloud.xyz[row], int(cloud.class_codes[row]), int(cloud.segment_ids[row])])
        expected = self.reference.run(
            "group", len(cloud), threshold, *rows, len(points), *points.indices
        )
        actual = group_points(points, threshold)
        group, cursor = read_group(expected, 0)
        assert_array_equal(actual.selected.indices, group)
        assert_array_equal(cloud.segment_ids, expected[cursor : cursor + len(cloud)])
        assert_array_equal(cloud.class_codes, expected[cursor + len(cloud) :])

    def test_grouping_dfs_and_duplicate_pointer_identity_match(self):
        xyz = [[0, 0, 0], [0.1, 0, 100], [-0.1, 0, -100], [0.2, 0, 0], [0, 0, 0]]
        self.compare_group(PointCloud(xyz), 0.15)
        self.compare_group(PointCloud(xyz), 0.15, indices=[0, 1, 0, 3, 2])

    def test_component_retention_and_label_side_effects_match_source(self):
        for counts in ((2, 100), (2, 101), (102, 101)):
            with self.subTest(counts=counts):
                self.compare_group(component_cloud(counts), 0.015)

    def test_grouping_threshold_boundaries_and_empty_selection_match_source(self):
        for threshold in (0.15, np.nextafter(0.15, np.inf), 0):
            with self.subTest(threshold=threshold):
                self.compare_group(PointCloud([[0, 0, 0], [0.15, 0, 2]]), threshold)
        self.compare_group(component_cloud((3,)), 0.15, indices=[])

    def compare_mdl(self, cloud, *, angle=45, mode=1, weight=0.4, skip_rate=1, indices=None):
        points = cloud.select(indices)
        stream = draws(3000)
        expected = self.reference.run(
            "mdl",
            len(cloud),
            mode,
            angle,
            0.06,
            0.15,
            weight,
            skip_rate,
            *cloud.xyz.ravel(),
            len(points),
            *points.indices,
            len(stream),
            *stream,
        )
        random = RecordedRand(stream)
        result = segment_track_points_mdl(
            points,
            mode,
            angle,
            rng=random,
            config=ExtractionConfig(mdl_weight=weight, skip_rate=skip_rate),
        )
        self.assertTrue(result.reference_comparable)
        self.assertEqual(result.selected_count, int(expected[0]))
        cursor = 1
        for line in result.lines:
            assert_allclose(endpoints(line), expected[cursor : cursor + 6], rtol=0, atol=2e-12)
            members, cursor = read_group(expected, cursor + 6)
            actual_members = line.members.indices if line.members is not None else []
            assert_array_equal(actual_members, members)
        assert_allclose(
            [h.score for h in result.hypotheses],
            expected[cursor : cursor + 3],
            rtol=1e-12,
            atol=2e-12,
        )
        assert_allclose(
            [h.scaled_closeness for h in result.hypotheses],
            expected[cursor + 3 : cursor + 6],
            rtol=1e-12,
            atol=2e-12,
        )
        assert_array_equal(
            [h.eligible for h in result.hypotheses], expected[cursor + 6 : cursor + 9]
        )
        self.assertEqual(random.draw_count, expected[cursor + 9])
        assert_array_equal(cloud.segment_ids, expected[cursor + 10 :])
        return result

    def test_fully_initialized_native_mdl_can_select_all_three_model_counts(self):
        for weight, count in ((0, 1), (0.05, 2), (0.4, 3)):
            with self.subTest(weight=weight):
                actual = self.compare_mdl(parallel_cloud(), weight=weight)
                self.assertEqual(actual.selected_count, count)

    def test_mdl_orientation_modes_and_skip_rate_match_source(self):
        self.compare_mdl(parallel_cloud(angle=135), angle=135, mode=4)
        self.compare_mdl(parallel_cloud(), mode=3, skip_rate=2)

    def test_native_valid_third_candidate_includes_invalid_but_initialized_second(self):
        actual = self.compare_mdl(invalid_second_valid_third_cloud())
        self.assertEqual([h.eligible for h in actual.hypotheses], [True, False, True])

    def test_native_pointer_subtraction_preserves_duplicate_support_references(self):
        cloud = parallel_cloud()
        self.compare_mdl(cloud, indices=[*range(len(cloud)), 0, 0, 1])


if __name__ == "__main__":
    unittest.main()
