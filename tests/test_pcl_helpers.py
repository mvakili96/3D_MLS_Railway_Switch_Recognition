import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_array_equal

from railway_mls import (
    PclHeader,
    PclXYZCloud,
    PointCloud,
    center_point_cloud,
    decrease_density,
    PipelineOptions,
    run_pipeline,
)
from railway_mls.coverage import check_source_coverage, coverage_summary, pcl_reference
from railway_mls.synthetic import initialization_scene
from tests.cpp_pcl_reference import CppPclReference


def vector(values, cursor):
    count = int(values[cursor])
    return values[cursor + 1 : cursor + 1 + count], cursor + 1 + count


def compare_cloud(test, actual, values, cursor):
    count = int(values[cursor])
    test.assertEqual(len(actual), count)
    expected = [
        actual.width,
        actual.height,
        int(actual.is_dense),
        actual.header.sequence,
        actual.header.stamp,
        1,
        *actual.sensor_origin,
        *actual.sensor_orientation,
    ]
    assert_array_equal(expected, values[cursor + 1 : cursor + 15])
    xyz = np.array(values[cursor + 15 : cursor + 15 + count * 3], dtype=np.float32).reshape(-1, 3)
    assert_array_equal(actual.xyz, xyz)
    # Signed zeros are observable even when ordinary floating comparison is equal.
    finite = np.isfinite(xyz)
    assert_array_equal(actual.xyz[finite].view(np.uint32), xyz[finite].view(np.uint32))
    return cursor + 15 + count * 3


class PclHelperCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppPclReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def compare_voxel(self, cloud):
        native = self.reference.run("voxel", cloud)
        cursor = 6
        counts, cursor = vector(native, cursor)
        bins, cursor = vector(native, cursor)
        order, cursor = vector(native, cursor)
        fallback = bool(native[cursor])
        cursor += 1
        result = decrease_density(cloud, native_sorted_rows=[int(i) for i in order])
        self.assertIsNotNone(result.cloud, result.status)
        assert_array_equal(result.minimum, native[:3])
        assert_array_equal(result.maximum, native[3:6])
        assert_array_equal(result.extent_counts, counts)
        if bins:
            assert_array_equal(
                (*result.minimum_voxel, *result.maximum_voxel, *result.divisions), bins
            )
        self.assertEqual(result.status == "extent_fallback", fallback)
        self.assertEqual(compare_cloud(self, result.cloud, native, cursor), len(native))
        self.assertTrue(result.reference_comparable)
        return result, order

    def compare_center(self, cloud):
        native = self.reference.run("center", cloud)
        result = center_point_cloud(cloud)
        self.assertEqual(result.centroid_count, native[0])
        assert_array_equal(result.centroid, native[1:5])
        assert_array_equal(np.asarray(result.matrix).reshape(-1), native[5:21])
        self.assertEqual(compare_cloud(self, result.cloud, native, 21), len(native))
        return result

    def test_voxel_centroids_fixed_point_one_leaf_and_all_xyz_dimensions(self):
        result, _ = self.compare_voxel(
            PclXYZCloud([[0.01, 0.02, 0.03], [0.03, 0.04, 0.05], [0.2, 0.21, 0.22]])
        )
        self.assertEqual(result.groups, ((0, 1), (2,)))

    def test_voxel_keys_are_x_fast_then_y_then_z(self):
        result, _ = self.compare_voxel(
            PclXYZCloud([[1, 0, 0], [0, 0, 0.2], [0, 0.2, 0], [0, 0, 0]])
        )
        self.assertEqual(result.sorted_rows, (3, 0, 2, 1))

    def test_float32_boundaries_negative_coordinates_and_signed_zero(self):
        edge = np.float32(0.1)
        xs = [
            np.nextafter(edge, -np.inf, dtype=np.float32),
            edge,
            np.nextafter(edge, np.inf, dtype=np.float32),
            -edge,
            np.nextafter(-edge, -np.inf, dtype=np.float32),
            np.nextafter(-edge, np.inf, dtype=np.float32),
            -0.0,
            0.0,
        ]
        self.compare_voxel(PclXYZCloud([[v, 0, 0] for v in xs]))

    def test_float32_narrowing_precedes_voxel_assignment_at_large_world_coordinates(self):
        self.compare_voxel(
            PclXYZCloud(
                [
                    [1000000.01, 6000000.01, 100],
                    [1000000.02, 6000000.02, 100.01],
                    [1000000.17, 6000000.4, 100.1],
                ]
            )
        )
        # Importing std::floor into the oracle would round the relative index to
        # float32 and incorrectly merge the last two distinct cells here.
        result, _ = self.compare_voxel(
            PclXYZCloud([[-1000000, 0, 0], [1000000, 0, 0], [1000000.1, 0, 0]])
        )
        self.assertEqual(len(result.cloud), 3)

    def test_native_sort_permutation_is_replayed_before_float32_accumulation(self):
        cloud = PclXYZCloud([[0.09, 0, 0], *[[1e-9, 0, 0]] * 31])
        replay, order = self.compare_voxel(cloud)
        self.assertEqual(sorted(order), list(range(32)))
        stable = decrease_density(cloud)
        self.assertFalse(stable.reference_comparable)
        self.assertEqual(stable.sort_sensitive_voxels, (0,))
        self.assertTrue(replay.reference_comparable)

    def test_many_voxels_with_multiple_points_match_native_sort_and_accumulator(self):
        rng = np.random.default_rng(913)
        for count in (1, 15, 33, 128):
            self.compare_voxel(PclXYZCloud(rng.uniform(-0.4, 0.4, (count, 3))))

    def test_sparse_nonfinite_rows_are_skipped_and_output_is_dense_unorganized(self):
        cloud = PclXYZCloud(
            [[0.01, 0, 0], [np.nan, 0, 0], [0.03, 0, 0], [0, np.inf, 0]],
            is_dense=False,
            width=2,
            height=2,
        )
        result, _ = self.compare_voxel(cloud)
        self.assertEqual(result.groups, ((0, 2),))
        self.assertEqual(
            (result.cloud.width, result.cloud.height, result.cloud.is_dense), (1, 1, True)
        )

    def test_extent_fallback_returns_complete_input_and_organization_including_invalid_rows(self):
        cloud = PclXYZCloud(
            [[0, 0, 0], [1000, 1000, 1000], [np.nan, 0, 0], [1, 2, 3]],
            is_dense=False,
            width=2,
            height=2,
        )
        result, _ = self.compare_voxel(cloud)
        self.assertEqual(result.status, "extent_fallback")
        self.assertEqual(
            (result.cloud.width, result.cloud.height, result.cloud.is_dense), (2, 2, False)
        )
        assert_array_equal(result.cloud.xyz, cloud.xyz)

    def test_filter_and_transform_copy_header_and_sensor_metadata(self):
        cloud = PclXYZCloud(
            [[0.01, 0.02, 0.03], [0.03, 0.04, 0.05]],
            width=7,
            height=9,
            header=PclHeader(7, 12345, "frame/id"),
            sensor_origin=(1, 2, 3, 4),
            sensor_orientation=(0.1, 0.2, 0.3, 0.4),
        )
        down, _ = self.compare_voxel(cloud)
        centered = self.compare_center(cloud)
        self.assertEqual(down.cloud.header, cloud.header)
        self.assertEqual(centered.cloud.header, cloud.header)
        self.assertEqual((centered.cloud.width, centered.cloud.height), (7, 9))

    def test_centering_double_accumulation_float32_output_and_no_rotation(self):
        result = self.compare_center(
            PclXYZCloud(
                [
                    [1000000.125, 6000000.5, 100],
                    [1000000.25, 6000001, 101],
                    [1000000.5, 6000001.5, 103],
                ]
            )
        )
        self.assertEqual(result.cloud.xyz.dtype, np.float32)
        assert_array_equal(np.asarray(result.matrix)[:3, :3], np.eye(3))

    def test_centering_retains_full_four_term_affine_rounding_and_signed_zero(self):
        for xyz in (
            [[1, 2, 3]],
            [[-0.0, 0.0, -0.0], [0, -0.0, 0]],
            [[16777216, 1, 0], [1, 2, 0], [-16777216, 3, 0]],
        ):
            self.compare_center(PclXYZCloud(xyz))

    def test_empty_center_keeps_identity_vector_centroid_and_original_shape(self):
        result = self.compare_center(PclXYZCloud(np.empty((0, 3))))
        self.assertEqual(result.centroid, (1, 0, 0, 0))
        self.assertEqual(result.matrix[0][3], -1)
        self.assertEqual(result.status, "empty")

    def test_sparse_center_skips_invalid_rows_but_copies_them_unchanged(self):
        cloud = PclXYZCloud(
            [[1, 2, 3], [np.nan, 9, 10], [4, 8, 6], [np.inf, 11, 12]], is_dense=False
        )
        result = self.compare_center(cloud)
        self.assertEqual(result.centroid_count, 2)
        assert_array_equal(result.cloud.xyz[[1, 3]], cloud.xyz[[1, 3]])

    def test_all_invalid_sparse_center_has_nan_centroid_and_copied_rows(self):
        cloud = PclXYZCloud([[np.nan, 1, 2], [3, np.inf, 4]], is_dense=False)
        result = self.compare_center(cloud)
        self.assertEqual(result.centroid_count, 0)
        self.assertTrue(np.isnan(result.centroid[:3]).all())
        assert_array_equal(result.cloud.xyz, cloud.xyz)

    def test_dense_nonfinite_center_executes_zero_products_in_identity_affine(self):
        for invalid in (np.nan, np.inf, -np.inf):
            result = self.compare_center(PclXYZCloud([[invalid, 2, 3], [4, 5, 6]], is_dense=True))
            self.assertTrue(np.isnan(result.cloud.xyz[0]).all())
            self.assertEqual(result.status, "nonfinite_pcl_centering")

    def test_unsafe_voxel_cases_compare_only_the_original_bounds_prefix(self):
        for cloud in (
            PclXYZCloud(np.empty((0, 3))),
            PclXYZCloud([[np.nan, 1, 2]], is_dense=False),
            PclXYZCloud([[-1e38, 0, 0], [1e38, 0, 0]]),
            PclXYZCloud([[0, 0, 0], [1e7, 1e7, 1e7]]),
            PclXYZCloud([[3e8, 0, 0], [3e8, 0, 0]]),
            PclXYZCloud([[0.09, 0.09, 0], [4634.01, 4634.01, 0]]),
            PclXYZCloud([[0.09, 0.09, 0.09], [129.01, 129.01, 129.01]]),
        ):
            native = self.reference.run("prefix", cloud)
            result = decrease_density(cloud)
            assert_array_equal(result.minimum, native[:3])
            assert_array_equal(result.maximum, native[3:6])
            self.assertIsNone(result.cloud)
            self.assertTrue(result.status.startswith("undefined_source"))


class PclHelperContractTests(unittest.TestCase):
    def test_demo_runs_without_point_cloud_data_or_native_runtime(self):
        result = subprocess.run(
            [sys.executable, "-m", "railway_mls", "pcl-helpers-demo"],
            capture_output=True,
            text=True,
            check=True,
        )
        report = json.loads(result.stdout, parse_constant=lambda value: self.fail(value))
        self.assertEqual(report["status"], "ok")
        self.assertEqual(report["input_count"], 6)
        self.assertEqual(len(report["voxel"]["output"]["xyz"]), 3)
        self.assertEqual(report["centering"]["count"], 6)
        self.assertEqual(report["extent_fallback"]["status"], "extent_fallback")
        self.assertEqual(report["empty_voxel"]["status"], "undefined_source_voxel_extent_cast")
        self.assertEqual(report["empty_center"]["centroid"], [1, 0, 0, 0])
        self.assertFalse(report["native_sort_ties"]["reference_comparable"])
        self.assertFalse(report["automatic_processing_active"])

    def test_core_bridge_narrows_explicitly_and_does_not_carry_or_mutate_labels(self):
        core = PointCloud([[1000000.01, 2, 3]], intensity=[123], class_codes=[7], segment_ids=[9])
        cloud = PclXYZCloud.from_core(core)
        self.assertNotEqual(float(cloud.xyz[0, 0]), core.xyz[0, 0])
        self.assertEqual((cloud.width, cloud.height), (1, 1))
        restored = cloud.as_core()
        self.assertEqual(
            (restored.intensity[0], restored.class_codes[0], restored.segment_ids[0]), (0, 0, -1)
        )
        self.assertEqual((core.intensity[0], core.class_codes[0], core.segment_ids[0]), (123, 7, 9))
        with self.assertRaises(TypeError):
            decrease_density(core)

    def test_density_flag_is_explicit_and_nonfinite_core_conversion_is_rejected(self):
        cloud = PclXYZCloud([[np.nan, 1, 2]])
        self.assertTrue(cloud.is_dense)
        with self.assertRaises(ValueError):
            cloud.as_core()
        result = decrease_density(cloud)
        self.assertEqual(result.status, "unsupported_source_dense_nonfinite")
        self.assertFalse(result.reference_comparable)

    def test_empty_all_invalid_huge_extent_and_world_index_casts_are_guarded(self):
        cases = [
            (PclXYZCloud(np.empty((0, 3))), "undefined_source_voxel_extent_cast"),
            (PclXYZCloud([[np.nan, 1, 2]], is_dense=False), "undefined_source_voxel_extent_cast"),
            (PclXYZCloud([[0, 0, 0], [1e7, 1e7, 1e7]]), "undefined_source_voxel_extent_product"),
            (PclXYZCloud([[3e8, 0, 0]]), "undefined_source_voxel_bound_cast"),
            (
                PclXYZCloud([[0.09, 0.09, 0], [4634.01, 4634.01, 0]]),
                "undefined_source_voxel_division_product",
            ),
            (
                PclXYZCloud([[0.09, 0.09, 0.09], [129.01, 129.01, 129.01]]),
                "undefined_source_voxel_index_product",
            ),
        ]
        for cloud, status in cases:
            result = decrease_density(cloud)
            self.assertEqual(result.status, status)
            self.assertIsNone(result.cloud)
            self.assertFalse(result.reference_comparable)

    def test_null_inputs_report_source_dereference_without_fabricated_cloud(self):
        for helper in (center_point_cloud, decrease_density):
            result = helper(None)
            self.assertEqual(result.status, "undefined_source_null_pcl_cloud")
            self.assertIsNone(result.cloud)

    def test_supplied_sort_order_is_validated_and_can_expose_rounding_variation(self):
        cloud = PclXYZCloud([[0.09, 0, 0], *[[1e-9, 0, 0]] * 31])
        first = decrease_density(cloud)
        second = decrease_density(cloud, native_sorted_rows=range(31, -1, -1))
        self.assertFalse(first.reference_comparable)
        self.assertTrue(second.reference_comparable)
        self.assertNotEqual(first.cloud.xyz[0, 0], second.cloud.xyz[0, 0])
        with self.assertRaises(ValueError):
            decrease_density(cloud, native_sorted_rows=[0] * 32)
        with self.assertRaises(ValueError):
            decrease_density(PclXYZCloud([[0, 0, 0], [1, 0, 0]]), native_sorted_rows=[1, 0])

    def test_helpers_own_outputs_and_preserve_sensor_pose_without_transforming_it(self):
        cloud = PclXYZCloud([[1, 2, 3], [1.01, 2.01, 3.01]], sensor_origin=(10, 20, 30, 1))
        original = cloud.xyz.copy()
        centered = center_point_cloud(cloud)
        reduced = decrease_density(cloud)
        assert_array_equal(cloud.xyz, original)
        self.assertFalse(np.shares_memory(centered.cloud.xyz, cloud.xyz))
        self.assertFalse(np.shares_memory(reduced.cloud.xyz, cloud.xyz))
        assert_array_equal(centered.cloud.sensor_origin, cloud.sensor_origin)
        with self.assertRaises(ValueError):
            centered.cloud.xyz[0, 0] = 999

    def test_empty_metadata_defaults_and_uint_domains_remain_explicit(self):
        cloud = PclXYZCloud(np.empty((0, 3)))
        self.assertEqual((cloud.width, cloud.height, cloud.is_dense), (0, 0, True))
        assert_array_equal(cloud.sensor_origin, [0, 0, 0, 0])
        assert_array_equal(cloud.sensor_orientation, [0, 0, 0, 1])
        for values in ((-1, 0), (0, 1 << 64)):
            with self.assertRaises(ValueError):
                PclHeader(*values)
        with self.assertRaises(ValueError):
            PclXYZCloud([[0, 0, 0]], width=-1)

    def test_helpers_stay_out_of_loading_and_pipeline_scheduling(self):
        scene = initialization_scene()
        with (
            patch(
                "railway_mls.pcl_helpers.decrease_density", side_effect=AssertionError("inactive")
            ),
            patch(
                "railway_mls.pcl_helpers.center_point_cloud", side_effect=AssertionError("inactive")
            ),
        ):
            result = run_pipeline(
                scene.cloud,
                scene.seed,
                config=scene.config,
                options=PipelineOptions(bidirectional=False, max_steps=0),
            )
        self.assertEqual(result.initialization.status, "ok")

    def test_audit_has_no_pending_definitions_but_reconstruction_review_remains(self):
        report = coverage_summary()
        self.assertEqual(report["pending_definitions"], [])
        self.assertEqual(report["pending_file_workflows"], [])
        self.assertFalse(report["complete_reconstruction"])
        self.assertEqual(
            (report["pcl_helper_extract_count"], report["pcl_helper_dependency_file_count"]),
            (14, 19),
        )
        self.assertEqual(check_source_coverage(Path(__file__).resolve().parents[1]), ())
        self.assertIn("voxel_apply", pcl_reference()["extracts"])


if __name__ == "__main__":
    unittest.main()
