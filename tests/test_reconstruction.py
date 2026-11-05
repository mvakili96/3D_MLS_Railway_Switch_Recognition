from copy import deepcopy
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_array_equal

from railway_mls import (
    ExtractionConfig, InitialLocation, Point3D, PointCloud, PointXYZDouble,
    initial_location_dialog_defaults,
)
from railway_mls.config import source_int32
from railway_mls.coverage import (
    check_source_coverage, coverage_summary, cpp_definitions, cpp_function_calls,
    cpp_header_inventory, source_coverage,
)
from railway_mls.reconstruction import check_reconstruction_review, reconstruction_review
from tests.cpp_boundary_reference import CppBoundaryReference

ROOT = Path(__file__).resolve().parents[1]


class BoundaryTests(unittest.TestCase):
    def test_invalid_native_integer_casts_are_guarded(self):
        for value in (math.nan, math.inf, -math.inf, 2**31, -(2**31) - 1):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "undefined source"):
                source_int32(value)
        self.assertEqual(source_int32(2147483647.9), 2147483647)
        self.assertEqual(source_int32(-2147483648.9), -2147483648)

    def test_parameter_updates_preserve_non_parameter_settings_and_original(self):
        config = ExtractionConfig(grid_size=2.5, skip_rate=3)
        parameters = replace(config.to_source_parameters(), min_track_points=5.9, min_points=-20.9,
                             min_track_intensity=-3.4, max_track_intensity=1500.9)
        updated = config.with_source_parameters(parameters)
        self.assertEqual((updated.min_track_points, updated.min_points,
                          updated.min_track_intensity, updated.max_track_intensity), (5, -20, -3, 1500))
        self.assertEqual((updated.grid_size, updated.skip_rate), (2.5, 3))
        self.assertEqual(config.min_points, 20)

    def test_approximate_point_equality_never_changes_row_identity(self):
        a, b = PointXYZDouble(), PointXYZDouble(1e-6, 0, 0, class_code=7)
        self.assertEqual(a, b)
        self.assertFalse(b.is_null())
        self.assertNotEqual(a, PointXYZDouble(1e-5, 0, 0))
        cloud = PointCloud([[0, 0, 0], [1e-6, 0, 0], [0, 0, 0]])
        assert_array_equal(cloud.select().indices, [0, 1, 2])
        for value in (math.nan, math.inf, -math.inf):
            point = PointXYZDouble(value, 0, 0)
            self.assertFalse(point.is_equal(point))

    def test_dialog_adapter_drops_uninitialized_second_coordinates_and_point_tags(self):
        seed = InitialLocation(Point3D(1, 2, 3, 17), -45.9, 0, Point3D(4, 5, 6))
        result = initial_location_dialog_defaults(seed)
        self.assertEqual(result, InitialLocation(Point3D(1, 2, 3), 315, 1))
        self.assertEqual(seed.angle, -45.9)


class BoundaryCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppBoundaryReference()
        cls.addClassCleanup(cls.reference.close)

    def test_all_ten_parameter_fields_and_four_integer_narrowings(self):
        config = ExtractionConfig(grid_size=2.5, skip_rate=3)
        parameters = replace(config.to_source_parameters(), track_window_length=3.1,
                             track_window_width=.8, track_gauge=1.6, min_track_points=5.9,
                             min_points=-20.9, max_distance_for_ransac=.04, grouping_distance=.3,
                             mdl_weight=.5, min_track_intensity=-3.9, max_track_intensity=1500.9)
        updated = config.with_source_parameters(parameters)
        expected = self.reference.run("parameters", *asdict(parameters).values())
        assert_array_equal([updated.grid_size, updated.skip_rate,
                            *asdict(updated.to_source_parameters()).values()], expected)

    def test_point_constructors_copy_metadata_and_homogeneous_coordinate(self):
        for xyz in ((1.25, -2.5, 3.75), (0.0, -0.0, 0.0)):
            point = PointXYZDouble(*xyz, 3, 42, 250)
            copied = PointXYZDouble.from_point(point)
            default, initialized = PointXYZDouble(), PointXYZDouble(*xyz)
            expected = self.reference.run("point", *xyz, 3, 42, 250)
            actual = [initialized.data[3], initialized.class_code, initialized.segment_id,
                      initialized.intensity, copied.data[3], copied.x, copied.y, copied.z,
                      copied.class_code, copied.segment_id, copied.intensity,
                      *default.data, default.class_code, default.segment_id, default.intensity,
                      int(point.is_null())]
            assert_array_equal(actual, expected)
            copied.x = 99
            self.assertEqual(point.x, xyz[0])

    def test_point_tolerance_boundary_equality_and_exact_null(self):
        for xyz in ((0, 0, 0), (0, 0, -0.0), (np.nextafter(1e-5, 0), 0, 0), (1e-5, 0, 0),
                    (0, -1e-6, 0), (0, 0, -1e-5)):
            a, b = PointXYZDouble(), PointXYZDouble(*xyz)
            expected = self.reference.run("equal", *a.data[:3], *b.data[:3])
            assert_array_equal([a == b, a.is_equal(b), a.is_null()], expected)

    def test_active_seed_dialog_integer_angle_and_mode_normalization(self):
        for angle in (-720.9, -360.1, -45.9, -.9, 0, 359.9, 360.9, 725.1):
            for mode in (-1, 0, 1, 4, 5):
                seed = InitialLocation(Point3D(1, 2, 3), angle, mode)
                actual = initial_location_dialog_defaults(seed)
                assert_array_equal([actual.angle, actual.mode], self.reference.run("seed", angle, mode))


class ReconstructionReviewTests(unittest.TestCase):
    def test_review_completes_computational_scope_with_validation_explicitly_pending(self):
        self.assertEqual(check_source_coverage(ROOT), ())
        self.assertEqual(check_reconstruction_review(ROOT), ())
        report = coverage_summary()
        self.assertTrue(report["complete_reconstruction"])
        self.assertEqual(report["reconstruction_review_status"], "complete")
        self.assertEqual((report["definition_count"], report["support_definition_count"]), (265, 19))
        self.assertFalse(report["whole_application_parity_verified"])
        self.assertFalse(report["real_data_validation_complete"])
        self.assertFalse(report["limitations_review_complete"])
        self.assertEqual(reconstruction_review()["open_limitations"],
                         [f"LIM-{i:03d}" for i in range(1, 41)])

    def test_lexical_calls_exclude_comment_string_and_following_function_lookalikes(self):
        text = '''void Owner::first() {
          real(); if (x) { nested(); }
          // disabled();
          const char *s = "quoted(); }";
          /* absent(); */
        }
        void Owner::second() { later(); }
        '''
        entry = cpp_definitions(text, "Owner")[0]
        self.assertEqual(cpp_function_calls(text, entry), ("real", "if", "nested"))
        with self.assertRaisesRegex(ValueError, "unbalanced"):
            cpp_function_calls("void Owner::first() { real();", {"line": 1})

    def test_support_header_parser_preserves_pointer_return_and_equality_operator(self):
        entries, _ = cpp_header_inventory('''class Value {
          Owner *get(int x) { return nullptr; }
          bool operator ==(Value &other) { return true; }
        };''')
        self.assertEqual([item["id"] for item in entries], ["Value::get#1", "Value::operator==#1"])

    def check_changed_review(self, review):
        with patch("railway_mls.reconstruction.reconstruction_review", return_value=review):
            return check_reconstruction_review(ROOT)

    def test_missing_file_review_and_wrong_baseline_are_rejected(self):
        review = deepcopy(reconstruction_review())
        review["baseline_id"] = "different"
        review["file_reviews"].pop()
        errors = self.check_changed_review(review)
        self.assertIn("reconstruction review belongs to a different source baseline", errors)
        self.assertIn("reconstruction file reviews must cover each project source file once", errors)

    def test_missing_test_target_and_limitation_references_are_rejected(self):
        review = deepcopy(reconstruction_review())
        review["workflows"][0]["python"] = ["railway_mls.points:AbsentTarget"]
        review["workflows"][0]["tests"] = ["tests/nonexistent.py"]
        review["open_limitations"].pop()
        errors = self.check_changed_review(review)
        self.assertIn("missing workflow Python target: railway_mls.points:AbsentTarget", errors)
        self.assertIn("missing review test source: tests/nonexistent.py", errors)
        self.assertIn("reconstruction review and limitations register IDs differ", errors)

    def test_call_inventory_cannot_silently_drop_a_function_or_change_a_token(self):
        review = deepcopy(reconstruction_review())
        review["call_inventory"].pop()
        review["call_inventory"][0]["project_named_call_tokens"] = {}
        errors = self.check_changed_review(review)
        self.assertIn("reconstruction lexical call inventory differs from definition inventory", errors)
        self.assertIn("reviewed call tokens changed: RailwayExtraction::RailwayExtraction#1", errors)

    def test_call_witnesses_detect_missing_and_unexpected_active_calls(self):
        review = deepcopy(reconstruction_review())
        review["call_sites"] = [{"caller": "RailwayExtraction::RailwayExtraction#1",
                                 "must_call": ["DoesNotExist"], "must_not_call": ["DefineTemplates"]}]
        errors = self.check_changed_review(review)
        self.assertIn("missing reviewed call: RailwayExtraction::RailwayExtraction#1 -> DoesNotExist", errors)
        self.assertIn("inactive reviewed call was enabled: RailwayExtraction::RailwayExtraction#1 -> DefineTemplates", errors)

    def test_packaged_review_hash_is_checked(self):
        coverage = deepcopy(source_coverage())
        coverage["reconstruction_review"]["sha256"] = "tampered"
        with patch("railway_mls.coverage.source_coverage", return_value=coverage):
            self.assertIn("reconstruction review resource changed", check_source_coverage(ROOT))

    def test_audit_cli_reports_review_and_separate_validation_status(self):
        result = subprocess.run([sys.executable, "-m", "railway_mls", "reconstruction-audit",
                                 "--repo", str(ROOT)], capture_output=True, text=True, check=True)
        report = json.loads(result.stdout)
        self.assertEqual((report["status"], report["workflow_count"], report["call_site_count"]),
                         ("ok", 15, 29))
        self.assertFalse(report["real_data_validation_complete"])
        self.assertEqual(len(report["open_limitations"]), 40)


if __name__ == "__main__":
    unittest.main()
