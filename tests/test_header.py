from copy import copy
from dataclasses import asdict
from pathlib import Path
import json
import math
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls import (
    GMMParameters,
    LegacyLiDPoint,
    Line3D,
    LoopCross,
    NeighborRegions,
    Point2D,
    Point3D,
    PointCloud,
    TrackLayer,
    TrackSegment,
    TrackWindow,
    higher_z_value,
    larger_group,
    source_float_round,
    source_is_nan,
)
from railway_mls.coverage import (
    SOURCE_DIRECTORY,
    check_source_coverage,
    coverage_summary,
    cpp_header_inventory,
)
from tests.cpp_reference import CppReference


def typed(points):
    return [
        len(points),
        *(value for point in points for value in (*point.as_array(), point.point_type)),
    ]


def fixture_line():
    line = Line3D(Point3D(4, 5, 6, 7), Point3D(1, 2, 3, 9))
    line.center = Point3D(100, 200, 300, 99)
    line.angle, line.a, line.b, line.c = 35, 2, -1, 4
    line._unit_vector = Point3D(1, 2, 3, 13)
    line.d3d = 42
    return line


def line_input(line):
    return [
        *typed((line.begin, line.end)),
        *line.center.as_array(),
        line.center.point_type,
        line.angle,
        line.a,
        line.b,
        line.c,
        *line.stored_unit_vector.as_array(),
        line.stored_unit_vector.point_type,
        line.d3d,
    ]


def line_output(line, flag=False, extended=()):
    return [
        int(flag),
        *typed((line.begin, line.end, line.center)),
        line.angle,
        line.a,
        line.b,
        line.c,
        *typed((line.stored_unit_vector,)),
        line.d3d,
        *typed(extended),
    ]


class HeaderCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def compare_line(self, action, args=(), *, line=None):
        line = fixture_line() if line is None else line
        before = line_input(line)
        flag, extended = False, ()
        if action == "coefficients":
            line.update_coefficients()
        elif action == "angle":
            line.update_angle()
        elif action == "parameters_at":
            line.set_parameters_from_angle_at(*args)
        elif action == "center":
            line.update_center()
        elif action in ("swap", "swapvertex"):
            line.swap_endpoints()
        elif action == "order":
            line.order_endpoints(*args)
        elif action == "reference":
            line.order_endpoints_by_reference(Point3D(*args))
        elif action == "vertex":
            flag = line.is_vertex(Point3D(*args))
        elif action == "extend":
            boundary = line.extend_endpoints(*args)
            extended = boundary.begin, boundary.end
            self.assertIsNone(boundary.a)
        elif action == "unit":
            line.update_unit_vector()
        elif action == "radius":
            line.set_ending_points_by_center_and_length(*args)
        elif action == "ending3d":
            line.set_ending_points_3d(*args)
        expected = self.reference.run("header_line", action, *before, *args)
        assert_allclose(
            line_output(line, flag, extended), expected, rtol=1e-14, atol=1e-12, equal_nan=True
        )
        return line

    def test_float_rounding_matches_positive_half_up_negative_truncation_and_float32_narrowing(
        self,
    ):
        for value in (
            -3.8,
            -1.5,
            -0.9,
            -0.0,
            0,
            0.49999999,
            0.5,
            1.49999999,
            1.5,
            2.6,
            2147483520,
            -2147483648,
        ):
            with self.subTest(value=value):
                self.assertEqual(
                    source_float_round(value), self.reference.run("header_round", value)[0]
                )

    def test_point3d_assignment_sign_set_and_normalization_keep_native_tags(self):
        for action in ("assign", "set", "sign", "normalize"):
            point, other = Point3D(3, 4, 0, 7), Point3D(8, 9, 10, 13)
            actual = {
                "assign": lambda: point.assigned_xyz(other),
                "set": lambda: point.set_coordinates(other.x, other.y, other.z),
                "sign": point.changed_sign,
                "normalize": point.normalized,
            }[action]()
            expected = self.reference.run("header_point3d", action, *typed((point, other)))
            assert_allclose(typed((actual,)), expected, rtol=0, atol=0)

    def test_point3d_zero_normalization_retains_nan_and_destination_tag(self):
        point = Point3D(0, 0, 0, 17)
        assert_allclose(
            typed((point.normalized(),)),
            self.reference.run("header_point3d", "normalize", *typed((point,))),
            equal_nan=True,
        )

    def test_point2d_arithmetic_and_zero_division_flag_transport(self):
        for action, scalar in (
            ("add", 2),
            ("subtract", 2),
            ("multiply", -2),
            ("divide", 2),
            ("divide", 0),
            ("default", 1),
        ):
            first, second = Point2D(3, 4, True), Point2D(3.000001, 4)
            actual = {
                "add": lambda: first + second,
                "subtract": lambda: first - second,
                "multiply": lambda: first * scalar,
                "divide": lambda: first / scalar,
                "default": Point2D,
            }[action]()
            expected = self.reference.run(
                "header_point2d", action, first.x, first.y, 1, second.x, second.y, scalar
            )
            assert_allclose(
                [
                    actual.x,
                    actual.y,
                    actual.flag,
                    first.is_equal(second),
                    first.is_exactly_equal(second),
                ],
                expected,
            )

    def test_legacy_coordinates_setters_arithmetic_defaults_and_unsigned_intensity(self):
        for action in ("add", "subtract", "set", "set_from"):
            first = LegacyLiDPoint(
                1,
                2,
                3,
                segment_id=7,
                class_code=9,
                intensity=70000,
                alpha=3,
                height_over_ground=4,
                delta_z=5,
                deleted=True,
            )
            other = LegacyLiDPoint(4, 5, 6)
            if action == "add":
                actual = first + other
            elif action == "subtract":
                actual = first - other
            else:
                actual = copy(first)
                if action == "set":
                    actual.set_xyz(*other.coordinates())
                else:
                    actual.set_from(other)
            expected = self.reference.run(
                "header_legacy", action, *first.coordinates(), *other.coordinates()
            )
            assert_allclose(
                [
                    *actual.coordinates(),
                    actual.class_code,
                    actual.segment_id,
                    actual.intensity,
                    actual.alpha,
                    actual.height_over_ground,
                    actual.delta_z,
                    actual.deleted,
                    actual.is_equal(other),
                ],
                expected,
            )

    def test_line_coefficients_and_angle_refresh_independently(self):
        self.compare_line("coefficients")
        self.compare_line("angle")
        for angle in (0, 90, 35):
            line = fixture_line()
            line.angle = angle
            self.compare_line("parameters_at", (7, 11), line=line)

    def test_line_swaps_and_mode_reference_order_keep_slot_tags_and_stale_caches(self):
        for action, args in (
            ("swap", ()),
            ("swapvertex", ()),
            *(("order", (mode,)) for mode in (0, 1, 2, 3, 4)),
            ("reference", (1, 2, 99)),
        ):
            self.compare_line(action, args)

    def test_line_vertex_membership_checks_z_and_swaps_only_on_end_match(self):
        for point in ((4, 5, 6), (1, 2, 3), (1.000001, 2, 3), (1, 2, 99), (8, 8, 8)):
            self.compare_line("vertex", point)

    def test_line_extension_refreshes_source_center_and_returns_endpoint_only_line(self):
        for times in (2.53, 0, -1):
            self.compare_line("extend", (times,))

    def test_line_center_and_unit_refresh_preserve_cache_tags(self):
        self.compare_line("center")
        self.compare_line("unit")
        line = fixture_line()
        line.end = Point3D(*line.begin.as_array(), 9)
        self.compare_line("unit", line=line)

    def test_line_3d_bounds_reset_z_except_zero_coefficients_and_preserve_other_fields(self):
        for a, b, c in ((0, -1, 4), (1, 0, -3), (-2, -1, 4), (2, -1, 4), (0, 0, 0)):
            line = fixture_line()
            line.a, line.b, line.c = a, b, c
            self.compare_line("ending3d", (-2, 5, -7, 9), line=line)

    def test_line_radius_uses_stored_center_and_literal_plus_minus_angle(self):
        for angle in (0, 90, 35, 215):
            line = fixture_line()
            line.angle = angle
            self.compare_line("radius", (0.5,), line=line)

    def test_neighbor_regions_match_all_five_source_polygons(self):
        for first, second in (
            (Point3D(1000, 2000, 10), Point3D(1000, 2001.5, 99)),
            (Point3D(1000, 2000, 10), Point3D(1001.2, 1999.1, 99)),
            (Point3D(0, 0, 1), Point3D(1.5, 0, 2)),
        ):
            segment = TrackSegment(TrackWindow(first, 35), TrackWindow(second, 35))
            regions = segment.generate_neighbor_regions(1.1, 0.5, 0.42)
            expected = self.reference.run(
                "header_neighbor", *first.as_array(), *second.as_array(), 1.1, 0.5, 0.42
            )
            actual = [
                value
                for polygon in (
                    regions.middle,
                    regions.middle_left,
                    regions.middle_right,
                    regions.left,
                    regions.right,
                )
                for value in typed(polygon)
            ]
            assert_allclose(actual, expected, rtol=1e-14, atol=1e-12)

    def test_neighbor_coincident_centers_keep_nonfinite_vertices_without_repair(self):
        point = Point3D(1, 2, 3)
        segment = TrackSegment(TrackWindow(point, 0), TrackWindow(point, 0))
        regions = segment.generate_neighbor_regions(1, 0.5, 0.42)
        expected = self.reference.run(
            "header_neighbor", *point.as_array(), *point.as_array(), 1, 0.5, 0.42
        )
        actual = [
            value
            for polygon in (
                regions.middle,
                regions.middle_left,
                regions.middle_right,
                regions.left,
                regions.right,
            )
            for value in typed(polygon)
        ]
        assert_allclose(actual, expected, equal_nan=True)
        self.assertEqual(regions.status, "nonfinite_neighbor_vertices")

    def test_member_bounds_empty_extreme_and_signed_zero_preserve_output_tags(self):
        for xyz in ([], [[-0.0, 0.0, 1], [0.0, -0.0, 3]], [[2e100, -2e100, 0]]):
            cloud = PointCloud(np.array(xyz).reshape(-1, 3))
            window = TrackWindow(Point3D(1, 2, 3), 35, members=cloud.select())
            maximum, minimum = Point3D(11, 12, 13, 7), Point3D(21, 22, 23, 9)
            actual = window.member_bounds(maximum, minimum)
            expected = self.reference.run(
                "header_bounds",
                len(cloud),
                *cloud.xyz.ravel(),
                len(cloud),
                *range(len(cloud)),
                *typed((maximum, minimum)),
            )
            assert_allclose(typed(actual), expected, rtol=0, atol=0)
            self.assertEqual(
                [math.copysign(1, x) for x in typed(actual)],
                [math.copysign(1, x) for x in expected],
            )

    def test_duplicate_pair_preserves_distance_regions_and_assignment_slot_tags(self):
        for old_vertices, new_vertices in (
            ((), (Point3D(5, 6, 7, 8),)),
            ((Point3D(1, 2, 3, 11), Point3D(4, 5, 6, 12)), (Point3D(7, 8, 9, 21),)),
        ):
            destination = duplicate_fixture(False, old_vertices)
            source = duplicate_fixture(True, new_vertices)
            assigned = destination.duplicate_from(source)
            self.assertTrue(assigned.reference_comparable)
            actual = [
                destination.distance_from_start,
                destination.gauge,
                *typed(
                    (
                        destination.stored_center,
                        destination.track1.center,
                        destination.track2.center,
                    )
                ),
                *typed(destination.track1.vertices),
                *typed(destination.neighbor_regions.middle),
                *typed((destination.track1.member_line.begin, destination.track1.member_line.end)),
                *typed(
                    (
                        destination.track1.member_line.center,
                        destination.track1.member_line.stored_unit_vector,
                    )
                ),
                destination.track1.member_line.angle,
                destination.track1.member_line.a,
                destination.track1.member_line.b,
                destination.track1.member_line.c,
                destination.track1.member_line.d3d,
                0,
            ]
            expected = self.reference.run(
                "header_duplicate", *typed(old_vertices), *typed(new_vertices)
            )
            assert_allclose(actual, expected, rtol=0, atol=0)


def duplicate_fixture(source, vertices):
    windows = [
        TrackWindow(
            Point3D(
                *(20 + i * 3, 21 + i * 3, 22 + i * 3)
                if source
                else (1 + i * 3, 2 + i * 3, 3 + i * 3),
                point_type=(30 if source else 10) + i,
            ),
            35,
        )
        for i in range(2)
    ]
    for i, window in enumerate(windows):
        window.member_line = Line3D(
            Point3D(1, 2, 3, (20 if source else 40) if i == 0 else 0),
            Point3D(4, 5, 6, (21 if source else 41) if i == 0 else 0),
        )
        window.member_line.center = Point3D(2.5, 3.5, 4.5)
        window.member_line.angle = 35
        window.member_line.a, window.member_line.b, window.member_line.c = 1, -1, 0
        window.member_line._unit_vector = Point3D(1, 0, 0)
        window.member_line.d3d = 42
    windows[0].vertices = vertices
    segment = TrackSegment(
        *windows,
        gauge=2 if source else 1,
        distance_from_start=5 if source else 99,
        stored_center=Point3D(10, 11, 12, 7) if source else Point3D(1, 2, 3, 9),
    )
    segment.neighbor_regions = NeighborRegions(
        middle=(Point3D(99, 98, 97),) if source else (Point3D(7, 8, 9),)
    )
    return segment


class HeaderContractTests(unittest.TestCase):
    def test_header_inventory_includes_all_methods_operators_types_and_excludes_comments(self):
        text = (SOURCE_DIRECTORY / "LIDARPT.h").read_text()
        methods, types = cpp_header_inventory(text)
        self.assertEqual((len(methods), len(types)), (77, 19))
        self.assertIn("LiDPOINT::operator/#1", {item["id"] for item in methods})
        self.assertNotIn("Rotation3D", {item["name"] for item in methods})
        self.assertNotIn("InverseRotation3D", {item["name"] for item in methods})
        summary = coverage_summary()
        self.assertEqual(
            (
                summary["definition_count"],
                summary["header_definition_count"],
                summary["header_type_count"],
            ),
            (201, 77, 19),
        )
        self.assertEqual(check_source_coverage(Path(__file__).resolve().parents[1]), ())

    def test_header_parser_ignores_nested_conditionals_calls_comments_and_string_bodies(self):
        text = """struct Item {\n void Set(int x) { if(x) { Set(1); } }\n Item() {}\n void operator=(Item x) {}\n};
        // inline void fake() {}
        /* struct Fake { void missing() {} }; */
        inline bool real(int x) { return x != 0; }
        """
        methods, types = cpp_header_inventory(text)
        self.assertEqual([item["name"] for item in types], ["Item"])
        self.assertEqual(
            [item["id"] for item in methods],
            ["Item::Set#1", "Item::Item#1", "Item::operator=#1", "LIDARPT::real#1"],
        )

    def test_header_source_drift_reports_new_inline_definitions_and_types(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / SOURCE_DIRECTORY / "LIDARPT.h"
            path.parent.mkdir(parents=True)
            path.write_text(
                (SOURCE_DIRECTORY / "LIDARPT.h").read_text()
                + "\nstruct NewState {\n void NewComputation() {}\n};\n"
            )
            errors = check_source_coverage(root)
            self.assertIn("definition inventory differs: LIDARPT.h", errors)
            self.assertIn("type inventory differs: LIDARPT.h", errors)
            self.assertIn("source changed: LIDARPT.h", errors)

    def test_header_source_drift_reports_unbalanced_braces_without_inventing_coverage(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / SOURCE_DIRECTORY / "LIDARPT.h"
            path.parent.mkdir(parents=True)
            path.write_text("struct Broken {\n void method() {\n")
            self.assertIn(
                "cannot inventory malformed source: LIDARPT.h", check_source_coverage(root)
            )

    def test_legacy_undefined_division_and_uninitialized_coordinates_are_explicit(self):
        point = LegacyLiDPoint()
        self.assertIsNone(point.x)
        with self.assertRaisesRegex(ValueError, "coordinates"):
            point + LegacyLiDPoint(1, 2, 3)
        with self.assertRaisesRegex(ValueError, "undefined_source_lidpoint_division"):
            LegacyLiDPoint(1, 2, 3) / 2

    def test_float_round_rejects_undefined_integer_conversions(self):
        for value in (float("nan"), float("inf"), 2147483648, -2147483904):
            with self.assertRaisesRegex(ValueError, "undefined source"):
                source_float_round(value)

    def test_point2d_constructor_flags_and_zero_division_value_copy(self):
        self.assertFalse(Point2D().flag)
        point = Point2D(1, 2)
        self.assertIsNone(point.flag)
        result = point / 0
        self.assertIsNone(result.flag)
        self.assertIsNot(point, result)
        self.assertFalse((point * 1).flag)

    def test_stored_unit_vector_and_center_cache_require_explicit_initialization(self):
        line = Line3D(Point3D(0, 0, 0), Point3D(1, 0, 0))
        self.assertIsNone(line.stored_unit_vector)
        with self.assertRaisesRegex(ValueError, "center cache"):
            line.set_ending_points_by_center_and_length(1)
        vector = line.update_unit_vector()
        line.end = Point3D(0, 1, 0)
        self.assertEqual(line.stored_unit_vector, vector)
        self.assertNotEqual(line.unit_vector, vector)

    def test_duplicate_flags_unknown_capacity_when_tag_transport_can_differ(self):
        destination = duplicate_fixture(False, (Point3D(1, 2, 3, 7),))
        source = duplicate_fixture(True, (Point3D(4, 5, 6, 9), Point3D(7, 8, 9, 11)))
        result = destination.duplicate_from(source)
        self.assertFalse(result.reference_comparable)
        self.assertEqual(result.status, "capacity_dependent_tags")

    def test_window_reset_keeps_cloud_ownership_side_data_and_empty_gmm_input(self):
        cloud = PointCloud([[1, 2, 3]])
        points = cloud.select()
        window = TrackWindow(Point3D(1, 2, 3), 35, members=points, left_members=points)
        window.reset()
        self.assertEqual(len(window.members), 0)
        self.assertIs(window.members.cloud, cloud)
        self.assertIs(window.left_members, points)
        assert_array_equal(points.indices, [0])

    def test_remaining_header_containers_defaults_comparators_and_assignment(self):
        first, second = TrackLayer(), TrackLayer()
        first.loop_crosses.append(LoopCross())
        self.assertEqual(second.loop_crosses, [])
        self.assertIsNone(first.loop_crosses[0].line)
        self.assertTrue(source_is_nan(float("nan")))
        self.assertFalse(source_is_nan(float("inf")))
        self.assertFalse(higher_z_value(Point3D(0, 0, 1), Point3D(1, 2, 1)))
        cloud = PointCloud([[1, 2, 3]])
        self.assertTrue(larger_group(cloud.select([0, 0]), cloud.select([0])))
        parameters = GMMParameters()
        parameters.assign_from(GMMParameters(1, 2, 3))
        self.assertEqual(asdict(parameters), {"mu": 1, "sigma": 2, "alpha": 3})

    def test_header_demo_runs_without_data_and_exposes_source_policies(self):
        process = subprocess.run(
            [sys.executable, "-m", "railway_mls", "header-demo"],
            capture_output=True,
            text=True,
            check=True,
        )
        result = json.loads(process.stdout)
        self.assertEqual(result["rounding"], [-1, 1, 2])
        self.assertEqual(result["normalized_marked_point"]["point_type"], 17)
        self.assertEqual(result["endpoint_tags_after_swap"], [7, 9])
        self.assertEqual(result["retained_distance"], 99)
        self.assertEqual(result["neighbor_status"], "ok")
        self.assertIsNone(result["legacy_default"]["x"])
        self.assertEqual(result["source_coverage"]["header_definition_count"], 77)


if __name__ == "__main__":
    unittest.main()
