from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls import (
    ConvertedPoint,
    CoordinateConverter,
    PointCloud,
    UndefinedCoordinateArithmetic,
    arc_length_of_meridian,
    convert_legacy_files,
    deg_to_rad,
    ecef_to_geodetic,
    footpoint_latitude,
    format_converted_points,
    geodetic_to_ecef,
    lat_lon_to_utm_xy,
    map_lat_lon_to_xy,
    map_xy_to_lat_lon,
    rad_to_deg,
    utm_central_meridian,
    utm_xy_to_lat_lon,
    write_converted_points,
    source_menu_conversion,
)
from railway_mls.coordinates import CONVERTER_PI, UTM_PI
from tests.cpp_coordinate_reference import CppCoordinateReference


def scene(count=12):
    xyz = [
        geodetic_to_ecef(
            (52 + i * 0.01) * CONVERTER_PI / 180, (9 + i * 0.001) * CONVERTER_PI / 180, 100 + i
        )
        for i in range(count)
    ]
    return PointCloud(
        np.array(xyz).reshape(-1, 3),
        np.arange(count) * 100,
        class_codes=np.arange(count) % 4,
        segment_ids=np.arange(count) + 7,
    )


def point_values(points):
    return [
        len(points),
        *(v for p in points for v in (p.x, p.y, p.z, p.class_code, p.segment_id, p.intensity)),
    ]


def cloud_points(cloud):
    return [
        ConvertedPoint(*xyz, int(c), int(s), int(i))
        for xyz, c, s, i in zip(cloud.xyz, cloud.class_codes, cloud.segment_ids, cloud.intensity)
    ]


class CoordinateCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppCoordinateReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def compare(self, operation, function, cases):
        for args in cases:
            with self.subTest(operation=operation, args=args):
                assert_allclose(
                    function(*args),
                    self.reference.run(
                        operation, *map(lambda v: int(v) if isinstance(v, bool) else v, args)
                    ),
                    rtol=2e-14,
                    atol=1e-8,
                    equal_nan=True,
                )

    def test_degree_radian_helpers_keep_source_pi_and_signed_zero(self):
        for operation, function in (("deg", deg_to_rad), ("rad", rad_to_deg)):
            self.compare(operation, function, [(v,) for v in (-720, -180, -0.0, 0, 1, 180, 540)])
            self.assertEqual(
                np.signbit(function(-0.0)), np.signbit(self.reference.run(operation, -0.0)[0])
            )

    def test_arc_and_footpoint_series_including_signed_and_extreme_inputs(self):
        self.compare(
            "arc",
            arc_length_of_meridian,
            [(v,) for v in (-1.5, -0.9, 0, 0.3, 0.9, 1.5, float("inf"))],
        )
        self.compare(
            "foot", footpoint_latitude, [(v,) for v in (-1e7, -5e6, 0, 5e6, 1e7, float("inf"))]
        )

    def test_central_meridian_uses_supplied_int_zone_without_range_normalization(self):
        self.compare(
            "central", utm_central_meridian, [(v,) for v in (-100, 0, 1, 32, 60, 61, 2147483647)]
        )

    def test_forward_transverse_mercator_series(self):
        self.compare(
            "map",
            map_lat_lon_to_xy,
            [
                (0.9, 0.15, 0.157),
                (-0.6, 0.32, 0.31),
                (0, 0, 0),
                (1.4, 0.01, 0),
                (-1.4, -0.2, -0.19),
                (float("nan"), 0, 0),
            ],
        )

    def test_inverse_transverse_mercator_series(self):
        self.compare(
            "unmap",
            map_xy_to_lat_lon,
            [
                (0, 0, 0),
                (100000, 5700000, 0.15),
                (-200000, -4000000, 0.3),
                (1e8, 2e7, 0),
                (float("inf"), 0, 0),
            ],
        )

    def test_forward_utm_degrees_hemispheres_and_false_easting_northing(self):
        self.compare(
            "utm",
            lat_lon_to_utm_xy,
            [
                (52, 9, 32),
                (-33, 18, 34),
                (0, -177, 1),
                (80, 179, 60),
                (-79, -179, 1),
                (0, 9, 32),
                (-0.0, 9, 32),
                (float("nan"), 0, 0),
            ],
        )

    def test_seeded_projection_sweep_matches_native_forward_and_inverse(self):
        rng = np.random.default_rng(321)
        for lat, lon in zip(rng.uniform(-79, 83, 40), rng.uniform(-179, 179, 40)):
            zone, x, y = lat_lon_to_utm_xy(lat, lon, 0)
            assert_allclose(
                (zone, x, y), self.reference.run("utm", lat, lon, 0), rtol=2e-14, atol=1e-8
            )
            assert_allclose(
                utm_xy_to_lat_lon(x, y, zone, bool(lat < 0)),
                self.reference.run("unutm", x, y, zone, int(lat < 0)),
                rtol=2e-14,
                atol=1e-12,
            )

    def test_auto_zone_preserves_outside_range_endpoints_and_ignored_header_comment(self):
        self.compare(
            "utm",
            lat_lon_to_utm_xy,
            [(0, -180, 0), (0, 180, 0), (20, 9, -1), (-20, 9, 100), (0, -181, 61), (0, 540, 0)],
        )
        self.assertEqual(lat_lon_to_utm_xy(0, 180, 0)[0], 61)
        self.assertEqual(lat_lon_to_utm_xy(0, -181, 0)[0], 0)

    def test_inverse_utm_radians_and_explicit_hemisphere(self):
        self.compare(
            "unutm",
            utm_xy_to_lat_lon,
            [
                (500000, 0, 32, False),
                (500000, 1e7, 32, True),
                (219700, 6344700, 34, True),
                (219700, 6344700, 34, False),
                (500000, 1, 0, False),
            ],
        )

    def test_geodetic_to_ecef_radians_altitude_and_native_ieee_values(self):
        self.compare(
            "geo_ecef",
            geodetic_to_ecef,
            [
                (0, 0, 0),
                (0.9, 0.15, 100),
                (-0.6, -2, 1000),
                (CONVERTER_PI / 2, 0, -50),
                (0, CONVERTER_PI, 0),
                (float("inf"), 0, 0),
            ],
        )

    def test_ecef_to_geodetic_both_branches_poles_equator_and_origin(self):
        cases = [
            geodetic_to_ecef(lat, lon, h)
            for lat, lon, h in (
                (0, 0, 0),
                (0.2, -2, 10),
                (0.99, 1, 100),
                (1.2, -1, 1000),
                (-1.2, 1, -100),
            )
        ]
        cases += [
            (0, 0, 6356752.314),
            (0, 0, -6356752.314),
            (0, 0, 0),
            (1, 1, 1),
            (-6378137, -0.0, 0),
        ]
        self.compare("ecef_geo", ecef_to_geodetic, cases)

    def compare_conversion(self, cloud, actions, old=()):
        converter = CoordinateConverter(cloud)
        converter.converted_points = [replace(p) for p in old]
        payload = [*point_values(cloud_points(cloud)), *point_values(old), len(actions)]
        expected = [len(actions)]
        results = []
        for direction, zone, south, progress in actions:
            payload.extend((direction, zone, int(south), int(progress)))
            events = []

            def callback(current, total):
                events.extend((current, total, *point_values(converter.converted_points)))

            if direction == "ecef_to_utm":
                result = converter.convert_ecef_to_utm(
                    zone, callback=callback if progress else None
                )
            else:
                result = converter.convert_utm_to_ecef(
                    zone, south, callback=callback if progress else None
                )
            # Event count is separate: each includes a variable-size state vector.
            cursor, count = 0, 0
            while cursor < len(events):
                cursor += 3 + int(events[cursor + 2]) * 6
                count += 1
            expected.extend((*point_values(result.points), count, *events))
            results.append(result)
        actual = self.reference.run("convert", *payload)
        assert_allclose(expected, actual, rtol=2e-13, atol=1e-7, equal_nan=True)
        return converter, results

    def test_complete_ecef_conversion_progress_captures_append_geodetic_and_utm_state(self):
        self.compare_conversion(scene(), [("ecef_to_utm", 32, False, True)])

    def test_complete_inverse_conversion_with_both_hemispheres_and_callbacks(self):
        for south in (False, True):
            cloud = PointCloud(
                [[500000 + i * 10, 6344700 + i * 15, 100 + i] for i in range(12)], np.arange(12)
            )
            self.compare_conversion(cloud, [("utm_to_ecef", 34, south, True)])

    def test_both_no_callback_overloads_match_source(self):
        self.compare_conversion(scene(3), [("ecef_to_utm", 32, False, False)])
        self.compare_conversion(
            PointCloud([[500000, 5760000, 100]]), [("utm_to_ecef", 32, False, False)]
        )

    def test_repeated_and_mixed_direction_calls_reconvert_old_and_new_records(self):
        self.compare_conversion(scene(4), [("ecef_to_utm", 32, False, True)] * 3)
        self.compare_conversion(
            scene(4), [("ecef_to_utm", 32, False, False), ("utm_to_ecef", 32, False, True)]
        )

    def test_empty_input_still_reprocesses_preexisting_output_with_zero_total(self):
        self.compare_conversion(
            PointCloud(np.empty((0, 3))), [("ecef_to_utm", 32, False, True)], cloud_points(scene(5))
        )

    def test_source_origin_and_initialized_nonfinite_outputs_are_preserved(self):
        _, results = self.compare_conversion(
            PointCloud([[0, 0, 0], [0, 0, 6356752.314]]), [("ecef_to_utm", 32, False, True)]
        )
        self.assertEqual(results[0].status, "nonfinite_conversion")

    def test_export_all_overloads_fixed_precision_intensity_and_delimiters(self):
        points = [
            ConvertedPoint(1.2345, -1.2345, -0.0, 7, 9, 100),
            ConvertedPoint(500000.123456789, 6e6, 100, 4, 5, -7),
        ]
        for overload, delimiter in ((0, " "), (1, " "), (2, ","), (2, "::"), (2, "\t"), (2, "")):
            expected, events = self.reference.export(points, delimiter, overload)
            actual = format_converted_points(points, delimiter=delimiter)
            self.assertEqual(actual.text, expected)
            self.assertEqual(events, [0.0])

    def test_export_periodic_callbacks_have_no_forced_final_event(self):
        for n in (0, 9, 10, 11, 20):
            points = [ConvertedPoint(i, 2, 3, intensity=i) for i in range(n)]
            expected, events = self.reference.export(points)
            recorded = []
            actual = format_converted_points(
                points, callback=lambda cur, total: recorded.append((cur, total))
            )
            self.assertEqual(actual.text, expected)
            self.assertEqual(recorded, [(i, n) for i in range(10, n + 1, 10)])
            cursor = 1
            self.assertEqual(events[0], len(recorded))
            for current, total in recorded:
                self.assertEqual(events[cursor : cursor + 3], [current, total, n])
                cursor += 3 + n * 6
            self.assertEqual(cursor, len(events))


class CoordinateContractTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.folder = Path(self.directory.name)

    def test_pi_constants_and_unit_contracts_are_deliberately_distinct(self):
        self.assertNotEqual(UTM_PI, CONVERTER_PI)
        self.assertEqual(deg_to_rad(180), UTM_PI)
        lat, lon = utm_xy_to_lat_lon(500000, 5761038.212300773, 32, False)
        self.assertAlmostEqual(lat, 52 * CONVERTER_PI / 180, places=9)
        self.assertAlmostEqual(lon, 9 * CONVERTER_PI / 180, places=12)

    def test_input_stays_untouched_and_labels_are_copied_at_call_time(self):
        cloud = scene(2)
        converter = CoordinateConverter(cloud)
        cloud.class_codes[0], cloud.segment_ids[0], cloud.intensity[0] = 19, 29, 65535
        original = cloud.xyz.copy()
        result = converter.convert_ecef_to_utm(32)
        self.assertEqual(
            (result.points[0].class_code, result.points[0].segment_id, result.points[0].intensity),
            (19, 29, 65535),
        )
        assert_array_equal(cloud.xyz, original)
        converter.converted_points[0].x = 999
        self.assertNotEqual(result.points[0].x, 999)
        assert_array_equal(result.as_cloud().intensity, cloud.intensity)

    def test_repeated_conversion_progress_can_exceed_nominal_total_without_clamping(self):
        converter = CoordinateConverter(scene(4))
        converter.convert_ecef_to_utm(32)
        events = []
        converter.convert_ecef_to_utm(32, callback=lambda *v: events.append(v))
        self.assertEqual(events, [(10, 12), (20, 12)])

    def test_callback_failure_leaves_reached_mutable_buffer_state(self):
        converter = CoordinateConverter(scene())

        def fail(*_):
            raise RuntimeError("callback interrupted")

        with self.assertRaisesRegex(RuntimeError, "callback interrupted"):
            converter.convert_ecef_to_utm(32, callback=fail)
        self.assertEqual(len(converter.converted_points), 10)
        assert_array_equal(converter.converted_points[0].as_array(), converter.point_cloud.xyz[0])

    def test_undefined_automatic_zone_cast_is_explicit_and_preserves_prior_stage(self):
        for lon in (float("nan"), float("inf"), 1e30):
            with self.assertRaises(UndefinedCoordinateArithmetic):
                lat_lon_to_utm_xy(0, lon, 0)
        converter = CoordinateConverter(PointCloud([[0, 0, 0]]))
        converter.convert_ecef_to_utm(32)  # First call produces initialized NaN X/Y.
        with self.assertRaises(UndefinedCoordinateArithmetic):
            converter.convert_ecef_to_utm(0)
        self.assertTrue(np.isnan(converter.converted_points[0].x))
        with self.assertRaises(ValueError):
            utm_central_meridian(1 << 31)

    def test_nonfinite_state_and_export_remain_visible_without_fabricated_core_cloud(self):
        result = CoordinateConverter(PointCloud([[0, 0, 0]])).convert_ecef_to_utm(32)
        self.assertFalse(result.reference_comparable)
        with self.assertRaises(ValueError):
            result.as_cloud()
        export = format_converted_points(result.points)
        self.assertEqual(export.status, "nonfinite_coordinate_export")
        self.assertFalse(export.reference_comparable)

    def test_writer_flushes_before_callback_and_requires_explicit_overwrite(self):
        path = self.folder / "output.pts"
        points = cloud_points(scene())
        observed = []
        result = write_converted_points(
            path, points, callback=lambda *_: observed.append(len(path.read_text().splitlines()))
        )
        self.assertEqual(observed, [11])
        self.assertEqual(path.read_text(), result.text)
        with self.assertRaises(FileExistsError):
            write_converted_points(path, [])
        write_converted_points(path, [], overwrite=True)
        self.assertEqual(path.read_text(), "0\n")

    def test_batch_csv_dispatch_names_missing_files_and_phase_order(self):
        cloud = scene()
        source = self.folder / "input.csv"
        source.write_text(format_converted_points(cloud_points(cloud), delimiter=",").text)
        events = []
        result = convert_legacy_files(
            [self.folder / "missing.csv", source],
            self.folder,
            "ecef_to_utm",
            32,
            callback=lambda *v: events.append(("file", *v)),
            all_files_callback=lambda *v: events.append(("all", *v)),
        )
        self.assertEqual(result[0].status, "missing_source_skipped")
        self.assertEqual(result[1].destination.name, "input_utm.csv")
        self.assertEqual(
            [v for v in events if v[0] == "all"], [("all", 1, 6), ("all", 2, 6), ("all", 3, 6)]
        )
        self.assertLess(events.index(("file", 30, 36)), events.index(("all", 2, 6)))

    def test_source_batch_inverse_forces_north_and_advances_before_conversion(self):
        source = self.folder / "input.pts"
        points = [ConvertedPoint(500000, 6344700 + i, 100) for i in range(12)]
        source.write_text(format_converted_points(points).text)
        events = []
        result = convert_legacy_files(
            [source],
            self.folder,
            "utm_to_ecef",
            34,
            callback=lambda *v: events.append(("file", *v)),
            all_files_callback=lambda *v: events.append(("all", *v)),
        )[0]
        self.assertFalse(result.conversion.south_hemi)
        expected = geodetic_to_ecef(*utm_xy_to_lat_lon(500000, 6344700, 34, False), 100)
        assert_allclose(result.conversion.points[0].as_array(), expected)
        self.assertLess(events.index(("all", 2, 3)), events.index(("file", 10, 36)))

    def test_batch_fallback_is_legacy_csv_but_output_csv_requires_exact_suffix(self):
        source = self.folder / "input.CSV"
        source.write_text(format_converted_points(cloud_points(scene(1)), delimiter=",").text)
        result = convert_legacy_files([source], self.folder, "ecef_to_utm", 32)[0]
        self.assertEqual(result.loaded.source_format, "csv")
        self.assertEqual(result.destination.suffix, ".pts")
        self.assertNotIn(",", result.export.text)

    def test_source_inverse_menu_uses_zone_32_north_and_optional_separate_save(self):
        source = self.folder / "input.pts"
        source.write_text("1\n500000 5761038.212 100 7\n")
        result = source_menu_conversion(source, "utm_to_ecef")
        self.assertEqual(result.conversion.zone_argument, 32)
        self.assertFalse(result.conversion.south_hemi)
        self.assertIsNone(result.export)
        output = self.folder / "converted.pts"
        saved = source_menu_conversion(source, "utm_to_ecef", destination=output)
        self.assertEqual(saved.export.text, output.read_text())
        self.assertTrue(output.read_text().endswith(" 7\n"))

    def test_demo_cli_exposes_progress_repetition_and_typed_nonfinite_values(self):
        process = subprocess.run(
            [sys.executable, "-m", "railway_mls", "conversion-demo"],
            capture_output=True,
            text=True,
            check=True,
        )
        report = json.loads(process.stdout, parse_constant=lambda v: self.fail(v))
        self.assertEqual(report["status"], "ok")
        self.assertEqual(report["conversion_progress"], [[10, 36], [20, 36], [30, 36]])
        self.assertEqual(report["export_progress"], [[10, 12]])
        self.assertEqual(report["repeated_call"]["buffer_count"], 8)
        self.assertEqual(report["origin"]["status"], "nonfinite_conversion")
        self.assertEqual(len(report["nonfinite_values"]), 3)

    def test_file_cli_requires_inverse_hemisphere_and_preserves_existing_outputs(self):
        source = self.folder / "input.pts"
        source.write_text("1\n500000 5761038.212 100 7\n")
        output = self.folder / "out.pts"
        args = [
            sys.executable,
            "-m",
            "railway_mls",
            "convert-cloud",
            str(source),
            str(output),
            "--direction",
            "utm-to-ecef",
            "--zone",
            "32",
        ]
        process = subprocess.run(args, capture_output=True, text=True)
        self.assertEqual(process.returncode, 2)
        self.assertFalse(output.exists())
        process = subprocess.run(
            [*args, "--hemisphere", "north"], capture_output=True, text=True, check=True
        )
        self.assertEqual(json.loads(process.stdout)["status"], "ok")
        original = output.read_text()
        process = subprocess.run([*args, "--hemisphere", "north"], capture_output=True, text=True)
        self.assertEqual(process.returncode, 1)
        self.assertEqual(output.read_text(), original)

    def test_scoped_audit_includes_all_converter_projection_and_dialog_definitions(self):
        from railway_mls.coverage import check_source_coverage, coverage_summary

        report = coverage_summary()
        self.assertEqual(
            (
                report["definition_count"],
                report["engine_ui_definition_count"],
                report["coordinate_definition_count"],
            ),
            (246, 124, 27),
        )
        self.assertEqual(report["pending_definitions"], [])
        self.assertFalse(report["complete_reconstruction"])
        self.assertEqual(check_source_coverage(Path(__file__).resolve().parents[1]), ())

    def test_free_function_inventory_excludes_commented_and_string_lookalikes(self):
        from railway_mls.coverage import cpp_free_definitions

        text = """// FLOAT fake() {}
        const char* s="int fake2() {}";
        FLOAT First(FLOAT x) { return x; }
        /* void fake3() {} */
        void Second(FLOAT &x, FLOAT &y) { x=y; }
        """
        entries = cpp_free_definitions(text, "UTM")
        self.assertEqual([v["name"] for v in entries], ["First", "Second"])


if __name__ == "__main__":
    unittest.main()
