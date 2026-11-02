import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls import (
    LegacyIOError,
    LegacyFileSession,
    LegacyLasReader,
    LegacyTextReader,
    Point3D,
    ReaderBounds,
    TrackLayer,
    format_rail_vector,
    load_point_cloud,
    write_layer_vectors,
    write_rail_vector,
)
from tests.cpp_io_reference import CppIOReference


def compare_read(test, result, events, native):
    count = int(native[0])
    test.assertEqual(count, len(result.cloud))
    assert_allclose(
        [*result.reader_bounds.minimum.as_array(), *result.reader_bounds.maximum.as_array()],
        native[1:7],
        rtol=0,
        atol=0,
    )
    values = np.array(native[7 : 7 + count * 6]).reshape(count, 6)
    assert_allclose(result.cloud.xyz, values[:, :3], rtol=0, atol=0)
    assert_array_equal(result.cloud.intensity, values[:, 3])
    assert_array_equal(result.cloud.class_codes, values[:, 4])
    assert_array_equal(result.cloud.segment_ids, values[:, 5])
    cursor = 7 + count * 6
    test.assertEqual(len(events), native[cursor])
    assert_array_equal(
        np.array(events).reshape(-1, 2), np.array(native[cursor + 1 :]).reshape(-1, 2)
    )


def make_las(path, count=21):
    import laspy

    header = laspy.LasHeader(version="1.2", point_format=3)
    header.scales = [0.01, 0.02, 0.001]
    header.offsets = [500000.0, 5000000.0, 100.0]
    las = laspy.LasData(header)
    raw = np.column_stack(
        (np.arange(count) * 17, np.arange(count)[::-1] * 19, np.arange(count) * -23)
    )
    if count > 1:
        raw[1] = [100000, -200000, 300000]  # Header extrema belong to a skipped record.
    las.X, las.Y, las.Z = raw.T
    intensity = (np.arange(count) * 6523 % 65536).astype(np.uint16)
    las.intensity = intensity
    las.classification = np.full(count, 9, dtype=np.uint8)
    las.write(path)
    return raw * header.scales + header.offsets, intensity


class LegacyIOCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppIOReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "points.pts"

    def compare_text(self, payload, format="pts", progress=True):
        self.path.write_text(payload)
        events = []
        result = LegacyTextReader().read(
            self.path, format=format, callback=lambda *v: events.append(v) if progress else None
        )
        # No-callback overload must really omit callbacks on the Python side.
        if not progress:
            events = []
            result = LegacyTextReader().read(self.path, format=format)
        compare_read(self, result, events, self.reference.read(format, payload, progress))
        assert_array_equal(result.record_indices, np.arange(len(result.cloud)))
        return result, events

    def test_pts_all_records_double_coordinates_labels_intensities_and_trailing_data(self):
        result, _ = self.compare_text(
            "3\n500000.123456789 6000000.123456789 9.25 0\n1 2 3 -17\n1 2 3 65535\nignored data\n"
        )
        self.assertEqual(len(result.cloud), 3)
        self.assertEqual(result.cloud.xyz[0, 0], 500000.123456789)

    def test_count_and_numeric_prefixes_and_collapsed_delimiters(self):
        self.compare_text(
            " +2 records\n  1.25suffix   -2e0suffix  junk  -7.8extra ignored\n3 4 5 +19tail\n"
        )
        self.compare_text("2\n,1.25,,2,3,4,unused\n5,6,7,8\n", format="csv")

    def test_progress_every_ten_then_unconditional_final_including_duplicate(self):
        for count in (0, 1, 9, 10, 11, 20, 21):
            payload = str(count) + "\n" + "1 2 3 4\n" * count
            _, events = self.compare_text(payload)
            self.assertEqual(
                events, [(i, count) for i in range(10, count + 1, 10)] + [(count, count)]
            )
        self.compare_text("1\n1 2 3 4\n", progress=False)

    def test_negative_and_non_numeric_count_read_no_records_and_keep_xy_sentinels(self):
        for header in ("-2", "garbage", "0"):
            result, _ = self.compare_text(header + "\n1 2 3 4\n")
            self.assertEqual(result.reader_bounds.minimum, Point3D(1e100, 1e100, 0))
            self.assertEqual(result.reader_bounds.maximum, Point3D(-1e100, -1e100, 0))

    def test_csv_fourth_column_becomes_intensity_without_importing_labels_or_index(self):
        self.compare_text("2\n1,2,3,2,99\n4,5,6,100,17\n", format="csv")

    def test_center_and_paired_export_exact_text_order_tags_and_rounding(self):
        center = [Point3D(1.2345, -1.2345, -0.0, 2), Point3D(500000.123456789, 6e6, 3, 7)]
        left = [Point3D(1, 2, 3, 88)]
        right = [Point3D(4, 5, 6, 99), Point3D(7, 8, 9)]
        self.assertEqual(format_rail_vector(center).text, self.reference.export("center", center))
        actual = format_rail_vector(center, left, right)
        self.assertEqual(actual.text, self.reference.export("paired", center, left, right))
        self.assertEqual(actual.declared_count, 5)
        self.assertTrue(actual.text.endswith("7.000,8.000,9.000,100\n"))
        self.assertEqual(format_rail_vector([]).text, self.reference.export("center", []))

    def test_indexed_export_skips_one_marker_uses_left_count_and_ignores_unused_tails(self):
        center = [Point3D(i, 2, 3, tag) for i, tag in enumerate((2, 2, 5, 9, 8))]
        left = [Point3D(i, 1, 3) for i in range(3)]
        right = [Point3D(i, 3, 3) for i in range(4)]
        result = format_rail_vector(center, left, right, indexed=True)
        self.assertEqual(result.text, self.reference.export("indexed", center, left, right))
        self.assertEqual(result.declared_count, 9)
        self.assertEqual(result.text.splitlines()[2], "1.000,2.000,3.000,2,0")

    @unittest.skipUnless(importlib.util.find_spec("laspy"), "optional LAS decoder not installed")
    def test_las_decoding_selection_header_extents_intensity_and_callbacks_across_odd_chunks(self):
        for count in (0, 1, 10, 21):
            xyz, intensity = make_las(self.path.with_suffix(".las"), count)
            for chunk_size in (1, 3, 10):
                events = []
                reader = LegacyLasReader()
                result = reader.read(
                    self.path.with_suffix(".las"),
                    callback=lambda *v: events.append(v),
                    chunk_size=chunk_size,
                )
                bounds = reader.read_bounds(self.path.with_suffix(".las"))
                tokens = [count, *bounds.minimum.as_array(), *bounds.maximum.as_array()]
                tokens.extend(v for row, value in zip(xyz, intensity) for v in (*row, value))
                compare_read(
                    self, result, events, self.reference.read("las", " ".join(map(str, tokens)))
                )
                assert_array_equal(result.cloud.xyz, xyz[::2])
                assert_array_equal(result.record_indices, np.arange(count)[::2])
                self.assertIs(reader.loaded_data(), result.cloud)

    @unittest.skipUnless(importlib.util.find_spec("laspy"), "optional LAS decoder not installed")
    def test_las_no_callback_overload_keeps_the_first_record_and_skipped_extrema(self):
        xyz, intensity = make_las(self.path.with_suffix(".las"), 3)
        reader = LegacyLasReader()
        result = reader.read(self.path.with_suffix(".las"), chunk_size=1)
        tokens = [
            3,
            *result.reader_bounds.minimum.as_array(),
            *result.reader_bounds.maximum.as_array(),
        ]
        tokens.extend(v for row, value in zip(xyz, intensity) for v in (*row, value))
        compare_read(
            self, result, [], self.reference.read("las", " ".join(map(str, tokens)), False)
        )
        self.assertGreater(result.reader_bounds.maximum.x, result.cloud.xyz[:, 0].max())


class LegacyIOContractTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "points.pts"

    def test_missing_fields_tabs_short_file_and_overlong_lines_are_explicit(self):
        cases = [
            ("1\n1 2 3\n", "undefined_source_missing_text_field"),
            ("1\n1\t2\t3\t4\n", "undefined_source_missing_text_field"),
            ("2\n1 2 3 4\n", "undefined_source_missing_text_record"),
            ("1\n" + "1" * 200 + "\n", "unsupported_source_text_line_length"),
            ("2147483648\n", "undefined_source_text_integer_range"),
            ("1\n1 2 3 2147483648\n", "undefined_source_text_integer_range"),
            ("1\nNaN 2 3 4\n", "nonfinite_legacy_coordinates"),
        ]
        for payload, status in cases:
            self.path.write_text(payload)
            with self.assertRaises(LegacyIOError) as caught:
                LegacyTextReader().read(self.path)
            self.assertEqual(caught.exception.status, status)

    def test_text_failure_retains_old_loaded_cloud_but_bounds_record_reached_progress(self):
        reader = LegacyTextReader()
        self.assertIsNone(reader.loaded_data())
        self.path.write_text("1\n1 2 3 4\n")
        old = reader.read(self.path)
        self.path.write_text("2\n5 6 7 8\nmissing\n")
        with self.assertRaises(LegacyIOError):
            reader.read(self.path)
        self.assertIs(reader.loaded_data(), old.cloud)
        self.assertEqual(reader.bounds.minimum, Point3D(5, 6, 0))

    def test_periodic_callback_sees_previous_cloud_and_final_sees_committed_cloud(self):
        reader = LegacyTextReader()
        self.path.write_text("1\n1 2 3 4\n")
        old = reader.read(self.path)
        self.path.write_text("10\n" + "2 3 4 5\n" * 10)
        seen = []
        new = reader.read(self.path, callback=lambda *args: seen.append(reader.loaded_data()))
        self.assertIs(seen[0], old.cloud)
        self.assertIs(seen[1], new.cloud)

    def test_final_reentrant_read_returns_outer_cloud_while_loaded_pointer_changes(self):
        reader = LegacyTextReader()
        self.path.write_text("2\n1 2 3 4\n2 3 4 5\n")
        other = self.path.with_name("other.pts")
        other.write_text("1\n5 6 7 8\n")
        result = reader.read(self.path, callback=lambda *_: reader.read(other))
        self.assertEqual(len(result.cloud), 2)
        self.assertEqual(len(reader.loaded_data()), 1)

    def test_grid_adaptation_preserves_reported_zero_z_and_xy_indices(self):
        self.path.write_text("3\n1 2 100 4\n2 3 -100 4\n3 4 50 4\n")
        result = load_point_cloud(self.path)
        self.assertEqual((result.reader_bounds.minimum.z, result.reader_bounds.maximum.z), (0, 0))
        self.assertEqual(
            (result.grid_bounds().minimum.z, result.grid_bounds().maximum.z), (-100, 100)
        )
        self.assertIs(result.make_grid().cloud, result.cloud)
        with self.assertRaises(ValueError):
            result.record_indices[0] = 9

    def test_pts_roundtrip_runs_existing_pipeline_with_identical_initialized_state(self):
        from railway_mls import PipelineOptions, run_pipeline
        from railway_mls.synthetic import initialization_scene

        scene = initialization_scene()
        self.path.write_text(
            str(len(scene.cloud))
            + "\n"
            + "".join(
                f"{x:.17g} {y:.17g} {z:.17g} {int(intensity)}\n"
                for (x, y, z), intensity in zip(scene.cloud.xyz, scene.cloud.intensity)
            )
        )
        loaded = load_point_cloud(self.path)
        assert_array_equal(loaded.cloud.xyz, scene.cloud.xyz)
        options = PipelineOptions(bidirectional=False, max_steps=0)
        actual = run_pipeline(
            loaded.cloud,
            scene.seed,
            config=scene.config,
            options=options,
            bounds=loaded.grid_bounds(),
        )
        expected = run_pipeline(scene.cloud, scene.seed, config=scene.config, options=options)
        self.assertEqual(actual.status, expected.status)
        self.assertEqual(actual.random_metadata, expected.random_metadata)
        self.assertEqual(
            actual.engine.layers[0].center_vector, expected.engine.layers[0].center_vector
        )
        assert_array_equal(
            actual.engine.grid.cloud.class_codes, expected.engine.grid.cloud.class_codes
        )

    def test_crlf_text_mode_and_bare_carriage_return_keep_native_delimiter_rules(self):
        self.path.write_bytes(b"1\r\n1 2 3 4\r\n")
        self.assertEqual(load_point_cloud(self.path).cloud.intensity[0], 4)
        self.path.write_bytes(b"1\n1\rjunk 2 3 4\n")
        self.assertEqual(load_point_cloud(self.path).cloud.xyz[0, 0], 1)

    def test_constructor_and_repeated_text_reads_preserve_non_xy_bound_fields(self):
        reader = LegacyTextReader()
        self.assertEqual(reader.bounds.minimum, Point3D(0, 0, 0))
        reader.bounds = ReaderBounds(Point3D(0, 0, 7), Point3D(0, 0, 9))
        self.path.write_text("1\n1 2 3 4\n")
        result = reader.read(self.path)
        self.assertEqual(result.reader_bounds.minimum.z, 7)
        self.assertEqual(result.reader_bounds.maximum.z, 9)

    def test_empty_result_and_unsupported_extensions_do_not_invent_data_or_grid(self):
        self.path.write_text("0\n")
        result = load_point_cloud(self.path)
        self.assertEqual(result.cloud.xyz.shape, (0, 3))
        with self.assertRaises(LegacyIOError):
            result.make_grid()
        with self.assertRaises(ValueError):
            load_point_cloud(self.path.with_suffix(".LAS"))
        self.assertEqual(load_point_cloud(self.path, format="csv").source_format, "csv")

    def test_199_character_record_and_extra_columns_are_supported(self):
        line = "1 2 3 4" + "x" * 192
        self.assertEqual(len(line), 199)
        self.path.write_text("1\n" + line)
        self.assertEqual(LegacyTextReader().read(self.path).cloud.intensity[0], 4)

    def test_undefined_indexed_export_reports_defined_prefix_without_writing(self):
        left = [Point3D(1, 2, 3)]
        for center, right, status, rows in (
            ([], [], "undefined_source_indexed_center", 1),
            ([Point3D(0, 0, 0, point_type=2)], [], "undefined_source_indexed_center", 1),
            ([Point3D(0, 0, 0)], [], "undefined_source_indexed_rail2", 2),
        ):
            result = format_rail_vector(center, left, right, indexed=True)
            self.assertEqual(result.status, status)
            self.assertEqual(result.rows_written, rows)
            self.assertEqual(result.failed_index, 0)
            with self.assertRaises(LegacyIOError):
                write_rail_vector(self.path, center, left, right, indexed=True)
            self.assertFalse(self.path.exists())

    def test_file_writers_preserve_logical_text_and_require_explicit_overwrite(self):
        center = [Point3D(1, 2, 3, 7)]
        result = write_rail_vector(self.path, center)
        self.assertEqual(self.path.read_bytes(), result.text.encode("ascii"))
        with self.assertRaises(FileExistsError):
            write_rail_vector(self.path, [])
        write_rail_vector(self.path, [], overwrite=True)
        self.assertEqual(self.path.read_text(), "0\n")

    def test_nonfinite_export_keeps_logical_output_with_explicit_comparison_status(self):
        result = write_rail_vector(self.path, [Point3D(float("nan"), float("inf"), 0)])
        self.assertEqual(result.status, "nonfinite_legacy_export")
        self.assertFalse(result.reference_comparable)
        self.assertEqual(self.path.read_text(), "1\nnan,inf,0.000,0\n")

    def test_layer_export_names_center_and_paired_files_and_indexed_only_pair(self):
        layer = TrackLayer(
            center_vector=[Point3D(1, 2, 3)],
            rail1_vector=[Point3D(0, 0, 0)],
            rail2_vector=[Point3D(0, 0, 0)],
        )
        base = self.path.with_suffix(".csv")
        paths = write_layer_vectors(base, layer)
        self.assertEqual([p.name for p in paths], ["points.csv", "points_with_track_points.csv"])
        indexed = base.with_name("indexed.csv")
        paths = write_layer_vectors(indexed, layer, indexed=True)
        self.assertFalse(indexed.exists())
        self.assertEqual(paths[0].name, "indexed_with_track_points.csv")

    def test_las_dependency_is_optional_and_has_an_actionable_install_error(self):
        from railway_mls.legacy_io import _laspy

        with patch.dict(sys.modules, {"laspy": None}):
            with self.assertRaisesRegex(ImportError, "pip install"):
                _laspy()

    def test_file_session_cache_preserves_engine_labels_layers_rng_and_exact_names(self):
        self.path.write_text("2\n1 2 3 4\n2 3 4 5\n")
        session = LegacyFileSession()
        first = session.load_file(self.path)
        first.engine.layers.append(TrackLayer(number=7))
        first.loaded.cloud.class_codes[0] = 3
        first.engine.rng.rand()
        other = self.path.with_name("other.pts")
        other.write_text(self.path.read_text())
        second = session.load_file(other)
        self.assertIs(second.engine.rng, first.engine.rng)
        self.path.unlink()  # Cached reentry does not touch the file.
        again = session.load_file(self.path, callback=lambda *_: self.fail("cached read"))
        self.assertIs(again, first)
        self.assertEqual(again.loaded.cloud.class_codes[0], 3)
        self.assertEqual(again.engine.layers[0].number, 7)
        self.assertEqual(second.engine.rng.draw_count, 1)
        unsupported = self.path.with_suffix(".CSV")
        self.assertIsNone(session.load_file(unsupported))
        self.assertIn(str(unsupported), session.files)
        self.assertIs(session.current, first)

    def test_direct_pts_menu_always_creates_fresh_uncached_engine(self):
        self.path.write_text("1\n1 2 3 4\n")
        session = LegacyFileSession()
        cached = session.load_file(self.path)
        cached.loaded.cloud.class_codes[0] = 7
        first, second = session.open_pts(self.path), session.open_pts(self.path)
        self.assertIsNot(first.engine, second.engine)
        self.assertEqual(second.loaded.cloud.class_codes[0], 0)
        self.assertIs(session.files[str(self.path)], cached)

    def test_new_cli_demo_and_inspection_run_with_no_database_or_las_dependency(self):
        process = subprocess.run(
            [sys.executable, "-m", "railway_mls", "legacy-io-demo"],
            capture_output=True,
            text=True,
            check=True,
        )
        report = json.loads(process.stdout)
        self.assertEqual(report["pts"]["loaded_count"], 12)
        self.assertEqual(report["csv"]["loaded_count"], 7)
        self.assertEqual(report["progress"], [[10, 12], [12, 12]])
        self.assertEqual(report["exports"]["indexed"]["rows_written"], 6)
        self.path.write_text("1\n1 2 3 4\n")
        process = subprocess.run(
            [sys.executable, "-m", "railway_mls", "inspect-cloud", str(self.path)],
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertEqual(json.loads(process.stdout)["loaded_count"], 1)

    def test_inventory_covers_all_reader_bodies_and_keeps_unused_pcl_helpers_pending(self):
        from railway_mls.coverage import check_source_coverage, coverage_summary

        report = coverage_summary()
        self.assertEqual(
            (
                report["definition_count"],
                report["engine_ui_definition_count"],
                report["legacy_io_definition_count"],
            ),
            (246, 124, 18),
        )
        self.assertEqual(
            {item["name"] for item in report["pending_definitions"]},
            {
                "DecreaseDensity",
                "Transform",
            },
        )
        self.assertEqual(check_source_coverage(Path(__file__).resolve().parents[1]), ())

    @unittest.skipUnless(importlib.util.find_spec("laspy"), "optional LAS decoder not installed")
    def test_laz_and_las_decode_identically_and_keep_format_independent_labels(self):
        import laspy

        if not laspy.LazBackend.detect_available():
            self.skipTest("LAZ backend unavailable")
        xyz, intensity = make_las(self.path.with_suffix(".laz"))
        result = load_point_cloud(self.path.with_suffix(".laz"), chunk_size=3)
        assert_array_equal(result.cloud.xyz, xyz[::2])
        assert_array_equal(result.cloud.intensity, intensity[::2])
        assert_array_equal(result.cloud.class_codes, 0)
        assert_array_equal(result.cloud.segment_ids, -1)

    @unittest.skipUnless(importlib.util.find_spec("laspy"), "optional LAS decoder not installed")
    def test_invalid_chunk_size_is_rejected_before_opening_las(self):
        for size in (0, -1):
            with self.assertRaises(ValueError):
                LegacyLasReader().read(self.path, chunk_size=size)


if __name__ == "__main__":
    unittest.main()
