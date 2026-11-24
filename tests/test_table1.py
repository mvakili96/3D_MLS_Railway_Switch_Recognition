from copy import deepcopy
import csv
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

from railway_mls.evaluation import RailSegments, ReferencePoints, evaluate_rails
from railway_mls.evaluation.core import source_error_statistics
from railway_mls.evaluation.io import file_fingerprint, iter_reference_las, read_rail_segments
from railway_mls.evaluation.table1 import (_ResidualPool, iter_path_reference,
                                           run_table1_report, segment_length_xy)
from tests.cpp_evaluation_reference import CppEvaluationReference
from tests import test_benchmark as benchmark_fixtures


class TableStatisticsTests(unittest.TestCase):
    def test_length_uses_xy_and_existing_segments_without_bridging_fragments(self):
        left = RailSegments([[[0, 0, 10], [3, 4, 30]], [[20, 0, 0], [23, 4, 100]]])
        right = RailSegments([[[0, 1, 0], [0, 7, 50]]])
        self.assertEqual(segment_length_xy(left, right), 8)
        self.assertEqual(segment_length_xy(RailSegments([]), RailSegments([])), 0)

    def test_pooled_statistics_preserve_per_path_assignments_and_source_signed_vertical_rule(self):
        reference = ReferencePoints([[.2, .05, -.02], [.8, .1, -.04]], np.array([3, 3]))
        a = evaluate_rails(reference, RailSegments([[[0, 0, 0], [1, 0, 0]]]), RailSegments([]), collect_points=True)
        b = evaluate_rails(reference, RailSegments([[[0, .15, -.06], [1, .15, -.06]]]), RailSegments([]), collect_points=True)
        with tempfile.TemporaryDirectory() as temp:
            pool = _ResidualPool(Path(temp)/"pool")
            pool.append(2, b.points)
            pool.append(1, a.points)  # Reduced in path order, independent of append order.
            blocks = list(pool.blocks())
            assert_array_equal(blocks[0][:, 1], a.points["signed_vertical"])
            assert_array_equal(blocks[1][:, 1], b.points["signed_vertical"])
            result = source_error_statistics(pool.blocks, 4)
            values = np.concatenate(blocks)
            self.assertAlmostEqual(result["mean_absolute_vertical"], np.mean(np.abs(values[:, 1])))
            mean = np.mean(np.abs(values[:, 1]))
            self.assertAlmostEqual(result["std_vertical_source"], np.sqrt(np.sum((mean-values[:, 1])**2)/3))
            self.assertNotAlmostEqual(result["std_vertical_source"], np.std(values[:, 1], ddof=1))

    def test_empty_and_single_positive_source_ieee_statistics_are_preserved(self):
        empty = source_error_statistics(lambda: iter(()), 0)
        self.assertTrue(np.isnan(empty["mean_horizontal"]))
        singleton = source_error_statistics(lambda: iter([np.array([[.03, -.04]])]), 1)
        self.assertTrue(np.isnan(singleton["std_horizontal"]))
        self.assertTrue(np.isinf(singleton["std_vertical_source"]))


@unittest.skipUnless(importlib.util.find_spec("laspy"), "optional LAS support")
class TableReportTests(unittest.TestCase):
    def setUp(self):
        self.fixture = benchmark_fixtures.BenchmarkTests(methodName="runTest")
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.root = self.fixture.root
        self.mapping = None

    def freeze(self, paths=(3,)):
        for pid in paths:
            self.fixture.document["inference"]["runs"][pid-1].update(status="ready", seed=self.fixture.seeds[pid])
        self.fixture.save()
        self.fixture.predict()
        self.mapping = self.fixture.review(["batch"])

    def report(self, name="table", paths=(3,), **kwargs):
        if self.mapping is None:
            self.freeze(paths)
        return run_table1_report(self.fixture.manifest, [self.root/"batch"], self.mapping,
                                  self.root/name, path_ids=list(paths), **kwargs)

    def test_complete_path_row_recomputes_all_parts_and_length_once(self):
        report = self.report(chunk_size=2)
        self.assertEqual(report["status"], "completed")
        self.assertTrue(report["complete_requested_scope"])
        self.assertFalse(report["complete_experiment_scope"])
        self.assertFalse(report["full_spatial_coverage"])
        self.assertEqual(report["requested_path_ids"], [3])
        self.assertEqual(len(report["rows"]), 2)
        python, original = report["rows"]
        self.assertAlmostEqual(original["length_xy_m"], 4, places=9)
        self.assertEqual(original["total"], 16)  # All four parts, retaining repeated records.
        self.assertEqual(python["total"], original["total"])
        self.assertEqual(python["prediction_status"], "partial_prediction")
        self.assertFalse(python["native_tracking_comparability"])
        details = report["path_details"][0]
        self.assertEqual(details["run_statuses"], ["step_limit"])
        self.assertIn("LIM-002", details["limitations"])
        self.assertTrue((self.root/"table/parts/original/P3_04.json").is_file())
        self.assertTrue((self.root/"table/path-3/reference-record-map.json").is_file())
        self.assertTrue((self.root/"table/table1.md").is_file())
        self.assertTrue((self.root/"table/table1.csv").is_file())

    def test_combined_metrics_match_actual_cpp_evaluation_of_concatenated_records(self):
        report = self.report()
        metadata = json.loads((self.root/"table/path-3/reference-record-map.json").read_text())
        chunks = list(iter_path_reference(metadata["parts"], 3))
        reference = ReferencePoints(np.concatenate([c.xyz for c in chunks]),
                                    np.concatenate([c.raw_classification for c in chunks]))
        native = CppEvaluationReference()
        self.addCleanup(native.close)
        for row in report["rows"]:
            files = self.root/f"table/parts/path-3-vectors"
            rails = [read_rail_segments(files/f"{row['model']}-{side}.txt") for side in ("left", "right")]
            expected = native.evaluate(reference, *rails)
            self.assertEqual([row[k] for k in ("nTP", "nFP", "nTN", "nFN")], expected["counts"])
            actual = [row[k] for k in ("mean_horizontal_m", "std_horizontal_m", "mean_absolute_vertical_m",
                                       "std_vertical_source_m", "source_precision_percent", "source_recall_percent")]
            assert_allclose(actual, expected["metrics"], rtol=1e-12, atol=1e-9, equal_nan=True)

    def test_record_identity_and_raw_labels_are_retained_across_parts(self):
        import laspy
        document = json.loads(self.fixture.reference.read_text())
        case = next(c for c in document["cases"] if c["case_id"] == "P3_01")
        file = self.fixture.refs/case["las"]
        data = laspy.read(file)
        data.raw_classification[-1] = 35  # Classification 3 plus a legacy flag is background in source.
        data.write(file)
        case["sha256"]["las"] = file_fingerprint(file)["sha256"]
        self.fixture.reference.write_text(json.dumps(document))
        self.report()
        parts = json.loads((self.root/"table/path-3/reference-record-map.json").read_text())["parts"]
        chunks = list(iter_path_reference(parts, 3))
        assert_array_equal(np.concatenate([c.record_ids for c in chunks]), np.arange(16))
        self.assertEqual([p["record_begin"] for p in parts], [0, 4, 8, 12])
        original = [c for p in parts for c in iter_reference_las(p["las"]["path"], 3)]
        assert_array_equal(np.concatenate([c.raw_classification for c in chunks]),
                           np.concatenate([c.raw_classification for c in original]))
        assert_array_equal(chunks[0].xyz, original[0].xyz)
        self.assertIn(35, np.concatenate([c.raw_classification for c in chunks]))

    def test_path_standard_deviation_is_not_average_of_part_standard_deviations(self):
        report = self.report()
        part_report = json.loads((self.root/"table/parts/report.json").read_text())
        for row in [r for r in report["rows"] if r["model"] == "original"]:
            native_parts = [c for c in part_report["cases"] if c["model"] == row["model"]]
            naive = np.mean([c["std_horizontal"] for c in native_parts])
            self.assertNotAlmostEqual(row["std_horizontal_m"], naive, places=8)

    def test_multi_path_summary_is_residual_pooled_and_length_is_not_multiplied_by_parts(self):
        report = self.report(paths=(1, 3))
        self.assertEqual(len(report["rows"]), 4)
        self.assertFalse(report["complete_experiment_scope"])
        originals = [r for r in report["rows"] if r["model"] == "original"]
        pool = report["selected_path_summary"]["original"]
        self.assertEqual(pool["scope_path_ids"], [1, 3])
        self.assertAlmostEqual(pool["length_xy_m"], 8, places=9)
        self.assertEqual(pool["positive"], sum(row["positive"] for row in originals))
        self.assertNotAlmostEqual(pool["std_horizontal"], np.mean([row["std_horizontal_m"] for row in originals]), places=8)
        self.assertIn("Report-level pooled summary for selected paths 1, 3", (self.root/"table/table1.md").read_text())
        with (self.root/"table/selected-path-summary.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(rows[0]["scope_path_ids"], "1;3")

    def test_metres_centimetres_full_precision_csv_and_markdown_rounding(self):
        report = self.report()
        for row in report["rows"]:
            self.assertEqual(row["mean_horizontal_cm"], row["mean_horizontal_m"]*100)
            self.assertEqual(row["std_vertical_source_cm"], row["std_vertical_source_m"]*100)
        with (self.root/"table/table1.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(float(rows[0]["std_horizontal_m"]), report["rows"][0]["std_horizontal_m"])
        text = (self.root/"table/table1.md").read_text()
        self.assertIn("Absolute vertical mean", text)
        self.assertIn("Precision (source, %)", text)
        self.assertIn("not a native Total row", text)
        self.assertIn(f"{report['rows'][0]['length_xy_m']:.3f}", text)

    def test_single_path_summary_and_path_row_agree_exactly(self):
        report = self.report(chunk_size=1)
        for row in report["rows"]:
            summary = report["selected_path_summary"][row["model"]]
            for key in ("mean_horizontal", "std_horizontal", "mean_absolute_vertical", "std_vertical_source"):
                self.assertEqual(row[key+"_m"], summary[key])

    def test_chunk_size_does_not_change_reduced_results(self):
        a, b = self.report("a", chunk_size=1), self.report("b", chunk_size=5)
        self.assertEqual(a["rows"], b["rows"])

    def test_selected_path_does_not_require_other_paths_reference_files(self):
        self.freeze()
        for file in self.fixture.refs.glob('*'):
            if not file.name.startswith(("P3_", "p3-")):
                file.unlink()
        report = self.report()
        self.assertEqual(report["status"], "completed")

    def test_pending_or_duplicate_path_selection_fails_before_creating_output(self):
        self.freeze()
        for ids in ([1], [3, 3], [99], [True]):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                run_table1_report(self.fixture.manifest, [self.root/"batch"], self.mapping, self.root/"invalid", path_ids=ids)
            self.assertFalse((self.root/"invalid").exists())

    def test_frozen_inputs_remain_unchanged_and_tracking_is_never_executed(self):
        self.freeze()
        files = [self.mapping, self.fixture.manifest, *self.fixture.refs.glob('*'),
                 *self.root.glob('batch/predictions/**/*.json'), *self.root.glob('batch/predictions/**/vectors.csv')]
        before = {str(p): file_fingerprint(p)["sha256"] for p in files}
        with patch("railway_mls.prediction.runner.run_predictions", side_effect=AssertionError("no tracker")), \
                patch("railway_mls.prediction.runner.open_scene_cache", side_effect=AssertionError("no cache")):
            report = self.report()
        self.assertFalse(report["tracking_executed"])
        self.assertEqual(before, {str(p): file_fingerprint(p)["sha256"] for p in files})

    def test_existing_output_and_input_directories_are_protected(self):
        self.report()
        with self.assertRaises(FileExistsError):
            self.report()
        for output in (self.fixture.raw/"new", self.fixture.refs/"new", self.root/"batch/new"):
            with self.subTest(output=output), self.assertRaises(ValueError):
                run_table1_report(self.fixture.manifest, [self.root/"batch"], self.mapping, output, path_ids=[3])

    def test_hash_drift_and_part_count_drift_are_rejected(self):
        self.report()
        parts = json.loads((self.root/"table/path-3/reference-record-map.json").read_text())["parts"]
        bad = deepcopy(parts)
        bad[0]["record_end"] += 1
        with self.assertRaisesRegex(ValueError, "record count changed"):
            list(iter_path_reference(bad, 3))
        file = Path(parts[0]["las"]["path"])
        file.write_bytes(file.read_bytes()+b' ')
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            list(iter_path_reference(parts, 3))

    def test_combined_read_failure_is_durable_and_never_a_complete_report(self):
        with patch("railway_mls.evaluation.table1.iter_path_reference", side_effect=ValueError("generated combined read failure")):
            report = self.report()
        self.assertEqual(report["status"], "failed")
        self.assertFalse(report["complete_requested_scope"])
        self.assertEqual(report["selected_path_summary"], {})
        self.assertIn("generated combined read failure", report["failures"][0]["error"])
        stored = json.loads((self.root/"table/report.json").read_text())
        self.assertEqual(stored["status"], "failed")

    def test_initial_part_failure_is_recorded_without_creating_fake_rows(self):
        with patch("railway_mls.evaluation.table1.run_matched_evaluation", side_effect=ValueError("generated part failure")):
            report = self.report()
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["rows"], [])
        self.assertEqual(report["failures"][0]["stage"], "part_evaluation")

    def test_historical_reference_is_optional_separate_and_does_not_supply_measured_values(self):
        source = self.root/"paper.pdf"
        source.write_bytes(b"generated historical source fixture")
        document = {"schema_version": 1, "kind": "historical_table1",
                    "source": {"path": source.name, "sha256": file_fingerprint(source)["sha256"], "description": "Generated historical fixture"},
                    "rows": [{"path_id": 3, "length_m": 338., "mean_horizontal_cm": 2.8, "std_horizontal_cm": 2.4,
                              "signed_mean_vertical_cm": -.3, "std_vertical_cm": .5, "reported_recall_percent": 99.23}]}
        historical = self.root/"historical.json"
        historical.write_text(json.dumps(document))
        a = self.report("a")
        b = self.report("b", historical=historical)
        self.assertEqual(a["rows"], b["rows"])
        self.assertIn("Historical paper comparison (different protocol)", (self.root/"b/table1.md").read_text())
        self.assertTrue((self.root/"b/historical-snapshot.json").is_file())
        source.write_bytes(source.read_bytes()+b' ')
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            self.report("invalid", historical=historical)

    def test_no_assignment_retains_nonfinite_source_results_and_complete_scoring_scope(self):
        import laspy
        reference = json.loads(self.fixture.reference.read_text())
        for case in reference["cases"]:
            if case["path_id"] != 3:
                continue
            file = self.fixture.refs/case["las"]
            data = laspy.read(file)
            data.x = np.asarray(data.x)+500
            data.write(file)
            case["sha256"]["las"] = file_fingerprint(file)["sha256"]
        self.fixture.reference.write_text(json.dumps(reference))
        report = self.report()
        self.assertEqual(report["status"], "completed")
        self.assertTrue(np.isnan(report["rows"][0]["source_precision_percent"]))
        self.assertEqual(report["rows"][0]["all_positive_success_percent"], 0)
        saved = json.loads((self.root/"table/report.json").read_text())
        self.assertTrue(saved["nonfinite_values"])
        self.assertIsNone(saved["rows"][0]["mean_horizontal_m"])
        self.assertIn("N/A", (self.root/"table/table1.md").read_text())

    def test_cli_produces_selected_path_report(self):
        self.freeze()
        result = subprocess.run([sys.executable, "-m", "railway_mls", "table1-report", "--experiment", str(self.fixture.manifest),
                                 "--predictions", str(self.root/"batch"), "--mapping", str(self.mapping), "--path", "3",
                                 "--output", str(self.root/"cli")], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["requested_paths"], [3])

    @unittest.skipUnless(importlib.util.find_spec("matplotlib"), "optional static overlay")
    def test_report_overlay_uses_existing_full_scope_inspection_plot(self):
        report = self.report(overlay=True)
        self.assertEqual(len(report["overlays"]), 1)
        self.assertTrue(Path(report["overlays"][0]["path"]).is_file())
        self.assertIn("parts/path-3-overlay.png", (self.root/"table/table1.md").read_text())
