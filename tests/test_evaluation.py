import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls.artifacts import strict_json_value
from railway_mls.evaluation import (EvaluationConfig, RailSegments, ReferencePoints,
                                    evaluate_chunks, evaluate_rails)
from railway_mls.evaluation.audit import evaluation_audit, source_fingerprint
from railway_mls.evaluation.geometry import prepare_line
from railway_mls.evaluation.io import (iter_reference_las, read_python_vectors,
                                       read_rail_segments, rails_from_layer)
from railway_mls.evaluation.history import saved_outcome_counts, workbook_paths, reconcile_history
from railway_mls.evaluation.runner import evaluation_demo, load_cases, run_evaluation
from railway_mls.models import TrackLayer
from railway_mls.points import Point3D
from tests.cpp_evaluation_reference import CppEvaluationReference, SOURCE


ROOT = Path(__file__).resolve().parents[1]
EMPTY = RailSegments([])
STRAIGHT = RailSegments([[[0, 0, 0], [1, 0, 0]]])


def compare_native(test, native, reference, left, right, **kwargs):
    expected = native.evaluate(reference, left, right)
    actual = evaluate_rails(reference, left, right, collect_points=True, **kwargs)
    test.assertEqual([actual.counts[k] for k in ("nTP", "nFP", "nTN", "nFN")], expected["counts"])
    metrics = [actual.metrics[k] for k in ("mean_horizontal", "std_horizontal", "mean_absolute_vertical",
                                          "std_vertical_source", "source_precision_percent", "source_recall_percent")]
    assert_allclose(metrics, expected["metrics"], rtol=1e-12, atol=1e-9, equal_nan=True)
    if len(reference):
        points = np.asarray(expected["points"])
        assert_array_equal(actual.points["segment_id"], points[:, 0])
        assert_array_equal(actual.points["outcome"], points[:, 1])
        assert_allclose(actual.points["horizontal"], points[:, 2], rtol=1e-12, atol=1e-9, equal_nan=True)
        assert_allclose(actual.points["signed_vertical"], points[:, 3], rtol=1e-12, atol=1e-9, equal_nan=True)
    for segment, polygon in zip(np.concatenate((left.endpoints, right.endpoints)), expected["polygons"]):
        assert_allclose(prepare_line(segment).polygon, polygon, rtol=1e-12, atol=1e-9)
    return actual


class EvaluationCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = CppEvaluationReference()

    @classmethod
    def tearDownClass(cls):
        cls.native.close()

    def test_basic_all_four_outcomes_and_unassigned(self):
        reference = ReferencePoints([[.5, 0, 0], [.6, .2, .06], [.7, .05, 0],
                                     [.8, .2, 0], [2, 0, 0]], np.array([3, 3, 1, 1, 3]))
        actual = compare_native(self, self.native, reference, STRAIGHT, EMPTY)
        self.assertEqual(actual.counts, dict(nTP=1, nFP=1, nTN=1, nFN=1))
        self.assertEqual(actual.coverage["unassigned_positive"], 1)

    def test_positive_inclusive_background_strict_thresholds(self):
        xyz = [[.5, .12, 0], [.5, 0, .05], [.5, .12, .05]]
        reference = ReferencePoints(xyz+xyz, np.array([3]*3+[1]*3))
        result = compare_native(self, self.native, reference, STRAIGHT, EMPTY)
        assert_array_equal(result.points["outcome"], [1, 1, 1, 2, 2, 2])

    def test_polygon_edges_and_near_edge_cross_product_precision(self):
        reference = ReferencePoints([[0, 0, 0], [1, 0, 0], [.5, .3, 0], [.5, -.3, 0],
                                     [.5, .3-2e-10, 0], [.5, .3-2e-11, 0]], np.full(6, 3))
        result = compare_native(self, self.native, reference, STRAIGHT, EMPTY)
        assert_array_equal(result.points["segment_id"][:4], [-1]*4)

    def test_file_order_wins_over_nearest_segment_and_right_rail(self):
        left = RailSegments([[[0, .2, 0], [1, .2, 0]], [[0, 0, 0], [1, 0, 0]]])
        result = compare_native(self, self.native, ReferencePoints([[.5, 0, 0]], np.array([3])), left, STRAIGHT)
        self.assertEqual(result.points["segment_id"][0], 0)
        self.assertEqual(result.counts["nFP"], 1)

    def test_reversing_segment_order_changes_membership(self):
        left = RailSegments([[[0, 0, 0], [1, 0, 0]], [[0, .2, 0], [1, .2, 0]]])
        result = compare_native(self, self.native, ReferencePoints([[.5, 0, 0]], np.array([3])), left, EMPTY)
        self.assertEqual(result.counts["nTP"], 1)

    def test_vertical_segment_and_slope(self):
        line = RailSegments([[[5, 2, 3], [5, 8, 4]]])
        compare_native(self, self.native, ReferencePoints([[5.01, 3, 3.17], [5.2, 7, 4],
                                                          [5, 4, 3.1]], np.array([3, 3, 0])), line, EMPTY)

    def test_negative_vertical_residual_source_standard_deviation(self):
        reference = ReferencePoints([[.2, 0, -.02], [.8, 0, -.04]], np.array([3, 3]))
        result = compare_native(self, self.native, reference, STRAIGHT, EMPTY)
        self.assertAlmostEqual(result.metrics["mean_absolute_vertical"], .03)
        self.assertGreater(result.metrics["std_vertical_source"], .08)

    def test_oblique_utm_rails_both_orientations(self):
        for direction in (1, -1):
            with self.subTest(direction=direction):
                ends = np.array([[513194.41, 5458237.85, 6.61], [513196.89, 5458237.42, 6.65]])[::direction]
                rails = RailSegments([ends])
                xyz = ends[0] + np.linspace(.02, .98, 91)[:, None]*(ends[1]-ends[0])
                xyz[:, 1] += np.sin(np.arange(91))*.25
                xyz[:, 2] += np.cos(np.arange(91))*.09
                compare_native(self, self.native, ReferencePoints(xyz, np.where(np.arange(91)%3, 3, 1)), rails, EMPTY)

    def test_empty_reference(self):
        compare_native(self, self.native, ReferencePoints(np.empty((0, 3)), np.empty(0, dtype=int)), STRAIGHT, EMPTY)

    def test_no_segments(self):
        compare_native(self, self.native, ReferencePoints([[1, 2, 3]], np.array([3])), EMPTY, EMPTY)

    def test_single_positive_zero_denominator_statistics(self):
        compare_native(self, self.native, ReferencePoints([[.5, 0, -.04]], np.array([3])), STRAIGHT, EMPTY)

    def test_no_positive_assigned(self):
        compare_native(self, self.native, ReferencePoints([[.5, 0, 0]], np.array([1])), STRAIGHT, EMPTY)

    def test_degenerate_xy_polygon_accepts_collinear_points_outside_bounds(self):
        line = RailSegments([[[5, 2, 0], [5, 2, 0]]])
        reference = ReferencePoints([[5, -100, 0], [5, 100, .03], [4, 2, 0]], np.array([3, 3, 1]))
        compare_native(self, self.native, reference, line, EMPTY)

    def test_degenerate_xy_with_nonzero_height_retains_nan(self):
        line = RailSegments([[[5, 2, 0], [5, 2, 1]]])
        compare_native(self, self.native, ReferencePoints([[5, 2, 0], [5, 100, 0]], np.array([3, 3])), line, EMPTY)

    def test_tiny_polygon_index_fallback(self):
        rails = RailSegments([[[0, 0, 0], [1e-12, 0, 0]]])
        compare_native(self, self.native, ReferencePoints([[0, 1, 0], [0, 100, 0], [1, 1, 0]], np.array([3, 3, 3])), rails, EMPTY)

    def test_seeded_mixed_segments_index_matches_native_and_full_scan(self):
        rng = np.random.default_rng(438)
        left = RailSegments([[[0, 0, 0], [1, .3, .04]], [[1, .3, .04], [2, -.1, .08]],
                             [[2, -.1, .08], [3, .4, .02]]])
        right = RailSegments(left.endpoints + [0, .5, .02])
        xyz = rng.uniform([-.2, -.5, -.1], [3.2, 1, .2], (1300, 3))
        reference = ReferencePoints(xyz, rng.choice([0, 1, 3, 35], 1300))
        result = compare_native(self, self.native, reference, left, right)
        scan = evaluate_rails(reference, left, right, use_index=False, collect_points=True)
        for name in result.points.dtype.names:
            assert_array_equal(result.points[name], scan.points[name])


class EvaluationContractsTests(unittest.TestCase):
    def test_reference_labels_and_coordinates_are_immutable_owned_copies(self):
        xyz = np.array([[.5, 0, 0.]])
        labels = np.array([3])
        reference = ReferencePoints(xyz, labels)
        xyz[:] = 100
        labels[:] = 1
        self.assertEqual(reference.xyz[0, 0], .5)
        self.assertEqual(reference.raw_classification[0], 3)
        with self.assertRaises(ValueError):
            reference.raw_classification[0] = 1

    def test_raw_flags_are_not_masked_to_class_three(self):
        ref = ReferencePoints([[.5, 0, 0], [.5, 0, 0]], np.array([3, 35]))
        result = evaluate_rails(ref, STRAIGHT, EMPTY)
        self.assertEqual(result.counts["nTP"], 1)
        self.assertEqual(result.counts["nFN"], 1)

    def test_invalid_input_shapes_labels_ids_and_nonfinite_rejected(self):
        for xyz, labels, ids in (([[0, 0]], [3], None), ([[np.nan, 0, 0]], [3], None),
                                  ([[0, 0, 0]], [256], None), ([[0, 0, 0]], [3.], None),
                                  ([[0, 0, 0]], [3], [-1]), ([[0, 0, 0]]*2, [3, 3], [1, 1])):
            with self.subTest(xyz=xyz, labels=labels), self.assertRaises(ValueError):
                ReferencePoints(xyz, np.array(labels), ids)
        with self.assertRaises(ValueError):
            RailSegments([[[0, 0, 0], [np.inf, 0, 0]]])

    def test_source_profile_cannot_silently_change_thresholds(self):
        with self.assertRaises(ValueError):
            EvaluationConfig(horizontal_threshold=.1)

    def test_chunk_and_index_independence_source_ordered_reductions(self):
        rng = np.random.default_rng(11)
        xyz = rng.uniform([.01, -.29, -.1], [.99, .29, .1], (500, 3))
        # Alternate segments in input order to exercise segment-first reductions.
        xyz[::2, 0] += 1
        left = RailSegments([[[0, 0, 0], [1, 0, 0]], [[1, 0, 0], [2, 0, 0]]])
        ref = ReferencePoints(xyz, np.where(np.arange(500)%3, 3, 1))
        baseline = evaluate_rails(ref, left, EMPTY, collect_points=True)
        for size in (1, 7, 100, 499):
            with self.subTest(size=size):
                chunks = (ReferencePoints(ref.xyz[i:i+size], ref.raw_classification[i:i+size], ref.record_ids[i:i+size])
                          for i in range(0, len(ref), size))
                result = evaluate_chunks(chunks, left, EMPTY, collect_points=True)
                self.assertEqual(baseline.summary(), result.summary())
                for name in result.points.dtype.names:
                    assert_array_equal(baseline.points[name], result.points[name])

    def test_repeat_evaluation_does_not_reuse_membership(self):
        reference = ReferencePoints([[.5, 0, 0], [.8, .2, .1]], np.array([3, 3]))
        self.assertEqual(evaluate_rails(reference, STRAIGHT, EMPTY).summary(),
                         evaluate_rails(reference, STRAIGHT, EMPTY).summary())

    def test_decreasing_chunk_record_ids_rejected(self):
        ref = ReferencePoints([[.5, 0, 0]], np.array([3]))
        with self.assertRaises(ValueError):
            evaluate_chunks([ref, ref], STRAIGHT, EMPTY)

    def test_nonfinite_report_strict_json_retains_paths(self):
        result = evaluate_rails(ReferencePoints(np.empty((0, 3)), np.empty(0, dtype=int)), EMPTY, EMPTY)
        payload = strict_json_value(result.summary())
        json.dumps(payload, allow_nan=False)
        self.assertTrue(payload["nonfinite_values"])
        self.assertIsNone(payload["metrics"]["mean_horizontal"])

    def test_layer_adapter_never_joins_separate_layers(self):
        layer = TrackLayer()
        layer.rail1_vector.extend([Point3D(0, 0, 0), Point3D(1, 0, 0)])
        layer.rail2_vector.extend([Point3D(0, 1, 0), Point3D(1, 1, 0)])
        left, right = rails_from_layer(layer)
        self.assertEqual(len(left), 1)
        self.assertEqual(len(right), 1)

    def test_python_csv_adapter_requires_explicit_contiguous_layer(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"vectors.csv"
            path.write_text("layer,vector,vertex,x,y,z,point_type\n"
                            "0,rail1,0,0,0,0,0\n0,rail1,1,1,0,0,0\n"
                            "1,rail1,0,100,0,0,0\n1,rail1,1,101,0,0,0\n")
            left, right = read_python_vectors(path, layer=0)
            assert_array_equal(left.endpoints, STRAIGHT.endpoints)
            self.assertEqual(len(right), 0)
            with self.assertRaises(ValueError):
                read_python_vectors(path, layer=2)
            path.write_text(path.read_text().replace("0,rail1,1", "0,rail1,2"))
            with self.assertRaises(ValueError):
                read_python_vectors(path, layer=0)

    def test_segment_reader_preserves_order_and_rejects_bad_files(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"rails.txt"
            path.write_text("2\n0,0,0,1,0,0\n1,0,0,2,0,0\n")
            self.assertEqual(len(read_rail_segments(path)), 2)
            for text in ("1\n0,0,0\n", "1\n0,0,0,nan,0,0\n", "-1\n", "0\nextra\n"):
                path.write_text(text)
                with self.subTest(text=text), self.assertRaises(ValueError):
                    read_rail_segments(path)

    def test_demo_all_outcomes_and_explicit_generated_data(self):
        demo = evaluation_demo()
        self.assertEqual(demo["counts"], dict(nTP=1, nFP=1, nTN=1, nFN=1))
        self.assertIn("generated", demo["data"])

    def test_canonical_manifest_24_cases_nine_paths_no_aggregates(self):
        cases, _ = load_cases(ROOT/"configs/evaluation-vancouver.json")
        self.assertEqual(len(cases), 24)
        self.assertEqual({c["path_id"] for c in cases}, set(range(1, 10)))
        self.assertTrue(all("head" not in c["case_id"].lower() for c in cases))
        selected, _ = load_cases(ROOT/"configs/evaluation-vancouver.json", selected=["P6_01", "P6_02"])
        self.assertEqual(len(selected), 2)
        with self.assertRaises(ValueError):
            load_cases(ROOT/"configs/evaluation-vancouver.json", selected=["unknown"])

    def test_audit_missing_source_report_not_pass(self):
        with tempfile.TemporaryDirectory() as temp:
            report = evaluation_audit(temp)
            self.assertNotEqual(report["status"], "ok")
            self.assertEqual(len(report["errors"]), 35)
        self.assertEqual(len(source_fingerprint()), 64)


@unittest.skipUnless(importlib.util.find_spec("laspy"), "optional LAS dependency")
class EvaluationIOTests(unittest.TestCase):
    def setUp(self):
        import laspy
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.las = self.root/"reference.las"
        header = laspy.LasHeader(version="1.2", point_format=1)
        header.scales = [.01, .01, .001]
        points = laspy.LasData(header)
        points.x, points.y, points.z = [.2, .4, .6, .8, 2], [0, .2, .05, .2, 0], [0, .06, 0, 0, 0]
        points.raw_classification = np.array([3, 3, 1, 1, 35], dtype=np.uint8)
        points.write(self.las)
        self.left = self.root/"left.txt"
        self.right = self.root/"right.txt"
        self.left.write_text("1\n0,0,0,1,0,0\n")
        self.right.write_text("0\n")

    def case(self):
        return {"case_id": "fixture", "path_id": 1, "las": self.las, "left": self.left, "right": self.right}

    def test_las_keeps_all_records_raw_flag_byte_world_coordinates_and_order(self):
        chunks = list(iter_reference_las(self.las, 2))
        assert_array_equal(np.concatenate([c.record_ids for c in chunks]), np.arange(5))
        assert_array_equal(np.concatenate([c.raw_classification for c in chunks]), [3, 3, 1, 1, 35])
        self.assertEqual(sum(len(c) for c in chunks), 5)

    def test_las_world_scales_offsets_format_zero(self):
        import laspy
        header = laspy.LasHeader(version="1.2", point_format=0)
        header.scales, header.offsets = [.01, .02, .001], [513000, 5458000, 6]
        data = laspy.LasData(header)
        data.X, data.Y, data.Z = [1, -3], [4, -7], [8, -9]
        data.classification = [3, 1]
        data.write(self.las)
        ref = next(iter_reference_las(self.las))
        assert_array_equal(ref.xyz, np.array([[1, 4, 8], [-3, -7, -9]])*header.scales+header.offsets)

    def test_truncated_las_rejected_before_scoring(self):
        self.las.write_bytes(self.las.read_bytes()[:-10])
        with self.assertRaises(ValueError):
            list(iter_reference_las(self.las))

    def test_runner_reports_preserve_inputs_and_do_not_overwrite(self):
        original = self.las.read_bytes()
        output = self.root/"result"
        result = run_evaluation([self.case()], output, chunk_size=2, point_outcomes=True)
        self.assertEqual(result["status"], "completed")
        self.assertEqual(self.las.read_bytes(), original)
        with (output/"fixture-points.csv").open() as stream:
            self.assertEqual(len(list(csv.DictReader(stream))), 5)
        self.assertEqual(json.loads((output/"report.json").read_text())["status"], "completed")
        with self.assertRaises(FileExistsError):
            run_evaluation([self.case()], output)

    def test_input_hash_mismatch_is_recorded_failed_case(self):
        case = {**self.case(), "sha256": {"las": "0"*64}}
        result = run_evaluation([case], self.root/"result")
        self.assertEqual(result["status"], "failed")
        self.assertEqual(len(result["failures"]), 1)
        self.assertEqual(result["cases"], [])

    def test_runner_rejects_duplicate_and_path_like_case_ids_before_writing(self):
        for cases in ([self.case(), self.case()], [{**self.case(), "case_id": "../reference"}]):
            with self.subTest(cases=cases), self.assertRaises(ValueError):
                run_evaluation(cases, self.root/"unsafe")
            self.assertFalse((self.root/"unsafe").exists())

    def test_python_prediction_cli_adapter(self):
        vectors = self.root/"vectors.csv"
        vectors.write_text("layer,vector,vertex,x,y,z,point_type\n"
                           "0,rail1,0,0,0,0,0\n0,rail1,1,1,0,0,0\n")
        result = run_evaluation([{**self.case(), "python_vectors": vectors, "layer": 0}], self.root/"python-result")
        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["cases"][0]["inputs"]["python_vectors"]["path"], str(vectors))

    def test_cli_single_case_and_conflicting_inputs(self):
        base = [sys.executable, "-m", "railway_mls", "evaluate-rails", "--output", str(self.root/"cli")]
        result = subprocess.run([*base, "--las", str(self.las), "--left", str(self.left), "--right", str(self.right)],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["cases_completed"], 1)
        conflict = subprocess.run([*base, "--manifest", "ignored", "--las", str(self.las)], capture_output=True, text=True)
        self.assertEqual(conflict.returncode, 2)


class EvaluationHistoryTests(unittest.TestCase):
    def test_historical_outcome_codes_are_separate_and_validated(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"outcomes.txt"
            path.write_text("1 2 3 1\n4 5 6 2\n7 8 9 1\n")
            self.assertEqual(saved_outcome_counts(path), {"success": 2, "failure": 1})
            for invalid in ("1 2 3 3\n", "nan 2 3 1\n", "bad 2 3 1\n"):
                path.write_text(invalid)
                with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                    saved_outcome_counts(path)

    @unittest.skipUnless((ROOT/"evaluation data/DataForPaper/Evaluation.xlsx").is_file(), "local historical workbook")
    def test_workbook_macro_and_pooled_metrics_are_distinct(self):
        rows = workbook_paths(ROOT/"evaluation data/DataForPaper/Evaluation.xlsx")
        self.assertEqual(rows[1]["success"], 1413950)
        self.assertAlmostEqual(sum(r["displayed_recall_percent"] for r in rows.values())/9, 99.46333333333334)
        self.assertAlmostEqual(100*sum(r["success"] for r in rows.values())/
                               sum(r["success"]+r["failure"] for r in rows.values()), 99.73468857984237)

    @unittest.skipUnless((ROOT/"evaluation data/DataForPaper/Evaluation.xlsx").is_file(), "local historical workbook")
    def test_partial_path_comparison_is_marked_incomplete(self):
        result = {"case_id": "P1_01", "path_id": 1,
                  "counts": {"nTP": 1, "nFP": 1}, "coverage": {"positive": 3}}
        with tempfile.TemporaryDirectory() as temp:
            Path(temp, "Path1_TPFN.txt").write_text("1 2 3 1\n")
            report = reconcile_history([result], ROOT/"evaluation data/DataForPaper/Evaluation.xlsx", temp,
                                       expected_case_ids={"1": ["P1_01", "P1_02"]})
            self.assertFalse(report["paths"][0]["complete_manifest_path"])
            self.assertFalse(report["paths"][0]["saved_counts_match_workbook"])

@unittest.skipUnless((SOURCE/"ViewerDoc.cpp").is_file() and importlib.util.find_spec("laspy"), "local evaluation package")
class EvaluationVancouverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.native = CppEvaluationReference()

    @classmethod
    def tearDownClass(cls):
        cls.native.close()

    def test_representative_real_records_all_24_cases_match_native(self):
        cases, _ = load_cases(ROOT/"configs/evaluation-vancouver.json")
        for case in cases:
            with self.subTest(case=case["case_id"]):
                xyz, labels, record_ids = [], [], []
                for chunk in iter_reference_las(case["las"], 250_000):
                    # Deterministic actual records, balanced by reference class;
                    # native comparisons use all model segments for each path.
                    ids = np.r_[np.flatnonzero(chunk.raw_classification == 3)[::997],
                                np.flatnonzero(chunk.raw_classification != 3)[::4999]]
                    ids.sort()
                    xyz.extend(chunk.xyz[ids])
                    labels.extend(chunk.raw_classification[ids])
                    record_ids.extend(chunk.record_ids[ids])
                reference = ReferencePoints(xyz, np.array(labels), record_ids)
                compare_native(self, self.native, reference,
                               read_rail_segments(case["left"]), read_rail_segments(case["right"]))
