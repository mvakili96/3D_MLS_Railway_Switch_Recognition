from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_array_equal

from railway_mls.evaluation import ReferencePoints, evaluate_rails
from railway_mls.evaluation.io import file_fingerprint, read_rail_segments
from railway_mls.evaluation.matched import (concatenate_fragments, coverage_diagnostics,
                                             load_matched_plan, run_matched_evaluation)
from railway_mls.evaluation.runner import _write_json
from tests.test_prediction_catalog import experiment_document


@unittest.skipUnless(importlib.util.find_spec("laspy"), "optional LAS support")
class MatchedEvaluationTests(unittest.TestCase):
    def setUp(self):
        import laspy
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.inputs = self.root/"reference"
        self.inputs.mkdir()
        xyz = [[.2, 0, 0], [.4, .2, .06], [.6, .05, 0], [.8, .2, 0],
               [2, 0, 0], [.5, 0, 0], [.7, 1.5, 0]]
        for name, points, labels in (("part1", xyz, [3, 3, 1, 1, 3, 35, 3]),
                                      ("part2", [[2.2, 0, 0], [2.4, 0, 0]], [3, 1])):
            data = laspy.LasData(laspy.LasHeader(version="1.2", point_format=1))
            data.header.scales = [.001]*3
            data.x, data.y, data.z = np.array(points).T
            data.raw_classification = np.array(labels, dtype=np.uint8)
            data.write(self.inputs/f"{name}.las")
        for side, y in (("left", 0), ("right", 1.5)):
            (self.inputs/f"{side}.txt").write_text(f"1\n0,{y},0,3,{y},0\n")
        self.reference = self.root/"reference.json"
        cases = []
        for pid, names in ((1, ["part1", "part2"]), (2, ["part1"])):
            for i, name in enumerate(names):
                files = {"las": f"{name}.las", "left": "left.txt", "right": "right.txt"}
                cases.append({"case_id": f"P{pid}_{i+1:02}", "path_id": pid, **files,
                              "sha256": {key: self.digest(self.inputs/value) for key, value in files.items()}})
        self.reference.write_text(json.dumps({"schema_version": 1, "profile": "source2018",
                                              "dataset_root": "reference", "cases": cases}))
        self.definition = experiment_document()
        self.definition["evaluation"] = None  # Deliberately not read by inference.
        self.definition_path = self.root/"prediction.json"
        self.definition_path.write_text(json.dumps(self.definition))
        self.export = self.root/"frozen"
        self.run_dir = self.export/"pilot"
        self.run_dir.mkdir(parents=True)
        with (self.run_dir/"vectors.csv").open("w") as stream:
            stream.write("layer,vector,vertex,x,y,z,point_type\n")
            for layer, xs in ((0, [0, 1]), (1, [2, 3])):
                for name, y in (("rail1", 0), ("rail2", 1.5), ("center", .75)):
                    for vertex, x in enumerate(xs):
                        stream.write(f"{layer},{name},{vertex},{x},{y},0,0\n")
        run = self.definition["inference"]["runs"][0]
        self.metadata = {"schema_version": 1, "run_id": "pilot", "status": "completed_with_limitations",
                         "seed": run["seed"], "direction": run["direction"], "branch_policy": "strict",
                         "source_baseline": self.definition["inference"]["source_baseline"],
                         "config": self.definition["inference"]["config"],
                         "reference_files_read": False, "evaluation_performed": False,
                         "random": {"seed": 1}, "layers": [{"layer": 0}, {"layer": 1}],
                         "limitations_encountered": ["LIM-001", "LIM-002"], "reference_comparable": False,
                         "artifacts": {"vectors.csv": {key: value for key, value in
                             file_fingerprint(self.run_dir/"vectors.csv").items() if key != "path"}}}
        self.document = {"schema_version": 1, "kind": "matched_prediction_evaluation", "profile": "source2018",
                         "reference_manifest": {"path": "reference.json", "sha256": self.digest(self.reference)},
                         "prediction_sources": [{"source_id": "frozen", "directory": "frozen",
                                                  "manifest_sha256": "", "inference_manifest": "prediction.json"}],
                         "paths": [{"path_id": 1, "status": "reviewed", "case_ids": ["P1_01", "P1_02"],
                                    "review_note": "Explicit generated route; partial prediction, full reference scope",
                                    "fragments": [{"source_id": "frozen", "run_id": "pilot", "layer": 0,
                                                   "run_manifest_sha256": ""}]},
                                   {"path_id": 2, "status": "pending", "case_ids": ["P2_01"],
                                    "fragments": [], "review_note": "No route assignment yet"}]}
        self.manifest = self.root/"matched.json"
        self.freeze_metadata()

    def digest(self, path):
        return file_fingerprint(path)["sha256"]

    def save(self):
        self.manifest.write_text(json.dumps(self.document))

    def freeze_metadata(self):
        _write_json(self.run_dir/"manifest.json", self.metadata)
        _write_json(self.export/"manifest.json", {
            "schema_version": 1, "kind": "headless_prediction_run", "status": "completed_with_limitations",
            "manifest_sha256": self.digest(self.definition_path), "reference_files_read": False,
            "evaluation_performed": False, "runs": [self.metadata]})
        self.document["prediction_sources"][0]["manifest_sha256"] = self.digest(self.export/"manifest.json")
        for mapping in self.document["paths"]:
            for fragment in mapping["fragments"]:
                fragment["run_manifest_sha256"] = self.digest(self.run_dir/"manifest.json")
        self.save()

    def run_evaluation(self, name="out", **kwargs):
        return run_matched_evaluation(self.manifest, self.root/name, **kwargs)

    def test_full_reference_scope_and_source_metrics_expose_short_prediction(self):
        report = self.run_evaluation(point_outcomes=True, chunk_size=2)
        self.assertEqual(report["status"], "completed")
        python, original = report["paths"]
        self.assertEqual(python["model"], "python")
        self.assertEqual(python["positive"], 5)
        self.assertEqual(python["assigned_positive"], 3)
        self.assertEqual(python["unassigned_positive"], 2)
        self.assertEqual(python["positive_assignment_percent"], 60)
        self.assertEqual(python["all_positive_success_percent"], 40)
        self.assertAlmostEqual(python["source_precision_percent"], 100*2/3)
        self.assertEqual(original["positive"], python["positive"])
        self.assertEqual(original["all_positive_success_percent"], 80)
        self.assertTrue(python["complete_manifest_path"])
        self.assertEqual(report["unmapped_path_ids"], [2])
        self.assertEqual(len(report["pending_mappings"]), 1)
        self.assertFalse(report["nine_path_complete"])
        self.assertEqual(report["models"]["python"]["settings"], report["models"]["original"]["settings"])
        self.assertEqual(report["models"]["python"]["source_fingerprint"], report["models"]["original"]["source_fingerprint"])
        first = next(c for c in report["cases"] if c["model"] == "python" and c["case_id"] == "P1_01")
        self.assertEqual([first[key] for key in ("nTP", "nFP", "nTN", "nFN")], [2, 1, 1, 2])
        self.assertTrue((self.root/"out/python/P1_02-points.csv").is_file())
        stored = json.loads((self.root/"out/report.json").read_text())
        self.assertTrue(stored["nonfinite_values"])  # Unassigned-only part retains native NaN stats.

    def test_all_consumed_inputs_remain_unchanged_and_frozen_provenance_is_retained(self):
        files = [self.manifest, self.reference, self.definition_path, *self.inputs.iterdir(),
                 self.export/"manifest.json", *self.run_dir.iterdir()]
        before = {p: self.digest(p) for p in files}
        report = self.run_evaluation()
        self.assertEqual(before, {p: self.digest(p) for p in files})
        fragment = report["mappings"][0]["fragments"][0]
        self.assertEqual(fragment["run_summary"]["limitations_encountered"], ["LIM-001", "LIM-002"])
        self.assertFalse(fragment["run_summary"]["reference_comparable"])
        self.assertEqual(fragment["copied_vectors"]["sha256"], fragment["vectors"]["sha256"])

    def test_separate_fragments_keep_order_without_connecting_across_gap(self):
        extra = deepcopy(self.document["paths"][0]["fragments"][0])
        extra["layer"] = 1
        self.document["paths"][0]["fragments"].append(extra)
        self.save()
        plan = load_matched_plan(self.manifest)
        rails = plan["mappings"][0]["rails"]
        self.assertEqual([len(r) for r in rails], [2, 2])
        assert_array_equal(rails[0].endpoints[:, :, 0], [[0, 1], [2, 3]])
        result = evaluate_rails(ReferencePoints([[1.5, 0, 0]], np.array([3])), *rails)
        self.assertEqual(result.coverage["unassigned_positive"], 1)
        report = self.run_evaluation()
        self.assertEqual(report["mappings"][0]["fragments"][1]["segment_ranges_per_side"]["left"], [1, 2])
        exported = read_rail_segments(self.root/"out/path-1-vectors/python-left.txt")
        assert_array_equal(exported.endpoints, rails[0].endpoints)

    def test_duplicate_fragment_is_rejected_even_across_aliases_or_paths(self):
        for mode in ("same_mapping", "another_path", "source_alias"):
            with self.subTest(mode=mode):
                original = deepcopy(self.document)
                fragment = deepcopy(self.document["paths"][0]["fragments"][0])
                if mode == "same_mapping":
                    self.document["paths"][0]["fragments"].append(fragment)
                else:
                    if mode == "source_alias":
                        alias = deepcopy(self.document["prediction_sources"][0])
                        alias["source_id"] = "alias"
                        self.document["prediction_sources"].append(alias)
                        fragment["source_id"] = "alias"
                    self.document["paths"][1].update(status="reviewed", fragments=[fragment])
                self.save()
                with self.assertRaisesRegex(ValueError, "more than once|identical segment"):
                    load_matched_plan(self.manifest)
                self.document = original

    def test_identical_segment_across_distinct_layers_requires_review(self):
        plan = load_matched_plan(self.manifest)
        f = plan["mappings"][0]["fragments"][0]
        with self.assertRaisesRegex(ValueError, "identical segment"):
            concatenate_fragments([f, f])

    def test_fragment_adapter_matches_native_evaluator_without_a_bridge(self):
        from tests.cpp_evaluation_reference import CppEvaluationReference
        extra = deepcopy(self.document["paths"][0]["fragments"][0])
        extra["layer"] = 1
        self.document["paths"][0]["fragments"].append(extra)
        self.save()
        rails = load_matched_plan(self.manifest)["mappings"][0]["rails"]
        reference = ReferencePoints([[.2, 0, -.02], [1.5, 0, 0], [2.5, 0, 0],
                                     [2.6, 1.52, .04], [.7, .2, 0]], np.array([3, 3, 3, 3, 1]))
        native = CppEvaluationReference()
        self.addCleanup(native.close)
        expected = native.evaluate(reference, *rails)
        actual = evaluate_rails(reference, *rails, collect_points=True)
        self.assertEqual([actual.counts[k] for k in ("nTP", "nFP", "nTN", "nFN")], expected["counts"])
        assert_array_equal(actual.points["segment_id"], np.asarray(expected["points"])[:, 0])
        self.assertEqual(actual.points["segment_id"][1], -1)

    def test_pending_mapping_cannot_be_silently_scored(self):
        self.document["paths"][0].update(status="pending", fragments=[])
        self.save()
        with self.assertRaisesRegex(ValueError, "no reviewed"):
            self.run_evaluation()
        self.assertFalse((self.root/"out").exists())

    def test_cropped_missing_reordered_and_duplicate_cases_are_rejected(self):
        for cases in (["P1_01"], ["P1_02", "P1_01"], ["P1_01", "P1_01"], ["P2_01"]):
            with self.subTest(cases=cases):
                self.document["paths"][0]["case_ids"] = cases
                self.save()
                with self.assertRaisesRegex(ValueError, "complete reference path"):
                    load_matched_plan(self.manifest)

    def test_unknown_layer_and_invalid_ids_are_rejected(self):
        for field, value in (("layer", 55), ("layer", True), ("layer", -1),
                             ("run_id", "../pilot"), ("source_id", "unknown")):
            with self.subTest(field=field, value=value):
                original = deepcopy(self.document)
                self.document["paths"][0]["fragments"][0][field] = value
                self.save()
                with self.assertRaises(ValueError):
                    load_matched_plan(self.manifest)
                self.document = original

    def test_missing_review_note_is_rejected(self):
        self.document["paths"][0]["review_note"] = " "
        self.save()
        with self.assertRaises(ValueError):
            load_matched_plan(self.manifest)

    def test_hash_drift_of_each_consumed_input_is_rejected_before_output(self):
        files = [self.reference, self.definition_path, self.export/"manifest.json",
                 self.run_dir/"manifest.json", self.run_dir/"vectors.csv",
                 self.inputs/"part1.las", self.inputs/"left.txt"]
        for file in files:
            with self.subTest(file=file.name):
                original = file.read_bytes()
                file.write_bytes(original+b" ")
                with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
                    self.run_evaluation()
                self.assertFalse((self.root/"out").exists())
                file.write_bytes(original)

    def test_repaired_profile_and_changed_initialization_are_rejected(self):
        for key, value in (("branch_policy", "fitted_references"), ("seed", {"xyz": [0, 0, 0], "angle": 0, "mode": 4})):
            with self.subTest(key=key):
                original = deepcopy(self.metadata)
                self.metadata[key] = value
                self.freeze_metadata()
                with self.assertRaises(ValueError):
                    load_matched_plan(self.manifest)
                self.metadata = original

    def test_recorded_partial_run_is_evaluated_with_its_failure_status_visible(self):
        self.metadata["status"] = "resource_limited"
        self.metadata["failure"] = {"error": "generated resource limit"}
        self.freeze_metadata()
        report = self.run_evaluation()
        self.assertEqual(report["status"], "completed")  # Evaluation completed; extraction was partial.
        self.assertEqual(report["mappings"][0]["fragments"][0]["run_summary"]["status"], "resource_limited")
        self.assertEqual(report["paths"][0]["positive"], 5)

    def test_failing_part_is_reported_as_incomplete_pair(self):
        from railway_mls.evaluation.io import iter_reference_las
        def read(path, size):
            if Path(path).name == "part2.las":
                raise ValueError("generated read failure")
            return iter_reference_las(path, size)
        with patch("railway_mls.evaluation.runner.iter_reference_las", side_effect=read):
            report = self.run_evaluation()
        self.assertEqual(report["status"], "failed")
        self.assertTrue(report["failures"])
        self.assertFalse(report["paths"][0]["complete_manifest_path"])

    def test_chunk_size_does_not_change_membership_counts_or_metrics(self):
        a, b = self.run_evaluation("a", chunk_size=1), self.run_evaluation("b", chunk_size=7)
        for ca, cb in zip(a["cases"], b["cases"]):
            self.assertEqual(ca.keys(), cb.keys())
            for key in ca:
                if isinstance(ca[key], float) and np.isnan(ca[key]):
                    self.assertTrue(np.isnan(cb[key]))
                else:
                    self.assertEqual(ca[key], cb[key])

    def test_outputs_are_new_and_separate_from_frozen_inputs(self):
        self.run_evaluation()
        with self.assertRaises(FileExistsError):
            self.run_evaluation()
        for output in (self.export/"child", self.inputs/"child"):
            with self.assertRaisesRegex(ValueError, "separate from frozen"):
                run_matched_evaluation(self.manifest, output)
            self.assertFalse(output.exists())

    def test_zero_positive_coverage_is_unavailable_without_renaming_native_metrics(self):
        self.assertEqual(coverage_diagnostics({"nTP": 0}, {"positive": 0, "assigned_positive": 0}),
                         {"positive_assignment_percent": None, "all_positive_success_percent": None})

    def test_duplicate_manifest_json_keys_are_rejected(self):
        self.manifest.write_text('{"kind": "a", "kind": "b"}')
        with self.assertRaisesRegex(ValueError, "duplicate JSON field"):
            load_matched_plan(self.manifest)

    def test_cli_runs_without_gui_or_executing_tracking(self):
        script = """
import sys
from unittest.mock import patch
from railway_mls.cli import main
sys.argv = ['railway_mls', 'evaluate-predictions', '--manifest', sys.argv[1], '--output', sys.argv[2]]
with patch('railway_mls.prediction.runner.run_predictions', side_effect=AssertionError('tracking must not run')):
    code = main()
assert 'open3d' not in sys.modules
raise SystemExit(code)
"""
        result = subprocess.run([sys.executable, "-c", script, str(self.manifest), str(self.root/"cli")],
                                text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["status"], "completed")

    @unittest.skipUnless(importlib.util.find_spec("matplotlib"), "optional evaluation plots")
    def test_overlay_is_static_sampled_and_does_not_change_scores(self):
        from railway_mls.evaluation.overlay import plot_matched_evaluation
        report = self.run_evaluation(overlay=True)
        self.assertEqual(len(report["overlays"]), 1)
        image = Path(report["overlays"][0]["path"])
        self.assertEqual(image.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
        self.assertEqual(report["overlays"][0]["evaluation_sampling"], "none")
        with self.assertRaises(FileExistsError):
            plot_matched_evaluation(self.root/"out/report.json")
        self.assertEqual(report["paths"][0]["positive"], 5)
