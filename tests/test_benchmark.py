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

from railway_mls.config import ExtractionConfig
from railway_mls.evaluation.io import file_fingerprint
from railway_mls.prediction.benchmark import plan_benchmark, run_benchmark, evaluate_benchmark
from railway_mls.prediction.cache import build_scene_cache
from railway_mls.prediction.manifest import load_inference_spec, load_prediction_manifest
from railway_mls.prediction.seeds import prepare_seed_manifest, write_seed_capture
from railway_mls.synthetic import tracking_scene
from tests.test_prediction_catalog import experiment_document


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(importlib.util.find_spec("laspy"), "optional LAS support")
class BenchmarkTests(unittest.TestCase):
    def setUp(self):
        import laspy
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.raw = self.root/"raw"
        self.raw.mkdir()
        self.refs = self.root/"references"
        self.refs.mkdir()
        self.scene = tracking_scene(length=4)
        xyz = np.concatenate([self.scene.cloud.xyz+[0, 20*i, 0] for i in range(9)])
        data = laspy.LasData(laspy.LasHeader(version="1.2", point_format=1))
        data.header.scales = [.001]*3
        data.x, data.y, data.z = np.repeat(xyz, 2, axis=0).T
        data.write(self.raw/"cloud.las")
        self.document = experiment_document()
        inf = self.document["inference"]
        inf["tiles"] = [{"path": "cloud.las", "sha256": self.digest(self.raw/"cloud.las")}]
        first = deepcopy(inf["runs"][0])
        inf["runs"] = []
        theta = self.scene.seed.angle*np.pi/180
        self.along = np.array([np.cos(theta), np.sin(theta), 0])
        self.across = np.array([-np.sin(theta), np.cos(theta), 0])
        self.seeds = {}
        cases, mappings = [], []
        for pid, count in enumerate((6, 6, 4, 1, 1, 2, 2, 1, 1), 1):
            origin = self.scene.seed.point.as_array()+[0, 20*(pid-1), 0]
            seed = {"xyz": (origin+2*self.along).tolist(), "angle": self.scene.seed.angle, "mode": self.scene.seed.mode}
            self.seeds[pid] = seed
            run = deepcopy(first)
            run.update(run_id=f"path-{pid}", status="ready" if pid == 3 else "pending",
                       seed=seed if pid == 3 else None, direction="bidirectional", seed_provenance="Generated fixture")
            inf["runs"].append(run)
            mappings.append({"run_id": f"path-{pid}", "path_id": pid, "status": "pending", "layer": None,
                             "note": "Generated intended route; no automatic layer selection"})
            for side, transverse in (("left", 0), ("right", 1.5)):
                endpoints = [origin+transverse*self.across, origin+4*self.along+transverse*self.across]
                (self.refs/f"p{pid}-{side}.txt").write_text("1\n"+','.join(map(str, np.array(endpoints).ravel()))+'\n')
            points = np.array([origin+.5*self.along, origin+2.2*self.along,
                               origin+3.5*self.along+1.5*self.across, origin+1.2*self.along+.1*self.across])
            for part in range(1, count+1):
                cid = f"P{pid}_{part:02}"
                ref = laspy.LasData(laspy.LasHeader(version="1.2", point_format=1))
                ref.header.scales = [.001]*3
                ref.x, ref.y, ref.z = points.T
                ref.raw_classification = np.array([3, 3, 3, 1], dtype=np.uint8)
                ref.write(self.refs/f"{cid}.las")
                files = {"las": f"{cid}.las", "left": f"p{pid}-left.txt", "right": f"p{pid}-right.txt"}
                cases.append({"case_id": cid, "path_id": pid, **files,
                              "sha256": {key: self.digest(self.refs/value) for key, value in files.items()}})
        inf["data_root"] = "raw"
        self.reference = self.root/"reference.json"
        self.reference.write_text(json.dumps({"schema_version": 1, "profile": "source2018",
                                              "dataset_root": "references", "cases": cases}))
        self.document["evaluation"] = {"reference_manifest": "reference.json", "expected_path_ids": list(range(1, 10)),
                                       "mappings": mappings}
        self.manifest = self.root/"experiment.json"
        self.save()
        self.cache = self.root/"cache"
        build_scene_cache(load_inference_spec(self.manifest), self.cache)

    def digest(self, path):
        return file_fingerprint(path)["sha256"]

    def save(self):
        self.manifest.write_text(json.dumps(self.document))

    def predict(self, name="batch", **kwargs):
        return run_benchmark(self.manifest, self.cache, self.root/name, max_steps=2, **kwargs)

    def review(self, batches, output="reviewed.json"):
        doc = json.loads((self.root/batches[0]/"mapping-template.json").read_text())
        doc["prediction_sources"] = []
        candidates = []
        for i, batch in enumerate(batches):
            template = json.loads((self.root/batch/"mapping-template.json").read_text())
            for source in template["prediction_sources"]:
                source["source_id"] = f"batch-{i}"
                doc["prediction_sources"].append(source)
            rows = json.loads((self.root/batch/"layer-candidates.json").read_text())["candidates"]
            for row in rows:
                row["fragment"]["source_id"] = f"batch-{i}"
                candidates.append(row)
        for mapping in doc["paths"]:
            candidate = next((row for row in candidates if row["intended_path_id"] == mapping["path_id"]
                              and row["fragment"]["layer"] == 0), None)
            if candidate:
                mapping.update(status="reviewed", fragments=[candidate["fragment"]],
                               review_note="Explicit review of generated independent rail corridors")
        file = self.root/output
        file.write_text(json.dumps(doc))
        return file

    def test_plan_accounts_for_nine_paths_and_all_parts_without_opening_points(self):
        with patch("railway_mls.evaluation.io.iter_reference_las", side_effect=AssertionError("no reference decode")), \
                patch("railway_mls.prediction.runner.run_predictions", side_effect=AssertionError("no tracking")):
            report = plan_benchmark(self.manifest, self.root/"plan")
        self.assertEqual((report["accounted_path_count"], report["accounted_case_count"]), (9, 24))
        self.assertEqual(report["paths"][2]["prediction_status"], "not_run")
        self.assertEqual(report["paths"][0]["prediction_status"], "missing_seed")
        self.assertIsNone(report["cases"][0]["python"])
        self.assertFalse(report["all_paths_have_seeds"])
        self.assertFalse(report["reference_point_records_read"])

    def test_predictions_keep_missing_paths_and_require_explicit_layer_review(self):
        report = self.predict()
        self.assertEqual(report["paths"][2]["prediction_status"], "partial_prediction")
        self.assertEqual(report["paths"][2]["layer_review_status"], "pending")
        self.assertEqual(len(report["run_summaries"]), 1)
        self.assertEqual(report["run_summaries"][0]["status"], "step_limit")
        template = json.loads((self.root/"batch/mapping-template.json").read_text())
        self.assertTrue(all(m["status"] == "pending" and not m["fragments"] for m in template["paths"]))
        self.assertFalse(report["all_reference_cases_evaluated"])

    def test_partial_evaluation_retains_unavailable_scores_for_missing_paths(self):
        self.predict()
        mapping = self.review(["batch"])
        report = evaluate_benchmark(self.manifest, [self.root/"batch"], mapping, self.root/"evaluation", chunk_size=2)
        self.assertEqual(report["status"], "partial")
        self.assertEqual(sum(c["evaluation_status"] == "evaluated" for c in report["cases"]), 4)
        self.assertEqual(report["cases"][0]["reason"], "missing_seed")
        self.assertIsNone(report["cases"][0]["python"])
        self.assertFalse(report["full_spatial_coverage"])
        self.assertEqual(report["paths"][2]["evaluation_status"], "evaluated")
        self.assertEqual(report["paths"][2]["layer_review_status"], "reviewed")
        self.assertTrue((self.root/"evaluation/matched/python/P3_04.json").is_file())

    def test_all_nine_independent_runs_and_24_parts_are_accounted_when_seeded(self):
        for pid, run in enumerate(self.document["inference"]["runs"], 1):
            run.update(status="ready", seed=self.seeds[pid])
        self.save()
        predictions = self.predict()
        self.assertEqual(len(predictions["run_summaries"]), 9)
        self.assertTrue(predictions["all_paths_have_seeds"])
        mapping = self.review(["batch"])
        report = evaluate_benchmark(self.manifest, [self.root/"batch"], mapping, self.root/"evaluation")
        self.assertEqual(report["status"], "evaluated")
        self.assertTrue(report["all_reference_cases_evaluated"])
        self.assertEqual(len(report["cases"]), 24)
        self.assertTrue(all(c["python"] is not None and c["original"] is not None for c in report["cases"]))
        self.assertFalse(report["full_spatial_coverage"])  # Scoring a scope never proves its coverage.

    def test_disjoint_batches_can_be_evaluated_without_resuming_process_state(self):
        run = self.document["inference"]["runs"][0]
        run.update(status="ready", seed=self.seeds[1])
        self.save()
        self.predict("a", run_ids=["path-1"])
        self.predict("b", run_ids=["path-3"])
        mapping = self.review(["a", "b"])
        report = evaluate_benchmark(self.manifest, [self.root/"a", self.root/"b"], mapping, self.root/"evaluation")
        self.assertEqual(sum(c["evaluation_status"] == "evaluated" for c in report["cases"]), 10)

    def test_separately_named_restart_gets_fresh_labels_and_identical_rng_for_same_pose(self):
        run = deepcopy(self.document["inference"]["runs"][2])
        run["run_id"] = "path-3-restart"
        self.document["inference"]["runs"].append(run)
        mapping = deepcopy(self.document["evaluation"]["mappings"][2])
        mapping["run_id"] = run["run_id"]
        self.document["evaluation"]["mappings"].append(mapping)
        self.save()
        report = self.predict()
        a, b = report["run_summaries"]
        self.assertEqual(a["random"], b["random"])
        self.assertEqual(a["artifacts"]["vectors.csv"]["sha256"], b["artifacts"]["vectors.csv"]["sha256"])
        self.assertEqual(a["artifacts"]["state/class_codes.bin"]["sha256"], b["artifacts"]["state/class_codes.bin"]["sha256"])

    def test_repeating_the_same_run_across_batches_cannot_select_a_favorable_result(self):
        self.predict("a")
        self.predict("b")
        with self.assertRaisesRegex(ValueError, "several batches"):
            evaluate_benchmark(self.manifest, [self.root/"a", self.root/"b"], self.review(["a"]), self.root/"evaluation")

    def test_all_pending_manifest_writes_missing_status_without_needing_cache(self):
        run = self.document["inference"]["runs"][2]
        run.update(status="pending", seed=None)
        self.save()
        report = run_benchmark(self.manifest, self.root/"absent-cache", self.root/"pending")
        self.assertTrue(all(p["prediction_status"] == "missing_seed" for p in report["paths"]))
        self.assertEqual(report["prediction_exports"], [])

    def test_run_selection_failures_do_not_create_outputs(self):
        for ids in (["path-1"], ["path-3", "path-3"], ["unknown"]):
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                self.predict(run_ids=ids)
            self.assertFalse((self.root/"batch").exists())

    def test_definition_drift_cannot_mix_prior_batches_with_new_seeds(self):
        self.predict()
        mapping = self.review(["batch"])
        self.document["inference"]["runs"][0].update(status="ready", seed=self.seeds[1])
        self.save()
        with self.assertRaisesRegex(ValueError, "frozen benchmark"):
            evaluate_benchmark(self.manifest, [self.root/"batch"], mapping, self.root/"evaluation")

    def test_reviewed_path_must_match_intended_route_before_scoring(self):
        self.predict()
        mapping = self.review(["batch"])
        doc = json.loads(mapping.read_text())
        a, b = doc["paths"][0], doc["paths"][2]
        a.update(status="reviewed", fragments=b["fragments"], review_note="Wrong intended path")
        b.update(status="pending", fragments=[])
        mapping.write_text(json.dumps(doc))
        with self.assertRaisesRegex(ValueError, "intended path"):
            evaluate_benchmark(self.manifest, [self.root/"batch"], mapping, self.root/"evaluation")
        self.assertFalse((self.root/"evaluation").exists())

    def test_batch_root_hash_drift_is_rejected(self):
        self.predict()
        mapping = self.review(["batch"])
        path = self.root/"batch/predictions/manifest.json"
        path.write_text(path.read_text()+' ')
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            evaluate_benchmark(self.manifest, [self.root/"batch"], mapping, self.root/"evaluation")

    def test_primary_profile_rejects_repairs_and_missing_path_contracts(self):
        original = deepcopy(self.document)
        for change in ("repair", "scope", "ambiguous"):
            self.document = deepcopy(original)
            if change == "repair":
                self.document["inference"]["runs"][0]["branch_policy"] = "fitted_references"
            elif change == "scope":
                self.document["evaluation"]["expected_path_ids"] = list(range(1, 9))
            else:
                extra = deepcopy(self.document["evaluation"]["mappings"][2])
                extra.update(status="reviewed", path_id=1, layer=1)
                self.document["evaluation"]["mappings"].append(extra)
            self.save()
            with self.subTest(change=change), self.assertRaises(ValueError):
                plan_benchmark(self.manifest, self.root/"invalid")

    def test_prediction_failure_is_recorded_instead_of_claiming_success(self):
        with patch("railway_mls.prediction.benchmark.run_predictions", side_effect=ValueError("generated cache mismatch")):
            report = self.predict()
        self.assertEqual(report["status"], "failed")
        self.assertIn("generated cache mismatch", report["failures"][0]["error"])

    def test_interrupted_prediction_admission_is_recorded(self):
        with patch("railway_mls.prediction.benchmark.run_predictions", side_effect=KeyboardInterrupt()):
            report = self.predict()
        self.assertEqual(report["status"], "interrupted")
        self.assertEqual(report["accounted_case_count"], 24)

    def test_evaluation_stage_failure_keeps_all_cases_unavailable(self):
        self.predict()
        mapping = self.review(["batch"])
        with patch("railway_mls.prediction.benchmark.run_matched_evaluation", side_effect=ValueError("generated plot/dependency failure")):
            report = evaluate_benchmark(self.manifest, [self.root/"batch"], mapping, self.root/"evaluation")
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["accounted_case_count"], 24)
        self.assertTrue(all(c["python"] is None for c in report["cases"]))

    def test_new_output_paths_and_inputs_are_protected(self):
        plan_benchmark(self.manifest, self.root/"plan")
        with self.assertRaises(FileExistsError):
            plan_benchmark(self.manifest, self.root/"plan")
        for destination in (self.raw/"child", self.refs/"child", self.cache/"child"):
            with self.subTest(destination=destination), self.assertRaises(ValueError):
                run_benchmark(self.manifest, self.cache, destination)

    def test_manual_seed_capture_versions_manifest_and_rebases_paths(self):
        captured = self.root/"seed.json"
        before = self.digest(self.manifest)
        write_seed_capture(captured, self.scene.seed, data_file=self.raw/"cloud.las", random_seed=1,
                           config=ExtractionConfig(), provenance="Manual generated cloud selection")
        output = self.root/"new/experiment.json"
        prepare_seed_manifest(self.manifest, "path-1", captured, output, provenance="Reviewed intended path 1")
        doc = load_prediction_manifest(output)
        self.assertEqual(doc.inference.runs[0].status, "ready")
        self.assertEqual(doc.inference.runs[0].seed.xyz, tuple(self.scene.seed.point.as_array()))
        self.assertEqual(doc.inference.runs[0].direction, "bidirectional")
        self.assertEqual(doc.inference.runs[0].branch_policy, "strict")
        self.assertEqual(doc.inference.data_root, self.raw)
        self.assertEqual(doc.evaluation.reference_manifest, self.reference)
        self.assertEqual(self.digest(self.manifest), before)
        self.assertIn(self.digest(captured), doc.inference.runs[0].seed_provenance)

    def test_capture_cannot_reuse_unknown_raw_tile_or_incomplete_pose(self):
        captured = self.root/"seed.json"
        write_seed_capture(captured, self.scene.seed, data_file=self.raw/"cloud.las", random_seed=1,
                           config=ExtractionConfig(), provenance="Generated manual pose")
        valid = json.loads(captured.read_text())
        for bad in ("tile", "height", "angle"):
            doc = deepcopy(valid)
            if bad == "tile":
                doc["raw_source"]["sha256"] = "0"*64
            elif bad == "height":
                doc["seed"]["xyz"] = [1, 2]
            else:
                doc["seed"]["angle"] = float('nan')
            captured.write_text(json.dumps(doc))
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                prepare_seed_manifest(self.manifest, "path-1", captured, self.root/"invalid.json", provenance="Review")

    def test_seed_capture_and_definition_never_overwrite_existing_files(self):
        captured = self.root/"seed.json"
        args = dict(data_file=self.raw/"cloud.las", random_seed=1, config=ExtractionConfig(), provenance="Manual pose")
        write_seed_capture(captured, self.scene.seed, **args)
        with self.assertRaises(FileExistsError):
            write_seed_capture(captured, self.scene.seed, **args)
        with self.assertRaises(FileExistsError):
            prepare_seed_manifest(self.manifest, "path-1", captured, self.manifest, provenance="Review")

    def test_cli_reports_incomplete_experiment_exit_code_without_gui(self):
        result = subprocess.run([sys.executable, "-m", "railway_mls", "benchmark-plan", "--manifest", str(self.manifest),
                                 "--output", str(self.root/"cli")], text=True, capture_output=True)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(json.loads(result.stdout)["accounted_cases"], 24)

    @unittest.skipUnless(importlib.util.find_spec("matplotlib"), "optional evaluation plots")
    def test_route_preview_does_not_read_reference_points_or_calculate_scores(self):
        from railway_mls.prediction.review import preview_benchmark
        self.predict()
        with patch("railway_mls.evaluation.core.evaluate_chunks", side_effect=AssertionError("no scoring")), \
                patch("railway_mls.evaluation.io.iter_reference_las", side_effect=AssertionError("no label reads")):
            report = preview_benchmark(self.manifest, [self.root/"batch"], self.root/"review")
        self.assertFalse(report["evaluation_performed"])
        self.assertFalse(report["reference_point_records_read"])
        self.assertTrue(report["plots"])
        image = Path(report["plots"][0]["path"])
        self.assertEqual(image.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")


class VancouverBenchmarkDefinitionTests(unittest.TestCase):
    def test_all_47_original_tiles_nine_runs_and_only_complete_source_seed(self):
        experiment = load_prediction_manifest(ROOT/"configs/prediction-vancouver-benchmark.json")
        self.assertEqual(len(experiment.inference.tiles), 47)
        self.assertEqual(len(experiment.inference.runs), 9)
        ready = [r for r in experiment.inference.runs if r.status == "ready"]
        self.assertEqual([r.run_id for r in ready], ["vancouver-path-3"])
        self.assertEqual(ready[0].seed.xyz, (513194.41, 5458237.85, 6.61))
        self.assertTrue(all(r.direction == "bidirectional" and r.branch_policy == "strict" for r in experiment.inference.runs))
        self.assertEqual(experiment.evaluation.expected_path_ids, tuple(range(1, 10)))
        self.assertTrue(all(m.status == "pending" and m.layer is None for m in experiment.evaluation.mappings))
        self.assertNotIn("57.las", [t.path.name for t in experiment.inference.tiles])
