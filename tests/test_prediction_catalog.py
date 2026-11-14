from copy import deepcopy
from dataclasses import asdict
from hashlib import sha256
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from railway_mls.config import ExtractionConfig
from railway_mls.prediction import (CatalogError, audit_prediction_manifest, catalog_scene,
                                     discover_tiles, load_inference_spec,
                                     load_prediction_manifest, write_json_new)
from railway_mls.prediction.catalog import footprint_summary, read_tile_metadata
from railway_mls.prediction.manifest import ADE_BASELINE


ROOT = Path(__file__).resolve().parents[1]


def experiment_document():
    return {
        "schema_version": 1, "kind": "prediction_experiment", "experiment_id": "synthetic-pilot",
        "inference": {
            "data_root": "raw", "tiles": [{"path": "16.las", "sha256": "0"*64}],
            "input_collection": "generated_fixture", "source_baseline": ADE_BASELINE,
            "record_selection": "per_file_even_zero_based", "bounds_policy": "fixed_header_union",
            "header_roundoff": "omit_outside_within_four_ulps", "coordinate_units": "m",
            "config": ExtractionConfig().to_dict(),
            "budgets": {"memory_bytes": 8*1024**3, "cache_disk_bytes": 100*1024**3},
            "runs": [
                {"run_id": "pilot", "status": "ready", "seed": {"xyz": [.5, 0, 0], "angle": 0, "mode": 4},
                 "seed_provenance": "Generated seed", "random_seed": 1, "direction": "single",
                 "branch_policy": "strict", "max_steps": None},
                {"run_id": "path-2", "status": "pending", "seed": None,
                 "seed_provenance": "Manual seed still required", "random_seed": 1,
                 "direction": "bidirectional", "branch_policy": "strict", "max_steps": None},
            ],
        },
        "evaluation": {
            "reference_manifest": "reference.json", "expected_path_ids": [1, 2],
            "mappings": [
                {"run_id": "pilot", "status": "pending", "path_id": None, "layer": None,
                 "note": "No route inferred from tile number"},
                {"run_id": "path-2", "status": "pending", "path_id": 2, "layer": None,
                 "note": "Intended path only"},
            ],
        },
    }


def footprint(path, xmin, xmax, ymin, ymax):
    return {"path": path, "bounds": {"minimum": [xmin, ymin, 0], "maximum": [xmax, ymax, 1]}}


class PredictionManifestTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.path = self.root/"experiment.json"
        self.document = experiment_document()
        self.save()

    def save(self):
        self.path.write_text(json.dumps(self.document))

    def test_ready_and_pending_seed_contracts_and_defaults(self):
        result = load_prediction_manifest(self.path)
        self.assertEqual(result.inference.runs[0].seed.xyz, (.5, 0, 0))
        self.assertIsNone(result.inference.runs[1].seed)
        self.assertEqual(result.inference.memory_bytes, 8*1024**3)
        self.assertEqual(result.inference.config, ExtractionConfig())
        self.assertIsNone(result.evaluation.mappings[0].path_id)

    def test_inference_loader_is_independent_of_evaluation_content(self):
        expected = load_inference_spec(self.path)
        self.document["evaluation"] = None
        self.save()
        actual = load_inference_spec(self.path)
        self.assertEqual(expected, actual)
        self.assertFalse(hasattr(actual, "evaluation"))
        self.assertNotIn("reference.json", repr(asdict(actual)))
        with self.assertRaises(CatalogError):
            load_prediction_manifest(self.path)

    def test_relative_paths_are_manifest_relative_not_working_directory(self):
        result = load_prediction_manifest(self.path)
        self.assertEqual(result.inference.tiles[0].path, self.root/"raw/16.las")
        self.assertEqual(result.evaluation.reference_manifest, self.root/"reference.json")
        moved = load_prediction_manifest(self.path, data_root=self.root/"relocated")
        self.assertEqual(moved.inference.tiles[0].path, self.root/"relocated/16.las")
        self.assertEqual(moved.evaluation.reference_manifest, result.evaluation.reference_manifest)

    def test_invalid_schema_source_sampling_and_inference_reference_fields(self):
        invalids = []
        for field, value in (("record_selection", "all_records"), ("source_baseline", "2018"),
                             ("bounds_policy", "moving_tile_origin"), ("header_roundoff", "clamp")):
            document = deepcopy(self.document)
            document["inference"][field] = value
            invalids.append(document)
        document = deepcopy(self.document)
        document["inference"]["reference_las"] = "ground-truth.las"
        invalids.append(document)
        document = deepcopy(self.document)
        document["schema_version"] = True
        invalids.append(document)
        for document in invalids:
            self.path.write_text(json.dumps(document))
            with self.subTest(document=document), self.assertRaises(CatalogError):
                load_prediction_manifest(self.path)

    def test_missing_and_nonfinite_ready_seeds_and_guessed_pending_seed_rejected(self):
        for value in (None, {"xyz": [0, 0], "angle": 0, "mode": 4},
                      {"xyz": [float("nan"), 0, 0], "angle": 0, "mode": 4}):
            self.document = experiment_document()
            self.document["inference"]["runs"][0]["seed"] = value
            self.save()
            with self.subTest(seed=value), self.assertRaises(ValueError):
                load_prediction_manifest(self.path)
        self.document = experiment_document()
        self.document["inference"]["runs"][1]["seed"] = {"xyz": [1, 2, 3], "angle": 0, "mode": 4}
        self.save()
        with self.assertRaisesRegex(CatalogError, "pending seeds"):
            load_prediction_manifest(self.path)

    def test_duplicate_tiles_runs_and_escaped_tile_paths_rejected(self):
        for kind in ("tiles", "runs", "escape"):
            self.document = experiment_document()
            if kind == "escape":
                self.document["inference"]["tiles"][0]["path"] = "../reference.las"
            else:
                self.document["inference"][kind].append(self.document["inference"][kind][0])
            self.save()
            with self.subTest(kind=kind), self.assertRaises(CatalogError):
                load_prediction_manifest(self.path)

    def test_duplicate_json_fields_rejected(self):
        text = self.path.read_text().replace('"schema_version": 1', '"schema_version": 1, "schema_version": 1')
        self.path.write_text(text)
        with self.assertRaisesRegex(CatalogError, "duplicate JSON"):
            load_prediction_manifest(self.path)

    def test_reviewed_mapping_requires_declared_path_and_explicit_layer(self):
        self.document["evaluation"]["mappings"][0]["status"] = "reviewed"
        self.save()
        with self.assertRaisesRegex(CatalogError, "explicit path ID and layer"):
            load_prediction_manifest(self.path)
        self.document["evaluation"]["mappings"][0].update(path_id=3, layer=0)
        self.save()
        with self.assertRaisesRegex(CatalogError, "outside expected scope"):
            load_prediction_manifest(self.path)

    def test_all_runs_require_an_explicit_mapping_or_pending_declaration(self):
        self.document["evaluation"]["mappings"].pop()
        self.save()
        with self.assertRaisesRegex(CatalogError, "each declared run"):
            load_prediction_manifest(self.path)

    def test_distinct_layers_in_one_run_can_map_to_different_paths(self):
        mappings = self.document["evaluation"]["mappings"]
        mappings[0].update(status="reviewed", path_id=1, layer=0)
        mappings.append({"run_id": "pilot", "status": "reviewed", "path_id": 2, "layer": 1,
                         "note": "Explicit reviewed branch layer"})
        self.save()
        self.assertEqual(len(load_prediction_manifest(self.path).evaluation.mappings), 3)
        mappings[-1]["layer"] = 0
        self.save()
        with self.assertRaisesRegex(CatalogError, "only one mapping"):
            load_prediction_manifest(self.path)

    def test_checked_in_pilot_uses_original_tiles_and_preserves_unresolved_paths(self):
        result = load_prediction_manifest(ROOT/"configs/prediction-vancouver-pilot.json")
        self.assertEqual([t.path.name for t in result.inference.tiles], ["16.las", "17.las"])
        pilot = result.inference.runs[0]
        self.assertEqual(pilot.seed.xyz, (513194.41, 5458237.85, 6.61))
        self.assertEqual((pilot.seed.angle, pilot.seed.mode, pilot.random_seed), (355, 4, 1))
        self.assertEqual((pilot.direction, pilot.branch_policy), ("single", "strict"))
        self.assertEqual(sum(r.seed is None for r in result.inference.runs), 9)
        self.assertTrue(all(m.status == "pending" for m in result.evaluation.mappings))

    def test_schema_loading_does_not_import_laspy_or_open3d(self):
        program = ("import sys; from railway_mls.prediction import load_inference_spec; "
                   f"load_inference_spec({str(self.path)!r}); "
                   "assert 'open3d' not in sys.modules; assert 'laspy' not in sys.modules")
        result = subprocess.run([sys.executable, "-c", program], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


class PredictionFootprintTests(unittest.TestCase):
    def test_connected_rectangles_and_axis_gaps_are_header_claims_only(self):
        result = footprint_summary([footprint("a", 0, 2, 0, 2), footprint("b", 1, 3, 1, 3),
                                    footprint("c", 5, 6, 0, 1)])
        self.assertEqual(result["xy_rectangle_components"], [["a", "b"], ["c"]])
        self.assertEqual(result["x_axis_gaps"], [{"minimum": 3, "maximum": 5, "width": 2}])
        self.assertFalse(result["point_occupancy_verified"])
        self.assertFalse(result["rail_continuity_verified"])

    def test_no_axis_gap_does_not_imply_rectangle_connectivity(self):
        result = footprint_summary([footprint("a", 0, 2, 0, 1), footprint("b", 0, 2, 2, 3)])
        self.assertEqual(result["x_axis_gaps"], [])
        self.assertEqual(len(result["xy_rectangle_components"]), 2)

    def test_empty_footprints_have_no_fabricated_union(self):
        self.assertIsNone(footprint_summary([])["header_bounds_union"])

    def test_new_json_writer_does_not_overwrite_and_rejects_nonfinite_payloads(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/"catalog.json"
            with self.assertRaises(ValueError):
                write_json_new(path, {"bad": float("nan")})
            self.assertFalse(path.exists())
            write_json_new(path, {"status": "ok"})
            with self.assertRaises(FileExistsError):
                write_json_new(path, {"status": "changed"})
            self.assertEqual(json.loads(path.read_text()), {"status": "ok"})


@unittest.skipUnless(importlib.util.find_spec("laspy"), "optional LAS support")
class PredictionCatalogLasTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.raw = self.root/"raw"
        self.raw.mkdir()
        self.las = self.raw/"16.las"
        self.make_las(self.las, [0, .5, 1, 1.5, 2])
        self.path = self.root/"experiment.json"
        self.document = experiment_document()
        self.document["inference"]["tiles"][0]["sha256"] = sha256(self.las.read_bytes()).hexdigest()
        self.path.write_text(json.dumps(self.document))
        # Reference/model point files intentionally do not exist. Audits may read
        # the scope JSON, but must never open these inputs or copy them into inference.
        (self.root/"reference.json").write_text(json.dumps({"schema_version": 1, "profile": "source2018",
            "cases": [{"case_id": f"P{p}_01", "path_id": p, "las": "missing-gt.las",
                       "left": "missing-original.txt", "right": "missing-original.txt"} for p in (1, 2)]}))

    def make_las(self, path, xs):
        import laspy
        header = laspy.LasHeader(point_format=1, version="1.2")
        header.scales = [.01]*3
        data = laspy.LasData(header)
        data.x, data.y, data.z = xs, [0]*len(xs), [0]*len(xs)
        data.intensity = np.arange(len(xs), dtype=np.uint16)
        data.classification = [3]*len(xs)  # Never read into inference by the catalog.
        data.write(path)

    def test_catalog_records_metadata_hashes_and_per_file_even_count(self):
        other = self.raw/"17.las"
        self.make_las(other, [2, 2.5, 3])
        result = catalog_scene([self.las, other], root=self.raw)
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["totals"]["point_count"], 8)
        self.assertEqual(result["totals"]["source_even_record_count_before_boundary_policy"], 5)
        first = result["tiles"][0]
        self.assertEqual(first["sha256"], sha256(self.las.read_bytes()).hexdigest())
        self.assertEqual(first["point_format"], 1)
        self.assertEqual(first["scales"], [.01]*3)
        self.assertEqual(first["offsets"], [0]*3)
        self.assertEqual(first["bounds"]["maximum"], [2, 0, 0])
        self.assertNotIn("classification", first)

    def test_directory_natural_order_and_explicit_order_preserved(self):
        self.make_las(self.raw/"2.LAS", [0, 1])
        self.make_las(self.raw/"11.las", [1, 2])
        (self.raw/"1.rar").write_text("archive is not a source")
        (self.raw/"1.csv").write_text("unrelated file")
        self.assertEqual([p.name for p in discover_tiles([self.raw])], ["2.LAS", "11.las", "16.las"])
        self.assertEqual([p.name for p in discover_tiles([self.las, self.raw/"2.LAS"])], ["16.las", "2.LAS"])
        with self.assertRaisesRegex(CatalogError, "duplicate"):
            discover_tiles([self.raw, self.las])

    def test_duplicate_hard_links_rejected(self):
        other = self.raw/"copy.las"
        os.link(self.las, other)
        with self.assertRaisesRegex(CatalogError, "duplicate"):
            discover_tiles([self.las, other])

    def test_empty_directories_and_explicit_archives_rejected(self):
        empty = self.root/"empty"
        empty.mkdir()
        for source in (empty, self.raw/"tiles.rar"):
            with self.subTest(source=source), self.assertRaises(CatalogError):
                discover_tiles([source])

    def test_headers_and_hashing_do_not_decode_point_arrays(self):
        import laspy
        with patch.object(laspy.LasReader, "read_points", side_effect=AssertionError("decoded points")):
            result = catalog_scene([self.las], root=self.raw)
        self.assertEqual(result["status"], "ok")

    def test_catalog_is_deterministic_across_mtime_changes(self):
        first = catalog_scene([self.las], root=self.raw)
        stat = self.las.stat()
        os.utime(self.las, ns=(stat.st_atime_ns, stat.st_mtime_ns+1000000))
        self.assertEqual(first, catalog_scene([self.las], root=self.raw))

    def test_truncated_invalid_and_missing_inputs_report_incomplete(self):
        self.las.write_bytes(self.las.read_bytes()[:-1])
        bad = self.raw/"bad.las"
        bad.write_bytes(b"not a LAS header")
        result = catalog_scene([self.las, bad, self.raw/"missing.las"], root=self.raw)
        self.assertEqual(result["status"], "incomplete")
        self.assertEqual(len(result["issues"]), 3)
        self.assertEqual(result["tiles"], [])

    def test_nonfinite_header_metadata_rejected(self):
        import laspy
        reader = laspy.open(self.las)
        reader.header.scales[0] = float("nan")
        with patch("laspy.open", return_value=reader):
            result = catalog_scene([self.las], root=self.raw)
        self.assertEqual(result["status"], "incomplete")
        self.assertIn("nonfinite", result["issues"][0]["message"])

    def test_mutation_during_hashing_detected(self):
        from railway_mls.prediction.catalog import _hash_file
        def changed(path):
            with path.open("ab") as stream:
                stream.write(b"x")
            return _hash_file(path)
        with patch("railway_mls.prediction.catalog._hash_file", side_effect=changed):
            result = catalog_scene([self.las], root=self.raw)
        self.assertEqual(result["status"], "incomplete")
        self.assertIn("changed while cataloging", result["issues"][0]["message"])

    def test_audit_validates_bytes_and_leaves_seeds_mappings_and_predictions_pending(self):
        original = self.las.read_bytes()
        result = audit_prediction_manifest(self.path)
        self.assertEqual(result["status"], "valid_with_pending")
        self.assertEqual(result["errors"], [])
        ready = result["readiness"]
        self.assertTrue(ready["input_configuration_ready"])
        self.assertEqual(ready["pending_seed_runs"], ["path-2"])
        self.assertFalse(ready["nine_path_complete"])
        self.assertFalse(ready["seed_initialization_verified"])
        self.assertFalse(ready["predictions_generated"])
        self.assertFalse(result["reference_scope"]["annotation_or_model_files_opened"])
        self.assertEqual(self.las.read_bytes(), original)

    def test_changed_hash_and_missing_file_are_not_ready(self):
        with self.las.open("ab") as stream:
            stream.write(b"different bytes")
        result = audit_prediction_manifest(self.path)
        self.assertEqual(result["status"], "invalid")
        self.assertEqual(result["errors"][0]["status"], "sha256_mismatch")
        self.assertFalse(result["readiness"]["input_configuration_ready"])
        self.las.unlink()
        self.assertEqual(audit_prediction_manifest(self.path)["status"], "invalid")

    def test_seed_inside_union_but_outside_every_footprint_rejected(self):
        self.make_las(self.las, [0, 1])
        other = self.raw/"17.las"
        self.make_las(other, [3, 4])
        self.document["inference"]["tiles"] = [{"path": p.name, "sha256": sha256(p.read_bytes()).hexdigest()}
                                                 for p in (self.las, other)]
        self.document["inference"]["runs"][0]["seed"]["xyz"] = [2, 0, 0]
        self.path.write_text(json.dumps(self.document))
        result = audit_prediction_manifest(self.path)
        self.assertEqual(result["status"], "invalid")
        self.assertTrue(any(e["status"] == "seed_outside_available_tile_footprints" for e in result["errors"]))

    def test_missing_reference_scope_is_an_error_without_touching_inference_inputs(self):
        (self.root/"reference.json").unlink()
        result = audit_prediction_manifest(self.path)
        self.assertEqual(result["status"], "invalid")
        self.assertTrue(any(e["status"] == "reference_manifest_error" for e in result["errors"]))
        self.assertEqual(result["catalog"]["status"], "ok")

    def test_invalid_reference_manifest_schema_reports_error(self):
        (self.root/"reference.json").write_text("[]")
        result = audit_prediction_manifest(self.path)
        self.assertEqual(result["status"], "invalid")
        self.assertEqual(result["errors"][0]["status"], "reference_manifest_error")

    def test_headless_cli_catalog_and_audit_return_json_and_reject_overwrite(self):
        output = self.root/"catalog.json"
        args = [sys.executable, "-m", "railway_mls", "scene-catalog", str(self.las),
                "--root", str(self.raw), "--output", str(output)]
        result = subprocess.run(args, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["status"], "ok")
        self.assertEqual(subprocess.run(args, capture_output=True).returncode, 1)
        args = [sys.executable, "-m", "railway_mls", "prediction-audit", "--manifest", str(self.path)]
        result = subprocess.run(args, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["status"], "valid_with_pending")


if __name__ == "__main__":
    unittest.main()
