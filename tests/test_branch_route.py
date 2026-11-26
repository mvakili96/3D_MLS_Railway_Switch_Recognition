import importlib.util
import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from railway_mls.evaluation.io import file_fingerprint
from railway_mls.evaluation.matched import load_matched_plan
from railway_mls.evaluation.table1 import run_table1_report
from railway_mls.points import Point3D
from railway_mls.prediction.route import _TrackingGeometry, export_branch_route
from railway_mls.prediction.runner import _layer_from_rail_centers, _write_geometry
from tests import test_benchmark


def write_vectors(directory, stations, y=2000):
    directory.mkdir(parents=True)
    layer = _layer_from_rail_centers([
        (Point3D(x, y, 0.2), Point3D(x, y + 1.5, 0.2)) for x in stations
    ])
    _write_geometry(directory, [layer])
    return layer


class RouteGeometryTests(unittest.TestCase):
    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        path = self.root / "source" / "accepted"
        write_vectors(path, range(10))
        self.report = {
            "source": {"journal_routes": [{"route": "source-00", "layer": 0, "accepted_layer": 0,
                                           "invocation": 0, "accepted_steps": 9}],
                       "artifacts": {"accepted/vectors.csv": file_fingerprint(path / "vectors.csv")}},
            "branches": [
                {"route": "branch-001", "parent": "source-00", "parent_window": 4,
                 "confirmed_at": 3, "exported_steps": 3, "status": "rejoined",
                 "rejoin": {"route": "source-00", "station": 6.4, "direction": 1, "window": 3}},
            ],
        }
        write_vectors(self.root / "branch-001", [3.8, 4.8, 5.8, 6.4], y=2000.4)

    def test_fork_and_rejoin_keep_whole_recorded_segments(self):
        geometry = _TrackingGeometry(self.root, self.report)
        spans = geometry.lineage("branch-001", True)
        layers, fragments = geometry.layers(spans)
        self.assertEqual([p.x for p in layers[0].rail1_vector], [0, 1, 2, 3])
        self.assertEqual([p.x for p in layers[1].rail1_vector], [3.8, 4.8, 5.8, 6.4])
        self.assertEqual([p.x for p in layers[2].rail1_vector], [7, 8, 9])
        self.assertGreater(fragments[1]["gap_before_m"], 0.8)
        self.assertGreater(fragments[2]["gap_before_m"], 0.6)

    def test_reverse_rejoin_rejects_repeated_segments(self):
        self.report["branches"][0]["rejoin"]["direction"] = -1
        geometry = _TrackingGeometry(self.root, self.report)
        with self.assertRaisesRegex(ValueError, "repeat"):
            geometry.layers(geometry.lineage("branch-001", True))

    def test_drift_in_pinned_route_geometry_is_rejected(self):
        path = self.root / "source" / "accepted" / "vectors.csv"
        with path.open("a") as stream:
            stream.write("\n")
        geometry = _TrackingGeometry(self.root, self.report)
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            geometry.layers(geometry.lineage("branch-001", False))

    def test_legacy_source_alignment_keeps_only_the_matched_forward_traversal(self):
        source = self.report["source"]
        source.pop("artifacts")
        row = source["journal_routes"][0]
        row.pop("accepted_layer")
        row.update(invocation=1, accepted_steps=4)
        # Simulate the prefix of a bidirectional run followed by its forward seed.
        layer = write_vectors(self.root / "combined", [-2, 0, 1, 2, 3, 4])
        (self.root / "combined" / "vectors.csv").replace(self.root / "source" / "vectors.csv")
        journal = self.root / "source" / "steps.jsonl"
        events = [
            {"invocation": 0, "layer": 0, "accepted": True, "number": 1,
             "source_status": "accepted", "center": layer.center_vector[0].as_array().tolist()}
        ]
        events.extend({"invocation": 1, "layer": 0, "accepted": True, "number": i,
                       "source_status": "accepted", "center": point.as_array().tolist()}
                      for i, point in enumerate(layer.center_vector[2:], 1))
        journal.write_text("".join(json.dumps(event) + "\n" for event in events))
        geometry = _TrackingGeometry(self.root, self.report)
        np.testing.assert_array_equal(geometry.vertices("source-00")[:, 0, 0], range(5))
        events[-1]["center"][0] = 400
        journal.write_text("".join(json.dumps(event) + "\n" for event in events))
        with self.assertRaisesRegex(ValueError, "unambiguously"):
            _TrackingGeometry(self.root, self.report).vertices("source-00")


@unittest.skipUnless(importlib.util.find_spec("laspy"), "optional LAS support")
class AutomaticRouteEvaluationTests(unittest.TestCase):
    def test_explicitly_reviewed_branch_export_uses_complete_existing_table_report(self):
        fixture = test_benchmark.BenchmarkTests(methodName="runTest")
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        root = fixture.root
        raw = root / "tracked"
        spec = fixture.document["inference"]
        rng = spec["runs"][2]["random_seed"]
        seed = fixture.seeds[3]
        origin = np.array(seed["xyz"])
        for name, stations in (("source/accepted", (0, 1, 2, 3)), ("branch-001", (2, 3, 4, 5))):
            directory = raw / name
            directory.mkdir(parents=True)
            poses = [(Point3D(*(origin + station * fixture.along)),
                      Point3D(*(origin + station * fixture.along + 1.5 * fixture.across)))
                     for station in stations]
            _write_geometry(directory, [_layer_from_rail_centers(poses)])
        report = {
            "kind": "automatic_branch_tracking", "schema_version": 1, "status": "partial",
            "reference_files_read": False, "config": spec["config"], "seed": seed,
            "run_id": "path-3", "direction": "bidirectional", "random_seed": rng,
            "seed_provenance": "Generated fixture", "source_baseline": spec["source_baseline"],
            "inference_manifest_sha256": fixture.digest(fixture.manifest),
            "cache_manifest_sha256": "0" * 64,
            "source": {"status": "undefined_source_head_extraction",
                       "journal_routes": [{"route": "source-01", "accepted_layer": 0,
                                           "accepted_steps": 3}]},
            "branches": [{"route": "branch-001", "parent": "source-01", "parent_window": 2,
                          "confirmed_at": 3, "exported_steps": 3, "status": "step_limit"}],
        }
        (raw / "report.json").write_text(json.dumps(report))
        export = root / "export"
        export_branch_route(fixture.manifest, raw, export, branch="branch-001")
        mapping = json.loads((export / "mapping-template.json").read_text())
        self.assertEqual(mapping["paths"][2]["status"], "pending")
        candidates = json.loads((export / "route-candidates.json").read_text())
        mapping["paths"][2].update(status="reviewed", fragments=candidates["fragments"],
                                   review_note="Explicitly reviewed the fixture route.")
        reviewed = root / "reviewed.json"
        reviewed.write_text(json.dumps(mapping))
        plan = load_matched_plan(reviewed, path_ids=[3])
        self.assertEqual(len(plan["mappings"][0]["rails"][0]), 4)
        table = run_table1_report(fixture.manifest, [export], reviewed, root / "table", path_ids=[3])
        self.assertEqual(table["status"], "completed")
        self.assertEqual(table["path_details"][0]["case_ids"],
                         [f"P3_{i:02d}" for i in range(1, 5)])
        row = next(row for row in table["rows"] if row["model"] == "python")
        self.assertAlmostEqual(row["length_xy_m"], 4)
        self.assertFalse(row["native_tracking_comparability"])
