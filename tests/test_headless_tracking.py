from copy import deepcopy
from dataclasses import replace
from hashlib import sha256
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_array_equal

from railway_mls import GridIndex, MsvcRand, Point3D, TrackLayer, TrackingEngine
from railway_mls.artifacts import strict_json_value
from railway_mls.branching import BranchStart, TraceCoverage
from railway_mls.continuations import RailPair
from railway_mls.pipeline import _state_signature
from railway_mls.prediction import build_scene_cache, load_inference_spec, open_scene_cache
from railway_mls.prediction.branching import _trace_branch, track_branches
from railway_mls.prediction.resources import ResourceLimitError
from railway_mls.prediction.runner import _Journal, run_predictions
from railway_mls.prediction.switch_probe import probe_switch
from railway_mls.synthetic import bidirectional_scene, branch_scene, tracking_scene
from railway_mls.tracking import SequentialTracker


ROOT = Path(__file__).resolve().parents[1]


def state(value):
    return strict_json_value({"state": _state_signature(value)})


def make_tracker(*, compact, side_rails=False, callback=None):
    scene = tracking_scene(side_rails=side_rails)
    rng = MsvcRand(1)
    engine = TrackingEngine(GridIndex(scene.cloud), config=scene.config, rng=rng)
    initialized = engine.initialize(scene.seed)
    if not initialized.success:
        raise AssertionError(initialized.status)
    tracker = SequentialTracker(engine.current_layer, engine.grid, initialized.initial_heights,
                                config=scene.config, rng=rng, retain_history=not compact,
                                step_callback=callback)
    return tracker, scene, rng


class CompactTrackingTests(unittest.TestCase):
    def test_compact_history_preserves_vectors_full_source_state_labels_and_rng(self):
        for side_rails in (False, True):
            with self.subTest(side_rails=side_rails):
                a, first, ar = make_tracker(compact=False, side_rails=side_rails)
                b, second, br = make_tracker(compact=True, side_rails=side_rails)
                ra, rb = a.run(), b.run()
                self.assertEqual((ra.status, ra.attempted_steps, ra.accepted_steps),
                                 (rb.status, rb.attempted_steps, rb.accepted_steps))
                self.assertEqual(state(ra.layer), state(rb.layer))
                self.assertEqual(state(ra.completed_stacks), state(rb.completed_stacks))
                assert_array_equal(first.cloud.class_codes, second.cloud.class_codes)
                assert_array_equal(first.cloud.segment_ids, second.cloud.segment_ids)
                self.assertEqual((ar.state, ar.draw_count), (br.state, br.draw_count))
                self.assertEqual(ra.reference_comparable, rb.reference_comparable)
                self.assertEqual((ra.diagnostic_counts, ra.model_status_counts),
                                 (rb.diagnostic_counts, rb.model_status_counts))
                self.assertGreater(len(ra.steps), 1)
                self.assertEqual(len(rb.steps), 1)
                self.assertEqual(rb.steps[0].number, rb.attempted_steps)
                self.assertFalse(rb.history_complete)

    def test_compact_resume_and_zero_limit_use_counters_not_retained_list_length(self):
        a, _, ar = make_tracker(compact=False)
        b, _, br = make_tracker(compact=True)
        self.assertEqual(b.run(max_steps=0).attempted_steps, 0)
        for count in (1, 2, 1):
            ra, rb = a.run(max_steps=count), b.run(max_steps=count)
            self.assertEqual(ra.attempted_steps, rb.attempted_steps)
            self.assertEqual(ra.accepted_steps, rb.accepted_steps)
        ra, rb = a.run(), b.run()
        self.assertEqual(state(ra.layer), state(rb.layer))
        self.assertEqual((ar.state, ar.draw_count), (br.state, br.draw_count))
        with self.assertRaises(ValueError):
            b.run(max_steps=-1)

    def test_dropped_model_findings_survive_a_last_step_without_models(self):
        tracker, _, _ = make_tracker(compact=True)
        result = tracker.run()
        self.assertEqual(result.steps[-1].models, ())
        self.assertFalse(result.reference_comparable)
        self.assertFalse(result.models_reference_comparable)
        self.assertTrue(dict(result.diagnostic_counts))
        self.assertGreater(result.accepted_steps, len(result.steps))

    def test_observer_is_identical_for_full_compact_and_callback_failure_retains_state(self):
        logs = [[], []]
        a, _, ar = make_tracker(compact=False, callback=lambda s: logs[0].append((s.number, s.status)))
        b, _, br = make_tracker(compact=True, callback=lambda s: logs[1].append((s.number, s.status)))
        a.run()
        b.run()
        self.assertEqual(logs[0], logs[1])
        self.assertEqual((ar.state, ar.draw_count), (br.state, br.draw_count))
        def abort(_):
            raise ResourceLimitError("controlled observer stop")
        c, _, _ = make_tracker(compact=True, callback=abort)
        with self.assertRaises(ResourceLimitError):
            c.run()
        self.assertEqual(c.attempted_count, 1)
        self.assertEqual(c.steps[-1].number, 1)

    def test_source_engine_branch_orchestration_and_bidirectional_results_match(self):
        for scenario in ("branches", "bidirectional"):
            engines, workflows, randoms = [], [], []
            for compact in (False, True):
                scene = branch_scene() if scenario == "branches" else bidirectional_scene()
                rng = MsvcRand(1)
                engine = TrackingEngine(GridIndex(scene.cloud), config=scene.config, rng=rng,
                                        retain_history=not compact)
                if scenario == "branches":
                    engine.toe_points.append(replace(scene.toe))
                self.assertTrue(engine.initialize(scene.seed).success)
                workflows.append(engine.run_pass() if scenario == "branches" else engine.run_bidirectional())
                engines.append(engine)
                randoms.append((rng.state, rng.draw_count))
            self.assertEqual(workflows[0].status, workflows[1].status)
            self.assertEqual(workflows[0].reference_comparable, workflows[1].reference_comparable)
            self.assertEqual(state(engines[0].layers), state(engines[1].layers))
            assert_array_equal(engines[0].grid.cloud.class_codes, engines[1].grid.cloud.class_codes)
            self.assertEqual(randoms[0], randoms[1])


@unittest.skipUnless(importlib.util.find_spec("laspy"), "optional LAS support")
class HeadlessPredictionTests(unittest.TestCase):
    def setUp(self):
        import laspy
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        scene = tracking_scene()
        self.scene = scene
        cut = scene.seed.point.x+3
        groups = [scene.cloud.xyz[scene.cloud.xyz[:, 0] <= cut], scene.cloud.xyz[scene.cloud.xyz[:, 0] > cut]]
        tiles = []
        for number, xyz in enumerate(groups):
            path = self.root/f"{number}.las"
            data = laspy.LasData(laspy.LasHeader(version="1.2", point_format=1))
            data.header.scales = [.001]*3
            data.x, data.y, data.z = np.repeat(xyz, 2, axis=0).T
            data.write(path)
            tiles.append({"path": path.name, "sha256": sha256(path.read_bytes()).hexdigest()})
        document = json.loads((ROOT/"configs/prediction-vancouver-pilot.json").read_text())
        document["inference"]["data_root"] = str(self.root)
        document["inference"]["tiles"] = tiles
        run = deepcopy(document["inference"]["runs"][0])
        run.update(run_id="generated", seed={"xyz": scene.seed.point.as_array().tolist(),
                   "angle": scene.seed.angle, "mode": scene.seed.mode}, seed_provenance="generated fixture")
        pending = deepcopy(document["inference"]["runs"][1])
        self.document = document
        document["inference"]["runs"] = [run, pending]
        document["evaluation"] = None  # Even invalid/missing evaluation content is not an inference input.
        self.manifest = self.root/"prediction.json"
        self.save()
        self.cache = self.root/"cache"
        build_scene_cache(load_inference_spec(self.manifest), self.cache, chunk_size=101)

    def save(self):
        self.manifest.write_text(json.dumps(self.document))

    def predict(self, name, **kwargs):
        result = run_predictions(self.manifest, self.cache, self.root/name, **kwargs)
        return result, result["runs"][0]

    def test_raw_run_crosses_tile_boundary_and_exports_evaluator_compatible_vectors(self):
        from railway_mls.evaluation.io import read_python_vectors
        result, run = self.predict("result")
        self.assertIn(run["status"], ("completed", "completed_with_limitations"))
        self.assertGreater(run["coverage"]["accepted_centers_beyond_first_tile"], 0)
        self.assertGreater(run["invocations"][0]["accepted_steps"], 3)
        self.assertIn("1", run["journal_invocations"][0]["support_tiles"])
        self.assertFalse(result["reference_files_read"])
        self.assertFalse(result["evaluation_performed"])
        self.assertFalse(result["nine_path_complete"])
        left, right = read_python_vectors(self.root/"result/generated/vectors.csv", layer=0)
        self.assertGreater(len(left), 3)
        self.assertEqual(len(left), len(right))

    def test_compact_full_and_repeated_runs_have_identical_vectors_labels_rng_and_journal(self):
        a, ra = self.predict("compact")
        b, rb = self.predict("full", retain_history=True)
        _, rc = self.predict("repeat")
        for name in ("vectors.csv", "segments.csv", "steps.jsonl", "state/class_codes.bin", "state/segment_ids.bin"):
            self.assertEqual(ra["artifacts"][name], rb["artifacts"][name], name)
            self.assertEqual(ra["artifacts"][name], rc["artifacts"][name], name)
        self.assertEqual(ra["random"], rb["random"])
        self.assertEqual(ra["limitations_encountered"], rb["limitations_encountered"])
        self.assertEqual(ra["invocations"][0]["retained_diagnostic_steps"], 1)
        self.assertGreater(rb["invocations"][0]["retained_diagnostic_steps"], 1)

    def test_zero_and_positive_caps_are_explicit_and_defer_postpasses(self):
        result, run = self.predict("zero", max_steps=0)
        self.assertEqual(result["status"], "partial")
        self.assertEqual(run["status"], "step_limit")
        self.assertTrue(run["diagnostic_limit_reached"])
        self.assertEqual(run["invocations"][0]["attempted_steps"], 0)
        self.assertEqual(run["invocations"][0]["recognition"], [])
        _, run = self.predict("one", max_steps=1)
        self.assertEqual(run["invocations"][0]["accepted_steps"], 1)
        self.assertEqual(run["invocations"][0]["toes"], [])

    def test_switch_probe_without_a_split_preserves_the_source_prefix(self):
        self.predict("strict", max_steps=3)
        output = self.root / "probe"
        report = probe_switch(
            self.manifest, self.cache, output, run_id="generated", split_step=3, lookahead_steps=4
        )
        self.assertEqual(report["status"], "no_split")
        self.assertEqual(report["prefix"]["split"]["step"], 3)
        self.assertEqual(report["branches"], [])
        self.assertFalse(report["reference_files_read"])
        self.assertEqual(
            (output / "prefix" / "steps.jsonl").read_bytes(),
            (self.root / "strict" / "generated" / "steps.jsonl").read_bytes(),
        )
        self.assertFalse(list(output.glob(".state-*")))

    def test_automatic_discovery_preserves_source_journal_on_a_straight_track(self):
        self.predict("strict", max_steps=5)
        output = self.root / "branches"
        report = track_branches(self.manifest, self.cache, output, run_id="generated", max_steps=5)
        self.assertEqual(report["status"], "limits_reached")
        self.assertEqual(report["source"]["journal_routes"][0]["accepted_steps"], 5)
        self.assertEqual(report["branches"], [])
        self.assertFalse(report["reference_files_read"])
        self.assertEqual(
            (output / "source" / "steps.jsonl").read_bytes(),
            (self.root / "strict" / "generated" / "steps.jsonl").read_bytes(),
        )
        self.assertFalse(list(output.glob(".states-*")))

    def test_branch_queue_survives_source_and_trial_stops_and_tracks_child_budget(self):
        begins = (Point3D(1000, 2000, 0.2), Point3D(1000, 2001.5, 0.2))
        pair = RailPair((1, 1), begins, tuple(p + Point3D(1, 0, 0) for p in begins),
                        (60, 60), 1.5, 1.0, 0.15)
        first = BranchStart("source-00", 1, 1, pair, 123)
        second = BranchStart("source-00", 2, 2, pair, 456)
        child = BranchStart("branch-002", 4, 6, pair, 789)
        visited = []

        def source(template, config, run, output, cap, coverage, queue, progress):
            queue.extend((first, second))
            return {"status": "undefined_source_head_extraction", "error": None,
                    "initialization": {"source_return": True}}

        def branch(template, config, start, output, route, cap, coverage):
            visited.append((route, start, cap))
            if start == first:
                raise RuntimeError("controlled trial failure")
            return {"route": route, "confirmed_at": 3, "error": None,
                    "status": "rejoined"}, [child] if start == second else []

        with (
            patch("railway_mls.prediction.branching._trace_source", side_effect=source),
            patch("railway_mls.prediction.branching._trace_branch", side_effect=branch),
        ):
            report = track_branches(self.manifest, self.cache, self.root / "branches",
                                    run_id="generated", max_steps=10)
        self.assertEqual(visited, [
            ("branch-001", first, 9), ("branch-002", second, 8), ("branch-003", child, 4),
        ])
        self.assertEqual(report["status"], "partial")
        self.assertEqual(report["pending_candidates"], 0)
        self.assertEqual(report["branches"][0]["error"]["message"], "controlled trial failure")
        self.assertEqual(report["branches"][-1]["status"], "rejoined")

    def test_confirmed_branch_stops_at_a_rejoin_and_exports_only_to_the_join(self):
        spec = load_inference_spec(self.manifest)
        cache = open_scene_cache(self.cache, inference=spec)
        seed, config = self.scene.seed, spec.config
        angle = np.deg2rad(seed.angle)
        tangent = Point3D(np.cos(angle), np.sin(angle), 0)
        normal = Point3D(-tangent.y, tangent.x, 0)

        def rails(station):
            first = seed.point + tangent * station
            return first, first + normal * config.track_gauge

        coverage = TraceCoverage(config)
        coverage.add("known", [rails(station) for station in np.arange(3.5, 8.5, 0.5)])
        pair = RailPair((1, 1), rails(-0.5), rails(0.5), (60, 60), config.track_gauge, 1.0, 0.15)
        start = BranchStart("parent", 1, 1, pair, 1)
        with cache.new_run(self.root / "template") as template:
            report, discoveries = _trace_branch(
                template, config, start, self.root, "trial", 8, coverage
            )
        self.assertEqual(report["confirmed_at"], 3)
        self.assertEqual(report["status"], "rejoined")
        self.assertEqual(report["rejoin"]["route"], "known")
        self.assertEqual(report["rejoin"]["window"], report["exported_steps"])
        self.assertEqual(report["accepted_steps"], report["exported_steps"] + 2)
        self.assertEqual(report["layers"][0]["vertices"]["center"], report["exported_steps"] + 1)
        self.assertEqual(discoveries, [])

    def test_pending_unknown_and_duplicate_selection_cannot_silently_run(self):
        for runs in (["vancouver-path-1"], ["unknown"], ["generated", "generated"]):
            with self.subTest(runs=runs), self.assertRaises(ValueError):
                self.predict("invalid", run_ids=runs)
            self.assertFalse((self.root/"invalid").exists())

    def test_repair_profile_is_not_silently_used_for_primary_source_run(self):
        self.document["inference"]["runs"][0]["branch_policy"] = "fitted_references"
        self.save()
        with self.assertRaisesRegex(ValueError, "strict branches"):
            self.predict("repaired")

    def test_output_protection_and_cache_compatibility(self):
        self.predict("result", max_steps=0)
        with self.assertRaises(FileExistsError):
            self.predict("result")
        with self.assertRaisesRegex(ValueError, "outside the immutable"):
            run_predictions(self.manifest, self.cache, self.cache/"inside")
        self.document["inference"]["tiles"].reverse()
        self.save()
        with self.assertRaisesRegex(ValueError, "incompatible"):
            self.predict("mismatch")

    def test_resource_stop_keeps_partial_vectors_and_known_diagnostics(self):
        real = _Journal.resources
        def limit(journal, extra=0):
            if journal.last is not None:
                raise ResourceLimitError("controlled memory exhaustion after one source step")
            real(journal, extra)
        with patch.object(_Journal, "resources", limit):
            result, run = self.predict("limited")
        self.assertEqual(result["status"], "partial")
        self.assertEqual(run["status"], "resource_limited")
        self.assertEqual(run["last_step"]["number"], 1)
        self.assertGreater(run["layers"][0]["vertices"]["rail1"], 1)
        self.assertFalse(run["postpass_summary_complete"])
        self.assertTrue(run["journal_invocations"][0]["diagnostic_counts"])
        self.assertIn("LIM-002", run["limitations_encountered"])

    def test_interrupt_keeps_partial_source_state_as_explicit_interrupted_result(self):
        real = _Journal.resources
        def interrupt(journal, extra=0):
            if journal.last is not None:
                raise KeyboardInterrupt()
            real(journal, extra)
        with patch.object(_Journal, "resources", interrupt):
            _, run = self.predict("interrupted")
        self.assertEqual(run["status"], "interrupted")
        self.assertEqual(run["last_step"]["number"], 1)
        self.assertTrue((self.root/"interrupted/generated/vectors.csv").is_file())

    def test_bidirectional_headless_run_preserves_source_pass_identifiers(self):
        run = self.document["inference"]["runs"][0]
        run["direction"] = "bidirectional"
        theta = self.scene.seed.angle*np.pi/180
        run["seed"]["xyz"] = (self.scene.seed.point.as_array()+[4*np.cos(theta), 4*np.sin(theta), 0]).tolist()
        self.save()
        _, result = self.predict("both")
        self.assertEqual([r["name"] for r in result["invocations"]], ["reverse.main", "forward.main"])
        self.assertFalse(result["resume_supported"])

    def test_cli_exposes_diagnostic_exit_code_and_headless_import(self):
        args = [sys.executable, "-m", "railway_mls", "predict-rails", "--manifest", str(self.manifest),
                "--cache", str(self.cache), "--output", str(self.root/"cli"), "--max-steps", "1"]
        result = subprocess.run(args, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(json.loads(result.stdout)["status"], "partial")
        result = subprocess.run([sys.executable, "-c", "import sys; import railway_mls.prediction.runner; "
                                "assert 'open3d' not in sys.modules"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_declared_continuation_adds_raw_tile18_without_changing_original_pilot(self):
        a = load_inference_spec(ROOT/"configs/prediction-vancouver-pilot.json")
        b = load_inference_spec(ROOT/"configs/prediction-vancouver-continuation.json")
        self.assertEqual([t.path.name for t in a.tiles], ["16.las", "17.las"])
        self.assertEqual([t.path.name for t in b.tiles], ["16.las", "17.las", "18.las"])
        self.assertEqual(a.runs[0].seed, b.runs[0].seed)
        self.assertEqual(a.config, b.config)
        self.assertEqual(b.runs[0].branch_policy, "strict")


class FootprintHintTests(unittest.TestCase):
    def test_union_box_does_not_hide_a_missing_tile_footprint(self):
        first = {"row_end": 1, "bounds": {"minimum": [0, 0, 0], "maximum": [1, 3, 1]}}
        second = {"row_end": 2, "bounds": {"minimum": [0, 0, 0], "maximum": [3, 1, 1]}}
        cache = SimpleNamespace(metadata={"tiles": [first, second], "files": {},
            "bounds": {"minimum": [0, 0, 0], "maximum": [3, 3, 1]}})
        window = SimpleNamespace(vertices=[Point3D(2, 2, 0), Point3D(2.5, 2, 0),
                                           Point3D(2.5, 2.5, 0), Point3D(2, 2.5, 0)], members=None)
        segment = SimpleNamespace(track1=window, track2=window, center=Point3D(2.25, 2.25, 0))
        step = SimpleNamespace(number=1, accepted=False, status="insufficient_window_points", models=(),
            selection=None, fits=(), candidate=segment,
            attempts=[SimpleNamespace(segment=segment, phase="point_retry", length=6, extraction=None)])
        cache.path = ROOT
        with patch("pathlib.Path.stat", return_value=SimpleNamespace(st_size=0)):
            journal = _Journal(io.StringIO(), cache, None, Path("."), None, "fixture", MsvcRand(1))
        with patch.object(journal, "resources", return_value=None):
            journal(0, 1, True, step)
        self.assertEqual(journal.last["boundary_hint"], "scene_footprint_gap")


if __name__ == "__main__":
    unittest.main()
