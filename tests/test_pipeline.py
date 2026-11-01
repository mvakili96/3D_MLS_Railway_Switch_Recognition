from dataclasses import replace
from hashlib import sha256
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls import (
    Bounds3D,
    GridIndex,
    MsvcRand,
    PipelineOptions,
    Point3D,
    PointCloud,
    RecordedRand,
    TrackingEngine,
    build_gwad_rows,
    check_source_coverage,
    initial_location_from_picks,
    pipeline_summary,
    run_pipeline,
    write_pipeline_outputs,
)
from railway_mls.artifacts import strict_json_value
from railway_mls.coverage import (
    SOURCE_DIRECTORY,
    cpp_definitions,
    source_coverage,
    source_text_hash,
)
from railway_mls.synthetic import pipeline_scene, tracking_scene
from tests.cpp_reference import CppReference


ROOT = Path(__file__).resolve().parents[1]


def execute_scene(name, **options):
    scene = pipeline_scene(name)
    selected = PipelineOptions(bidirectional=scene.bidirectional, grades=True, gwad=True, **options)
    return run_pipeline(
        scene.cloud,
        scene.seed,
        config=scene.config,
        options=selected,
        supplied_toes=scene.supplied_toes,
        supplied_stacks=scene.supplied_stacks,
    )


class PipelineIntegrationTests(unittest.TestCase):
    def test_single_pass_matches_manual_engine_labels_rng_vectors_and_grade_actions(self):
        scene = tracking_scene(length=3)
        actual_cloud = PointCloud(
            scene.cloud.xyz,
            scene.cloud.intensity,
            class_codes=scene.cloud.class_codes,
            segment_ids=scene.cloud.segment_ids,
        )
        actual = run_pipeline(
            actual_cloud,
            scene.seed,
            config=scene.config,
            options=PipelineOptions(bidirectional=False, grades=True, gwad=True),
        )
        random = MsvcRand()
        expected = TrackingEngine(GridIndex(scene.cloud), config=scene.config, rng=random)
        initialized = expected.initialize(scene.seed)
        tracking = expected.run_pass()
        grades = expected.compute_grades()
        gwad = build_gwad_rows(expected.layers, [f"layer-{i}" for i in range(len(expected.layers))])
        self.assertTrue(actual.completed)
        self.assertEqual(actual.initialization.location, initialized.location)
        self.assertEqual(actual.workflow.status, tracking.status)
        self.assertEqual(actual.engine.layers[0].center_vector, expected.layers[0].center_vector)
        self.assertEqual(actual.engine.layers[0].rail1_vector, expected.layers[0].rail1_vector)
        self.assertEqual(actual.grades[0].sections, grades[0].sections)
        self.assertEqual(actual.gwad.rows, gwad.rows)
        self.assertEqual(actual.random_metadata["final_state"], random.state)
        self.assertEqual(actual.random_metadata["draw_count"], random.draw_count)
        assert_array_equal(actual_cloud.class_codes, scene.cloud.class_codes)
        assert_array_equal(actual_cloud.segment_ids, scene.cloud.segment_ids)
        self.assertIs(actual.engine.grid.cloud, actual_cloud)

    def test_bidirectional_composition_matches_direct_engine_without_rng_reset(self):
        scene = pipeline_scene()
        actual = execute_scene("straight")
        random = MsvcRand()
        expected = TrackingEngine(GridIndex(scene.cloud), config=scene.config, rng=random)
        expected.initialize(scene.seed)
        workflow = expected.run_bidirectional()
        expected.compute_grades()
        export = build_gwad_rows(expected.layers, ["layer-0"])
        self.assertEqual(actual.workflow.reverse_location, workflow.reverse_location)
        self.assertEqual(actual.engine.layers[0].rail1_vector, expected.layers[0].rail1_vector)
        self.assertEqual(actual.engine.layers[0].rail2_vector, expected.layers[0].rail2_vector)
        self.assertEqual(actual.engine.layers[0].center_vector, expected.layers[0].center_vector)
        self.assertEqual(actual.gwad.rows, export.rows)
        self.assertEqual(actual.random_metadata["draw_count"], random.draw_count)
        self.assertEqual(actual.random_metadata["final_state"], random.state)
        assert_array_equal(actual.engine.grid.cloud.class_codes, scene.cloud.class_codes)

    def test_components_exercise_actual_postpasses_and_branch_with_disclosed_state(self):
        actual = execute_scene("components")
        self.assertTrue(actual.completed)
        main = actual.workflow.main.result
        self.assertEqual(main.recognition.entries[0].binary.type, 6)
        self.assertEqual(main.toe_detection.entries[0].status, "ok")
        self.assertEqual(len(main.toe_detection.new_toes), 1)
        self.assertEqual(len(actual.engine.switch_boxes), 2)
        branch = actual.workflow.branches[0]
        self.assertEqual(branch.seed.point.z, branch.toe.right.y)  # Source Y-to-Z bug retained.
        self.assertGreater(branch.tracking.result.accepted_steps, 0)
        self.assertFalse(branch.tracking.main_track)
        self.assertTrue(
            all(
                entry.status == "main_track_disabled"
                for entry in branch.tracking.result.recognition.entries
            )
        )
        self.assertEqual(actual.fixture_metadata["supplied_toes"], 1)
        self.assertEqual(actual.fixture_metadata["supplied_stacks"], 1)
        self.assertEqual(actual.fixture_metadata["initialized_reference_lines"], 12)
        self.assertTrue(actual.grades)
        self.assertIsNotNone(actual.gwad)
        self.assertIn("LIM-025", {f.limitation for f in actual.findings})

    def test_missing_references_are_not_repaired_by_pipeline(self):
        actual = execute_scene("missing-references")
        self.assertTrue(actual.completed)
        self.assertFalse(actual.reference_comparable)
        self.assertEqual(
            actual.workflow.main.result.toe_detection.entries[0].status,
            "undefined_source_reference_line",
        )
        self.assertEqual(actual.fixture_metadata["initialized_reference_lines"], 0)
        self.assertEqual(
            len(actual.engine.toe_points), 1
        )  # Only the explicitly supplied branch toe.
        self.assertIn("LIM-001", {f.limitation for f in actual.findings})

    def test_raw_turnout_stack_is_recognized_without_injecting_type_or_reference_lines(self):
        actual = execute_scene("turnout")
        self.assertEqual(actual.fixture_metadata["supplied_stacks"], 0)
        self.assertEqual(len(actual.engine.completed_stacks), 1)
        self.assertGreater(len(actual.engine.completed_stacks[0].track_segments), 4)
        self.assertTrue(
            all(
                segment.track1.member_line is None
                for segment in actual.engine.completed_stacks[0].track_segments
            )
        )
        self.assertEqual(actual.workflow.main.result.recognition.entries[0].status, "ambiguous")
        self.assertEqual(
            actual.workflow.main.result.toe_detection.entries[0].status, "score_below_threshold"
        )
        self.assertEqual(actual.engine.toe_points, [])

    def test_step_limit_defers_postpasses_grades_gwad_and_is_not_completed(self):
        result = execute_scene("straight", max_steps=0)
        self.assertEqual(result.status, "step_limit")
        self.assertFalse(result.completed)
        self.assertFalse(result.reference_comparable)
        self.assertIsNone(result.workflow.reverse_pass.main.result.recognition)
        self.assertIsNone(result.workflow.reversal)
        self.assertIsNone(result.workflow.forward_pass)
        self.assertIsNone(result.grades)
        self.assertIsNone(result.gwad)
        self.assertEqual(
            [(stage.name, stage.status) for stage in result.stages[-2:]],
            [("grades", "deferred"), ("gwad", "deferred")],
        )

    def test_grade_and_gwad_are_opt_in_and_export_alone_computes_grades(self):
        scene = tracking_scene(length=2)
        plain = run_pipeline(scene.cloud, scene.seed, options=PipelineOptions(bidirectional=False))
        self.assertIsNone(plain.grades)
        self.assertIsNone(plain.gwad)
        self.assertTrue(all(p.point_type == 0 for p in plain.engine.layers[0].center_vector))
        scene = tracking_scene(length=2)
        export = run_pipeline(
            scene.cloud, scene.seed, options=PipelineOptions(bidirectional=False, gwad=True)
        )
        self.assertIs(export.grades, export.gwad.analyses)
        self.assertTrue(any(p.point_type == 1 for p in export.engine.layers[0].center_vector))

    def test_empty_input_with_explicit_bounds_fails_initialization_cleanly(self):
        scene = tracking_scene(length=2)
        cloud = PointCloud(np.empty((0, 3)))
        result = run_pipeline(
            cloud,
            scene.seed,
            bounds=Bounds3D(Point3D(0, 0, 0), Point3D(1, 1, 1)),
            options=PipelineOptions(grades=True, gwad=True),
        )
        self.assertEqual(result.status, "initialization_failed")
        self.assertEqual(result.initialization.status, "no_head_points")
        self.assertEqual(result.random_metadata["draw_count"], 0)
        self.assertIsNone(result.workflow)
        self.assertIsNone(result.gwad)
        self.assertFalse(result.completed)

    def test_recorded_draws_replay_identical_geometry_and_labels(self):
        scene = tracking_scene(length=2)
        options = PipelineOptions(bidirectional=False, record_random=True)
        first = run_pipeline(scene.cloud, scene.seed, options=options)
        random = RecordedRand(first.random_metadata["recorded_draws"])
        scene = tracking_scene(length=2)
        second = run_pipeline(scene.cloud, scene.seed, options=options, rng=random)
        self.assertEqual(second.random_metadata["draw_count"], first.random_metadata["draw_count"])
        self.assertIsNone(second.random_metadata["seed"])
        self.assertEqual(
            first.engine.layers[0].center_vector, second.engine.layers[0].center_vector
        )
        assert_array_equal(
            first.engine.grid.cloud.class_codes, second.engine.grid.cloud.class_codes
        )
        assert_array_equal(
            first.engine.grid.cloud.segment_ids, second.engine.grid.cloud.segment_ids
        )

    def test_invalid_options_seed_and_foreign_memberships_fail_before_labels_change(self):
        with self.assertRaises(ValueError):
            PipelineOptions(max_steps=-1)
        with self.assertRaises(TypeError):
            PipelineOptions(max_steps=0.5)
        scene = pipeline_scene("components")
        before = scene.cloud.class_codes.copy()
        with self.assertRaises(ValueError):
            run_pipeline(scene.cloud, replace(scene.seed, angle=float("nan")))
        foreign = tracking_scene(length=2).cloud.select([])
        scene.supplied_stacks[0].track_segments[0].track1.members = foreign
        with self.assertRaisesRegex(ValueError, "pipeline cloud"):
            run_pipeline(scene.cloud, scene.seed, supplied_stacks=scene.supplied_stacks)
        assert_array_equal(before, scene.cloud.class_codes)

    def test_input_fingerprints_capture_initial_labels_and_fixture_state(self):
        scene = tracking_scene(length=2)
        expected = sha256(scene.cloud.class_codes.astype("<i4").tobytes()).hexdigest()
        result = run_pipeline(scene.cloud, scene.seed, options=PipelineOptions(bidirectional=False))
        self.assertEqual(result.input_metadata["sha256"]["initial_class_codes_i32_le"], expected)
        self.assertNotEqual(
            sha256(scene.cloud.class_codes.astype("<i4").tobytes()).hexdigest(), expected
        )
        self.assertIsNone(result.input_metadata["crs"])
        self.assertIsNone(result.input_metadata["coordinate_units"])
        self.assertEqual(result.status, "completed_with_limitations")
        self.assertFalse(result.reference_comparable)
        self.assertFalse(result.coverage["complete_reconstruction"])


class PipelineArtifactTests(unittest.TestCase):
    def test_outputs_roundtrip_geometry_tags_gwad_hashes_and_do_not_overwrite(self):
        result = execute_scene("components")
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "run"
            manifest = write_pipeline_outputs(
                result, destination, context={"scenario": "components"}
            )
            report = json.loads(manifest.read_text())
            self.assertEqual(report["status"], result.status)
            self.assertEqual(report["context"], {"scenario": "components"})
            self.assertEqual(report["supplied_fixtures"]["initialized_reference_lines"], 12)
            for artifact in report["artifacts"]:
                self.assertEqual(
                    sha256((destination / artifact["path"]).read_bytes()).hexdigest(),
                    artifact["sha256"],
                )
            with (destination / "vectors.csv").open(newline="") as stream:
                centers = [
                    row
                    for row in csv.DictReader(stream)
                    if row["layer"] == "0" and row["vector"] == "center"
                ]
            expected = result.engine.layers[0].center_vector
            assert_allclose(
                [[float(row[k]) for k in ("x", "y", "z")] for row in centers],
                [point.as_array() for point in expected],
                rtol=0,
                atol=0,
            )
            self.assertEqual(
                [int(row["point_type"]) for row in centers], [p.point_type for p in expected]
            )
            self.assertEqual(len(list(destination.glob("*"))), 6)
            prior = manifest.read_bytes()
            with self.assertRaises(FileExistsError):
                write_pipeline_outputs(result, destination)
            self.assertEqual(manifest.read_bytes(), prior)

    def test_strict_json_distinguishes_nonfinite_from_uninitialized_null(self):
        result = strict_json_value(
            {"missing": None, "a/b~c": [float("nan"), float("inf"), -float("inf")]}
        )
        self.assertEqual(result["a/b~c"], [None] * 3)
        self.assertEqual(
            result["nonfinite_values"],
            [
                {"path": "/a~1b~0c/0", "value": "NaN"},
                {"path": "/a~1b~0c/1", "value": "+Infinity"},
                {"path": "/a~1b~0c/2", "value": "-Infinity"},
            ],
        )
        self.assertEqual(json.loads(json.dumps(result, allow_nan=False)), result)

    def test_detailed_steps_include_ordered_point_ids_and_filter_states(self):
        scene = tracking_scene(length=2)
        result = run_pipeline(
            scene.cloud, scene.seed, options=PipelineOptions(bidirectional=False, max_steps=1)
        )
        compact = pipeline_summary(result)
        self.assertNotIn("steps", compact["invocations"][0])
        report = pipeline_summary(result, include_steps=True)
        step = report["invocations"][0]["steps"][0]
        self.assertTrue(step["accepted"])
        ids = step["attempts"][0]["segment"]["track1"]["members"]["point_ids"]
        self.assertEqual(
            ids,
            result.workflow.main.result.steps[0]
            .attempts[0]
            .segment.track1.members.indices.tolist(),
        )
        self.assertTrue(step["updates"])
        json.dumps(report, allow_nan=False)

    def test_cli_completed_limited_and_audit_exit_codes(self):
        for args, code in (
            (["pipeline-demo", "--max-steps", "0"], 2),
            (["pipeline-demo", "--single-direction", "--skip-grades"], 0),
            (["source-audit", "--repo", str(ROOT)], 0),
        ):
            with self.subTest(args=args):
                process = subprocess.run(
                    [sys.executable, "-m", "railway_mls", *args], capture_output=True, text=True
                )
                self.assertEqual(process.returncode, code, process.stderr)
                report = json.loads(process.stdout)
                self.assertNotIn("NaN", process.stdout)
                if args[0] == "pipeline-demo":
                    self.assertEqual(report["context"]["synthetic_scenario"]["name"], "straight")


class SourceCoverageTests(unittest.TestCase):
    def test_local_snapshot_and_python_targets_validate(self):
        self.assertEqual(check_source_coverage(ROOT), ())
        dispositions = {
            entry["disposition"]
            for file in source_coverage()["definition_files"]
            for entry in file["definitions"]
        }
        self.assertIn("inactive_pending", dispositions)
        self.assertIn("unused_helper_pending", dispositions)
        self.assertIn("io_or_conversion_pending", dispositions)

    def test_parser_ignores_comments_strings_and_preserves_overloads_and_destructor(self):
        source = """// void Owner::fake() {}
        const char* text = "void Owner::fake2() {}";
        /* void Owner::fake3() {} */
        void Owner::method(int first) {}
        double Owner::method(
            double second) {}
        Owner::~Owner() {}
        """
        entries = cpp_definitions(source, "Owner")
        self.assertEqual(
            [entry["id"] for entry in entries],
            ["Owner::method#1", "Owner::method#2", "Owner::~Owner#1"],
        )
        self.assertEqual(entries[1]["signature"], "double Owner::method( double second)")

    def test_hash_is_newline_stable_and_source_drift_detects_unclassified_definitions(self):
        self.assertEqual(
            source_text_hash("first\r\nsecond\r\n"), source_text_hash("first\nsecond\n")
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / SOURCE_DIRECTORY
            source.mkdir(parents=True)
            path = source / "RailwayExtraction.cpp"
            path.write_text(
                (ROOT / SOURCE_DIRECTORY / path.name).read_text()
                + "\nvoid RailwayExtraction::UnclassifiedNewAlgorithm() {}\n"
            )
            errors = check_source_coverage(root)
            self.assertIn("source changed: RailwayExtraction.cpp", errors)
            self.assertIn("definition inventory differs: RailwayExtraction.cpp", errors)
            self.assertTrue(any(error.startswith("missing source:") for error in errors))


class PickedSeedCppReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def test_received_points_computational_prefix_all_quadrants_axes_and_coincident(self):
        first = Point3D(1000, 2000, 3, 7)
        for dx, dy in (
            (2, 3),
            (-2, 3),
            (-2, -3),
            (2, -3),
            (0, 3),
            (0, -3),
            (3, 0),
            (-3, 0),
            (0, 0),
        ):
            with self.subTest(dx=dx, dy=dy):
                second = Point3D(first.x + dx, first.y + dy, 9, 11)
                actual = initial_location_from_picks(first, second)
                expected = self.reference.run("picked_seed", *first.as_array(), *second.as_array())
                assert_allclose(
                    [*actual.point.as_array(), actual.angle, actual.mode, actual.point.point_type],
                    expected,
                    rtol=0,
                    atol=0,
                )
                self.assertIsNone(actual.second_point)


if __name__ == "__main__":
    unittest.main()
