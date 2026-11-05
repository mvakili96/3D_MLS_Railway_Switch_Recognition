"""Interactive branch scheduling against the source-based blocking workflow.

The small branch fixture supplies a toe to isolate scheduling. Real automatic
Vancouver recognition/recovery is exercised separately in desktop acceptance.
"""

from dataclasses import replace
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_array_equal

from railway_mls import Bounds3D, GridIndex, MsvcRand, Point3D, TrackingEngine
from railway_mls.synthetic import branch_scene, tracking_scene
from railway_mls.viewer_model import LasInfo, TrackingCrop, ViewerSession


def prepared(scene=None, *, branches=True):
    scene = scene or branch_scene()
    info = LasInfo(Path("synthetic.las"), len(scene.cloud), Bounds3D.from_cloud(scene.cloud))
    viewer = ViewerSession(TrackingCrop(info, scene.cloud, scene.seed.point, 50),
                           config=scene.config, trace_branches=branches)
    viewer.preview(scene.seed)
    viewer.confirm(scene.seed)
    return scene, viewer


def reference(scene):
    engine = TrackingEngine(GridIndex(scene.cloud, scene.config.grid_size),
                            config=scene.config, rng=MsvcRand(1))
    engine.initialize(scene.seed)
    engine.initialize(scene.seed)
    return engine


def finish(viewer):
    for _ in range(100):
        frame = viewer.step()
        if frame.phase == "stopped":
            return frame
    raise AssertionError("viewer did not terminate")


def typed(points):
    return np.array([(*p.as_array(), p.point_type) for p in points]).reshape(-1, 4)


class ViewerBranchTests(unittest.TestCase):
    def assert_engines_equal(self, actual, expected):
        self.assertEqual(len(actual.layers), len(expected.layers))
        self.assertEqual(actual.current_layer_index, expected.current_layer_index)
        self.assertEqual(actual.rng.state, expected.rng.state)
        self.assertEqual(actual.rng.draw_count, expected.rng.draw_count)
        self.assertEqual(actual.distance_from_start, expected.distance_from_start)
        self.assertEqual(actual.initial_location, expected.initial_location)
        assert_array_equal(actual.grid.cloud.class_codes, expected.grid.cloud.class_codes)
        assert_array_equal(actual.grid.cloud.segment_ids, expected.grid.cloud.segment_ids)
        for first, second in zip(actual.layers, expected.layers):
            for name in ("rail1_vector", "rail2_vector", "center_vector"):
                assert_array_equal(typed(getattr(first, name)), typed(getattr(second, name)))

    def test_real_branch_steps_match_source_orchestration_labels_and_rng(self):
        scene, viewer = prepared()
        viewer.engine.toe_points.append(scene.toe)
        viewer.tracker.layer.toe_points.append(replace(scene.toe))
        other = branch_scene()
        engine = reference(other)
        engine.toe_points.append(other.toe)
        engine.current_layer.toe_points.append(replace(other.toe))
        expected = engine.run_pass()
        frame = finish(viewer)
        self.assertEqual([t.accepted_steps for t in frame.traces], [6, 5])
        self.assertEqual(frame.total_steps, 11)
        self.assertEqual([t.track_direction for t in viewer.trackers], [1, 0])
        self.assertEqual([t.main_track for t in viewer.trackers], [True, False])
        self.assertEqual(viewer.branch_attempts, [(b.toe_index, b.status) for b in expected.branches])
        self.assertEqual(viewer.tags, list(expected.tags))
        self.assert_engines_equal(viewer.engine, engine)
        self.assertTrue(viewer._toes[0].used)
        self.assertFalse(viewer.engine.toe_points[0].used)

    def test_pausing_at_main_stop_defers_branch_initialization(self):
        scene, viewer = prepared()
        viewer.engine.toe_points.append(scene.toe)
        for _ in range(20):
            frame = viewer.step()
            if frame.status == "branches_pending":
                break
        self.assertEqual(frame.status, "branches_pending")
        self.assertEqual(frame.phase, "tracking")
        self.assertEqual(len(viewer.engine.layers), 1)
        main = frame.traces[0].paths[2].copy()
        rng = (viewer.engine.rng.state, viewer.engine.rng.draw_count)
        viewer.frame()
        viewer.frame()
        self.assertEqual((viewer.engine.rng.state, viewer.engine.rng.draw_count), rng)
        seeded = viewer.step()
        self.assertEqual(seeded.status, "branch_initialized")
        self.assertEqual(seeded.active_path, "Branch 1")
        self.assertEqual(seeded.accepted_steps, 0)
        self.assertEqual(len(seeded.traces), 2)
        assert_array_equal(seeded.traces[0].paths[2], main)
        viewer.step()
        assert_array_equal(seeded.traces[0].paths[2], main)  # retained GUI snapshot

    def test_toe_queue_is_a_snapshot_and_global_used_flags_are_retained(self):
        scene, viewer = prepared()
        viewer.engine.toe_points.append(replace(scene.toe, used=True))
        for _ in range(20):
            frame = viewer.step()
            if frame.status == "branch_initialized":
                break
        self.assertEqual(frame.status, "branch_initialized")
        viewer.engine.toe_points.append(replace(scene.toe))
        frame = finish(viewer)
        self.assertEqual(len(frame.traces), 2)
        self.assertEqual(len(viewer.branch_attempts), 1)
        self.assertTrue(viewer._toes[0].used)
        self.assertEqual([t.used for t in viewer.engine.toe_points], [True, False])

    def test_type_gates_empty_layers_and_failed_initializations_match_source(self):
        for types in ((2, 6), (2, 0), (-1, 999)):
            with self.subTest(types=types):
                scene, viewer = prepared()
                other = branch_scene()
                engine = reference(other)
                for target, toe in ((viewer.engine, scene.toe), (engine, other.toe)):
                    for kind in types:
                        # -1 tests a defined empty initialization before a valid unknown type.
                        right = (toe.right + Point3D(100, 100, 0)) if kind == -1 else toe.right
                        target.toe_points.append(replace(toe, track_type=kind, right=right))
                expected = engine.run_pass()
                finish(viewer)
                self.assertEqual(viewer.branch_attempts,
                                 [(b.toe_index, b.status) for b in expected.branches])
                self.assertEqual([t.used for t in viewer._toes], [b.toe.used for b in expected.branches])
                self.assert_engines_equal(viewer.engine, engine)

    def test_missing_or_nonfinite_seed_preserves_source_failure_and_main_path(self):
        for kind in ("missing", "nonfinite"):
            with self.subTest(kind=kind):
                scene, viewer = prepared()
                other = branch_scene()
                engine = reference(other)
                for target, toe in ((viewer.engine, scene.toe), (engine, other.toe)):
                    target.toe_points.append(replace(toe, right=None) if kind == "missing"
                                             else replace(toe, angle=float("nan")))
                expected = engine.run_pass()
                frame = finish(viewer)
                self.assertEqual(frame.status, expected.status)
                self.assertEqual(len(frame.traces), 1)
                self.assertEqual(frame.traces[0].accepted_steps, 6)
                self.assertFalse(viewer._toes[0].used)
                self.assert_engines_equal(viewer.engine, engine)

    def test_recovery_copies_same_accepted_fit_without_consuming_random_draws(self):
        _, demo = prepared(tracking_scene(length=11, side_rails=True))
        _, strict = prepared(tracking_scene(length=11, side_rails=True), branches=False)
        actual, baseline = demo.step(), strict.step()
        self.assertEqual(actual.status, baseline.status)
        self.assertEqual(demo.engine.rng.draw_count, strict.engine.rng.draw_count)
        self.assertEqual(demo.engine.rng.state, strict.engine.rng.state)
        for first, second in zip(actual.paths, baseline.paths):
            assert_array_equal(first, second)
        saved = demo.tracker.active_stack.track_segments[-1]
        native = strict.tracker.active_stack.track_segments[-1]
        accepted = demo.tracker.steps[-1].candidate
        for window, fitted, uninitialized in zip((saved.track1, saved.track2),
                                                (accepted.track1, accepted.track2),
                                                (native.track1, native.track2)):
            self.assertIsNone(uninitialized.member_line)
            self.assertIsNot(window.member_line, fitted.member_line)
            self.assertEqual(window.member_line.begin, fitted.member_line.begin)
            self.assertEqual(window.member_line.end, fitted.member_line.end)
        self.assertIsNone(saved.distance_from_start)
        self.assertEqual(demo.recovered_references, 1)

    def test_open_active_stack_is_not_flushed_into_an_invented_switch(self):
        _, viewer = prepared(tracking_scene(length=11, side_rails=True))
        frame = finish(viewer)
        self.assertEqual(frame.total_steps, 11)
        self.assertEqual(len(viewer.tracker.active_stack.track_segments), 11)
        self.assertEqual(viewer.engine.completed_stacks, [])
        self.assertEqual(viewer.engine.toe_points, [])
        self.assertEqual(viewer.branch_attempts, [])
        self.assertIn("A switch region is incomplete at this boundary.", frame.notes)

    def test_rejected_or_nonfinite_segments_do_not_supply_switch_references(self):
        _, viewer = prepared(tracking_scene(length=11, side_rails=True))
        viewer.step()
        step = viewer.tracker.steps[-1]
        for status, accepted in (("gauge_difference", False), ("nonfinite_tracking_geometry", True)):
            with self.subTest(status=status):
                saved = viewer.tracker.active_stack.track_segments[-1]
                saved.track1.member_line = saved.track2.member_line = None
                viewer.recovered_references = 0
                viewer.tracker.steps[-1] = replace(step, accepted=accepted, status=status)
                viewer._save_switch_references()
                self.assertIsNone(saved.track1.member_line)
                self.assertIsNone(saved.track2.member_line)
                self.assertEqual(viewer.recovered_references, 0)

    def test_main_only_does_not_schedule_supplied_branches(self):
        scene, viewer = prepared(branches=False)
        viewer.engine.toe_points.append(scene.toe)
        frame = finish(viewer)
        self.assertEqual(len(frame.traces), 1)
        self.assertEqual(frame.total_steps, 6)
        self.assertEqual(len(viewer.engine.layers), 1)
        self.assertEqual(viewer.branch_attempts, [])
        self.assertEqual(viewer.recovered_references, 0)

    def test_reset_clears_branch_queue_paths_recovery_and_labels(self):
        scene, viewer = prepared()
        viewer.engine.toe_points.append(scene.toe)
        finish(viewer)
        viewer.reset()
        self.assertEqual(viewer.phase, "empty")
        self.assertEqual(viewer.trackers, [])
        self.assertEqual(viewer.branch_attempts, [])
        self.assertEqual(viewer.tags, [])
        self.assertEqual(viewer._toes, [])
        self.assertEqual(viewer.recovered_references, 0)
        self.assertEqual(viewer.engine.rng.draw_count, 0)
        self.assertTrue(np.all(viewer.crop.cloud.class_codes == 0))
        self.assertTrue(np.all(viewer.crop.cloud.segment_ids == -1))

    def test_cli_main_only_disables_demo_recovery(self):
        from railway_mls.cli import main

        with patch("railway_mls.viewer.launch_viewer") as launch:
            self.assertEqual(main(["view"]), 0)
            self.assertTrue(launch.call_args.kwargs["trace_branches"])
            self.assertEqual(main(["view", "--main-only"]), 0)
            self.assertFalse(launch.call_args.kwargs["trace_branches"])
