from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from dataclasses import replace
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls import Bounds3D, Point3D
from railway_mls.synthetic import tracking_scene
from railway_mls.viewer_model import (
    LasInfo, TrackingCrop, ViewerDataError, ViewerSession, load_preview, load_tracking_crop,
    search_boundary,
)


def session():
    scene = tracking_scene()
    info = LasInfo(Path("synthetic.las"), len(scene.cloud), Bounds3D.from_cloud(scene.cloud))
    return ViewerSession(TrackingCrop(info, scene.cloud, scene.seed.point, 20)), scene.seed


class ViewerSessionTests(unittest.TestCase):
    def test_restart_replays_original_preview_and_preserves_confirmed_edits(self):
        original, seed = session()
        original.preview(seed)
        edited = replace(seed, point=Point3D(seed.point.x + .05, seed.point.y, seed.point.z))
        original.confirm(edited)
        expected = original.step()
        replay, _ = session()
        replay.preview(original.preview_seed)
        replay.confirm(original.confirmed_seed)
        actual = replay.step()
        self.assertEqual(replay.preview_seed, seed)
        self.assertEqual(replay.confirmed_seed, edited)
        self.assertEqual(original.engine.rng.draw_count, replay.engine.rng.draw_count)
        for first, second in zip(expected.paths, actual.paths):
            assert_array_equal(first, second)

    def test_boundary_detection_distinguishes_crop_from_file_coverage(self):
        viewer, _ = session()
        info = replace(viewer.crop.info, bounds=Bounds3D(Point3D(0, 0, 0), Point3D(100, 100, 10)))
        crop = replace(viewer.crop, info=info, center=Point3D(50, 50, 1), radius=8)
        for vertices, radius, expected in (
            ([[58.1, 50, 1]], 8, "crop"), ([[100.1, 50, 1]], 80, "file"),
            ([[50, 50, 1]], 8, None), ([[float("nan"), 50, 1]], 8, None),
        ):
            with self.subTest(expected=expected, radius=radius):
                self.assertEqual(search_boundary(replace(crop, radius=radius), np.array(vertices)), expected)
        edge = replace(crop, center=Point3D(8, 50, 1))
        self.assertEqual(search_boundary(edge, np.array([[-.1, 50, 1]])), "file")

    def test_algorithm_stop_is_not_reported_as_a_crop_stop(self):
        viewer, seed = session()
        viewer.preview(seed)
        viewer.confirm(seed)
        viewer.step()
        viewer.phase = "stopped"
        with patch("railway_mls.viewer_model.search_boundary") as check:
            frame = viewer.frame("gauge_difference")
        self.assertIsNone(frame.boundary)
        self.assertEqual(frame.status, "gauge_difference")
        check.assert_not_called()

    def test_cli_uses_larger_defaults_and_accepts_full_tile_radius(self):
        from railway_mls.cli import main

        with patch("railway_mls.viewer.launch_viewer") as launch:
            self.assertEqual(main(["view"]), 0)
            self.assertEqual(launch.call_args.kwargs["radius"], 30)
            self.assertEqual(launch.call_args.kwargs["max_tracking_points"], 30_000_000)
            self.assertEqual(main(["view", "--radius", "30"]), 0)
            self.assertEqual(launch.call_args.kwargs["radius"], 30)

    def test_preview_and_confirmation_retain_native_rng_call_order(self):
        viewer, seed = session()
        preview = viewer.preview(seed)
        self.assertEqual((preview.phase, preview.status), ("preview", "ok"))
        self.assertEqual(viewer.engine.rng.draw_count, 400)
        confirmed = viewer.confirm(seed)
        self.assertEqual(confirmed.phase, "ready")
        self.assertEqual(viewer.engine.rng.draw_count, 800)
        self.assertEqual(len(confirmed.windows), 2)
        self.assertEqual([w.shape for w in confirmed.windows], [(4, 3), (4, 3)])

    def test_step_controls_preserve_the_continuous_tracker_and_draw_stream(self):
        one, seed = session()
        two, _ = session()
        for v in (one, two):
            v.preview(seed)
            v.confirm(seed)
        first = one.step()
        original = first.paths[0].copy()
        second = one.step()
        uninterrupted = two.tracker.run(max_steps=2)
        self.assertEqual(second.accepted_steps, uninterrupted.accepted_steps)
        self.assertEqual(one.engine.rng.draw_count, two.engine.rng.draw_count)
        assert_allclose(second.paths[0], [p.as_array() for p in uninterrupted.layer.rail1_vector])
        assert_array_equal(first.paths[0], original)  # UI frame owns its small geometry snapshot.

    def test_reset_clears_run_labels_rng_and_tracking_state(self):
        viewer, seed = session()
        viewer.preview(seed)
        viewer.confirm(seed)
        viewer.step()
        viewer.crop.cloud.class_codes[0] = 99
        viewer.crop.cloud.segment_ids[0] = 99
        viewer.reset()
        self.assertIsNone(viewer.tracker)
        self.assertEqual(viewer.engine.rng.draw_count, 0)
        self.assertTrue(np.all(viewer.crop.cloud.class_codes == 0))
        self.assertTrue(np.all(viewer.crop.cloud.segment_ids == -1))
        self.assertEqual(viewer.preview(seed).phase, "preview")

    def test_invalid_controller_transitions_and_outside_confirm_fail_explicitly(self):
        viewer, seed = session()
        with self.assertRaises(ViewerDataError):
            viewer.step()
        with self.assertRaises(ViewerDataError):
            viewer.confirm(seed)
        viewer.preview(seed)
        with self.assertRaises(ViewerDataError):
            viewer.preview(seed)
        with self.assertRaisesRegex(ViewerDataError, "outside"):
            viewer.confirm(replace(seed, point=Point3D(seed.point.x + 100, seed.point.y, seed.point.z)))

    def test_natural_stop_keeps_status_and_postpass_frame_available(self):
        viewer, seed = session()
        viewer.preview(seed)
        viewer.confirm(seed)
        for _ in range(20):
            frame = viewer.step()
            if frame.phase == "stopped":
                break
        self.assertEqual(frame.phase, "stopped")
        self.assertIn(frame.status, ("insufficient_window_points", "insufficient_head_points",
                                     "gauge_difference", "trajectory_end", "no_support_points"))
        self.assertIsNotNone(viewer.tracker.toe_detection)

    def test_cli_rejects_invalid_limits_without_importing_gui(self):
        for options in (("--radius", "nan"), ("--radius", "-1"), ("--preview-points", "0")):
            p = subprocess.run([sys.executable, "-m", "railway_mls", "view", *options],
                               capture_output=True, text=True)
            self.assertEqual(p.returncode, 2)
            self.assertIn("error:", p.stderr)

    def test_model_does_not_import_optional_gui_runtime(self):
        p = subprocess.run([sys.executable, "-c",
                            "import sys; import railway_mls.viewer_model; "
                            "assert 'open3d' not in sys.modules"], capture_output=True, text=True)
        self.assertEqual(p.returncode, 0, p.stderr)


class ViewerLasTests(unittest.TestCase):
    def setUp(self):
        try:
            import laspy
        except ImportError:
            self.skipTest("optional LAS support not installed")
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "scene.las"
        header = laspy.LasHeader(point_format=1, version="1.2")
        header.scales = [.001] * 3
        header.offsets = [500000, 5400000, 0]
        data = laspy.LasData(header)
        data.x = 500000 + np.arange(15) * .1
        data.y = 5400000 + np.arange(15) * .001
        data.z = np.arange(15) * .01
        data.intensity = np.arange(15, dtype=np.uint16) + 100
        data.write(self.path)

    def test_preview_cap_and_record_order_are_independent_of_odd_chunk_sizes(self):
        first = load_preview(self.path, max_points=3, chunk_size=3)
        second = load_preview(self.path, max_points=3, chunk_size=8)
        self.assertEqual(first.stride, 6)
        self.assertEqual(len(first.xyz), 3)
        assert_array_equal(first.xyz, second.xyz)
        assert_allclose(first.xyz[:, 0], [500000, 500000.6, 500001.2])
        self.assertFalse(first.xyz.flags.writeable)

    def test_crop_keeps_global_even_ids_world_xyz_intensity_and_header_origin(self):
        crop = load_tracking_crop(self.path, Point3D(500000.7, 5400000, .07),
                                  radius=.41, chunk_size=3)
        assert_array_equal(crop.cloud.intensity, [104, 106, 108, 110])
        assert_allclose(crop.cloud.xyz[:, 0], [500000.4, 500000.6, 500000.8, 500001])
        self.assertEqual(crop.info.bounds.minimum.x, 500000)
        self.assertEqual(crop.info.bounds.maximum.x, 500001.4)
        self.assertTrue(np.all(crop.cloud.class_codes == 0))

    def test_budget_guard_never_silently_downsamples_tracking(self):
        with self.assertRaisesRegex(ViewerDataError, "exceeds"):
            load_tracking_crop(self.path, Point3D(500000.7, 5400000, .07),
                               radius=2, max_points=2, chunk_size=3)

    def test_header_roundoff_omissions_preserve_origin_and_remaining_record_values(self):
        info = load_preview(self.path).info
        low = info.bounds.minimum
        low = Point3D(low.x, np.nextafter(low.y, np.inf), low.z)
        info = replace(info, bounds=Bounds3D(low, info.bounds.maximum))
        with patch("railway_mls.viewer_model._info", return_value=info):
            crop = load_tracking_crop(self.path, Point3D(500000.7, 5400000.007, .07), radius=2)
        self.assertEqual(crop.omitted_header_roundoff, 1)
        self.assertEqual(crop.info.bounds.minimum.y, low.y)
        assert_array_equal(crop.cloud.intensity, np.arange(102, 115, 2))
        assert_allclose(crop.cloud.xyz[:, 0], 500000 + np.arange(2, 15, 2) * .1)
        from railway_mls import GridIndex
        GridIndex(crop.cloud, bounds=crop.info.bounds)

    def test_real_header_bounds_mismatch_is_rejected_instead_of_silently_filtered(self):
        info = load_preview(self.path).info
        low = info.bounds.minimum
        info = replace(info, bounds=Bounds3D(Point3D(low.x, low.y+.0001, low.z),
                                            info.bounds.maximum))
        with patch("railway_mls.viewer_model._info", return_value=info):
            with self.assertRaisesRegex(ViewerDataError, "declared file bounds"):
                load_tracking_crop(self.path, Point3D(500000.7, 5400000.007, .07), radius=2)

    def test_cancellation_and_empty_crop_are_explicit(self):
        with self.assertRaisesRegex(ViewerDataError, "cancelled"):
            load_preview(self.path, cancel=lambda: True)
        with self.assertRaisesRegex(ViewerDataError, "outside"):
            load_tracking_crop(self.path, Point3D(0, 0, 0))
        with self.assertRaisesRegex(ViewerDataError, "no source-selected"):
            load_tracking_crop(self.path, Point3D(500000.1, 5400000, .01), radius=.001)
