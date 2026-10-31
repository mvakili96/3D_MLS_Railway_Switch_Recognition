from dataclasses import replace
import json
import subprocess
import sys
import unittest
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls.config import ExtractionConfig
from railway_mls.gauge import GaugeAdjustment
from railway_mls.geometry import Line3D
from railway_mls.initialization import initialize_tracks
from railway_mls.models import TrackLayer
from railway_mls.mdl import segment_track_points_mdl
from railway_mls.points import Point3D
from railway_mls.sampling import MsvcRand, RecordedRand
from railway_mls.spatial import GridIndex
from railway_mls.synthetic import tracking_scene
from railway_mls.tracking import SequentialTracker, track_railway
from tests.cpp_reference import CppReference


def prepared(*, config=None, side_rails=False, length=8, mode=1, gaps=()):
    scene = tracking_scene(
        config=config, side_rails=side_rails, length=length, mode=mode, gaps=gaps
    )
    grid = GridIndex(scene.cloud, scene.config.grid_size)
    rng = MsvcRand(1)
    initialized = initialize_tracks(scene.seed, grid, config=scene.config, rng=rng)
    if not initialized.success:
        raise AssertionError(initialized.status)
    return scene, grid, rng, initialized


def ids(points):
    return [0] if points is None else [len(points), *points.indices]


def state_line(line):
    return [
        *line.begin.as_array(),
        *line.end.as_array(),
        *line.center.as_array(),
        line.angle,
        line.a,
        line.b,
        line.c,
        *ids(line.members),
    ]


def segment_state(segment, fitted=True):
    result = [*segment.center.as_array(), segment.gauge]
    if fitted:
        result.append(segment.distance_from_start)
    for window in (segment.track1, segment.track2):
        result.extend(
            [
                *window.center.as_array(),
                window.orientation,
                *(value for point in window.vertices for value in point.as_array()),
                *ids(window.members),
                *ids(window.left_members),
                *ids(window.right_members),
                int(window.turnout),
                window.left_tag,
                window.right_tag,
            ]
        )
        if fitted:
            result.extend(state_line(window.member_line))
    return result


class TrackingTests(unittest.TestCase):
    def test_gap_is_crossed_by_expanding_prediction_window(self):
        _, grid, rng, initialized = prepared(gaps=((0.4, 2.2),))
        result = track_railway(initialized, grid, rng=rng, max_steps=2)
        self.assertEqual(result.accepted_steps, 2)
        self.assertEqual([attempt.length for attempt in result.steps[0].attempts], [1, 2])

    def test_curved_graded_scene_updates_orientation_and_elevation(self):
        scene = tracking_scene(curvature=0.01, grade=0.005)
        grid, rng = GridIndex(scene.cloud), MsvcRand()
        initialized = initialize_tracks(scene.seed, grid, rng=rng)
        initial_angle = initialized.segment.track1.orientation
        initial_z = initialized.segment.track1.center.z
        result = track_railway(initialized, grid, rng=rng, max_steps=6)
        self.assertEqual(result.accepted_steps, 6)
        self.assertGreater(result.layer.track_segments[-1].track1.orientation - initial_angle, 2)
        self.assertGreater(result.layer.track_segments[-1].track1.center.z - initial_z, 0.02)

    def test_short_active_stack_is_discarded_on_single_model_transition(self):
        _, grid, rng, initialized = prepared()
        tracker = SequentialTracker(initialized.layer, grid, initialized.initial_heights, rng=rng)
        tracker.active_stack.track_segments.extend([initialized.segment, initialized.segment])
        result = tracker.run(max_steps=1)
        self.assertEqual(len(result.active_stack.track_segments), 0)
        self.assertEqual(len(result.completed_stacks), 0)

    def test_multi_model_in_one_window_marks_both_windows_and_clears_members(self):
        _, grid, rng, initialized = prepared(side_rails=True)
        calls = 0

        def mixed_models(*args, **kwargs):
            nonlocal calls
            model = segment_track_points_mdl(*args, **kwargs)
            calls += 1
            return replace(model, lines=model.lines[:1], selected_count=1) if calls == 1 else model

        with patch("railway_mls.tracking.segment_track_points_mdl", side_effect=mixed_models):
            result = track_railway(initialized, grid, rng=rng, max_steps=1)
        self.assertEqual([len(model.lines) for model in result.steps[0].models], [1, 3])
        for window in (
            result.layer.track_segments[-1].track1,
            result.layer.track_segments[-1].track2,
        ):
            self.assertTrue(window.turnout)
            self.assertEqual(len(window.members), 0)

    def test_nonfinite_gauge_keeps_source_appends_then_stops_before_unsafe_grid_query(self):
        _, grid, rng, initialized = prepared()

        def nan_gauge(first, second, *_args, **_kwargs):
            for line in (first, second):
                line.end = Point3D(float("nan"), float("nan"), float("nan"))
                line.center = line.end
            return GaugeAdjustment(
                (first, second),
                first.angle,
                1.5,
                float("nan"),
                float("nan"),
                (),
                (),
                "nonfinite_weighted_distance",
            )

        with patch("railway_mls.tracking.adjust_two_lines_with_observation", side_effect=nan_gauge):
            result = track_railway(initialized, grid, rng=rng)
        self.assertEqual(result.status, "nonfinite_tracking_geometry")
        self.assertEqual(result.accepted_steps, 1)
        self.assertEqual(len(result.layer.track_segments), 2)
        self.assertEqual(len(result.layer.rail1_vector), 2)
        self.assertTrue(np.isnan(result.distance_from_start))
        self.assertFalse(result.reference_comparable)

    def test_raw_scene_tracks_to_natural_point_stop(self):
        scene, grid, rng, initialized = prepared(length=7)
        result = track_railway(initialized, grid, rng=rng)
        self.assertEqual(result.status, "insufficient_window_points")
        self.assertEqual(result.accepted_steps, 7)
        self.assertEqual(len(result.layer.track_segments), 8)
        self.assertEqual(len(result.layer.rail1_vector), 8)
        self.assertEqual(len(result.layer.updated_boundaries), 64)
        self.assertEqual(
            [attempt.length for attempt in result.steps[-1].attempts], [1, 2, 3, 4, 5, 6]
        )
        self.assertFalse(result.reference_comparable)  # Missing native MDL candidates are exposed.
        self.assertTrue(
            all(
                model.status == "source_uninitialized_intermediates"
                for step in result.steps
                for model in step.models
            )
        )
        self.assertGreater(np.count_nonzero(scene.cloud.class_codes == 3), 0)
        self.assertTrue(
            all(abs(segment.gauge - 1.5) < 0.01 for segment in result.layer.track_segments)
        )

    def test_all_four_geometric_travel_modes(self):
        for mode in (1, 2, 3, 4):
            with self.subTest(mode=mode):
                scene, grid, rng, initialized = prepared(mode=mode)
                result = track_railway(initialized, grid, rng=rng, max_steps=2)
                self.assertEqual(result.accepted_steps, 2)
                self.assertEqual(result.steps[0].mode, mode)

    def test_point_retry_equality_fails_strict_retry_threshold(self):
        config = ExtractionConfig(min_points=720)
        _, grid, rng, initialized = prepared(config=config)
        result = track_railway(initialized, grid, config=config, rng=rng, max_steps=1)
        attempts = result.steps[0].attempts
        self.assertEqual([attempt.length for attempt in attempts], [1, 2, 3])
        self.assertEqual(
            [len(attempt.segment.track1.members) for attempt in attempts], [360, 720, 1080]
        )
        self.assertEqual(result.accepted_steps, 1)
        # Retries enlarge the search window; the saved window returns to base length.
        vertices = result.layer.track_segments[-1].track1.vertices
        self.assertAlmostEqual(np.linalg.norm(vertices[1].as_array() - vertices[0].as_array()), 1.0)

    def test_base_window_equality_does_not_trigger_retry(self):
        config = ExtractionConfig(min_points=360)
        _, grid, rng, initialized = prepared(config=config)
        result = track_railway(initialized, grid, config=config, rng=rng, max_steps=1)
        self.assertEqual(len(result.steps[0].attempts), 1)
        self.assertEqual(result.accepted_steps, 1)

    def test_head_retry_accumulates_duplicates_and_equality_requires_another_attempt(self):
        config = ExtractionConfig(min_track_points=360)
        _, grid, rng, initialized = prepared(config=config)
        result = track_railway(initialized, grid, config=config, rng=rng, max_steps=1)
        attempts = result.steps[0].attempts
        self.assertEqual(
            [attempt.phase for attempt in attempts], ["initial", "head_retry", "head_retry"]
        )
        self.assertEqual(
            [len(attempt.extraction.heads[0]) for attempt in attempts], [120, 360, 720]
        )
        final = attempts[-1].extraction.heads[0]
        self.assertLess(len(np.unique(final.indices)), len(final))

    def test_head_retries_restart_at_twice_base_after_point_expansion(self):
        config = ExtractionConfig(min_points=720, min_track_points=800)
        _, grid, rng, initialized = prepared(config=config)
        result = track_railway(initialized, grid, config=config, rng=rng, max_steps=1)
        attempts = result.steps[0].attempts
        self.assertEqual([attempt.length for attempt in attempts[:4]], [1, 2, 3, 2])
        self.assertEqual(attempts[3].phase, "head_retry")

    def test_exhausted_head_retries_stop_and_keep_label_changes(self):
        config = ExtractionConfig(min_track_points=100000)
        scene, grid, rng, initialized = prepared(config=config)
        result = track_railway(initialized, grid, config=config, rng=rng)
        self.assertEqual(result.status, "insufficient_head_points")
        self.assertEqual(
            [attempt.length for attempt in result.steps[0].attempts], [1, 2, 3, 4, 5, 6]
        )
        self.assertEqual(len(result.layer.track_segments), 1)
        self.assertGreater(np.count_nonzero(scene.cloud.class_codes == 3), 0)

    def test_covariance_is_identity_each_iteration_and_heights_use_pre_gauge_posterior(self):
        _, grid, rng, initialized = prepared()
        result = track_railway(initialized, grid, rng=rng, max_steps=3)
        for step in result.steps:
            for state in step.current_states:
                assert_array_equal(state.covariance, np.eye(3))
        first_update = result.steps[0].updates[0]
        self.assertNotEqual(first_update.state.covariance[0, 0], 1)
        self.assertEqual(
            result.next_heights, tuple(update.state.point.z for update in result.steps[-1].updates)
        )
        next_band = result.steps[1].attempts[0].extraction.windows[0].band_points
        previous_height = first_update.state.point.z
        self.assertTrue(np.all(next_band.xyz[:, 2] <= previous_height + 0.1))

    def test_cached_angle_and_center_are_not_recomputed_after_replacing_line_begin(self):
        _, grid, rng, initialized = prepared()
        result = track_railway(initialized, grid, rng=rng, max_steps=1)
        line = result.layer.track_segments[-1].track1.member_line
        self.assertEqual(line.angle, result.steps[0].adjustment.common_angle)
        self.assertNotEqual(line.center, (line.begin + line.end) / 2)
        self.assertNotEqual(line.angle, Line3D(line.begin, line.end).angle)

    def test_step_limit_resume_preserves_random_stream_and_source_state(self):
        _, grid, rng, initialized = prepared()
        tracker = SequentialTracker(initialized.layer, grid, initialized.initial_heights, rng=rng)
        self.assertEqual(tracker.run(max_steps=0).accepted_steps, 0)
        self.assertEqual(tracker.run(max_steps=1).accepted_steps, 1)
        resumed = tracker.run(max_steps=2)
        _, grid2, rng2, initialized2 = prepared()
        full = track_railway(initialized2, grid2, rng=rng2, max_steps=3)
        self.assertEqual(resumed.accepted_steps, full.accepted_steps)
        self.assertEqual(rng.draw_count, rng2.draw_count)
        assert_allclose(
            [point.as_array() for point in resumed.layer.rail1_vector],
            [point.as_array() for point in full.layer.rail1_vector],
            rtol=0,
            atol=0,
        )

    def test_turnout_stack_contains_predicted_copies_and_is_not_flushed_at_limit(self):
        _, grid, rng, initialized = prepared(side_rails=True)
        result = track_railway(initialized, grid, rng=rng, max_steps=3)
        self.assertTrue(result.reference_comparable)
        self.assertEqual(len(result.active_stack.track_segments), 3)
        self.assertEqual(len(result.completed_stacks), 0)
        self.assertEqual(len(result.layer.turnout_boundaries), 24)
        stack_first, saved_first = (
            result.active_stack.track_segments[0],
            result.layer.track_segments[1],
        )
        self.assertIsNone(stack_first.distance_from_start)
        self.assertIsNone(stack_first.track1.member_line)
        self.assertNotEqual(stack_first.track1.center, saved_first.track1.center)
        self.assertEqual(len(stack_first.track1.members), 0)
        self.assertTrue(saved_first.track1.turnout and saved_first.track2.turnout)

    def test_three_turnout_steps_flush_when_single_model_region_follows(self):
        _, grid, rng, initialized = prepared(side_rails=True)
        result = track_railway(initialized, grid, rng=rng, max_steps=4)
        self.assertEqual([len(step.models[0].lines) for step in result.steps], [3, 3, 3, 1])
        self.assertEqual(len(result.completed_stacks), 1)
        self.assertEqual(len(result.completed_stacks[0].track_segments), 3)
        self.assertEqual(len(result.active_stack.track_segments), 0)

    def test_undefined_head_fit_stops_without_fabricated_segment(self):
        _, grid, _, initialized = prepared()
        result = track_railway(initialized, grid, rng=RecordedRand([0] * 400))
        self.assertEqual(result.status, "undefined_source_head_extraction")
        self.assertEqual(len(result.layer.track_segments), 1)

    def test_no_segments_and_invalid_step_limit(self):
        scene, grid, _, _ = prepared()
        tracker = SequentialTracker(TrackLayer(), grid, (0.2, 0.2))
        self.assertEqual(tracker.run().status, "no_segments")
        with self.assertRaises(ValueError):
            tracker.run(max_steps=-1)
        with self.assertRaises(TypeError):
            tracker.run(max_steps=1.2)
        self.assertGreater(len(scene.cloud), 0)

    def test_gauge_stop_keeps_rail_vectors_distance_and_unsaved_candidate(self):
        _, grid, rng, initialized = prepared()

        def bad_gauge(first, second, *_args, **_kwargs):
            first.end = first.begin + Point3D(1, 0, 0)
            second.end = second.begin + Point3D(2, 0, 0)
            return GaugeAdjustment((first, second), 0, 1.5, 1, 1.5, (), (), "ok")

        with patch("railway_mls.tracking.adjust_two_lines_with_observation", side_effect=bad_gauge):
            result = track_railway(initialized, grid, rng=rng, max_steps=1)
        self.assertEqual(result.status, "gauge_difference")
        self.assertEqual(result.accepted_steps, 0)
        self.assertEqual(len(result.layer.track_segments), 1)
        self.assertEqual(len(result.layer.rail1_vector), 2)
        self.assertGreater(result.distance_from_start, 0)
        self.assertIsNotNone(result.steps[0].candidate)


class TrackingCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def compare_loop(self, *, config=None, steps=3, direction=0, main_track=True, curved=False):
        if curved:
            scene = tracking_scene(side_rails=True, curvature=0.01, grade=0.005, config=config)
            grid, rng = GridIndex(scene.cloud), MsvcRand(1)
            initialized = initialize_tracks(scene.seed, grid, rng=rng, config=scene.config)
        else:
            scene, grid, rng, initialized = prepared(config=config, side_rails=True)
        config, segment = scene.config, initialized.segment
        inputs = [
            config.grid_size,
            config.track_gauge,
            config.track_window_length,
            config.track_window_width,
            config.max_distance_for_ransac,
            config.min_points,
            config.min_track_points,
            config.skip_rate,
            config.mdl_weight,
            config.grouping_distance,
            direction,
            int(main_track),
            7,
            steps,
            *initialized.initial_heights,
            *grid.bounds.minimum.as_array(),
            *grid.bounds.maximum.as_array(),
            len(scene.cloud),
        ]
        for i, xyz in enumerate(scene.cloud.xyz):
            inputs.extend([*xyz, scene.cloud.class_codes[i], scene.cloud.segment_ids[i]])
        inputs.append(segment.distance_from_start)
        for window in (segment.track1, segment.track2):
            inputs.extend(
                [
                    *window.center.as_array(),
                    window.orientation,
                    *(value for point in window.vertices for value in point.as_array()),
                    *ids(window.members),
                    *state_line(window.member_line),
                ]
            )
        draws = MsvcRand()
        draws.state = rng.state
        raw = [draws.rand() for _ in range(150000)]
        inputs.extend([len(raw), *raw])
        previous_draws = rng.draw_count
        tracker = SequentialTracker(
            initialized.layer,
            grid,
            initialized.initial_heights,
            config=config,
            rng=rng,
            track_direction=direction,
            main_track=main_track,
            distance_from_start=7,
        )
        result = tracker.run(max_steps=steps)
        # Never execute uninitialized MDL intermediates as a native oracle.
        self.assertTrue(result.reference_comparable, result.status)
        native = self.reference.run("track", *inputs)
        stop_codes = {
            "insufficient_window_points": 1,
            "insufficient_head_points": 2,
            "gauge_difference": 3,
            "trajectory_end": 4,
            "step_limit": 5,
        }
        expected = [
            stop_codes[result.status],
            result.distance_from_start,
            len(result.layer.track_segments),
        ]
        for item in result.layer.track_segments:
            expected.extend(segment_state(item))
        for points in (
            result.layer.rail1_vector,
            result.layer.rail2_vector,
            result.layer.center_vector,
        ):
            expected.extend(
                [len(points), *(value for point in points for value in point.as_array())]
            )
        for edges in (result.layer.updated_boundaries, result.layer.turnout_boundaries):
            expected.append(len(edges))
            for edge in edges:
                expected.extend([*edge.begin.as_array(), *edge.end.as_array()])
        expected.append(len(result.completed_stacks))
        for stack in (*result.completed_stacks, result.active_stack):
            expected.append(len(stack.track_segments))
            for item in stack.track_segments:
                expected.extend(segment_state(item, fitted=False))
        attempts = [attempt for step in result.steps for attempt in step.attempts]
        expected.append(len(attempts))
        for attempt in attempts:
            expected.extend(
                [
                    attempt.length,
                    len(attempt.segment.track1.members),
                    len(attempt.segment.track2.members),
                ]
            )
        for code, segment_id in zip(scene.cloud.class_codes, scene.cloud.segment_ids):
            expected.extend([code, segment_id])
        expected.append(rng.draw_count - previous_draws)
        assert_allclose(expected, native, rtol=1e-10, atol=1e-9, equal_nan=True)
        return result

    def test_three_iterations_with_turnout_selection_match_original_loop(self):
        result = self.compare_loop()
        self.assertEqual(result.accepted_steps, 3)
        self.assertEqual(len(result.active_stack.track_segments), 3)

    def test_curved_and_graded_inference_matches_original_loop(self):
        result = self.compare_loop(curved=True, steps=2)
        self.assertEqual(result.accepted_steps, 2)

    def test_reverse_observation_direction_matches_original_loop(self):
        self.compare_loop(direction=1)

    def test_single_model_path_and_branch_distance_reset_match_original(self):
        result = self.compare_loop(config=ExtractionConfig(mdl_weight=0.01), main_track=False)
        self.assertTrue(
            all(len(model.lines) == 1 for step in result.steps for model in step.models)
        )
        self.assertLess(result.distance_from_start, 7)

    def test_point_retry_sequence_and_final_base_window_match_original(self):
        self.compare_loop(config=ExtractionConfig(min_points=2580), steps=1)

    def test_head_retry_accumulation_matches_original_loop(self):
        self.compare_loop(config=ExtractionConfig(min_track_points=1290), steps=1)

    def test_trajectory_stop_preserves_source_partial_side_effects(self):
        result = self.compare_loop(
            config=ExtractionConfig(track_window_length=0.2, skip_rate=10), steps=1
        )
        self.assertEqual(result.status, "trajectory_end")
        self.assertEqual(len(result.layer.rail1_vector), 2)
        self.assertEqual(len(result.layer.track_segments), 1)


class TrackingDemoTests(unittest.TestCase):
    def test_tracking_demo_runs_raw_cloud_through_sequential_stage(self):
        completed = subprocess.run(
            [sys.executable, "-m", "railway_mls", "tracking-demo"],
            text=True,
            capture_output=True,
            check=True,
        )
        result = json.loads(completed.stdout)
        self.assertEqual(result["status"], "insufficient_window_points")
        self.assertGreaterEqual(result["accepted_steps"], 5)
        self.assertTrue(result["tracking_implemented"])
        self.assertTrue(result["recognition_implemented"])
        self.assertTrue(result["recognition_postpass_ran"])
        self.assertFalse(result["toe_detection_implemented"])


if __name__ == "__main__":
    unittest.main()
