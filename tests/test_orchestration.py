from dataclasses import replace
import json
import subprocess
import sys
import unittest
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls import (
    ExtractionConfig,
    GridIndex,
    InitialLocation,
    InitializationResult,
    Point3D,
    PointCloud,
    RecordedRand,
    ToePoint,
    TrackLayer,
    TrackStack,
    TrackingEngine,
    TrackingInvocation,
    reverse_layer_vectors,
    reverse_seed,
    seed_windows,
    tag_rail_vector,
)
from railway_mls.synthetic import bidirectional_scene, branch_scene, tracking_scene, toe_scene
from railway_mls.toes import detect_toe_points
from tests.cpp_reference import CppReference
from tests.test_toes import toe_state


def typed_vector(points):
    return [len(points), *(v for p in points for v in (*p.as_array(), p.point_type))]


def layer_fixture():
    return TrackLayer(
        rail1_vector=[Point3D(i, i, i + 1) for i in range(3)],
        rail2_vector=[Point3D(i + 2, i + 2, i + 3) for i in range(3)],
        center_vector=[Point3D(i + 1, i + 1, i + 2) for i in range(3)],
    )


def toe_fixture(type_id=0, *, used=False):
    return ToePoint(
        center=Point3D(2, 3, 9),
        left=Point3D(3, 4, 5),
        right=Point3D(11, 13, 17),
        track_type=type_id,
        used=used,
        angle=35.123456789,
        mode=1,
    )


def prepared(scene=None):
    scene = branch_scene() if scene is None else scene
    engine = TrackingEngine(GridIndex(scene.cloud), config=scene.config)
    initialized = engine.initialize(scene.seed)
    if not initialized.success:
        raise AssertionError(initialized.status)
    return scene, engine


def prepared_completed_stack(*, initialized_references=True):
    track = tracking_scene(length=2)
    toe = toe_scene(6, initialized_references=initialized_references)
    shift = np.array([100.0, 100.0, 0.0])
    cloud = PointCloud(np.vstack((track.cloud.xyz, toe.cloud.xyz + shift)))
    engine = TrackingEngine(GridIndex(cloud))
    engine.initialize(track.seed)
    for segment in toe.stack.track_segments:
        for window in (segment.track1, segment.track2):
            line = window.member_line
            if line is not None:
                line.begin = Point3D(*(line.begin.as_array() + shift))
                line.end = Point3D(*(line.end.as_array() + shift))
                line.update_parameters()
                line.update_center()
                window.center = line.center
            window.members = cloud.select([])
            window.left_members = cloud.select(window.left_members.indices + len(track.cloud))
            window.right_members = cloud.select(window.right_members.indices + len(track.cloud))
    engine.completed_stacks.append(toe.stack)
    return engine


class VectorOrchestrationTests(unittest.TestCase):
    def test_exact_xy_match_updates_only_first_type_and_keeps_z(self):
        points = [Point3D(1, 2, 7), Point3D(1, 2, 8)]
        result = tag_rail_vector(points, Point3D(1, 2, 99), 4)
        self.assertEqual((result.status, result.index), ("existing_vertex", 0))
        self.assertEqual(points, [Point3D(1, 2, 7, 4), Point3D(1, 2, 8)])

    def test_oblique_projection_uses_mean_endpoint_height(self):
        points = [Point3D(0, 0, 2), Point3D(4, 4, 6)]
        result = tag_rail_vector(points, Point3D(1, 3, 99))
        self.assertEqual((result.status, result.index), ("inserted_projection", 1))
        self.assertEqual(points[1], Point3D(2, 2, 4, 2))

    def test_axis_aligned_segments_fall_back_to_first_nearest_vertex(self):
        for points, query in (
            ([Point3D(0, 0, 2), Point3D(4, 0, 6)], Point3D(2, 1, 99)),
            ([Point3D(0, 0, 2), Point3D(0, 4, 6)], Point3D(1, 2, 99)),
        ):
            result = tag_rail_vector(points, query)
            self.assertEqual((result.status, result.index), ("nearest_vertex", 0))
            self.assertEqual(len(points), 2)

    def test_empty_vector_and_nonfinite_distance_are_explicit(self):
        self.assertEqual(
            tag_rail_vector([], Point3D(0, 0, 0)).status, "undefined_source_empty_vector"
        )
        points = [Point3D(0, 0, 1)]
        result = tag_rail_vector(points, Point3D(float("nan"), 1, 2))
        self.assertEqual(result.status, "unchanged")
        self.assertEqual(points[0].point_type, 0)

    def test_reversal_changes_only_vectors_and_pops_only_first_layer(self):
        main, branch = layer_fixture(), layer_fixture()
        stack, toe = TrackStack(), toe_fixture()
        main.track_segments.append(stack)  # Sentinel; reversal must not inspect it.
        main.toe_points.append(toe)
        original_first = main.rail1_vector
        result = reverse_layer_vectors([main, branch])
        self.assertEqual(result.status, "ok")
        self.assertIs(main.rail1_vector, original_first)
        self.assertEqual(main.rail1_vector, [Point3D(4, 4, 5), Point3D(3, 3, 4)])
        self.assertEqual(
            branch.rail1_vector, [Point3D(4, 4, 5), Point3D(3, 3, 4), Point3D(2, 2, 3)]
        )
        self.assertEqual(main.track_segments, [stack])
        self.assertIs(main.toe_points[0], toe)
        self.assertEqual(
            result.popped_points, (Point3D(2, 2, 3), Point3D(0, 0, 1), Point3D(1, 1, 2))
        )

    def test_undefined_reversal_preserves_prior_mutations(self):
        self.assertEqual(reverse_layer_vectors([]).status, "undefined_source_reverse_layer")
        main = TrackLayer(rail2_vector=[Point3D(1, 2, 3)], center_vector=[Point3D(4, 5, 6)])
        branch = layer_fixture()
        result = reverse_layer_vectors([main, branch])
        self.assertEqual(result.status, "undefined_source_vector_pop")
        self.assertEqual(result.popped_points, (Point3D(1, 2, 3),))
        self.assertEqual(main.center_vector, [Point3D(4, 5, 6)])
        self.assertEqual(branch.rail1_vector[0], Point3D(4, 4, 5))

    def test_reverse_seed_preserves_literal_unwrapped_angle_and_zero_mode(self):
        for mode in range(5):
            location = InitialLocation(Point3D(1, 2, 3), 280.25, mode, Point3D(4, 5, 6))
            seed = reverse_seed(location)
            self.assertEqual(seed.point, location.second_point)
            self.assertEqual(seed.angle, 460.25)
            self.assertEqual(seed.mode, (mode + 2) % 4)
        self.assertEqual(reverse_seed(replace(location, mode=-5)).mode, -3)
        with self.assertRaises(ValueError):
            reverse_seed(replace(location, second_point=None))


class EngineTests(unittest.TestCase):
    def test_undefined_initialization_cannot_be_reported_as_a_defined_empty_pass(self):
        scene = tracking_scene(length=2)
        engine = TrackingEngine(GridIndex(scene.cloud), rng=RecordedRand([0] * 400))
        initialized = engine.initialize(scene.seed)
        self.assertEqual(initialized.status, "undefined_source_initial_line")
        result = engine.run_pass()
        self.assertEqual(result.status, "undefined_source_initial_line")
        self.assertFalse(result.reference_comparable)
        self.assertEqual(result.branches, ())

    def test_reset_preserves_global_pools_and_failed_initialization_state(self):
        scene, engine = prepared()
        location, heights = engine.initial_location, engine.initial_heights
        stack, toe = TrackStack(), toe_fixture()
        engine.completed_stacks.append(stack)
        engine.toe_points.append(toe)
        failed = engine.initialize(replace(scene.seed, point=Point3D(1e5, 1e5, 0)))
        self.assertEqual(failed.status, "no_head_points")
        self.assertEqual(len(engine.layers), 1)
        self.assertEqual(engine.layers[0].track_segments, [])
        self.assertIs(engine.initial_location, location)
        self.assertEqual(engine.initial_heights, heights)
        self.assertIs(engine.completed_stacks[0], stack)
        self.assertIs(engine.toe_points[0], toe)

    def test_reset_wrapper_narrows_angle_but_direct_initialization_does_not(self):
        scene, engine = prepared()
        seed = replace(scene.seed, angle=35.123456789)
        from railway_mls.initialization import initialize_tracks

        with patch("railway_mls.orchestration.initialize_tracks", wraps=initialize_tracks) as init:
            engine.initialize(seed)
            self.assertEqual(init.call_args.args[0].angle, float(np.float32(seed.angle)))
            engine.initialize(seed, reset_layers=False)
            self.assertEqual(init.call_args.args[0].angle, seed.angle)

    def test_branch_runs_actual_inference_and_keeps_local_used_and_y_as_z(self):
        scene, engine = prepared()
        main_location = engine.initial_location
        engine.toe_points.append(scene.toe)
        result = engine.run_pass()
        self.assertEqual(result.status, "completed")
        self.assertEqual(result.main.direction, 1)
        self.assertEqual(result.main.result.accepted_steps, 6)
        branch = result.branches[0]
        self.assertEqual(branch.tracking.direction, 0)
        self.assertFalse(branch.tracking.main_track)
        self.assertEqual(branch.tracking.result.accepted_steps, 5)
        self.assertTrue(branch.toe.used)
        self.assertFalse(engine.toe_points[0].used)
        self.assertEqual(branch.seed.point.z, scene.toe.right.y)
        self.assertNotEqual(branch.seed.point.z, scene.toe.right.z)
        self.assertLess(branch.initialization.location.point.z, 1)
        self.assertIs(engine.initial_location, branch.initialization.location)
        self.assertNotEqual(engine.initial_location.point, main_location.point)
        self.assertGreater(np.count_nonzero(scene.cloud.class_codes == 3), 100)
        self.assertFalse(result.reference_comparable)  # Single-candidate MDL policy.

    def test_skipped_and_failed_branches_retain_empty_layers_and_last_pointer(self):
        scene, engine = prepared()
        failed = replace(scene.toe, right=Point3D(1e5, 1e5, 9), used=True)
        engine.toe_points.extend([failed, replace(scene.toe, track_type=6)])
        location = engine.initial_location
        result = engine.run_pass()
        self.assertEqual(
            [entry.status for entry in result.branches], ["initialization_failed", "skipped_type"]
        )
        self.assertEqual([len(layer.track_segments) for layer in engine.layers], [7, 0, 0])
        self.assertEqual(engine.current_layer_index, 2)
        self.assertEqual(engine.distance_from_start, 0)
        self.assertIs(engine.initial_location, location)
        self.assertTrue(result.branches[0].toe.used)
        self.assertFalse(result.branches[1].toe.used)
        self.assertTrue(engine.toe_points[0].used)

    def test_missing_branch_metadata_stops_after_allocating_layer(self):
        _, engine = prepared()
        engine.toe_points.extend([ToePoint(track_type=0), toe_fixture()])
        result = engine.run_pass()
        self.assertEqual(result.status, "undefined_source_branch_seed")
        self.assertEqual(len(result.branches), 1)
        self.assertEqual(len(engine.layers), 2)
        self.assertFalse(result.branches[0].toe.used)

    def test_nonfinite_branch_seed_stops_before_grid_conversion(self):
        scene, engine = prepared()
        engine.toe_points.append(replace(scene.toe, right=Point3D(float("nan"), 2, 3)))
        result = engine.run_pass()
        self.assertEqual(result.status, "nonfinite_branch_seed")
        self.assertEqual(engine.distance_from_start, 0)
        self.assertEqual(engine.layers[-1].track_segments, [])

    def test_undefined_branch_fit_stops_before_used_write_and_next_branch(self):
        scene, engine = prepared()
        engine.toe_points.extend([scene.toe, scene.toe])
        track = engine._track_current

        def after_main(*args):
            result = track(*args)
            engine.rng = RecordedRand([0] * 400)
            return result

        with patch.object(engine, "_track_current", side_effect=after_main):
            result = engine.run_pass()
        self.assertEqual(result.status, "undefined_source_initial_line")
        self.assertEqual(len(result.branches), 1)
        self.assertFalse(result.branches[0].toe.used)
        self.assertEqual(engine.rng.draw_count, 400)

    def test_main_step_limit_defers_branch_scheduling_and_tags(self):
        scene, engine = prepared()
        engine.toe_points.append(scene.toe)
        result = engine.run_pass(max_steps=1)
        self.assertEqual(result.status, "step_limit")
        self.assertEqual(result.branches, ())
        self.assertEqual(result.tags, ())
        self.assertEqual(len(engine.layers), 1)
        self.assertEqual(engine.run_pass().status, "undefined_source_tracking_reentry")

    def test_branch_step_limit_defers_local_used_write_and_tags(self):
        scene = branch_scene()
        # Shorten only the main corridor so it terminates within three steps.
        xyz = scene.cloud.xyz
        along = np.array([np.cos(np.deg2rad(35)), np.sin(np.deg2rad(35))])
        mask = ((xyz[:, :2] - scene.seed.point.as_array()[:2]) @ along < 1.5) | (xyz[:, 0] > 1010)
        scene = replace(scene, cloud=PointCloud(xyz[mask]))
        _, engine = prepared(scene)
        engine.toe_points.append(scene.toe)
        result = engine.run_pass(max_steps=3)
        self.assertEqual(result.main.status, "insufficient_window_points")
        self.assertEqual(result.status, "step_limit")
        self.assertEqual(result.branches[0].tracking.result.accepted_steps, 3)
        self.assertFalse(result.branches[0].toe.used)
        self.assertEqual(result.tags, ())

    def test_invalid_limits_and_grid_sizes_fail_before_mutations(self):
        _, engine = prepared()
        draws = engine.rng.draw_count
        for method in (engine.run_pass, engine.run_bidirectional):
            with self.assertRaises(ValueError):
                method(max_steps=-1)
            self.assertEqual(engine.rng.draw_count, draws)
        with self.assertRaises(ValueError):
            TrackingEngine(engine.grid, config=ExtractionConfig(grid_size=2))

    def test_two_directions_append_to_main_history_and_assemble_ordered_vectors(self):
        _, engine = prepared(bidirectional_scene())
        original = engine.initial_location
        result = engine.run_bidirectional()
        self.assertEqual(result.status, "completed")
        self.assertEqual(result.original_location, original)
        self.assertEqual(result.reverse_location.point, original.second_point)
        self.assertEqual(result.reverse_location.mode, 3)
        self.assertEqual(result.reversal.status, "ok")
        self.assertEqual(result.reverse_pass.main.direction, 1)
        self.assertEqual(result.forward_pass.main.direction, 1)
        self.assertEqual(len(engine.layers), 1)
        self.assertGreater(result.reverse_pass.main.result.accepted_steps, 3)
        self.assertGreater(result.forward_pass.main.result.accepted_steps, 3)
        self.assertGreater(len(engine.layers[0].track_segments), 10)
        radians = original.angle * np.pi / 180
        positions = np.array([p.as_array()[:2] for p in engine.layers[0].center_vector])
        self.assertGreater(np.ptp(positions @ [np.cos(radians), np.sin(radians)]), 10)
        self.assertTrue(np.all(np.diff(positions @ [np.cos(radians), np.sin(radians)]) > -0.01))

    def test_reverse_diagnostic_limit_defers_reversal_and_forward_pass(self):
        _, engine = prepared(bidirectional_scene())
        result = engine.run_bidirectional(max_steps=1)
        self.assertEqual(result.status, "step_limit")
        self.assertIsNone(result.reversal)
        self.assertIsNone(result.forward_pass)

    def test_failed_reverse_initialization_reports_unsafe_pop(self):
        _, engine = prepared()
        engine.initial_location = InitialLocation(
            Point3D(1e5, 1e5, 0), 35, 1, Point3D(1e5 + 1, 1e5 + 1, 0)
        )
        result = engine.run_bidirectional()
        self.assertEqual(result.reverse_initialization.status, "no_head_points")
        self.assertEqual(result.status, "undefined_source_vector_pop")
        self.assertIsNone(result.forward_initialization)

    def test_undefined_reverse_fit_stops_before_tracking(self):
        _, engine = prepared(bidirectional_scene())
        engine.rng = RecordedRand([0] * 400)
        result = engine.run_bidirectional()
        self.assertEqual(result.status, "undefined_source_initial_line")
        self.assertIsNone(result.reverse_pass)
        self.assertIsNone(result.reversal)

    def test_missing_layer_toe_center_is_reported_after_tracking(self):
        _, engine = prepared()
        engine.current_layer.toe_points.append(ToePoint(track_type=6))
        result = engine.run_pass()
        self.assertEqual(result.status, "undefined_source_toe_center")
        self.assertGreater(result.main.result.accepted_steps, 0)
        self.assertEqual(result.tags[0].result.status, "undefined_source_toe_center")


class CppOrchestrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def compare_tag(self, points, query, point_type=2):
        native = self.reference.run(
            "tag_vector", *typed_vector(points), *query.as_array(), point_type
        )
        result = tag_rail_vector(points, query, point_type)
        assert_allclose(typed_vector(points), native, atol=1e-12, rtol=1e-12)
        return result

    def test_exact_match_and_duplicates_match_original(self):
        self.compare_tag([Point3D(1, 2, 3, 7), Point3D(1, 2, 9, 8)], Point3D(1, 2, 100), 5)

    def test_oblique_projection_and_mean_height_match_original(self):
        self.compare_tag([Point3D(0, 0, 2), Point3D(4, 4, 6)], Point3D(1, 3, 100))

    def test_axis_aligned_fallback_and_nearest_ties_match_original(self):
        self.compare_tag([Point3D(0, 0, 2), Point3D(4, 0, 6)], Point3D(2, 1, 100))
        self.compare_tag([Point3D(0, 0, 2), Point3D(0, 4, 6)], Point3D(1, 2, 100))
        self.compare_tag([Point3D(0, 0, 2)], Point3D(9, 9, 100))

    def test_vector_reversal_all_layers_and_first_layer_pop_match_original(self):
        layers = [layer_fixture(), TrackLayer(), layer_fixture()]
        values = [len(layers)]
        for layer in layers:
            for vector in (layer.rail1_vector, layer.rail2_vector, layer.center_vector):
                values.extend(typed_vector(vector))
        native = self.reference.run("reverse_vectors", *values)
        self.assertEqual(reverse_layer_vectors(layers).status, "ok")
        expected = [0, len(layers)]
        for layer in layers:
            for vector in (layer.rail1_vector, layer.rail2_vector, layer.center_vector):
                expected.extend(typed_vector(vector))
        assert_array_equal(expected, native)

    def test_reversal_moves_xyz_while_preserving_destination_tag_slots(self):
        layers = [layer_fixture(), layer_fixture()]
        for layer in layers:
            for vector in (layer.rail1_vector, layer.rail2_vector, layer.center_vector):
                vector[:] = [replace(p, point_type=10 + i) for i, p in enumerate(vector)]
        values = [len(layers)]
        for layer in layers:
            for vector in (layer.rail1_vector, layer.rail2_vector, layer.center_vector):
                values.extend(typed_vector(vector))
        native = self.reference.run("reverse_vectors", *values)
        reverse_layer_vectors(layers)
        expected = [0, len(layers)]
        for layer in layers:
            for vector in (layer.rail1_vector, layer.rail2_vector, layer.center_vector):
                expected.extend(typed_vector(vector))
        assert_array_equal(expected, native)
        self.assertEqual([p.point_type for p in layers[1].center_vector], [10, 11, 12])

    def compare_schedule(self, toes, successes, new_toes=(), location=None):
        # Controlled callbacks isolate the ORIGINAL wrappers' ordering and state
        # mutations; actual numerical tracking is covered by separate integrations.
        cloud = PointCloud([(0, 0, 0), (100, 100, 0)])
        engine = TrackingEngine(GridIndex(cloud))
        engine.layers.append(layer_fixture())
        engine.current_layer = engine.layers[0]
        engine.current_layer.track_segments.append(
            seed_windows(InitialLocation(Point3D(0, 0, 0), 35, 1))
        )
        engine.toe_points.extend(replace(toe) for toe in toes)
        engine.current_layer.toe_points.extend(replace(toe) for toe in toes)
        engine.initial_location = location
        trace, success_iter = [], iter(successes)

        def initialize(seed, grid, *, layer, **kwargs):
            success = next(success_iter)
            layer_index = next((i for i, item in enumerate(engine.layers) if item is layer), -1)
            trace.append(
                [1, layer_index, seed.mode, *seed.point.as_array(), seed.angle, int(success)]
            )
            segment = seed_windows(seed)
            empty = grid.cloud.select([])
            if not success:
                return InitializationResult(
                    segment, layer, (), (empty, empty), None, (), None, None, "no_head_points"
                )
            first = seed.point + Point3D(0.25, 0.5, 0.75)
            second = first + Point3D(1, 2, 3)
            layer.rail1_vector.append(first)
            layer.rail2_vector.append(second)
            layer.center_vector.append((first + second) / 2)
            layer.track_segments.append(segment)
            fitted = InitialLocation(first, seed.angle + 0.125, seed.mode, second)
            return InitializationResult(
                segment, layer, (), (empty, empty), (first.z, second.z), (), None, fitted, "ok"
            )

        def track(direction, main_track, max_steps):
            trace.append([2, engine.current_layer_index, direction, int(main_track)])
            if main_track:
                engine.toe_points.extend(replace(toe) for toe in new_toes)
                engine.current_layer.toe_points.extend(replace(toe) for toe in new_toes)
            return TrackingInvocation(
                engine.current_layer_index, direction, main_track, None, "no_segments"
            )

        with (
            patch("railway_mls.orchestration.initialize_tracks", side_effect=initialize),
            patch.object(engine, "_track_current", side_effect=track),
        ):
            result = engine.run_pass() if location is None else engine.run_bidirectional()
        values = [
            len(toes),
            *(v for toe in toes for v in toe_state(toe)),
            len(new_toes),
            *(v for toe in new_toes for v in toe_state(toe)),
            len(successes),
            *map(int, successes),
        ]
        if location is not None:
            values.extend(
                [
                    *location.point.as_array(),
                    *location.second_point.as_array(),
                    location.angle,
                    location.mode,
                ]
            )
        native = self.reference.run(
            "orchestrate" if location is None else "bidirectional_orchestrate", *values
        )
        expected = [
            len(trace),
            *(v for event in trace for v in (len(event), *event)),
            engine.current_layer_index,
            len(engine.layers),
        ]
        for layer in engine.layers:
            for vector in (layer.rail1_vector, layer.rail2_vector, layer.center_vector):
                expected.extend(typed_vector(vector))
            expected.extend([len(layer.track_segments), len(layer.toe_points)])
        local = result.branches if location is None else result.forward_pass.branches
        for pool in (engine.toe_points, [entry.toe for entry in local]):
            expected.extend(
                [len(pool), *(v for toe in pool for v in (toe.track_type, int(toe.used)))]
            )
        assert_allclose(expected, native, rtol=1e-12, atol=1e-10)
        self.assertEqual(result.status, "completed")
        return engine, result, trace

    def test_all_source_type_filters_and_unknown_types_match_original(self):
        types = list(range(-1, 21)) + [999]
        engine, result, _ = self.compare_schedule([toe_fixture(t) for t in types], [True] * 9)
        self.assertEqual(len(engine.layers), len(types) + 1)
        self.assertEqual(sum(b.status == "skipped_type" for b in result.branches), 14)

    def test_defined_initialization_failure_and_used_copy_match_original(self):
        self.compare_schedule(
            [toe_fixture(0, used=True), toe_fixture(1), toe_fixture(6)], [False, True]
        )

    def test_main_generated_toes_enter_snapshot_once_and_match_original(self):
        self.compare_schedule([], [True], [toe_fixture(0, used=True)])

    def test_empty_pool_and_all_skipped_types_match_original(self):
        self.compare_schedule([], [])
        self.compare_schedule([toe_fixture(6), toe_fixture(19)], [])

    def test_bidirectional_ui_order_original_seed_capture_and_float_wrapper_match_original(self):
        for mode in (-5, 0, 1, 2, 3, 4):
            location = InitialLocation(Point3D(1, 2, 3), 280.123456789, mode, Point3D(4, 5, 6))
            _, _, trace = self.compare_schedule(
                [toe_fixture(6)],
                [True, True, True, True, True],
                [toe_fixture(0)],
                location,
            )
            initial_calls = [event for event in trace if event[0] == 1]
            self.assertEqual(initial_calls[0][6], float(np.float32(location.angle + 180)))
            forward = [event for event in initial_calls if event[3:6] == [1, 2, 3]]
            self.assertEqual(forward[0][6], location.angle)

    def test_failed_forward_initialization_still_calls_tracking_as_in_original_ui(self):
        location = InitialLocation(Point3D(1, 2, 3), 35, 1, Point3D(4, 5, 6))
        _, result, trace = self.compare_schedule([toe_fixture(6)], [True, False], location=location)
        self.assertEqual(result.forward_initialization.source_return, False)
        self.assertEqual(sum(event[0] == 2 for event in trace), 2)


class OrchestrationIntegrationTests(unittest.TestCase):
    def test_real_main_postpass_toe_is_scheduled_and_layer_center_is_tagged(self):
        engine = prepared_completed_stack()
        result = engine.run_pass()
        self.assertEqual(len(result.main.result.toe_detection.new_toes), 1)
        self.assertEqual(len(result.branches), 1)
        self.assertEqual(result.branches[0].toe.track_type, 6)
        self.assertEqual(result.branches[0].status, "skipped_type")
        self.assertEqual(len(engine.layers), 2)
        self.assertEqual(len(result.tags), 1)
        self.assertEqual(sum(p.point_type == 2 for p in engine.layers[0].center_vector), 1)
        self.assertFalse(engine.toe_points[0].used)

    def test_undefined_automatic_toe_does_not_become_a_branch_seed(self):
        engine = prepared_completed_stack(initialized_references=False)
        result = engine.run_pass()
        self.assertEqual(
            result.main.result.toe_detection.entries[0].status, "undefined_source_reference_line"
        )
        self.assertEqual(result.branches, ())
        self.assertEqual(engine.toe_points, [])
        self.assertFalse(result.reference_comparable)

    def test_detected_component_toe_can_seed_actual_raw_branch_tracking(self):
        scene, engine = prepared()
        fixture = toe_scene(angle=50)
        detected = detect_toe_points(0, fixture.stack.track_segments, rng=engine.rng)
        self.assertTrue(detected.success)
        shift = Point3D(15, 10, 0)
        engine.toe_points.append(
            replace(
                detected.toe,
                center=detected.toe.center + shift,
                left=detected.toe.left + shift,
                right=detected.toe.right + shift,
            )
        )
        result = engine.run_pass()
        self.assertTrue(result.branches[0].initialization.success)
        self.assertGreater(result.branches[0].tracking.result.accepted_steps, 3)

    def test_branch_and_bidirectional_cli_demos_and_limit(self):
        for command in ("branch-demo", "bidirectional-demo"):
            completed = subprocess.run(
                [sys.executable, "-m", "railway_mls", command],
                text=True,
                capture_output=True,
                check=True,
            )
            payload = json.loads(completed.stdout)
            self.assertEqual(payload["status"], "completed")
            self.assertGreater(payload["random_draws"], 10000)
            self.assertFalse(payload["reference_comparable"])
        completed = subprocess.run(
            [sys.executable, "-m", "railway_mls", "branch-demo", "--max-steps", "1"],
            text=True,
            capture_output=True,
            check=True,
        )
        payload = json.loads(completed.stdout)
        self.assertEqual(payload["status"], "step_limit")
        self.assertEqual(payload["layer_count"], 1)


if __name__ == "__main__":
    unittest.main()
