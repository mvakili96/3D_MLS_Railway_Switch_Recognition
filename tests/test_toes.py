from copy import copy
from dataclasses import asdict, replace
import json
import subprocess
import sys
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls.config import ExtractionConfig
from railway_mls.geometry import Line3D
from railway_mls.models import ToePoint, TrackLayer, TrackSegment, TrackStack, TrackWindow
from railway_mls.points import Point3D, PointCloud
from railway_mls.sampling import MsvcRand, RecordedRand
from railway_mls.synthetic import toe_scene, tracking_scene
from railway_mls.spatial import GridIndex
from railway_mls.initialization import initialize_tracks
from railway_mls.toes import detect_stack_toes, detect_toe_points
from railway_mls.tracking import SequentialTracker
from tests.cpp_reference import CppReference
from tests.test_recognition import binary_state
from tests.test_tracking import state_line


SUPPORTED = (0, 1, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15)


def toe_state(toe):
    geometry = toe.center is not None
    values = [toe.track_type, int(toe.used), int(geometry)]
    if geometry:
        values.extend(
            [
                *toe.center.as_array(),
                *toe.left.as_array(),
                *toe.right.as_array(),
                toe.angle,
                toe.mode,
            ]
        )
    return values


def point_ids(points):
    return [0] if points is None else [len(points), *points.indices]


def segment_inputs(segments):
    values = [len(segments)]
    for segment in segments:
        for window in (segment.track1, segment.track2):
            values.append(int(window.member_line is not None))
            if window.member_line is not None:
                values.extend(state_line(window.member_line))
            values.extend([*point_ids(window.left_members), *point_ids(window.right_members)])
    return values


def horizontal_fixture(points=None, reference_x=-0.5):
    if points is None:
        points = [(i * 0.01, 1, 10 + i * 0.001) for i in range(61)]
    cloud = PointCloud(points, class_codes=[9] * len(points), segment_ids=[7] * len(points))
    empty = cloud.select([])
    segments = []
    for _ in range(6):
        first = Line3D(Point3D(reference_x, 0, 1), Point3D(reference_x + 1, 0, 3))
        second = Line3D(Point3D(reference_x, 2, 3), Point3D(reference_x + 1, 2, 7))
        segments.append(
            TrackSegment(
                TrackWindow(
                    first.center,
                    first.angle,
                    member_line=first,
                    left_members=empty,
                    right_members=empty,
                ),
                TrackWindow(
                    second.center,
                    second.angle,
                    member_line=second,
                    left_members=empty,
                    right_members=empty,
                ),
            )
        )
    middle = len(points) // 2
    for start in (0, 4):
        for i, indices in enumerate((np.arange(middle), np.arange(middle, len(points)))):
            segments[start + i].track2.right_members = cloud.select(indices)
            segments[start + i].track1.left_members = cloud.select(indices)
    return cloud, TrackStack(segments)


class ToeDetectionTests(unittest.TestCase):
    def test_all_supported_types_choose_exact_source_side_and_segment_pairs(self):
        for type_id in SUPPORTED:
            scene = toe_scene(type_id)
            random = MsvcRand()
            result = detect_toe_points(type_id, scene.stack.track_segments, rng=random)
            self.assertTrue(result.success)
            self.assertTrue(result.source_return)
            self.assertEqual(random.draw_count, 1000)
            start = type_id in (0, 1, 4, 5, 12, 13)
            right = type_id in (0, 1, 6, 7, 12, 13, 14, 15)
            offset = (0 if start else 120) + (0 if right else 60)
            assert_array_equal(result.candidates.indices, np.arange(offset, offset + 60))
            self.assertEqual(result.source_segments, (0, 1) if start else (4, 5))
            self.assertEqual(result.reference_segment, 0 if start else 5)
            self.assertEqual(result.side, "track2_right" if right else "track1_left")

    def test_unsupported_types_leave_existing_toe_unchanged_and_consume_no_draws(self):
        output = ToePoint(Point3D(1, 2, 3), Point3D(4, 5, 6), Point3D(7, 8, 9), 55, True, 20.0, 3)
        before = asdict(output)
        for type_id in (2, 3, 8, 9, 16, 17, 18, 19, -1, 999):
            random = RecordedRand([])
            result = detect_toe_points(type_id, [], toe=output, rng=random)
            self.assertFalse(result.source_return)
            self.assertEqual(result.status, "unsupported_type")
            self.assertEqual(asdict(output), before)
            self.assertEqual(random.draw_count, 0)

    def test_short_stack_and_failed_fit_are_explicit(self):
        result = detect_toe_points(0, [])
        self.assertEqual(result.status, "undefined_source_short_stack")
        self.assertEqual(result.toe.track_type, 0)
        self.assertIsNone(result.source_return)
        scene = toe_scene()
        random = RecordedRand([0] * 1000)
        result = detect_toe_points(0, scene.stack.track_segments, rng=random)
        self.assertEqual(result.status, "undefined_source_toe_fit")
        self.assertEqual(random.draw_count, 1000)
        self.assertTrue(np.any(scene.cloud.segment_ids == 0))

    def test_real_tracking_snapshot_missing_reference_is_reported_after_grouping_and_fit(self):
        scene = toe_scene(initialized_references=False)
        random = MsvcRand()
        result = detect_toe_points(0, scene.stack.track_segments, rng=random)
        self.assertEqual(result.status, "undefined_source_reference_line")
        self.assertIsNone(result.toe.center)
        self.assertIsNotNone(result.refined_line)
        self.assertEqual(random.draw_count, 1000)
        assert_array_equal(scene.cloud.segment_ids[:60], 0)
        assert_array_equal(scene.cloud.segment_ids[60:], 7)
        assert_array_equal(scene.cloud.class_codes, 9)

    def test_projection_uses_reference_line_heights_and_preserves_used_flag(self):
        cloud, stack = horizontal_fixture()
        output = ToePoint(used=True)
        result = detect_toe_points(0, stack.track_segments, toe=output, rng=MsvcRand())
        self.assertTrue(result.success)
        self.assertTrue(output.used)
        assert_allclose(output.right.as_array(), [0, 0, 2])
        assert_allclose(output.left.as_array(), [0, 2, 5])
        assert_allclose(output.center.as_array(), [0, 1, 3.5])
        self.assertGreater(result.refined_line.begin.z, 10)
        self.assertEqual(output.angle, 0)
        self.assertEqual(output.mode, 1)

    def test_start_nearest_end_farthest_and_equal_distance_ties_choose_end(self):
        cloud, stack = horizontal_fixture(reference_x=-0.5)
        near = detect_toe_points(0, stack.track_segments, rng=MsvcRand())
        far = detect_toe_points(6, stack.track_segments, rng=MsvcRand())
        self.assertEqual(near.selected_endpoint, "begin")
        self.assertEqual(far.selected_endpoint, "end")
        points = [(i / 64, 1, i * 0.001) for i in range(65)]
        cloud, stack = horizontal_fixture(points, reference_x=0.5)
        for type_id in SUPPORTED:
            result = detect_toe_points(
                type_id, stack.track_segments, rng=RecordedRand([0, 1] * 500)
            )
            self.assertEqual(result.selected_endpoint, "end")

    def test_crop_preserves_z_angle_coefficients_and_old_cached_center(self):
        points = [(i * 0.01, 1, i * 0.001) for i in range(101)]
        points += [(1 + i * 0.01, 1 + i * 0.02, 1 + i * 0.001) for i in range(1, 31)]
        cloud, stack = horizontal_fixture(points)
        result = detect_toe_points(0, stack.track_segments, rng=RecordedRand([0, 1] * 500))
        self.assertTrue(result.success)
        self.assertLess(result.refined_line.end.x, result.fit.line.end.x)
        self.assertEqual(result.refined_line.center, result.fit.line.center)
        self.assertNotEqual(
            result.refined_line.center, (result.refined_line.begin + result.refined_line.end) / 2
        )
        self.assertEqual(result.refined_line.begin.z, result.fit.line.begin.z)
        self.assertEqual(
            (
                result.refined_line.a,
                result.refined_line.b,
                result.refined_line.c,
                result.refined_line.angle,
            ),
            (result.fit.line.a, result.fit.line.b, result.fit.line.c, result.fit.line.angle),
        )

    def test_grouping_is_hardcoded_three_centimetres_and_keeps_first_component_bias(self):
        points = [(i * 0.01, 1, i * 0.001) for i in range(2)]
        points += [(5 + i * 0.01, 1, 0.1 + i * 0.001) for i in range(10)]
        cloud, stack = horizontal_fixture(points)
        result = detect_toe_points(
            0,
            stack.track_segments,
            config=ExtractionConfig(grouping_distance=100, skip_rate=100),
            rng=MsvcRand(),
        )
        self.assertEqual(len(result.grouping.selected), 2)
        assert_array_equal(result.grouping.selected.indices, [0, 1])
        assert_array_equal(cloud.segment_ids, 0)  # Includes the discarded ten-point group.

    def test_side_point_duplicates_use_shared_identity_and_other_points_are_untouched(self):
        scene = toe_scene()
        scene.stack.track_segments[1].track2.right_members = scene.cloud.select(np.arange(20, 60))
        result = detect_toe_points(0, scene.stack.track_segments, rng=MsvcRand())
        self.assertEqual(len(result.candidates), 70)
        self.assertEqual(len(result.grouping.selected), 60)
        assert_array_equal(scene.cloud.segment_ids[60:], 7)

    def test_reference_coefficients_are_preserved_and_input_geometry_is_not_mutated(self):
        cloud, stack = horizontal_fixture()
        first = stack.track_segments[0].track1.member_line
        first.c, first.angle = 10.0, 77.0
        before = state_line(first)
        result = detect_toe_points(0, stack.track_segments, rng=MsvcRand())
        self.assertEqual(result.toe.right.y, 10)
        self.assertEqual(result.toe.angle, 0)
        self.assertEqual(state_line(first), before)

    def test_zero_projection_coefficients_keep_defined_nan_and_true_source_return(self):
        cloud, stack = horizontal_fixture()
        first = stack.track_segments[0].track1.member_line
        first.a = first.b = first.c = 0.0
        result = detect_toe_points(0, stack.track_segments, rng=MsvcRand())
        self.assertEqual(result.status, "nonfinite_toe_geometry")
        self.assertTrue(result.source_return)
        self.assertTrue(np.isnan(result.toe.right.y))

    def test_mixed_cloud_memberships_are_rejected(self):
        scene = toe_scene()
        scene.stack.track_segments[1].track2.right_members = PointCloud([[0, 0, 0]]).select()
        with self.assertRaises(ValueError):
            detect_toe_points(0, scene.stack.track_segments)


class ToeCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def cloud_inputs(self, cloud):
        return [
            len(cloud),
            *(
                value
                for xyz, code, segment in zip(cloud.xyz, cloud.class_codes, cloud.segment_ids)
                for value in (*xyz, code, segment)
            ),
        ]

    def compare_toe(self, type_id, cloud, stack, *, draws=None, output=None, config=None):
        output = output if output is not None else ToePoint()
        config = config if config is not None else ExtractionConfig()
        if draws is None:
            generator = MsvcRand()
            raw = [generator.rand() for _ in range(1000)]
        else:
            raw = draws
        inputs = [
            config.max_distance_for_ransac,
            *self.cloud_inputs(cloud),
            type_id,
            *segment_inputs(stack.track_segments),
            *toe_state(output),
            len(raw),
            *raw,
        ]
        random = RecordedRand(raw)
        result = detect_toe_points(
            type_id, stack.track_segments, config=config, rng=random, toe=output, cloud=cloud
        )
        self.assertIsNotNone(result.source_return)  # Never execute undefined cases as an oracle.
        native = self.reference.run("toe", *inputs)
        expected = [int(result.source_return), *toe_state(output), 2 if result.refined_line else 0]
        if result.refined_line:
            for points in (result.candidates, result.grouping.selected, result.fit.support):
                expected.extend(point_ids(points))
            expected.extend(
                [
                    result.fit.sigma,
                    *state_line(result.refined_line),
                    *result.selected_point.as_array(),
                ]
            )
        expected.extend(
            value
            for code, segment in zip(cloud.class_codes, cloud.segment_ids)
            for value in (code, segment)
        )
        expected.append(random.draw_count)
        assert_allclose(expected, native, rtol=1e-12, atol=1e-10, equal_nan=True)
        return result

    def test_all_twelve_type_branches_match_original(self):
        for type_id in SUPPORTED:
            scene = toe_scene(type_id)
            self.compare_toe(type_id, scene.cloud, scene.stack)

    def test_all_modes_axis_and_signed_slope_cases_match_original(self):
        for mode in (1, 2, 3, 4):
            for angle in (0, 35, 70, 90):
                scene = toe_scene(0, mode=mode, angle=angle)
                self.compare_toe(0, scene.cloud, scene.stack)

    def test_both_distance_comparisons_and_exact_ties_match_original(self):
        for reference_x in (-0.5, 0.5, 2.0):
            for type_id in (0, 4, 6, 10):
                points = [(i / 64, 1, i * 0.001) for i in range(65)]
                cloud, stack = horizontal_fixture(points, reference_x=reference_x)
                self.compare_toe(type_id, cloud, stack, draws=[0, 1] * 500)

    def test_support_crop_and_cached_center_match_original(self):
        points = [(i * 0.01, 1, i * 0.001) for i in range(101)]
        points += [(1 + i * 0.01, 1 + i * 0.02, 1 + i * 0.001) for i in range(1, 31)]
        cloud, stack = horizontal_fixture(points)
        self.compare_toe(0, cloud, stack, draws=[0, 1] * 500)

    def test_duplicate_side_ids_and_group_retention_match_original(self):
        scene = toe_scene()
        scene.stack.track_segments[1].track2.right_members = scene.cloud.select(np.arange(20, 60))
        self.compare_toe(0, scene.cloud, scene.stack)
        cloud, stack = horizontal_fixture(
            [(i * 0.01, 1, i * 0.001) for i in range(2)]
            + [(5 + i * 0.01, 1, 0.1 + i * 0.001) for i in range(10)]
        )
        self.compare_toe(0, cloud, stack)

    def test_reference_cached_coefficients_and_nonfinite_projection_match_original(self):
        cloud, stack = horizontal_fixture()
        stack.track_segments[0].track1.member_line.c = 10.0
        stack.track_segments[0].track1.member_line.angle = 77.0
        self.compare_toe(0, cloud, stack)
        cloud, stack = horizontal_fixture()
        line = stack.track_segments[0].track1.member_line
        line.a = line.b = line.c = 0.0
        self.compare_toe(0, cloud, stack)

    def test_unsupported_types_preserve_prior_output_and_draw_count(self):
        for type_id in (2, 3, 8, 9, 16, 17, 18, 19, -1, 999):
            scene = toe_scene()
            output = ToePoint(
                Point3D(1, 2, 3), Point3D(4, 5, 6), Point3D(7, 8, 9), 55, True, 20.0, 3
            )
            result = self.compare_toe(type_id, scene.cloud, scene.stack, output=output, draws=[])
            self.assertFalse(result.source_return)

    def compare_batch(self, scene, stacks, *, main_track=True, existing=()):
        inputs = [0.06, *self.cloud_inputs(scene.cloud), int(main_track), len(stacks)]
        for stack in stacks:
            inputs.extend([*binary_state(stack.binary), *segment_inputs(stack.track_segments)])
        inputs.extend([len(existing), *(value for toe in existing for value in toe_state(toe))])
        generator = MsvcRand()
        raw = [generator.rand() for _ in range(len(stacks) * 1000)]
        inputs.extend([len(raw), *raw])
        random, layer, pool = RecordedRand(raw), TrackLayer(), [replace(toe) for toe in existing]
        result = detect_stack_toes(
            stacks, rng=random, cloud=scene.cloud, layer=layer, toes=pool, main_track=main_track
        )
        self.assertTrue(result.reference_comparable)
        native = self.reference.run("toe_batch", *inputs)
        expected = [
            len(result.toes),
            *(value for toe in result.toes for value in toe_state(toe)),
            len(layer.toe_points),
            *(value for toe in layer.toe_points for value in toe_state(toe)),
        ]
        expected.extend(
            value
            for code, segment in zip(scene.cloud.class_codes, scene.cloud.segment_ids)
            for value in (code, segment)
        )
        expected.append(random.draw_count)
        assert_allclose(expected, native, rtol=1e-12, atol=1e-10, equal_nan=True)
        return result

    def test_batch_type_length_score_and_main_track_gates_match_original(self):
        scene = toe_scene()
        stacks = []
        for score in (0.899, 0.9, 0.9375, 1.0, float("nan")):
            stacks.append(
                TrackStack(scene.stack.track_segments, replace(scene.stack.binary, score=score))
            )
        stacks.extend(
            [
                TrackStack(scene.stack.track_segments[:4], copy(scene.stack.binary)),
                TrackStack(scene.stack.track_segments, replace(scene.stack.binary, type=999)),
            ]
        )
        result = self.compare_batch(scene, stacks)
        self.assertEqual(len(result.new_toes), 4)
        self.assertEqual(result.entries[0].status, "score_below_threshold")
        disabled = toe_scene()
        self.compare_batch(disabled, [disabled.stack], main_track=False)

    def test_batch_appends_existing_pool_and_preserves_used_markers(self):
        scene = toe_scene()
        old = ToePoint(Point3D(1, 2, 3), Point3D(4, 5, 6), Point3D(7, 8, 9), 55, True, 20.0, 3)
        result = self.compare_batch(scene, [scene.stack], existing=[old])
        self.assertEqual(len(result.toes), 2)
        self.assertTrue(result.toes[0].used)
        self.assertFalse(result.toes[1].used)


class ToeIntegrationTests(unittest.TestCase):
    def test_tracker_postpass_produces_toe_and_defers_it_at_diagnostic_limit(self):
        scene, track = toe_scene(), tracking_scene(length=2)
        shift = np.array([100.0, 100.0, 0.0])
        cloud = PointCloud(np.vstack((track.cloud.xyz, scene.cloud.xyz + shift)))
        grid, rng = GridIndex(cloud), MsvcRand()
        initialized = initialize_tracks(track.seed, grid, rng=rng)
        # A completed fixture from another region of the same global cloud.
        stack = scene.stack
        for segment in stack.track_segments:
            for window in (segment.track1, segment.track2):
                line = window.member_line
                line.begin = Point3D(*(line.begin.as_array() + shift))
                line.end = Point3D(*(line.end.as_array() + shift))
                line.update_parameters()
                line.update_center()
                window.center = line.center
                window.members = cloud.select([])
                window.left_members = cloud.select(window.left_members.indices + len(track.cloud))
                window.right_members = cloud.select(window.right_members.indices + len(track.cloud))
        tracker = SequentialTracker(
            initialized.layer, grid, initialized.initial_heights, rng=rng, completed_stacks=[stack]
        )
        limited = tracker.run(max_steps=1)
        self.assertIsNone(limited.toe_detection)
        result = tracker.run()
        self.assertIsNotNone(result.toe_detection)
        self.assertEqual(result.toe_detection.entries[0].status, "ok")
        self.assertEqual(len(result.layer.toe_points), 1)
        self.assertEqual(len(result.toe_detection.new_toes), 1)
        self.assertFalse(result.layer.toe_points[0].used)

    def test_batch_reports_missing_reference_and_retains_preceding_side_effects(self):
        scene = toe_scene(initialized_references=False)
        random, layer = MsvcRand(), TrackLayer()
        result = detect_stack_toes([scene.stack], rng=random, layer=layer)
        self.assertEqual(result.entries[0].status, "undefined_source_reference_line")
        self.assertEqual(result.toes, ())
        self.assertEqual(layer.toe_points, [])
        self.assertEqual(random.draw_count, 1000)
        self.assertFalse(result.reference_comparable)

    def test_toe_demo_runs_supported_types_and_exposes_missing_reference_case(self):
        completed = subprocess.run(
            [sys.executable, "-m", "railway_mls", "toe-demo"],
            text=True,
            capture_output=True,
            check=True,
        )
        result = json.loads(completed.stdout)
        self.assertEqual(result["detected_count"], 12)
        self.assertEqual(result["random_draws"], 12000)
        self.assertEqual(
            result["uninitialized_reference_example"]["status"], "undefined_source_reference_line"
        )
        self.assertFalse(result["branch_orchestration_implemented"])


if __name__ == "__main__":
    unittest.main()
