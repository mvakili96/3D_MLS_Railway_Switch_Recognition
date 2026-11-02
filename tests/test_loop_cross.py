from copy import copy
from dataclasses import replace
from pathlib import Path
import json
import subprocess
import sys
import unittest
from unittest.mock import patch

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls import (
    ExtractionConfig,
    Line3D,
    LoopCross,
    MsvcRand,
    PipelineOptions,
    Point3D,
    PointCloud,
    RecordedRand,
    detect_loop_cross,
    run_pipeline,
    validate_loop_cross,
)
from railway_mls.coverage import check_source_coverage, coverage_summary
from railway_mls.synthetic import loop_cross_scene, tracking_scene
from tests.cpp_reference import CppReference


def selection(points):
    return [len(points), *map(int, points.indices)]


def line_state(line):
    return [
        *line.begin.as_array(),
        *line.end.as_array(),
        *line._center.as_array(),
        line.angle,
        line.a,
        line.b,
        line.c,
    ]


def input_line(line):
    return [*line_state(line), *(selection(line.members) if line.members is not None else [0])]


def output_line(line):
    return input_line(line)


def read_selection(values, cursor):
    count = int(values[cursor])
    return values[cursor + 1 : cursor + 1 + count], cursor + 1 + count


def prior_cross(scene, offset=0.0):
    direction = scene.reference_line.unit_vector
    across = Point3D(-direction.y, direction.x, 0)
    begin = scene.segment.track1.center + direction * offset
    end = begin + across * scene.config.track_gauge
    line = Line3D(begin, end)
    line.update_center()
    return LoopCross(line, 50.0)


class LoopCrossCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def compare_detection(self, scene=None, *, ids=None, pool=None, seed=1):
        scene = loop_cross_scene() if scene is None else scene
        points = scene.cloud.select(ids)
        original_pool = [] if pool is None else pool
        random = MsvcRand(seed)
        draws = [random.rand() for _ in range(max(512, len(points) * 3))]
        inputs = [len(scene.cloud)]
        for i, xyz in enumerate(scene.cloud.xyz):
            inputs.extend((*xyz, int(scene.cloud.class_codes[i]), int(scene.cloud.segment_ids[i])))
        inputs.extend(selection(points))
        for window in (scene.segment.track1, scene.segment.track2):
            inputs.extend(
                (*window.center.as_array(), *input_line(window.member_line), int(window.turnout))
            )
        inputs.extend(
            (
                scene.segment.distance_from_start,
                *input_line(scene.reference_line),
                scene.ground_height,
                scene.track_height,
                scene.config.track_window_length,
                scene.config.track_window_width,
                scene.config.skip_rate,
                len(original_pool),
            )
        )
        for value in original_pool:
            inputs.extend((*input_line(value.line), value.distance_from_start))
        inputs.extend((len(draws), *draws))
        native = self.reference.run("loop_detect", *inputs)
        replay = RecordedRand(draws)
        actual = detect_loop_cross(
            scene.segment,
            points,
            scene.ground_height,
            scene.track_height,
            scene.reference_line,
            config=scene.config,
            rng=replay,
            loop_crosses=original_pool,
        )
        cursor = 1
        if actual.region_points is None:
            groups = (scene.cloud.select([]),) * 5
        else:
            groups = (
                *actual.region_points,
                actual.height_points,
                actual.candidates,
                actual.fit.support if actual.fit is not None else scene.cloud.select([]),
            )
        for group in groups:
            expected, cursor = read_selection(native, cursor)
            assert_array_equal(group.indices, expected)
        fit_flag, fit_stage, draw_count, projected = native[cursor : cursor + 4]
        cursor += 4
        self.assertEqual(bool(fit_flag), actual.fit.success if actual.fit is not None else False)
        self.assertEqual(replay.draw_count, draw_count)
        self.assertEqual(bool(projected), actual.projected_line is not None)
        if projected:
            expected = output_line(actual.projected_line)
            assert_allclose(
                expected,
                native[cursor : cursor + len(expected)],
                rtol=1e-12,
                atol=1e-9,
                equal_nan=True,
            )
            cursor += len(expected)
        duplicates, cursor = read_selection(native, cursor)
        assert_allclose(actual.duplicate_distances or (), duplicates, equal_nan=True)
        self.assertEqual(len(actual.loop_crosses), native[cursor])
        cursor += 1
        for value in actual.loop_crosses:
            expected = [*output_line(value.line), value.distance_from_start]
            assert_allclose(
                expected,
                native[cursor : cursor + len(expected)],
                rtol=1e-12,
                atol=1e-9,
                equal_nan=True,
            )
            cursor += len(expected)
        labels = np.array(native[cursor:]).reshape(-1, 2)
        assert_array_equal(scene.cloud.class_codes, labels[:, 0])
        assert_array_equal(scene.cloud.segment_ids, labels[:, 1])
        if not native[0]:
            self.assertLess(fit_stage, 2)
            self.assertEqual(actual.status, "undefined_source_loop_fit_line")
        return actual, replay

    def test_complete_detection_matches_source_memberships_projection_chainage_and_draws(self):
        result, random = self.compare_detection()
        self.assertEqual(result.status, "accepted")
        self.assertTrue(result.appended)
        self.assertEqual(random.draw_count, 60)
        self.assertEqual(result.candidates.indices.tolist(), [*range(30, 60), *range(30)])

    def test_duplicate_gate_compares_both_stored_lines_and_only_the_last_record(self):
        for offsets in ((0.0,), (3.0,), (0.0, 3.0), (3.0, 0.0)):
            scene = loop_cross_scene()
            result, _ = self.compare_detection(
                scene, pool=[prior_cross(scene, offset) for offset in offsets]
            )
            self.assertEqual(result.appended, offsets[-1] == 3.0)

    def test_one_band_short_fit_is_projected_even_when_false_and_does_not_append(self):
        result, random = self.compare_detection(ids=list(range(30)))
        self.assertEqual(result.fit.status, "line_too_short")
        self.assertEqual(result.status, "fit_rejected")
        self.assertIsNotNone(result.projected_line)
        self.assertFalse(result.appended)
        self.assertEqual(random.draw_count, 30)

    def test_empty_and_insufficient_fit_prefixes_compare_without_reading_native_output(self):
        for ids in ([], list(range(10)), list(range(10)) + list(range(30, 39))):
            result, random = self.compare_detection(ids=ids)
            self.assertEqual(result.status, "undefined_source_loop_fit_line")
            self.assertEqual(random.draw_count, len(ids))

    def test_skip_rate_changes_integer_minimum_and_preserves_draw_order(self):
        scene = loop_cross_scene(config=ExtractionConfig(skip_rate=2))
        result, random = self.compare_detection(scene, ids=[*range(8), *range(30, 37)])
        self.assertEqual(result.status, "accepted")
        self.assertEqual(random.draw_count, 15)

    def test_ordered_duplicate_point_ids_are_preserved_through_both_polygons(self):
        result, random = self.compare_detection(ids=[*range(60), 0, 0, 30, 30])
        self.assertEqual(len(result.candidates), 64)
        self.assertEqual(random.draw_count, 64)

    def test_nan_ground_bound_and_unused_track_height_keep_literal_source_behavior(self):
        scene = replace(loop_cross_scene(), ground_height=float("nan"), track_height=float("nan"))
        result, _ = self.compare_detection(scene)
        self.assertEqual(len(result.height_points), len(result.polygon_points))

    def test_scaled_rail_coefficients_keep_literal_intersection_and_stale_angle(self):
        scene = loop_cross_scene()
        line = scene.segment.track2.member_line
        line.a, line.b, line.c = line.a * 2, line.b * 2, line.c * 2
        result, _ = self.compare_detection(scene)
        derived = Line3D(result.projected_line.begin, result.projected_line.end).angle
        self.assertGreater(abs(derived - result.projected_line.angle), 1)

    def test_zero_normal_stored_previous_line_produces_nan_duplicate_test(self):
        scene = loop_cross_scene()
        previous = prior_cross(scene)
        previous.line.a = previous.line.b = previous.line.c = 0
        result, _ = self.compare_detection(scene, pool=[previous])
        self.assertEqual(result.status, "duplicate_suppressed")
        self.assertTrue(np.isnan(result.duplicate_distances[0]))

    def test_duplicate_mean_equal_to_one_is_suppressed(self):
        scene = loop_cross_scene(angle=90)
        initial = detect_loop_cross(
            scene.segment,
            scene.cloud.select(),
            scene.ground_height,
            scene.track_height,
            scene.reference_line,
        )
        line = copy(initial.projected_line)
        line.begin = line.begin + Point3D(0, 1, 0)
        line.end = line.end + Point3D(0, 1, 0)
        line.update_coefficients()
        line.update_center()
        result, _ = self.compare_detection(scene, pool=[LoopCross(line, 50)])
        self.assertEqual(result.duplicate_distances, (1.0, 1.0))
        self.assertEqual(result.status, "duplicate_suppressed")

    def test_turnout_on_either_window_returns_before_regions_or_random_draws(self):
        for name in ("track1", "track2"):
            scene = loop_cross_scene()
            getattr(scene.segment, name).turnout = True
            result, random = self.compare_detection(scene, pool=[prior_cross(scene)])
            self.assertEqual(result.status, "turnout_skipped")
            self.assertEqual(random.draw_count, 0)

    def test_strict_height_and_rail_distance_boundaries_match_source(self):
        scene = loop_cross_scene(angle=90)
        shift = Point3D(-1000, -2000, 0)
        for window in (scene.segment.track1, scene.segment.track2):
            window.center = window.center + shift
            line = window.member_line
            line.begin, line.end = line.begin + shift, line.end + shift
            line.update_coefficients()
            line.update_center()
        xyz = scene.cloud.xyz + shift.as_array()
        xyz[:4, 0] = [-0.3, np.nextafter(-0.3, 0), -1.2, np.nextafter(-1.2, -1.5)]
        xyz[4:8, 2] = [0.1, 0, np.nextafter(0.1, 1), -1e-12]
        scene = replace(
            scene,
            cloud=PointCloud(
                xyz, class_codes=scene.cloud.class_codes, segment_ids=scene.cloud.segment_ids
            ),
        )
        result, _ = self.compare_detection(scene)
        self.assertTrue({0, 2, 4, 5} <= set(result.candidates.indices))
        self.assertTrue({1, 3, 6, 7}.isdisjoint(result.candidates.indices))

    def test_nonfinite_neighbor_regions_keep_native_polygon_predicate_behavior(self):
        scene = loop_cross_scene()
        scene.segment.track2.center = scene.segment.track1.center
        result, _ = self.compare_detection(scene)
        self.assertEqual(result.regions.status, "nonfinite_neighbor_vertices")
        self.assertGreater(len(result.polygon_points), len(scene.cloud))

    def test_initialized_zero_normal_rail_can_append_nonfinite_projection(self):
        scene = loop_cross_scene()
        line = scene.segment.track1.member_line
        line.a = line.b = line.c = 0
        # Avoid compiler-dependent equal-height support order in this fixture.
        xyz = scene.cloud.xyz.copy()
        xyz[62, 2] = 0.049
        scene = replace(scene, cloud=PointCloud(xyz))
        result, _ = self.compare_detection(scene)
        self.assertEqual(result.status, "nonfinite_loop_cross_geometry")
        self.assertTrue(result.appended)
        self.assertTrue(np.isnan(result.candidate.distance_from_start))

    def test_validation_phase_truncation_strict_edges_negative_and_large_chainages(self):
        for distances in (
            [],
            [0],
            [0, 1.999, 2, 23, 23.999, 24, 24.999, 25, 26.999, 27],
            [-25, -1, 0, 24, 49],
            [0, 21474836.9],
        ):
            values = [LoopCross(distance_from_start=v) for v in distances]
            result = validate_loop_cross(values)
            native = self.reference.run("loop_validate", len(distances), *distances)
            cursor = 1
            self.assertEqual(len(result.groups), native[0])
            for group in result.groups:
                expected, cursor = read_selection(native, cursor)
                assert_array_equal(group, expected)
            self.assertEqual(
                result.winner_index if result.winner_index is not None else -1, native[cursor]
            )
            cursor += 1
            selected, cursor = read_selection(native, cursor)
            assert_array_equal(result.selected_indices, selected)
            self.assertEqual(cursor, len(native))

    def test_validation_selects_first_largest_star_not_a_clique_or_sorted_component(self):
        for distances, expected in (
            ([0, 1.5, 3], [1, 0, 2]),
            ([0, 1, 10, 11], [0, 1]),
            ([0, 25, 50, 25], [0, 1, 2, 3]),
        ):
            native = self.reference.run("loop_validate", len(distances), *distances)
            result = validate_loop_cross([LoopCross(distance_from_start=v) for v in distances])
            self.assertEqual(list(result.selected_indices), expected)
            self.assertEqual(native[-len(expected) :], expected)


class LoopCrossContractTests(unittest.TestCase):
    def test_turnout_gate_avoids_all_missing_reference_height_and_fit_reads(self):
        scene = loop_cross_scene()
        scene.segment.track2.turnout = True
        scene.segment.track1.member_line = None
        random = RecordedRand([])
        result = detect_loop_cross(
            scene.segment, scene.cloud.select(), None, None, None, rng=random
        )
        self.assertEqual(result.status, "turnout_skipped")
        self.assertIsNone(result.regions)
        self.assertEqual(random.draw_count, 0)

    def test_missing_track_coefficients_fail_only_after_an_eligible_height_point(self):
        scene = loop_cross_scene()
        scene.segment.track1.member_line = None
        random = MsvcRand()
        result = detect_loop_cross(
            scene.segment,
            scene.cloud.select(),
            scene.ground_height,
            None,
            scene.reference_line,
            rng=random,
        )
        self.assertEqual(result.status, "undefined_source_loop_track_line")
        self.assertEqual(random.draw_count, 0)
        result = detect_loop_cross(
            scene.segment,
            scene.cloud.select([]),
            scene.ground_height,
            None,
            scene.reference_line,
            rng=random,
        )
        self.assertEqual(result.status, "undefined_source_loop_fit_line")

    def test_missing_chainage_is_read_only_for_successful_fit_after_projection(self):
        scene = loop_cross_scene()
        scene.segment.distance_from_start = None
        random = MsvcRand()
        result = detect_loop_cross(
            scene.segment,
            scene.cloud.select(),
            scene.ground_height,
            None,
            scene.reference_line,
            rng=random,
        )
        self.assertEqual(result.status, "undefined_source_loop_chainage")
        self.assertIsNotNone(result.projected_line)
        self.assertEqual(random.draw_count, 60)
        result = detect_loop_cross(
            scene.segment,
            scene.cloud.select(range(30)),
            scene.ground_height,
            None,
            scene.reference_line,
        )
        self.assertEqual(result.status, "fit_rejected")

    def test_missing_previous_center_preserves_pool_and_consumed_draws(self):
        scene = loop_cross_scene()
        previous = prior_cross(scene)
        previous.line._center = None
        pool, random = [previous], MsvcRand()
        result = detect_loop_cross(
            scene.segment,
            scene.cloud.select(),
            scene.ground_height,
            None,
            scene.reference_line,
            rng=random,
            loop_crosses=pool,
        )
        self.assertEqual(result.status, "undefined_source_previous_loop_center")
        self.assertIs(pool[0], previous)
        self.assertEqual(random.draw_count, 60)

    def test_missing_height_or_angle_is_reported_at_first_reached_read(self):
        scene = loop_cross_scene()
        random = RecordedRand([])
        result = detect_loop_cross(
            scene.segment, scene.cloud.select(), None, None, scene.reference_line, rng=random
        )
        self.assertEqual(result.status, "undefined_source_ground_height")
        scene.reference_line.angle = None
        result = detect_loop_cross(
            scene.segment, scene.cloud.select([]), None, None, scene.reference_line, rng=random
        )
        self.assertEqual(result.status, "undefined_source_loop_reference_angle")
        self.assertEqual(random.draw_count, 0)

    def test_demo_reports_success_duplicate_and_undefined_sparse_fit(self):
        result = subprocess.run(
            [sys.executable, "-m", "railway_mls", "loop-cross-demo"],
            check=True,
            text=True,
            capture_output=True,
        )
        report = json.loads(result.stdout, parse_constant=lambda value: self.fail(value))
        self.assertEqual(report["status"], "accepted")
        self.assertEqual(report["random_draws"], 120)
        self.assertEqual(report["repeat"]["status"], "duplicate_suppressed")
        self.assertEqual(report["sparse_fit"]["status"], "undefined_source_loop_fit_line")
        self.assertFalse(report["automatic_scheduling_active"])

    def test_detection_preserves_input_regions_labels_intensity_references_and_owns_output_values(
        self,
    ):
        scene = loop_cross_scene()
        regions = scene.segment.neighbor_regions
        first_line = copy(scene.segment.track1.member_line)
        original = [
            a.copy()
            for a in (scene.cloud.class_codes, scene.cloud.segment_ids, scene.cloud.intensity)
        ]
        pool = []
        result = detect_loop_cross(
            scene.segment,
            scene.cloud.select(),
            scene.ground_height,
            None,
            scene.reference_line,
            loop_crosses=pool,
        )
        self.assertIs(scene.segment.neighbor_regions, regions)
        self.assertEqual(scene.segment.track1.member_line, first_line)
        for actual, expected in zip(
            (scene.cloud.class_codes, scene.cloud.segment_ids, scene.cloud.intensity), original
        ):
            assert_array_equal(actual, expected)
        self.assertIsNot(pool[0].line, result.projected_line)
        result.projected_line.a = 99
        self.assertNotEqual(pool[0].line.a, 99)

    def test_validation_undefined_pair_keeps_input_vector_and_reports_first_pair(self):
        for distance, status in (
            (None, "undefined_source_loop_chainage"),
            (float("nan"), "undefined_source_loop_distance_cast"),
            (float("inf"), "undefined_source_loop_distance_cast"),
            (2147483648.0, "undefined_source_loop_distance_cast"),
            (21474837.0, "undefined_source_loop_distance_product"),
        ):
            first, second = (
                LoopCross(distance_from_start=0),
                LoopCross(distance_from_start=distance),
            )
            values = [first, second]
            result = validate_loop_cross(values)
            self.assertEqual(result.status, status)
            self.assertEqual(result.failed_pair, (0, 1))
            self.assertIs(values[0], first)
            self.assertIs(values[1], second)

    def test_singleton_unknown_chainage_is_not_read_and_validation_uses_value_copies(self):
        original = LoopCross()
        values = [original]
        result = validate_loop_cross(values)
        self.assertEqual(result.status, "ok")
        self.assertEqual(result.selected_indices, (0,))
        self.assertIsNot(values[0], original)

    def test_validation_duplicate_records_and_tags_preserve_selected_values(self):
        scene = loop_cross_scene()
        values = [prior_cross(scene, i) for i in range(3)]
        for item, distance in zip(values, (0, 1.5, 3)):
            item.distance_from_start = distance
            item.line.begin = replace(item.line.begin, point_type=7)
        source = values.copy()
        result = validate_loop_cross(values)
        self.assertEqual(result.selected_indices, (1, 0, 2))
        self.assertEqual([value.line.begin.point_type for value in values], [7] * 3)
        self.assertIsNot(values[0].line, source[1].line)

    def test_components_stay_out_of_automatic_pipeline(self):
        scene = tracking_scene(length=2)
        with (
            patch(
                "railway_mls.loop_cross.detect_loop_cross", side_effect=AssertionError("inactive")
            ),
            patch(
                "railway_mls.loop_cross.validate_loop_cross", side_effect=AssertionError("inactive")
            ),
        ):
            result = run_pipeline(
                scene.cloud, scene.seed, options=PipelineOptions(bidirectional=False)
            )
        self.assertTrue(result.completed)
        self.assertEqual(result.engine.layers[0].loop_crosses, [])

    def test_audit_marks_both_bodies_implemented_without_enabling_scheduling(self):
        self.assertEqual(check_source_coverage(Path(__file__).resolve().parents[1]), ())
        self.assertFalse(
            any(
                item["name"] in ("DetectLoopCross", "ValidateLoopCross")
                for item in coverage_summary()["pending_definitions"]
            )
        )


if __name__ == "__main__":
    unittest.main()
