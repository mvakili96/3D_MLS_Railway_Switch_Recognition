from dataclasses import replace
import json
import subprocess
import sys
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls import (
    ExtractionConfig,
    GridIndex,
    MsvcRand,
    PlaneParameters,
    Point3D,
    PointCloud,
    RecordedRand,
    SequentialTracker,
    SwitchBox,
    ToePoint,
    TrackLayer,
    TrackingEngine,
    detect_switch_boxes,
    fit_plane_ransac,
    initialize_tracks,
    plane_distances,
    plane_from_eigen,
    plane_from_three_points,
    switch_box_candidates,
)
from railway_mls.geometry import points_in_convex_polygon
from railway_mls.synthetic import switch_box_scene, tracking_scene
from tests.cpp_reference import CppReference
from tests.test_toes import toe_state


def selection_state(points):
    return [len(points), *points.indices]


def box_state(box, candidate=False):
    result = [*box.center.as_array(), box.angle, int(candidate)]
    if candidate:
        result.append(box.reference_height)
    return [*result, len(box.vertices), *(v for p in box.vertices for v in p.as_array())]


def prior_box():
    box = SwitchBox(Point3D(9, 8, 7), 12.0)
    box.generate_region(0.7)
    return box


def integrated_engine():
    track, switches = tracking_scene(length=2), switch_box_scene()
    shift = Point3D(100, 100, 0)
    cloud = PointCloud(np.vstack((track.cloud.xyz, switches.cloud.xyz + shift.as_array())))
    engine = TrackingEngine(GridIndex(cloud))
    engine.initialize(track.seed)
    toe = switches.toes[0]
    engine.toe_points.append(
        replace(
            toe,
            center=toe.center + shift,
            left=toe.left + shift,
            right=toe.right + shift,
            track_type=6,
        )
    )
    return track, engine


class PlaneTests(unittest.TestCase):
    def test_three_point_plane_uses_unnormalized_negative_cross_product(self):
        plane = plane_from_three_points(Point3D(0, 0, 2), Point3D(2, 0, 2), Point3D(0, 3, 2))
        assert_array_equal(plane.as_array(), [0, 0, -6, 12])
        zero = plane_from_three_points(Point3D(1, 2, 3), Point3D(1, 2, 3), Point3D(1, 2, 3))
        assert_array_equal(zero.as_array(), 0)

    def test_eigen_empty_singleton_and_collinear_are_defined(self):
        empty = PointCloud(np.empty((0, 3)))
        assert_array_equal(plane_from_eigen(empty.select()).plane.as_array(), [1, 0, 0, 0])
        one = PointCloud([(1, 2, 3)])
        assert_array_equal(plane_from_eigen(one.select()).plane.as_array(), [1, 0, 0, -1])
        line = PointCloud([(1, 2, 3), (2, 4, 6), (3, 6, 9)])
        fitted = plane_from_eigen(line.select())
        assert_allclose(plane_distances(line.select(), fitted.plane), 0, atol=1e-14)

    def test_zero_normal_distance_keeps_ieee_nan(self):
        cloud = PointCloud([(1, 2, 3)])
        self.assertTrue(np.isnan(plane_distances(cloud.select(), PlaneParameters(0, 0, 0, 0))[0]))

    def test_short_fit_preserves_old_plane_and_support_without_draws(self):
        cloud = PointCloud([(1, 2, 3), (2, 3, 4), (3, 4, 5), (4, 5, 6)])
        plane, support = PlaneParameters(0, 0, 1, -3), cloud.select([3, 1, 1])
        random = RecordedRand([])
        fitted = fit_plane_ransac(cloud.select([0, 1, 2]), rng=random, plane=plane, support=support)
        self.assertIs(fitted.plane, plane)
        self.assertIs(fitted.support, support)
        self.assertEqual(fitted.status, "too_few_points")
        self.assertFalse(fitted.support_cleared)
        self.assertTrue(fitted.reference_comparable)
        self.assertEqual(random.draw_count, 0)

    def test_no_winner_clears_support_but_retains_old_plane(self):
        cloud = PointCloud([(1000 + i, 2000, 0.05) for i in range(4)])
        random = RecordedRand([0] * 300)
        old = PlaneParameters(0, 0, 1, -0.05)
        fitted = fit_plane_ransac(cloud.select(), rng=random, plane=old, support=cloud.select([1]))
        self.assertIs(fitted.plane, old)
        self.assertEqual(len(fitted.support), 0)
        self.assertEqual(fitted.status, "no_winning_model")
        self.assertFalse(fitted.parameters_written)
        self.assertEqual(random.draw_count, 300)

    def test_duplicate_samples_still_refit_empty_support_to_source_x_plane(self):
        cloud = PointCloud([(0, i, 0.05) for i in range(4)])
        random = RecordedRand([0] * 300)
        fitted = fit_plane_ransac(cloud.select(), rng=random)
        self.assertEqual(fitted.sampled_counts, (0,) * 100)
        self.assertEqual(fitted.refined_counts, (4,) * 100)
        self.assertEqual(fitted.best_iteration, 0)
        assert_array_equal(fitted.plane.as_array(), [1, 0, 0, 0])
        self.assertEqual(random.draw_count, 300)

    def test_nonfinite_scatter_reports_failure_without_eigensolver_hang(self):
        cloud = PointCloud([(1e308, 1, 2), (-1e308, 2, 3)])
        fitted = plane_from_eigen(cloud.select())
        self.assertEqual(fitted.status, "nonfinite_plane_scatter")
        self.assertIsNone(fitted.plane)


class SwitchBoxTests(unittest.TestCase):
    def test_axis_candidate_centers_expansion_height_and_order(self):
        toe = ToePoint(left=Point3D(10, 11.5, 0.2), right=Point3D(10, 10, 0.2))
        candidates = switch_box_candidates([toe])
        self.assertEqual(candidates.status, "ok")
        self.assertEqual([c.side for c in candidates.candidates], ["begin", "end"])
        assert_allclose([c.region.center.y for c in candidates.candidates], [8.8525, 12.6475])
        for candidate in candidates.candidates:
            self.assertEqual(candidate.region.angle, 90)
            self.assertEqual(candidate.region.reference_height, 0.2)
            self.assertEqual(len(candidate.region.vertices), 4)
            self.assertTrue(all(p.z == 0 for p in candidate.region.vertices))

    def test_used_skips_and_stored_toe_angle_mode_center_type_are_not_required(self):
        toe = ToePoint(left=Point3D(10, 11.5, 0.2), right=Point3D(10, 10, 0.2), track_type=999)
        result = switch_box_candidates([ToePoint(used=True), toe])
        self.assertEqual(result.skipped_toes, (0,))
        self.assertEqual([c.toe_index for c in result.candidates], [1, 1])

    def test_convex_predicate_excludes_rectangle_edges_but_accepts_all_zero_tags(self):
        vertices = tuple(Point3D(x, y, 0) for x, y in ((0, 0), (1, 0), (1, 1), (0, 1)))
        xy = np.array([(0.5, 0.5), (0, 0.5), (0, 0), (2, 2), (1e-12, 0.5), (1e-8, 0.5)])
        assert_array_equal(
            points_in_convex_polygon(vertices, xy), [True, False, False, False, False, True]
        )
        assert_array_equal(points_in_convex_polygon((Point3D(0, 0, 0),) * 4, xy), True)

    def test_missing_later_toe_geometry_aborts_before_evaluating_earlier_candidates(self):
        scene = switch_box_scene()
        random = RecordedRand([])
        result = detect_switch_boxes([*scene.toes, ToePoint()], GridIndex(scene.cloud), rng=random)
        self.assertEqual(result.status, "undefined_source_switch_box_toe")
        self.assertEqual(len(result.candidates.candidates), 2)
        self.assertEqual(result.entries, ())
        self.assertEqual(random.draw_count, 0)
        assert_array_equal(scene.cloud.class_codes, 9)
        assert_array_equal(scene.cloud.segment_ids, 7)

    def test_empty_candidate_fits_are_explicit_and_produce_no_fabricated_boxes(self):
        scene = switch_box_scene()
        cloud = PointCloud([(1000, 2000, 0), (1001, 2001, 0)])
        random = RecordedRand([])
        result = detect_switch_boxes(scene.toes, GridIndex(cloud), rng=random)
        self.assertEqual(result.status, "partial_undefined_source")
        self.assertEqual(
            [e.status for e in result.entries], ["undefined_source_switch_box_plane"] * 2
        )
        self.assertEqual(result.new_boxes, ())
        self.assertFalse(result.reference_comparable)

    def test_dense_patches_accept_with_strict_band_and_source_constants(self):
        scene = switch_box_scene(
            config=ExtractionConfig(max_distance_for_ransac=1e-10, grouping_distance=1e-10)
        )
        random, layer = MsvcRand(), TrackLayer()
        result = detect_switch_boxes(
            scene.toes, GridIndex(scene.cloud), config=scene.config, rng=random, layer=layer
        )
        self.assertEqual(result.status, "completed")
        self.assertTrue(result.reference_comparable)
        self.assertEqual(len(result.new_boxes), 2)
        self.assertEqual(random.draw_count, 1200)
        self.assertEqual(np.count_nonzero(scene.cloud.class_codes == 5), 2312)
        for entry in result.entries:
            self.assertEqual(len(entry.region_points), 1158)
            self.assertEqual(len(entry.height_points), 1156)
            self.assertEqual(entry.status, "accepted")
            self.assertIsNone(entry.box.reference_height)
            self.assertTrue(all(p.z == entry.box.center.z for p in entry.box.vertices))
        self.assertEqual(len(layer.switch_boxes), 2)
        self.assertIsNot(layer.switch_boxes[0], result.new_boxes[0])

    def test_tilt_rejection_preserves_group_segment_labels(self):
        scene = switch_box_scene(slope=0.06)
        result = detect_switch_boxes(scene.toes, GridIndex(scene.cloud))
        self.assertEqual([e.status for e in result.entries], ["not_horizontal"] * 2)
        self.assertEqual(result.new_boxes, ())
        self.assertEqual(np.count_nonzero(scene.cloud.segment_ids == 0), 2312)
        assert_array_equal(scene.cloud.class_codes, 9)

    def test_second_short_fit_reuses_plane_and_support_but_labels_only_group(self):
        scene = switch_box_scene(points_per_side=2, config=ExtractionConfig(skip_rate=1001))
        random = MsvcRand()
        result = detect_switch_boxes(
            scene.toes, GridIndex(scene.cloud), config=scene.config, rng=random
        )
        self.assertEqual(len(result.new_boxes), 2)
        self.assertEqual(random.draw_count, 600)
        for entry in result.entries:
            self.assertEqual(len(entry.grouping.selected), 1)
            self.assertEqual(entry.second_fit.status, "too_few_points")
            self.assertEqual(len(entry.second_fit.support), 4)
            self.assertFalse(entry.second_fit.parameters_written)
            assert_allclose(entry.box.center.as_array(), entry.grouping.selected.xyz[0])
        self.assertEqual(np.count_nonzero(scene.cloud.class_codes == 5), 2)

    def test_repeated_appends_keep_old_boxes_and_do_not_change_toe_used(self):
        scene = switch_box_scene()
        pool, layer, random = [prior_box()], TrackLayer(), MsvcRand()
        for _ in range(2):
            result = detect_switch_boxes(
                scene.toes, GridIndex(scene.cloud), boxes=pool, layer=layer, rng=random
            )
            self.assertEqual(len(result.new_boxes), 2)
        self.assertEqual((len(pool), len(layer.switch_boxes), random.draw_count), (5, 4, 2400))
        self.assertFalse(scene.toes[0].used)

    def test_main_track_gate_and_invalid_configuration_do_not_mutate_labels(self):
        scene = switch_box_scene()
        grid, random = GridIndex(scene.cloud), RecordedRand([])
        result = detect_switch_boxes(
            [ToePoint()], grid, config=ExtractionConfig(skip_rate=0), rng=random, main_track=False
        )
        self.assertEqual(result.status, "main_track_disabled")
        self.assertTrue(result.reference_comparable)
        with self.assertRaises(ValueError):
            detect_switch_boxes(scene.toes, grid, config=ExtractionConfig(skip_rate=0))
        with self.assertRaises(ValueError):
            detect_switch_boxes(scene.toes, grid, config=ExtractionConfig(grid_size=2))
        assert_array_equal(scene.cloud.segment_ids, 7)

    def test_nonfinite_candidate_stops_before_grid_conversion(self):
        scene = switch_box_scene()
        toe = replace(scene.toes[0], right=Point3D(float("nan"), 2000, 0.2))
        result = detect_switch_boxes([toe], GridIndex(scene.cloud), rng=RecordedRand([]))
        self.assertEqual(result.status, "nonfinite_switch_box_candidate")
        self.assertEqual(result.new_boxes, ())


class SwitchBoxCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def test_three_point_coefficients_and_degenerate_samples_match_original(self):
        for points in (
            ((0, 0, 2), (2, 0, 2), (0, 3, 2)),
            ((1, 2, 3), (2, 4, 6), (3, 6, 9)),
            ((1000, 2000, 0.2), (1001, 2000, 0.201), (1000, 2001, 0.202)),
        ):
            native = self.reference.run("plane_three", *(v for p in points for v in p))
            actual = plane_from_three_points(*(Point3D(*p) for p in points))
            assert_allclose(actual.as_array(), native, atol=1e-12, rtol=1e-12)

    def compare_eigen(self, xyz, indices=None):
        cloud = PointCloud(np.array(xyz, dtype=float).reshape(-1, 3))
        points = cloud.select(indices)
        native = self.reference.run(
            "plane_eigen", len(cloud), *cloud.xyz.ravel(), *selection_state(points)
        )
        actual = plane_from_eigen(points)
        assert_allclose(actual.plane.as_array(), native, rtol=2e-12, atol=2e-10)
        return actual

    def test_empty_singleton_repeated_and_collinear_eigen_bases_match_bundled_jama(self):
        for xyz in (
            [],
            [(1, 2, 3)],
            [(1, 2, 3)] * 5,
            [(1, 2, 3), (2, 4, 6), (3, 6, 9)],
            [(0, 0, 1), (1, 0, 1), (0, 1, 1), (1, 1, 1)],
        ):
            self.compare_eigen(xyz)

    def test_oblique_large_origin_and_duplicate_identity_refits_match_original(self):
        random = np.random.default_rng(12)
        xyz = random.normal(size=(40, 3))
        self.compare_eigen(xyz)
        xyz[:, 2] = 0.001 * xyz[:, 0] + 0.002 * xyz[:, 1] + 0.0001 * np.sin(np.arange(40))
        xyz += [1e6, 2e6, 3]
        self.compare_eigen(xyz, [*range(40), 1, 1, 3])

    def compare_ransac(
        self, cloud, *, indices=None, stream=None, plane=None, support=None, threshold=0.008
    ):
        points = cloud.select(indices)
        if stream is None:
            source = MsvcRand(1)
            stream = [source.rand() for _ in range(300)]
        previous = cloud.select([]) if support is None else support
        native = self.reference.run(
            "plane_ransac",
            len(cloud),
            *cloud.xyz.ravel(),
            *selection_state(points),
            threshold,
            int(plane is not None),
            *(plane.as_array() if plane is not None else []),
            *selection_state(previous),
            len(stream),
            *stream,
        )
        random = RecordedRand(stream)
        fitted = fit_plane_ransac(points, threshold, rng=random, plane=plane, support=previous)
        expected = [int(fitted.plane is not None)]
        if fitted.plane is not None:
            expected.extend(fitted.plane.as_array())
        expected.extend(
            [
                *selection_state(fitted.support),
                int(fitted.parameters_written),
                fitted.best_iteration if fitted.best_iteration is not None else -1,
                len(fitted.sampled_counts),
                *fitted.sampled_counts,
                len(fitted.refined_counts),
                *fitted.refined_counts,
                random.draw_count,
            ]
        )
        assert_allclose(expected, native, rtol=2e-12, atol=2e-10)
        return fitted

    def test_plane_trials_refit_support_order_and_draw_counts_match_original(self):
        cloud = PointCloud(
            [
                (1000 + i % 10 * 0.01, 2000 + i // 10 * 0.01, 0.25 + 0.0004 * np.sin(i))
                for i in range(80)
            ]
            + [(1001, 2001, 1)]
        )
        random = MsvcRand(7)
        self.compare_ransac(cloud, stream=[random.rand() for _ in range(300)])

    def test_duplicate_draws_and_missing_outputs_are_compared_without_undefined_reads(self):
        for origin in (0, 1000):
            cloud = PointCloud([(origin, i, 0.05) for i in range(4)])
            self.compare_ransac(cloud, stream=[0] * 300)

    def test_short_fit_and_no_winner_retain_source_outputs(self):
        cloud = PointCloud([(1000 + i, 2000, 0.05) for i in range(4)])
        plane = PlaneParameters(0, 0, 1, -0.05)
        self.compare_ransac(
            cloud, indices=[0, 1, 2], stream=[], plane=plane, support=cloud.select([3, 1, 1])
        )
        self.compare_ransac(cloud, stream=[0] * 300, plane=plane, support=cloud.select([1]))
        self.compare_ransac(cloud, stream=[0] * 300, plane=plane, threshold=0)

    def test_convex_polygon_winding_boundaries_precision_and_degeneracy_match_source(self):
        vertices = tuple(Point3D(x, y, 0) for x, y in ((0, 0), (1, 0), (1, 1), (0, 1)))
        for polygon in (vertices, vertices[::-1], (Point3D(0, 0, 0),) * 4):
            for x, y in ((0.5, 0.5), (0, 0.5), (0, 0), (2, 2), (1e-12, 0.5), (1e-8, 0.5)):
                native = self.reference.run(
                    "convex_polygon", len(polygon), *(v for p in polygon for v in (p.x, p.y)), x, y
                )
                self.assertEqual(bool(native[0]), points_in_convex_polygon(polygon, [[x, y]])[0])

    def test_candidate_expansion_regions_heights_and_used_gate_match_original(self):
        for angle in (0, 35, 90, 145, 215, 305):
            toe = switch_box_scene(angle=angle, points_per_side=2).toes[0]
            toes = [
                replace(toe, used=True),
                replace(toe, left=replace(toe.left, z=0.4), track_type=999),
            ]
            native = self.reference.run(
                "switch_candidates", len(toes), *(v for t in toes for v in toe_state(t))
            )
            candidates = switch_box_candidates(toes)
            expected = [
                len(candidates.candidates),
                *(v for c in candidates.candidates for v in box_state(c.region, True)),
            ]
            assert_allclose(expected, native, atol=2e-10, rtol=1e-12)

    def compare_boxes(self, scene, *, old=False):
        random, grid = MsvcRand(), GridIndex(scene.cloud, scene.config.grid_size)
        pool = [prior_box()] if old else []
        layer = TrackLayer(switch_boxes=[prior_box()] if old else [])
        # Copy input labels BEFORE running Python, for the original source process.
        values = [
            scene.config.grid_size,
            scene.config.skip_rate,
            *grid.bounds.minimum.as_array(),
            *grid.bounds.maximum.as_array(),
            len(scene.cloud),
            *(
                v
                for xyz, code, seg in zip(
                    scene.cloud.xyz, scene.cloud.class_codes, scene.cloud.segment_ids
                )
                for v in (*xyz, code, seg)
            ),
            len(scene.toes),
            *(v for toe in scene.toes for v in toe_state(toe)),
        ]
        for boxes in (pool, layer.switch_boxes):
            values.append(len(boxes))
            for box in boxes:
                values.extend(
                    [
                        *box.center.as_array(),
                        box.angle,
                        len(box.vertices),
                        *(v for p in box.vertices for v in p.as_array()),
                    ]
                )
        result = detect_switch_boxes(
            scene.toes, grid, config=scene.config, rng=random, boxes=pool, layer=layer
        )
        # The native complete function is an oracle ONLY for initialized plane reads.
        self.assertTrue(result.reference_comparable)
        raw = MsvcRand()
        stream = [raw.rand() for _ in range(random.draw_count)]
        native = self.reference.run("switch_boxes", *values, len(stream), *stream)
        expected = [
            len(result.candidates.candidates),
            *(v for c in result.candidates.candidates for v in box_state(c.region, True)),
            len(result.entries),
        ]
        for entry in result.entries:
            for points in (
                entry.region_points,
                entry.height_points,
                entry.grouping.selected,
                entry.second_fit.support,
            ):
                expected.extend(selection_state(points))
            expected.extend(entry.second_fit.plane.as_array())
        for boxes in (pool, layer.switch_boxes):
            expected.extend([len(boxes), *(v for b in boxes for v in box_state(b))])
        expected.extend(
            v for pair in zip(scene.cloud.class_codes, scene.cloud.segment_ids) for v in pair
        )
        expected.append(random.draw_count)
        assert_allclose(expected, native, rtol=2e-12, atol=2e-9)
        return result

    def test_full_detection_labels_group_identity_centers_and_vertices_match_original(self):
        result = self.compare_boxes(switch_box_scene())
        self.assertEqual(len(result.new_boxes), 2)

    def test_tilt_count_threshold_and_skip_rate_rejections_match_original(self):
        result = self.compare_boxes(switch_box_scene(slope=0.06))
        self.assertEqual([e.status for e in result.entries], ["not_horizontal"] * 2)
        result = self.compare_boxes(switch_box_scene(points_per_side=31))
        self.assertEqual([e.status for e in result.entries], ["too_few_group_points"] * 2)
        result = self.compare_boxes(
            switch_box_scene(points_per_side=31, config=ExtractionConfig(skip_rate=2))
        )
        self.assertEqual(len(result.new_boxes), 2)

    def test_short_second_fit_reuse_and_pre_refit_group_labels_match_original(self):
        result = self.compare_boxes(
            switch_box_scene(points_per_side=2, config=ExtractionConfig(skip_rate=1001))
        )
        self.assertEqual(len(result.new_boxes), 2)
        self.assertEqual(result.entries[0].second_fit.status, "too_few_points")

    def test_existing_pool_layer_boxes_and_used_toes_match_original(self):
        scene = switch_box_scene()
        scene = replace(scene, toes=(*scene.toes, replace(scene.toes[0], used=True)))
        result = self.compare_boxes(scene, old=True)
        self.assertEqual(len(result.boxes), 3)

    def test_exact_minimum_group_size_gate_matches_original(self):
        for count in (999, 1000):
            scene = switch_box_scene(points_per_side=32)
            rows = np.concatenate(
                (np.arange(count), [1024, 1025], 1026 + np.arange(count), [2050, 2051])
            )
            scene = replace(scene, cloud=PointCloud(scene.cloud.xyz[rows]))
            result = self.compare_boxes(scene)
            self.assertEqual(len(result.new_boxes), 2 if count == 1000 else 0)


class SwitchBoxIntegrationTests(unittest.TestCase):
    def test_engine_postpass_appends_global_and_main_layer_boxes_before_branch_loop(self):
        _, engine = integrated_engine()
        result = engine.run_pass()
        self.assertEqual(result.status, "completed")
        self.assertEqual(len(result.main.result.switch_box_detection.new_boxes), 2)
        self.assertEqual(len(engine.switch_boxes), 2)
        self.assertEqual(len(engine.layers[0].switch_boxes), 2)
        self.assertEqual(result.branches[0].status, "skipped_type")
        self.assertEqual(engine.layers[1].switch_boxes, [])
        self.assertGreater(np.count_nonzero(engine.grid.cloud.class_codes == 5), 2000)

    def test_reset_retains_global_boxes_and_repeated_main_calls_append_duplicates(self):
        track, engine = integrated_engine()
        engine.run_pass()
        engine.initialize(track.seed)
        self.assertEqual(len(engine.switch_boxes), 2)
        self.assertEqual(engine.layers[0].switch_boxes, [])
        engine.run_pass()
        self.assertEqual(len(engine.switch_boxes), 4)
        self.assertEqual(len(engine.layers[0].switch_boxes), 2)

    def test_diagnostic_limit_and_branch_main_gate_defer_switch_boxes(self):
        _, engine = integrated_engine()
        result = engine.run_pass(max_steps=1)
        self.assertIsNone(result.main.result.switch_box_detection)
        self.assertEqual(engine.switch_boxes, [])
        scene = tracking_scene(length=2)
        grid, random = GridIndex(scene.cloud), MsvcRand()
        initialized = initialize_tracks(scene.seed, grid, rng=random)
        tracker = SequentialTracker(
            initialized.layer,
            grid,
            initialized.initial_heights,
            rng=random,
            main_track=False,
            toe_points=[ToePoint()],
        )
        result = tracker.run()
        self.assertEqual(result.switch_box_detection.status, "main_track_disabled")
        self.assertEqual(result.switch_box_detection.entries, ())

    def test_cli_reports_actual_two_box_results_and_random_draws(self):
        completed = subprocess.run(
            [sys.executable, "-m", "railway_mls", "switch-box-demo"],
            text=True,
            capture_output=True,
            check=True,
        )
        result = json.loads(completed.stdout)
        self.assertEqual(result["status"], "completed")
        self.assertEqual(
            (result["detected_count"], result["random_draws"], result["class_5_points"]),
            (2, 1200, 2312),
        )
        self.assertTrue(result["reference_comparable"])


if __name__ == "__main__":
    unittest.main()
