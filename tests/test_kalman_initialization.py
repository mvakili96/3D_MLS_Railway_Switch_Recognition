import json
import subprocess
import sys
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from railway_mls.config import ExtractionConfig
from railway_mls.geometry import direction_mode
from railway_mls.initialization import initial_kalman_states, initialize_tracks
from railway_mls.kalman import KalmanState, kalman_predict, kalman_update
from railway_mls.models import InitialLocation, TrackLayer
from railway_mls.points import Point3D, PointCloud
from railway_mls.sampling import MsvcRand, RecordedRand
from railway_mls.spatial import GridIndex
from railway_mls.synthetic import initialization_scene
from tests.cpp_reference import CppReference


class KalmanTests(unittest.TestCase):
    def test_prediction_is_source_coordinate_dependent_and_has_fixed_process_noise(self):
        covariance = np.array([[4.0, 0.3, 0.2], [0.3, 1.0, -0.1], [0.2, -0.1, 0.25]])
        current = KalmanState(Point3D(10, 20, 3), covariance)
        result = kalman_predict(current, mode=1, angle=0, step_length=2)
        self.assertTrue(result.success)
        assert_allclose(result.state.point.as_array(), [12, 20, 3])
        transition = np.diag([1.2, 1, 1])
        assert_allclose(result.transition, transition)
        assert_array_equal(result.process_noise, np.diag([0.005, 0.005, 0.01]))
        assert_allclose(
            result.state.covariance, transition @ covariance @ transition.T + result.process_noise
        )
        assert_array_equal(current.covariance, covariance)

    def test_modes_absolute_steps_and_unrecognized_mode_else_branch(self):
        current = KalmanState(Point3D(-10, 20, 3))
        for mode, signs in ((1, (1, 1)), (2, (-1, 1)), (3, (-1, -1)), (4, (1, -1)), (0, (1, -1))):
            result = kalman_predict(current, mode=mode, angle=-30, step_length=-2)
            assert_allclose(
                result.state.point.as_array(), [-10 + signs[0] * np.sqrt(3), 20 + signs[1], 3]
            )

    def test_zero_coordinate_preserves_native_division_and_dense_nan_propagation(self):
        result = kalman_predict(KalmanState(Point3D(0, 20, 3)), mode=1, angle=0, step_length=1)
        self.assertEqual(result.status, "nonfinite_prediction")
        self.assertTrue(np.isnan(result.state.point.x))
        self.assertTrue(np.isnan(result.state.covariance[0]).all())
        self.assertTrue(np.isnan(result.state.covariance[:, 0]).all())
        self.assertEqual(result.state.point.y, 20)

    def test_update_uses_sigma_directly_and_source_covariance_formula(self):
        predicted = KalmanState(Point3D(10, 20, 3), np.diag([4, 1, 0.25]))
        result = kalman_update(predicted, Point3D(12, 19, 4), sigma=0.5)
        self.assertTrue(result.success)
        assert_allclose(np.diag(result.gain), [8 / 9, 2 / 3, 1 / 3])
        assert_allclose(result.state.point.as_array(), [10 + 16 / 9, 20 - 2 / 3, 3 + 1 / 3])
        assert_allclose(np.diag(result.state.covariance), [4 / 9, 1 / 3, 1 / 6])
        assert_array_equal(result.measurement_noise, np.eye(3) * 0.5)

    def test_zero_row_inverse_returns_zero_gain_and_preserves_state(self):
        state = KalmanState(Point3D(10, 20, 3), np.zeros((3, 3)))
        result = kalman_update(state, Point3D(100, 200, 30), sigma=0)
        self.assertEqual(result.status, "source_invalid_inverse")
        self.assertFalse(result.inverse_valid)
        self.assertEqual(result.zero_pivots, 0)
        assert_array_equal(result.gain, np.zeros((3, 3)))
        assert_array_equal(result.state.point.as_array(), state.point.as_array())
        assert_array_equal(result.state.covariance, state.covariance)

    def test_dependent_nonzero_rows_use_source_tiny_pivot(self):
        covariance = [[1, 1, 0], [1, 1, 0], [0, 0, 1]]
        result = kalman_update(
            KalmanState(Point3D(10, 20, 3), covariance), Point3D(12, 19, 4), sigma=0
        )
        self.assertEqual(result.status, "source_zero_pivot")
        self.assertTrue(result.inverse_valid)
        self.assertEqual(result.zero_pivots, 2)

    def test_state_owns_readonly_covariance_and_rejects_wrong_shapes(self):
        covariance = np.eye(3)
        state = KalmanState(Point3D(1, 2, 3), covariance)
        covariance[0, 0] = 10
        self.assertEqual(state.covariance[0, 0], 1)
        with self.assertRaises(ValueError):
            state.covariance[0, 0] = 2
        with self.assertRaises(ValueError):
            KalmanState(Point3D(1, 2, 3), np.eye(2))


class InitializationTests(unittest.TestCase):
    def test_complete_initialization_retains_memberships_and_appends_layer_geometry(self):
        scene = initialization_scene()
        grid = GridIndex(scene.cloud)
        random = MsvcRand(1)
        layer = TrackLayer(number=5)
        result = initialize_tracks(scene.seed, grid, config=scene.config, rng=random, layer=layer)
        self.assertTrue(result.success)
        self.assertIs(result.layer, layer)
        self.assertEqual(layer.number, 5)
        self.assertEqual(len(layer.track_segments), 1)
        self.assertEqual(len(layer.updated_boundaries), 8)
        self.assertEqual(random.draw_count, 400)
        self.assertEqual(result.segment.distance_from_start, 0)
        for window, classification, head, height, fit in zip(
            (result.segment.track1, result.segment.track2),
            result.classifications,
            result.heads,
            result.initial_heights,
            result.fits,
        ):
            self.assertIs(window.members, classification.points)
            self.assertEqual(len(head), len(classification.heads))
            self.assertEqual(height, head.xyz[int(len(head) * 0.15), 2])
            self.assertEqual(fit.iterations, 100)
            self.assertEqual(window.center, window.member_line.center)
            assert_allclose([point.z for point in window.vertices], window.member_line.begin.z)
            # The 100-attempt source fit does not assign MemPts.
            self.assertIsNone(window.member_line.members)
        assert_array_equal(scene.cloud.class_codes, 0)
        assert_array_equal(scene.cloud.segment_ids, -1)
        self.assertEqual(result.location.second_point, result.segment.track2.center)
        self.assertEqual(
            result.location.mode,
            direction_mode(
                result.segment.track2.member_line.end, result.segment.track2.member_line.begin
            ),
        )
        states = initial_kalman_states(result.segment)
        self.assertEqual(states[0].point, result.segment.track1.center)
        assert_array_equal(states[0].covariance, np.eye(3))
        self.assertIsNot(states[0].covariance, states[1].covariance)

    def test_repeated_initialization_appends_and_does_not_clear_layer(self):
        scene = initialization_scene()
        layer = TrackLayer()
        grid = GridIndex(scene.cloud)
        random = MsvcRand()
        for _ in range(2):
            self.assertTrue(initialize_tracks(scene.seed, grid, layer=layer, rng=random).success)
        self.assertEqual(len(layer.track_segments), 2)
        self.assertEqual(len(layer.rail1_vector), 2)
        self.assertEqual(len(layer.updated_boundaries), 16)
        self.assertEqual(random.draw_count, 800)

    def test_initialization_ignores_supplied_second_point_and_tracking_ransac_threshold(self):
        scene = initialization_scene()
        seed = InitialLocation(
            scene.seed.point, scene.seed.angle, scene.seed.mode, Point3D(-999, -999, 0)
        )
        first = initialize_tracks(
            seed,
            GridIndex(scene.cloud),
            rng=MsvcRand(1),
            config=ExtractionConfig(max_distance_for_ransac=0),
        )
        second = initialize_tracks(scene.seed, GridIndex(scene.cloud), rng=MsvcRand(1))
        self.assertTrue(first.success)
        self.assertEqual(first.location, second.location)
        self.assertNotEqual(first.location.second_point, seed.second_point)

    def test_empty_window_is_explicit_and_does_not_consume_draws_or_modify_layer(self):
        scene = initialization_scene()
        seed = InitialLocation(Point3D(-100, -100, 0.2), 35, 1)
        random = RecordedRand([])
        layer = TrackLayer()
        result = initialize_tracks(seed, GridIndex(scene.cloud), rng=random, layer=layer)
        self.assertEqual(result.status, "no_head_points")
        self.assertEqual(len(layer.track_segments), 0)
        self.assertEqual(random.draw_count, 0)
        self.assertIsNone(result.location)

    def test_ignored_native_fit_failure_is_explicit_after_both_calls(self):
        scene = initialization_scene()
        random = RecordedRand([0] * 400)
        result = initialize_tracks(scene.seed, GridIndex(scene.cloud), rng=random)
        self.assertEqual(result.status, "undefined_source_initial_line")
        self.assertEqual(len(result.fits), 2)
        self.assertIsNotNone(result.initial_heights)
        self.assertEqual(random.draw_count, 400)
        self.assertEqual(len(result.layer.track_segments), 0)

    def test_grid_configuration_mismatch_is_rejected(self):
        scene = initialization_scene()
        with self.assertRaises(ValueError):
            initialize_tracks(
                scene.seed, GridIndex(scene.cloud), config=ExtractionConfig(grid_size=0.5)
            )


class KalmanInitializationCppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = CppReference()

    @classmethod
    def tearDownClass(cls):
        cls.reference.close()

    def compare_prediction(self, state, mode=1, angle=35, length=1):
        native = self.reference.run(
            "predict", mode, angle, length, *state.point.as_array(), *state.covariance.ravel()
        )
        result = kalman_predict(state, mode=mode, angle=angle, step_length=length)
        assert_allclose(
            [*result.state.point.as_array(), *result.state.covariance.ravel()],
            native,
            rtol=1e-13,
            atol=1e-13,
            equal_nan=True,
        )
        return result

    def compare_update(self, state, sigma=0.03, observation=None):
        observation = observation or Point3D(12, 19, 4)
        native = self.reference.run(
            "update",
            sigma,
            *state.point.as_array(),
            *state.covariance.ravel(),
            *observation.as_array(),
        )
        result = kalman_update(state, observation, sigma)
        assert_allclose(
            [*result.state.point.as_array(), *result.state.covariance.ravel()],
            native[:12],
            rtol=1e-12,
            atol=1e-12,
            equal_nan=True,
        )
        self.assertEqual(result.inverse_valid, bool(native[12]))
        self.assertEqual(result.zero_pivots, int(native[13]))
        return result

    def test_prediction_all_modes_angles_and_step_signs_matches_original(self):
        covariance = [[4, 0.3, 0.2], [0.3, 1, -0.1], [0.2, -0.1, 0.25]]
        for mode in (0, 1, 2, 3, 4, 7):
            for angle in (0, 35, 90, 145, -30):
                for length in (0, 1, -2):
                    self.compare_prediction(
                        KalmanState(Point3D(-10, 20, 3), covariance), mode, angle, length
                    )

    def test_prediction_large_coordinates_and_zero_axes_match_original(self):
        for xyz in ((600000, 4800000, 125), (0, 20, 3), (10, 0, 3), (0, 0, 0)):
            self.compare_prediction(KalmanState(Point3D(*xyz)))

    def test_updates_full_covariance_pivoting_and_direct_sigma_match_original(self):
        for covariance in (
            [[4, 0.3, 0.2], [0.3, 1, -0.1], [0.2, -0.1, 0.25]],
            [[0.001, 0.003, 0.0001], [0.003, 4, 0.02], [0.0001, 0.02, 0.3]],
            [[4, 2, 3], [1, 5, 0.5], [3, 1, 6]],
        ):
            for sigma in (0, 0.03, 2, -2):
                self.compare_update(KalmanState(Point3D(10, 20, 3), covariance), sigma)

    def test_zero_rows_and_dependent_rows_follow_original_inverse_rules(self):
        for covariance in (
            np.zeros((3, 3)),
            np.diag([1, 0, 1]),
            [[1, 1, 0], [1, 1, 0], [0, 0, 1]],
            [[1, 1, 1]] * 3,
        ):
            self.compare_update(KalmanState(Point3D(10, 20, 3), covariance), sigma=0)

    def test_nonfinite_update_inputs_match_original_dense_arithmetic(self):
        self.compare_update(KalmanState(Point3D(10, 20, 3)), sigma=float("nan"))
        self.compare_update(KalmanState(Point3D(float("nan"), 20, 3)))
        self.compare_update(
            KalmanState(Point3D(10, 20, 3)), observation=Point3D(12, float("inf"), 4)
        )

    def test_repeated_predict_update_recurrence_matches_original(self):
        state = KalmanState(Point3D(1000, 2000, 0.2))
        for step in range(12):
            predicted = self.compare_prediction(state, angle=35 + step * 0.2)
            observation = predicted.state.point + Point3D(0.01, -0.02, 0.001)
            state = self.compare_update(predicted.state, sigma=0.01, observation=observation).state

    def compare_initialization(self, scene, seed_value=1):
        grid = GridIndex(scene.cloud, scene.config.grid_size)
        draws = MsvcRand(seed_value)
        inputs = [
            scene.seed.mode,
            scene.seed.angle,
            *scene.seed.point.as_array(),
            scene.config.track_gauge,
            scene.config.track_window_length,
            scene.config.track_window_width,
            scene.config.grid_size,
            *grid.bounds.minimum.as_array(),
            *grid.bounds.maximum.as_array(),
            len(scene.cloud),
        ]
        for i, xyz in enumerate(scene.cloud.xyz):
            inputs.extend([*xyz, int(scene.cloud.class_codes[i]), int(scene.cloud.segment_ids[i])])
        inputs.extend([400, *(draws.rand() for _ in range(400))])
        native = self.reference.run("initialize", *inputs)
        random = MsvcRand(seed_value)
        result = initialize_tracks(scene.seed, grid, config=scene.config, rng=random)
        self.assertEqual(bool(native[0]), result.source_return)
        expected = [int(result.source_return), 1, 0, len(result.layer.track_segments)]
        if result.source_return:
            segment, location = result.segment, result.location
            expected.extend(
                [
                    *result.initial_heights,
                    *location.point.as_array(),
                    *location.second_point.as_array(),
                    location.angle,
                    location.mode,
                    *segment.center.as_array(),
                    segment.gauge,
                    0,
                ]
            )
            for window in (segment.track1, segment.track2):
                line = window.member_line
                expected.extend(
                    [
                        *window.center.as_array(),
                        window.orientation,
                        *(value for point in window.vertices for value in point.as_array()),
                        len(window.members),
                        *window.members.indices,
                        *line.begin.as_array(),
                        *line.end.as_array(),
                        *line.center.as_array(),
                        line.angle,
                        line.a,
                        line.b,
                        line.c,
                        0,
                    ]
                )
        for points in (
            result.layer.rail1_vector,
            result.layer.rail2_vector,
            result.layer.center_vector,
        ):
            expected.extend(
                [len(points), *(value for point in points for value in point.as_array())]
            )
        expected.append(len(result.layer.updated_boundaries))
        for line in result.layer.updated_boundaries:
            expected.extend([*line.begin.as_array(), *line.end.as_array()])
        for code, segment_id in zip(scene.cloud.class_codes, scene.cloud.segment_ids):
            expected.extend([code, segment_id])
        expected.append(random.draw_count)
        assert_allclose(expected, native, rtol=1e-11, atol=1e-10, equal_nan=True)
        return result

    def test_complete_initialization_matches_original_in_all_modes(self):
        for mode in (0, 1, 2, 3, 4):
            with self.subTest(mode=mode):
                self.compare_initialization(initialization_scene(mode=mode))

    def test_initialization_modified_dimensions_gauge_and_seed_match_original(self):
        config = ExtractionConfig(
            track_gauge=1.435, track_window_length=1.3, track_window_width=0.5
        )
        scene = initialization_scene(angle=57, config=config)
        scene.cloud.class_codes[:] = 9
        scene.cloud.segment_ids[:] = 7
        self.compare_initialization(scene, seed_value=42)
        assert_array_equal(scene.cloud.class_codes, 9)
        assert_array_equal(scene.cloud.segment_ids, 7)

    def test_axis_coefficient_quirk_is_preserved_during_complete_initialization(self):
        self.compare_initialization(initialization_scene(angle=0))

    def test_no_heads_returns_false_without_layer_output_or_draws(self):
        scene = initialization_scene()
        xyz = scene.cloud.xyz.copy()
        xyz[:, 2] = np.where(np.arange(len(xyz)) % 2, 1000.0, 0.0)
        from railway_mls.synthetic import SyntheticScene

        result = self.compare_initialization(
            SyntheticScene(PointCloud(xyz), scene.seed, scene.config)
        )
        self.assertEqual(result.status, "no_head_points")

    def test_empty_windows_match_defined_native_no_head_return(self):
        from railway_mls.synthetic import SyntheticScene

        scene = initialization_scene()
        seed = InitialLocation(Point3D(-100, -100, 0.2), 35, 1)
        result = self.compare_initialization(SyntheticScene(scene.cloud, seed, scene.config))
        self.assertEqual(result.status, "no_head_points")
        self.assertEqual(result.classifications[0].fit.iterations, 101)
        native = self.reference.run("gmm", 0, 3)
        self.assertTrue(np.isnan(native[:9]).all())
        self.assertEqual(native[9:], [0, 0, 0])

    def test_nonfinite_initialization_preserves_native_true_return_and_layer_appends(self):
        from railway_mls.synthetic import SyntheticScene

        points = []
        for y in (1.0, 2.5):
            for i in range(-4, 5):
                for z in (0.0, 0.1, 0.2):
                    points.append((1 + i * 0.1, y, z + len(points) * 1e-7))
        scene = SyntheticScene(
            PointCloud(points), InitialLocation(Point3D(1, 1, 0.2), 0, 1), ExtractionConfig()
        )
        result = self.compare_initialization(scene)
        self.assertEqual(result.status, "nonfinite_initial_geometry")
        self.assertFalse(result.success)
        self.assertTrue(result.source_return)
        self.assertEqual(len(result.layer.track_segments), 1)


class InitializationDemoTests(unittest.TestCase):
    def test_demo_initializes_raw_cloud_and_runs_one_kalman_component_step(self):
        completed = subprocess.run(
            [sys.executable, "-m", "railway_mls", "initialization-demo"],
            capture_output=True,
            text=True,
            check=True,
        )
        payload = json.loads(completed.stdout)
        self.assertEqual(payload["status"], "ok")
        self.assertEqual(payload["saved_segments"], 1)
        self.assertEqual(payload["updated_boundaries"], 8)
        self.assertEqual(payload["random_draws"], 400)
        self.assertFalse(payload["tracking_implemented"])
        self.assertEqual(
            [step["update_status"] for step in payload["kalman_example"]], ["ok", "ok"]
        )


if __name__ == "__main__":
    unittest.main()
