import unittest
from types import SimpleNamespace

import numpy as np

from railway_mls.continuations import ContinuationTracker, fitted_window_layer, pair_rail_lines
from railway_mls.geometry import Line3D
from railway_mls.kalman import KalmanState
from railway_mls.models import TrackSegment, TrackWindow
from railway_mls.points import Point3D, PointCloud
from railway_mls.sampling import MsvcRand
from railway_mls.spatial import GridIndex
from railway_mls.synthetic import tracking_scene


class RailPairTests(unittest.TestCase):
    def setUp(self):
        self.set_scene(90.0)

    def set_scene(self, angle):
        self.scene = tracking_scene(length=7, angle=angle)
        self.config = self.scene.config
        self.gauge = self.config.track_gauge
        self.origin = self.scene.seed.point.as_array()
        radians = np.deg2rad(angle)
        self.forward = np.array([np.cos(radians), np.sin(radians), 0.0])
        self.across = np.array([-self.forward[1], self.forward[0], 0.0])

    def point(self, station, offset, height_offset=0.0):
        return Point3D(*(self.origin + station * self.forward + offset * self.across
                         + [0.0, 0.0, height_offset]))

    def line(self, offset, *, start=0.5, end=1.5, height_offset=0.0, cloud=None):
        owner = self.scene.cloud if cloud is None else cloud
        return Line3D(self.point(start, offset, height_offset), self.point(end, offset, height_offset),
                      members=owner.select([0]))

    def predictions(self, offset):
        return self.point(1, offset), self.point(1, self.gauge + offset)

    def pairs(self, options):
        return pair_rail_lines(
            options, (self.point(0, 0), self.point(0, self.gauge)),
            self.predictions(0), self.config,
        )

    def test_pairing_uses_common_station_in_travel_direction_and_preserves_heights(self):
        options = (
            (self.line(0, start=1.5, end=0.5), self.line(0.18)),
            (self.line(self.gauge, start=0.8, height_offset=0.04), self.line(self.gauge + 0.18)),
        )
        first, second = self.pairs(options)
        self.assertEqual((first.indices, second.indices), ((0, 0), (1, 1)))
        self.assertEqual(first.begins[0].y, first.begins[1].y)
        self.assertLess(first.begins[0].y, first.ends[0].y)
        self.assertAlmostEqual(first.shared_length, 0.7)
        self.assertAlmostEqual(first.gauge, self.gauge)
        self.assertAlmostEqual(first.seed_layer(self.config).rail2_vector[0].z, 0.24)
        self.assertAlmostEqual(second.continuity_error, 0.18)

    def test_pairing_rejects_disjoint_wrong_side_crossing_or_unsupported_lines(self):
        unsupported = self.line(self.gauge)
        unsupported.members = None
        other_lines = (
            self.line(self.gauge, start=1.6, end=2.6),
            self.line(-self.gauge),
            Line3D(self.point(0.5, 1.15), self.point(1.5, 1.85),
                   members=self.scene.cloud.select([0])),
            unsupported,
        )
        for invalid in other_lines:
            with self.subTest(line=invalid):
                self.assertEqual(self.pairs(((self.line(0),), (invalid,))), ())

    def test_each_tracker_prefers_the_pair_aligned_with_its_own_prediction(self):
        options = (
            (self.line(0.18), self.line(0)),
            (self.line(self.gauge), self.line(self.gauge + 0.18)),
        )
        models = tuple(SimpleNamespace(lines=lines) for lines in options)
        grid = GridIndex(self.scene.cloud)
        pairs = self.pairs(options)
        self.assertEqual([pair.indices for pair in pairs], [(0, 1), (1, 0)])
        for pair, offset in zip(pairs, (0.18, 0)):
            tracker = ContinuationTracker(pair.seed_layer(self.config), grid, (0.2, 0.2))
            predicted = tuple(self.point(2, offset + rail * self.gauge) for rail in range(2))
            attempt = SimpleNamespace(predictions=tuple(
                SimpleNamespace(state=KalmanState(point)) for point in predicted
            ))
            selected = tracker._select_turnout_observation(models, attempt, 1)
            self.assertEqual(selected.selected_indices, pair.indices)

    def test_selected_branch_seed_retains_fitted_heading_and_detaches_memberships(self):
        first = self.line(0)
        second = self.line(self.gauge, height_offset=0.04)
        source = TrackSegment(
            TrackWindow(first.end, 84.0, members=first.members, member_line=first),
            TrackWindow(second.end, 85.0, members=second.members, member_line=second),
        )
        seed = fitted_window_layer(source).track_segments[0]
        self.assertEqual((seed.track1.center, seed.track2.center), (first.end, second.end))
        self.assertEqual((seed.track1.orientation, seed.track2.orientation), (84.0, 85.0))
        self.assertIsNone(seed.track1.members)
        self.assertIsNone(seed.track1.member_line.members)
        self.assertIsNotNone(first.members)

    def test_two_pairs_follow_separate_raw_rails_with_independent_labels(self):
        self.set_scene(35.0)
        gap = 0.36
        raw = np.vstack((self.scene.cloud.xyz, self.scene.cloud.xyz + gap * self.across))
        options = ((self.line(0), self.line(gap)),
                   (self.line(self.gauge), self.line(self.gauge + gap)))
        clouds, ends = [], []
        for pair in self.pairs(options):
            cloud = PointCloud(raw)
            rng = MsvcRand(1)
            tracker = ContinuationTracker(
                pair.seed_layer(self.config), GridIndex(cloud), (0.2, 0.2),
                config=self.config, rng=rng, main_track=False,
            )
            result = tracker.run(max_steps=4)
            self.assertEqual((result.status, result.accepted_steps), ("step_limit", 4))
            self.assertGreater(rng.draw_count, 0)
            ends.append(result.layer.center_vector[-1].as_array())
            clouds.append(cloud)
        self.assertAlmostEqual(float((ends[1] - ends[0]) @ self.across), gap, places=2)
        n = len(self.scene.cloud)
        self.assertGreater(np.count_nonzero(clouds[0].class_codes[:n]), 0)
        self.assertGreater(np.count_nonzero(clouds[1].class_codes[n:]), 0)
        self.assertFalse(np.any(clouds[0].class_codes[n:]))
        self.assertFalse(np.any(clouds[1].class_codes[:n]))
