import unittest
from types import SimpleNamespace

from railway_mls.branching import (
    CONFIRM_LIMIT,
    BranchProgress,
    TraceCoverage,
    TraceMatch,
    outgoing_starts,
)
from railway_mls.config import ExtractionConfig
from railway_mls.geometry import Line3D
from railway_mls.kalman import KalmanState
from railway_mls.models import TrackSegment, TrackWindow
from railway_mls.points import Point3D, PointCloud


def rails(x, y=1000, z=10):
    return Point3D(x, y, z), Point3D(x, y + 1.5, z)


class AutomaticDiscoveryTests(unittest.TestCase):
    def test_discovers_alternate_fits_at_the_observed_window_and_keeps_rejected_forks(self):
        support = PointCloud([[0, 0, 0]]).select([0])
        lines = tuple(
            tuple(Line3D(Point3D(1.5, y + offset, 10), Point3D(2.5, y + offset, 10),
                         members=support) for offset in (0, 0.18))
            for y in (1000, 1001.5)
        )
        step = SimpleNamespace(
            number=23,
            models=tuple(SimpleNamespace(lines=options) for options in lines),
            attempts=(SimpleNamespace(predictions=tuple(
                SimpleNamespace(state=KalmanState(point)) for point in rails(2)
            )),),
            current_states=tuple(KalmanState(point) for point in rails(1)),
            candidate=TrackSegment(*(TrackWindow(point, 0) for point in rails(2))),
            accepted=True,
            status="accepted",
            selection=SimpleNamespace(selected_indices=(0, 0)),
        )
        candidates = outgoing_starts(step, ExtractionConfig(), "parent", 123, preceding_steps=12)
        self.assertEqual([item.pair.indices for item in candidates], [(1, 1)])
        start = candidates[0]
        self.assertEqual((start.parent, start.step, start.elapsed_steps, start.random_state),
                         ("parent", 23, 35, 123))

        step.accepted = False
        step.status = "gauge_difference"
        candidates = outgoing_starts(step, ExtractionConfig(), "parent", 123)
        self.assertEqual([item.pair.indices for item in candidates], [(0, 0), (1, 1)])


class TraceCoverageTests(unittest.TestCase):
    def setUp(self):
        self.coverage = TraceCoverage(ExtractionConfig())
        self.coverage.add("source-00", [rails(x) for x in range(11)])

    def test_same_track_is_found_in_both_directions_with_either_rail_order(self):
        forward = self.coverage.match(rails(1.5, 1000.05), rails(2.5, 1000.05))
        reverse = self.coverage.match(rails(3.5)[::-1], rails(2.5)[::-1])
        self.assertEqual((forward.route, forward.station, forward.direction), ("source-00", 2.5, 1))
        self.assertEqual((reverse.route, reverse.station, reverse.direction), ("source-00", 2.5, -1))
        self.assertTrue(forward.has_continuation)
        self.assertAlmostEqual(forward.distance_xy, 0.05)

    def test_crossing_adjacent_and_different_level_rails_stay_distinct(self):
        crossing = (Point3D(1.75, 1000.75, 10), Point3D(3.25, 1000.75, 10))
        approach = tuple(point - Point3D(0, 1, 0) for point in crossing)
        self.assertIsNone(self.coverage.match(approach, crossing))
        self.assertIsNone(self.coverage.match(rails(1, 1000.5), rails(2, 1000.5)))
        self.assertIsNone(self.coverage.match(rails(1, z=11), rails(2, z=11)))

    def test_covered_endpoint_keeps_new_forward_search_open(self):
        final = self.coverage.match(rails(9.5), rails(10))
        self.assertIsNotNone(final)
        self.assertFalse(final.has_continuation)
        self.assertIsNone(self.coverage.match(rails(10), rails(11)))


class BranchProgressTests(unittest.TestCase):
    def test_overlapping_trial_expires_without_becoming_a_branch(self):
        monitor = BranchProgress()
        for step in range(1, CONFIRM_LIMIT):
            self.assertIsNone(monitor.observe(step, TraceMatch("source", step, 1, True, 0)))
        stop = monitor.observe(CONFIRM_LIMIT, TraceMatch("source", CONFIRM_LIMIT, 1, True, 0))
        self.assertEqual(stop, "unconfirmed")
        self.assertIsNone(monitor.confirmed_at)

    def test_confirm_then_require_consecutive_advancing_matches_to_rejoin(self):
        monitor = BranchProgress()
        for step in range(1, 4):
            self.assertIsNone(monitor.observe(step, None))
        self.assertEqual(monitor.confirmed_at, 3)
        self.assertIsNone(monitor.observe(4, TraceMatch("source", 1, 1, True, 0)))
        self.assertIsNone(monitor.observe(5, TraceMatch("source", 1, 1, True, 0)))
        self.assertIsNone(monitor.observe(6, TraceMatch("source", 2, 1, False, 0)))
        for step in (7, 8):
            self.assertIsNone(monitor.observe(step, TraceMatch("earlier", step + 1, 1, True, 0)))
        self.assertEqual(monitor.observe(9, TraceMatch("earlier", 10, 1, True, 0)), "rejoined")
        self.assertEqual(monitor.rejoin_at, 7)
        self.assertEqual((monitor.rejoin.route, monitor.rejoin.station), ("earlier", 8))
