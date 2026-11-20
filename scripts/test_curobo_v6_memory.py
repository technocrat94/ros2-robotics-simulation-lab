"""GPU-free checks for v6 box classification, projection and clearing."""
import unittest

import numpy as np

from curobo_v6_memory import DepthFrame, ObjectMemory


def box(x=0., y=0., z=.5, size=.10):
    return dict(center_m=[x, y, z], dims_m=[size]*3)


def frame(depth=0., mask=None, origin=(0., 0., 0.), rotation=None):
    return DepthFrame(np.full((100, 100), depth, dtype=float),
                      np.zeros((100, 100), dtype=bool) if mask is None else mask,
                      (50., 50., 50., 50.), np.asarray(origin),
                      np.eye(3) if rotation is None else rotation)


class MemoryRules(unittest.TestCase):
    def observed(self, track, time, value):
        return track.observe(value, value['center_m'], frame(), time)

    def test_reclassifies_as_the_object_starts_and_stops(self):
        track = ObjectMemory()
        _, _, event = self.observed(track, 0., box())
        self.assertEqual(event['class'], 'STATIC')
        _, _, event = self.observed(track, .5, box(x=.02))
        self.assertEqual(event['class'], 'MOVING')
        _, _, event = self.observed(track, 1., box(x=.02))
        self.assertEqual(event['class'], 'STATIC')
        _, _, event = self.observed(track, 1.5, box(x=.07))
        self.assertEqual(event['class'], 'MOVING')

    def test_static_retains_the_identical_box_without_expiry(self):
        track = ObjectMemory()
        obstacle = box(x=1.1)
        self.observed(track, 0., obstacle)
        actual, velocity, event = track.observe(None, None, frame(), 12.)
        self.assertEqual(actual, obstacle)
        np.testing.assert_array_equal(velocity, [0., 0., 0.])
        self.assertEqual(event['since_seen_s'], 12.)
        self.assertEqual(event['cleared_by'], 'none')

    def test_moving_toward_robot_propagates_and_grows_xy(self):
        track = ObjectMemory()
        self.observed(track, 0., box(x=1.))
        self.observed(track, .5, box(x=.95))
        for time in (.7, 1.):
            actual, velocity, event = track.observe(None, None, frame(), time)
            age = time-.5
            np.testing.assert_allclose(actual['center_m'], [.95-.1*age, 0., .5])
            np.testing.assert_allclose(actual['dims_m'], [.1+.4*age, .1+.4*age, .1])
            np.testing.assert_allclose(velocity, [-.1, 0., 0.])
            self.assertEqual(event['class'], 'MOVING')
            self.assertTrue(event['growth_rule'])

    def test_moving_away_keeps_size_and_never_balloons(self):
        track = ObjectMemory()
        self.observed(track, 0., box(x=.5))
        self.observed(track, .5, box(x=.55))
        for time in (1., 2., 4.):
            actual, _, event = track.observe(None, None, frame(), time)
            np.testing.assert_allclose(actual['dims_m'], [.1, .1, .1])
            np.testing.assert_allclose(actual['center_m'], [.55+.1*(time-.5), 0., .5])
            self.assertTrue(event['moving_away'])
            self.assertFalse(event['growth_rule'])

    def test_carving_removes_only_the_part_seen_empty(self):
        track = ObjectMemory()
        self.observed(track, 0., box(size=.4))
        image = frame(2.)
        image.depth[:, :50] = 0.     # left half of the image: no evidence
        actual, _, event = track.observe(None, None, image, 1.)
        self.assertEqual(event['cleared_by'], 'none')
        self.assertLess(actual['dims_m'][0], .4)
        self.assertLess(actual['center_m'][0], 0.)

    def test_box_stays_while_any_part_lacks_empty_evidence(self):
        track = ObjectMemory()
        obstacle = box()
        self.observed(track, 0., obstacle)
        occluded = frame(2.)
        occluded.robot_mask[50, 50] = True
        blocking = frame(2.)
        blocking.depth[54, 54] = .57
        unknown = frame(2.)
        unknown.depth[50, 50] = np.inf
        for t, image in enumerate((occluded, blocking, unknown, frame(2., origin=(.9, 0., 0.))), 1):
            single = ObjectMemory()
            self.observed(single, 0., obstacle)
            actual, _, event = single.observe(None, None, image, t)
            self.assertIsNotNone(actual)
            self.assertEqual(event['cleared_by'], 'none')
        actual, _, event = track.observe(None, None, frame(2.), 5.)
        self.assertIsNone(actual)
        self.assertEqual(event['cleared_by'], 'seen_empty')
        _, _, event = self.observed(track, 6., obstacle)
        self.assertEqual(event['cleared_by'], 'none')

    def test_evidence_accumulates_across_frames(self):
        track = ObjectMemory()
        self.observed(track, 0., box())
        first = frame(2.)
        first.robot_mask[50, 50] = True          # the centre is hidden behind the arm
        actual, _, _ = track.observe(None, None, first, 1.)
        self.assertIsNotNone(actual)
        self.assertLess(actual['dims_m'][0], .1)  # the rest was carved away
        actual, _, event = track.observe(None, None, frame(2.), 2.)
        self.assertIsNone(actual)
        self.assertEqual(event['cleared_by'], 'seen_empty')

    def test_moving_track_can_clear_from_observed_empty_depth(self):
        track = ObjectMemory()
        self.observed(track, 0., box())
        self.observed(track, .5, box(x=.05))
        actual, _, event = track.observe(None, None, frame(2.), .6)
        self.assertIsNone(actual)
        self.assertEqual(event['class'], 'MOVING')
        self.assertEqual(event['cleared_by'], 'seen_empty')

    def test_uses_camera_pose_and_optical_depth(self):
        rotation = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
        camera = frame(1., origin=(1., 2., 3.), rotation=rotation)
        self.assertTrue(camera.proves_empty(box(1.5, 2., 3.)))
        self.assertFalse(camera.proves_empty(box(.5, 2., 3.)))

    def test_full_nearest_point_distance_controls_moving_expiry(self):
        track = ObjectMemory()
        self.observed(track, 0., box(x=.8, z=0., size=.2))
        self.observed(track, .5, box(x=1.05, z=0., size=.2))
        actual, _, event = track.observe(None, None, frame(), .6)
        self.assertGreater(actual['center_m'][0], 1.)
        self.assertEqual(event['cleared_by'], 'none')
        actual, _, event = track.observe(None, None, frame(), 1.)
        self.assertIsNone(actual)
        self.assertEqual(event['cleared_by'], 'out_of_reach')


if __name__ == '__main__':
    unittest.main()
