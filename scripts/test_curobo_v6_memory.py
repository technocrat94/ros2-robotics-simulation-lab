"""GPU-free checks for v6 box classification, projection and clearing."""
import unittest

import numpy as np

from curobo_v6_memory import (
    clip_sweep, box_sphere_distance,
    DepthFrame, ObjectMemory, path_target, summarize_pokes, tip_projection, visible_path_target,
)


def box(x=0., y=0., z=.5, size=.10):
    return dict(center_m=[x, y, z], dims_m=[size]*3)


def frame(depth=0., mask=None, origin=(0., 0., 0.), rotation=None):
    return DepthFrame(np.full((100, 100), depth, dtype=float),
                      np.zeros((100, 100), dtype=bool) if mask is None else mask,
                      (50., 50., 50., 50.), np.asarray(origin),
                      np.eye(3) if rotation is None else rotation)


class DanceGeometry(unittest.TestCase):
    def test_aims_fifteen_centimeters_along_the_path_then_clips_height(self):
        np.testing.assert_allclose(path_target(np.array([0., 0., .50]), [.6, .8, .50]),
                                   [.09, .12, .38])
        np.testing.assert_allclose(path_target(np.array([0., 0., .30]), [.03, .04, .17]),
                                   [.03, .04, .17])
        np.testing.assert_allclose(path_target(np.array([0., 0., .28]), [0., 0., 0.]),
                                   [0., 0., .15])

    def test_peak_projection_checks_image_bounds_in_the_camera_frame(self):
        camera = frame(origin=(1., 0., 0.))
        self.assertEqual(tip_projection(camera, [1., 0., .5]), (True, [50., 50.]))
        self.assertFalse(tip_projection(camera, [2., 0., .5])[0])
        self.assertEqual(tip_projection(camera, [1., 0., -.5]), (False, None))


class SweepClip(unittest.TestCase):
    observed = dict(center_m=[.5, 0., .2], dims_m=[.1, .1, .1])
    predicted = dict(center_m=[.4, 0., .2], dims_m=[.3, .1, .1])   # swept 0.2 m toward -X

    def test_far_arm_keeps_the_full_sweep(self):
        box, kept = clip_sweep(self.observed, self.predicted, [[-1., 0., .2, .05]])
        self.assertEqual(kept, 1.)
        self.assertEqual(box, self.predicted)

    def test_sweep_stops_two_centimetres_before_the_arm(self):
        sphere = [[.3, 0., .2, .05]]                     # arm surface at x = 0.35
        box, kept = clip_sweep(self.observed, self.predicted, sphere)
        self.assertTrue(0. < kept < 1.)
        low = np.asarray(box['center_m'])-np.asarray(box['dims_m'])/2
        high = np.asarray(box['center_m'])+np.asarray(box['dims_m'])/2
        self.assertAlmostEqual(box_sphere_distance(low, high, sphere), .02, places=4)
        self.assertAlmostEqual(high[0], .55)               # the observed far side is untouched

    def test_observed_box_already_near_arm_is_not_swept(self):
        box, kept = clip_sweep(self.observed, self.predicted, [[.43, 0., .2, .01]])
        self.assertEqual(kept, 0.)
        self.assertEqual(box, self.observed)


class VisibleDance(unittest.TestCase):
    def down_camera(self, origin):
        # Optical +Z = world -Z (looking straight down), as on the wrist mount.
        rotation = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
        return frame(5., origin=origin, rotation=rotation)

    def test_target_is_inside_the_view_and_below_the_camera(self):
        camera = self.down_camera((.6, .12, .45))
        tip = visible_path_target(np.array([.5, .12, .50]), [.5, .45, .24], camera)
        self.assertIsNotNone(tip)
        self.assertLess(tip[2], .45-.1)
        inside, _ = tip_projection(camera, tip)
        self.assertTrue(inside)

    def test_no_target_when_the_camera_sees_no_part_of_the_path(self):
        camera = self.down_camera((3., 3., .45))
        self.assertIsNone(visible_path_target(np.array([.5, .12, .50]), [.5, .45, .24], camera))


    def test_poke_report_attributes_late_clear_to_last_sighting(self):
        pokes = [{'start_s': 0.}, {'start_s': 3.}]
        frames = [
            dict(time_s=2.8, obstacle_pixels=120, fresh_seen=True, **{'class': 'MOVING'}),
            dict(time_s=2.9, obstacle_pixels=5, fresh_seen=False, **{'class': 'MOVING'}),
        ]
        clears = [dict(time_s=3.3, since_seen_s=.5, cleared_by='moved_on')]
        summarize_pokes(pokes, frames, clears)
        self.assertEqual(pokes[0]['class_while_seen'], {'STATIC': 0, 'MOVING': 1})
        self.assertEqual(pokes[0]['visible_without_fit_frames'], 1)
        self.assertEqual(pokes[0]['clears'], [{'time_s': 3.3, 'cleared_by': 'moved_on'}])
        self.assertEqual(pokes[1]['clears'], [])


class MemoryRules(unittest.TestCase):
    def test_short_fast_track_is_moving_not_static(self):
        track = ObjectMemory()
        self.observed(track, 0., box())
        _, _, event = self.observed(track, .1, box(x=.01))   # 1 cm in 0.1 s = 0.1 m/s
        self.assertEqual(event['class'], 'MOVING')


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

    def test_moving_toward_robot_moves_but_keeps_its_size(self):
        track = ObjectMemory()
        self.observed(track, 0., box(x=1.))
        self.observed(track, .5, box(x=.95))
        for time in (.7, 1.):
            actual, velocity, event = track.observe(None, None, frame(), time)
            age = time-.5
            np.testing.assert_allclose(actual['center_m'], [.95-.1*age, 0., .5])
            np.testing.assert_allclose(actual['dims_m'], [.1, .1, .1])
            np.testing.assert_allclose(velocity, [-.1, 0., 0.])
            self.assertEqual(event['class'], 'MOVING')
            self.assertTrue(event['heading_in'])
            self.assertFalse(event['growth_rule'])

    def test_moving_away_keeps_size_and_never_balloons(self):
        track = ObjectMemory()
        self.observed(track, 0., box(x=.5))
        self.observed(track, .5, box(x=.55))
        for time in (.6, .8, 1.):
            actual, _, event = track.observe(None, None, frame(), time)
            np.testing.assert_allclose(actual['dims_m'], [.1, .1, .1])
            np.testing.assert_allclose(actual['center_m'], [.55+.1*(time-.5), 0., .5])
            self.assertTrue(event['moving_away'])
            self.assertFalse(event['growth_rule'])

    def test_moving_box_is_dropped_after_its_measurement_window(self):
        track = ObjectMemory()
        self.observed(track, 0., box(x=1.))
        self.observed(track, .5, box(x=.95))
        actual, _, _ = track.observe(None, None, frame(), 1.)
        self.assertIsNotNone(actual)
        actual, _, event = track.observe(None, None, frame(), 1.05)
        self.assertIsNone(actual)
        self.assertEqual(event['cleared_by'], 'moved_on')
        # A static object is never dropped by time.
        still = ObjectMemory()
        self.observed(still, 0., box())
        self.observed(still, .5, box())
        actual, _, _ = still.observe(None, None, frame(), 60.)
        self.assertIsNotNone(actual)

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
