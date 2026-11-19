"""Horizontal X-axis bar and its v5 perception policy."""
import copy
import math

import numpy as np

from curobo_moving_rod_v5_rules import HeightMemory
from curobo_moving_rod_live import LiveCameraWorker

RADIUS = .020
LENGTH = .60
START = (.50, .05, .35)
Y_MIN, Y_MAX = .05, .55
# Both the Newton and Viser cylinders start with their axis on local Z.
ROTATION_WXYZ = (math.sqrt(.5), 0., math.sqrt(.5), 0.)


def at(time, speed):
    span = Y_MAX-Y_MIN
    return Y_MAX-abs((time*speed) % (2*span)-span)


def center(y):
    return (START[0], float(y), START[2])


def fit_at_center(position):
    return {'center_m': list(map(float, position)), 'dims_m': [LENGTH, 2*RADIUS, 2*RADIUS]}


def fit(y):
    return fit_at_center(center(y))


def clearances(spheres, centers):
    """Sphere-to-finite-cylinder signed clearance, with the cylinder on X."""
    relative = spheres[..., :3]-np.asarray(centers)[:, None, :]
    radial = np.linalg.norm(relative[..., 1:], axis=-1)-RADIUS
    axial = np.abs(relative[..., 0])-LENGTH/2
    pair = np.stack((radial, axial), axis=-1)
    signed = np.linalg.norm(np.maximum(pair, 0.), axis=-1)+np.minimum(np.max(pair, axis=-1), 0.)
    return np.min(signed-spheres[..., 3], axis=-1)


def distances(spheres, ys):
    return clearances(spheres, np.array([center(y) for y in ys]))


def covers(box, y):
    if box is None:
        return False
    truth = fit(y)
    c, d = np.asarray(box['center_m']), np.asarray(box['dims_m'])
    tc, td = np.asarray(truth['center_m']), np.asarray(truth['dims_m'])
    return bool(np.all(c-d/2 <= tc-td/2+1e-6) and np.all(c+d/2 >= tc+td/2-1e-6))


class BarMemory(HeightMemory):
    def __init__(self, _fixed_top):
        super().__init__(0.)
        self.border_length_events = 0

    def observe(self, capture, time):
        box, velocity, points, event = super().observe(capture, time)
        extended = box is not None and event['fresh_seen'] and any(
            capture.metadata.get('border_touches', {}).values())
        if extended:
            # Center on the visible cluster rather than its previously padded box.
            box['center_m'][0] = float((capture.points[:, 0].min()+capture.points[:, 0].max())/2)
            box['dims_m'][0] = LENGTH
            self.last_box = copy.deepcopy(box)
            self.border_length_events += 1
        event['bar_border_length_extended'] = bool(extended)
        return box, velocity, points, event


class BarWorker(LiveCameraWorker):
    def scene_args(self):
        return ['--horizontal-bar']

    def capture(self, q, y):
        return self.capture_xy(np.asarray(q), np.asarray(center(y)))
