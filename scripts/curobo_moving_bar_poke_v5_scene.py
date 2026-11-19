"""Short X-aligned bar with a repeating, tip-defined poke."""
import numpy as np

import curobo_moving_bar_v5_scene as sweep
from curobo_moving_bar_v5_scene import RADIUS, ROTATION_WXYZ

LENGTH = .30
MOTION_AXIS = 0
OUTER_TIP_X = .85
INNER_TIP_X = .45
HOLD_S = .5
WAIT_S = 1.
SPEED = .10
START = (OUTER_TIP_X+LENGTH/2, .28, .35)
MOTION = dict(inner_tip_x_limits_m=[INNER_TIP_X, OUTER_TIP_X], hold_s=HOLD_S, wait_s=WAIT_S)


def period(speed):
    return 2*(OUTER_TIP_X-INNER_TIP_X)/speed+HOLD_S+WAIT_S


def at(time, speed):
    if speed == 0:
        return START[0]
    travel = (OUTER_TIP_X-INNER_TIP_X)/speed
    elapsed = time % period(speed)
    if elapsed < travel:
        offset = speed*elapsed
    elif elapsed < travel+HOLD_S:
        offset = OUTER_TIP_X-INNER_TIP_X
    else:
        offset = max(0., OUTER_TIP_X-INNER_TIP_X-speed*(elapsed-travel-HOLD_S))
    return START[0]-offset


def center(x):
    return float(x), START[1], START[2]


def fit_at_center(position):
    return sweep.fit_at_center(position, length=LENGTH)


def fit(x):
    return fit_at_center(center(x))


def clearances(spheres, centers):
    return sweep.clearances(spheres, centers, length=LENGTH)


def distances(spheres, xs):
    return clearances(spheres, np.asarray([center(x) for x in xs]))


def covers(box, x):
    return sweep.covers_fit(box, fit(x))


class BarMemory(sweep.BarMemory):
    length = LENGTH


class BarWorker(sweep.BarWorker):
    length = LENGTH
    center = staticmethod(center)
