"""v5 camera height memory and sampled acceleration-limited floor stopping."""
import copy

import numpy as np
import torch

from curobo_moving_rod_v4 import CONTROL_DT, sphere_series, v2
from curobo_moving_rod_v4_rules import MEMORY_TTL_S, MAX_UNSEEN_SPEED_MPS


class HeightMemory:
    fit_points = staticmethod(v2.sixth.fit_with_unknown_extent)

    def __init__(self, fixed_top):
        self.fixed_top = fixed_top
        self.cloud = v2.TimedPointCloud()
        self.highest = None
        self.seen_top = False
        self.last_seen = None
        self.last_box = None
        self.previous_center = None
        self.last_event = {}
        self.top_events = 0
        self.growth_events = 0
        self.cap_events = 0

    def observe(self, capture, time):
        points, _ = self.cloud.update(capture, time)
        # Zero suppresses the legacy near-camera extension. Border metadata still
        # drives the existing horizontal extent rule.
        fresh, _ = self.fit_points(capture.points, capture, 0.)
        velocity = np.zeros(3)
        event = dict(time_s=time, fresh_seen=fresh is not None, top_rule=False,
                     growth_rule=False, since_seen_s=None, grown_dims_xy_m=None)
        if fresh is not None:
            fit, _ = self.fit_points(points, capture, 0.)
            maximum = float(np.max(capture.points[:, 2]))
            self.highest = maximum if self.highest is None else max(self.highest, maximum)
            cropped = any(capture.metadata.get('border_touches', {}).values())
            self.seen_top |= not cropped
            center = np.asarray(fresh['center_m'])
            if self.previous_center is not None:
                dt = time-self.last_seen
                if dt > 0:
                    velocity = (center-self.previous_center)/dt
            self.previous_center = center
            self.last_seen = time
            self.last_box = copy.deepcopy(fit)
            event['since_seen_s'] = 0.
        elif self.last_seen is not None:
            age = time-self.last_seen
            event['since_seen_s'] = age
            fit = None
            if age <= MEMORY_TTL_S+1e-9:
                fit = copy.deepcopy(self.last_box)
                fit['dims_m'][:2] = [d+2*MAX_UNSEEN_SPEED_MPS*age for d in fit['dims_m'][:2]]
                event.update(growth_rule=True, grown_dims_xy_m=fit['dims_m'][:2])
                self.growth_events += 1
        else:
            fit = None
        event.update(highest_observed_top_m=self.highest, top_seen=self.seen_top)
        self.last_event = event
        return fit, velocity, points, event

    def apply_height(self, fit, spheres):
        """Adjust the current box, including the swept footprint, without altering memory."""
        if fit is None:
            return None, False, False
        c, dims = np.asarray(fit['center_m']), np.asarray(fit['dims_m'])
        low, high = c-dims/2, c+dims/2
        dx = np.maximum(np.maximum(low[:2]-spheres[:, :2], spheres[:, :2]-high[:2]), 0.)
        overlap = (dx**2).sum(axis=1) <= spheres[:, 3]**2
        top = max(high[2], self.highest+.020)
        raised = not self.seen_top and not overlap.any() and top < self.fixed_top
        if raised:
            top = self.fixed_top
        capped = False
        if overlap.any():
            limit = float(np.min(spheres[overlap, 2]-spheres[overlap, 3]))-.020
            capped = top > limit
            top = min(top, limit)
        if top <= low[2]:
            return None, raised, capped
        high[2] = top
        return dict(fit, center_m=((low+high)/2).tolist(), dims_m=(high-low).tolist()), raised, capped

    def boxes(self, raw, velocity, horizon, robot, q, mask):
        if raw is None:
            return None, None, False, False
        from curobo_moving_rod_v4 import stretch_fit

        spheres = sphere_series(robot, torch.as_tensor(q[None], device='cuda', dtype=torch.float32))[0, mask]
        observed, raised, capped = self.apply_height(raw, spheres)
        if horizon:
            swept, swept_raised, swept_capped = self.apply_height(stretch_fit(raw, velocity, horizon), spheres)
            raised |= swept_raised
            capped |= swept_capped
        else:
            swept = observed
        self.top_events += int(raised)
        self.cap_events += int(capped)
        return observed, swept, raised, capped


class FloorPlane:
    """Screen a 25 ms held command followed by discrete braking, at 5 ms samples."""

    def __init__(self, robot, mask, position_limits, acceleration_limits):
        self.robot = robot
        self.mask = mask
        self.positions = position_limits
        self.acceleration = acceleration_limits
        self.acts = 0
        self.last_factor = 1.

    def braking_positions(self, q, command):
        decel = np.where(command >= 0, -self.acceleration[0], self.acceleration[1])
        increments = decel*CONTROL_DT
        steps = int(np.ceil(np.max(np.abs(command)/increments)))+1
        held = np.sign(command)*np.maximum(np.abs(command)-np.arange(steps)[:, None]*increments, 0.)
        starts = np.vstack((q, q+np.cumsum(held[:-1]*CONTROL_DT, axis=0)))
        offsets = np.linspace(0., CONTROL_DT, 6)
        return np.clip((starts[:, None, :]+held[:, None, :]*offsets[None, :, None]).reshape(-1, 6),
                       self.positions[0], self.positions[1])

    def filter(self, q, previous, desired):
        self.last_factor = 1.

        def safe(command):
            poses = self.braking_positions(q, command)
            spheres = sphere_series(self.robot, torch.as_tensor(poses, device='cuda', dtype=torch.float32))
            bottoms = spheres[:, self.mask, 2]-spheres[:, self.mask, 3]
            if not np.isfinite(bottoms).all():
                raise RuntimeError('non-finite braking clearance')
            return float(np.min(bottoms)) >= .005

        if safe(desired):
            return desired
        self.acts += 1
        low_velocity = previous+self.acceleration[0]*CONTROL_DT
        high_velocity = previous+self.acceleration[1]*CONTROL_DT
        brake = np.clip(np.zeros_like(previous), low_velocity, high_velocity)

        def candidate(factor):
            # Project requested scale into the set reachable this control step.
            return np.clip(factor*desired, low_velocity, high_velocity)

        if not safe(brake):
            raise RuntimeError('floor plane found no safe acceleration-limited braking command')
        lo, hi = 0., 1.
        accepted = brake
        for _ in range(12):
            mid = (lo+hi)/2
            tested = candidate(mid)
            if safe(tested):
                lo, accepted = mid, tested
            else:
                hi = mid
        self.last_factor = lo
        return accepted
