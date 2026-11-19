"""Live short bar with a dance and optional vertical rod."""
import time

import numpy as np

from curobo_moving_bar_v5 import base
from curobo_moving_bar_poke_height import LiveLowerPoke, scene
from curobo_moving_bar_poke_v5_scene import BarWorker
from curobo_moving_rod_mpc import update_box
from curobo_moving_rod_v5_rules import HeightMemory

GROUPS = ('A', 'B', 'E', 'E-nopred')
DANCE_PERIOD_S = 3.
DANCE_SPEED_MPS = .10
DANCE_TRAVEL_M = .125
ROD_RADIUS_M = .025
ROD_BOX = 'vertical_rod'
ROD_UNKNOWN_TOP_M = .545


def dance_center(tool, elapsed):
    # Extend and retract in 2.5 s, then pause until the next target sample.
    travel = DANCE_TRAVEL_M/DANCE_SPEED_MPS
    displacement = DANCE_SPEED_MPS*max(0., min(elapsed, 2*travel-elapsed))
    x, y, z = map(float, tool)
    return (x+scene.LENGTH/2+DANCE_TRAVEL_M-displacement, y, float(np.clip(z, .15, .30)))


class DemoBarWorker(BarWorker):
    def scene_args(self):
        return [*super().scene_args(), '--vertical-obstacle']


class VerticalControls:
    def __init__(self, server):
        self.enabled = server.gui.add_checkbox('Enable vertical rod', initial_value=False)
        self.height = server.gui.add_dropdown('Vertical rod height m', options=('0.25', '0.50'),
                                              initial_value='0.50', disabled=True)
        self.status = server.gui.add_text('Vertical rod perception', 'disabled', disabled=True)
        self.box_top = server.gui.add_text('Vertical rod box top', '—', disabled=True)
        self.rod = server.scene.add_cylinder('/truth/vertical_rod', radius=ROD_RADIUS_M,
                                             height=.50, color=(200, 110, 52), visible=False)
        self.gizmo = server.scene.add_transform_controls(
            '/controls/vertical_rod', position=(.38, .44, 0.), scale=.14,
            active_axes=(True, True, False), disable_rotations=True, visible=False)
        self.observed = server.scene.add_box('/observed/vertical_rod', dimensions=(.01, .01, .01),
                                             color=(255, 145, 35), opacity=.42, visible=False)
        self.predicted = server.scene.add_box('/predicted/vertical_rod', dimensions=(.01, .01, .01),
                                              color=(80, 150, 255), opacity=.85, wireframe=True, visible=False)
        self.pose()
        for handle in (self.enabled, self.height, self.gizmo):
            handle.on_update(lambda _event: self.pose())

    def pose(self):
        enabled = self.enabled.value
        self.height.disabled = not enabled
        self.gizmo.visible = self.rod.visible = enabled
        self.rod.height = height = float(self.height.value)
        x, y = map(float, self.gizmo.position[:2])
        self.rod.position = (x, y, height/2)
        return (x, y, height) if enabled else None

    def show(self, observed, predicted, status):
        self.status.value = status
        top = None if observed is None else observed['center_m'][2]+observed['dims_m'][2]/2
        self.box_top.value = '—' if top is None else f'{top:.3f} m'
        for handle, fit in ((self.observed, observed), (self.predicted, predicted)):
            handle.visible = fit is not None
            if fit is not None:
                handle.position = tuple(fit['center_m'])
                handle.dimensions = tuple(fit['dims_m'])


class LiveVerticalRod:
    """Vertical-rod perception and truth for one live controller loop."""
    def __init__(self, controls, show_prediction):
        self.controls = controls
        self.show_prediction = show_prediction
        self.pose = None
        self.reset_memory()
        self.controls.show(None, None, 'disabled' if self.pose is None else 'awaiting depth')

    def reset_memory(self):
        self.memory = HeightMemory(ROD_UNKNOWN_TOP_M)
        self.fit, self.estimate, self.points = None, np.zeros(3), np.empty((0, 3))
        self.first_box = None

    def read(self):
        pose = self.controls.pose()
        if (None if pose is None else pose[2]) != (None if self.pose is None else self.pose[2]):
            self.reset_memory()
        self.pose = pose

    def capture(self, worker, q, center, time_s):
        capture = worker.capture_xy(q, center, vertical_rod=self.pose)
        if capture.vertical_camera is not None:
            self.fit, self.estimate, self.points, _ = self.memory.observe(capture.vertical_camera, time_s)
        return capture

    def visible_points(self, bar_points):
        return np.concatenate((bar_points, self.points))

    def update(self, mpc, family, q, mask, horizon):
        observed = predicted = None
        status = 'disabled'
        if self.pose is not None:
            status = 'ignored by A'
            if family == 'B':
                x, y, height = self.pose
                observed = predicted = dict(center_m=[x, y, height/2],
                                            dims_m=[2*ROD_RADIUS_M, 2*ROD_RADIUS_M, height])
                status = 'true geometry'
            elif family == 'E':
                observed, predicted, raised, capped = self.memory.boxes(
                    self.fit, self.estimate, horizon, mpc.kinematics, q, mask)
                status = ('capped by arm' if capped else 'assumed tall' if raised else
                          'top seen' if self.memory.seen_top else
                          'partial view' if self.fit else 'awaiting depth')
                if self.first_box is None and observed is not None:
                    self.first_box = dict(
                        pose=self.pose,
                        status=status,
                        top_seen=self.memory.seen_top,
                        visible_top_m=self.memory.highest,
                        box_top_m=observed['center_m'][2]+observed['dims_m'][2]/2,
                    )
        update_box(mpc, predicted, name=ROD_BOX)
        self.controls.show(observed, predicted if horizon and self.show_prediction() else None, status)

    def clearance(self, spheres):
        if self.pose is None:
            return float('inf')
        x, y, height = self.pose
        radial = np.linalg.norm(spheres[:, :2]-(x, y), axis=1)-ROD_RADIUS_M
        axial = np.abs(spheres[:, 2]-height/2)-height/2
        pair = np.stack((radial, axial), axis=-1)
        return float(np.min(np.linalg.norm(np.maximum(pair, 0.), axis=-1)
                            + np.minimum(np.max(pair, axis=-1), 0.)-spheres[:, 3]))


class LiveDance(LiveLowerPoke):
    group_options = GROUPS
    worker_type = DemoBarWorker
    version = 'short-bar poke and dance'

    def __init__(self, args, baseline):
        self.dancing = False
        self.dance_started = None
        self.dance_samples = [] if args.viewer_smoke else None
        super().__init__(args, baseline)
        self.dance_button = self.server.gui.add_button('Dance')
        self.dance_button.on_click(self.toggle_dance)

    def obstacle_handles(self, scene_api):
        self.vertical = VerticalControls(self.server)
        return super().obstacle_handles(scene_api)

    def extra_obstacle(self):
        with self.poke_lock:
            self.dance_started = None
        self.active_vertical = LiveVerticalRod(self.vertical, lambda: self.graphics.visible.value)
        return self.active_vertical

    @staticmethod
    def make_mpc(config, settings):
        mpc, details = base.v41.make_mpc(config, settings, floor_activation_m=base.FLOOR_ACTIVATION_M,
                                        additional_boxes=(ROD_BOX,))
        update_box(mpc, None, name=ROD_BOX)
        return mpc, details

    def request_poke(self, event=None):
        # Poke and Dance share the same bar translation control.
        with self.poke_lock:
            if not self.dancing:
                super().request_poke(event)

    def toggle_dance(self, _event=None):
        with self.poke_lock:
            if self.poke_elapsed is not None:
                return
            self.dancing = not self.dancing
            self.dance_started = None
            self.dance_button.label = 'Stop Dance' if self.dancing else 'Dance'
            self.poke_button.disabled = self.dancing

    def before_obstacle_step(self, tool, sim_clock):
        with self.poke_lock:
            if self.dancing:
                if self.dance_started is None or sim_clock-self.dance_started >= DANCE_PERIOD_S-1e-9:
                    self.dance_started = sim_clock
                    self.dance_target = tool.copy()
                position = dance_center(self.dance_target, sim_clock-self.dance_started)
                self.gizmo.position = position
                if self.dance_samples is not None:
                    self.dance_samples.append((sim_clock, self.dance_started, self.dance_target.copy(), position))
                return
        super().before_obstacle_step(tool, sim_clock)



def wait_for_demo(app, read, description, timeout=20.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if app.current and app.current['error']:
            raise RuntimeError(app.current['error'])
        result = read()
        if result is not None:
            return result
        time.sleep(.025)
    raise RuntimeError(f'Timed out waiting for {description}')


def verify_demo(app):
    app.toggle_dance()
    try:
        wait_for_demo(app, lambda: True if len(app.dance_samples) >= 125 else None, 'two Dance targets')
    finally:
        app.toggle_dance()
    samples = app.dance_samples.copy()
    starts = list(dict.fromkeys(start for _, start, _, _ in samples))
    first = np.asarray([position for _, start, _, position in samples if start == starts[0]])
    targets = [next(target for _, start, target, _ in samples if start == t) for t in starts]
    np.testing.assert_allclose(starts[1]-starts[0], DANCE_PERIOD_S, atol=1e-8)
    np.testing.assert_allclose(first[:, 1:], np.tile(first[0, 1:], (len(first), 1)))
    np.testing.assert_allclose(first[[0, -1], 0], [first[0, 0]]*2)
    np.testing.assert_allclose(first[0, 0]-first[:, 0].min(), DANCE_TRAVEL_M)
    np.testing.assert_allclose(np.max(np.abs(np.diff(first[:, 0])))/base.v4.CONTROL_DT, DANCE_SPEED_MPS)
    for _, _, target, position in samples:
        np.testing.assert_allclose(position[1:], [target[1], np.clip(target[2], .15, .30)])
    if np.allclose(targets[0], targets[1]):
        raise RuntimeError('Dance did not sample a new tool target')
    dance = dict(cycle_period_s=starts[1]-starts[0], travel_m=DANCE_TRAVEL_M,
                 target_xyz_m=[target.tolist() for target in targets])

    rod_checks = []
    for x, y, status in ((.38, .44, 'assumed tall'), (.58, .10, 'capped by arm')):
        app.vertical.gizmo.position = (x, y, 0.)
        app.vertical.enabled.value = True
        app.selector.value = 'E'
        evidence = wait_for_demo(
            app, lambda: app.active_vertical.first_box if app.current['group'] == 'E' else None, status)
        if evidence['status'] != status or evidence['top_seen'] or not evidence['visible_top_m'] < .50:
            raise RuntimeError(f'Unexpected vertical rod observation: {evidence}')
        if status == 'assumed tall':
            np.testing.assert_allclose(evidence['box_top_m'], ROD_UNKNOWN_TOP_M)
        elif not evidence['box_top_m'] < evidence['visible_top_m']:
            raise RuntimeError(f'Vertical box was not reduced by the arm: {evidence}')
        rod_checks.append(evidence)
        app.selector.value = 'A'
        wait_for_demo(app, lambda: True if app.current['group'] == 'A' and app.current['steps'] else None,
                      'A restart after E')
    app.vertical.enabled.value = False

    app.toggle_dance()
    try:
        dance['switched_groups'] = []
        for mode in (*GROUPS[1:], 'A'):
            app.selector.value = mode

            def restarted():
                current = app.current
                if current['group'] == mode and current['steps'] >= 3:
                    return app.dancing and app.dance_started == 0.

            if not wait_for_demo(app, restarted, f'Dance restart in {mode}'):
                raise RuntimeError(f'Dance did not restart in {mode}')
            dance['switched_groups'].append(mode)
    finally:
        app.toggle_dance()
    return dict(dance_verified=dance, vertical_rod_verified=rod_checks)
