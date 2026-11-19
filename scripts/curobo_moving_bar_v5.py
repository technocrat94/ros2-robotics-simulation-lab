#!/usr/bin/env python3
"""Run or watch the three-case v5 horizontal bar checkpoint."""
import argparse
import json
import os
import tempfile
from pathlib import Path

import curobo_moving_bar_v5_scene as bar
import curobo_moving_rod_v42 as base
from curobo_moving_rod_v5_rules import FloorPlane

v41, v2, tuned = base.v41, base.v2, base.tuned
RESULT = v2.RESULTS_DIR / 'moving_bar_v5.checkpoint.json'
SPEED = .10
GROUPS = ('A', 'B', 'E')


def saved_case(config, settings, baseline, mode, worker=None, *, no_rod=False, scene=bar):
    memory = scene.BarMemory(0.) if mode == 'E' else None
    outcome = base.run_case(
        config, settings, baseline, mode, worker=worker, speed=SPEED, no_rod=no_rod,
        guard=lambda d: base.floor_stop_reason(mode, d),
        height_memory=memory, plane_factory=FloorPlane, obstacle=scene,
    )
    summary = outcome['case']['summary']
    summary.update(initial_x_m=scene.START[0], initial_y_m=scene.START[1], repeat=1)
    return outcome


def measure(args, baseline, preflight, scene=bar, result=RESULT):
    settings = preflight['frozen_config']
    report = dict(complete=False, scene_status='UNVERIFIED', bar=dict(radius_m=scene.RADIUS,
        length_m=scene.LENGTH, axis='X', start_m=scene.START, motion_axis='XY'[scene.MOTION_AXIS]),
        preflight=preflight, cases={})
    if hasattr(scene, "MOTION"):
        report["bar"].update(scene.MOTION)
    else:
        report["bar"]["y_limits_m"] = [scene.Y_MIN, scene.Y_MAX]
    mpc, _ = v41.make_mpc(args.robot_config, settings, floor_activation_m=base.FLOOR_ACTIVATION_M)
    try:
        costs = base.endpoint_costs(mpc, baseline)
    finally:
        mpc.destroy()
    report['endpoint_costs'] = costs
    if any(v != 0. for methods in costs.values() for values in methods.values() for v in values.values()):
        raise RuntimeError('endpoint floor cost or gradient is nonzero')
    qualification = saved_case(args.robot_config, settings, baseline, 'A', no_rod=True, scene=scene)
    report['qualification'] = qualification
    tuned.save_json(result, report)
    print('BAR_QUALIFICATION', qualification['passed'],
          qualification['case']['summary']['time_to_first_goal_s'], flush=True)
    if not qualification['passed']:
        return 2
    with tempfile.TemporaryDirectory(prefix='bar-v5-') as temp:
        worker = scene.BarWorker(args.newton_python, Path(temp))
        try:
            for mode in GROUPS:
                axis = scene.MOTION_AXIS
                name = f'Bar {mode} 0.10 m/s {"XY"[axis]}={scene.START[axis]:.2f}'
                outcome = saved_case(args.robot_config, settings, baseline, mode, worker, scene=scene)
                case = outcome['case']
                summary = case['summary']
                report['cases'][name] = case
                if mode == 'A':
                    report['scene_status'] = 'VALID_SCENE' if summary['rod_collisions'] else 'INVALID_SCENE'
                summary['scene_status'] = report['scene_status']
                tuned.save_json(result, report)
                print(f"BAR_CASE {mode} reaches={summary['reaches']} bar={summary['rod_collisions']} "
                      f"floor={summary['floor_contacts']} clearance={summary['minimum_true_clearance_m']:.6f} "
                      f"border={summary.get('bar_border_length_extensions', 0)}", flush=True)
                if summary['stop_reason'] or report['scene_status'] != 'VALID_SCENE':
                    return 2
        finally:
            worker.close()
    report['complete'] = True
    tuned.save_json(result, report)
    return 0


def main(*, scene=bar, result=RESULT, live_class=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--smoke', action='store_true')
    mode.add_argument('--replay', action='store_true')
    mode.add_argument('--interactive', action='store_true')
    parser.add_argument('--viewer-smoke', action='store_true')
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--port', type=int, default=int(os.environ.get('NEWTON_VIEWER_PORT', '30004')))
    parser.add_argument('--robot-config', type=Path, default=Path.home()/'.cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml')
    parser.add_argument('--newton-python', type=Path, default=Path.home()/'newton_ws/.venv-cpu/bin/python')
    args = parser.parse_args()
    if args.viewer_smoke and args.smoke:
        parser.error('--viewer-smoke requires replay or interactive')
    preflight = json.loads(v41.PREFLIGHT_FILE.read_text())
    v41.require_preflight(preflight)
    baseline = json.loads(tuned.original.BASELINE_FILE.read_text())
    tuned.original.configure_scene(baseline)
    if args.smoke:
        return measure(args, baseline, preflight, scene, result)
    from curobo_moving_bar_v5_viewer import watch
    return watch(args, baseline, result, scene=scene, live_class=live_class)


if __name__ == '__main__':
    raise SystemExit(main())
