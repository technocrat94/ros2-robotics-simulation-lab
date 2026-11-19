#!/usr/bin/env python3
"""Run the v4.2 gates with a floor-only 10 mm MPC activation distance."""
import argparse
import json
import os
import signal
import tempfile
import time
from pathlib import Path

import numpy as np
import torch

import curobo_moving_rod_v41 as v41
from curobo._src.geom.collision.buffer_collision import CollisionBuffer
from curobo._src.robot.kinematics.kinematics_state import KinematicsState

v4, v2, tuned = v41.v4, v41.v2, v41.tuned
FLOOR_ACTIVATION_M = .010
ROD_ACTIVATION_M = .030
REPORT = v2.RESULTS_DIR / 'moving_rod_v42.gates.json'
PREVIOUS_BATCH = v2.RESULTS_DIR / 'moving_rod_v42.json'
BATCH = v2.RESULTS_DIR / 'moving_rod_v42.resumed.json'
EXPECTED_WALL_S = {mode: 50 if mode.startswith('E') else 30 for mode in v4.GROUPS}


def endpoint_costs(mpc, baseline):
    checker = mpc.scene_collision_checker
    checker.enable_obstacle('camera_obstacle', False)
    reference = json.loads(v41.v3.REFERENCE_FILE.read_text())
    poses = {
        'start': v2.sixth.CAMERA_FRONT_SEED,
        'raised_start': baseline['raised_start']['joints_rad'],
        'goal': reference['attempts'][-1]['case']['trace'][-1]['q'],
    }
    results = {}
    for name, pose in poses.items():
        q = torch.as_tensor(pose, device='cuda', dtype=torch.float32).reshape(1, -1)
        spheres = mpc.compute_kinematics(v4.joint_state(q, np.zeros(6))).robot_spheres
        # Use a stationary rollout with two states to exercise swept and discrete queries.
        spheres = spheres.reshape(1, 1, -1, 4).detach().repeat(1, 2, 1, 1).requires_grad_()
        results[name] = {}
        for method in ('get_sphere_distance', 'get_swept_sphere_distance'):
            device = checker.device_cfg
            buffer = CollisionBuffer.from_shape(spheres.shape, device)
            options = {'trajectory_dt': device.to_device([.020])} if 'swept' in method else {}
            costs = getattr(checker, method)(
                KinematicsState(robot_spheres=spheres), buffer, weight=device.to_device([1.]),
                activation_distance=device.to_device([ROD_ACTIVATION_M]), return_loss=True, **options,
            )
            gradient, = torch.autograd.grad(costs.sum(), spheres)
            results[name][method] = {
                'max_abs_cost': costs.abs().max().item(),
                'max_abs_gradient': gradient.abs().max().item(),
            }
    return results


def run_case(config, settings, baseline, mode, *, no_rod=False, speed=.05, worker=None,
             guard=v41.floor_guard, **policies):
    start = np.asarray(baseline['raised_start']['joints_rad'], dtype=np.float32)
    mpc, _ = v41.make_mpc(config, settings, floor_activation_m=FLOOR_ACTIVATION_M)
    try:
        mpc.setup(v4.joint_state(start, np.zeros(6)))
        case = v4.run_case(
            mpc, start, 0. if no_rod else speed, not no_rod and not mode.endswith('-nopred'),
            mode, worker, baseline,
            duration=v41.QUALIFICATION_DURATION_S if no_rod else 20.,
            no_rod=no_rod, stop_at_goal=no_rod,
            floor_mask=v41.nonbase_mask(mpc.kinematics), step_guard=guard, **policies,
        )
    finally:
        mpc.destroy()
    case['summary']['initial_x_m'] = v2.ROD_START_X_M
    plane_factors = [c['floor_plane_factor'] for c in case['cycles'] if 'floor_plane_factor' in c]
    result = {'goal': v41.goal_distances(case['distance_to_goal']), 'case': v4.compact_case(case)}
    if plane_factors:
        for cycle, factor in zip(result['case']['cycles'], plane_factors):
            cycle['floor_plane_factor'] = factor
    summary = result['case']['summary']
    result['passed'] = (summary['survives'] if no_rod or mode == 'B' else summary['floor_contacts'] == 0)
    result['passed'] = bool(result['passed'] and summary['stop_reason'] is None)
    return result


def floor_stop_reason(mode, clearance):
    if clearance < -.20:
        return f'floor penetration {-clearance*1000:.3f} mm exceeds 200 mm'
    if mode in ('A', 'B'):
        return v41.floor_guard(clearance)
    return None


def checkpoint(report):
    temp = BATCH.with_suffix('.tmp')
    temp.write_text(json.dumps(report, separators=(',', ':'), allow_nan=False)+'\n')
    temp.replace(BATCH)


def batch(config, newton_python, *, run=run_case, resume=True, version="4.2",
          expected_wall_s=EXPECTED_WALL_S):
    if BATCH.exists():
        raise RuntimeError(f'Existing batch checkpoint: {BATCH}. Archive it before a new run.')
    gates = json.loads(REPORT.read_text())['gates']
    required = ('endpoint_costs', 'qualification', 'smoke_A', 'smoke_B')
    if not all(gates.get(name, {}).get('passed') for name in required):
        raise RuntimeError('Batch requires all four earlier gates to pass')
    preflight = json.loads(v41.PREFLIGHT_FILE.read_text())
    v41.require_preflight(preflight)
    baseline = json.loads(tuned.original.BASELINE_FILE.read_text())
    tuned.original.configure_scene(baseline)
    matrix = [(speed, phase, mode) for speed in v4.SPEEDS for phase in v4.PHASES
              for mode in v4.GROUPS]
    carried = {}
    first_b = []
    if resume:
        previous = json.loads(PREVIOUS_BATCH.read_text())
        if (previous['preflight'] != preflight or previous['rod'] != baseline['rod']
                or previous['raised_start'] != baseline['raised_start']
                or previous['floor_activation_m'] != FLOOR_ACTIVATION_M
                or previous['rod_activation_m'] != ROD_ACTIVATION_M):
            raise RuntimeError('Previous checkpoint configuration does not match')
        for speed, phase, mode in matrix:
            name = f'v{version} {mode} {speed:.2f} m/s X={phase:.2f}'
            case = previous['cases'].get(name)
            if case is None or case['summary']['stop_reason']:
                break
            summary = case['summary']
            if (summary['steps'] != 800 or summary['simulated_duration_s'] != 20.
                    or summary['minimum_floor_clearance_m'] < -.20
                    or (mode in ('A', 'B') and summary['floor_contacts'])):
                raise RuntimeError(f'Completed checkpoint case failed validation: {name}')
            carried[name] = case
            if mode == 'B':
                first_b.append(summary['survives'])
    remaining = matrix[len(carried):]
    expected_s = sum(expected_wall_s[mode] for _, _, mode in remaining)
    print(f'V{version} batch: {len(carried)} carried, {len(remaining)} to run, '
          f'estimated remaining wall time {expected_s/60:.1f} minutes', flush=True)
    report = dict(schema=version, complete=False, stop_reason=None, preflight=preflight,
                  floor_activation_m=FLOOR_ACTIVATION_M, rod_activation_m=ROD_ACTIVATION_M,
                  rod=baseline['rod'], raised_start=baseline['raised_start'], cases=carried,
                  carried_cases=list(carried), resumed_from=PREVIOUS_BATCH.name if resume else None,
                  expected_wall_s_by_mode=expected_wall_s, expected_remaining_wall_s=expected_s)
    checkpoint(report)

    def timeout(_signal, _frame):
        raise TimeoutError('run exceeded 3x expected wall time')

    old_handler = signal.signal(signal.SIGALRM, timeout)
    try:
        with tempfile.TemporaryDirectory(prefix='rod-v42-') as temp:
            worker = v2.MovingRodWorker(newton_python, Path(temp))
            try:
                for speed, phase, mode in remaining:
                    guard = lambda clearance: floor_stop_reason(mode, clearance)
                    v2.ROD_START_X_M = phase
                    name = f'v{version} {mode} {speed:.2f} m/s X={phase:.2f}'
                    began = time.monotonic()
                    signal.setitimer(signal.ITIMER_REAL, 3*expected_wall_s[mode])
                    try:
                        if mode.startswith('E') and worker.process is None:
                            worker.start()
                            worker.capture(np.asarray(baseline['raised_start']['joints_rad']), phase)
                        outcome = run(config, preflight['frozen_config'], baseline, mode,
                                           speed=speed, worker=worker, guard=guard)
                        case = outcome['case']
                        summary = case['summary']
                        summary.update(repeat=1, wall_time_s=time.monotonic()-began)
                        json.dumps(case, allow_nan=False)
                    except Exception as error:
                        report['stop_reason'] = f'{name}: {type(error).__name__}: {error}'
                        break
                    finally:
                        signal.setitimer(signal.ITIMER_REAL, 0)
                    report['cases'][name] = case
                    print(f'V{version}_CASE {len(report["cases"]):02d}/75 {name} '
                          f'success={summary["survives"]} floor={summary["floor_contacts"]} '
                          f'wall={summary["wall_time_s"]:.2f}s', flush=True)
                    if summary['stop_reason']:
                        report['stop_reason'] = f'{name}: {summary["stop_reason"]}'
                    if mode == 'B' and len(first_b) < 3:
                        first_b.append(summary['survives'])
                        if len(first_b) == 3 and not any(first_b):
                            report['stop_reason'] = 'First three B cases all failed'
                    checkpoint(report)
                    if report['stop_reason']:
                        break
            finally:
                worker.close()
    except KeyboardInterrupt:
        report['stop_reason'] = 'Batch interrupted'
    finally:
        signal.signal(signal.SIGALRM, old_handler)
    if len(report['cases']) == len(matrix) and not report['stop_reason']:
        v4.summarize(report)
        if version == '5':
            for row in report['rates']:
                selected = [case['summary'] for case in report['cases'].values()
                            if case['summary']['mode'] == row['mode']
                            and case['summary']['rod_speed_mps'] == row['speed_mps']
                            and case['summary']['scene_status'] == 'VALID_SCENE']
                row['deepest_floor_depth_m'] = max(
                    (max(0., -case['minimum_floor_clearance_m']) for case in selected), default=0.)
                for key in ('box_cap_actions', 'floor_plane_actions'):
                    row[key] = sum(case.get(key, 0) for case in selected)
        report['complete'] = True
    checkpoint(report)
    BATCH.with_suffix('.log').write_text(json.dumps(
        {key: report[key] for key in ('complete', 'stop_reason', 'rates') if key in report}, indent=2)+'\n')
    print(f'V{version}_BATCH complete={report["complete"]} stop={report["stop_reason"]} cases={len(report["cases"])}', flush=True)
    return 0 if report['complete'] else 2


def watch(args, *, memory_factory=None, plane_factory=None, version="4.2"):
    v41.v3.check_viewer_port(args.port)
    baseline = json.loads(tuned.original.BASELINE_FILE.read_text())
    tuned.original.configure_scene(baseline)
    if args.interactive:
        from curobo_moving_rod_v4_live import LiveGroups, smoke

        class LiveV42(LiveGroups):
            preflight_file = v41.PREFLIGHT_FILE
            make_mpc = staticmethod(lambda config, settings:
                                    v41.make_mpc(config, settings, floor_activation_m=FLOOR_ACTIVATION_M))
            floor_mask = staticmethod(v41.nonbase_mask)
        LiveV42.version = f'v{version}'
        if memory_factory is not None:
            LiveV42.camera_memory = staticmethod(memory_factory)
        if plane_factory is not None:
            LiveV42.plane_factory = staticmethod(plane_factory)

        app = LiveV42(args, baseline)
        try:
            print(f'V{version}_INTERACTIVE http://127.0.0.1:{args.port}', flush=True)
            if args.interactive_smoke:
                report = smoke(app)
                print(f'V{version}_INTERACTIVE_SMOKE ' + json.dumps(report), flush=True)
                return 0 if report['all_passed'] else 2
            wait_for_viewer(args.duration)
        finally:
            if not app.closed:
                app.close()
        return 0

    from curobo_moving_rod_v4_viewer import FrameCache, start_viewer
    report = json.loads(BATCH.read_text())
    with tempfile.TemporaryDirectory(prefix='rod-v42-replay-') as temp:
        worker = v2.MovingRodWorker(args.newton_python, Path(temp))
        viewer = None
        try:
            frames = FrameCache(worker, report)
            frames[next(iter(report['cases']))]
            viewer, verification = start_viewer(args.robot_config, report, frames, args.port,
                                                verify=args.viewer_smoke, floor_mask=v41.nonbase_mask)
            if args.viewer_smoke and not verification["all_passed"]:
                return 2
            print(f'V{version}_REPLAY http://127.0.0.1:{args.port}', flush=True)
            wait_for_viewer(args.duration)
        finally:
            if viewer is not None:
                viewer._mpc_stop_event.set()
                for thread in viewer._mpc_workers:
                    thread.join(timeout=10)
                viewer._server.stop()
            worker.close()
    return 0


def wait_for_viewer(duration):
    began = time.monotonic()
    try:
        while duration <= 0 or time.monotonic()-began < duration:
            time.sleep(.25)
    except KeyboardInterrupt:
        pass


def main(*, run=run_case, resume=True, version="4.2", expected_wall_s=EXPECTED_WALL_S,
         watch_modes=watch):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-config', type=Path, default=Path.home()/'.cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--batch', action='store_true')
    mode.add_argument('--replay', action='store_true')
    mode.add_argument('--interactive', action='store_true')
    parser.add_argument('--interactive-smoke', action='store_true')
    parser.add_argument('--viewer-smoke', action='store_true')
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--port', type=int, default=int(os.environ.get('NEWTON_VIEWER_PORT', '30000')))
    parser.add_argument('--newton-python', type=Path, default=Path.home()/'newton_ws/.venv-cpu/bin/python')
    args = parser.parse_args()
    if args.viewer_smoke and not args.replay:
        parser.error("--viewer-smoke requires --replay")
    if args.interactive_smoke and not args.interactive:
        parser.error("--interactive-smoke requires --interactive")
    if args.replay or args.interactive:
        return watch_modes(args)
    if args.batch:
        return batch(args.robot_config, args.newton_python, run=run, resume=resume,
                     version=version, expected_wall_s=expected_wall_s)
    config = args.robot_config
    print(f'V{version} estimated qualification and smoke wall time: 2 minutes', flush=True)
    preflight = v41.preflight(config, save=False)
    v41.require_preflight(preflight)
    baseline = json.loads(tuned.original.BASELINE_FILE.read_text())
    settings = preflight['frozen_config']
    report = dict(floor_activation_m=FLOOR_ACTIVATION_M, rod_activation_m=ROD_ACTIVATION_M,
                  endpoint_floor_clearance_m=preflight['endpoint_floor_clearance_m'],
                  smoke_phase_m=.36, smoke_speed_mps=.05, gates={})
    mpc, _ = v41.make_mpc(config, settings, floor_activation_m=FLOOR_ACTIVATION_M)
    try:
        costs = endpoint_costs(mpc, baseline)
    finally:
        mpc.destroy()
    passed = all(value == 0. for methods in costs.values() for metrics in methods.values() for value in metrics.values())
    report['gates']['endpoint_costs'] = dict(passed=passed, poses=costs)
    tuned.save_json(REPORT, report)
    print(f'V{version} endpoint costs passed={passed}', flush=True)
    if not passed:
        return 2
    v2.ROD_START_X_M = report['smoke_phase_m']
    for name, mode, no_rod in (('qualification', 'A', True), ('smoke_A', 'A', False), ('smoke_B', 'B', False)):
        start = time.monotonic()
        result = run(config, settings, baseline, mode, no_rod=no_rod)
        result['wall_time_s'] = time.monotonic()-start
        report['gates'][name] = result
        tuned.save_json(REPORT, report)
        summary = result['case']['summary']
        print(f"V{version} {name} passed={result['passed']} reaches={summary['reaches']} "
              f"floor={summary['floor_contacts']} clearance={summary['minimum_floor_clearance_m']:.6f} "
              f"rod={summary['rod_collisions']} wall={result['wall_time_s']:.2f}s", flush=True)
        if not result['passed']:
            return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
