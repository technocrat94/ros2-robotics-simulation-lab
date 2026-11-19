#!/usr/bin/env python3
"""Run the v4.2 gates with a floor-only 10 mm MPC activation distance."""
import argparse
import json
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
BATCH = v2.RESULTS_DIR / 'moving_rod_v42.json'
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
             guard=v41.floor_guard):
    start = np.asarray(baseline['raised_start']['joints_rad'], dtype=np.float32)
    mpc, _ = v41.make_mpc(config, settings, floor_activation_m=FLOOR_ACTIVATION_M)
    try:
        mpc.setup(v4.joint_state(start, np.zeros(6)))
        case = v4.run_case(
            mpc, start, 0. if no_rod else speed, not no_rod and not mode.endswith('-nopred'),
            mode, worker, baseline,
            duration=v41.QUALIFICATION_DURATION_S if no_rod else 20.,
            no_rod=no_rod, stop_at_goal=no_rod,
            floor_mask=v41.nonbase_mask(mpc.kinematics), step_guard=guard,
        )
    finally:
        mpc.destroy()
    case['summary']['initial_x_m'] = v2.ROD_START_X_M
    result = {'goal': v41.goal_distances(case['distance_to_goal']), 'case': v4.compact_case(case)}
    summary = result['case']['summary']
    result['passed'] = (summary['survives'] if no_rod or mode == 'B' else summary['floor_contacts'] == 0)
    result['passed'] = bool(result['passed'] and summary['stop_reason'] is None)
    return result


def checkpoint(report):
    temp = BATCH.with_suffix('.tmp')
    temp.write_text(json.dumps(report, separators=(',', ':'), allow_nan=False)+'\n')
    temp.replace(BATCH)


def batch(config, newton_python):
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
    expected_s = sum(EXPECTED_WALL_S[mode] for _, _, mode in matrix)
    print(f'V42 estimated batch wall time: {expected_s/60:.0f} minutes for {len(matrix)} runs', flush=True)
    report = dict(schema='4.2', complete=False, stop_reason=None, preflight=preflight,
                  floor_activation_m=FLOOR_ACTIVATION_M, rod_activation_m=ROD_ACTIVATION_M,
                  rod=baseline['rod'], raised_start=baseline['raised_start'], cases={},
                  expected_wall_s_by_mode=EXPECTED_WALL_S, expected_total_wall_s=expected_s)
    checkpoint(report)
    first_b = []

    def timeout(_signal, _frame):
        raise TimeoutError('run exceeded 3x expected wall time')

    def guard(clearance):
        if clearance < -.020:
            return f'floor penetration {-clearance*1000:.3f} mm exceeds 20 mm'
        return None

    old_handler = signal.signal(signal.SIGALRM, timeout)
    try:
        with tempfile.TemporaryDirectory(prefix='rod-v42-') as temp:
            worker = v2.MovingRodWorker(newton_python, Path(temp))
            try:
                for speed, phase, mode in matrix:
                    v2.ROD_START_X_M = phase
                    name = f'v4.2 {mode} {speed:.2f} m/s X={phase:.2f}'
                    began = time.monotonic()
                    signal.setitimer(signal.ITIMER_REAL, 3*EXPECTED_WALL_S[mode])
                    try:
                        if mode.startswith('E') and worker.process is None:
                            worker.start()
                            worker.capture(np.asarray(baseline['raised_start']['joints_rad']), phase)
                        outcome = run_case(config, preflight['frozen_config'], baseline, mode,
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
                    print(f'V42_CASE {len(report["cases"]):02d}/75 {name} '
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
        report['complete'] = True
    checkpoint(report)
    BATCH.with_suffix('.log').write_text(json.dumps(
        {key: report[key] for key in ('complete', 'stop_reason', 'rates') if key in report}, indent=2)+'\n')
    print(f'V42_BATCH complete={report["complete"]} stop={report["stop_reason"]} cases={len(report["cases"])}', flush=True)
    return 0 if report['complete'] else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-config', type=Path, default=Path.home()/'.cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml')
    parser.add_argument('--batch', action='store_true')
    parser.add_argument('--newton-python', type=Path, default=Path.home()/'newton_ws/.venv-cpu/bin/python')
    args = parser.parse_args()
    if args.batch:
        return batch(args.robot_config, args.newton_python)
    config = args.robot_config
    print('V42 estimated qualification and smoke wall time: 2 minutes', flush=True)
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
    print(f'V42 endpoint costs passed={passed}', flush=True)
    if not passed:
        return 2
    v2.ROD_START_X_M = report['smoke_phase_m']
    for name, mode, no_rod in (('qualification', 'A', True), ('smoke_A', 'A', False), ('smoke_B', 'B', False)):
        start = time.monotonic()
        result = run_case(config, settings, baseline, mode, no_rod=no_rod)
        result['wall_time_s'] = time.monotonic()-start
        report['gates'][name] = result
        tuned.save_json(REPORT, report)
        summary = result['case']['summary']
        print(f"V42 {name} passed={result['passed']} reaches={summary['reaches']} "
              f"floor={summary['floor_contacts']} clearance={summary['minimum_floor_clearance_m']:.6f} "
              f"rod={summary['rod_collisions']} wall={result['wall_time_s']:.2f}s", flush=True)
        if not result['passed']:
            return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
