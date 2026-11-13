#!/usr/bin/env python3
"""Separate perception from counted latency with the frozen MPC and scene."""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time
import numpy as np
import curobo_moving_rod_rates as rates

tuned, v2 = rates.tuned, rates.v2
GROUPS = ("A20", "B20", "B50", "E20", "E50")
RESULT_FILE = v2.RESULTS_DIR / "moving_rod_latency.json"


def summarize(results):
    rows, table = [], []
    for speed in rates.SPEEDS:
        for phase in rates.PHASES:
            cases = {g: next(c for c in results["cases"].values()
                            if c["summary"]["mode"] == g and c["summary"]["rod_speed_mps"] == speed
                            and c["summary"]["initial_x_m"] == phase) for g in GROUPS}
            valid = cases["A20"]["summary"]["true_collisions"] > 0
            status = "VALID_SCENE" if valid else "INVALID_SCENE"
            for c in cases.values():
                c["summary"]["scene_status"] = status
            rows.append({"speed_mps": speed, "initial_x_m": phase, "status": status,
                         "cases": {g:c["summary"] for g,c in cases.items()}})
        for group in GROUPS:
            cs = [c for c in results["cases"].values() if c["summary"]["mode"] == group
                  and c["summary"]["rod_speed_mps"] == speed
                  and c["summary"]["scene_status"] == "VALID_SCENE"]
            measured = [step["measured_compute_ms"] for c in cs for step in c["cycles"]]
            successes = sum(c["summary"]["survives"] for c in cs)
            table.append({"group": group, "speed_mps": speed, "successes": successes,
                          "valid_phases": len(cs), "invalid_phases": len(rates.PHASES)-len(cs),
                          "collision_episodes": sum(c["summary"]["true_collisions"] for c in cs),
                          "worst_clearance_m": min((c["summary"]["minimum_true_clearance_m"] for c in cs), default=None),
                          "real_compute_median_ms": float(np.median(measured)) if measured else None,
                          "real_compute_p95_ms": float(np.percentile(measured,95)) if measured else None})
    results["per_phase"], results["rates"] = rows, table
    results["repeatability"] = []
    for group,c in results["repeats"].items():
        first = next(c0 for c0 in results["cases"].values() if c0["summary"]["mode"] == group
                     and c0["summary"]["rod_speed_mps"] == .10 and c0["summary"]["initial_x_m"] == .60)
        a,b = first["summary"], c["summary"]
        results["repeatability"].append({"group": group, "speed_mps": .10, "initial_x_m": .60,
            "first_success": a["survives"], "repeat_success": b["survives"],
            "first_worst_clearance_m": a["minimum_true_clearance_m"],
            "repeat_worst_clearance_m": b["minimum_true_clearance_m"],
            "clearance_delta_m": b["minimum_true_clearance_m"]-a["minimum_true_clearance_m"]})


def build(config, worker, checkpoint):
    baseline = json.loads(tuned.original.BASELINE_FILE.read_text())
    reference = json.loads(tuned.REFERENCE_FILE.read_text())
    if reference["selected_config"] != tuned.NO_ROD_SELECTED_MPC:
        raise RuntimeError("frozen qualification mismatch")
    tuned.original.configure_scene(baseline)
    start = np.array(baseline["raised_start"]["joints_rad"], dtype=np.float32)
    mpc, loaded = tuned.make_mpc(config, tuned.NO_ROD_SELECTED_MPC)
    results = {"schema": 1, "date": "2026-11-13", "complete": False,
        "frozen_config": tuned.NO_ROD_SELECTED_MPC, "loaded_config": loaded,
        "reference_sha256": hashlib.sha256(tuned.REFERENCE_FILE.read_bytes()).hexdigest(),
        "rod": baseline["rod"], "raised_start": baseline["raised_start"],
        "groups": GROUPS, "phases_m": rates.PHASES, "speeds_mps": rates.SPEEDS,
        "duration_per_case_s": 20., "initial_direction": -1,
        "clock": "fixed counted compute replaces measured compute in advance, index, horizon and pacing; real compute may pause simulated world",
        "case_order": "speed ascending, phase ascending, A20 B20 B50 E20 E50; then B20/E20 repeat at 0.10 X=0.60",
        "camera_memory": "unchanged v2 TimedPointCloud and fresh-frame velocity; reset each case",
        "cases": {}, "repeats": {}}
    def run(group, speed, phase):
        v2.ROD_START_X_M = phase
        if group.startswith("E") and worker.process is None:
            worker.start()
            worker.capture(start, phase)
        print(f"LATENCY_CASE_START {group} {speed:.2f} X={phase:.2f}", flush=True)
        c = rates.run_case(mpc, start, speed, True, group[0], worker, baseline,
                           counted_latency_s=int(group[1:])/1000.)
        c["summary"].update(mode=group, perception_mode=group[0], initial_x_m=phase,
                             counted_latency_s=int(group[1:])/1000.)
        return c
    try:
        mpc.setup(rates.joint_state(start, np.zeros(6)))
        for speed in rates.SPEEDS:
            for phase in rates.PHASES:
                for group in GROUPS:
                    name = f"Latency {group} {speed:.2f} m/s X={phase:.2f}"
                    results["cases"][name] = run(group, speed, phase)
                    if checkpoint:
                        tuned.save_json(RESULT_FILE, results)
        for group in ("B20", "E20"):
            results["repeats"][group] = run(group, .10, .60)
        summarize(results)
        results["complete"] = True
        return results
    finally:
        mpc.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--replay", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--save-results", action="store_true")
    parser.add_argument("--duration", type=float, default=0.)
    parser.add_argument("--port", type=int, default=int(os.environ.get("NEWTON_VIEWER_PORT", "30000")))
    parser.add_argument("--robot-config", type=Path, default=Path.home()/".cache/ur5_moveit_demo/curobo_v2/ur5_robotiq_curobo.yml")
    parser.add_argument("--newton-python", type=Path, default=Path.home()/"newton_ws/.venv-cpu/bin/python")
    args = parser.parse_args()
    if args.replay and args.save_results:
        parser.error("--replay cannot write results")
    viewer = None
    with tempfile.TemporaryDirectory(prefix="moving-rod-latency-") as temp:
        worker = v2.MovingRodWorker(args.newton_python, Path(temp))
        try:
            results = json.loads(RESULT_FILE.read_text()) if args.replay else build(args.robot_config, worker, args.save_results)
            if not results["complete"]:
                raise RuntimeError("incomplete latency checkpoint")
            if args.save_results:
                tuned.save_json(RESULT_FILE, results)
                RESULT_FILE.with_suffix(".log").write_text(json.dumps({k:results[k] for k in ("clock","rates","per_phase","repeatability")},indent=2)+"\n")
                print("LATENCY_RESULTS_SAVED", flush=True)
            if not args.headless:
                tuned.original.configure_scene(results)
                frames = rates.render_frames(worker, results)
                worker.close()
                viewer, report = rates.start_viewer(args.robot_config, results, frames, args.port, GROUPS)
                if not report["all_passed"]:
                    raise RuntimeError("latency viewer verification failed")
                began = time.monotonic()
                while args.duration <= 0 or time.monotonic()-began < args.duration:
                    time.sleep(.25)
        finally:
            worker.close()
            if viewer is not None:
                viewer._mpc_stop_event.set()
                for thread in viewer._mpc_workers:
                    thread.join(timeout=5)
                viewer._server.stop()
    if not args.save_results:
        print("LATENCY_RESULTS_NOT_SAVED", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
