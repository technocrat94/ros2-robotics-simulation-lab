"""v3 interactive loop with the existing serialized group/worker lifecycle."""
from __future__ import annotations
from collections import deque
import json
import math
from pathlib import Path
import tempfile
import threading
import time
from typing import Any
import numpy as np
import torch
import curobo_moving_rod_v3 as v3
from curobo_moving_rod_live import LiveGroups as OriginalLiveGroups, LiveCameraWorker, cylinder_clearance, smoke

tuned,v2 = v3.tuned,v3.v2


class LiveGroups(OriginalLiveGroups):
    def __init__(self,args,baseline):
        ref=json.loads(v3.REFERENCE_FILE.read_text())
        self.selected_config=ref["selected_config"]
        super().__init__(args,baseline)
        self.speed_text=self.server.gui.add_text("Joint speed","",disabled=True)
        self.server.gui.add_markdown("v3: fixed 25 ms simulated steps, 30 Hz observations, config acceleration limit. Simulated time slows when compute is longer.")

    def _run(self, mode: str, stop: threading.Event, record: dict[str, Any]) -> None:
        mpc = None
        worker = None
        q = self.start_q.copy()
        with self.state_lock:
            self.active_loops += 1
            self.max_active_loops = max(self.max_active_loops,self.active_loops)
        try:
            with tempfile.TemporaryDirectory(prefix="rod-live-camera-") as temp:
                try:
                    # The old thread is already joined before this reset or any GPU setup.
                    self.viewer.set_joint_state(v2.sixth.arm_state(q).squeeze(0))
                    record["initial_q"] = q.tolist()
                    record["reset_error_rad"] = float(np.max(np.abs(q-self.start_q)))
                    mpc,_ = v3.make_mpc(self.args.robot_config,self.selected_config)
                    velocity = np.zeros(6)
                    state = tuned.joint_state(q,velocity)
                    mpc.setup(state)
                    raised = mpc.compute_kinematics(state).tool_poses.as_goal(mpc.tool_frames)
                    goal = v2.sixth.goal_from_scene()
                    mpc.update_goal_tool_poses(goal,run_ik=False)
                    mpc.optimize_action_sequence(state)
                    targets = {"goal":goal.position.detach().cpu().numpy().reshape(-1,3)[0],
                               "raised_start":v2.sixth.fourth.tool_position(mpc.kinematics,state.position)}
                    limits = mpc.kinematics.get_joint_limits()
                    positions = limits.position.detach().cpu().numpy()
                    speeds = limits.velocity.detach().cpu().numpy()
                    accel_limits = limits.acceleration.detach().cpu().numpy()
                    horizon = v3.horizon_seconds(mpc)
                    sim_clock = 0.
                    observation_index = 0
                    fit = None
                    estimate = np.zeros(3)
                    capture = None
                    points = None
                    prior_q = q.copy()
                    prior_center = self.read_rod()
                    prior_sim = 0.
                    if mode=="E":
                        worker = LiveCameraWorker(self.args.newton_python,Path(temp))
                        worker.start()
                        record["worker_pid"] = worker.process.pid
                        worker.capture_xy(q,self.read_rod())
                        record["renderer_xy_checked"] = True
                    cloud = v2.TimedPointCloud()
                    previous_frame = None
                    previous_truth = None
                    expected_compute = .025
                    pending_goal = False
                    target = "goal"
                    collision_active = False
                    periods: deque[float] = deque(maxlen=40)
                    running_since = last_update = previous_done = time.monotonic()
                    if stop.is_set():
                        return
                    record["state"] = "running"
                    record["running_since"] = running_since
                    self.state_text.value = "running"
                    while not stop.is_set():
                        tick = time.monotonic()
                        center = self.read_rod()
                        if pending_goal:
                            mpc.update_goal_tool_poses(raised if target=="raised_start" else goal,run_ik=False)
                            pending_goal = False
                        while mode!="A" and observation_index/v3.OBS_HZ <= sim_clock+1e-9:
                            observed_time = observation_index/v3.OBS_HZ
                            observation_index += 1
                            fraction = np.clip((observed_time-prior_sim)/max(sim_clock-prior_sim,1e-9),0.,1.)
                            observed_q = prior_q+(q-prior_q)*fraction
                            observed_center = prior_center+(center-prior_center)*fraction
                            estimate = np.zeros(3)
                            if mode=="B":
                                fit = {"center_m":observed_center.tolist(),"dims_m":[2*v2.ROD_RADIUS_M,2*v2.ROD_RADIUS_M,v2.ROD_TOP_M]}
                                fresh = fit
                            else:
                                capture = worker.capture_xy(observed_q.astype(np.float32),observed_center)
                                points,_ = cloud.update(capture.camera,observed_time)
                                fixed_top = self.baseline["border_rule"]["fixed_unknown_top_m"]
                                fit,_ = v2.sixth.fit_with_unknown_extent(points,capture.camera,fixed_top)
                                fresh,_ = v2.sixth.fit_with_unknown_extent(capture.camera.points,capture.camera,fixed_top)
                            if fresh is not None:
                                box_center = np.asarray(fresh["center_m"])
                                if previous_frame is not None:
                                    estimate = (box_center-previous_frame[1])/(observed_time-previous_frame[0])
                                previous_frame = (observed_time,box_center)
                        prior_q,prior_center,prior_sim = q.copy(),center.copy(),sim_clock
                        predicted = v3.stretch_fit(fit,estimate,horizon+1/v3.OBS_HZ) if mode!="A" else None
                        tuned.update_box(mpc,predicted)
                        result = mpc.optimize_action_sequence(tuned.joint_state(q,velocity))
                        torch.cuda.synchronize()
                        commands = result.action_sequence.velocity.detach().cpu().numpy().reshape(-1,6)
                        done = time.monotonic()
                        _,path = tuned.integrate_command(q,velocity,v3.CONTROL_DT,positions)
                        if len(path):q = path[-1]
                        last_update = done
                        sim_clock += v3.CONTROL_DT
                        index = min(max(1,int(math.ceil(v3.CONTROL_DT/(mpc.config.optimization_dt/4)))),len(commands)-1)
                        if not np.isfinite(commands[index]).all():
                            raise RuntimeError("non-finite live MPC velocity")
                        old_velocity = velocity.copy()
                        velocity = v3.limited_command(commands[index],velocity,speeds,accel_limits)
                        record["max_speed_radps"] = max(record.get("max_speed_radps",0.),float(np.max(np.abs(velocity))))
                        record["max_acceleration_radps2"] = max(record.get("max_acceleration_radps2",0.),float(np.max(np.abs(velocity-old_velocity))/v3.CONTROL_DT))
                        state = tuned.joint_state(q,velocity)
                        kin = mpc.compute_kinematics(state)
                        spheres = kin.robot_spheres.detach().cpu().numpy().reshape(-1,4)
                        clearance = cylinder_clearance(spheres,self.read_rod())
                        colliding = clearance<0
                        if colliding and not collision_active:
                            record["collision_episodes"] += 1
                        collision_active = colliding
                        tool = v2.sixth.fourth.tool_position(mpc.kinematics,state.position)
                        if np.linalg.norm(tool-targets[target]) <= v2.sixth.GOAL_TOLERANCE_M:
                            target = "raised_start" if target=="goal" else "goal"
                            pending_goal = True
                        self.viewer.set_joint_state(state.squeeze(0))
                        camera_position,camera_wxyz = v2.sixth.fourth.camera_pose_for_q(mpc.kinematics,state.position,self.mount)
                        self.frustum.position,self.frustum.wxyz = tuple(camera_position),tuple(camera_wxyz)
                        for box,data in ((self.observed,fit),(self.predicted,predicted)):
                            box.visible = data is not None
                            if data:
                                box.position,box.dimensions = tuple(data["center_m"]),tuple(data["dims_m"])
                        if capture is not None:
                            self.points.points = points if len(points) else np.zeros((1,3))
                            self.depth.image = capture.depth_preview
                        periods.append(done-previous_done)
                        previous_done = done
                        record["steps"] += 1
                        record["rate_hz"] = len(periods)/sum(periods)
                        record["live_elapsed_s"] = done-running_since
                        record["final_q"] = q.tolist()
                        self.clearance_text.value = f"{clearance:+.4f} m"
                        self.collisions_text.value = str(record["collision_episodes"])
                        self.rate_text.value = f"{record['rate_hz']:.2f} Hz"
                        self.clock_text.value = f"sim {sim_clock:.3f} s; wall {done-running_since:.3f} s"
                        if hasattr(self,"speed_text"):self.speed_text.value = f"{np.max(np.abs(velocity)):.3f} rad/s"
                        record["sim_time_s"] = sim_clock
                        record["counted_compute_s"] = v3.CONTROL_DT
                        record["observation_count"] = observation_index
                        record["prediction_horizon_s"] = horizon+1/v3.OBS_HZ
                        expected_compute = done-tick
                        stop.wait(max(0.,tuned.CONTROL_DT-(time.monotonic()-tick)))
                finally:
                    if worker is not None:
                        worker.close()
                        record["worker_exitcode"] = worker.process.poll() if worker.process is not None else None
                    record["worker_stopped"] = worker is None or worker.process is None or worker.process.poll() is not None
                    if mpc is not None:
                        mpc.destroy()
                        torch.cuda.synchronize()
        except Exception as exc:
            record["error"] = f"{type(exc).__name__}: {exc}"
            self.error_text.value = record["error"]
        finally:
            record["final_q"] = q.tolist()
            record["state"] = "stopped"
            record["stopped_at"] = time.monotonic()
            self.state_text.value = "stopped"
            with self.state_lock:
                self.active_loops -= 1
            print("ROD_LIVE_GROUP_STOPPED "+json.dumps(record),flush=True)


def live_main(args):
    results=json.loads(v3.RESULT_FILE.read_text())
    if not results.get("complete") or len(results["cases"])!=110:
        raise RuntimeError("complete v3 measurement before interactive mode")
    ref=json.loads(v3.REFERENCE_FILE.read_text())
    if ref["robot_config_sha256"]!=v3.hashlib.sha256(args.robot_config.read_bytes()).hexdigest():
        raise RuntimeError("v3 robot config differs from qualification")
    if ref["selected_config"]!=results["frozen_config"]:
        raise RuntimeError("v3 frozen config mismatch")
    baseline=json.loads(tuned.original.BASELINE_FILE.read_text())
    tuned.original.configure_scene(baseline)
    began=time.monotonic()
    app=None
    try:
        app=LiveGroups(args,baseline)
        if args.interactive_smoke:
            report=smoke(app,began)
            report["frozen_config"]=ref["selected_config"]
            report["v3_rules"]=dict(counted_compute_s=.025,observation_hz=30)
            report["limits_pass"] = all(x.get("max_speed_radps",0.)<=math.pi+2e-7 and x.get("max_acceleration_radps2",0.)<=10.+1e-6 for x in app.history)
            report["observation_counts_pass"] = all(x["observation_count"] == (0 if x["group"]=="A" else math.floor((x["steps"]-1)*.025*30+1e-8)+1) for x in app.history)
            report["all_passed"] = report["all_passed"] and report["limits_pass"] and report["observation_counts_pass"]
            print("V3_LIVE_SMOKE "+json.dumps(report),flush=True)
            return 0 if report["all_passed"] else 2
        while args.duration<=0 or time.monotonic()-began<args.duration:time.sleep(.1)
    except KeyboardInterrupt:
        pass
    finally:
        if app is not None and not app.closed:app.close()
    return 0
