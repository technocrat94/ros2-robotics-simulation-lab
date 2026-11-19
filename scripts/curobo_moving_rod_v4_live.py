"""v4 live drag and five-group switching with isolated lifecycle ownership."""
from __future__ import annotations
from collections import deque
import json,math,tempfile,threading,time
from pathlib import Path
from typing import Any
import numpy as np
import torch
import curobo_moving_rod_v4 as v4
from curobo_moving_rod_v4_rules import ConservativeRodMemory,nonbase_mask,sample_held_pose
from curobo_moving_rod_v4_visuals import PredictionVisuals
from curobo_moving_rod_live import LiveGroups as OriginalLiveGroups,LiveCameraWorker,cylinder_clearance

v3=v4
tuned,v2=v4.tuned,v4.v2


class LiveGroups(OriginalLiveGroups):
    def __init__(self,args,baseline):
        self.selected_config=json.loads(v4.PREFLIGHT_FILE.read_text())["frozen_config"]
        try:
            super().__init__(args,baseline)
            self.selector.options=v4.GROUPS
            self.predicted.wireframe=True
            self.predicted.opacity=.85
            self.graphics=PredictionVisuals(self.server,json.loads(v4.PREFLIGHT_FILE.read_text())["floor_top_m"])
            self.speed_text=self.server.gui.add_text("Joint command speed","",disabled=True)
            self.server.gui.add_markdown("v4: 25 ms simulated clock, 30 Hz observations, config speed/acceleration bounds. Floor contact is scored. Dragging is external input.")
        except BaseException:
            if hasattr(self,"server") and not self.closed:self.close()
            raise

    def switch(self, mode: str) -> None:
        if mode not in v4.GROUPS:
            raise ValueError("unknown group")
        with self.switch_lock:
            if self.closed:
                return
            if self.current and self.current["group"]==mode and self.thread and self.thread.is_alive():
                return
            old = self._stop_old()
            if old:
                self.switches.append({"from":old["group"],"to":mode,
                    "old_thread_stopped":not self.thread.is_alive(),"old_worker_pid":old["worker_pid"],
                    "old_worker_exitcode":old["worker_exitcode"],"old_worker_stopped":old["worker_stopped"],
                    "old_stopped_at":old["stopped_at"],"new_requested_at":time.monotonic()})
            self.stop_event = threading.Event()
            record = {"group":mode,"state":"starting","started_at":time.monotonic(),
                      "steps":0,"worker_pid":None,"worker_exitcode":None,"worker_stopped":False,
                      "error":None,"initial_q":None,"final_q":None,"reset_error_rad":None,
                      "collision_episodes":0,"rate_hz":0.,"renderer_xy_checked":False}
            self.current = record
            self.history.append(record)
            self.group_text.value = mode
            self.state_text.value = "starting"
            self.error_text.value = "none"
            self.collisions_text.value = "0"
            self.depth.visible = mode.startswith("E")
            self.points.visible = mode.startswith("E")
            self.observed.visible = False
            self.predicted.visible = False
            self.points.points=np.zeros((1,3))
            if hasattr(self,"graphics"):
                self.graphics.update(None,np.zeros(3),np.zeros(3),.4+1/30,False,self.predicted)
            self.thread = threading.Thread(target=self._run,args=(mode,self.stop_event,record),
                        name=f"rod-live-{mode}-{len(self.history)}",daemon=False)
            self.thread.start()

    def _run(self, mode: str, stop: threading.Event, record: dict[str, Any]) -> None:
        mpc = None
        worker = None
        family=mode.split("-")[0]
        prediction=not mode.endswith("-nopred") and family!="A"
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
                    prior_executed_velocity=np.zeros(6)
                    if family=="E":
                        worker = LiveCameraWorker(self.args.newton_python,Path(temp))
                        worker.start()
                        record["worker_pid"] = worker.process.pid
                        worker.capture_xy(q,self.read_rod())
                        record["renderer_xy_checked"] = True
                    memory=ConservativeRodMemory(self.baseline["border_rule"]["fixed_unknown_top_m"])
                    mask=nonbase_mask(mpc.kinematics)
                    previous_true_center=self.read_rod()
                    floor_active=False
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
                        true_velocity=(center-previous_true_center)/v3.CONTROL_DT
                        previous_true_center=center.copy()
                        if pending_goal:
                            mpc.update_goal_tool_poses(raised if target=="raised_start" else goal,run_ik=False)
                            pending_goal = False
                        while family!="A" and observation_index/v3.OBS_HZ <= sim_clock+1e-9:
                            observed_time = observation_index/v3.OBS_HZ
                            observation_index += 1
                            fraction = np.clip((observed_time-prior_sim)/max(sim_clock-prior_sim,1e-9),0.,1.)
                            observed_q = sample_held_pose(prior_q,prior_executed_velocity,observed_time-prior_sim,positions)
                            observed_center = prior_center+(center-prior_center)*fraction
                            estimate = np.zeros(3)
                            if family=="B":
                                fit = {"center_m":observed_center.tolist(),"dims_m":[2*v2.ROD_RADIUS_M,2*v2.ROD_RADIUS_M,v2.ROD_TOP_M]}
                                fresh = fit
                            else:
                                capture = worker.capture_xy(observed_q.astype(np.float32),observed_center)
                                fit,estimate,points,event=memory.observe(capture.camera,observed_time)
                                record["top_rule_events"]=memory.top_events
                                record["growth_rule_events"]=memory.growth_events
                                continue
                            if fresh is not None:
                                box_center = np.asarray(fresh["center_m"])
                                if previous_frame is not None:
                                    estimate = (box_center-previous_frame[1])/(observed_time-previous_frame[0])
                                previous_frame = (observed_time,box_center)
                        prior_q,prior_center,prior_sim = q.copy(),center.copy(),sim_clock
                        predicted = v3.stretch_fit(fit,estimate,horizon+1/v3.OBS_HZ) if prediction else fit
                        tuned.update_box(mpc,predicted)
                        result = mpc.optimize_action_sequence(tuned.joint_state(q,velocity))
                        torch.cuda.synchronize()
                        commands = result.action_sequence.velocity.detach().cpu().numpy().reshape(-1,6)
                        done = time.monotonic()
                        prior_executed_velocity=velocity.copy()
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
                        floor_clearance=float(np.min(spheres[mask,2]-spheres[mask,3]-mpc._v4_floor_top))
                        if floor_clearance<0 and not floor_active:record["floor_contacts"]=record.get("floor_contacts",0)+1
                        floor_active=floor_clearance<0
                        clearance=min(clearance,floor_clearance)
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
                        if hasattr(self,"graphics"):
                            self.graphics.update(fit,estimate,true_velocity,horizon+1/v3.OBS_HZ,prediction,self.predicted)
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
                        record["prediction_horizon_s"] = horizon+1/v3.OBS_HZ if prediction else 0.
                        record["prediction"]=prediction
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
            print("V4_LIVE_GROUP_STOPPED "+json.dumps(record),flush=True)


def smoke(app):
    began=time.monotonic()
    observations=[]
    try:
        for mode in ("A","B","B-nopred","E","E-nopred","A"):
            app.gizmo.position=(.52,.22,v2.ROD_TOP_M/2)
            app.read_rod()
            app.selector.value=mode
            deadline=time.monotonic()+15
            while True:
                record=app.current
                if record and record["error"]:raise RuntimeError(record["error"])
                if record and record["group"]==mode and record["steps"]>=8:
                    observations.append(dict(group=mode,steps=record['steps'],rate_hz=record['rate_hz']))
                    break
                if time.monotonic()>deadline:raise RuntimeError('v4 live group startup timed out')
                time.sleep(.025)
    finally:app.close()
    report=dict(elapsed_s=time.monotonic()-began,observations=observations,groups=app.history,switches=app.switches,
        max_concurrent_gpu_loops=app.max_active_loops,all_threads_stopped=not app.thread.is_alive(),
        all_workers_stopped=all(x['worker_stopped'] for x in app.history),writes_results=False)
    report['observation_counts_pass']=all(x['observation_count']==(0 if x['group']=='A' else math.floor((x['steps']-1)*.025*30+1e-8)+1) for x in app.history)
    report['all_passed']=bool(report['observation_counts_pass'] and app.max_active_loops==1 and report['all_threads_stopped'] and report['all_workers_stopped']
        and len(app.switches)==5 and all(x['error'] is None and x['reset_error_rad']==0 and x.get('max_speed_radps',0)<=np.pi+2e-7
        and x.get('max_acceleration_radps2',0)<=10+1e-6 for x in app.history)
        and all(x['old_thread_stopped'] and x['old_worker_stopped'] and x['old_stopped_at']<=x['new_requested_at'] for x in app.switches))
    return report


def live_main(args):
    results=json.loads(v4.RESULT_FILE.read_text())
    pf=json.loads(v4.PREFLIGHT_FILE.read_text())
    if not results.get('complete') or len(results['cases'])!=75:raise RuntimeError('finish v4 measurement before live mode')
    if pf['robot_config_sha256']!=v4.hashlib.sha256(args.robot_config.read_bytes()).hexdigest():raise RuntimeError('robot config changed')
    baseline=json.loads(tuned.original.BASELINE_FILE.read_text());tuned.original.configure_scene(baseline)
    app=None;began=time.monotonic()
    try:
        app=LiveGroups(args,baseline)
        if args.interactive_smoke:
            report=smoke(app);print('V4_LIVE_SMOKE '+json.dumps(report),flush=True)
            return 0 if report['all_passed'] else 2
        print(f'V4_LIVE_URL http://127.0.0.1:{args.port}',flush=True)
        while args.duration<=0 or time.monotonic()-began<args.duration:time.sleep(.1)
    except KeyboardInterrupt:pass
    finally:
        if app is not None and not app.closed:app.close()
    return 0
