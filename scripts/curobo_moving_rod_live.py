#!/usr/bin/env python3
"""Real-time, write-free rod dragging with serialized A/B/E group lifecycles."""
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
from curobo.types import ContentPath
from curobo.viewer import ViserVisualizer

import curobo_moving_rod_mpc_tuned as tuned

v2 = tuned.v2


def floor_center(position: Any) -> np.ndarray:
    """Keep the unchanged cylinder upright and standing on the floor."""
    center = np.asarray(position, dtype=float).copy()
    if center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError("rod translation must contain three finite coordinates")
    center[2] = v2.ROD_TOP_M/2
    return center


def cylinder_clearance(spheres: np.ndarray, center: np.ndarray) -> float:
    """Use the same sphere/cylinder signed distance as the offline scorer."""
    radial = np.linalg.norm(spheres[:,:2]-center[:2], axis=1)-v2.ROD_RADIUS_M
    axial = np.abs(spheres[:,2]-center[2])-v2.ROD_TOP_M/2
    pair = np.stack((radial,axial),axis=-1)
    signed = np.linalg.norm(np.maximum(pair,0),axis=-1)+np.minimum(np.max(pair,axis=-1),0)-spheres[:,3]
    return float(np.min(signed))


class LiveCameraWorker(v2.MovingRodWorker):
    """Use the existing renderer protocol with optional per-capture rod Y."""

    def capture_xy(self, q: np.ndarray, center: np.ndarray) -> v2.RodCapture:
        if self.process is None:
            self.start()
        self.counter += 1
        output = self.temp_dir/f"live_{self.counter:06d}.npz"
        request = {"id":self.counter,"arm_q":q.tolist(),"rod_x_m":float(center[0]),
                   "rod_y_m":float(center[1]),"output":str(output)}
        self.process.stdin.write(json.dumps(request)+"\n")
        self.process.stdin.flush()
        metadata = json.loads(self._read_prefix("NEWTON_MOVING_ROD_WORKER_RESPONSE "))
        if metadata["scene_builds"] != 1:
            raise RuntimeError("live renderer rebuilt its scene")
        if not np.allclose(metadata["rod_body_m"],center,atol=1e-6):
            raise RuntimeError("renderer did not apply the live XY rod pose")
        with np.load(output) as arrays:
            capture = v2.sixth.fourth.Capture(q=q.astype(np.float32),points=arrays["points"].copy(),
                        camera_transform=arrays["camera_transform"].copy(),metadata=metadata)
            depth = arrays["depth_preview"].copy()
        output.unlink()
        return v2.RodCapture(capture,depth)


class LiveGroups:
    """Own only this viewer's loop threads, controller instances and camera workers."""

    def __init__(self, args: Any, baseline: dict[str, Any]):
        self.args, self.baseline = args, baseline
        self.start_q = np.array(baseline["raised_start"]["joints_rad"],dtype=np.float32)
        self.switch_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.current: dict[str, Any] | None = None
        self.history: list[dict[str, Any]] = []
        self.switches: list[dict[str, Any]] = []
        self.closed = False
        self.active_loops = 0
        self.max_active_loops = 0
        self.viewer = ViserVisualizer(content_path=ContentPath(robot_config_file=str(args.robot_config)),
            connect_ip="127.0.0.1",connect_port=args.port,add_control_frames=True,
            visualize_robot_spheres=False,add_robot_to_scene=True)
        server = self.viewer._server
        if int(server._websock_server._port) != args.port:
            server.stop()
            raise RuntimeError(f"personal port {args.port} occupied; refusing fallback")
        self.server = server
        center = (.60,v2.ROD_Y_M,v2.ROD_TOP_M/2)
        self.rod = server.scene.add_cylinder("/truth/rod",radius=v2.ROD_RADIUS_M,
                        height=v2.ROD_TOP_M,color=(235,55,45),position=center)
        self.gizmo = server.scene.add_transform_controls("/controls/rod",position=center,scale=.12,
                active_axes=(True,True,False),disable_rotations=True,
                translation_limits=((-1000.,1000.),(-1000.,1000.),(center[2],center[2])))
        self.points = server.scene.add_point_cloud("/camera/points",points=np.zeros((1,3)),
                        colors=(220,0,220),point_size=.011)
        self.observed = server.scene.add_box("/observed/box",dimensions=(.01,.01,.01),color=(255,145,35),opacity=.42)
        self.predicted = server.scene.add_box("/predicted/box",dimensions=(.01,.01,.01),color=(80,150,255),opacity=.30)
        self.frustum = server.scene.add_camera_frustum("/camera/live",fov=v2.DEPTH_FOV_RAD,
                        aspect=v2.DEPTH_ASPECT,scale=.13)
        self.depth = server.gui.add_image(np.zeros((120,212,3),dtype=np.uint8),label="Live E depth")
        server.gui.add_markdown("Drag the rod on the floor. A: no obstacle; B: true box; E: camera. Same frozen MPC.")
        self.selector = server.gui.add_dropdown("Group",options=("A","B","E"),initial_value="A")
        self.group_text = server.gui.add_text("Running group","A",disabled=True)
        self.state_text = server.gui.add_text("Group state","stopped",disabled=True)
        self.clearance_text = server.gui.add_text("Truth clearance","",disabled=True)
        self.collisions_text = server.gui.add_text("Collisions since group start","0",disabled=True)
        self.rate_text = server.gui.add_text("Controller rate","0 Hz",disabled=True)
        self.clock_text = server.gui.add_text("Live elapsed","0 s",disabled=True)
        self.error_text = server.gui.add_text("Error","none",disabled=True)
        self.mount = v2.sixth.fourth.read_mount_config(v2.MOUNT_CONFIG)
        self.selector.on_update(lambda _event:self.switch(self.selector.value))
        self.gizmo.on_update(lambda _event:self.read_rod())
        self.switch("A")
        print(f"ROD_LIVE_URL http://127.0.0.1:{args.port}",flush=True)

    def read_rod(self) -> np.ndarray:
        center = floor_center(self.gizmo.position)
        self.gizmo.position = tuple(center)
        self.gizmo.wxyz = (1.,0.,0.,0.)
        self.rod.position = tuple(center)
        self.rod.wxyz = (1.,0.,0.,0.)
        return center

    def _stop_old(self) -> dict[str, Any] | None:
        if self.thread is None:
            return None
        self.stop_event.set()
        self.thread.join(timeout=10)
        if self.thread.is_alive():
            raise RuntimeError("old live control loop did not exit; refusing a second GPU loop")
        record = self.current
        if record is not None and not record.get("worker_stopped",False):
            raise RuntimeError("old camera worker did not exit; refusing group switch")
        return record

    def switch(self, mode: str) -> None:
        if mode not in ("A","B","E"):
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
            self.depth.visible = mode=="E"
            self.points.visible = mode=="E"
            self.observed.visible = False
            self.predicted.visible = False
            self.thread = threading.Thread(target=self._run,args=(mode,self.stop_event,record),
                        name=f"rod-live-{mode}-{len(self.history)}",daemon=False)
            self.thread.start()

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
                    mpc,_ = tuned.make_mpc(self.args.robot_config,tuned.NO_ROD_SELECTED_MPC)
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
                        _,path = tuned.integrate_command(q,velocity,tick-last_update,positions)
                        if len(path):q = path[-1]
                        last_update = tick
                        center = self.read_rod()
                        estimate = np.zeros(3)
                        capture = None
                        points = None
                        if pending_goal:
                            mpc.update_goal_tool_poses(raised if target=="raised_start" else goal,run_ik=False)
                            pending_goal = False
                        if mode=="A":
                            fit = None
                        elif mode=="B":
                            fit = {"center_m":center.tolist(),"dims_m":[2*v2.ROD_RADIUS_M,2*v2.ROD_RADIUS_M,v2.ROD_TOP_M]}
                            if previous_truth is not None and tick>previous_truth[0]:
                                estimate = (center-previous_truth[1])/(tick-previous_truth[0])
                            previous_truth = (tick,center.copy())
                        else:
                            capture = worker.capture_xy(q.astype(np.float32),center)
                            points,_ = cloud.update(capture.camera,tick-running_since)
                            fixed_top = self.baseline["border_rule"]["fixed_unknown_top_m"]
                            fit,_ = v2.sixth.fit_with_unknown_extent(points,capture.camera,fixed_top)
                            fresh,_ = v2.sixth.fit_with_unknown_extent(capture.camera.points,capture.camera,fixed_top)
                            if fresh is not None:
                                box_center = np.asarray(fresh["center_m"])
                                if previous_frame is not None and tick>previous_frame[0]:
                                    estimate = (box_center-previous_frame[1])/(tick-previous_frame[0])
                                previous_frame = (tick,box_center)
                        predicted = v2.predicted_fit(fit,estimate,expected_compute+tuned.CONTROL_DT) if mode!="A" else None
                        tuned.update_box(mpc,predicted)
                        result = mpc.optimize_action_sequence(tuned.joint_state(q,velocity))
                        torch.cuda.synchronize()
                        commands = result.action_sequence.velocity.detach().cpu().numpy().reshape(-1,6)
                        done = time.monotonic()
                        _,path = tuned.integrate_command(q,velocity,done-last_update,positions)
                        if len(path):q = path[-1]
                        last_update = done
                        index = min(max(1,int(math.ceil((done-tick)/(mpc.config.optimization_dt/4)))),len(commands)-1)
                        if not np.isfinite(commands[index]).all():
                            raise RuntimeError("non-finite live MPC velocity")
                        velocity = np.clip(commands[index],speeds[0],speeds[1])
                        velocity = np.where(((q<=positions[0])&(velocity<0))|((q>=positions[1])&(velocity>0)),0,velocity)
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
                        self.clock_text.value = f"{done-running_since:.3f} s"
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

    def close(self) -> None:
        with self.switch_lock:
            self.closed = True
            self._stop_old()
        self.server.stop()


def smoke(app: LiveGroups, began: float) -> dict[str, Any]:
    """Exercise actual dropdown callbacks and wait for clean serial transitions."""
    deadline = began+25.
    observations = []
    for mode in ("A","E","B","A"):
        app.gizmo.position = (.52,.22,v2.ROD_TOP_M/2)
        app.read_rod()
        app.selector.value = mode  # Viser invokes the real callback synchronously.
        while True:
            record = app.current
            if record and record["error"]:
                raise RuntimeError(record["error"])
            if record and record["group"]==mode and record["state"]=="running" and record["steps"]>=8:
                observations.append({"group":mode,"steps":record["steps"],"rate_hz":record["rate_hz"],
                                     "reset_error_rad":record["reset_error_rad"]})
                break
            if time.monotonic()>=deadline:
                raise RuntimeError("live smoke did not finish switching within 25 seconds")
            time.sleep(.025)
    app.close()
    elapsed = time.monotonic()-began
    report = {"elapsed_s":elapsed,"observations":observations,"switches":app.switches,
              "groups":app.history,"max_concurrent_gpu_loops":app.max_active_loops,
              "all_threads_stopped":app.thread is None or not app.thread.is_alive(),
              "all_workers_stopped":all(x["worker_stopped"] for x in app.history),
              "upright_floor_center":floor_center(app.gizmo.position).tolist(),
              "frozen_config":tuned.NO_ROD_SELECTED_MPC,"writes_results":False}
    report["all_passed"] = bool(elapsed<=30 and len(app.switches)>=2 and app.max_active_loops==1
        and report["all_threads_stopped"] and report["all_workers_stopped"]
        and all(x["reset_error_rad"]==0 and x["error"] is None for x in app.history)
        and all(x["old_thread_stopped"] and x["old_worker_stopped"] and x["old_stopped_at"]<=x["new_requested_at"] for x in app.switches)
        and any(x["group"]=="E" and x["renderer_xy_checked"] and x["worker_exitcode"]==0 for x in app.history))
    return report


def live_main(args: Any) -> int:
    # Enforce the requested ordering; the live demo cannot precede the saved report.
    result_file = v2.RESULTS_DIR/"moving_rod_rates.json"
    report_file = v2.REPO/"docs/SIM_CAMERA_MOVING_ROD_REPORT.zh-TW.md"
    if not report_file.exists() or not result_file.exists():
        raise RuntimeError("save phase rates and the senior report before live mode")
    rates = json.loads(result_file.read_text())
    if not rates.get("complete") or len(rates["cases"])!=30:
        raise RuntimeError("live mode requires the completed 30-case report")
    if rates["frozen_config"] != tuned.NO_ROD_SELECTED_MPC:
        raise RuntimeError("live config differs from the saved frozen report")
    baseline = json.loads(tuned.original.BASELINE_FILE.read_text())
    tuned.original.configure_scene(baseline)
    began = time.monotonic()
    app = None
    try:
        app = LiveGroups(args,baseline)
        if args.interactive_smoke:
            report = smoke(app,began)
            print("ROD_LIVE_SMOKE "+json.dumps(report),flush=True)
            return 0 if report["all_passed"] else 2
        while args.duration<=0 or time.monotonic()-began<args.duration:
            time.sleep(.1)
    except KeyboardInterrupt:
        pass
    finally:
        if app is not None and not app.closed:
            app.close()
    return 0
