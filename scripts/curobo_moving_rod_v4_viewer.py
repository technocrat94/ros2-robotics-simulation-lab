"""Repeat-selectable v3 replay; uses imported renderer and viewer primitives."""
from __future__ import annotations
from pathlib import Path
from typing import Any
import json
import threading
import time
import numpy as np
import torch
from curobo_moving_rod_rates import ViserVisualizer, ContentPath, v2, PHASES
import curobo_moving_rod_v4 as v4
from curobo_moving_rod_v4_visuals import PredictionVisuals

def start_viewer(config: Path, results: dict[str, Any], frames: dict[str, list[dict[str, Any]]],
                 port: int, groups: tuple[str, ...] = v4.GROUPS, verify: bool = False) -> tuple[Any, dict[str, Any]]:
    """Replay selectable MPC cases without writing files."""
    viewer = ViserVisualizer(content_path=ContentPath(robot_config_file=str(config)),
                             connect_ip="127.0.0.1", connect_port=port,
                             add_control_frames=True, visualize_robot_spheres=False, add_robot_to_scene=True)
    server = viewer._server
    if int(server._websock_server._port) != port:
        server.stop()
        raise RuntimeError(f"requested port {port} occupied; refusing alternate binding")
    graphics=PredictionVisuals(server,results["preflight"]["floor_top_m"])
    rod = server.scene.add_cylinder("/truth/rod", radius=v2.ROD_RADIUS_M,
                                   height=v2.ROD_TOP_M, color=(235,55,45))
    points = server.scene.add_point_cloud("/camera/points", points=np.zeros((1,3)),
                                         colors=(220,0,220), point_size=.011)
    observed = server.scene.add_box("/observed/box", dimensions=(.01,.01,.01), color=(255,145,35), opacity=.42)
    predicted = server.scene.add_box("/predicted/box", dimensions=(.01,.01,.01), color=(80,150,255), opacity=.85, wireframe=True)
    first = next(iter(frames))
    panel = server.gui.add_image(frames[first][0]["depth"], label="Live simulated depth")
    index = {(c["summary"]["mode"], c["summary"]["rod_speed_mps"], c["summary"]["initial_x_m"], c["summary"]["repeat"]): n
             for n,c in results["cases"].items()}
    group_select = server.gui.add_dropdown("Case", options=groups, initial_value=groups[0])
    speed_select = server.gui.add_dropdown("Rod speed m/s", options=("0.05", "0.10", "0.20"), initial_value="0.05")
    phase_select = server.gui.add_dropdown("Initial X m", options=tuple(f"{x:.2f}" for x in PHASES), initial_value="0.36")
    repeat_select = server.gui.add_dropdown("Repeat",options=("1",),initial_value="1")
    joint_speed = server.gui.add_text("Joint command speed","",disabled=True)
    validity = server.gui.add_text("Phase validity", "", disabled=True)

    def selected_name() -> str:
        return index[(group_select.value, float(speed_select.value), float(phase_select.value), (1 if group_select.value=="A" else int(repeat_select.value)))]
    controller = server.gui.add_text("Controller", "cuRobo MPC", disabled=True)
    rate = server.gui.add_text("Simulated step rate", "", disabled=True)
    clock_text = server.gui.add_text("Sim time", "0", disabled=True)
    clearance = server.gui.add_text("Live truth clearance", "", disabled=True)
    error = server.gui.add_text("Playback error", "none", disabled=True)
    play = server.gui.add_button("Play selected case")
    mount = v2.sixth.fourth.read_mount_config(v2.MOUNT_CONFIG)
    # Use a fresh kinematics object so the controller is destroyed before playback.
    robot = v2.Kinematics(v2.KinematicsCfg.from_robot_yaml_file(
        str(config), tool_frames=[v2.sixth.preflight.TOOL_FRAME]))
    frustum = server.scene.add_camera_frustum("/camera/live", fov=v2.DEPTH_FOV_RAD,
                                             aspect=v2.DEPTH_ASPECT, scale=.13)
    busy = threading.Lock()
    stop_playback = threading.Event()
    workers = []

    def show(name: str, frame: dict[str, Any]) -> None:
        summary = results["cases"][name]["summary"]
        controller.value = summary["controller"]
        validity.value = summary["scene_status"]
        rate.value = f"{summary['achieved_step_rate_hz']:.2f} Hz"
        clock_text.value = f"{frame['time_s']:.3f} s"
        q_for_clearance = torch.as_tensor(frame["q"],device="cuda",dtype=torch.float32).reshape(1,-1)
        current_clearance = frame.get("clearance_m")
        if current_clearance is None:
            spheres=v4.sphere_series(robot,q_for_clearance)
            current_clearance=float(min(v4.cylinder_distances(spheres,np.array([frame["rod_x"]]))[0],v4.floor_distances(spheres,v4.nonbase_mask(robot),results["preflight"]["floor_top_m"])[0]))
        clearance.value = f"{current_clearance:+.4f} m"
        cycles = results["cases"][name]["cycles"]
        cycle = cycles[min(len(cycles)-1,max(0,int(round(frame["time_s"]/.025))-1))]
        current_speed = max(abs(v) for v in cycle["new_velocity_radps"]) if frame["time_s"]>0 else 0.
        joint_speed.value = f"{current_speed:.3f} rad/s"
        q = np.array(frame["q"], dtype=np.float32)
        viewer.set_joint_state(v2.sixth.arm_state(q).squeeze(0))
        pos, quat = v2.sixth.fourth.camera_pose_for_q(robot, torch.as_tensor(q, device="cuda"), mount)
        frustum.position, frustum.wxyz = tuple(pos), tuple(quat)
        rod.position = (frame["rod_x"], v2.ROD_Y_M, v2.ROD_CENTER_Z_M)
        points.points = frame["points"] if len(frame["points"]) else np.zeros((1,3))
        panel.image = frame["depth"]
        for box, fit in ((observed, frame["fit"]), (predicted, frame["predicted_fit"])):
            box.visible = fit is not None
            if fit:
                box.position, box.dimensions = tuple(fit["center_m"]), tuple(fit["dims_m"])
        # Show only one box when prediction is disabled, avoiding coincident surfaces.
        t=frame["time_s"]
        true_vx=(v2.rod_x_at(t+.001,summary["rod_speed_mps"],summary["initial_x_m"],-1.)-frame["rod_x"])/.001
        graphics.update(cycle["fit"],cycle["velocity_estimate_mps"],[true_vx,0,0],.4+1/30,summary["prediction"],predicted)

    def playback(name: str, fast: bool = False) -> dict[str, Any]:
        with busy:
            try:
                previous_time = 0.
                frame_index = 0
                for sample in results["cases"][name]["trace"]:
                    if stop_playback.is_set():
                        break
                    t = sample["time_s"]
                    while frame_index+1 < len(frames[name]) and frames[name][frame_index+1]["time_s"] <= t:
                        frame_index += 1
                    frame = {**frames[name][frame_index], **sample,
                             "rod_x": v2.rod_x_at(t, results["cases"][name]["summary"]["rod_speed_mps"], results["cases"][name]["summary"]["initial_x_m"], -1.)}
                    if not fast:
                        time.sleep(max(0., t-previous_time))
                    if not fast and name!=selected_name():break
                    show(name, frame)
                    previous_time = t
                return {"case": name, "error": None,
                        "callback_non_main": threading.current_thread() is not threading.main_thread(),
                        "robot_changed": any(not np.allclose(frames[name][0]["q"], f["q"]) for f in frames[name][1:]),
                        "rod_changed": bool(np.ptp([f["rod_x"] for f in frames[name]]) > 0),
                        "depth_updates": len(frames[name]), "predicted_box_available": any(f["predicted_fit"] for f in frames[name])}
            except Exception as exc:
                error.value = f"{type(exc).__name__}: {exc}"
                return {"case": name, "error": error.value}

    def click(_event: Any) -> None:
        thread = threading.Thread(target=playback, args=(selected_name(),), daemon=True)
        workers.append(thread)
        thread.start()

    play.on_click(click)
    def filter_changed(_event: Any) -> None:
        repeat_select.disabled = True
        if group_select.value=="A":repeat_select.value="1"
        name = selected_name()
        show(name, frames[name][0])
    for handle in (group_select, speed_select, phase_select, repeat_select):
        handle.on_update(filter_changed)
    repeat_select.disabled = True
    show(first, frames[first][0])
    if verify:
        verification = []
        for mode, repeat in ((g,1) for g in groups):
            group_select.value = mode
            repeat_select.value = str(repeat)
            group_select.value, speed_select.value, phase_select.value = mode, "0.10", "0.60"
            name = selected_name()
            output = []
            thread = threading.Thread(target=lambda n=name: output.append(playback(n, fast=True)))
            thread.start()
            thread.join(timeout=30)
            if thread.is_alive():
                server.stop()
                raise RuntimeError("MPC playback verification timed out")
            verification.extend(output)
        repeat_select.value = "1"
        group_select.value, speed_select.value, phase_select.value = groups[0], "0.05", "0.36"
        show(first, frames[first][0])
        group_select.value="B"
        name=selected_name()
        show(name,frames[name][min(1,len(frames[name])-1)])
        before=graphics.ghost.visible and graphics.arrow.visible and predicted.visible
        graphics.visible.value=False
        graphics.refresh()
        toggle_off=not graphics.ghost.visible and not graphics.arrow.visible and not predicted.visible
        graphics.visible.value=True
        graphics.refresh()
        toggle_on=before and graphics.ghost.visible and graphics.arrow.visible and predicted.visible
        group_select.value, speed_select.value, phase_select.value = groups[0], "0.05", "0.36"
        show(first,frames[first][0])
        report = {"cases": verification,"verification_run":True,"prediction_toggle_off_verified":toggle_off,"prediction_toggle_on_verified":toggle_on, "all_passed": toggle_off and toggle_on and all(
            c.get("error") is None and c.get("callback_non_main") and c.get("robot_changed") and c.get("rod_changed")
            for c in verification)}
    else:
        report={"verification_run":False,"all_passed":None}
    print("MOVING_ROD_V4_VIEWER " + json.dumps(report), flush=True)
    print(f"MOVING_ROD_V4_URL http://127.0.0.1:{port}", flush=True)
    # Main owns the server and only the playback threads started here.
    viewer._mpc_workers = workers
    viewer._mpc_stop_event = stop_playback
    return viewer, report


class FrameCache(dict):
    """Lazy, serialized camera rendering for a selected saved case only."""
    def __init__(self,worker,results):
        super().__init__()
        self.worker,self.results=worker,results
        self.lock=threading.RLock()

    def __missing__(self,name):
        with self.lock:
            if name not in self:
                self.update(v4.v3.rates.render_frames(self.worker,{"cases":{name:self.results["cases"][name]}}))
            return self[name]
