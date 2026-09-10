#!/usr/bin/env python3
"""Newton 1.5.1 process: simulate a sphere and exchange JSON over loopback UDP."""
import csv
import functools
import json
import os
from pathlib import Path
import signal
import socket
import time

import newton
import viser
import warp as wp
from newton.solvers import SolverXPBD
from newton.viewer import ViewerViser

STATE_ADDRESS = ("127.0.0.1", 15100)
COMMAND_ADDRESS = ("127.0.0.1", 15101)
PHYSICS_DT = 1.0 / 240.0
PROTOCOL = 1
VIEWER_PORT = int(os.environ.get("NEWTON_VIEWER_PORT", "8082"))
LOG_PATH = os.environ.get("NEWTON_BRIDGE_LOG", "")

newton.use_coord_layout_targets = True
wp.init()
wp.set_device("cpu")


class Simulation:
    def __init__(self):
        builder = newton.ModelBuilder(gravity=(0.0, 0.0, -9.81))
        body = builder.add_body(
            xform=wp.transform((0.0, 0.0, 1.0), wp.quat_identity()), label="sphere"
        )
        builder.add_shape_sphere(body, radius=0.2, color=(0.1, 0.6, 0.9))
        builder.add_ground_plane()
        self.model = builder.finalize()
        self.solver = SolverXPBD(self.model, iterations=10)
        self.pipeline = newton.CollisionPipeline(self.model)
        self.contacts = self.pipeline.contacts()
        self.control = self.model.control()
        self.reset()

    def reset(self):
        self.state = self.model.state()
        self.next_state = self.model.state()
        self.sim_time = 0.0

    def step(self):
        self.state.clear_forces()
        self.pipeline.collide(self.state, self.contacts)
        self.solver.step(
            self.state, self.next_state, self.control, self.contacts, PHYSICS_DT
        )
        self.state, self.next_state = self.next_state, self.state
        self.sim_time += PHYSICS_DT

    def z(self):
        return float(self.state.body_q.numpy()[0, 2])


alive = True


def stop(_signum, _frame):
    global alive
    alive = False


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
command_socket.bind(COMMAND_ADDRESS)
command_socket.setblocking(False)
state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

simulation = Simulation()
running = False
sequence = 0
last_command_id = ""
last_command_kind = "none"
next_step = time.monotonic()
next_state = time.monotonic()

# Bind the browser viewer to loopback. Access from the Mac uses an SSH tunnel.
server_class = viser.ViserServer
viser.ViserServer = functools.partial(server_class, host="127.0.0.1")
try:
    viewer = ViewerViser(port=VIEWER_PORT, label="ROS 2 ↔ Newton Bridge", share=False)
finally:
    viser.ViserServer = server_class
viewer.set_model(simulation.model)
server = viewer._server
server.scene.set_up_direction("+z")
server.initial_camera.position = (2.8, -4.0, 2.3)
server.initial_camera.look_at = (0.0, 0.0, 0.5)
server.initial_camera.up = (0.0, 0.0, 1.0)
server.scene.add_grid(
    "/bridge_reference/z_zero", width=8.0, height=8.0, plane="xy",
    cell_color=(150, 170, 190), section_color=(70, 100, 140),
)
server.gui.add_markdown(
    "## ROS 2 ↔ Newton Bridge\n"
    "This view has no motion buttons. Use the ROS services to command Newton; "
    "the values below come from Newton state packets returned to ROS."
)
status = server.gui.add_markdown("Waiting for the first state...")
server.gui.add_markdown(
    "### Evidence to check\n"
    "1. A ROS command changes **running**.\n"
    "2. Simulation time and measured height change only while running.\n"
    "3. Reset restores time to 0 and center height to 1 m.\n"
    "4. Stopping this endpoint makes ROS report `STALE`."
)

log_file = None
log_writer = None
if LOG_PATH:
    log_path = Path(LOG_PATH).expanduser()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", newline="", encoding="utf-8")
    log_writer = csv.DictWriter(
        log_file,
        fieldnames=[
            "sequence", "wall_time_ns", "sim_time_s", "center_z_m",
            "running", "last_command", "last_command_id",
        ],
    )
    log_writer.writeheader()

print(
    f"NEWTON_ENDPOINT_READY protocol=1 command=15101 state=15100 viewer={VIEWER_PORT}",
    flush=True,
)

try:
    while alive and viewer.is_running():
        try:
            payload, sender = command_socket.recvfrom(4096)
            command = json.loads(payload.decode("utf-8"))
            if command.get("protocol") != PROTOCOL:
                raise ValueError("protocol mismatch")
            kind = command.get("command")
            if kind == "set_running":
                running = bool(command["value"])
            elif kind == "reset":
                simulation.reset()
                running = False
            else:
                raise ValueError("unknown command")
            last_command_id = str(command.get("id", ""))
            last_command_kind = (
                f"set_running({str(running).lower()})"
                if kind == "set_running" else str(kind)
            )
            reply = {"type": "ack", "protocol": PROTOCOL, "id": last_command_id,
                     "command": kind, "ok": True}
            command_socket.sendto(json.dumps(reply).encode("utf-8"), sender)
        except BlockingIOError:
            pass
        except Exception as error:
            reply = {"type": "ack", "protocol": PROTOCOL, "id": "",
                     "ok": False, "error": str(error)}
            command_socket.sendto(json.dumps(reply).encode("utf-8"), sender)

        now = time.monotonic()
        if running and now >= next_step:
            simulation.step()
            next_step = now + PHYSICS_DT
        if now >= next_state:
            wall_time_ns = time.time_ns()
            center_z = simulation.z()
            message = {
                "type": "state", "protocol": PROTOCOL, "sequence": sequence,
                "wall_time_ns": wall_time_ns, "sim_time_s": simulation.sim_time,
                "running": running, "frame_id": "world", "object": "sphere",
                "position_m": [0.0, 0.0, center_z],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                "last_command_id": last_command_id,
            }
            state_socket.sendto(json.dumps(message).encode("utf-8"), STATE_ADDRESS)
            viewer.begin_frame(simulation.sim_time)
            viewer.log_state(simulation.state)
            viewer.end_frame()
            status.content = (
                f"### {'RUNNING' if running else 'PAUSED'}\n"
                f"- Simulation time: **{simulation.sim_time:.3f} s**\n"
                f"- Measured center height: **{center_z:.4f} m**\n"
                f"- Last ROS command: **{last_command_kind}**\n"
                f"- Returned state sequence: **{sequence}**\n"
                f"- Physics step: **{PHYSICS_DT:.6f} s**"
            )
            if log_writer:
                log_writer.writerow({
                    "sequence": sequence,
                    "wall_time_ns": wall_time_ns,
                    "sim_time_s": simulation.sim_time,
                    "center_z_m": center_z,
                    "running": running,
                    "last_command": last_command_kind,
                    "last_command_id": last_command_id,
                })
                log_file.flush()
            sequence += 1
            next_state = now + 0.05
        time.sleep(0.001)
finally:
    if log_file:
        log_file.close()
    viewer.close()
    command_socket.close()
    state_socket.close()
