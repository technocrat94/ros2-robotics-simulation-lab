#!/usr/bin/env python3
"""Newton 1.5.1 process: simulate a sphere and exchange JSON over loopback UDP."""
import json
import signal
import socket
import time

import newton
import warp as wp
from newton.solvers import SolverXPBD

STATE_ADDRESS = ("127.0.0.1", 15100)
COMMAND_ADDRESS = ("127.0.0.1", 15101)
PHYSICS_DT = 1.0 / 240.0
PROTOCOL = 1

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
next_step = time.monotonic()
next_state = time.monotonic()
print("NEWTON_ENDPOINT_READY protocol=1 command=15101 state=15100", flush=True)

while alive:
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
        message = {
            "type": "state", "protocol": PROTOCOL, "sequence": sequence,
            "wall_time_ns": time.time_ns(), "sim_time_s": simulation.sim_time,
            "running": running, "frame_id": "world", "object": "sphere",
            "position_m": [0.0, 0.0, simulation.z()],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "last_command_id": last_command_id,
        }
        state_socket.sendto(json.dumps(message).encode("utf-8"), STATE_ADDRESS)
        sequence += 1
        next_state = now + 0.05
    time.sleep(0.001)
