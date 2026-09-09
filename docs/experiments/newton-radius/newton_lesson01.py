"""Lesson 01: verify Newton CPU free fall and ground contact with measured data.

Run with the isolated Python 3.12 environment. No ROS or GUI dependency.
The animation replays logged Newton positions; it is not a second simulation.
"""
import csv
import json
import math
import platform
from pathlib import Path
from importlib.metadata import version

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
import warp as wp
import newton
from newton.solvers import SolverXPBD

newton.use_coord_layout_targets = True

OUT = Path(__file__).resolve().parent / "results"
OUT.mkdir(exist_ok=True)
HEIGHT = 1.0  # sphere CENTER height above the ground, metres
RADIUS = 0.2  # sphere radius
DURATION = 2.0
CASES = [
    ("earth_ground", 9.81, True, 1 / 240),
    ("earth_half_dt", 9.81, True, 1 / 480),
    ("earth_no_ground", 9.81, False, 1 / 240),
    ("half_gravity", 4.905, True, 1 / 240),
]


def simulate(name, gravity, ground, dt):
    with wp.ScopedDevice("cpu"):
        builder = newton.ModelBuilder(gravity=(0.0, 0.0, -gravity))
        body = builder.add_body(
            xform=wp.transform(wp.vec3(0.0, 0.0, HEIGHT), wp.quat_identity()),
            label="test_sphere",
        )
        builder.add_shape_sphere(body, radius=RADIUS)
        if ground:
            builder.add_ground_plane()
        model = builder.finalize()
        solver = SolverXPBD(model, iterations=10)
        state, next_state = model.state(), model.state()
        control = model.control()
        pipeline = newton.CollisionPipeline(model)
        contacts = pipeline.contacts()
        rows = [(0.0, HEIGHT)]
        for step in range(round(DURATION / dt)):
            state.clear_forces()
            pipeline.collide(state, contacts)
            solver.step(state, next_state, control, contacts, dt)
            state, next_state = next_state, state
            rows.append(((step + 1) * dt, float(state.body_q.numpy()[body, 2])))
    data = np.asarray(rows)
    if not np.isfinite(data).all():
        raise RuntimeError(f"Nonfinite data in {name}")
    with (OUT / f"{name}.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "center_z_m"])
        writer.writerows(rows)
    early = data[:, 0] <= 0.3 + 1e-10
    expected = HEIGHT - 0.5 * gravity * data[early, 0] ** 2
    arrival = np.flatnonzero(data[:, 1] <= RADIUS + 0.005)
    tail = data[data[:, 0] >= DURATION - 0.25, 1]
    metrics = {
        "gravity_m_s2": gravity, "ground": ground, "dt_s": dt,
        "ideal_touch_time_s": math.sqrt(2 * (HEIGHT - RADIUS) / gravity),
        "measured_near_ground_time_s": float(data[arrival[0], 0]) if len(arrival) else None,
        "near_ground_definition": "first sample with center_z <= radius + 0.005 m; not exact solver contact time",
        "max_freefall_error_first_0_3s_m": float(np.max(np.abs(data[early, 1] - expected))),
        "final_center_z_m": float(data[-1, 1]),
        "min_center_z_m": float(data[:, 1].min()),
        "last_quarter_second_z_range_m": float(np.ptp(tail)),
    }
    return data, metrics


def main():
    series, metrics = {}, {}
    for case in CASES:
        name = case[0]
        series[name], metrics[name] = simulate(*case)
        print(name, json.dumps(metrics[name]), flush=True)
    earth = metrics["earth_ground"]
    fine = metrics["earth_half_dt"]
    half = metrics["half_gravity"]
    checks = {
        "earth_freefall_error_under_1cm": earth["max_freefall_error_first_0_3s_m"] < 0.01,
        "ground_final_height_within_5mm": abs(earth["final_center_z_m"] - RADIUS) < 0.005,
        "ground_penetration_under_5mm": earth["min_center_z_m"] > RADIUS - 0.005,
        "ground_tail_motion_under_2mm": earth["last_quarter_second_z_range_m"] < 0.002,
        "no_ground_falls_below_plane": metrics["earth_no_ground"]["final_center_z_m"] < 0.0,
        "smaller_dt_reduces_freefall_error": fine["max_freefall_error_first_0_3s_m"] < earth["max_freefall_error_first_0_3s_m"],
        "earth_arrival_within_20ms": abs(earth["measured_near_ground_time_s"] - earth["ideal_touch_time_s"]) < 0.02,
        "half_gravity_arrival_within_20ms": abs(half["measured_near_ground_time_s"] - half["ideal_touch_time_s"]) < 0.02,
    }
    summary = {
        "environment": {"python": platform.python_version(), "architecture": platform.machine(),
                        "newton": version("newton"), "warp": version("warp-lang"), "device": "cpu"},
        "sphere_center_initial_z_m": HEIGHT, "sphere_radius_m": RADIUS,
        "duration_s": DURATION, "solver": "SolverXPBD", "iterations": 10,
        "cases": metrics, "checks": checks,
        "scope": "One run per case. Educational acceptance criteria, not hardware safety limits. No grasping or ROS integration.",
    }
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), layout="constrained")
    colors = {"earth_ground": "#176b95", "earth_half_dt": "#8755a3", "earth_no_ground": "#db7441", "half_gravity": "#26856b"}
    for name in ("earth_ground", "half_gravity", "earth_no_ground"):
        d = series[name]
        axes[0].plot(d[:, 0], d[:, 1], label=name.replace("_", " "), color=colors[name])
    t = np.linspace(0, earth["ideal_touch_time_s"], 150)
    axes[0].plot(t, HEIGHT - 0.5 * 9.81 * t**2, "k--", label="ideal free fall (before touch)")
    axes[0].axhline(RADIUS, color="gray", linestyle=":", label=f"resting center height = {RADIUS:.2f} m")
    axes[0].set(xlabel="Time (s)", ylabel="Sphere center height (m)", ylim=(-0.4, 1.05), title="Gravity and ground contact")
    axes[0].legend(fontsize=8)
    for name in ("earth_ground", "earth_half_dt"):
        d = series[name]
        d = d[d[:, 0] <= 0.3 + 1e-10]
        error = (d[:, 1] - (HEIGHT - 0.5 * 9.81 * d[:, 0] ** 2)) * 1000
        axes[1].plot(d[:, 0], error, color=colors[name], label=f"dt = 1/{round(1/metrics[name]['dt_s'])} s")
    axes[1].set(xlabel="Time (s), before contact", ylabel="Simulation minus theory (mm)", title="Time-step sensitivity")
    axes[1].legend()
    fig.suptitle("Newton CPU experiment: predictions checked against logged positions", fontsize=14)
    fig.savefig(OUT / "lesson01_results.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4), layout="constrained")
    ax.set(xlim=(-0.5, 2.5), ylim=(-0.4, 1.2), ylabel="Height (m)", title="Measured Newton trajectories (CPU)")
    ax.set_xticks([0, 1, 2], ["Earth + ground", "Half gravity + ground", "Earth, no ground"])
    ax.tick_params(axis="x", labelsize=9)
    circles = []
    replay_names = ["earth_ground", "half_gravity", "earth_no_ground"]
    for x, name in enumerate(replay_names):
        if metrics[name]["ground"]:
            ax.plot([x - 0.4, x + 0.4], [0, 0], color="black", linewidth=2)
        else:
            ax.plot([x - 0.4, x + 0.4], [0, 0], color="gray", linestyle=":")
        circle = plt.Circle((x, HEIGHT), RADIUS, color=colors[name])
        ax.add_patch(circle)
        circles.append(circle)
    label = ax.text(0.02, 0.95, "", transform=ax.transAxes)
    def update(frame):
        timestamp = frame / 30
        for x, name in enumerate(replay_names):
            d = series[name]
            z = float(np.interp(timestamp, d[:, 0], d[:, 1]))
            circles[x].center = (x, z)
        label.set_text(f"t = {timestamp:.2f} s | no-ground sphere exits view")
        return circles + [label]
    animation = FuncAnimation(fig, update, frames=61, interval=1000 / 30)
    animation.save(OUT / "lesson01_replay.gif", writer=PillowWriter(fps=30))
    plt.close(fig)
    print("CHECKS", json.dumps(checks), flush=True)
    if not all(checks.values()):
        raise SystemExit("Some checks failed; inspect summary.json before reporting success.")


if __name__ == "__main__":
    main()
