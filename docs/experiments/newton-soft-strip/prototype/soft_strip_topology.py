#!/usr/bin/env python3
"""Build and inspect a cantilever soft-body mesh without simulating it."""

import argparse
import json

import newton
import warp as wp


# Parameters intended for the learner to change.
LENGTH_M = 0.40
WIDTH_M = 0.05
THICKNESS_M = 0.02

DEFAULT_CELLS_X = 20
CELLS_Y = 3
CELLS_Z = 2

YOUNG_MODULUS_PA = 1_000_000.0
POISSON_RATIO = 0.45
DENSITY_KG_M3 = 1100.0
DAMPING_PA_S = 0.0  # No dynamics yet; damping will be selected later.


def parse_args():
    parser = argparse.ArgumentParser(
        description="Build and inspect a cantilever soft-body mesh."
    )
    parser.add_argument(
        "--cells-x",
        type=int,
        default=DEFAULT_CELLS_X,
        help="number of cells along the 0.40 m strip length (default: 20)",
    )
    args = parser.parse_args()
    if args.cells_x < 1:
        parser.error("--cells-x must be at least 1")
    return args


def material_parameters():
    mu = YOUNG_MODULUS_PA / (2.0 * (1.0 + POISSON_RATIO))
    lam = (
        YOUNG_MODULUS_PA
        * POISSON_RATIO
        / ((1.0 + POISSON_RATIO) * (1.0 - 2.0 * POISSON_RATIO))
    )
    return mu, lam


def build_builder(cells_x, damping_pa_s=DAMPING_PA_S):
    mu, lam = material_parameters()

    builder = newton.ModelBuilder()
    builder.add_soft_grid(
        pos=wp.vec3(0.0, 0.0, 0.50),
        rot=wp.quat_identity(),
        vel=wp.vec3(0.0, 0.0, 0.0),
        dim_x=cells_x,
        dim_y=CELLS_Y,
        dim_z=CELLS_Z,
        cell_x=LENGTH_M / cells_x,
        cell_y=WIDTH_M / CELLS_Y,
        cell_z=THICKNESS_M / CELLS_Z,
        density=DENSITY_KG_M3,
        k_mu=mu,
        k_lambda=lam,
        k_damp=damping_pa_s,
        fix_left=True,
        label="cantilever_rubber_strip",
    )
    return builder


def main():
    args = parse_args()
    cells_x = args.cells_x
    mu, lam = material_parameters()
    builder = build_builder(cells_x)

    expected = {
        "particles": (cells_x + 1) * (CELLS_Y + 1) * (CELLS_Z + 1),
        "tetrahedra": 5 * cells_x * CELLS_Y * CELLS_Z,
        "surface_triangles": 4
        * (cells_x * CELLS_Y + cells_x * CELLS_Z + CELLS_Y * CELLS_Z),
        "fixed_left_particles": (CELLS_Y + 1) * (CELLS_Z + 1),
    }
    actual = {
        "particles": builder.particle_count,
        "tetrahedra": builder.tet_count,
        "surface_triangles": builder.tri_count,
        "fixed_left_particles": sum(float(mass) == 0.0 for mass in builder.particle_mass),
    }

    coordinates = [[float(p[axis]) for axis in range(3)] for p in builder.particle_q]
    minimum = [min(p[axis] for p in coordinates) for axis in range(3)]
    maximum = [max(p[axis] for p in coordinates) for axis in range(3)]
    extent = [maximum[axis] - minimum[axis] for axis in range(3)]
    target_extent = [LENGTH_M, WIDTH_M, THICKNESS_M]

    checks = {name: actual[name] == expected[name] for name in expected}
    checks["dimensions_match"] = all(
        abs(measured - target) < 1.0e-6
        for measured, target in zip(extent, target_extent)
    )

    config = {
        "dimensions_m": target_extent,
        "cells": [cells_x, CELLS_Y, CELLS_Z],
        "young_modulus_pa": YOUNG_MODULUS_PA,
        "poisson_ratio": POISSON_RATIO,
        "k_mu_pa": mu,
        "k_lambda_pa": lam,
    }
    geometry = {"bbox_min_m": minimum, "bbox_max_m": maximum, "extent_m": extent}

    print("CONFIG", json.dumps(config))
    print("EXPECTED", json.dumps(expected))
    print("ACTUAL", json.dumps(actual))
    print("GEOMETRY", json.dumps(geometry))
    print("CHECKS", json.dumps(checks))


if __name__ == "__main__":
    main()
