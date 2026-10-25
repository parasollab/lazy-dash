"""Sorting scenes: N UR5e arms on one table, blocks sorted to goal regions across it.

Bases, table and regions follow the paper's scenes; block placements match them in distribution only.
"""
from __future__ import annotations

import math
import random

from lazydash.scene import (TABLE_TOP, Arm, Block, Pose, Scene, upright_or_rolled,
                            face_grasps, with_symmetric_grasps)
from lazydash.scenarios.grid import grid_points, variant_suffix, round_robin

BASE_Z = 0.705
BASE_YAW = math.pi
BLOCK_Z = 0.75
BLOCK_YAW = math.pi

# Base positions, in robot order.
RELAY_BASES = [
    (-1.4, -0.7), (-1.4, 0.7), (0.0, 0.7), (0.0, -0.7), (1.4, 0.7), (1.4, -0.7),
    (0.0, 2.1), (-1.4, 2.1), (1.4, 2.1), (1.4, -2.1), (2.8, 0.7), (2.8, -0.7),
    (0.0, -2.1), (2.8, 2.1), (-1.4, -2.1), (2.8, -2.1),
]

# Table slab: 8 x 8 m, x offset per scene.
TABLE_HALF = 4.0
TABLE_X_OFFSET = {4: 0.0, 9: 0.7, 16: 0.7}

R = tuple  # region: ((xmin, xmax), (ymin, ymax))

# Sorting: 4 robots: four start and four goal regions, each with an exclusion.
# The remainder goes to the last start region and the first goal region.
SORT4_STARTS = [
    ([((-1.5, -1.0), (-0.8, -0.2))], [((-1.5, -1.25), (-0.8, -0.4))]),
    ([((-0.45, 0.15), (-0.8, -0.2))], [((-0.15, 0.3), (-0.8, -0.4))]),
    ([((-1.5, -1.0), (0.2, 0.8))], [((-1.5, -1.25), (0.5, 1.0))]),
    ([((-0.45, 0.15), (0.2, 0.8))], [((-0.15, 0.3), (0.5, 1.0))]),
]
SORT4_GOALS = [
    ([((-0.2, 0.4), (0.5, 1.1))], [((-0.3, 0.2), (0.4, 0.9))]),
    ([((-1.8, -1.2), (0.5, 1.1))], [((-1.5, -1.1), (0.4, 0.9))]),
    ([((-0.2, 0.4), (-1.15, -0.55))], [((-0.3, 0.2), (-0.85, -0.25))]),
    ([((-1.8, -1.2), (-1.15, -0.55))], [((-1.5, -1.1), (-0.85, -0.25))]),
]

# Sorting: 9 robots: eight start and eight goal regions, blocks dealt round robin.
SORT9_STARTS = [
    ((-1.7, -1.3), (1.5, 1.7)), ((1.15, 1.65), (1.5, 1.7)), ((1.15, 1.65), (-0.4, -0.2)),
    ((-1.7, -1.3), (-0.4, -0.2)), ((-0.2, 0.2), (1.55, 1.8)), ((0.85, 1.0), (0.5, 0.9)),
    ((-0.2, 0.2), (-0.4, -0.2)), ((-1.1, -0.95), (0.45, 0.95)),
]
SORT9_GOALS = [
    ((1.65, 1.86), (0.55, 0.95)), ((-0.2, 0.2), (-1.3, -1.1)), ((-1.9, -1.8), (0.45, 0.95)),
    ((-0.2, 0.2), (2.35, 2.5)), ((1.15, 1.65), (-1.3, -1.1)), ((-1.45, -1.05), (-1.3, -1.1)),
    ((-1.45, -1.05), (2.35, 2.5)), ((1.15, 1.65), (2.35, 2.5)),
]

# Sorting: 16 robots: twelve start and twelve goal regions, dealt round robin.
SORT16_STARTS = [
    ((-1.6, -1.1), (1.6, 1.7)), ((2.6, 3.0), (1.6, 1.7)), ((-1.6, -1.1), (-1.85, -1.6)),
    ((2.6, 3.0), (-1.8, -1.6)), ((-0.2, 0.2), (1.5, 1.7)), ((2.2, 2.4), (0.45, 0.95)),
    ((1.2, 1.6), (-1.85, -1.65)), ((-1.1, -0.9), (-0.95, -0.5)), ((1.15, 1.65), (1.5, 1.7)),
    ((2.2, 2.4), (-0.95, -0.45)), ((-0.3, 0.3), (-1.85, -1.65)), ((-1.1, -0.9), (0.45, 0.95)),
]
SORT16_GOALS = [
    ((3.1, 3.25), (0.45, 0.95)), ((1.15, 1.65), (-2.6, -2.5)), ((-0.25, 0.25), (2.4, 2.5)),
    ((-1.9, -1.8), (-0.95, -0.45)), ((3.1, 3.25), (-0.95, -0.45)), ((-0.25, 0.25), (-2.6, -2.4)),
    ((-1.8, -1.7), (0.45, 0.95)), ((1.15, 1.65), (2.35, 2.5)), ((3.1, 3.25), (-2.6, -2.4)),
    ((-1.9, -1.8), (-2.6, -2.4)), ((-1.9, -1.8), (2.3, 2.5)), ((3.1, 3.25), (2.25, 2.5)),
]

PARAMS = {
    "planning_padding": 0.01,
    "robot_range": 1.5,
    "handover_max_base_distance": 1.6,
    "handover_region": {"xy_half_width": 0.05, "z_above_base": [0.65, 0.9], "attempts": 1000},
}


def _sort4_points(rng, n):
    q, rem = divmod(n, 4)
    starts, goals = [], []
    for k, (inc, exc) in enumerate(SORT4_STARTS):
        starts += grid_points(rng, q + (rem if k == 3 else 0), inc, exc)
    for k, (inc, exc) in enumerate(SORT4_GOALS):
        goals += grid_points(rng, q + (rem if k == 0 else 0), inc, exc)
    return starts, goals


def _round_robin_points(rng, n, start_regions, goal_regions):
    slots = len(start_regions)
    counts = round_robin(n, slots)
    starts, goals = [None] * n, [None] * n
    for i, c in enumerate(counts):
        for j, (s, g) in enumerate(zip(grid_points(rng, c, [start_regions[i]]),
                                       grid_points(rng, c, [goal_regions[i]]))):
            starts[i + slots * j] = s
            goals[i + slots * j] = g
    return starts, goals


def make_sorting(num_arms: int, num_blocks: int, seed: int = 0,
                    joint_limits: str = "paper", grasp_symmetry: bool = False, grasp_set: str = "paper") -> Scene:
    if num_arms not in TABLE_X_OFFSET:
        raise ValueError("sorting scenes exist for 4, 9 and 16 arms")
    rng = random.Random(seed)
    if num_arms == 4:
        starts, goals = _sort4_points(rng, num_blocks)
    elif num_arms == 9:
        starts, goals = _round_robin_points(rng, num_blocks, SORT9_STARTS, SORT9_GOALS)
    else:
        starts, goals = _round_robin_points(rng, num_blocks, SORT16_STARTS, SORT16_GOALS)

    dx = TABLE_X_OFFSET[num_arms]
    scene = Scene(
        name=f"sorting-{num_arms}robots-{num_blocks}objects-s{seed}" + variant_suffix(joint_limits, grasp_symmetry, grasp_set),
        workspace_min=[dx - TABLE_HALF, -TABLE_HALF, 0.0],
        workspace_max=[dx + TABLE_HALF, TABLE_HALF, 2.5],
        params=dict(PARAMS),
        source={"generator": "lazydash.scenarios.sorting", "arms": num_arms,
                "blocks": num_blocks, "seed": seed,
                "joint_limits": joint_limits, "grasp_symmetry": grasp_symmetry,
                "grasp_set": grasp_set},
    )
    scene.add_slab("table", (dx - TABLE_HALF, -TABLE_HALF), (dx + TABLE_HALF, TABLE_HALF),
                   top=TABLE_TOP, bottom=-0.01)
    for i, (x, y) in enumerate(RELAY_BASES[:num_arms]):
        scene.arms.append(Arm(f"ur5e_{i}", [x, y, BASE_Z], BASE_YAW, joint_limits=joint_limits))
    for i, ((sx, sy), (gx, gy)) in enumerate(zip(starts, goals)):
        scene.blocks.append(Block(f"block_{i}", Pose([sx, sy, BLOCK_Z], [0.0, 0.0, BLOCK_YAW]),
                                  upright_or_rolled([gx, gy, BLOCK_Z], BLOCK_YAW)))
    if grasp_set == "faces":
        scene.grasps = face_grasps()
    if grasp_symmetry:
        scene.grasps = with_symmetric_grasps(scene.grasps)
    return scene
