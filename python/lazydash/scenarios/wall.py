"""Wall scenes: eight UR5e arms in a zig-zag line with K walls between neighbours.

Bases, table, walls and regions follow the paper's scenes.
"""
from __future__ import annotations

import math
import random

from lazydash.scene import (TABLE_TOP, Arm, Block, Pose, Scene, upright_or_rolled,
                            face_grasps, with_symmetric_grasps)
from lazydash.scenarios.grid import grid_points, variant_suffix

BASE_Z = 0.705
BASE_YAW = math.pi
BLOCK_Z = 0.75
BLOCK_YAW = math.pi

# x = -0.8 + 0.8 i, y alternating -0.5 / 1.0.
WALL_BASES = [(-0.8 + 0.8 * i, -0.5 if i % 2 == 0 else 1.0) for i in range(8)]

# Wall plates; a scene with K walls uses the first K.
WALL_SIZE = (0.03, 0.8, 1.2)
WALL_BOTTOM = 0.5
WALL_XY = [(0.0, -0.65), (2.4, 1.15), (0.8, 1.15), (3.2, -0.65), (1.6, -0.65), (4.0, 1.15)]

TABLE_MIN = (-2.0, -1.5)
TABLE_MAX = (8.0, 2.5)

START_REGION = ([((-1.3, -0.4), (-1.0, -0.1))], [((-1.0, -0.6), (-0.7, -0.3))])
# Goal regions: n // 7 blocks each, the remainder to the last.
GOAL_REGIONS = [
    ((4.55, 4.95), (1.3, 1.4)), ((3.75, 4.25), (-1.0, -0.85)), ((2.85, 3.35), (1.3, 1.4)),
    ((2.05, 2.55), (-1.0, -0.85)), ((1.35, 1.85), (1.3, 1.4)), ((-0.15, 0.25), (1.3, 1.4)),
    ((0.55, 1.05), (-1.0, -0.85)),
]

PARAMS = {
    "planning_padding": 0.01,
    "robot_range": 2.0,
    "handover_max_base_distance": 2.6,
    "handover_region": {"xy_half_width": 0.1, "z_above_base": [0.5, 0.8], "attempts": 1000},
}


def make_wall(num_walls: int, num_blocks: int, seed: int = 0,
              joint_limits: str = "paper", grasp_symmetry: bool = False, grasp_set: str = "paper") -> Scene:
    if num_walls not in (2, 4, 6):
        raise ValueError("wall scenes exist for 2, 4 and 6 walls")
    rng = random.Random(seed)
    starts = grid_points(rng, num_blocks, *START_REGION)
    q, rem = divmod(num_blocks, len(GOAL_REGIONS))
    goals = []
    for k, region in enumerate(GOAL_REGIONS):
        goals += grid_points(rng, q + (rem if k == len(GOAL_REGIONS) - 1 else 0), [region])

    scene = Scene(
        name=f"wall-{num_walls}walls-{num_blocks}objects-s{seed}" + variant_suffix(joint_limits, grasp_symmetry, grasp_set),
        workspace_min=[TABLE_MIN[0], TABLE_MIN[1], 0.0],
        workspace_max=[TABLE_MAX[0], TABLE_MAX[1], 2.5],
        params=dict(PARAMS),
        source={"generator": "lazydash.scenarios.wall", "walls": num_walls,
                "blocks": num_blocks, "seed": seed,
                "joint_limits": joint_limits, "grasp_symmetry": grasp_symmetry,
                "grasp_set": grasp_set},
    )
    scene.add_slab("table", TABLE_MIN, TABLE_MAX, top=TABLE_TOP, bottom=-0.01)
    for k, (x, y) in enumerate(WALL_XY[:num_walls]):
        scene.add_plate(f"wall_{k}", (x, y, WALL_BOTTOM + WALL_SIZE[2] / 2), WALL_SIZE)
    for i, (x, y) in enumerate(WALL_BASES):
        scene.arms.append(Arm(f"ur5e_{i}", [x, y, BASE_Z], BASE_YAW, joint_limits=joint_limits))
    for i, ((sx, sy), (gx, gy)) in enumerate(zip(starts, goals)):
        scene.blocks.append(Block(f"block_{i}", Pose([sx, sy, BLOCK_Z], [0.0, 0.0, BLOCK_YAW]),
                                  upright_or_rolled([gx, gy, BLOCK_Z], BLOCK_YAW)))
    if grasp_set == "faces":
        scene.grasps = face_grasps()
    if grasp_symmetry:
        scene.grasps = with_symmetric_grasps(scene.grasps)
    return scene
