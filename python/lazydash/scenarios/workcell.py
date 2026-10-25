"""Workcell scenes modelled on industrial tasks: `packing`, `depalletize` and `assembly`.

Blocks may rest on each other, so they use a sphere inside the cube and 2 mm padding.
"""
from __future__ import annotations

import math
import random

from lazydash.scene import BLOCK_SIZE, TABLE_TOP, Arm, Block, Pose, Scene, r, upright_or_rolled

BASE_Z = 0.705
BASE_YAW = math.pi
BLOCK_Z = 0.75
YAW = math.pi
STEP = 0.095                  # block grid, as the sorting scenes
RADIUS = 0.016                # inside the cube: blocks may rest on each other
PARAMS = {
    "planning_padding": 0.002,
    "robot_range": 1.5,
    "handover_max_base_distance": 1.6,
    "handover_region": {"xy_half_width": 0.05, "z_above_base": [0.65, 0.9], "attempts": 1000},
}
WALL_T = 0.01                 # bin, box and rail walls


def _grid(cx, cy, nx, ny):
    """nx x ny grid points centred on (cx, cy), STEP apart."""
    return [(cx + (i - (nx - 1) / 2) * STEP, cy + (j - (ny - 1) / 2) * STEP) for i in range(nx) for j in range(ny)]


def _container(scene: Scene, label: str, cx, cy, sx, sy, height):
    """An open-top box (tote, shipping box): four thin walls from the table up."""
    lo, hi = TABLE_TOP, TABLE_TOP + height
    x0, x1, y0, y1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
    scene.add_board(f"{label}_xlo", (x0 - WALL_T, y0, lo), (x0, y1, hi))
    scene.add_board(f"{label}_xhi", (x1, y0, lo), (x1 + WALL_T, y1, hi))
    scene.add_board(f"{label}_ylo", (x0, y0 - WALL_T, lo), (x1, y0, hi))
    scene.add_board(f"{label}_yhi", (x0, y1, lo), (x1, y1 + WALL_T, hi))


def _parking(scene: Scene, x, y):
    """A free spot on the table for move-out poses."""
    scene.placement_regions.append({"min": [r(x - 0.04), r(y - 0.04), BLOCK_Z],
                                    "max": [r(x + 0.04), r(y + 0.04), BLOCK_Z], "yaw": [YAW, YAW]})


def _scene(name, lo, hi, source) -> Scene:
    scene = Scene(name=name, workspace_min=[lo[0], lo[1], 0.0], workspace_max=[hi[0], hi[1], 2.5],
                  params=dict(PARAMS), source={"generator": "lazydash.scenarios.workcell", **source})
    scene.add_slab("table", lo, hi, top=TABLE_TOP, bottom=-0.01)
    return scene


def _block(scene: Scene, start, goal):
    k = len(scene.blocks)
    scene.blocks.append(Block(f"block_{k}", Pose(list(start), [0.0, 0.0, YAW]),
                              upright_or_rolled(list(goal), YAW), RADIUS))


def make_packing(num_items: int = 8, seed: int = 0) -> Scene:
    """Tote (3 x 3 slots) -> picker -> over the partition -> packer -> box (2 x 2, two layers)."""
    if not 1 <= num_items <= 8:
        raise ValueError("the box holds 8 items")
    rng = random.Random(seed)
    scene = _scene(f"packing-{num_items}-s{seed}", (-1.0, -0.8), (2.2, 0.8),
                   {"kind": "packing", "items": num_items, "seed": seed})
    scene.arms += [Arm("picker", [0.0, 0.0, BASE_Z], BASE_YAW), Arm("packer", [1.2, 0.0, BASE_Z], BASE_YAW)]
    _container(scene, "tote", -0.45, 0.0, 0.36, 0.36, 0.12)
    _container(scene, "box", 1.65, 0.0, 0.30, 0.30, 0.13)
    scene.add_plate("partition", (0.6, 0.0, TABLE_TOP + 0.225), (0.02, 1.2, 0.45))
    starts = rng.sample(_grid(-0.45, 0.0, 3, 3), num_items)
    columns = _grid(1.65, 0.0, 2, 2)
    goals = [(x, y, BLOCK_Z + level * BLOCK_SIZE) for level in (0, 1) for x, y in columns][:num_items]
    rng.shuffle(goals)
    for (sx, sy), g in zip(starts, goals):
        _block(scene, (sx, sy, BLOCK_Z), g)
    _parking(scene, 0.0, 0.45)
    _parking(scene, 1.2, 0.45)
    return scene


def make_depalletize(num_boxes: int = 12, seed: int = 0) -> Scene:
    """Pallet (2 x 2 towers of 3) -> three arms in a line -> conveyor (2 rows of 6)."""
    if not 1 <= num_boxes <= 12:
        raise ValueError("the pallet holds 12 boxes")
    rng = random.Random(seed)
    scene = _scene(f"depalletize-{num_boxes}-s{seed}", (-1.0, -0.8), (3.5, 0.8),
                   {"kind": "depalletize", "boxes": num_boxes, "seed": seed})
    for i in range(3):
        scene.arms.append(Arm(f"ur5e_{i}", [1.2 * i, 0.0, BASE_Z], BASE_YAW))
    # Pallet edge guards (low) and conveyor side rails.
    _container(scene, "pallet", -0.45, 0.0, 0.34, 0.34, 0.05)
    for k, x in enumerate((2.79, 3.11)):
        scene.add_plate(f"rail_{k}", (x, 0.0, TABLE_TOP + 0.03), (WALL_T, 0.8, 0.06))
    towers = _grid(-0.45, 0.0, 2, 2)
    starts = [(x, y, BLOCK_Z + level * BLOCK_SIZE) for x, y in towers for level in range(3)][:num_boxes]
    slots = _grid(2.95, 0.0, 2, 6)
    goals = rng.sample(slots, num_boxes)
    for s, (gx, gy) in zip(starts, goals):
        _block(scene, s, (gx, gy, BLOCK_Z))
    _parking(scene, 1.2, 0.45)
    return scene


def make_assembly(num_stacks: int = 3, seed: int = 0) -> Scene:
    """Feeders of bases, middles and tops at three arms -> assembly station at the fourth."""
    if not 1 <= num_stacks <= 3:
        raise ValueError("the assembly station has 3 fixtures")
    rng = random.Random(seed)
    scene = _scene(f"assembly-{num_stacks}-s{seed}", (-1.4, -1.4), (1.4, 1.4),
                   {"kind": "assembly", "stacks": num_stacks, "seed": seed})
    # Square, 1.2 m sides: arm 3 assembles, arm 1 (diagonal) feeds bases, arms 0 and 2 the rest.
    corners = [(-0.6, -0.6), (0.6, -0.6), (0.6, 0.6), (-0.6, 0.6)]
    for i, (x, y) in enumerate(corners):
        scene.arms.append(Arm(f"ur5e_{i}", [x, y, BASE_Z], BASE_YAW))
    half = (1.3 - TABLE_TOP) / 2
    scene.obstacles.append({"id": "cabinet", "type": "cylinder", "center": [0.0, 0.0, r(TABLE_TOP + half)],
                            "axis": [0.0, 0.0, 1.0], "radius": 0.3, "half_height": r(half)})

    def outward(i, d, side=0.0):
        """`d` out from arm i's base, away from the centre, and `side` across."""
        x, y = corners[i]
        n = math.hypot(x, y)
        ux, uy = x / n, y / n
        return x + d * ux - side * uy, y + d * uy + side * ux

    feeders = {"base": 1, "middle": 0, "top": 2}
    bins = {}
    for part, arm in feeders.items():
        cx, cy = outward(arm, 0.42)
        _container(scene, f"feeder_{part}", cx, cy, 0.30, 0.30, 0.08)
        bins[part] = rng.sample(_grid(cx, cy, 2, 2), num_stacks)
    # Fixtures in a row across the station's outward direction.
    fixtures = [outward(3, 0.42, (k - 1) * STEP) for k in range(3)][:num_stacks]
    for k, (gx, gy) in enumerate(fixtures):
        for level, part in enumerate(("base", "middle", "top")):
            sx, sy = bins[part][k]
            _block(scene, (sx, sy, BLOCK_Z), (gx, gy, BLOCK_Z + level * BLOCK_SIZE))
    for i in range(4):
        _parking(scene, *outward(i, 0.42, 0.32))
    return scene
