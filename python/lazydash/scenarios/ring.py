"""Ring scenes: N UR5e arms around a central pillar, each block bound for the opposite station.

Options: `bays` puts a wall between neighbouring stations; `stack` K > 1 starts blocks in
towers of K, rebuilt in order (stacked blocks use a 16 mm sphere and 2 mm padding).
"""
from __future__ import annotations

import math
import random

from lazydash.scene import (BLOCK_RADIUS, BLOCK_SIZE, TABLE_TOP, Arm, Block, Pose, Scene, face_grasps, r,
                            upright_or_rolled, with_symmetric_grasps)
from lazydash.scenarios.grid import GRID_STEP, round_robin, variant_suffix

BASE_Z = 0.705
BLOCK_Z = 0.75
NEIGHBOUR_DISTANCE = 1.2   # m between adjacent bases
PILLAR_RADIUS = 0.55       # m
PILLAR_TOP = 1.6           # m
# Per station, in the station frame (x outward from the base, y tangential):
# starts on one side of the outward axis, goals on the other.
STATION_X = (0.30, 0.50)
START_Y = (-0.24, -0.05)
GOAL_Y = (0.05, 0.24)
PARKING = (0.62, 0.0)         # station frame: centre of the parking spot (stacked scenes)
PARKING_HALF = 0.04
# Bay walls (`bays`): along the bisector between neighbours.
BAY_WALL_FROM = 0.2           # m beyond the base circle
BAY_WALL_HEIGHT = 0.6         # m above the table
BAY_WALL_THICKNESS = 0.03
STACKED_RADIUS = 0.016        # inside the cube (half side 0.019)
STACKED_PADDING = 0.002       # planning padding of stacked scenes

PARAMS = {
    "planning_padding": 0.01,
    "robot_range": 1.5,
    "handover_max_base_distance": 1.6,
    "handover_region": {"xy_half_width": 0.05, "z_above_base": [0.65, 0.9], "attempts": 1000},
}


def _patch(x_range, y_range):
    """Grid points of a station patch (station frame), as in grid.grid_points."""
    xs, ys = [], []
    x = x_range[0]
    while x <= x_range[1] + 1e-9:
        xs.append(x)
        x += GRID_STEP
    y = y_range[0]
    while y <= y_range[1] + 1e-9:
        ys.append(y)
        y += GRID_STEP
    return [(a, b) for a in xs for b in ys]


def _radial_wall(scene: Scene, label: str, angle: float, r0: float, r1: float, tolerance: float = 0.01):
    """A thin vertical wall along the ray at `angle` from radius r0 to r1, as
    a row of vertical cylinders (as Scene.add_plate, not axis-aligned)."""
    t = BAY_WALL_THICKNESS
    cell = 2 * math.sqrt(tolerance * (t + tolerance))
    n = max(1, math.ceil((r1 - r0) / cell))
    cell = (r1 - r0) / n
    radius = math.hypot(t / 2, cell / 2)
    half = BAY_WALL_HEIGHT / 2
    for i in range(n):
        d = r0 + (i + 0.5) * cell
        scene.obstacles.append({
            "id": f"{label}_{i}", "type": "cylinder",
            "center": [r(d * math.cos(angle)), r(d * math.sin(angle)), r(TABLE_TOP + half)],
            "axis": [0.0, 0.0, 1.0], "radius": r(radius), "half_height": r(half)})


def make_ring(num_arms: int, num_blocks: int, seed: int = 0, joint_limits: str = "paper",
              grasp_symmetry: bool = False, grasp_set: str = "paper", bays: bool = False,
              stack: int = 1) -> Scene:
    if num_arms < 4 or num_arms % 2:
        raise ValueError("ring scenes need an even number of arms, at least 4")
    if stack < 1:
        raise ValueError("stack is the tower height, at least 1")
    rng = random.Random(seed)
    radius = NEIGHBOUR_DISTANCE / (2 * math.sin(math.pi / num_arms))
    angles = [2 * math.pi * i / num_arms for i in range(num_arms)]

    def world(i, p, z):
        c, s = math.cos(angles[i]), math.sin(angles[i])
        x, y = radius + p[0], p[1]  # station frame: base at (radius, 0), x outward
        return [c * x - s * y, s * x + c * y, z]

    starts_per = round_robin(num_blocks, num_arms)
    towers_per = [math.ceil(k / stack) for k in starts_per]
    capacity = len(_patch(STATION_X, START_Y))
    if max(towers_per) > capacity:
        raise ValueError(f"at most {capacity * stack * num_arms} blocks for {num_arms} arms")
    start_pts = [rng.sample(_patch(STATION_X, START_Y), k) for k in towers_per]
    # Station s's towers go to station s + N/2; its goal patch takes them all.
    goal_pts = [rng.sample(_patch(STATION_X, GOAL_Y), k) for k in towers_per]

    table = radius + 0.9
    scene = Scene(
        name=f"ring-{num_arms}-{num_blocks}" + ("-bays" if bays else "") + (f"-stack{stack}" if stack > 1 else "")
        + f"-s{seed}" + variant_suffix(joint_limits, grasp_symmetry, grasp_set),
        workspace_min=[-table, -table, 0.0],
        workspace_max=[table, table, 2.5],
        params=dict(PARAMS, **({"planning_padding": STACKED_PADDING} if stack > 1 else {})),
        source={"generator": "lazydash.scenarios.ring", "arms": num_arms, "blocks": num_blocks,
                "seed": seed, "bays": bays, "stack": stack, "joint_limits": joint_limits,
                "grasp_symmetry": grasp_symmetry, "grasp_set": grasp_set},
    )
    scene.add_slab("table", (-table, -table), (table, table), top=TABLE_TOP, bottom=-0.01)
    half = (PILLAR_TOP - TABLE_TOP) / 2
    scene.obstacles.append({"id": "pillar", "type": "cylinder", "center": [0.0, 0.0, r(TABLE_TOP + half)],
                            "axis": [0.0, 0.0, 1.0], "radius": PILLAR_RADIUS, "half_height": r(half)})
    if bays:
        for i in range(num_arms):
            _radial_wall(scene, f"bay_wall_{i}", angles[i] + math.pi / num_arms,
                         radius + BAY_WALL_FROM, table)
    for i in range(num_arms):
        x, y, _ = world(i, (0.0, 0.0), 0.0)
        scene.arms.append(Arm(f"ur5e_{i}", [x, y, BASE_Z], angles[i] + math.pi, joint_limits=joint_limits))
    k = 0
    block_radius = STACKED_RADIUS if stack > 1 else BLOCK_RADIUS
    for s in range(num_arms):
        g = (s + num_arms // 2) % num_arms
        yaw_s, yaw_g = angles[s] + math.pi, angles[g] + math.pi
        left = starts_per[s]
        for p, q in zip(start_pts[s], goal_pts[s]):
            for level in range(min(stack, left)):  # bottom first, the same level at the goal
                z = BLOCK_Z + level * BLOCK_SIZE
                scene.blocks.append(Block(f"block_{k}", Pose(world(s, p, z), [0.0, 0.0, yaw_s]),
                                          upright_or_rolled(world(g, q, z), yaw_g), block_radius))
                k += 1
            left -= stack
    if stack > 1:
        for i in range(num_arms):
            cx, cy, _ = world(i, PARKING, 0.0)
            scene.placement_regions.append({
                "min": [r(cx - PARKING_HALF), r(cy - PARKING_HALF), BLOCK_Z],
                "max": [r(cx + PARKING_HALF), r(cy + PARKING_HALF), BLOCK_Z],
                "yaw": [r(angles[i] + math.pi), r(angles[i] + math.pi)]})
    if grasp_set == "faces":
        scene.grasps = face_grasps()
    if grasp_symmetry:
        scene.grasps = with_symmetric_grasps(scene.grasps)
    return scene
