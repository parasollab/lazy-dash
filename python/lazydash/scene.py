"""Lazy-DaSH scene files: CoMotion-style JSON plus a `lazydash` block.

Box scenery is approximated by spheres and cylinders when written.
Units: metres and radians; rpy is URDF-style, R = Rz(yaw) Ry(pitch) Rx(roll).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

FORMAT = "lazydash.scene/1"

# UR5e + Hand-e as used in the paper's runs.
UR5E_TYPE = "ur5e_hande"
UR5E_HOME = [0.0, -math.pi / 2, math.pi / 2, math.pi / 2, math.pi / 2, 0.0]
# "paper": every joint in [-pi, pi], as in the paper's runs; "ur5e": the real limits.
JOINT_LIMITS = {
    "paper": {"lower": [-math.pi] * 6, "upper": [math.pi] * 6},
    "ur5e": {"lower": [-2 * math.pi] * 2 + [-math.pi] + [-2 * math.pi] * 3,
             "upper": [2 * math.pi] * 2 + [math.pi] + [2 * math.pi] * 3},
}

# 3.8 cm blocks; the collision sphere circumscribes the cube (0.019 * sqrt(3)).
BLOCK_SIZE = 0.038
BLOCK_RADIUS = 0.033

# The paper's grasps: object frame in the `flange` frame (x out of the tool).
PAPER_GRASPS = [
    {"name": "upright", "link": "flange", "translation": [0.15, 0.0, 0.0],
     "rpy": [0.0, -1.57, 0.0], "approach_translation": [0.22, 0.0, 0.0]},
    {"name": "flipped", "link": "flange", "translation": [0.15, 0.0, 0.0],
     "rpy": [1.57, 0.0, 1.57], "approach_translation": [0.22, 0.0, 0.0]},
]

TABLE_TOP = 0.685


def rpy_matrix(rpy):
    """R = Rz(yaw) Ry(pitch) Rx(roll), as a list of rows."""
    (cr, sr), (cp, sp), (cy, sy) = [(math.cos(a), math.sin(a)) for a in rpy]
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matrix_rpy(m):
    """Inverse of rpy_matrix (pitch in [-pi/2, pi/2])."""
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(math.cos(pitch)) > 1e-9:
        return [math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0])]
    return [math.atan2(-m[1][2], m[1][1]), pitch, 0.0]  # gimbal lock: yaw folded into roll


def with_symmetric_grasps(grasps: list[dict]) -> list[dict]:
    """Add, for every grasp, the same grasp turned by pi about the flange x axis."""
    out = []
    for g in grasps:
        out.append(dict(g))
        m = rpy_matrix(g["rpy"])
        turned = [m[0], [-v for v in m[1]], [-v for v in m[2]]]  # Rx(pi) @ m
        out.append({**g, "name": g["name"] + "_r180", "rpy": [r(v) for v in matrix_rpy(turned)]})
    return out




def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rx(t):
    c, s = math.cos(t), math.sin(t)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _ry(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


# Per face (outward normal in the block frame): the rotation of the block that
# brings that face to the top, so the top grasp holds it by that face.
_FACES = [("top", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
          ("bottom", _rx(math.pi)),
          ("side_px", _ry(-math.pi / 2)), ("side_nx", _ry(math.pi / 2)),
          ("side_py", _rx(math.pi / 2)), ("side_ny", _rx(-math.pi / 2))]


def face_grasps(depth: float = 0.15, approach: float = 0.22) -> list[dict]:
    """Grasps from all six faces of a box, each in two finger orientations (12 in all),
    with the PAPER_GRASPS depth and approach convention."""
    top = _ry(-math.pi / 2)  # the block's +z face toward the flange (as "upright")
    out = []
    for name, to_top in _FACES:
        for turn, suffix in ((0.0, ""), (math.pi / 2, "_r90")):
            m = _matmul(_rx(turn), _matmul(top, to_top))
            out.append({"name": name + suffix, "link": "flange", "translation": [r(depth), 0.0, 0.0],
                        "rpy": [r(v) for v in matrix_rpy(m)], "approach_translation": [r(approach), 0.0, 0.0]})
    return out


def yaw_quaternion_xyzw(yaw: float) -> list[float]:
    return [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]


def r(v: float) -> float:
    """Round for stable, readable JSON."""
    return round(float(v), 6)


@dataclass
class Pose:
    position: list[float]
    rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

    def to_json(self) -> dict:
        return {"position": [r(v) for v in self.position], "rpy": [r(v) for v in self.rpy]}


@dataclass
class Arm:
    name: str
    base_position: list[float]
    base_yaw: float
    start: list[float] = field(default_factory=lambda: list(UR5E_HOME))
    joint_limits: str = "paper"  # key of JOINT_LIMITS

    def to_json(self) -> dict:
        return {
            "type": UR5E_TYPE,
            "name": self.name,
            "base": {"position": [r(v) for v in self.base_position],
                     "quaternion_xyzw": [r(v) for v in yaw_quaternion_xyzw(self.base_yaw)]},
            "start": [r(v) for v in self.start],
            "joint_limits": JOINT_LIMITS[self.joint_limits],
        }


@dataclass
class Block:
    name: str
    start: Pose
    goals: list[Pose]  # alternatives: reaching any one of them completes the object
    collision_radius: float = BLOCK_RADIUS

    def to_json(self) -> dict:
        return {
            "name": self.name,
            "shape": {"type": "box", "size": [BLOCK_SIZE] * 3},
            "collision_radius": self.collision_radius,
            "start": self.start.to_json(),
            "goals": [g.to_json() for g in self.goals],
        }


def upright_or_rolled(position: list[float], yaw: float) -> list[Pose]:
    """The XOR goal set of the sorting scenes: upright, or rolled by pi."""
    return [Pose(position, [0.0, 0.0, yaw]), Pose(position, [math.pi, 0.0, yaw])]


@dataclass
class Scene:
    name: str
    workspace_min: list[float]
    workspace_max: list[float]
    arms: list[Arm] = field(default_factory=list)
    blocks: list[Block] = field(default_factory=list)
    obstacles: list[dict] = field(default_factory=list)
    approximations: list[dict] = field(default_factory=list)
    params: dict = field(default_factory=dict)
    grasps: list[dict] = field(default_factory=lambda: [dict(g) for g in PAPER_GRASPS])
    source: dict = field(default_factory=dict)
    # Placement regions for move-out poses: {"min", "max"} position boxes with a yaw range.
    placement_regions: list[dict] = field(default_factory=list)

    # ---------------------------------------------------------------- scenery

    def add_slab(self, label: str, xy_min, xy_max, top: float, bottom: float = 0.0):
        """A horizontal slab (table) as one flat vertical cylinder that circumscribes it."""
        cx, cy = (xy_min[0] + xy_max[0]) / 2, (xy_min[1] + xy_max[1]) / 2
        hx, hy = (xy_max[0] - xy_min[0]) / 2, (xy_max[1] - xy_min[1]) / 2
        radius = math.hypot(hx, hy)
        half_height = (top - bottom) / 2
        self.obstacles.append({
            "id": f"{label}", "type": "cylinder",
            "center": [r(cx), r(cy), r(bottom + half_height)], "axis": [0.0, 0.0, 1.0],
            "radius": r(radius), "half_height": r(half_height)})
        self.approximations.append({
            "label": label, "box": {"min": [r(xy_min[0]), r(xy_min[1]), r(bottom)], "max": [r(xy_max[0]), r(xy_max[1]), r(top)]},
            "primitives": 1, "max_outward_error": r(radius - min(hx, hy)),
            "note": "top/bottom exact; side error only beyond the slab edges"})

    def add_plate(self, label: str, center, size, tolerance: float = 0.01):
        """A thin vertical plate (wall) as a row of vertical cylinders, each within
        `tolerance` of the faces. `size` = (thickness along x, length along y, height)."""
        t, length, height = size
        half_t = t / 2
        cell = 2 * math.sqrt(tolerance * (t + tolerance))  # largest cell length within tolerance
        n = max(1, math.ceil(length / cell))
        cell = length / n
        radius = math.hypot(half_t, cell / 2)
        z_mid = center[2]
        for i in range(n):
            y = center[1] - length / 2 + (i + 0.5) * cell
            self.obstacles.append({
                "id": f"{label}_{i}", "type": "cylinder",
                "center": [r(center[0]), r(y), r(z_mid)], "axis": [0.0, 0.0, 1.0],
                "radius": r(radius), "half_height": r(height / 2)})
        self.approximations.append({
            "label": label,
            "box": {"min": [r(center[0] - half_t), r(center[1] - length / 2), r(z_mid - height / 2)],
                    "max": [r(center[0] + half_t), r(center[1] + length / 2), r(z_mid + height / 2)]},
            "primitives": n, "max_outward_error": r(radius - half_t)})

    def add_board(self, label: str, lo, hi, tolerance: float = 0.002):
        """A thin axis-aligned box (shelf panel) as a row of cylinders along its longer
        in-plane axis, each within `tolerance` of the faces."""
        size = [h - l for l, h in zip(lo, hi)]
        thin = min(range(3), key=lambda k: size[k])
        along, across = sorted((k for k in range(3) if k != thin), key=lambda k: -size[k])
        half_t = size[thin] / 2
        cell = 2 * math.sqrt(tolerance * (size[thin] + tolerance))
        n = max(1, math.ceil(size[across] / cell))
        cell = size[across] / n
        radius = math.hypot(half_t, cell / 2)
        axis = [0.0, 0.0, 0.0]
        axis[along] = 1.0
        for i in range(n):
            center = [(l + h) / 2 for l, h in zip(lo, hi)]
            center[across] = lo[across] + (i + 0.5) * cell
            self.obstacles.append({
                "id": f"{label}_{i}", "type": "cylinder", "center": [r(v) for v in center], "axis": axis,
                "radius": r(radius), "half_height": r(size[along] / 2)})
        self.approximations.append({
            "label": label, "box": {"min": [r(v) for v in lo], "max": [r(v) for v in hi]},
            "primitives": n, "max_outward_error": r(radius - half_t)})

    # ---------------------------------------------------------------- output

    def to_json(self) -> dict:
        return {
            "format": FORMAT,
            "name": self.name,
            "environment": {"min": [r(v) for v in self.workspace_min],
                            "max": [r(v) for v in self.workspace_max]},
            "obstacles": self.obstacles,
            "robots": [a.to_json() for a in self.arms],
            "lazydash": {
                "objects": [b.to_json() for b in self.blocks],
                "grasps": self.grasps,
                "params": self.params,
                **({"placement_regions": self.placement_regions} if self.placement_regions else {}),
                "approximations": self.approximations,
                "source": self.source,
            },
        }

    def write(self, path: Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(json.dumps(self.to_json(), indent=1) + "\n")
        return path
