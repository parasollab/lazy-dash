"""Render a lazydash_solve result as an mp4 (offscreen, pyrender on EGL).

  PYOPENGL_PLATFORM=egl python3 viewer/render_video.py \
      resources/scenes/paper/<scene>.json results/videos/<scene>.json \
      --out results/videos/<scene>.mp4 [--title "Sorting: 4 robots"]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

import numpy as np
import pyrender
import trimesh
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
BLOCK_COLORS = [(0.85, 0.12, 0.10), (0.10, 0.70, 0.20), (0.12, 0.25, 0.80), (0.92, 0.92, 0.90)]
GRASP_REACH = 0.02  # m: a moving block this close to an arm's grasp point is held by it


# ------------------------------------------------------------------ geometry

def rpy_matrix(rpy):
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform(xyz=(0, 0, 0), rot=None):
    t = np.eye(4)
    t[:3, 3] = xyz
    if rot is not None:
        t[:3, :3] = rot
    return t


def look_at(eye, target):
    """Pose whose -z axis points from eye to target (camera / light convention)."""
    eye = np.asarray(eye, float)
    fwd = np.asarray(target, float) - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1])
    right = right / np.linalg.norm(right) if np.linalg.norm(right) > 1e-9 else np.array([1.0, 0, 0])
    up = np.cross(right, fwd)
    return transform(eye, np.column_stack([right, up, -fwd]))


def quat_matrix(q):  # xyzw
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def axis_angle(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def origin(elem):
    o = elem.find("origin") if elem is not None else None
    if o is None:
        return np.eye(4)
    xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    return transform(xyz, rpy_matrix(rpy))


class Urdf:
    """Kinematic tree and visual meshes of a URDF (fixed and revolute joints)."""

    def __init__(self, path: Path):
        root = ET.parse(path).getroot()
        self.joints = []  # (name, type, parent, child, origin, axis)
        for j in root.findall("joint"):
            axis = j.find("axis")
            self.joints.append((j.get("name"), j.get("type"), j.find("parent").get("link"),
                                j.find("child").get("link"), origin(j),
                                [float(v) for v in axis.get("xyz").split()] if axis is not None else [1, 0, 0]))
        children = {c for *_, c, _, _ in [(n, t, p, c, o, a) for n, t, p, c, o, a in self.joints]}
        links = [l.get("name") for l in root.findall("link")]
        self.root = next(l for l in links if l not in children)
        self.visuals = []  # (link, origin, trimesh list)
        for link in root.findall("link"):
            for vis in link.findall("visual"):
                mesh = vis.find("geometry/mesh")
                if mesh is None:
                    continue
                scale = [float(v) for v in mesh.get("scale", "1 1 1").split()]
                loaded = trimesh.load(path.parent / mesh.get("filename"), force="scene")
                meshes = []
                for name, geom in loaded.geometry.items():
                    for node in loaded.graph.geometry_nodes.get(name, []):
                        m = geom.copy()
                        m.apply_transform(loaded.graph.get(node)[0])
                        m.apply_scale(scale)
                        meshes.append(m)
                self.visuals.append((link.get("name"), origin(vis), meshes))

    def fk(self, values: dict, base: np.ndarray) -> dict:
        poses = {self.root: base}
        pending = list(self.joints)
        while pending:
            rest = []
            for name, kind, parent, child, o, axis in pending:
                if parent not in poses:
                    rest.append((name, kind, parent, child, o, axis))
                    continue
                t = poses[parent] @ o
                if kind in ("revolute", "continuous"):
                    t = t @ transform(rot=axis_angle(axis, values.get(name, 0.0)))
                poses[child] = t
            if len(rest) == len(pending):
                break
            pending = rest
        return poses


# ------------------------------------------------------------------ result

def times_of(path_len, stride, makespan):
    t = [i * stride for i in range(path_len)]
    if path_len > 1 and t[-1] != makespan and makespan > t[-2]:
        t[-1] = makespan
    return np.array(t, float)


def sample(path, times, t):
    """Linear interpolation of a waypoint list at timestep t."""
    p = np.asarray(path, float)
    if len(p) == 1 or t <= times[0]:
        return p[0]
    if t >= times[-1]:
        return p[-1]
    i = int(np.searchsorted(times, t, side="right")) - 1
    a = (t - times[i]) / max(times[i + 1] - times[i], 1e-9)
    return (1 - a) * p[i] + a * p[i + 1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", type=Path)
    ap.add_argument("result", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--title", default=None)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--min-seconds", type=float, default=20.0)
    ap.add_argument("--max-seconds", type=float, default=60.0)
    ap.add_argument("--speed", type=float, default=None, help="playback speed (default: fit min/max seconds)")
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--view", default="1,-1.25,0.9", help="camera direction from the target")
    ap.add_argument("--zoom", type=float, default=1.15)
    ap.add_argument("--crf", type=int, default=24)
    ap.add_argument("--stills", default=None,
                    help="comma-separated fractions of the makespan: write PNGs <out>_<k>.png instead of a video")
    args = ap.parse_args()
    width, height = (int(v) for v in args.size.split("x"))

    scene_doc = json.loads(args.scene.read_text())
    result = json.loads(args.result.read_text())
    stats = result["lazydash"]["stats"]
    stride = int(result["lazydash"].get("stride", 1))
    resolution = float(result.get("resolution", 128))
    makespan = float(stats.get("makespan") or 0)
    arms = [r for r in result["robots"] if r.get("robot_type") != "sphere"]
    blocks = [r for r in result["robots"] if r.get("robot_type") == "sphere"]
    objects = scene_doc["lazydash"]["objects"]
    grasps = scene_doc["lazydash"]["grasps"]
    grasp_t = transform(grasps[0]["translation"], rpy_matrix(grasps[0]["rpy"]))
    if not makespan:
        makespan = max((len(r["path"]) - 1) * stride for r in result["robots"])

    urdf = Urdf(ROOT / arms[0]["visual_urdf_path"])
    scene = pyrender.Scene(bg_color=[0.97, 0.97, 0.96, 1.0], ambient_light=[0.55, 0.55, 0.55])

    # Scenery: the boxes the scene approximated, else its primitives.
    boxes = scene_doc["lazydash"].get("approximations", [])
    pts = [np.array(a["base_pose"]["position"]) for a in arms]  # floor extent
    # Camera framing: the arms' working volume, the blocks and the non-table scenery.
    frame = []
    for a in arms:
        b = np.array(a["base_pose"]["position"])
        frame += [b + [dx, dy, dz] for dx in (-0.6, 0.6) for dy in (-0.6, 0.6) for dz in (0.0, 0.9)]
    if boxes:
        for b in boxes:
            lo, hi = np.array(b["box"]["min"]), np.array(b["box"]["max"])
            m = trimesh.creation.box(hi - lo, transform((lo + hi) / 2))
            thin = min(hi - lo) < 0.1 and (hi - lo)[2] > 0.2
            # Walls and other upright scenery see-through, so the arms behind stay visible.
            color = [0.55, 0.65, 0.78, 0.55] if thin else [0.93, 0.89, 0.82, 1.0]
            scene.add(pyrender.Mesh.from_trimesh(m, material=pyrender.MetallicRoughnessMaterial(
                baseColorFactor=color, roughnessFactor=0.9, alphaMode="BLEND" if thin else "OPAQUE")))
            pts += [lo, hi]
            if "note" not in b:  # not a slab
                frame += [lo, hi]
    else:
        for o in scene_doc.get("obstacles", []):
            c = np.array(o["center"])
            if o["type"] == "sphere":
                m = trimesh.creation.icosphere(radius=o["radius"])
                m.apply_translation(c)
            else:
                m = trimesh.creation.cylinder(radius=o["radius"], height=2 * o["half_height"])
                z = np.array(o.get("axis", [0, 0, 1]), float)
                rot = trimesh.geometry.align_vectors([0, 0, 1], z)
                m.apply_transform(rot)
                m.apply_translation(c)
            scene.add(pyrender.Mesh.from_trimesh(m, material=pyrender.MetallicRoughnessMaterial(
                baseColorFactor=[0.8, 0.72, 0.6, 1.0])))
            pts.append(c)

    # Floor.
    allp = np.array(pts)
    lo, hi = allp.min(0) - 0.6, allp.max(0) + 0.6
    floor = trimesh.creation.box([hi[0] - lo[0], hi[1] - lo[1], 0.01], transform([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, -0.005]))
    scene.add(pyrender.Mesh.from_trimesh(floor, material=pyrender.MetallicRoughnessMaterial(
        baseColorFactor=[0.90, 0.90, 0.88, 1.0], roughnessFactor=1.0)))

    # Arms.
    arm_nodes = []
    for a in arms:
        nodes = []
        for link, vis_o, meshes in urdf.visuals:
            node = scene.add(pyrender.Mesh.from_trimesh(meshes, smooth=False))
            nodes.append((link, vis_o, node))
        base = transform(a["base_pose"]["position"], quat_matrix(a["base_pose"]["quaternion_xyzw"]))
        names = a["joint_names"]
        arm_nodes.append((a, base, names, nodes, times_of(len(a["path"]), stride, makespan)))

    # Blocks and goals.
    size = objects[0]["shape"]["size"] if objects else [0.038] * 3
    block_state = []
    for i, b in enumerate(blocks):
        col = BLOCK_COLORS[i % 4]
        mat = pyrender.MetallicRoughnessMaterial(baseColorFactor=[*col, 1.0], roughnessFactor=0.6)
        node = scene.add(pyrender.Mesh.from_trimesh(trimesh.creation.box(size), material=mat))
        spec = objects[i] if i < len(objects) else None
        rot = rpy_matrix(spec["start"]["rpy"]) if spec else np.eye(3)
        if spec and spec.get("goals"):
            g = spec["goals"][0]
            gp = np.array(g["position"]) - [0, 0, size[2] / 2 - 0.001]
            square = trimesh.creation.box([size[0] * 1.6, size[1] * 1.6, 0.002], transform(gp, rpy_matrix([0, 0, g["rpy"][2]])))
            scene.add(pyrender.Mesh.from_trimesh(square, material=pyrender.MetallicRoughnessMaterial(
                baseColorFactor=[*col, 0.45], alphaMode="BLEND")))
        block_state.append({"node": node, "rot": rot, "prev": None,
                            "times": times_of(len(b["path"]), stride, makespan), "path": b["path"]})
        frame.append(np.array(b["path"][0]))
        frame += [np.array(g["position"]) for g in (spec or {}).get("goals", [])]

    # Camera and lights.
    allp = np.array(frame)
    center = (allp.min(0) + allp.max(0)) / 2
    center[2] = max(center[2], 0.5)
    radius = np.linalg.norm(allp.max(0) - allp.min(0)) / 2 + 0.3
    d = np.array([float(v) for v in args.view.split(",")])
    d /= np.linalg.norm(d)
    yfov = math.radians(40)
    eye = center + d * radius / math.sin(yfov / 2) * 0.85 / args.zoom
    cam = look_at(eye, center)
    scene.add(pyrender.PerspectiveCamera(yfov=yfov, aspectRatio=width / height, znear=0.05, zfar=50), pose=cam)
    scene.add(pyrender.DirectionalLight(color=[1, 1, 1], intensity=4.5),
              pose=look_at(center + np.array([0.6, -0.9, 2.0]), center))
    scene.add(pyrender.DirectionalLight(color=[1, 1, 1], intensity=1.5), pose=cam)

    # Timing.
    sim_seconds = makespan / resolution
    speed = args.speed or max(sim_seconds / args.max_seconds, min(1.0, sim_seconds / args.min_seconds), 1.0)
    frames = max(2, int(sim_seconds / speed * args.fps))

    renderer = pyrender.OffscreenRenderer(width, height)
    font = ImageFont.load_default(size=22) if hasattr(ImageFont, "load_default") else None
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 22)
        small = ImageFont.truetype("DejaVuSans.ttf", 17)
    except OSError:
        small = font
    title = args.title or scene_doc.get("name", args.scene.stem)
    sub = (f"{len(arms)} robots, {len(blocks)} objects   planning {stats.get('planning_time', 0):.1f} s"
           f"   makespan {sim_seconds:.1f} s   {'solved' if result.get('success') else 'FAILED'}")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    stills = [float(v) for v in args.stills.split(",")] if args.stills else None
    if stills:
        frames = len(stills)
    ff = None if stills else subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                           "-s", f"{width}x{height}", "-r", str(args.fps), "-i", "-", "-c:v", "libx264",
                           "-preset", "medium", "-crf", str(args.crf), "-pix_fmt", "yuv420p",
                           "-movflags", "+faststart", str(args.out)], stdin=subprocess.PIPE)
    flanges = [None] * len(arms)
    for f in range(frames):
        t = makespan * (stills[f] if stills else f / (frames - 1))
        for k, (a, base, names, nodes, times) in enumerate(arm_nodes):
            q = sample(a["path"], times, t)
            poses = urdf.fk(dict(zip(names, q)), base)
            for link, vis_o, node in nodes:
                scene.set_pose(node, poses[link] @ vis_o)
            flanges[k] = poses.get("flange")
        for s in block_state:
            p = sample(s["path"], s["times"], t)
            if s["prev"] is not None and np.linalg.norm(p - s["prev"]) > 1e-5:
                for fl in flanges:
                    if fl is not None and np.linalg.norm((fl @ grasp_t)[:3, 3] - p) < GRASP_REACH:
                        s["rot"] = (fl @ grasp_t)[:3, :3]
                        break
            s["prev"] = p
            scene.set_pose(s["node"], transform(p, s["rot"]))
        color, _ = renderer.render(scene)
        img = Image.fromarray(color)
        draw = ImageDraw.Draw(img)
        draw.rectangle([0, 0, width, 62], fill=(250, 250, 248))
        draw.text((18, 8), title, fill=(30, 30, 30), font=font)
        draw.text((18, 36), sub, fill=(90, 90, 90), font=small)
        clock = f"t = {t / resolution:5.1f} s   x{speed:.1f}"
        draw.text((width - 230, 36), clock, fill=(90, 90, 90), font=small)
        if stills:
            img.save(args.out.with_name(f"{args.out.stem}_{f}.png"))
        else:
            ff.stdin.write(img.tobytes())
    renderer.delete()
    if stills:
        print(f"wrote {frames} stills next to {args.out}")
        return
    ff.stdin.close()
    ff.wait()
    print(f"{args.out}: {frames} frames, x{speed:.2f}, {args.out.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
