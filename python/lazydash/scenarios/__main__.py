"""Write scene files.

  python -m lazydash.scenarios sorting --arms 4 --blocks 8 --seed 0 -o out.json
  python -m lazydash.scenarios set-grasps scene.json --set faces -o my-scenes
  python -m lazydash.scenarios generated resources/scenes
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from lazydash.scenarios.ring import make_ring
from lazydash.scenarios.sorting import make_sorting
from lazydash.scenarios.wall import make_wall
from lazydash.scenarios.workcell import make_assembly, make_depalletize, make_packing
from lazydash.scene import face_grasps

# Generated sorting and wall scenes (seed 0), used by the tests and as examples.
GENERATED = [("sorting", a, n) for a in (4, 9, 16) for n in (4, 16, 40)] + \
            [("wall", w, n) for w in (2, 4, 6) for n in (4, 16, 40)]


def extra_scenes():
    """The extra benchmark: (folder, scene). Ring cell with 6, 12 and 24 blocks;
    bay walls, towers of 2 and 3, and their combinations; three workcells."""
    rings = [make_ring(6, n) for n in (6, 12, 24)]
    rings += [make_ring(6, 12, bays=True), make_ring(6, 12, stack=2), make_ring(6, 12, bays=True, stack=2),
              make_ring(6, 18, stack=3), make_ring(6, 18, bays=True, stack=3)]
    cells = [make_packing(), make_depalletize(), make_assembly()]
    return [("ring", s) for s in rings] + [("workcell", s) for s in cells]


def set_grasps(paths, grasp_set, out_dir):
    """Copies of scene files with another grasp set, at each scene's grasp depth."""
    out_dir.mkdir(parents=True, exist_ok=True)
    for path in paths:
        scene = json.loads(path.read_text(encoding="utf-8"))
        ld = scene["lazydash"]
        first = ld["grasps"][0]
        approach = first.get("approach_translation", first["translation"])[0]
        ld["grasps"] = {"faces": face_grasps}[grasp_set](first["translation"][0], approach)
        scene["name"] = f"{scene['name']}-{grasp_set}"
        ld.setdefault("source", {})["grasp_set"] = grasp_set
        out = out_dir / f"{scene['name']}.json"
        with open(out, "w", encoding="utf-8", newline="\n") as f:
            f.write(json.dumps(scene, indent=1) + "\n")
        print(f"{out}: {len(ld['grasps'])} grasps")


def main():
    ap = argparse.ArgumentParser(prog="python -m lazydash.scenarios")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("sorting")
    s.add_argument("--arms", type=int, default=4)
    s.add_argument("--blocks", type=int, default=4)
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("-o", "--out", type=Path, required=True)
    s.add_argument("--joint-limits", choices=["paper", "ur5e"], default="paper")
    s.add_argument("--grasp-symmetry", action="store_true")
    s.add_argument("--grasp-set", choices=["paper", "faces"], default="paper",
                   help="paper: the paper's top and bottom grasps; faces: every face, two finger orientations")
    w = sub.add_parser("wall")
    w.add_argument("--walls", type=int, default=2)
    w.add_argument("--blocks", type=int, default=4)
    w.add_argument("--seed", type=int, default=0)
    w.add_argument("-o", "--out", type=Path, required=True)
    w.add_argument("--joint-limits", choices=["paper", "ur5e"], default="paper")
    w.add_argument("--grasp-symmetry", action="store_true")
    w.add_argument("--grasp-set", choices=["paper", "faces"], default="paper",
                   help="paper: the paper's top and bottom grasps; faces: every face, two finger orientations")
    g = sub.add_parser("ring")
    g.add_argument("--arms", type=int, default=6)
    g.add_argument("--blocks", type=int, default=12)
    g.add_argument("--seed", type=int, default=0)
    g.add_argument("-o", "--out", type=Path, required=True)
    g.add_argument("--joint-limits", choices=["paper", "ur5e"], default="paper")
    g.add_argument("--grasp-symmetry", action="store_true")
    g.add_argument("--grasp-set", choices=["paper", "faces"], default="paper")
    g.add_argument("--bays", action="store_true", help="a wall between every two neighbouring stations")
    g.add_argument("--stack", type=int, default=1, help="tower height: blocks start stacked, rebuilt in order")
    c = sub.add_parser("workcell")
    c.add_argument("kind", choices=["packing", "depalletize", "assembly"])
    c.add_argument("--count", type=int, help="items (packing), boxes (depalletize) or stacks (assembly)")
    c.add_argument("--seed", type=int, default=0)
    c.add_argument("-o", "--out", type=Path, required=True)
    t = sub.add_parser("set-grasps", help="copy scenes with another grasp set")
    t.add_argument("scenes", nargs="+", type=Path)
    t.add_argument("--set", choices=["faces"], default="faces")
    t.add_argument("-o", "--out-dir", type=Path, required=True)
    d = sub.add_parser("generated")
    d.add_argument("dir", type=Path, help="the scenes folder (writes generated/)")
    e = sub.add_parser("extra")
    e.add_argument("dir", type=Path, help="the scenes folder (writes extra/)")
    args = ap.parse_args()

    if args.cmd == "sorting":
        print(make_sorting(args.arms, args.blocks, args.seed, args.joint_limits,
                           args.grasp_symmetry, args.grasp_set).write(args.out))
    elif args.cmd == "wall":
        print(make_wall(args.walls, args.blocks, args.seed, args.joint_limits,
                        args.grasp_symmetry, args.grasp_set).write(args.out))
    elif args.cmd == "ring":
        print(make_ring(args.arms, args.blocks, args.seed, args.joint_limits,
                        args.grasp_symmetry, args.grasp_set, args.bays, args.stack).write(args.out))
    elif args.cmd == "workcell":
        make = {"packing": make_packing, "depalletize": make_depalletize, "assembly": make_assembly}[args.kind]
        scene = make(seed=args.seed) if args.count is None else make(args.count, args.seed)
        print(scene.write(args.out))
    elif args.cmd == "set-grasps":
        set_grasps(args.scenes, args.set, args.out_dir)
    elif args.cmd == "extra":
        for folder, scene in extra_scenes():
            print(scene.write(args.dir / "extra" / folder / f"{scene.name}.json"))
    else:
        for kind, k, n in GENERATED:
            scene = make_sorting(k, n) if kind == "sorting" else make_wall(k, n)
            print(scene.write(args.dir / "generated" / f"{scene.name}.json"))


if __name__ == "__main__":
    main()
