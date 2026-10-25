"""Compare lazydash trials with the paper's runs (comparison.md and comparison.csv).

  python3 runner/compare_paper.py results/trials.jsonl --out results/comparison
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PAPER_RUNS = ROOT / "runner" / "data" / "paper_runs.csv"
OURS_PER_PAPER_STEP = 128 * math.pi / 100  # ours: 128 steps/s at 1 rad/s; the paper's runs: 100/pi steps/rad

# The paper's scenarios: family id (scene-name prefix) and label.
FAMILIES = [
    ("sorting-4robots", "Sorting: 4 robots"), ("sorting-9robots", "Sorting: 9 robots"),
    ("sorting-16robots", "Sorting: 16 robots"), ("wall-2walls", "Wall: 8 robots / 2 walls"),
    ("wall-4walls", "Wall: 8 robots / 4 walls"), ("wall-6walls", "Wall: 8 robots / 6 walls"),
    ("shelf-wall-2robots", "Shelf-wall: 2 robots / 2 shelves"),
    ("shelf-wall-4robots", "Shelf-wall: 4 robots / 4 shelves"),
    ("stocking-2layers", "Stocking: 2 robots / 2 layers"), ("stocking-3layers", "Stocking: 2 robots / 3 layers"),
]


def family(scene: str) -> tuple[str, int] | None:
    """(family id, objects) of a paper scene name such as sorting-4robots-16objects."""
    m = re.match(r"(sorting-\d+robots|wall-\d+walls|shelf-wall-\d+robots|stocking-\d+layers)-(\d+)objects", scene)
    return (m.group(1), int(m.group(2))) if m else None


def median(xs):
    xs = [x for x in xs if x is not None]
    return statistics.median(xs) if xs else None


def paper_rows():
    """The paper's runs, (family, objects) -> rows."""
    groups = defaultdict(list)
    with PAPER_RUNS.open(newline="") as f:
        for r in csv.DictReader(f):
            groups[(r["family"], int(r["objects"]))].append(r)
    return groups


def load_paper():
    out = {}
    for key, rows in paper_rows().items():
        ok = [r for r in rows if r["success"] == "1"]
        num = lambda k: [float(r[k]) for r in ok if r[k]]
        out[key] = {"runs": len(rows), "success": len(ok) / len(rows), "time": median(num("planning_time")),
                    "makespan": median(num("makespan")), "ht": median(num("ht_vertices"))}
    return out


def load_ours(path: Path):
    """(family, objects) -> summary; makespan in the paper's timesteps."""
    groups = defaultdict(list)
    for line in path.read_text().splitlines():
        r = json.loads(line)
        key = family(r["scene"])
        if key:
            groups[key].append(r)
    out = {}
    for key, rows in groups.items():
        ok = [r for r in rows if r.get("success")]
        out[key] = {
            "runs": len(rows),
            "success": len(ok) / len(rows),
            "time": median([r.get("planning_time") for r in ok]),
            "makespan": median([r["makespan"] / OURS_PER_PAPER_STEP for r in ok if r.get("makespan")]),
            "ht": median([r.get("ht_vertices") for r in rows]),
        }
    return out


def fmt(x, digits=1):
    if x is None:
        return "-"
    if isinstance(x, float) and x <= 1.0 and digits == 0:
        return f"{100 * x:.0f}%"
    return f"{x:.{digits}f}" if isinstance(x, float) else str(x)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("trials", type=Path)
    ap.add_argument("--out", type=Path, default=ROOT / "results" / "comparison")
    args = ap.parse_args()
    paper, ours = load_paper(), load_ours(args.trials)
    order = {f: i for i, (f, _) in enumerate(FAMILIES)}
    keys = sorted(ours, key=lambda k: (order.get(k[0], len(order)), k[1]))
    args.out.mkdir(parents=True, exist_ok=True)
    header = ["scenario", "objects", "runs", "success", "paper success", "time s", "paper time s",
              "makespan", "paper makespan", "makespan / paper", "H_T V", "paper H_T V"]
    rows, ratios = [], []
    for k in keys:
        o, b = ours[k], paper.get(k, {})
        ratio = o["makespan"] / b["makespan"] if o["makespan"] and b.get("makespan") else None
        if ratio:
            ratios.append(ratio)
        rows.append([k[0], k[1], o["runs"], fmt(o["success"], 0), fmt(b.get("success"), 0),
                     fmt(o["time"]), fmt(b.get("time")), fmt(o["makespan"], 0), fmt(b.get("makespan"), 0),
                     fmt(ratio, 2), fmt(o["ht"], 0), fmt(b.get("ht"), 0)])
    with (args.out / "comparison.csv").open("w", newline="") as f:
        csv.writer(f).writerows([header] + rows)
    md = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    md += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    if ratios:
        md += ["", f"Makespan / paper (paper timesteps): median {median(ratios):.2f} over {len(ratios)} scenes "
                   f"(min {min(ratios):.2f}, max {max(ratios):.2f})."]
    (args.out / "comparison.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
