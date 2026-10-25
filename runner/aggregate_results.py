"""Aggregate trials for the results browser (viewer/results.html).

  python3 runner/aggregate_results.py lazydash=results/trials.jsonl \
      [--out results/paper/results.json]
"""
from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from compare_paper import FAMILIES, OURS_PER_PAPER_STEP, ROOT, family, paper_rows  # noqa: E402

FORMAT = "lazydash.results/1"

# Metric -> summed keys, for the paper's runs (runner/data/paper_runs.csv) and for ours.
PAPER_KEYS = {
    "total": ["planning_time"], "task": ["task_time"], "motion": ["motion_time"],
    "conflict": ["conflict_resolution_time"], "htv": ["ht_vertices"], "hte": ["ht_hyperarcs"],
    "hmv": ["hm_vertices"], "hme": ["hm_hyperarcs"], "makespan": ["makespan"],
}
OUR_KEYS = {
    "total": ["planning_time"], "task": ["task_query_time", "task_conflict_time"], "motion": ["motion_query_time"],
    "conflict": ["conflict_resolution_time"], "htv": ["ht_vertices"], "hte": ["ht_hyperarcs"],
    "hmv": ["hm_vertices"], "hme": ["hm_hyperarcs"], "makespan": ["makespan"],
}


def total(row, keys):
    vals = [row.get(k) for k in keys]
    if any(v in (None, "") for v in vals):
        return None
    return sum(float(v) for v in vals)


def summary(vals):
    """[median, q1, q3, n] of the positive values, or None."""
    vals = [v for v in vals if v is not None and v > 0]
    if not vals:
        return None
    q = statistics.quantiles(vals, n=4) if len(vals) > 1 else [vals[0]] * 3
    return [rnd(statistics.median(vals)), rnd(q[0]), rnd(q[2]), len(vals)]


def rnd(x):
    return float(f"{x:.5g}")


def point(objects, rows, ok, keys, scale=None):
    p = {"objects": objects, "runs": len(rows), "solved": len(ok), "success": rnd(len(ok) / len(rows))}
    for metric, ks in keys.items():
        vals = [total(r, ks) for r in ok]
        if scale and metric in scale:
            vals = [v / scale[metric] if v is not None else None for v in vals]
        s = summary(vals)
        if s:
            p[metric] = s
    return p


def load_trials(path: Path):
    """Family key -> rows."""
    out = defaultdict(list)
    with path.open(encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            r = json.loads(line)
            key = family(r["scene"])
            if key:
                out[key].append(r)
    return out


def slug(s: str) -> str:
    return re.sub(r"[^a-z0-9]+", "-", s.lower()).strip("-") or "series"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("trials", nargs="*", help="LABEL=PATH or PATH (JSONL)")
    ap.add_argument("--out", type=Path, default=ROOT / "results" / "paper" / "results.json")
    a = ap.parse_args()

    series, points = [], defaultdict(dict)  # points[family][series id] -> [point]
    for arg in a.trials:
        label, _, path = arg.rpartition("=") if "=" in arg else ("", "", arg)
        path = Path(path)
        label = label or path.stem
        fams = load_trials(path)
        sid = slug(label)
        while any(s["id"] == sid for s in series):
            sid += "-2"
        series.append({"id": sid, "label": label, "source": path.as_posix(), "runs": sum(len(r) for r in fams.values())})
        for (fam, n), rows in fams.items():
            ok = [r for r in rows if r.get("success")]
            points[fam].setdefault(sid, []).append(point(n, rows, ok, OUR_KEYS, {"makespan": OURS_PER_PAPER_STEP}))

    labels = dict(FAMILIES)
    wanted = set(points) or set(labels)
    paper = {"id": "paper", "label": "Paper", "source": "runner/data/paper_runs.csv", "reference": True, "runs": 0}
    for (fam, n), rows in paper_rows().items():
        if fam not in wanted:
            continue
        ok = [r for r in rows if r["success"] == "1"]
        points[fam].setdefault("paper", []).append(point(n, rows, ok, PAPER_KEYS))
        paper["runs"] += len(rows)

    order = {f: i for i, (f, _) in enumerate(FAMILIES)}
    families = []
    for fam in sorted(points, key=lambda f: (order.get(f, len(order)), f)):
        pts = {sid: sorted(ps, key=lambda p: p["objects"]) for sid, ps in points[fam].items()}
        families.append({"id": fam, "label": labels.get(fam, fam), "points": pts})
    doc = {"format": FORMAT, "paper_step": rnd(OURS_PER_PAPER_STEP), "series": [paper] + series, "families": families}
    a.out.parent.mkdir(parents=True, exist_ok=True)
    with a.out.open("w", encoding="utf-8", newline="\n") as f:
        json.dump(doc, f, separators=(",", ":"))
        f.write("\n")
    print(f"wrote {a.out}: {len(families)} families, {len(series)} series + paper")


if __name__ == "__main__":
    main()
