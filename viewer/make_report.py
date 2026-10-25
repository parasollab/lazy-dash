"""The results page (viewer/report/index.html): results against the paper's runs, videos, timelines.

  python3 viewer/make_report.py --trials results/paper/trials.jsonl \
      --scenes results/videos --out results/report
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "runner"))
from compare_paper import load_ours, load_paper  # noqa: E402

STEPS_PER_SECOND = 128

# (tab id, label, [(paper family, label, representative scene, robots)]); videos are named by family.
TABS = [
    ("sorting", "Sorting", [("sorting-4robots", "4 robots", "sorting-4robots-16objects", 4),
                            ("sorting-9robots", "9 robots", "sorting-9robots-16objects", 9),
                            ("sorting-16robots", "16 robots", "sorting-16robots-16objects", 16)]),
    ("wall", "Wall", [("wall-2walls", "2 walls", "wall-2walls-16objects", 8),
                      ("wall-4walls", "4 walls", "wall-4walls-16objects", 8),
                      ("wall-6walls", "6 walls", "wall-6walls-16objects", 8)]),
    ("shelf-wall", "Shelf-wall", [("shelf-wall-2robots", "2 robots / 2 shelves", "shelf-wall-2robots-16objects", 2),
                                  ("shelf-wall-4robots", "4 robots / 4 shelves", "shelf-wall-4robots-16objects", 4)]),
    ("stocking", "Stocking", [("stocking-2layers", "2 robots / 2 layers", "stocking-2layers-12objects", 2),
                              ("stocking-3layers", "2 robots / 3 layers", "stocking-3layers-9objects", 2)]),
]


def series(table, scenario):
    rows = []
    for (s, k), v in sorted(table.items(), key=lambda kv: kv[0][1]):
        if s != scenario:
            continue
        rows.append({"objects": k, "success": v.get("success"), "time": v.get("time"),
                     "makespan": v.get("makespan"), "runs": v.get("runs")})
    return rows


def scene_data(path: Path):
    doc = json.loads(path.read_text())
    ld = doc["lazydash"]
    s = ld["stats"]
    arms = [r["name"] for r in doc["robots"] if r.get("robot_type") == "ur5e_hande"]
    timeline = [[e["robot"], round(e["start"] / STEPS_PER_SECOND, 3), round(e["end"] / STEPS_PER_SECOND, 3),
                 e["kind"], e["action"], e["object"], int(e["holding"]), e["step"]] for e in ld["timeline"]]
    steps = ld["task_plan"]
    return {
        "arms": len(arms),
        "objects": sum(1 for r in doc["robots"] if r.get("robot_type") == "sphere"),
        "makespan_s": s["makespan"] / doc.get("resolution", STEPS_PER_SECOND),
        "planning_s": s["planning_time"],
        "transitions": sum(1 for t in steps if t.startswith(("pick", "place", "handover"))),
        "handovers": sum(1 for t in steps if t.startswith("handover")),
        "stats": {k: s[k] for k in ("task_query_time", "task_conflict_time", "motion_query_time", "conflict_resolution_time",
                                    "task_constraints", "motion_constraints", "outer_iterations",
                                    "hh_conflicts", "hv_conflicts", "move_outs")},
        "timeline": timeline,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=Path, required=True)
    ap.add_argument("--scenes", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    paper, ours = load_paper(), load_ours(a.trials)
    (a.out / "videos").mkdir(parents=True, exist_ok=True)
    data = {"families": []}
    for tid, tlabel, variants in TABS:
        fam = {"id": tid, "label": tlabel, "variants": []}
        for fam_id, vlabel, scene, robots in variants:
            shutil.copy(a.scenes / f"{fam_id}.mp4", a.out / "videos" / f"{fam_id}.mp4")
            fam["variants"].append({
                "id": fam_id, "label": vlabel, "robots": robots, "scene": scene, "video": f"videos/{fam_id}.mp4",
                "results": {"paper": series(paper, fam_id), "ours": series(ours, fam_id)},
                "run": scene_data(a.scenes / f"{scene}.json"),
            })
        data["families"].append(fam)
    (a.out / "data.json").write_text(json.dumps(data, separators=(",", ":")))
    shutil.copy(Path(__file__).resolve().parent / "report" / "index.html", a.out / "index.html")
    print(f"wrote {a.out / 'data.json'}")


if __name__ == "__main__":
    main()
