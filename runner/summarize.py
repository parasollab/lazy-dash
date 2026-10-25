"""Per-scene summary of a trials file, as a Markdown table.

  python3 runner/summarize.py results/extra/trials.jsonl
"""
from __future__ import annotations

import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path

STEPS_PER_SECOND = 128


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("trials", type=Path)
    a = ap.parse_args()
    rows = defaultdict(list)
    for line in a.trials.read_text().splitlines():
        if line.strip():
            r = json.loads(line)
            rows[r["scene"]].append(r)
    print("| scene | solved | planning s (median / max) | makespan s | task constraints |")
    print("|---|---|---|---|---|")
    total = solved = 0
    for scene in sorted(rows):
        rs = rows[scene]
        ok = [r for r in rs if r.get("success")]
        total, solved = total + len(rs), solved + len(ok)
        if ok:
            t = [r["planning_time"] for r in ok]
            cells = [f"{statistics.median(t):.1f} / {max(t):.1f}",
                     f"{statistics.median(r['makespan'] for r in ok) / STEPS_PER_SECOND:.0f}",
                     f"{statistics.median(r['task_constraints'] for r in ok):.0f}"]
        else:
            cells = ["-", "-", "-"]
        print(f"| {scene} | {len(ok)}/{len(rs)} | " + " | ".join(cells) + " |")
    print(f"\nSolved {solved} of {total}.")


if __name__ == "__main__":
    main()
