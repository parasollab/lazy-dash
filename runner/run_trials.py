"""Run lazydash_solve over scenes x seeds, in parallel, resumable.

  python3 runner/run_trials.py --scenes resources/scenes/paper/sorting-4robots-*.json \
      --seeds 1-50 --jobs 24 --time-limit 1800 --out results/trials.jsonl
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import glob
import json
import subprocess
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def parse_seeds(text: str) -> list[int]:
    seeds: list[int] = []
    for part in text.split(","):
        if "-" in part:
            a, b = part.split("-")
            seeds += list(range(int(a), int(b) + 1))
        else:
            seeds.append(int(part))
    return seeds


def run_one(solver: Path, scene: Path, seed: int, time_limit: float) -> dict:
    cmd = [str(solver), str(scene), "--seed", str(seed), "--time-limit", str(time_limit)]
    t0 = time.time()
    record = {"scene": scene.stem, "seed": seed, "time_limit": time_limit}
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=time_limit * 1.5 + 60)
        record["returncode"] = proc.returncode
        lines = [ln for ln in proc.stdout.splitlines() if ln.startswith("{")]
        if lines:
            record.update(json.loads(lines[-1]))
        else:
            record["success"] = False
            record["failure"] = "no stats (" + proc.stderr[-200:].strip() + ")"
    except subprocess.TimeoutExpired:
        record["returncode"] = None
        record["success"] = False
        record["failure"] = "wall-clock timeout"
    record["wall"] = time.time() - t0
    return record


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", nargs="+", required=True, help="scene files or globs")
    ap.add_argument("--seeds", default="1-10")
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--time-limit", type=float, default=300.0)
    ap.add_argument("--solver", type=Path, default=ROOT / "build" / "lazydash_solve")
    ap.add_argument("--out", type=Path, default=ROOT / "results" / "trials.jsonl")
    args = ap.parse_args()

    scenes = sorted({Path(p) for pattern in args.scenes for p in glob.glob(pattern)})
    seeds = parse_seeds(args.seeds)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    done = set()
    if args.out.exists():
        for line in args.out.read_text().splitlines():
            r = json.loads(line)
            done.add((r["scene"], r["seed"]))
    todo = [(s, seed) for s in scenes for seed in seeds if (s.stem, seed) not in done]
    print(f"{len(todo)} trials to run ({len(done)} done), {args.jobs} jobs")
    lock = threading.Lock()
    with cf.ThreadPoolExecutor(max_workers=args.jobs) as pool, args.out.open("a") as out:
        futures = [pool.submit(run_one, args.solver, s, seed, args.time_limit) for s, seed in todo]
        for i, fut in enumerate(cf.as_completed(futures), 1):
            r = fut.result()
            with lock:
                out.write(json.dumps(r) + "\n")
                out.flush()
            print(f"[{i}/{len(todo)}] {r['scene']} seed {r['seed']}: success {r.get('success')} "
                  f"in {r['wall']:.1f} s ({r.get('failure', '')})", flush=True)


if __name__ == "__main__":
    main()
