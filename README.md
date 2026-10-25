# Lazy-DaSH on CoMotion (v1)

A C++20 implementation of **Lazy-DaSH**, lazy hypergraph-based multi-robot
task and motion planning (Lee, Motes, Ngui, Morales, Amato,
[arXiv:2504.05552](https://arxiv.org/abs/2504.05552)), built on
[CoMotion](https://github.com/parasollab/comotion). Several UR5e arms with
Robotiq Hand-e grippers move blocks from start to goal poses by picks, places
and handovers. The planner follows the paper's four layers: a task query over
the task hypergraph, task conflict detection, lazy motion planning, and motion
conflict resolution with CoMotion's ARC.

## Requirements

Linux (tested on Ubuntu 24.04, gcc 14). CoMotion's dependencies:

```bash
sudo apt install build-essential cmake git pkg-config libeigen3-dev libfcl-dev \
    libboost-serialization-dev libboost-program-options-dev nlohmann-json3-dev python3
```

Python 3.10+ runs the benchmark scripts (standard library only). Scene
generation needs `numpy` (here run through [uv](https://docs.astral.sh/uv/)).

## Build and test

```bash
external/bootstrap_comotion.sh                   # CoMotion at the pinned commit, plus this repo's patches
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/tests/lazydash_tests
```

`external/bootstrap_comotion.sh` clones CoMotion into `external/comotion` at the
commit in `external/COMOTION_PIN` and applies `external/patches/*.patch`
(allowed contacts, collision padding, faster collision checks; the patches
leave validity answers unchanged). The build also makes the Python module
`lazydash._core` when Python development headers are found.

## Solve a scene

```bash
./build/lazydash_solve resources/scenes/paper/sorting-4robots-16objects.json \
    --seed 1 --time-limit 300 --out results/result.json
```

It prints one line of JSON statistics and exits 0 on success: `success`,
`makespan` (timesteps, 128 per second), `planning_time` and the time per layer
(`task_query_time`, `task_conflict_time`, `motion_query_time`,
`conflict_resolution_time`, in seconds), representation sizes (`ht_vertices`,
`ht_hyperarcs`, `hm_vertices`, `hm_hyperarcs`) and constraint and conflict
counts. Options: `--seed N`, `--time-limit S`, `--out FILE` (the plan:
joint paths of every arm, object paths, the task plan and an arm timeline),
`--stride K` (keep every K-th timestep in `--out`), `--log` (the planner's log
on stderr), `--verbose` (progress), and tuning values `--stage-time S`,
`--detour F` (task-order time estimates), `--rrt-time S`, `--roadmap-samples N`
(motion planning). Watch a result in the browser:

```bash
python3 viewer/serve.py        # http://127.0.0.1:8765/viewer/ : pick result.json in the result viewer
```

From Python (after the build, with `python/` on the path):

```python
import lazydash
scene = lazydash.Scene.load("resources/scenes/paper/sorting-4robots-16objects.json")
result = lazydash.solve(scene)
print(result.success, lazydash.stats(result)["makespan"])
```

## Benchmarks

Two benchmark suites, both in `resources/scenes/`. Every scene is one file
named after its scenario and object count, e.g. `sorting-4robots-16objects.json`.

**Paper benchmark** (`resources/scenes/paper/`): the scenarios of the paper's
evaluation, with the paper's own start and goal placements.

| Scenario | Files | Task |
|---|---|---|
| Sorting: 4, 9 and 16 robots | `sorting-{4,9,16}robots-*` | robots sort colored objects to boxes across the table; each object is handed over one or more times |
| Wall: 8 robots / 2, 4 and 6 walls | `wall-{2,4,6}walls-*` | Sorting with thin walls between robots; handovers must reach around them |
| Shelf-wall: 2 robots / 2 shelves, 4 robots / 4 shelves | `shelf-wall-{2,4}robots-*` | objects go onto shelves that also act as walls; a rear object must be placed before the one in front of it |
| Stocking: 2 robots / 2 and 3 layers | `stocking-{2,3}layers-*` | new items go on a shelf behind items already there; the planner must move items out of the way and back (non-monotone) |

`runner/run_eval.sh paper` runs Sorting and Wall at 4, 16 and 40 objects
(seeds 1-50) and Shelf-wall and Stocking at every object count (seeds 1-10).
The folder holds more object counts (and Sorting with 2 robots) for your own
runs. `runner/compare_paper.py` compares a run with the paper's runs
(`runner/data/paper_runs.csv`), makespans in the paper's units.

**Extra benchmark** (`resources/scenes/extra/`): scenarios the paper does not
have, to test generalization (seeds 1-10). Regenerate them with
`cd python && uv run --no-project --with numpy python -m lazydash.scenarios extra ../resources/scenes`.

| Scene | What it tests |
|---|---|
| `ring-6-{6,12,24}` | six robots on a hexagon around a central pillar; every object goes to the opposite station (three handovers, either way round, two-way traffic) |
| `ring-6-12-bays` | the ring with a wall between every two neighbouring stations |
| `ring-6-12-stack2`, `ring-6-18-stack3` | objects start in towers of 2 or 3, rebuilt in the same order at the opposite station (top off first, bottom down first) |
| `ring-6-12-bays-stack2`, `ring-6-18-bays-stack3` | walls and towers together |
| `packing-8` | 2 robots: tote, handover over a safety partition, shipping box filled in two layers |
| `depalletize-12` | 3 robots in a line: pallet of 3-high stacks to two rows on a conveyor |
| `assembly-3` | 4 robots around a cabinet: three feeders, one station building base-middle-top stacks |

Run a suite on a multi-core Linux host, after configuring the build as above:

```bash
runner/run_eval.sh paper       # -> results/paper/: trials.jsonl, summary.md, comparison/ (against the paper's runs)
runner/run_eval.sh extra       # -> results/extra/: trials.jsonl, summary.md
```

The script builds, runs the tests, then solves every scene and seed with an
1800 s limit (`JOBS` solves in parallel, default: the number of cores; `BUILD`
names the build folder, default `build`).

## View results

```bash
python3 viewer/serve.py    # http://127.0.0.1:8765/viewer/
```

- **One solve:** `result.html` plays back a `--out` file (arms, blocks, the
  arm timeline, statistics); it lists the files in `results/`.
- **A benchmark run:** `results.html` (default `results/paper/results.json`)
  plots planning time per layer, representation sizes, success rate and
  makespan per scenario against the paper's runs, as in the paper's figures.
  The extra suite has no paper runs to compare with: see
  `results/extra/summary.md`.
- **Videos and a results page:** `viewer/make_videos.sh` renders one solve per
  scenario to mp4, and `viewer/make_report.py` builds a self-contained page
  of videos, timelines and results; see `viewer/README.md` for its
  dependencies.

## Your own scenes

A scene is one JSON file (`lazydash.scene/1`): arms, sphere and cylinder
obstacles, blocks with a start pose and alternative goal poses, the grasp
set and task parameters. Write it from Python with
`python/lazydash/scene.py` (box-shaped scenery is turned into cylinders
there), or start from a generator:

```bash
cd python
uv run --no-project --with numpy python -m lazydash.scenarios ring --arms 6 --blocks 12 --bays --stack 2 -o ../resources/scenes/mine/my-ring.json
uv run --no-project --with numpy python -m lazydash.scenarios workcell packing -o ../resources/scenes/mine/my-packing.json
```

Other generators: `sorting`, `wall`; options `--grasp-set faces` (grasps
from every face of the block), `--grasp-symmetry` and `--joint-limits ur5e`;
`set-grasps scene.json --set faces -o DIR` gives an existing scene face grasps.
Scene names give the scenario, the object count and, for generated scenes,
the generator seed (`-s0`).

Then check the scene, solve it and watch the plan:

```bash
python3 viewer/serve.py    # scene viewer: http://127.0.0.1:8765/viewer/scene.html, pick mine/my-ring.json
./build/lazydash_solve resources/scenes/mine/my-ring.json --out results/my-ring.json
                           # result viewer: http://127.0.0.1:8765/viewer/result.html, pick my-ring.json
```

`resources/scenes/mine/` and `results/` are ignored by git. The viewers list
these folders, and the result viewer shows a scene's goals and scenery when
the scene is under `resources/scenes/`.

The scene viewer renders the scene, edits it live and lists problems (an
object inside an obstacle, out of reach or out of bounds); `build/ht_diag
scene.json` reports reach and IK per object pose and robot.

## Repository layout

| Folder | What |
|---|---|
| `src/lazydash/` | the planner: `task` (layers 1-2), `motion` (layer 3), `schedule` (layer 4), `planning` (the main loop, result JSON), `scene`, `robot`, `hypergraph` |
| `apps/` | `lazydash_solve`; `ht_diag` (why a scene lacks transitions) |
| `tests/` | Catch2 tests |
| `runner/` | benchmark runs and summaries; `data/` the paper's runs |
| `viewer/` | browser viewers, video rendering, the results page (`viewer/README.md`) |
| `python/lazydash/` | scene files and generators; the Python module |
| `resources/` | the robot model and the scenes (`paper/`, `extra/`, `generated/` for the tests) |
| `external/` | CoMotion setup: the pinned commit (`COMOTION_PIN`), this repo's patches and `bootstrap_comotion.sh`, which clones CoMotion here |

## Citation

```bibtex
@article{lee2025lazydash,
  title={Lazy-DaSH: Lazy Approach for Hypergraph-based Multi-robot Task and Motion Planning},
  author={Lee, Seongwon and Motes, James D. and Ngui, Isaac and Morales, Marco and Amato, Nancy M.},
  journal={arXiv preprint arXiv:2504.05552},
  year={2025}
}
```

Please also cite CoMotion as its README asks.
