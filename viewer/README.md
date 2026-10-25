# Lazy-DaSH viewers

Everything that shows scenes, solves and evaluations:

| File | What |
|---|---|
| `index.html`, `scene.*`, `result.*`, `results.*`, `common/` | three browser tools (below) |
| `serve.py` | local server for the browser tools |
| `render_video.py` | a solve as an mp4 (offscreen, Linux host) |
| `make_videos.sh` | one solve and video per scenario (seed 1) |
| `report/index.html`, `make_report.py` | the results page: per scenario its video, arm timeline and the results against the paper |

The browser tools are static pages, no build step; three.js 0.160.1 and
urdf-loader 0.12.6 load from cdn.jsdelivr.net (pinned in each page's import
map), so the browser needs network access.

## Launch

```bash
python viewer/serve.py            # serves the repository at http://127.0.0.1:8765/viewer/ and opens it
python viewer/serve.py --port 9000 --no-browser
```

The launcher (standard library only) serves the repository root, so the
pages read `resources/` (URDF, meshes, scenes) by relative URL. It also lists
repository files in each tool and lets the scene viewer save in place. Any
other static server rooted at the repository works too; then files open only
with the Open button or by drag and drop, and Save downloads.

## Tools

**Scene viewer** (`scene.html`). Renders a scene: arms at their start
configurations (meshes, or the spherized collision model), blocks at their
starts, goal alternatives as dashed outlines, placement regions, obstacles
(the planner's spheres and cylinders, or the scenery boxes they approximate),
environment bounds. The JSON editor re-renders on every valid edit and names
the line and column of a syntax error. Watch file reloads the scene when you
save it in your own editor (files opened from the repository). Save writes
back under `resources/scenes/` after asking for the path (launcher only),
otherwise downloads. Checks, clickable to show the item in the view and the
editor:
- schema as `Scene::parse` reads it (format, required fields, upright bases,
  6 joint values, unique names);
- a block start or goal inside an obstacle, or its collision sphere reaching
  into one;
- overlapping blocks (`collision_radius`): starts, and goals of different
  objects;
- starts and goals out of every arm's reach (`params.robot_range`, default
  1.5 m, base to object distance as in `Capabilities::reaches`);
- arm bases closer than 0.15 m; start joints outside the joint limits;
  anything outside the environment bounds.
These are point and sphere tests. For IK and capability per pose run
`build/ht_diag <scene>`.

**Result viewer** (`result.html`). Plays back what
`lazydash_solve <scene> --out result.json` writes: joint paths and block
positions, interpolated between the strided waypoints, on a time slider (play,
speed, previous and next event; space, arrows, `,` and `.` on the keyboard).
The arm timeline under the view (moves, carrying, grasp and release stages,
handover links) follows the playhead; click or drag it to seek. The side panel
has the makespan, planning time per layer, constraints and representation
sizes. Click an arm or a timeline row to trace its gripper. The result stores
block positions only: a carried block keeps its pose relative to the hand from
the grasp on. When the scene file is in `resources/scenes/`, its goals, start
orientations and scenery boxes are shown too.

**Results browser** (`results.html`). Evaluation curves per scene family
against the paper's runs. Its input, `results/paper/results.json` (the
page's default), is written by `runner/run_eval.sh`, or for any trials file by

```bash
python3 runner/aggregate_results.py lazydash=results/paper/trials.jsonl
```

(`LABEL=PATH` per trials file, one series each; `--out` for another path). Views: planning time by stage (Fig. 8:
task dashed and motion solid, conflict resolution, total), representation
sizes (Fig. 7: H_T and H_M vertices and hyperarcs), success rate, makespan in
the paper's timesteps, and a table. Medians over solved runs with
interquartile bands, log y shared across a row, as in the paper's figures.
Series and scene families can be toggled.

## Files the tools accept

| Tool | File | Written by |
|---|---|---|
| Scene viewer | scene JSON, format `lazydash.scene/1` | `python/lazydash/scene.py` |
| Result viewer | result JSON (`robots`, `obstacles`, `lazydash.timeline`) | `lazydash_solve --out` |
| Results browser | `lazydash.results/1` JSON | `runner/aggregate_results.py` |

Each page also takes `?file=<repository path>`, e.g.
`scene.html?file=resources/scenes/generated/sorting-4robots-4objects-s0.json`.

## Videos and the results page

On a Linux host with EGL and ffmpeg, and a Python with `numpy`, `pyrender`,
`trimesh`, `pycollada`, `Pillow` and `PyOpenGL` (`RENDER_PYTHON` points
`make_videos.sh` to it):

```bash
viewer/make_videos.sh                                  # results/videos/: solves and mp4s
python3 viewer/make_report.py --trials results/paper/trials.jsonl \
    --scenes results/videos --out results/report        # index.html, data.json, videos/
```

`results/report/` is self-contained: open `index.html` from a static server,
or publish the folder.
