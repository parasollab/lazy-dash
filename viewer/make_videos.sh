#!/usr/bin/env bash
# Solve one representative scene per scenario (seed 1) and render its video.
#
#   viewer/make_videos.sh [OUT]     # OUT defaults to results/videos
# RENDER_PYTHON: a Python with the renderer's packages (viewer/README.md).
set -u
cd "$(dirname "$0")/.."
D=${1:-results/videos}
mkdir -p "$D"
rm -f "$D/solve.log" "$D/render.log" "$D/done"
BUILD=${BUILD:-build}
run() { ./$BUILD/lazydash_solve resources/scenes/paper/$1.json --seed 1 --time-limit 1800 --stride 4 --out $D/$1.json > $D/$1.stats.json 2> $D/$1.err; echo "$1 rc=$?" >> $D/solve.log; }
S="sorting-4robots-16objects sorting-9robots-16objects sorting-16robots-16objects \
   wall-2walls-16objects wall-4walls-16objects wall-6walls-16objects \
   shelf-wall-2robots-16objects shelf-wall-4robots-16objects \
   stocking-2layers-12objects stocking-3layers-9objects"
for s in $S; do run $s & done
wait
export PYOPENGL_PLATFORM=egl
R() { "${RENDER_PYTHON:-python3}" viewer/render_video.py resources/scenes/paper/$1.json $D/$1.json --out $D/$2.mp4 --title "$3" "${@:4}" >> $D/render.log 2>&1; }
R sorting-4robots-16objects sorting-4robots "Sorting: 4 robots" &
R sorting-9robots-16objects sorting-9robots "Sorting: 9 robots" &
R sorting-16robots-16objects sorting-16robots "Sorting: 16 robots" &
R wall-2walls-16objects wall-2walls "Wall: 8 robots / 2 walls" &
R wall-4walls-16objects wall-4walls "Wall: 8 robots / 4 walls" &
wait
R wall-6walls-16objects wall-6walls "Wall: 8 robots / 6 walls" &
R shelf-wall-2robots-16objects shelf-wall-2robots "Shelf-wall: 2 robots / 2 shelves" --view=1.1,-1.0,0.8 --zoom 1.7 &
R shelf-wall-4robots-16objects shelf-wall-4robots "Shelf-wall: 4 robots / 4 shelves" --view=1.1,-1.0,0.8 --zoom 1.4 &
R stocking-2layers-12objects stocking-2layers "Stocking: 2 robots / 2 layers" --view=-1.3,-0.45,0.75 --zoom 2.0 &
R stocking-3layers-9objects stocking-3layers "Stocking: 2 robots / 3 layers" --view=-1.3,-0.45,0.75 --zoom 2.0 &
wait
echo DONE > "$D/done"
