#!/usr/bin/env bash
# Build, test, and solve a benchmark suite. Linux build host only.
#
#   runner/run_eval.sh paper [OUT]   # the paper's benchmark (OUT defaults to results/paper)
#   runner/run_eval.sh extra [OUT]   # the extra benchmark   (OUT defaults to results/extra)
set -u
cd "$(dirname "$0")/.."
SUITE=${1:?usage: runner/run_eval.sh paper|extra [OUT]}
OUT=${2:-results/$SUITE}
JOBS=${JOBS:-$(nproc)}
BUILD=${BUILD:-build}
mkdir -p "$OUT"
rm -f "$OUT/done"
nice cmake --build "$BUILD" -j"$(nproc)" > "$OUT/build.log" 2>&1 || { echo BUILD-FAILED > "$OUT/done"; exit 1; }
"./$BUILD/tests/lazydash_tests" > "$OUT/tests.txt" 2>&1 || { echo TESTS-FAILED > "$OUT/done"; exit 1; }
run() { python3 runner/run_trials.py --scenes "$@" --jobs "$JOBS" --time-limit 1800 --solver "$BUILD/lazydash_solve" \
          --out "$OUT/trials.jsonl"; }
case "$SUITE" in
paper)
  P=resources/scenes/paper
  run $P/shelf-wall-*.json $P/stocking-*.json --seeds 1-10
  run $(for n in 4 9 16; do for k in 4 16 40; do echo $P/sorting-${n}robots-${k}objects.json; done; done
        for w in 2 4 6; do for k in 4 16 40; do echo $P/wall-${w}walls-${k}objects.json; done; done) --seeds 1-50
  python3 runner/compare_paper.py "$OUT/trials.jsonl" --out "$OUT/comparison" > "$OUT/comparison.txt" 2>&1
  python3 runner/aggregate_results.py lazydash="$OUT/trials.jsonl" --out "$OUT/results.json"
  ;;
extra)
  run resources/scenes/extra/*/*.json --seeds 1-10
  ;;
*)
  echo "unknown suite $SUITE (paper or extra)" >&2; exit 2 ;;
esac
python3 runner/summarize.py "$OUT/trials.jsonl" > "$OUT/summary.md"
echo EVAL-DONE > "$OUT/done"
