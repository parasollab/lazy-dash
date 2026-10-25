#!/usr/bin/env bash
# Clone CoMotion at the pinned commit into external/comotion with its dependencies. Linux only.
#
#   external/bootstrap_comotion.sh            # clone/checkout + bootstrap deps
#   COMOTION_BUILD=1 external/bootstrap_comotion.sh   # also build CoMotion standalone (apps on)
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
PIN=$(tr -d '[:space:]' < "$ROOT/external/COMOTION_PIN")
DIR=${COMOTION_DIR:-$ROOT/external/comotion}
URL=${COMOTION_URL:-https://github.com/parasollab/comotion.git}

if [ ! -d "$DIR/.git" ]; then
  git clone --quiet "$URL" "$DIR"
fi
# Start from the clean pin every time; the patches are applied again below.
git -C "$DIR" checkout --quiet -- .
if [ "$(git -C "$DIR" rev-parse HEAD)" != "$PIN" ]; then
  git -C "$DIR" fetch --quiet origin
  git -C "$DIR" checkout --quiet "$PIN"
fi
# Newer CoMotion revisions ship a bootstrap script; the public pin uses plain submodules.
if [ -f "$DIR/scripts/bootstrap_dependencies.py" ]; then
  (cd "$DIR" && python3 scripts/bootstrap_dependencies.py)
else
  git -C "$DIR" submodule update --init --recursive --quiet
fi
# Apply lazydash's patches to CoMotion in order.
for patch in "$ROOT"/external/patches/*.patch; do
  [ -e "$patch" ] || continue
  git -C "$DIR" apply "$patch"
  echo "applied: $(basename "$patch")"
done
echo "comotion at $(git -C "$DIR" rev-parse --short HEAD) (+ patches) in $DIR"

if [ "${COMOTION_BUILD:-0}" = "1" ]; then
  cmake -S "$DIR" -B "$DIR/build" -DCMAKE_BUILD_TYPE=Release \
    -DCOMOTION_BUILD_APPS=ON -DCOMOTION_BUILD_EXAMPLES=ON -DCOMOTION_BUILD_TESTS=OFF
  cmake --build "$DIR/build" -j"${JOBS:-16}"
fi
