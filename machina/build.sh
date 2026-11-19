#!/usr/bin/env bash
# Build machina's INAV fork in rootless Podman, from this tree.
#
#   machina/build.sh sitl            the SITL binary: $OUT/inav-sitl (and inav-sitl-<git short hash>)
#   machina/build.sh unit [NAME...]  INAV's own unit tests (src/test/unit), built and run by ctest
#
# OUT is $MACHINA_CACHE/inav-fork (MACHINA_CACHE defaults to ~/.cache/machina). The source tree is
# mounted read-only; build trees and binaries stay under OUT, never in git. Every container runs
# with --memory 6g --cpus 4 --oom-score-adj 1000 and a hard time limit (BUILD_TIMEOUT, seconds).
set -euo pipefail
echo 1000 > /proc/self/oom_score_adj 2>/dev/null || true
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(cd "$HERE/.." && pwd)
OUT=${MACHINA_CACHE:-$HOME/.cache/machina}/inav-fork
IMAGE=localhost/machina-inav-build:24.04
JOBS=${JOBS:-4}
LIMIT=${BUILD_TIMEOUT:-1800}
NAME=machina-inav-build-$$
mkdir -p "$OUT"
trap 'podman rm -f "$NAME" >/dev/null 2>&1 || true' EXIT

run() {
  timeout --kill-after=30 "$((LIMIT + 60))" podman run --rm --name "$NAME" --timeout "$LIMIT" \
    --memory 6g --cpus 4 --oom-score-adj 1000 \
    -v "$SRC":/src:ro -v "$OUT":/out:rw "$IMAGE" sh -c "$1"
}

timeout 900 podman build -q -t "$IMAGE" -f "$HERE/Containerfile" "$HERE" >/dev/null
REV=$(git -C "$SRC" rev-parse --short=9 HEAD)
git -C "$SRC" diff --quiet HEAD -- src || REV=$REV+dirty
case "${1:-sitl}" in
sitl)
  run "cmake -S /src -B /out/build-sitl -DSITL=ON -GNinja >/out/build-sitl.log 2>&1 \
       && ninja -C /out/build-sitl -j$JOBS >>/out/build-sitl.log 2>&1 \
       || { tail -40 /out/build-sitl.log; exit 1; }"
  BIN=$(find "$OUT/build-sitl" -maxdepth 2 -name '*_SITL' -type f | head -1)
  cp "$BIN" "$OUT/inav-sitl-$REV"
  ln -sfn "inav-sitl-$REV" "$OUT/inav-sitl"
  echo "$OUT/inav-sitl-$REV sha256 $(sha256sum "$OUT/inav-sitl-$REV" | cut -c1-16)"
  ;;
unit)
  shift
  FILTER=""
  [ $# -gt 0 ] && FILTER="-R '$(echo "$*" | tr ' ' '|')'"
  run "cmake -S /src -B /out/build-unit -DTOOLCHAIN= -GNinja >/out/build-unit.log 2>&1 \
       && ninja -C /out/build-unit -j$JOBS >>/out/build-unit.log 2>&1 \
       && cd /out/build-unit/src/test && ctest --output-on-failure $FILTER >/out/ctest.log 2>&1; s=\$?; \
       tail -25 /out/ctest.log 2>/dev/null || tail -40 /out/build-unit.log; exit \$s"
  ;;
*)
  echo "usage: $0 sitl | unit [NAME...]" >&2
  exit 2
  ;;
esac
