#!/bin/sh
# Differential test: is the ported flood fill the same algorithm?
#
# Builds MicroMouseAlgorithm's maze.c + Main.c into one binary and the copies
# under Core/{Inc,Src}/Maze/floodfill into another, links BOTH against the same
# simulated maze in floodfill_sim_api.c, and compares their transcripts action
# for action. Everything about the two builds is identical except which copy of
# the algorithm they contain, so any difference is one the port introduced.
#
# Also runs each maze twice, once serving wall readings fresh and once from a
# per-cell snapshot, because that snapshot is the one adaptation the robot
# needed: a real sensor read costs about 200 ms and the algorithm asks several
# times per cell. If caching ever changes a decision it is not just saving time.
#
# Usage: tests/floodfill_diff.sh [path-to-MicroMouseAlgorithm]
# Exit code 0 = the port is faithful.
set -e

ALGO=${1:-../../MicroMouseAlgorithm}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

if [ ! -f "$ALGO/maze.c" ]; then
    echo "cannot find the original at $ALGO -- pass its path as an argument"
    exit 2
fi

gcc -O1 -o "$OUT/orig" "$ALGO/maze.c" "$ALGO/Main.c" \
    tests/floodfill_sim_api.c -I "$ALGO"

gcc -O1 -DSIM_PROVIDE_DEBUG_LOG -o "$OUT/port" \
    Core/Src/Maze/floodfill/maze.c Core/Src/Maze/floodfill/floodfill_run.c \
    tests/floodfill_sim_api.c tests/floodfill_port_main.c \
    -I Core/Inc/Maze/floodfill

fail=0

for seed in 1 2 3 7 42 1234 31337; do
    for cached in 0 1; do
        FF_SEED=$seed FF_CACHED=$cached "$OUT/orig" 2>/dev/null > "$OUT/o"
        FF_SEED=$seed FF_CACHED=$cached "$OUT/port" 2>/dev/null > "$OUT/p"

        n=$(wc -l < "$OUT/o")

        if cmp -s "$OUT/o" "$OUT/p"; then
            echo "  [PASS] seed $seed cached=$cached  $n actions, identical"
        else
            echo "  [FAIL] seed $seed cached=$cached  DIVERGED"
            diff "$OUT/o" "$OUT/p" | head -10
            fail=1
        fi
    done
done

if [ $fail -eq 0 ]; then
    echo "===== PORT IS FAITHFUL ====="
else
    echo "===== PORT HAS DIVERGED ====="
fi

exit $fail
