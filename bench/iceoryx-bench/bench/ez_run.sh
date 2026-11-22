#!/usr/bin/env bash
# Run the ezmsg matrix; appends one JSON line per cell to bench/ez_results.jsonl.
# Run from the project root as: bash bench/ez_run.sh   (uses the `ezmsg` pixi environment, Python 3.12)
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export EZ_OUT="$PWD/bench/.ez_cell.json"
for mode in "bytes two" "numpy two" "numpy same"; do
  set -- $mode
  for c in "1000 100" "64000 100" "1000000 30" "2000000 30" "8000000 30" "2000000 135"; do
    set -- $1 $2 $c
    : > "$EZ_OUT"
    timeout 60 pixi run -e ezmsg python bench/ez_pubsub.py "$1" "$3" "$4" 8 "$2" > /dev/null 2>&1
    echo "{\"transport\":\"ezmsg\",\"payload\":\"$1\",\"procs\":\"$2\",\"bytes\":$3,\"hz\":$4,\"cell\":$(cat "$EZ_OUT" | grep . || echo null)}" | tee -a bench/ez_results.jsonl
  done
done
