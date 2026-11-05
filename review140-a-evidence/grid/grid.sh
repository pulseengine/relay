#!/bin/bash
# Run the filter-rate oracle grid against one source tree.
#
#   TD=<cargo target dir> [GRID_PERTURB=1] ./grid.sh <relay-tree-root> <tag>
#
# Writes grid_<tag>.txt (one CELL line per cell, sorted) and grid_<tag>.log
# next to this script. Copies zg_grid.rs into the tree's falcon-core tests
# for the run and removes it afterwards.
#
# USE A SEPARATE CARGO_TARGET_DIR PER TREE. Cargo's freshness check is
# mtime-based; a tree extracted with `git archive` has old mtimes, and with a
# shared target dir cargo silently reuses the other tree's build (the test
# binary hash does not include the source path). This produced a "parent"
# run identical to the branch once.
set -u
B="$(cd "$(dirname "$0")" && pwd)"
cp "$B/zg_grid.rs" "$1/crates/falcon-core/tests/zg_grid.rs"
cd "$1"
start=$(date +%s)
CARGO_TARGET_DIR="$TD" GRID_OUT="$B/grid_$2.txt" \
  cargo test --release -p falcon-core --test zg_grid > "$B/grid_$2.log" 2>&1
rc=$?
rm "$1/crates/falcon-core/tests/zg_grid.rs"
echo "$2 rc=$rc $(( $(date +%s) - start )) s, $(wc -l < "$B/grid_$2.txt") cells"
grep -E "Compiling falcon-core|test result" "$B/grid_$2.log"
