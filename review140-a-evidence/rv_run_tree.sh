#!/bin/bash
# usage: rv_run_tree.sh <tree> <targetdir> <stem> <testfn> <tag> [perturb]
S=/private/tmp/claude-501/-Users-r-git-pulseengine-relay/f207d8bc-6a92-4057-9e33-a06782c12f0c/scratchpad
tree=$1; TD=$2; T=$3; F=$4; tag=$5
L=$tree/crates/falcon-core/plain/src/lib.rs
grep -q "mod $T " $L || printf '\n#[cfg(test)]\nmod %s { include!("%s/%s.rs"); }\n' $T $S $T >> $L
touch $L
cd $tree
if [ -n "$6" ]; then export RV_PERTURB=1; fi
{ time CARGO_TARGET_DIR=$TD RV_OUT=$S/${T}_$tag.txt cargo test --release -p falcon-core --lib $F ; } > $S/${T}_$tag.log 2>&1
echo "$tag: $(grep -E 'test result|^error|^real' $S/${T}_$tag.log | tr '\n' ' ') $(wc -l < $S/${T}_$tag.txt 2>/dev/null)"
