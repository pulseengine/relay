#!/bin/bash
# usage: run2.sh <stem> <testfn> <tagsuffix> ; env RV_* passed through
S=/private/tmp/claude-501/-Users-r-git-pulseengine-relay/f207d8bc-6a92-4057-9e33-a06782c12f0c/scratchpad
$S/rv_run_tree.sh $S/lockA-main $S/td-main $1 $2 main$3 $4 && $S/rv_run_tree.sh $S/lockA-proto $S/td-proto $1 $2 guard$3 $4
