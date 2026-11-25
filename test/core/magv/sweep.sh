#!/bin/sh
# sweep.sh OUT WASM KIND CLASSES SEEDLO SEEDHI MODE INTEGRATION : one process per board, hard limits (SIGKILL wall 60 s, CPU 40 s).
OUT=$1; WASM=$2; KIND=$3; CLASSES=$4; LO=$5; HI=$6; MODE=$7; INTEG=${8:-be}
W2=/Users/dev/Documents/DEVOLTProject/devolt/memory/progress/brownout-core-w1e-drafts/ship-logs/v1-hotfix-coal-gate/w2.sh
cd /Users/dev/Documents/DEVOLTProject/brownout/.claude/worktrees/p0-magv || exit 1
: > "$OUT"
for cls in $CLASSES; do
  s=$LO
  while [ $s -le $HI ]; do
    line=$($W2 60 40 node --import tsx test/core/magv/run-board.ts --wasm "$WASM" --kind $KIND --cls $cls --seed $s --mode $MODE --integration $INTEG 2>/dev/null | tail -1)
    rc=$?
    case "$line" in "{"*) ;; *) line="{\"status\":\"NORETURN\"}";; esac
    echo "$KIND $cls $s $MODE $INTEG $line" >> "$OUT"
    s=$((s+1))
  done
done
