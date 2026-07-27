#!/usr/bin/env bash
# =============================================================================
# run_omnet_resume.sh — idempotent resume of the OMNeT propagation sweep.
# Runs every (treatment x rate x seed) that lacks a dist_pdr_*.csv success
# marker; skips the rest. Each seed is its own opp_run (the -r 0..N form stops
# after run 0 with the Veins/launchd lifecycle). Safe to re-run any time.
# =============================================================================
set -uo pipefail

# Paths (OMNETPP_ROOT, VEINS_ROOT, SUMO_HOME, PROJ, ...) come from env.sh.
RR_ROOT="${RR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=../env.sh
. "$RR_ROOT/env.sh"
INI="omnetpp_inet.ini"
TREATMENTS=(Plain Analytical Calibrated Generalized)
RATES=(01 05 25 50 75 100)
REPS="${REPS:-5}"; LAST=$((REPS - 1))

cd "$PROJ"
echo "=== OMNeT resume start $(date) (REPS=$REPS) ==="
ran=0; skipped=0; failed=0
for t in "${TREATMENTS[@]}"; do for r in "${RATES[@]}"; do
  CFG="Prop_${t}_CAV${r}"
  for s in $(seq 0 "$LAST"); do
    OUTDIR="$PROJ/results/inet/${CFG}-${s}"
    if ls "$OUTDIR"/dist_pdr_*.csv >/dev/null 2>&1; then
      skipped=$((skipped+1)); continue
    fi
    mkdir -p "$OUTDIR"; LOG="$OUTDIR/${CFG}-${s}.out"
    echo "RUN ${CFG}-${s}"
    /bin/zsh -lc "
      cd '$PROJ' &&
      export OPP_ENV_VERSION=1 &&
      export OMNETPP_ROOT='$OMNETPP_ROOT' &&
      export SUMO_HOME='$SUMO_HOME' &&
      export PATH='$SUMO_HOME/bin':\$PATH &&
      source '$OMNETPP_ROOT/setenv' -q &&
      '$OPP_RUN' -M release -- -r $s -m -u Cmdenv -c '$CFG' --debug-on-errors=false '$INI'
    " >"$LOG" 2>&1
    RC=$?
    if ls "$OUTDIR"/dist_pdr_*.csv >/dev/null 2>&1; then
      echo "  OK ${CFG}-${s} (rc=$RC)"; ran=$((ran+1))
    else
      echo "  FAIL ${CFG}-${s} (rc=$RC) — see $LOG"; failed=$((failed+1))
    fi
  done
done; done
echo "=== OMNeT resume done $(date): ran=$ran skipped=$skipped failed=$failed ==="
