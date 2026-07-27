#!/usr/bin/env bash
# =============================================================================
# run_ns3learn.sh — run the NS-3-distilled OMNeT treatment (Prop_NS3Learn_*) at
# the 6 adoption rates x REPS seeds, mirroring run_omnet_resume.sh. Idempotent:
# skips runs that already produced a dist_pdr_*.csv marker.
#
# PREREQUISITE: veins_inet must be rebuilt after the realismModel/ns3learn code
# change. In your opp_env shell (workspace = omnetpp-6.0.3), or via opp_env run:
#   opp_env run omnetpp-6.0.3 -c \
#     'cd ${VEINS_ROOT}/subprojects/veins_inet && make -j4 MODE=release'
# Also requires the runtime bundle results/teacher_dataset/ns3learn_runtime/ (export_ns3learn_runtime.py).
# =============================================================================
set -uo pipefail

# Paths (OMNETPP_ROOT, VEINS_ROOT, SUMO_HOME, PROJ, ...) come from env.sh.
RR_ROOT="${RR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=../env.sh
. "$RR_ROOT/env.sh"
INI="omnetpp_inet.ini"
read -r -a RATES <<< "${RATES:-01 05 25 50 75 100}"
REPS="${REPS:-5}"; LAST=$((REPS - 1))
cd "$PROJ"
echo "=== Prop_NS3Learn run start $(date) (REPS=$REPS) ==="
ran=0; skipped=0; failed=0
for r in "${RATES[@]}"; do
  CFG="Prop_NS3Learn_CAV${r}"
  for s in $(seq 0 "$LAST"); do
    OUTDIR="$PROJ/results/inet/${CFG}-${s}"
    if ls "$OUTDIR"/dist_pdr_*.csv >/dev/null 2>&1; then skipped=$((skipped+1)); continue; fi
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
    if ls "$OUTDIR"/dist_pdr_*.csv >/dev/null 2>&1; then echo "  OK ${CFG}-${s} (rc=$RC)"; ran=$((ran+1))
    else echo "  FAIL ${CFG}-${s} (rc=$RC) — see $LOG"; failed=$((failed+1)); fi
  done
done
echo "=== done $(date): ran=$ran skipped=$skipped failed=$failed ==="
