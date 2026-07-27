#!/bin/zsh
# Crash-resilient network-side sweep: both models with CAM-CACC across adoption.
# Each treatment = 5 rates x 5 seeds = 25 runs; restart the runner if it dies.

# Paths (OMNETPP_ROOT, VEINS_ROOT, SUMO_HOME, PROJ, ...) come from env.sh.
RR_ROOT="${RR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=../env.sh
. "$RR_ROOT/env.sh"
cd "$PROJ"
MAXP=6; MIN_FREE_GB=6; REPS=5; RATES="05 25 50 75 100"

done_count() {  # $1 = treatment (e.g. Analytical_M3_Net)
  local n=0 r s
  for r in 05 25 50 75 100; do for s in 0 1 2 3 4; do
    ls results/inet/Prop_$1_CAV${r}-${s}/dist_pdr_*.csv >/dev/null 2>&1 && n=$((n+1))
  done; done
  echo $n
}

supervise() {  # $1 = treatment
  echo "[net-sup] $(date) phase: $1 (target 25)"
  while [ "$(done_count $1)" -lt 25 ]; do
    if ! pgrep -f "run_omnet_resume_par" >/dev/null 2>&1; then
      echo "[net-sup] $(date) (re)launching $1 ($(done_count $1)/25 done) MAXP=$MAXP"
      TREATMENTS="$1" RATES="$RATES" MAXP=$MAXP MIN_FREE_GB=$MIN_FREE_GB REPS=$REPS \
        ./run_omnet_resume_par.sh >> results/inet/_netsweep_$1.log 2>&1
    fi
    sleep 30
  done
  echo "[net-sup] $(date) $1 COMPLETE"
}

supervise Analytical_M3_Net
supervise NS3Learn_Net
echo "[net-sup] $(date) NETWORK-SIDE SWEEP COMPLETE (both models)."
