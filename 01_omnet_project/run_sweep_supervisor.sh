#!/bin/zsh
# Supervisor for the two-model intersection sweep: crash-resilient + sequential.
#  - Phase 1: ensure all 30 Analytical_M3 runs land (restart the runner if it dies).
#  - Phase 2: ensure all 30 NS3Learn runs land.
# Runs are SUMO-lockstep-bound (~55% CPU each), so MAXP=8 fills the 12 cores without
# CPU saturation; MIN_FREE_GB=6 still guards RAM (runner self-limits if memory is tight).

# Paths (OMNETPP_ROOT, VEINS_ROOT, SUMO_HOME, PROJ, ...) come from env.sh.
RR_ROOT="${RR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=../env.sh
. "$RR_ROOT/env.sh"
cd "$PROJ"
MAXP=8; MIN_FREE_GB=6; REPS=5

done_count() {  # $1 = model
  local n=0
  for d in results/inet/Prop_$1_CAV*-*/; do
    ls "$d"dist_pdr_*.csv >/dev/null 2>&1 && n=$((n+1))
  done
  echo $n
}

supervise() {   # $1 = model, $2 = target count, $3 = logfile
  echo "[sup] $(date) phase: $1 (target $2)"
  while [ "$(done_count $1)" -lt "$2" ]; do
    if ! pgrep -f "run_omnet_resume_par" >/dev/null 2>&1; then
      echo "[sup] $(date) no runner alive for $1 ($(done_count $1)/$2 done) -> (re)launching MAXP=$MAXP"
      TREATMENTS="$1" MAXP=$MAXP MIN_FREE_GB=$MIN_FREE_GB REPS=$REPS ./run_omnet_resume_par.sh >> "$3" 2>&1
    fi
    sleep 30
  done
  echo "[sup] $(date) $1 COMPLETE ($(done_count $1)/$2)"
}

supervise Analytical_M3 30 results/inet/_sweep_m3.log
supervise NS3Learn      30 results/inet/_sweep_ns3learn.log
echo "[sup] $(date) BOTH MODELS COMPLETE. Ready for build_deviation_analysis.py"
