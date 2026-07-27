#!/usr/bin/env bash
# =============================================================================
# run_omnet_resume_par.sh — PARALLEL idempotent resume of the OMNeT propagation
# sweep. Same skip logic as run_omnet_resume.sh (a (config,seed) with a
# dist_pdr_*.csv marker is skipped) but runs several opp_run processes at once.
# veins_launchd hands each concurrent connection its own SUMO on a free port.
# Self-regulating: won't launch a new run while free RAM < MIN_FREE_GB, so it
# stays small alongside the NS-3 sweep and fills cores once that frees RAM.
# =============================================================================
set -uo pipefail

# Paths (OMNETPP_ROOT, VEINS_ROOT, SUMO_HOME, PROJ, ...) come from env.sh.
RR_ROOT="${RR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=../env.sh
. "$RR_ROOT/env.sh"
INI="omnetpp_inet.ini"
# Default headline set = the two study models: Combined Analytical Reference (Cao+Rehman)
# and the NS-3-distilled surrogate. Ablations: TREATMENTS="Analytical_Col Analytical_Prop".
read -r -a TREATMENTS <<< "${TREATMENTS:-Analytical_M3 NS3Learn}"
read -r -a RATES <<< "${RATES:-01 05 25 50 75 100}"
REPS="${REPS:-5}"; LAST=$((REPS - 1))
MAXP="${MAXP:-6}"
MIN_FREE_GB="${MIN_FREE_GB:-6}"
STAGGER="${STAGGER:-5}"   # seconds between launches; OMNeT runs ramp RAM slowly, so use a longer value when sharing the box

cd "$PROJ"

free_gb() {
  # Portable free-memory probe: /proc/meminfo on Linux, vm_stat on macOS.
  if [ -r /proc/meminfo ]; then
    awk '/^MemAvailable:/ {printf "%d", $2/1048576; exit}' /proc/meminfo
    return
  fi
  local ps free inact
  ps=$(vm_stat | sed -n '1s/.*page size of \([0-9]*\) bytes.*/\1/p')
  free=$(vm_stat | awk '/Pages free/{gsub(/[^0-9]/,"",$3);print $3}')
  inact=$(vm_stat | awk '/Pages inactive/{gsub(/[^0-9]/,"",$3);print $3}')
  echo $(( (free + inact) * ps / 1073741824 ))
}

run_one() {
  local CFG="$1" s="$2" OUTDIR="$PROJ/results/inet/$1-$2" LOG
  mkdir -p "$OUTDIR"; LOG="$OUTDIR/${CFG}-${s}.out"
  /bin/zsh -lc "
    cd '$PROJ' &&
    export OPP_ENV_VERSION=1 &&
    export OMNETPP_ROOT='$OMNETPP_ROOT' &&
    export SUMO_HOME='$SUMO_HOME' &&
    export PATH='$SUMO_HOME/bin':\$PATH &&
    source '$OMNETPP_ROOT/setenv' -q &&
    '$OPP_RUN' -M release -- -r $s -m -u Cmdenv -c '$CFG' --debug-on-errors=false '$INI'
  " >"$LOG" 2>&1
  if ls "$OUTDIR"/dist_pdr_*.csv >/dev/null 2>&1; then echo "  OK ${CFG}-${s}"; else echo "  FAIL ${CFG}-${s} (see $LOG)"; fi
}

echo "=== OMNeT PARALLEL resume start $(date) (MAXP=$MAXP MIN_FREE_GB=$MIN_FREE_GB REPS=$REPS) ==="
for t in "${TREATMENTS[@]}"; do for r in "${RATES[@]}"; do
  CFG="Prop_${t}_CAV${r}"
  for s in $(seq 0 "$LAST"); do
    OUTDIR="$PROJ/results/inet/${CFG}-${s}"
    ls "$OUTDIR"/dist_pdr_*.csv >/dev/null 2>&1 && continue           # already done
    pgrep -f -- "-r ${s} -m -u Cmdenv -c ${CFG} " >/dev/null 2>&1 && { echo "  skip ${CFG}-${s} (already running)"; continue; }
    while :; do
      running=$(jobs -rp | wc -l | tr -d ' '); fg=$(free_gb)
      [ "$running" -lt "$MAXP" ] && [ "$fg" -ge "$MIN_FREE_GB" ] && break
      sleep 5
    done
    echo "  launch ${CFG}-${s} [running=$running freeGB=$(free_gb)]"
    run_one "$CFG" "$s" &
    sleep "$STAGGER"
  done
done; done
wait
echo "=== OMNeT PARALLEL resume done $(date) ==="
