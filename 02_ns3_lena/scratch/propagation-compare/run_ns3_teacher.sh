#!/usr/bin/env bash
# =============================================================================
# run_ns3_teacher.sh — collect NS-3 teacher data for the NS-3->OMNeT distillation.
# Defaults to --macSummary (TX scheduling: collisions + HARQ; cheap, all rates).
# Set PHYRX=1 to also enable --phyRxTrace (RX decode/SINR/latency; HUGE — low rate only).
# Writes to results/teacher/ + results/teacher_logs/ so the main sweep DBs are untouched.
# Resumable (per-tag .result), RAM-guarded, light->heavy.
#   RATES="10 40 90" SCENARIO=V2V_Urban ./run_ns3_teacher.sh
#   RATES="01 05 10" PHYRX=1 ./run_ns3_teacher.sh        # low-rate RX traces
# =============================================================================
set -uo pipefail

# Paths come from env.sh. setup.sh writes .rr_root here when it installs this
# directory into ns-3; otherwise export RR_ROOT before running.
if [ -z "${RR_ROOT:-}" ]; then
    _d="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    [ -f "$_d/.rr_root" ] && RR_ROOT="$(cat "$_d/.rr_root")"
    unset _d
fi
: "${RR_ROOT:?set RR_ROOT to the RadioRealismNS3_Learn directory}"
# shellcheck source=/dev/null
. "$RR_ROOT/env.sh"
# NS3_SCRATCH comes from env.sh (this directory, once installed into ns-3).
INPUTS="$NS3_SCRATCH/inputs"
RESULTS="$NS3_SCRATCH/results/teacher"
LOGDIR="$NS3_SCRATCH/results/teacher_logs"
BIN="$NS3_ROOT/build/scratch/propagation-compare/ns3.42-propagation-compare-default"
SCENARIO="${SCENARIO:-V2V_Urban}"
SEEDS="${SEEDS:-0 1 2 3 4}"
RATES="${RATES:-10 40 90}"
PHYRX="${PHYRX:-0}"
MAXP="${MAXP:-6}"
MIN_FREE_GB="${MIN_FREE_GB:-6}"
export DYLD_LIBRARY_PATH="$NS3_ROOT/build/lib:${DYLD_LIBRARY_PATH:-}"
# Linux equivalent (harmless on macOS).
export LD_LIBRARY_PATH="$NS3_ROOT/build/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$RESULTS" "$LOGDIR"; cd "$NS3_ROOT"

TRACEFLAGS="--macSummary=1"; SUFFIX="mac"
[ "$PHYRX" = "1" ] && { TRACEFLAGS="--macSummary=1 --phyRxTrace=1"; SUFFIX="macrx"; }
SUFFIX="${SCENARIO}_${SUFFIX}"   # scenario-namespaced so different channels don't skip each other

simtime_for() { [ -n "${SIMTIME:-}" ] && { echo "$SIMTIME"; return; }; local r=$((10#$1)); [ "$r" -le 50 ] && echo 200 || echo 60; }
free_gb() {
  # Portable free-memory probe: /proc/meminfo on Linux, vm_stat on macOS.
  if [ -r /proc/meminfo ]; then
    awk '/^MemAvailable:/ {printf "%d", $2/1048576; exit}' /proc/meminfo
  else
    local ps fr ia
    ps=$(vm_stat|sed -n '1s/.*page size of \([0-9]*\) bytes.*/\1/p')
    fr=$(vm_stat|awk '/Pages free/{gsub(/[^0-9]/,"",$3);print $3}')
    ia=$(vm_stat|awk '/Pages inactive/{gsub(/[^0-9]/,"",$3);print $3}')
    echo $(((fr+ia)*ps/1073741824))
  fi
}

run_one() { local tag="$1" st="$2" log="$LOGDIR/${1}_${SUFFIX}.log"
  rm -f "$RESULTS/${tag}-${SCENARIO}-propagation-compare.db"
  "$BIN" --tag="$tag" --inputsDir="$INPUTS" --simTime="$st" $TRACEFLAGS \
         --channelScenario="$SCENARIO" --outputDir="$RESULTS/" > "$log" 2>&1
  grep '^RESULTCSV,' "$log" | tail -1 > "$LOGDIR/${tag}_${SUFFIX}.result"; }

echo "=== teacher collection start $(date) scenario=$SCENARIO flags='$TRACEFLAGS' rates=[$RATES] ==="
for r in $RATES; do st=$(simtime_for "$r"); for s in $SEEDS; do
  tag="cav${r}_s${s}"; res="$LOGDIR/${tag}_${SUFFIX}.result"
  if [ -s "$res" ] && grep -q '^RESULTCSV,' "$res"; then echo "  skip $tag (done)"; continue; fi
  while :; do r_n=$(jobs -rp|wc -l|tr -d ' '); [ "$r_n" -lt "$MAXP" ] && [ "$(free_gb)" -ge "$MIN_FREE_GB" ] && break; sleep 5; done
  echo "  launch $tag (st=${st}s, running=$r_n, freeGB=$(free_gb))"; run_one "$tag" "$st" & sleep 4
done; done
wait
echo "=== teacher collection done $(date) -> $RESULTS ==="