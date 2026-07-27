#!/usr/bin/env bash
# =============================================================================
# run_ns3_flood_validate.sh — HELD-OUT asymmetric validation of the load-driven
# collision model. Normal CAVs stay at 10 Hz; ONE node floods (dynamic/aperiodic
# SL grant, 3GPP TS 38.321) at floodNodeRate pps, concentrating load on one UE.
#
# Tests the one assumption the load model rests on: does collision still collapse
# onto TOTAL offered load when load is CONCENTRATED in one node (a real flood),
# not distributed as in the symmetric rate sweep? Reuses the EXISTING mobility.
#
# REQUIRES a rebuilt binary (the scenario gained --floodNodeRate/Dynamic/Id):
#     cd ${NS3_ROOT} && ./ns3 build
# SMOKE TEST first (one short run; confirms the dynamic bearer is stable):
#     DENS="05" FLOODPPS="200" SEEDS="0" SIMTIME=30 ./run_ns3_flood_validate.sh
#
# macSummary only (tx schedule -> occupancy + collisions). Resumable, RAM-guarded.
#   ./run_ns3_flood_validate.sh
#   FLOODPPS="1000" DENS="05 25" ./run_ns3_flood_validate.sh
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
RESULTS="$NS3_SCRATCH/results/teacher_flood"
LOGDIR="$RESULTS/logs"
BIN="$NS3_ROOT/build/scratch/propagation-compare/ns3.42-propagation-compare-default"
SCENARIO="${SCENARIO:-V2V_Urban}"
SEEDS="${SEEDS:-0 1 2}"
FLOODPPS="${FLOODPPS:-200 1000}"     # one node floods at these rates (pps); 1000 matches the OMNeT attack
DENS="${DENS:-05 25 50}"
MAXP="${MAXP:-6}"
MIN_FREE_GB="${MIN_FREE_GB:-6}"
export DYLD_LIBRARY_PATH="$NS3_ROOT/build/lib:${DYLD_LIBRARY_PATH:-}"
# Linux equivalent (harmless on macOS).
export LD_LIBRARY_PATH="$NS3_ROOT/build/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$RESULTS" "$LOGDIR"; cd "$NS3_ROOT"

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

run_one() { local tag="$1" pps="$2" st="$3"
  local out log; out="$RESULTS/pps${pps}"; log="$LOGDIR/${tag}_pps${pps}.log"; mkdir -p "$out"
  rm -f "$out/${tag}-${SCENARIO}-propagation-compare.db"
  # normal CAVs at 10 Hz (default dataRateBe/RRI); one node floods at $pps via dynamic grant.
  "$BIN" --tag="$tag" --inputsDir="$INPUTS" --simTime="$st" --macSummary=1 \
         --floodNodeRate="$pps" --floodNodeDynamic=true \
         --channelScenario="$SCENARIO" --outputDir="$out/" > "$log" 2>&1
  grep '^RESULTCSV,' "$log" | tail -1 > "$out/${tag}_pps${pps}.result"
  grep '^FLOODNODE' "$log" | tail -1 >> "$out/${tag}_pps${pps}.result"; }

echo "=== flood validation start $(date) scenario=$SCENARIO floodpps=[$FLOODPPS] dens=[$DENS] seeds=[$SEEDS] ==="
for pps in $FLOODPPS; do for d in $DENS; do st=$(simtime_for "$d"); for s in $SEEDS; do
  tag="cav${d}_s${s}"; res="$RESULTS/pps${pps}/${tag}_pps${pps}.result"
  if [ -s "$res" ] && grep -q '^RESULTCSV,' "$res"; then echo "  skip $tag pps$pps (done)"; continue; fi
  while :; do r_n=$(jobs -rp|wc -l|tr -d ' '); [ "$r_n" -lt "$MAXP" ] && [ "$(free_gb)" -ge "$MIN_FREE_GB" ] && break; sleep 5; done
  echo "  launch $tag pps$pps (st=${st}s, freeGB=$(free_gb))"; run_one "$tag" "$pps" "$st" & sleep 4
done; done; done
wait
echo "=== flood validation done $(date) -> $RESULTS ==="
