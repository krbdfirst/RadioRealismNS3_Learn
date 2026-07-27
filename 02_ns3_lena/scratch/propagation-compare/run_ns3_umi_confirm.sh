#!/usr/bin/env bash
# =============================================================================
# run_ns3_umi_confirm.sh — like-for-like CHANNEL confirmation run.
# Re-runs the low-load tags (cav01, cav05 x 5 seeds) under
# channelScenario=UMi_StreetCanyon (== OMNeT tr38901_umi / Analytical) instead
# of the V2V_Urban (TR 37.885) used in the main sweep. Everything else is the
# DEFAULT binary config (identical to the main run's invocation), so ONLY the
# propagation/channel-condition model changes. Low load => contention minimal,
# so PRR-vs-distance is propagation-limited and directly comparable to OMNeT
# Analytical. DBs are scenario-namespaced; V2V results are untouched.
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
RESULTS="$NS3_SCRATCH/results"
LOGDIR="$RESULTS/umi_confirm_logs"
BIN="$NS3_ROOT/build/scratch/propagation-compare/ns3.42-propagation-compare-default"
SCENARIO="UMi_StreetCanyon"
SIMTIME=200
SEEDS="0 1 2 3 4"
RATES="01 05"
MAXP=5
export DYLD_LIBRARY_PATH="$NS3_ROOT/build/lib:${DYLD_LIBRARY_PATH:-}"
# Linux equivalent (harmless on macOS).
export LD_LIBRARY_PATH="$NS3_ROOT/build/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$LOGDIR"
cd "$NS3_ROOT"

run_one() {
  local tag="$1" log="$LOGDIR/$1.log"
  rm -f "$RESULTS/${tag}-${SCENARIO}-propagation-compare.db"
  "$BIN" --tag="$tag" --inputsDir="$INPUTS" --simTime="$SIMTIME" \
         --channelScenario="$SCENARIO" --outputDir="$RESULTS/" > "$log" 2>&1
  grep '^RESULTCSV,' "$log" | tail -1 > "$LOGDIR/${tag}.result"
}

echo "=== UMi_StreetCanyon confirmation start $(date) ==="
for r in $RATES; do for s in $SEEDS; do
  tag="cav${r}_s${s}"
  if [ -s "$LOGDIR/${tag}.result" ] && grep -q '^RESULTCSV,' "$LOGDIR/${tag}.result"; then
    echo "  skip $tag (done)"; continue
  fi
  while [ "$(jobs -rp | wc -l | tr -d ' ')" -ge "$MAXP" ]; do sleep 3; done
  echo "  launch $tag"; run_one "$tag" & sleep 2
done; done
wait
echo "=== all UMi confirmation jobs finished $(date) ==="

SUMMARY="$RESULTS/summary_${SCENARIO}_confirm.csv"
echo "tag,scenario,seed,numUes,activeTx,txPkt,rxPkt,avgLatency_ms" > "$SUMMARY"
for r in $RATES; do for s in $SEEDS; do
  tag="cav${r}_s${s}"; res="$LOGDIR/${tag}.result"
  if [ -s "$res" ]; then
    IFS=',' read -r _ ctag cscn cnum cact ctx crx clat < "$res"
    echo "${ctag},${cscn},${s},${cnum},${cact},${ctx},${crx},${clat}" >> "$SUMMARY"
  else
    echo "${tag},${SCENARIO},${s},NA,NA,NA,NA,NA" >> "$SUMMARY"
  fi
done; done
echo "=== summary -> $SUMMARY ==="
column -t -s, "$SUMMARY"
