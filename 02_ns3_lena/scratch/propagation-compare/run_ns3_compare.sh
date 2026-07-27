#!/usr/bin/env bash
# =============================================================================
# run_ns3_compare.sh
#
# Drive the NR V2X (5G-LENA) sidelink replay across CAV adoption rates, using
# the SUMO mobility produced by PropagationRealismProject/run_sumo_fcd_for_ns3.sh.
# For each rate it: (1) builds the CAV-only ns-2 mobility inputs, (2) runs the
# NS-3 program, (3) collects the RESULTCSV summary line.
#
# This is the NS-3 5G-LENA reference that the three OMNeT propagation treatments
# (Plain / Analytical / Calibrated / Generalized) are compared against.
#
# Usage:
#   ./run_ns3_compare.sh                       # all rates, V2V_Urban, simTime=60
#   SIMTIME=200 ./run_ns3_compare.sh           # full 200 s window (slow at high CAV%)
#   SCENARIO=UMi_StreetCanyon ./run_ns3_compare.sh   # direct analog of OMNeT tr38901_umi
#   ./run_ns3_compare.sh 01 05 25              # subset of rates
#
# NOTE on runtime: node count == #CAVs (cav01~6 ... cav100~443). NR sidelink with
# hundreds of UEs is heavy; start with low rates / short SIMTIME. cav100 over 200 s
# can take hours.
# =============================================================================
set -euo pipefail


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

SCENARIO="${SCENARIO:-V2V_Urban}"   # V2V_Urban | V2V_Highway | UMi_StreetCanyon
SIMTIME="${SIMTIME:-60}"            # seconds of CAM exchange
HEIGHT="${HEIGHT:-1.5}"             # antenna height (m), matches OMNeT spatUtHeight

DEFAULT_RATES=(01 05 25 50 75 100)
if [ "$#" -gt 0 ]; then RATES=("$@"); else RATES=("${DEFAULT_RATES[@]}"); fi

mkdir -p "$INPUTS" "$RESULTS"
SUMMARY="$RESULTS/summary_${SCENARIO}.csv"
echo "tag,scenario,numUes,activeTx,txPkt,rxPkt,avgLatency_ms" > "$SUMMARY"

cd "$NS3_ROOT"
echo "scenario=$SCENARIO  simTime=${SIMTIME}s  rates=${RATES[*]}"
echo "summary -> $SUMMARY"
echo

for r in "${RATES[@]}"; do
    csv="$FCD_ROOT/cav${r}/fcd_cav${r}.csv"
    if [ ! -f "$csv" ]; then
        echo "!! cav${r}: $csv missing — run run_sumo_fcd_for_ns3.sh first. skipping."
        continue
    fi
    # (1) build CAV-only mobility inputs (idempotent)
    if [ ! -f "$INPUTS/cav${r}_info.txt" ]; then
        python3 "$NS3_SCRATCH/make_cav_ns3_inputs.py" \
            --fcd-csv "$csv" --tag "cav${r}" --outdir "$INPUTS" \
            --height "$HEIGHT" --end 200
    fi
    nUes=$(grep '^numUes=' "$INPUTS/cav${r}_info.txt" | cut -d= -f2)
    echo "=== cav${r}: ${nUes} CAV UEs, scenario=${SCENARIO}, simTime=${SIMTIME}s ==="

    # (2) run NS-3
    line=$(./ns3 run "propagation-compare/propagation-compare \
        --tag=cav${r} --inputsDir=$INPUTS --simTime=$SIMTIME \
        --channelScenario=$SCENARIO --outputDir=$RESULTS/" 2>&1 | tee /dev/stderr | grep '^RESULTCSV,')

    # (3) collect summary (strip the RESULTCSV prefix)
    echo "${line#RESULTCSV,}" >> "$SUMMARY"
    echo
done

echo "================ summary (${SCENARIO}) ================"
column -t -s, "$SUMMARY"
echo
echo "Per-link PDR & PDR-vs-distance live in each results/cav<rate>-${SCENARIO}-propagation-compare.db :"
echo "  - avrgPrr : per-TX packet reception ratio within ${SCENARIO} KPI range (300 m)"
echo "  - thput   : per-link totalPktTxed/totalPktRxed  (PDR = rx/tx)"
echo "  - avrgPir : per-link avg inter-reception + TxRxDistance (PDR-vs-distance)"
