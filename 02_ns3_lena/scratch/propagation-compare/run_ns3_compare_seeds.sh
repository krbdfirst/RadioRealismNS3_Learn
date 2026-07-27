#!/usr/bin/env bash
# =============================================================================
# run_ns3_compare_seeds.sh
#
# Seed-aware NR V2X (5G-LENA) replay across the 5 SUMO seeds (0..4), matching the
# OMNeT sweep (REPS=5, manager.seed=${repetition}). Reads the seed-segregated FCD
# produced by PropagationRealismProject/run_sumo_fcd_seeds.sh:
#     ns3_mobility/cav<rate>/seed<S>/fcd_cav<rate>.csv
# NS-3 (rate,seed) lines up 1:1 with OMNeT (rate,repetition=S).
#
# Usage:
#   ./run_ns3_compare_seeds.sh                      # all rates x seeds 0..4, V2V_Urban, SIMTIME=60
#   SIMTIME=200 ./run_ns3_compare_seeds.sh          # full window (slow at high CAV%)
#   SEEDS="0" ./run_ns3_compare_seeds.sh 05 25      # one seed, subset of rates (good first pass)
#   SCENARIO=UMi_StreetCanyon ./run_ns3_compare_seeds.sh   # direct analog of OMNeT tr38901_umi
#
# Cost warning: one NS-3 run per (rate,seed). #UEs == #CAVs (cav100 ~443). At
# SIMTIME=200 the high rates take a long time; start with low rates / one seed.
# =============================================================================
set -eo pipefail


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
SIMTIME="${SIMTIME:-60}"
HEIGHT="${HEIGHT:-1.5}"
SEEDS="${SEEDS:-0 1 2 3 4}"

DEFAULT_RATES=(01 05 25 50 75 100)
if [ "$#" -gt 0 ]; then RATES=("$@"); else RATES=("${DEFAULT_RATES[@]}"); fi

mkdir -p "$INPUTS" "$RESULTS"
SUMMARY="$RESULTS/summary_${SCENARIO}_seeds.csv"
echo "tag,scenario,seed,numUes,activeTx,txPkt,rxPkt,avgLatency_ms" > "$SUMMARY"

cd "$NS3_ROOT"
echo "scenario=$SCENARIO  simTime=${SIMTIME}s  rates=${RATES[*]}  seeds=${SEEDS}"
echo "summary -> $SUMMARY"
echo

for r in "${RATES[@]}"; do
    for s in ${SEEDS}; do
        csv="$FCD_ROOT/cav${r}/seed${s}/fcd_cav${r}.csv"
        if [ ! -f "$csv" ]; then
            echo "!! cav${r} seed${s}: $csv missing — run run_sumo_fcd_seeds.sh. skipping."
            continue
        fi
        tag="cav${r}_s${s}"
        if [ ! -f "$INPUTS/${tag}_info.txt" ]; then
            python3 "$NS3_SCRATCH/make_cav_ns3_inputs.py" \
                --fcd-csv "$csv" --tag "$tag" --outdir "$INPUTS" --height "$HEIGHT" --end 200
        fi
        nUes=$(grep '^numUes=' "$INPUTS/${tag}_info.txt" | cut -d= -f2)
        echo "=== cav${r} seed${s}: ${nUes} CAV UEs, ${SCENARIO}, ${SIMTIME}s ==="

        # Remove any pre-existing DB so we don't inherit stale (possibly huge,
        # pre-phyTraces-fix) trace tables; the run rebuilds it trace-light.
        rm -f "$RESULTS/${tag}-${SCENARIO}-propagation-compare.db"

        line=$(./ns3 run "propagation-compare/propagation-compare \
            --tag=${tag} --inputsDir=$INPUTS --simTime=$SIMTIME \
            --channelScenario=$SCENARIO --outputDir=$RESULTS/" 2>&1 \
            | tee /dev/stderr | grep '^RESULTCSV,' || true)

        if [ -n "$line" ]; then
            # RESULTCSV,<tag>,<scenario>,<numUes>,<activeTx>,<txPkt>,<rxPkt>,<lat>
            # -> summary columns: tag,scenario,seed,numUes,activeTx,txPkt,rxPkt,lat
            IFS=',' read -r _ ctag cscn cnum cact ctx crx clat <<< "$line"
            echo "${ctag},${cscn},${s},${cnum},${cact},${ctx},${crx},${clat}" >> "$SUMMARY"
        else
            echo "cav${r}_s${s},${SCENARIO},${s},NA,NA,NA,NA,NA" >> "$SUMMARY"
        fi
        echo
    done
done

echo "================ summary (${SCENARIO}, seeds ${SEEDS}) ================"
column -t -s, "$SUMMARY"
echo
echo "Per-(rate,seed) DB: results/cav<rate>_s<seed>-${SCENARIO}-propagation-compare.db"
echo "  avrgPrr (PRR@300m) | thput (per-link PDR) | avrgPir (PDR-vs-distance)"
