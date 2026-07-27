#!/usr/bin/env bash
# =============================================================================
# run_sumo_fcd_seeds.sh
#
# Plain-SUMO FCD generation across the SAME 5 seeds the OMNeT sweep uses
# (Prop_* configs set *.manager.seed = ${repetition}, repeat=5 -> seeds 0..4).
# So seed S here reproduces the vehicle trajectories of OMNeT repetition S
# (same SUMO 1.22 binary + same cfg + same --seed => identical mobility), and
# the NS-3 replay of seed S lines up with OMNeT run S.
#
# Output is seed-segregated so seeds don't clobber each other:
#     ns3_mobility/cavNN/seedS/fcd_cavNN.xml
#     ns3_mobility/cavNN/seedS/fcd_cavNN.csv
#     ns3_mobility/cavNN/seedS/mobility_cavNN.tcl   (ns-2 mobility for ns-3)
#
# Usage:
#   ./run_sumo_fcd_seeds.sh                 # rates 01 05 25 50 75 100, seeds 0..4, END=200
#   SEEDS="0 1 2 3 4" ./run_sumo_fcd_seeds.sh
#   END=200 ./run_sumo_fcd_seeds.sh 25 100  # subset of rates
# =============================================================================
set -eo pipefail


# Paths (OMNETPP_ROOT, VEINS_ROOT, SUMO_HOME, PROJ, ...) come from env.sh.
RR_ROOT="${RR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=../env.sh
. "$RR_ROOT/env.sh"
PROJ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJ_DIR"

# Force SUMO 1.22 even if the shell already exports SUMO_HOME=sumo-1.11.0
# (the net uses vClasses unknown to 1.11). Override with SUMO122=/path if needed.
export SUMO_HOME="${SUMO122:-$SUMO_HOME}"
SUMO_BIN="$SUMO_HOME/bin/sumo"
XML2CSV="$SUMO_HOME/tools/xml/xml2csv.py"
TRACE_EXPORT="$SUMO_HOME/tools/traceExporter.py"
PYTHON="$(command -v python3 || command -v python)"
[ -x "$SUMO_BIN" ] || { echo "ERROR: sumo not found at $SUMO_BIN (set SUMO_HOME)"; exit 1; }

END="${END:-200}"            # match the Prop_* OMNeT sim-time-limit (200s)
STEP="${STEP:-0.1}"
SEEDS="${SEEDS:-0 1 2 3 4}"  # the OMNeT repetition seeds

DEFAULT_RATES=(01 05 25 50 75 100)
if [ "$#" -gt 0 ]; then RATES=("$@"); else RATES=("${DEFAULT_RATES[@]}"); fi

echo "SUMO   : $SUMO_BIN"
echo "rates  : ${RATES[*]}"
echo "seeds  : ${SEEDS}"
echo "END=${END}s step=${STEP}s"
echo

for r in "${RATES[@]}"; do
    cfg="intersection_cav${r}.sumo.cfg"
    [ -f "$cfg" ] || { echo "!! skip cav${r}: $cfg not found"; continue; }
    for s in ${SEEDS}; do
        out="$PROJ_DIR/ns3_mobility/cav${r}/seed${s}"
        mkdir -p "$out"
        fcd="$out/fcd_cav${r}.xml"
        csv="$out/fcd_cav${r}.csv"
        echo "=== cav${r} seed ${s} -> $fcd ==="
        "$SUMO_BIN" -c "$cfg" \
            --begin 0 --end "$END" --step-length "$STEP" \
            --seed "$s" \
            --fcd-output "$fcd" --fcd-output.acceleration \
            --no-step-log true --duration-log.statistics false --verbose false

        "$PYTHON" "$XML2CSV" "$fcd" -o "$csv" -s ,

        "$PYTHON" "$TRACE_EXPORT" \
            --fcd-input "$fcd" \
            --ns2mobility-output "$out/mobility_cav${r}.tcl" \
            --ns2config-output  "$out/config_cav${r}.tcl" \
            --ns2activity-output "$out/activity_cav${r}.tcl" \
            --begin 0 --end "$END" --penetration 1.0 --seed "$s" \
            || echo "  (traceExporter NS-2 step optional)"

        rows=$(($(wc -l < "$csv") - 1))
        echo "    cav${r} seed ${s}: ${rows} sample rows"
    done
    echo
done

echo "Done. Layout: ns3_mobility/cav<rate>/seed<0..4>/fcd_cav<rate>.csv (+ mobility .tcl)"
echo "Feed a given seed to NS-3, e.g.:"
echo "  python3 \$NS3/scratch/propagation-compare/make_cav_ns3_inputs.py \\"
echo "      --fcd-csv ns3_mobility/cav25/seed3/fcd_cav25.csv --tag cav25_s3 --outdir \$NS3/.../inputs"
