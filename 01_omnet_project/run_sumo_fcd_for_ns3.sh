#!/usr/bin/env bash
# =============================================================================
# run_sumo_fcd_for_ns3.sh
#
# PRIORITY DELIVERABLE: produce plain-SUMO vehicle trajectory logs for replay
# in NS-3 5G LENA. No OMNeT++, no Veins, no INET — just the SUMO ground truth.
#
# For every CAV adoption rate it runs the matching intersection_cavNN.sumo.cfg
# headless and writes Floating-Car-Data (FCD) containing exactly the fields the
# NS-3 mobility replay needs:
#
#     timestamp (time)  vehicle id (id)  position (x,y[,z])
#     speed             acceleration     heading angle (angle, degrees, 0=North, CW)
#     plus: vehicle type, lane, slope
#
# Outputs, per rate, under  ns3_mobility/cavNN/ :
#     fcd_cavNN.xml            raw SUMO FCD (XML)
#     fcd_cavNN.csv            same data flattened to CSV (easy NS-3 / pandas ingest)
#     mobility_cavNN.tcl       NS-2 mobility format for ns-3 Ns2MobilityHelper
#     config_cavNN.tcl         NS-2 node-count / config sidecar
#     activity_cavNN.tcl       per-node activation (depart/arrival) times
#
# REPRODUCIBILITY / cross-tool consistency:
#   SUMO is seeded with --seed $SEED. To make an OMNeT/Veins run use the SAME
#   vehicle trajectories, run the matching [Config Prop_*_CAVNN] with the OMNeT
#   run number whose repetition maps to this SEED (the ini sets
#   *.manager.seed = ${repetition}). Default SEED=0 == repetition 0.
#
# Usage:
#   ./run_sumo_fcd_for_ns3.sh                 # all rates, END=200s, SEED=0
#   END=1000 SEED=3 ./run_sumo_fcd_for_ns3.sh # custom horizon / seed
#   ./run_sumo_fcd_for_ns3.sh 25 100          # only the 25% and 100% rates
# =============================================================================
set -euo pipefail


# Paths (OMNETPP_ROOT, VEINS_ROOT, SUMO_HOME, PROJ, ...) come from env.sh.
RR_ROOT="${RR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=../env.sh
. "$RR_ROOT/env.sh"
PROJ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJ_DIR"

# ---- locate SUMO -----------------------------------------------------------
# Use SUMO 1.22 by default: intersection.net.xml uses vClasses (cable_car,
# subway, drone, ...) that the older 1.11.0 build does not recognise.
# SUMO_HOME comes from env.sh (SUMO 1.22 required — the net uses newer vClasses).
SUMO_BIN="$SUMO_HOME/bin/sumo"
XML2CSV="$SUMO_HOME/tools/xml/xml2csv.py"
TRACE_EXPORT="$SUMO_HOME/tools/traceExporter.py"
PYTHON="$(command -v python3 || command -v python)"

[ -x "$SUMO_BIN" ] || { echo "ERROR: sumo not found at $SUMO_BIN (set SUMO_HOME)"; exit 1; }

# ---- parameters ------------------------------------------------------------
END="${END:-200}"          # simulation horizon (s). 200 matches the Prop_* OMNeT baseline window.
SEED="${SEED:-0}"          # SUMO RNG seed; mirror with OMNeT *.manager.seed = ${repetition}
STEP="${STEP:-0.1}"        # step length (s) — matches CAM/manager update interval

DEFAULT_RATES=(01 05 25 50 75 100)
if [ "$#" -gt 0 ]; then
    RATES=("$@")
else
    RATES=("${DEFAULT_RATES[@]}")
fi

OUT_ROOT="$PROJ_DIR/ns3_mobility"
mkdir -p "$OUT_ROOT"

echo "SUMO          : $SUMO_BIN"
echo "horizon (END) : ${END}s   step=${STEP}s   seed=${SEED}"
echo "rates         : ${RATES[*]}"
echo "output root   : $OUT_ROOT"
echo

for r in "${RATES[@]}"; do
    cfg="intersection_cav${r}.sumo.cfg"
    if [ ! -f "$cfg" ]; then
        echo "!! skip cav${r}: $cfg not found"
        continue
    fi
    out="$OUT_ROOT/cav${r}"
    mkdir -p "$out"
    fcd="$out/fcd_cav${r}.xml"
    csv="$out/fcd_cav${r}.csv"

    echo "=== cav${r} : running SUMO -> $fcd ==="
    # --fcd-output.acceleration adds the 'acceleration' attribute.
    # x,y,speed,angle,type,lane,slope are emitted by default; id & time are structural.
    "$SUMO_BIN" -c "$cfg" \
        --begin 0 --end "$END" --step-length "$STEP" \
        --seed "$SEED" \
        --fcd-output "$fcd" \
        --fcd-output.acceleration \
        --no-step-log true --duration-log.statistics false --verbose false

    echo "--- cav${r} : FCD -> CSV ($csv) ---"
    "$PYTHON" "$XML2CSV" "$fcd" -o "$csv" -s ,

    echo "--- cav${r} : FCD -> NS-2 mobility (mobility_cav${r}.tcl) ---"
    "$PYTHON" "$TRACE_EXPORT" \
        --fcd-input "$fcd" \
        --ns2mobility-output "$out/mobility_cav${r}.tcl" \
        --ns2config-output  "$out/config_cav${r}.tcl" \
        --ns2activity-output "$out/activity_cav${r}.tcl" \
        --begin 0 --end "$END" --penetration 1.0 \
        --seed "$SEED" || echo "  (traceExporter NS-2 step optional — CSV/XML still produced)"

    rows=$(($(wc -l < "$csv") - 1))
    veh=$("$PYTHON" - "$csv" <<'PY'
import csv,sys
ids=set()
with open(sys.argv[1]) as f:
    for row in csv.DictReader(f):
        vid=row.get("vehicle_id") or row.get("id")
        if vid: ids.add(vid)
print(len(ids))
PY
)
    echo "    cav${r}: ${rows} sample rows, ${veh} distinct road users"
    echo
done

echo "Done. NS-3 ingest options:"
echo "  * mobility_cavNN.tcl  -> ns3 Ns2MobilityHelper (direct trajectory replay)"
echo "  * fcd_cavNN.csv       -> custom loader; filter column 'vehicle_type' to"
echo "                           veh_av (CAV) / veh_human (manual) and drop bike_human/ped_human."
echo "    CSV fields include: timestep_time, vehicle_id, vehicle_x, vehicle_y,"
echo "    vehicle_speed, vehicle_acceleration, vehicle_angle, vehicle_type, vehicle_lane, vehicle_slope"
