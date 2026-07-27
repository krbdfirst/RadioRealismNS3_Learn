#!/usr/bin/env bash
# =============================================================================
# run_ns3_rate_sweep.sh — decouple OFFERED LOAD from VEHICLE DENSITY for the
# flood / channel-busy-ratio (CBR) contention extension (closing the C2 gap).
#
# WHY: the ns3learn collision term was fit to vehicle DENSITY because, in the
# original teacher runs, every node beaconed at the same 10 Hz so load ∝ density.
# A flood (1 node, ~100x load) breaks that proxy. This sweep varies the CAM rate
# at fixed density so the fitter can learn collision = f(offered load / CBR).
#
# REUSES the EXISTING SUMO->ns-2 mobility inputs ($INPUTS/cav{dd}_s{s}_mobility.tcl)
# UNCHANGED. Nothing about topology / mobility / channel is regenerated — ONLY the
# application CAM transmit rate (dataRateBe + RRI) changes. The 10 Hz runs therefore
# also reproduce the existing results/teacher data (a built-in sanity check).
#
# Rate <-> RRI mapping:
#   10 Hz -> dataRateBe=20.48 kb/s, reservationPeriod=100 ms   (ETSI EN 302 637-2 CAM baseline)
#   25 Hz -> dataRateBe=51.20 kb/s, reservationPeriod=40  ms   (2.5x load; clean 2nd level)
#   HARD CONSTRAINT (5G-LENA nr-sl-comm-resource-pool.cc:554): the RRI in slots MUST be an
#   integer multiple of the physical SL pool length (= 20 slots here) AND >= selection window
#   T2 (= 33 slots). => valid RRIs are 40/60/80/100 ms (25/16.7/12.5/10 Hz). 50 ms (20 Hz) is
#   INVALID (50 % 20 != 0 -> NS_FATAL); 20 ms (50 Hz) needs T2<=20 (exposing `t2`, a later add).
#   RRI/period values per 3GPP TS 38.331 sl-ResourceReservePeriodList; traffic per 3GPP TR 37.885.
#   packetSize 256 B fixed => kb/s = 2.048 * Hz.
#
# macSummary only (cheap: TX scheduling -> simulPsschTx occupancy + SB-SPS collisions),
# matching run_ns3_teacher.sh. Resumable (per-tag .result), RAM-guarded, light->heavy.
#   ./run_ns3_rate_sweep.sh
#   RATES_HZ="20" DENS="05 25" SEEDS="0 1 2" ./run_ns3_rate_sweep.sh
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
RESULTS="$NS3_SCRATCH/results/teacher_rate"
LOGDIR="$RESULTS/logs"
BIN="$NS3_ROOT/build/scratch/propagation-compare/ns3.42-propagation-compare-default"
SCENARIO="${SCENARIO:-V2V_Urban}"
SEEDS="${SEEDS:-0 1 2}"
RATES_HZ="${RATES_HZ:-10 25}"
DENS="${DENS:-05 25 50}"
MAXP="${MAXP:-6}"
MIN_FREE_GB="${MIN_FREE_GB:-6}"
export DYLD_LIBRARY_PATH="$NS3_ROOT/build/lib:${DYLD_LIBRARY_PATH:-}"
# Linux equivalent (harmless on macOS).
export LD_LIBRARY_PATH="$NS3_ROOT/build/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$RESULTS" "$LOGDIR"; cd "$NS3_ROOT"

# packet = 256 B fixed => data rate (kb/s) = 256*8*Hz/1000 = 2.048*Hz; RRI = 1000/Hz ms.
dr_for()  { awk -v h="$1" 'BEGIN{printf "%.3f", 2.048*h}'; }
rri_for() { echo $((1000 / $1)); }
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

run_one() { local tag="$1" hz="$2" st="$3"
  local dr rri out log; dr=$(dr_for "$hz"); rri=$(rri_for "$hz")
  out="$RESULTS/hz${hz}"; log="$LOGDIR/${tag}_hz${hz}.log"; mkdir -p "$out"
  rm -f "$out/${tag}-${SCENARIO}-propagation-compare.db"
  "$BIN" --tag="$tag" --inputsDir="$INPUTS" --simTime="$st" --macSummary=1 \
         --dataRateBe="$dr" --reservationPeriod="$rri" \
         --channelScenario="$SCENARIO" --outputDir="$out/" > "$log" 2>&1
  grep '^RESULTCSV,' "$log" | tail -1 > "$out/${tag}_hz${hz}.result"; }

echo "=== rate sweep start $(date) scenario=$SCENARIO rates=[$RATES_HZ]Hz dens=[$DENS] seeds=[$SEEDS] ==="
for hz in $RATES_HZ; do for d in $DENS; do st=$(simtime_for "$d"); for s in $SEEDS; do
  tag="cav${d}_s${s}"; res="$RESULTS/hz${hz}/${tag}_hz${hz}.result"
  if [ -s "$res" ] && grep -q '^RESULTCSV,' "$res"; then echo "  skip $tag hz$hz (done)"; continue; fi
  while :; do r_n=$(jobs -rp|wc -l|tr -d ' '); [ "$r_n" -lt "$MAXP" ] && [ "$(free_gb)" -ge "$MIN_FREE_GB" ] && break; sleep 5; done
  echo "  launch $tag hz$hz (dr=$(dr_for "$hz")kb/s rri=$(rri_for "$hz")ms, st=${st}s, freeGB=$(free_gb))"
  run_one "$tag" "$hz" "$st" & sleep 4
done; done; done
wait
echo "=== rate sweep done $(date) -> $RESULTS ==="
