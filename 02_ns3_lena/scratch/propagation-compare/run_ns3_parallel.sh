#!/usr/bin/env bash
# =============================================================================
# run_ns3_parallel.sh — parallel full NR V2X (5G-LENA) sweep for the
# PropagationRealismProject benchmark. 6 rates x 5 seeds = 30 runs.
#
# Calls the prebuilt binary directly (no ./ns3 wrapper) so concurrent jobs do
# not race on the build lock. simTime matches the OMNeT side:
#   cav01/05/25/50 -> 200s, cav75/100 -> 60s.
# Scheduler: light->heavy, capped at MAXP concurrent, and refuses to launch a
# new job while free RAM < MIN_FREE_GB (guards against OOM on the 443-UE runs).
# Resumable: a tag whose .result file already holds a RESULTCSV line is skipped.
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
LOGDIR="$RESULTS/parallel_logs"
BIN="$NS3_ROOT/build/scratch/propagation-compare/ns3.42-propagation-compare-default"
SCENARIO="${SCENARIO:-V2V_Urban}"
SEEDS="${SEEDS:-0 1 2 3 4}"
RATES=(01 05 25 50 75 100)
MAXP="${MAXP:-8}"
MIN_FREE_GB="${MIN_FREE_GB:-5}"

export DYLD_LIBRARY_PATH="$NS3_ROOT/build/lib:${DYLD_LIBRARY_PATH:-}"
# Linux equivalent (harmless on macOS).
export LD_LIBRARY_PATH="$NS3_ROOT/build/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$INPUTS" "$RESULTS" "$LOGDIR"
cd "$NS3_ROOT"

simtime_for() { case "$1" in 01|05|25|50) echo 200 ;; 75|100) echo 60 ;; esac; }

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
  local tag="$1" st="$2" log="$LOGDIR/$1.log"
  rm -f "$RESULTS/${tag}-${SCENARIO}-propagation-compare.db"
  "$BIN" --tag="$tag" --inputsDir="$INPUTS" --simTime="$st" \
         --channelScenario="$SCENARIO" --outputDir="$RESULTS/" > "$log" 2>&1
  grep '^RESULTCSV,' "$log" | tail -1 > "$LOGDIR/${tag}.result"
}

echo "=== NS-3 parallel sweep start $(date) ==="
echo "scenario=$SCENARIO seeds=[$SEEDS] MAXP=$MAXP MIN_FREE_GB=$MIN_FREE_GB"

# 1) Pre-generate all inputs serially (cheap; avoids races + fails fast on missing FCD).
for r in "${RATES[@]}"; do for s in $SEEDS; do
  tag="cav${r}_s${s}"; csv="$FCD_ROOT/cav${r}/seed${s}/fcd_cav${r}.csv"
  [ -f "$csv" ] || { echo "MISSING FCD: $csv"; continue; }
  [ -f "$INPUTS/${tag}_info.txt" ] || \
    python3 "$NS3_SCRATCH/make_cav_ns3_inputs.py" --fcd-csv "$csv" --tag "$tag" \
      --outdir "$INPUTS" --height 1.5 --end 200 >/dev/null 2>&1
done; done
echo "inputs ready."

# 2) Launch light->heavy with concurrency cap + RAM guard.
for r in "${RATES[@]}"; do
  st=$(simtime_for "$r")
  for s in $SEEDS; do
    tag="cav${r}_s${s}"
    if [ -s "$LOGDIR/${tag}.result" ] && grep -q '^RESULTCSV,' "$LOGDIR/${tag}.result"; then
      echo "  skip $tag (done)"; continue
    fi
    # throttle: wait for a free slot AND enough RAM headroom
    while :; do
      running=$(jobs -rp | wc -l | tr -d ' ')
      fg=$(free_gb)
      [ "$running" -lt "$MAXP" ] && [ "$fg" -ge "$MIN_FREE_GB" ] && break
      sleep 5
    done
    echo "  launch $tag (simTime=${st}s) [running=$running freeGB=$(free_gb)]"
    run_one "$tag" "$st" &
    sleep 5   # let the new job's footprint register before the next RAM check
  done
done
wait
echo "all jobs finished."

# 3) Aggregate into the summary CSV.
SUMMARY="$RESULTS/summary_${SCENARIO}_seeds.csv"
echo "tag,scenario,seed,numUes,activeTx,txPkt,rxPkt,avgLatency_ms" > "$SUMMARY"
for r in "${RATES[@]}"; do for s in $SEEDS; do
  tag="cav${r}_s${s}"; res="$LOGDIR/${tag}.result"
  if [ -s "$res" ]; then
    IFS=',' read -r _ ctag cscn cnum cact ctx crx clat < "$res"
    echo "${ctag},${cscn},${s},${cnum},${cact},${ctx},${crx},${clat}" >> "$SUMMARY"
  else
    echo "${tag},${SCENARIO},${s},NA,NA,NA,NA,NA" >> "$SUMMARY"
  fi
done; done

echo "=== done $(date) -> $SUMMARY ==="
column -t -s, "$SUMMARY"
