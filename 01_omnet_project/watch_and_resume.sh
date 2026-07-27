#!/usr/bin/env bash
# =============================================================================
# watch_and_resume.sh — wait for the WeatherFloodingAttack ns-3 matrices to
# finish, then resume BOTH paused propagation sweeps (OMNeT + NS-3).
# Detached watcher: polls every 60s; survives across sessions.
# =============================================================================
set -uo pipefail

# Paths (OMNETPP_ROOT, VEINS_ROOT, SUMO_HOME, PROJ, ...) come from env.sh.
RR_ROOT="${RR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=../env.sh
. "$RR_ROOT/env.sh"
NS3_PROP="${NS3_SCRATCH}"
TS() { date "+%Y-%m-%d %H:%M:%S"; }

weather_running() {
  pgrep -f 'ns3.42-weather-v2x-mode2' >/dev/null 2>&1 && return 0
  pgrep -f 'run_ns3_matrix_par'       >/dev/null 2>&1 && return 0
  return 1
}

echo "[$(TS)] watcher up; waiting for weather sweep to finish..."
while weather_running; do sleep 60; done
echo "[$(TS)] weather sweep finished. Resuming propagation sweeps."

# Ensure a headless SUMO-1.22 launchd is alive for the OMNeT resume.
if ! pgrep -f 'bin/veins_launchd' >/dev/null 2>&1; then
  echo "[$(TS)] starting veins_launchd (SUMO 1.22)..."
  SUMO_HOME="$SUMO_HOME" PATH="$SUMO_HOME/bin:$PATH" \
    "$LAUNCHD" -vv > /tmp/veins_launchd_resume.log 2>&1 &
  sleep 3
fi

# Resume both, in parallel (resources are free once weather is done).
( cd "$PROJ" && REPS=5 bash run_omnet_resume.sh \
    > "$PROJ/results/omnet_resume_$(date +%Y%m%d_%H%M%S).log" 2>&1 ) &
OMNET_PID=$!
( cd "$NS3_PROP" && bash run_ns3_parallel.sh \
    > "$NS3_PROP/results/parallel_logs/_driver_resume.log" 2>&1 ) &
NS3_PID=$!

echo "[$(TS)] omnet_resume pid=$OMNET_PID  ns3_resume pid=$NS3_PID"
wait "$OMNET_PID"; echo "[$(TS)] OMNeT resume exited ($?)."
wait "$NS3_PID";   echo "[$(TS)] NS-3 resume exited ($?)."
echo "[$(TS)] ALL RESUMES COMPLETE."
