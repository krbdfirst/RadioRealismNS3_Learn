#!/bin/zsh
# Orchestrates the two-model intersection sweep sequentially (no RAM contention):
#   1) wait for the already-running Analytical_M3 sweep to finish
#   2) run the NS3Learn sweep on the current build
# Both are idempotent/resumable. Progress in results/inet/_sweep_*.log.

# Paths (OMNETPP_ROOT, VEINS_ROOT, SUMO_HOME, PROJ, ...) come from env.sh.
RR_ROOT="${RR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=../env.sh
. "$RR_ROOT/env.sh"
cd "$PROJ"

echo "[chain] $(date) waiting for Analytical_M3 sweep to finish..."
# M3 sweep done when its runner prints the done banner (or no M3 opp_run remain)
until grep -q "PARALLEL resume done" results/inet/_sweep_m3.log 2>/dev/null; do
  sleep 30
done
echo "[chain] $(date) Analytical_M3 done. Starting NS3Learn sweep."

TREATMENTS="NS3Learn" MAXP=6 MIN_FREE_GB=6 REPS=5 ./run_omnet_resume_par.sh > results/inet/_sweep_ns3learn.log 2>&1
echo "[chain] $(date) NS3Learn sweep finished. Both models complete."
