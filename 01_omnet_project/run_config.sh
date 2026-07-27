#!/usr/bin/env bash
# =============================================================================
# run_config.sh: run a single named configuration from omnetpp_inet.ini.
#
# run_propagation_compare.sh only builds the Prop_<treatment>_CAV<rate> sweep
# names. Use this for configs outside that grid: the _MS50 out-of-sample runs,
# the ablations, or a legacy treatment.
#
# PREREQUISITE: the Veins launch daemon must be running with SUMO 1.22 on PATH.
#     export PATH="$SUMO_HOME/bin:$PATH"
#     "$VEINS_ROOT"/bin/veins_launchd -vv
#
# Usage:
#   ./run_config.sh Prop_NS3Learn_MS50            # seed 0
#   ./run_config.sh Prop_Analytical_M3_MS50 3     # seed 3
#   REPS=5 ./run_config.sh Prop_NS3Learn_MS50     # seeds 0..4
#
# Skips runs that already produced a dist_pdr_*.csv marker.
# =============================================================================
set -uo pipefail

# Paths (OMNETPP_ROOT, VEINS_ROOT, SUMO_HOME, PROJ, ...) come from env.sh.
RR_ROOT="${RR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=../env.sh
. "$RR_ROOT/env.sh"

INI="omnetpp_inet.ini"

CFG="${1:-}"
if [ -z "$CFG" ]; then
    echo "Usage: $0 <ConfigName> [seed]" >&2
    echo "" >&2
    echo "Available configs:" >&2
    grep -oE '^\[Config [A-Za-z0-9_]+\]' "$PROJ/$INI" | sed 's/\[Config /  /; s/\]//' >&2
    exit 1
fi

if ! grep -q "^\[Config ${CFG}\]" "$PROJ/$INI"; then
    echo "ERROR: no [Config ${CFG}] in $INI." >&2
    echo "Run '$0' with no arguments to list the available configurations." >&2
    exit 1
fi

if [ -n "${2:-}" ]; then
    SEEDS="$2"
else
    REPS="${REPS:-1}"
    SEEDS="$(seq 0 $((REPS - 1)))"
fi

rr_require OMNETPP_ROOT OPP_RUN SUMO_HOME || exit 1

cd "$PROJ"
rc_final=0
for s in $SEEDS; do
    OUTDIR="$PROJ/results/inet/${CFG}-${s}"
    if ls "$OUTDIR"/dist_pdr_*.csv >/dev/null 2>&1; then
        echo "skip ${CFG}-${s} (already complete)"
        continue
    fi
    mkdir -p "$OUTDIR"
    LOG="$OUTDIR/${CFG}-${s}.out"
    echo "RUN ${CFG}-${s}"

    /bin/zsh -lc "
        cd '$PROJ' &&
        export OPP_ENV_VERSION=1 &&
        export OMNETPP_ROOT='$OMNETPP_ROOT' &&
        export SUMO_HOME='$SUMO_HOME' &&
        export PATH='$SUMO_HOME/bin':\$PATH &&
        source '$OMNETPP_ROOT/setenv' -q &&
        '$OPP_RUN' -M release -- -r $s -m -u Cmdenv -c '$CFG' --debug-on-errors=false '$INI'
    " >"$LOG" 2>&1
    RC=$?

    # Exit 138 is the teardown SIGBUS. dist_pdr is flushed during the run, so
    # treat it as success when the file is present.
    if ls "$OUTDIR"/dist_pdr_*.csv >/dev/null 2>&1; then
        echo "  OK ${CFG}-${s} (rc=$RC)"
    else
        echo "  FAIL ${CFG}-${s} (rc=$RC, no dist_pdr). See $LOG"
        rc_final=1
    fi
done
exit $rc_final
