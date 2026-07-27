#!/usr/bin/env bash
# =============================================================================
# run_propagation_compare.sh
#
# Baseline propagation-realism comparison sweep (PropagationRealismProject).
# 4 channel treatments x 6 CAV adoption rates = 24 OMNeT/Veins/INET configs,
# all BASELINE (no background interferer). Used to compare PC5 communication
# behavior against NS-3 5G LENA.
#
#   Treatments: Plain | Analytical (3GPP TR38.901) | Calibrated (MATLAB) | Generalized (5G Toolbox)
#   Rates     : 01 05 25 50 75 100  (%CAV)
#
# PREREQUISITE: the Veins launch daemon must be running and must spawn SUMO 1.22
#   (the net uses vClasses unknown to 1.11.0). In another terminal:
#       export PATH=${SUMO_HOME}/bin:$PATH
#       ${LAUNCHD} -vv
#
# Usage:
#   ./run_propagation_compare.sh                 # all 24, REPS=1 (seed 0)
#   REPS=5 ./run_propagation_compare.sh          # 5 seeds each (matches repeat=5)
#   ./run_propagation_compare.sh Calibrated      # only the Calibrated treatment (6)
#   ./run_propagation_compare.sh CAV50           # only the 50% rate across treatments (4)
#   ./run_propagation_compare.sh Plain CAV100    # substring filters are AND-combined
# =============================================================================
# Note: macOS /usr/bin/env bash is 3.2, which errors on empty-array expansion
# under `set -u`; we omit nounset deliberately. Also: in zsh, an inline "# ..."
# comment is NOT stripped and gets passed as arguments — don't append comments
# to the command (just run: REPS=5 ./run_propagation_compare.sh).
set -eo pipefail


# Paths (OMNETPP_ROOT, VEINS_ROOT, SUMO_HOME, PROJ, ...) come from env.sh.
RR_ROOT="${RR_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# shellcheck source=../env.sh
. "$RR_ROOT/env.sh"
OPP_RUN="${VEINS_ROOT}/subprojects/veins_inet/bin/veins_inet_run"
INI="omnetpp_inet.ini"

REPS="${REPS:-1}"          # repetitions per config (configs declare repeat=5)
LAST=$((REPS - 1))

# Headline set = the two study models. Legacy treatments (Plain/Analytical[tr38901]/
# Calibrated/Generalized) are retired from the comparison; pass them explicitly if needed.
TREATMENTS=(Analytical_M3 NS3Learn)
RATES=(01 05 25 50 75 100)

ALL_CONFIGS=()
for t in "${TREATMENTS[@]}"; do
    for r in "${RATES[@]}"; do
        ALL_CONFIGS+=("Prop_${t}_CAV${r}")
    done
done

# AND-combine any substring filters passed as arguments.
for FILTER in "$@"; do
    KEEP=()
    for cfg in "${ALL_CONFIGS[@]}"; do
        [[ "${cfg}" == *"${FILTER}"* ]] && KEEP+=("${cfg}")
    done
    ALL_CONFIGS=("${KEEP[@]}")
    if [ ${#ALL_CONFIGS[@]} -eq 0 ]; then
        echo "No configs match filter '${FILTER}'. Valid filters: Plain Analytical Calibrated Generalized CAV01 CAV05 CAV25 CAV50 CAV75 CAV100"
        echo "(Tip: in zsh, don't append a '# ...' comment — it is passed as arguments.)"
        exit 1
    fi
done

echo "======================================================================"
echo "  Propagation comparison sweep — ${#ALL_CONFIGS[@]} config(s) x ${REPS} rep(s)"
echo "  runs -r 0..${LAST}   (config repeat=5, seed-set=repetition, manager.seed=repetition)"
echo "======================================================================"

cd "${PROJ}"
PASSED=0
FAILED=()

# Each seed is run as its OWN opp_run process: a single "-r 0..N" invocation
# stops after run 0 due to a Veins/SUMO-launchd lifecycle issue. Also, a
# seed/traffic-dependent SIGBUS (exit 138) fires in module teardown AFTER the
# run completes; the in-sim dist_pdr flush makes that harmless, so we treat
# exit 138 as success when dist_pdr (or the .sca) was produced.
for CFG in "${ALL_CONFIGS[@]}"; do
    for s in $(seq 0 ${LAST}); do
        echo ""
        echo "----------------------------------------------------------------------"
        echo "  Running: ${CFG}  seed ${s}"
        echo "----------------------------------------------------------------------"
        OUTDIR="${PROJ}/results/inet/${CFG}-${s}"
        mkdir -p "${OUTDIR}"
        LOG="${OUTDIR}/${CFG}-${s}.out"

        set +e
        /bin/zsh -lc "
            cd '${PROJ}' &&
            export OPP_ENV_VERSION=1 &&
            export OMNETPP_ROOT='${OMNETPP_ROOT}' &&
            source '${OMNETPP_ROOT}/setenv' -q &&
            '${OPP_RUN}' -M release -- \
                -r ${s} \
                -m \
                -u Cmdenv \
                -c '${CFG}' \
                --debug-on-errors=false \
                '${INI}'
        " 2>&1 | tee "${LOG}"
        RC=${PIPESTATUS[0]:-$?}
        set -e

        # Success = clean exit, OR the known teardown SIGBUS (138) with output present.
        if [[ ${RC} -eq 0 ]] || { [[ ${RC} -eq 138 ]] && ls "${OUTDIR}"/dist_pdr_*.csv >/dev/null 2>&1; }; then
            [[ ${RC} -eq 138 ]] && echo "    OK ${CFG} seed ${s} (teardown SIGBUS ignored; dist_pdr written)" \
                                || echo "    OK ${CFG} seed ${s}"
            PASSED=$((PASSED + 1))
        else
            echo "    FAIL ${CFG} seed ${s} (exit ${RC}, no dist_pdr) — see ${LOG}"
            FAILED+=("${CFG}-${s}")
        fi
    done
done

echo ""
echo "======================================================================"
echo "  Sweep complete: ${PASSED} passed, ${#FAILED[@]} failed"
for f in "${FAILED[@]:-}"; do [[ -n "$f" ]] && echo "    - ${f}"; done
echo "  Results: ${PROJ}/results/inet/Prop_*"
echo "======================================================================"
[[ ${#FAILED[@]} -eq 0 ]]
