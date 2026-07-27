#!/usr/bin/env bash
# =============================================================================
# env.sh: path definitions shared by all run scripts.
#
# Sourced by every script in 01_omnet_project/ and by the ns-3 scratch scripts.
# No other file hard-codes a path.
#
# To adapt to a new machine, either export the variables in your shell or set
# them in env.local.sh next to this file (git-ignored, sourced last):
#
#   OMNETPP_ROOT=/opt/omnetpp-6.0.3
#   INET_ROOT=/opt/inet4.5
#   VEINS_ROOT=/opt/veins-veins-5.3.1
#   NS3_ROOT=/opt/ns-3-dev
#   SUMO_HOME=/opt/sumo-1.22.0
# =============================================================================

# --- repository root ---------------------------------------------------------
# Derived from this file's location so the checkout can live anywhere.
if [ -z "${RR_ROOT:-}" ]; then
    _rr_self="${BASH_SOURCE[0]:-${(%):-%x}}"
    RR_ROOT="$(cd "$(dirname "$_rr_self")" && pwd)"
    unset _rr_self
fi
export RR_ROOT

# --- external toolchains -----------------------------------------------------
# Defaults assume the simulators sit next to the checkout's parent directory.
_rr_parent="$(dirname "$RR_ROOT")"

export OMNETPP_ROOT="${OMNETPP_ROOT:-$_rr_parent/omnetpp-6.0.3}"
export INET_ROOT="${INET_ROOT:-$_rr_parent/inet4.5}"
export NS3_ROOT="${NS3_ROOT:-$_rr_parent/ns-3-dev}"

# Veins 5.3.1 is a dependency, downloaded separately; this repository ships only
# the study's modifications, applied into VEINS_ROOT by tools/setup.sh.
# A checkout placed in a Veins tree's own projects/ directory is detected first,
# since that is where Veins expects downstream projects to live.
if [ -z "${VEINS_ROOT:-}" ]; then
    if [ -f "$RR_ROOT/../../src/veins/veins.h" ]; then
        VEINS_ROOT="$(cd "$RR_ROOT/../.." && pwd)"
    else
        VEINS_ROOT="$_rr_parent/veins-veins-5.3.1"
    fi
fi
export VEINS_ROOT
export RR_VEINS_OVERLAY="${RR_VEINS_OVERLAY:-$RR_ROOT/03_veins/overlay}"

# --- derived paths -----------------------------------------------------------
# OMNeT working directory; all relative paths in omnetpp_inet.ini resolve here.
export PROJ="${PROJ:-$RR_ROOT/01_omnet_project}"
export OPP_RUN="${OPP_RUN:-$VEINS_ROOT/subprojects/veins_inet/bin/veins_inet_run}"
export LAUNCHD="${LAUNCHD:-$VEINS_ROOT/bin/veins_launchd}"

# ns-3 scenario directory, created by tools/setup.sh.
export NS3_SCRATCH="${NS3_SCRATCH:-$NS3_ROOT/scratch/propagation-compare}"

# SUMO FCD mobility traces exported for ns-3 replay.
export FCD_ROOT="${FCD_ROOT:-$PROJ/ns3_mobility}"

# Fitted coefficients loaded at runtime by VeinsInet5GVehicleApp.
export RR_COEFF_DIR="${RR_COEFF_DIR:-$PROJ/model_coefficients}"

# --- local overrides ---------------------------------------------------------
if [ -f "$RR_ROOT/env.local.sh" ]; then
    # shellcheck disable=SC1091
    . "$RR_ROOT/env.local.sh"
fi

# --- SUMO --------------------------------------------------------------------
# Resolved after env.local.sh so an explicit setting there wins outright.
#
# SUMO 1.22 is required: intersection.net.xml uses vClasses that earlier releases
# reject. A SUMO_HOME left over in the shell from an older install is an easy way
# to get a confusing failure, so a value is only trusted when it really holds a
# SUMO tree; otherwise it is re-derived from `sumo` on PATH. SUMO_HOME must point
# at the tree containing tools/ and data/, not merely a bin/ prefix — Veins and
# the TraCI Python bindings both read from it.
if [ -n "${SUMO_HOME:-}" ] && [ ! -x "${SUMO_HOME}/bin/sumo" ]; then
    echo "WARNING: ignoring SUMO_HOME='$SUMO_HOME' (no bin/sumo there)." >&2
    SUMO_HOME=""
fi
if [ -z "${SUMO_HOME:-}" ]; then
    _rr_sumo_bin="$(command -v sumo 2>/dev/null || true)"
    if [ -n "$_rr_sumo_bin" ]; then
        SUMO_HOME="$(cd "$(dirname "$_rr_sumo_bin")/.." && pwd)"
    else
        SUMO_HOME="$_rr_parent/sumo-1.22.0"
    fi
    unset _rr_sumo_bin
fi
export SUMO_HOME
case ":$PATH:" in
    *":$SUMO_HOME/bin:"*) ;;
    *) export PATH="$SUMO_HOME/bin:$PATH" ;;
esac

unset _rr_parent

# --- helper ------------------------------------------------------------------
# rr_require VAR [VAR ...]: check that each variable is set and its path exists.
rr_require() {
    local missing=0 v d
    for v in "$@"; do
        eval "d=\${$v:-}"
        if [ -z "$d" ]; then
            echo "ERROR: $v is not set. See README.md (Configuration)." >&2
            missing=1
        elif [ ! -e "$d" ]; then
            echo "ERROR: $v points to '$d', which does not exist." >&2
            echo "       Set it in env.local.sh or export it." >&2
            missing=1
        fi
    done
    [ "$missing" -eq 0 ] || return 1
}
