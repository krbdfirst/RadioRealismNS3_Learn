#!/usr/bin/env bash
# =============================================================================
# setup.sh: connect this repository to a local simulator installation.
#
#   1. Check that OMNeT++, INET, Veins, ns-3 (5G-LENA) and SUMO are present, and
#      that the versions match the ones used for the study.
#   2. Copy the ns-3 scenario into $NS3_ROOT/scratch/propagation-compare.
#   3. Copy the Veins overlay (03_veins/overlay) into $VEINS_ROOT.
#   4. Run configure for Veins and veins_inet against the local INET/OMNeT++.
#   5. Print the build commands.
#
# Nothing is compiled here, and no file outside the reported paths is touched.
# Re-running is safe.
#
# NOTE ON $VEINS_ROOT: step 3 copies the overlay INTO the Veins tree. The overlay
# carries only the files this study publishes, so a tree that also holds
# unrelated local work will have those files replaced. Any file that differs is
# copied to a timestamped backup directory inside $VEINS_ROOT first, and the
# path is printed. Point VEINS_ROOT at a dedicated checkout to keep the two
# separate.
#
# Usage:
#   ./tools/setup.sh              # check, install scenario, configure
#   ./tools/setup.sh --check      # check prerequisites only
# =============================================================================
set -uo pipefail

RR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=../env.sh
. "$RR_ROOT/env.sh"

CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

# Versions used for the study.
WANT_OMNETPP="6.0.3"
WANT_INET="4.5"
WANT_VEINS="5.3.1"
WANT_NS3="3.42"
WANT_NR="v2x-1.1"
WANT_SUMO="1.22"

ok()   { printf '  \033[32mOK\033[0m   %s\n' "$1"; }
warn() { printf '  \033[33mWARN\033[0m %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m %s\n' "$1"; PROBLEMS=$((PROBLEMS+1)); }
PROBLEMS=0

echo "======================================================================"
echo "  RadioRealismNS3_Learn environment check"
echo "======================================================================"
echo "  RR_ROOT      = $RR_ROOT"
echo "  OMNETPP_ROOT = $OMNETPP_ROOT"
echo "  INET_ROOT    = $INET_ROOT"
echo "  NS3_ROOT     = $NS3_ROOT"
echo "  VEINS_ROOT   = $VEINS_ROOT"
echo "  overlay      = $RR_VEINS_OVERLAY"
echo "  SUMO_HOME    = $SUMO_HOME"
echo ""

# --- OMNeT++ -----------------------------------------------------------------
if [ -f "$OMNETPP_ROOT/setenv" ]; then
    v="$(cat "$OMNETPP_ROOT/Version" 2>/dev/null || echo unknown)"
    case "$v" in
        *"$WANT_OMNETPP"*) ok "OMNeT++ $v" ;;
        *) warn "OMNeT++ is '$v'; this study used $WANT_OMNETPP" ;;
    esac
else
    bad "OMNeT++ not found at \$OMNETPP_ROOT ($OMNETPP_ROOT/setenv missing)"
fi

# --- INET --------------------------------------------------------------------
if [ -d "$INET_ROOT/src/inet" ]; then
    v="$(cat "$INET_ROOT/Version" 2>/dev/null || echo unknown)"
    case "$v" in
        *"$WANT_INET"*) ok "INET $v" ;;
        *) warn "INET is '$v'; this study used $WANT_INET" ;;
    esac
else
    bad "INET not found at \$INET_ROOT ($INET_ROOT/src/inet missing)"
fi

# --- Veins -------------------------------------------------------------------
# Version lives in src/veins/veins.h as VEINS_VERSION_MAJOR/MINOR/PATCH.
VEINS_HDR="$VEINS_ROOT/src/veins/veins.h"
if [ -f "$VEINS_HDR" ]; then
    v="$(awk '/#define VEINS_VERSION_MAJOR/{a=$3}
              /#define VEINS_VERSION_MINOR/{b=$3}
              /#define VEINS_VERSION_PATCH/{c=$3}
              END{if(a!=""&&b!=""&&c!="")print a"."b"."c; else print "unknown"}' "$VEINS_HDR")"
    if [ "$v" = "$WANT_VEINS" ]; then
        ok "Veins $v"
    else
        warn "Veins is '$v'; this study used $WANT_VEINS (overlay assumes 5.3.1)"
    fi
    if [ ! -d "$RR_VEINS_OVERLAY" ]; then
        bad "overlay not found at \$RR_VEINS_OVERLAY ($RR_VEINS_OVERLAY)"
    fi
else
    bad "Veins not found at \$VEINS_ROOT ($VEINS_HDR missing).
         Download Veins $WANT_VEINS and set VEINS_ROOT; see 03_veins/README.md"
fi

# --- ns-3 / 5G-LENA NR-V2X ---------------------------------------------------
if [ -x "$NS3_ROOT/ns3" ]; then
    v="$(cat "$NS3_ROOT/VERSION" 2>/dev/null || echo unknown)"
    case "$v" in
        *"$WANT_NS3"*) ok "ns-3 $v" ;;
        *) warn "ns-3 is '$v'; this study used $WANT_NS3" ;;
    esac
    if [ -d "$NS3_ROOT/contrib/nr" ]; then
        ok "5G-LENA 'nr' module present (expected release $WANT_NR)"
    else
        bad "contrib/nr missing. The NR-V2X module is required; see 02_ns3_lena/README.md"
    fi
else
    bad "ns-3 not found at \$NS3_ROOT ($NS3_ROOT/ns3 missing)"
fi

# --- SUMO --------------------------------------------------------------------
if [ -x "$SUMO_HOME/bin/sumo" ]; then
    v="$("$SUMO_HOME/bin/sumo" --version 2>/dev/null | sed -n '1s/.*Version //p')"
    case "$v" in
        "$WANT_SUMO"*) ok "SUMO $v" ;;
        *) bad "SUMO is '$v'. SUMO $WANT_SUMO+ is required: the intersection network
         uses vClasses that older SUMO releases reject." ;;
    esac
else
    bad "SUMO not found at \$SUMO_HOME/bin/sumo"
fi

echo ""
if [ "$PROBLEMS" -gt 0 ]; then
    echo "$PROBLEMS prerequisite(s) missing. Set the paths in $RR_ROOT/env.local.sh"
    echo "(see README.md, Configuration) and re-run."
    exit 1
fi
echo "All prerequisites satisfied."

if [ "$CHECK_ONLY" -eq 1 ]; then
    exit 0
fi

# --- 1. install the ns-3 scenario -------------------------------------------
echo ""
echo "======================================================================"
echo "  Installing ns-3 scenario"
echo "======================================================================"
DEST="$NS3_ROOT/scratch/propagation-compare"
if [ -e "$DEST" ] && [ ! -f "$DEST/.rr_root" ]; then
    echo "  Refusing to overwrite $DEST. It exists and was not created by this"
    echo "  script. Move it aside and re-run."
    exit 1
fi
mkdir -p "$DEST"
cp "$RR_ROOT/02_ns3_lena/scratch/propagation-compare/"* "$DEST/"

# v2x-kpi.{cc,h} belong to the nr module and are used unmodified, so they are not
# redistributed in this repository. propagation-compare.cc includes v2x-kpi.h, so
# take them from the local nr checkout.
NR_EXAMPLES="$NS3_ROOT/contrib/nr/examples/nr-v2x-examples"
for f in v2x-kpi.cc v2x-kpi.h; do
    if [ -f "$NR_EXAMPLES/$f" ]; then
        cp "$NR_EXAMPLES/$f" "$DEST/"
    else
        echo "  ERROR: $NR_EXAMPLES/$f not found."
        echo "         propagation-compare.cc includes v2x-kpi.h; see 02_ns3_lena/README.md"
        exit 1
    fi
done

# Lets the installed run scripts locate env.sh.
printf '%s\n' "$RR_ROOT" > "$DEST/.rr_root"
echo "  installed -> $DEST (scenario + v2x-kpi from the nr module)"

# --- 2. apply the Veins overlay ----------------------------------------------
echo ""
echo "======================================================================"
echo "  Applying the Veins overlay"
echo "======================================================================"
echo "  from $RR_VEINS_OVERLAY"
echo "  into $VEINS_ROOT"
echo ""
# Report which files change before touching anything, so a re-run over a
# hand-edited Veins tree is not silent.
_n_new=0; _n_over=0; _n_same=0
while IFS= read -r rel; do
    src="$RR_VEINS_OVERLAY/$rel"
    dst="$VEINS_ROOT/$rel"
    if [ ! -e "$dst" ]; then
        _n_new=$((_n_new+1))
    elif cmp -s "$src" "$dst"; then
        _n_same=$((_n_same+1))
    else
        _n_over=$((_n_over+1))
        printf '  overwrite  %s\n' "$rel"
    fi
done < <(cd "$RR_VEINS_OVERLAY" && find . -type f | sed 's|^\./||' | sort)

# Back up anything that differs before overwriting it. A Veins tree that also
# holds unrelated local work would otherwise lose it silently, and the overlay
# carries only the files this study publishes.
if [ "$_n_over" -gt 0 ]; then
    BACKUP="$VEINS_ROOT/.rr_overlay_backup_$(date +%Y%m%d-%H%M%S)"
    while IFS= read -r rel; do
        src="$RR_VEINS_OVERLAY/$rel"; dst="$VEINS_ROOT/$rel"
        if [ -e "$dst" ] && ! cmp -s "$src" "$dst"; then
            mkdir -p "$BACKUP/$(dirname "$rel")"
            cp -p "$dst" "$BACKUP/$rel"
        fi
    done < <(cd "$RR_VEINS_OVERLAY" && find . -type f | sed 's|^\./||' | sort)
    echo ""
    echo "  $_n_over file(s) differ and will be replaced."
    echo "  Previous versions saved to:"
    echo "    $BACKUP"
    echo "  Restore with:  cp -R \"$BACKUP\"/. \"$VEINS_ROOT\"/"
    echo ""
fi

cp -R "$RR_VEINS_OVERLAY"/. "$VEINS_ROOT"/ \
    || { echo "  overlay copy FAILED"; exit 1; }
echo "  applied: $_n_new added, $_n_over overwritten, $_n_same already current"

# --- 3. configure Veins ------------------------------------------------------
echo ""
echo "======================================================================"
echo "  Configuring Veins against the local INET / OMNeT++"
echo "======================================================================"
# shellcheck disable=SC1091
. "$OMNETPP_ROOT/setenv" -q || true

( cd "$VEINS_ROOT" && ./configure ) \
    && echo "  veins core configured" \
    || { echo "  veins ./configure FAILED"; exit 1; }

( cd "$VEINS_ROOT/subprojects/veins_inet" && ./configure --with-inet="$INET_ROOT" ) \
    && echo "  veins_inet configured against $INET_ROOT" \
    || { echo "  veins_inet ./configure FAILED"; exit 1; }

# --- 4. next steps -----------------------------------------------------------
cat <<EOF

======================================================================
  Setup complete. Build next:
======================================================================

  # 1. INET (once, if not already built)
  cd "$INET_ROOT" && make makefiles && make -j\$(getconf _NPROCESSORS_ONLN) MODE=release

  # 2. Veins + veins_inet (the model code lives here)
  cd "$VEINS_ROOT" && make -j\$(getconf _NPROCESSORS_ONLN) MODE=release
  cd "$VEINS_ROOT/subprojects/veins_inet" && make -j\$(getconf _NPROCESSORS_ONLN) MODE=release

  # 3. ns-3 with the 5G-LENA NR module and this scenario
  cd "$NS3_ROOT" && ./ns3 configure --enable-examples && ./ns3 build

Then run a first simulation; see "Reproducing the study" in README.md.
EOF
