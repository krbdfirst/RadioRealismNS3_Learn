"""
Path definitions for the analysis scripts.

Each root is taken from an environment variable if set, otherwise from a default
derived from this file's location:

    RR_ROOT        repository root                (default: two levels up)
    RR_PROJECT_DIR OMNeT working directory        (default: $RR_ROOT/01_omnet_project)
    RR_RESULTS_DIR OMNeT results                  (default: $RR_PROJECT_DIR/results)
    NS3_ROOT       ns-3 installation              (default: next to $RR_ROOT)
    RR_NS3_SCRATCH ns-3 scenario directory        (default: $NS3_ROOT/scratch/propagation-compare)

These are the same variables env.sh sets, so a value exported for the run
scripts applies here too.
"""

import os
from pathlib import Path

_HERE = Path(__file__).resolve()

REPO_ROOT = Path(os.environ.get("RR_ROOT", _HERE.parents[2]))

PROJECT_DIR = Path(os.environ.get("RR_PROJECT_DIR",
                                  os.environ.get("PROJ", REPO_ROOT / "01_omnet_project")))

RESULTS_DIR = Path(os.environ.get("RR_RESULTS_DIR", PROJECT_DIR / "results"))

NS3_ROOT = Path(os.environ.get("NS3_ROOT", REPO_ROOT.parent / "ns-3-dev"))

NS3_SCRATCH = Path(os.environ.get("RR_NS3_SCRATCH",
                                  os.environ.get("NS3_SCRATCH",
                                                 NS3_ROOT / "scratch" / "propagation-compare")))

NS3_RESULTS = NS3_SCRATCH / "results"

# Fitted coefficients loaded at runtime by the OMNeT model.
COEFF_DIR = Path(os.environ.get("RR_COEFF_DIR", PROJECT_DIR / "model_coefficients"))

# SUMO FCD traces exported for ns-3 replay.
FCD_ROOT = Path(os.environ.get("FCD_ROOT", PROJECT_DIR / "ns3_mobility"))


def require(*paths):
    """Exit with a readable message if any of the given paths is missing."""
    missing = [str(p) for p in paths if not Path(p).exists()]
    if missing:
        raise SystemExit(
            "Missing required path(s):\n  " + "\n  ".join(missing) +
            "\n\nThese are either produced by the simulation runs or belong to an\n"
            "external toolchain. Set RR_RESULTS_DIR / NS3_ROOT as needed."
        )


__all__ = ["REPO_ROOT", "PROJECT_DIR", "RESULTS_DIR", "NS3_ROOT", "NS3_SCRATCH",
           "NS3_RESULTS", "COEFF_DIR", "FCD_ROOT", "require"]
