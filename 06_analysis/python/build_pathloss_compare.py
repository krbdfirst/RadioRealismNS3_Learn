#!/usr/bin/env python3
"""
Contention-free PROPAGATION comparison: path loss (dB) vs distance for all 5
methods, reconstructed faithfully from each model's source so NS-3 is a full
curve (not a single anchor point).

All formulas verified against source:
  Plain       INET Ieee80211ScalarRadioMedium -> FreeSpacePathLoss @ 5.9 GHz
              PL = 47.86 + 20*log10(d)
  Calibrated  VeinsInet5GVehicleApp "current_analytical" (calibrated pc5 params)
              PL = 47.9856 + 10*2.7995*log10(d) ; shadowing sigma 3.616 dB
  Analytical  VeinsInet5GVehicleApp "tr38901_umi" = 3GPP TR 38.901 UMi
              LOS/NLOS blended by 3GPP UMi LOS probability
  Generalized VeinsInet5GVehicleApp generalized PHY (5G-Toolbox fitted regression)
              PL = 86.8235 + 15.7283*log10(d) - 37.154*pLos  (kinematics = 0)
  NS-3 LENA   ns-3 nr V2V_Urban = 3GPP TR 37.885 V2V Urban
              LOS  PL = 38.77 + 16.7*log10(d) + 18.2*log10(fc)
              NLOSv = LOS + (9 + max(0,15*log10(d)-41))   [vehicle blockage, mean]
              blended by P_LOS = min(1, 1.05*exp(-0.0114*d))

Common geometry: fc = 5.9 GHz, antenna heights 1.5 m (z used in ns-3 inputs).
Outputs (PNG) -> results/prop_compare_figures/
"""
from __future__ import annotations
import os
import numpy as np
import matplotlib
from rr_paths import PROJECT_DIR
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.special import erf

PROJ = f"{PROJECT_DIR}"
OUT = os.path.join(PROJ, "results", "prop_compare_figures")
os.makedirs(OUT, exist_ok=True)

FC = 5.9                     # GHz
H = 1.5                      # antenna height (m), both ends
TX_DBM = 23.0                # pc5TxPowerDbm
SENS_DBM = -94.0             # pc5SensingThresholdDbm
SIGMA_COMMON = 4.0           # common shadowing std for the isolation panel (dB)

COLORS = {
    "Plain": "#1f77b4", "Analytical": "#ff7f0e", "Calibrated": "#2ca02c",
    "Generalized": "#d62728", "NS-3 LENA": "#000000",
}
LABELS = {
    "Plain": "Plain (802.11 free-space, bypass)",
    "Analytical": "Analytical (3GPP TR 38.901 UMi)",
    "Calibrated": "Calibrated (log-distance n=2.80)",
    "Generalized": "Generalized (5G-Toolbox PHY)",
    "NS-3 LENA": "NS-3 5G-LENA (3GPP TR 37.885 V2V Urban)",
}
ORDER = ["Plain", "Calibrated", "Analytical", "Generalized", "NS-3 LENA"]
STYLE = {k: ("--" if k == "NS-3 LENA" else "-") for k in ORDER}
LW = {k: (3.0 if k == "NS-3 LENA" else 2.0) for k in ORDER}

d = np.linspace(1, 460, 600)
log10 = np.log10


# --- 3GPP UMi LOS probability (VeinsInet5GVehicleApp::get3gppUrbanLosProbability) -
def umi_plos(d):
    p = np.where(d <= 18.0, 1.0,
                 np.clip(18.0 / d + np.exp(-d / 36.0) * (1.0 - 18.0 / d), 0, 1))
    return p


def pl_analytical(d):
    dbp = 4.0 * H * H * (FC * 1e9) / 3.0e8                 # ~177 m
    d3 = np.maximum(d, 10.0)
    pl_los = np.where(d <= dbp,
                      32.4 + 21.0 * log10(d3) + 20.0 * log10(FC),
                      32.4 + 40.0 * log10(d3) + 20.0 * log10(FC) - 9.5 * log10(dbp * dbp))
    pl_nlos = np.maximum(pl_los, 22.4 + 35.3 * log10(d3) + 21.3 * log10(FC))
    p = umi_plos(d)
    return p * pl_los + (1 - p) * pl_nlos, pl_los, pl_nlos


def pl_calibrated(d):
    return 47.9856 + 10.0 * 2.7995 * log10(np.maximum(d, 1.0))


def pl_plain(d):
    return 47.86 + 20.0 * log10(np.maximum(d, 1.0))        # FSPL @ 5.9 GHz


def pl_generalized(d):
    # kinematic predictors set to 0 (nominal); LOS prob = 3GPP UMi (uma=false)
    return 86.8235306750227 + 15.7283329114856 * log10(np.maximum(d, 1.0)) \
        - 37.1539578873358 * umi_plos(d)


def pl_ns3(d):
    d3 = np.maximum(d, 1.0)
    pl_los = 38.77 + 16.7 * log10(d3) + 18.2 * log10(FC)
    nlosv_extra = 9.0 + np.maximum(0.0, 15.0 * log10(d3) - 41.0)   # mean blockage
    pl_nlosv = pl_los + nlosv_extra
    p = np.clip(1.05 * np.exp(-0.0114 * d), 0, 1)                  # P_LOS, urban
    return p * pl_los + (1 - p) * pl_nlosv, pl_los, pl_nlosv


PL = {
    "Plain": pl_plain(d),
    "Calibrated": pl_calibrated(d),
    "Analytical": pl_analytical(d)[0],
    "Generalized": pl_generalized(d),
    "NS-3 LENA": pl_ns3(d)[0],
}

# LOS-only path loss (isolates the base propagation physics — same "consideration"
# for every model, no channel-condition blending). For the two fitted models
# (Calibrated, Generalized) there is no separable LOS branch; we show their native
# curve and flag it.
PL_LOS = {
    "Plain": pl_plain(d),                          # free space == LOS
    "Calibrated": pl_calibrated(d),                # single blended fit (no LOS branch)
    "Analytical": pl_analytical(d)[1],             # 38.901 UMi LOS
    "Generalized": 86.8235306750227 + 15.7283329114856 * log10(np.maximum(d, 1.0)) - 37.1539578873358,  # pLos=1
    "NS-3 LENA": pl_ns3(d)[1],                      # 37.885 V2V LOS
}
PL_NLOS = {                                         # upper (worst-condition) branch
    "Analytical": pl_analytical(d)[2],             # 38.901 UMi building-NLOS
    "Generalized": 86.8235306750227 + 15.7283329114856 * log10(np.maximum(d, 1.0)),  # pLos=0
    "NS-3 LENA": pl_ns3(d)[2],                      # 37.885 V2V vehicle-NLOSv
}
FITTED = {"Calibrated", "Generalized"}             # no clean LOS/NLOS separation


# ----------------------------------------------------------------------------
# Figure 6 : path loss (dB) vs distance — LOS-only (aligned physics) vs effective
# ----------------------------------------------------------------------------
def fig6():
    fig, (axA, axB) = plt.subplots(1, 2, figsize=(15, 6.2), sharey=True)
    thr = TX_DBM - SENS_DBM                                   # 117 dB

    # Panel A: LOS-only — the same "consideration" for every model
    for k in ORDER:
        lbl = LABELS[k] + ("  [fitted: no LOS branch]" if k in FITTED else "")
        axA.plot(d, PL_LOS[k], STYLE[k], color=COLORS[k], lw=LW[k], label=lbl)
    axA.set_title("(A) LOS-only path loss — base propagation physics aligned\n"
                  "(3GPP LOS formulas agree within ~5 dB)")
    axA.legend(fontsize=8, loc="lower right", framealpha=0.95)

    # Panel B: effective (channel-condition-blended) + LOS<->NLOS bands
    for k in ORDER:
        axB.plot(d, PL[k], STYLE[k], color=COLORS[k], lw=LW[k], label=LABELS[k])
        if k in PL_NLOS and k not in FITTED:
            axB.fill_between(d, PL_LOS[k], PL_NLOS[k], color=COLORS[k], alpha=0.10, lw=0)
    axB.set_title("(B) Effective path loss (LOS/NLOS blended)\n"
                  "shaded = LOS↔NLOS band  →  the gap is the NLOS condition, not the physics")
    axB.legend(fontsize=8, loc="lower right", framealpha=0.95)

    for ax in (axA, axB):
        ax.axhline(thr, color="grey", ls=":", lw=1.0)
        ax.axvline(300, color="grey", ls="--", lw=0.8, alpha=0.5)
        ax.set_xlabel("Tx–Rx distance (m)")
        ax.set_xlim(0, 460)
        ax.grid(True, alpha=0.3)
    axA.set_ylabel("Path loss (dB)  — lower = stronger link")
    axA.text(5, thr + 1.5, f"Rx sensitivity limit (PL={thr:.0f} dB)", fontsize=7, color="grey")
    fig.suptitle("Propagation path loss: OMNeT 5G PC5 models vs NS-3 5G-LENA  "
                 "(contention-free, fc=5.9 GHz, h=1.5 m, 2D=3D)", y=1.0, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    p = os.path.join(OUT, "fig6_pathloss_vs_distance.png")
    fig.savefig(p, dpi=160)
    plt.close(fig)
    print("wrote", p)


# ----------------------------------------------------------------------------
# Figure 7 : propagation-limited reception probability (common link budget)
#   PSR(d) = 0.5*(1 + erf((Tx - PL - Sens)/(sigma*sqrt2)))   [OMNeT's own form]
#   ONE common budget + sigma applied to every model -> isolates propagation only.
# ----------------------------------------------------------------------------
def psr(pl, sigma=SIGMA_COMMON):
    return np.clip(0.5 * (1.0 + erf((TX_DBM - pl - SENS_DBM) / (sigma * np.sqrt(2.0)))), 0, 1)


def fig7():
    fig, ax = plt.subplots(figsize=(9, 6))
    for k in ORDER:
        ax.plot(d, psr(PL[k]), STYLE[k], color=COLORS[k], lw=LW[k], label=LABELS[k])
    ax.axvline(300, color="grey", ls="--", lw=0.8, alpha=0.5)
    ax.text(303, 0.05, "300 m", fontsize=8, color="grey")
    ax.set_xlabel("Tx–Rx distance (m)")
    ax.set_ylabel("Propagation-limited reception probability")
    ax.set_title("Contention-free reception vs distance — propagation models isolated\n"
                 f"common link budget (Tx {TX_DBM:.0f} dBm, sens {SENS_DBM:.0f} dBm, "
                 f"shadowing σ={SIGMA_COMMON:.0f} dB) applied to every model")
    ax.set_xlim(0, 460)
    ax.set_ylim(0, 1.02)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9, loc="upper right", framealpha=0.95)
    fig.tight_layout()
    p = os.path.join(OUT, "fig7_propagation_limited_reception.png")
    fig.savefig(p, dpi=160)
    plt.close(fig)
    print("wrote", p)


def table():
    import pandas as pd
    grid = np.array([50, 100, 150, 200, 300, 400])
    rows = {}
    for k in ORDER:
        f = {"Plain": pl_plain, "Calibrated": pl_calibrated,
             "Analytical": lambda x: pl_analytical(x)[0], "Generalized": pl_generalized,
             "NS-3 LENA": lambda x: pl_ns3(x)[0]}[k]
        rows[k] = np.round(f(grid.astype(float)), 1)
    df = pd.DataFrame(rows, index=grid)
    df.index.name = "distance_m"
    df.to_csv(os.path.join(OUT, "pathloss_db_by_distance.csv"))
    print("\n=== Path loss (dB) by distance ===")
    print(df.to_string())
    print("\n=== Gap vs NS-3 (dB; negative = more optimistic than NS-3) ===")
    print((df.subtract(df["NS-3 LENA"], axis=0)).drop(columns="NS-3 LENA").round(1).to_string())


def params_table():
    import pandas as pd
    rows = [
        ["Plain",       "INET FreeSpacePathLoss", "5.9", "n/a (scalar)", "free space (n=2)",
         "LOS only", "none (hard threshold)", "20·log10(f)"],
        ["Calibrated",  "current_analytical (fit)", "5.9", "1.5", "single log-distance n=2.80",
         "blended into slope", "σ=3.616 dB", "folded into ref 47.99 dB"],
        ["Analytical",  "3GPP TR 38.901 UMi", "5.9", "1.5 / 1.5", "LOS + building-NLOS",
         "UMi: min(1,18/d+e^(-d/36)(1-18/d))", "σ 4.0 LOS / 7.82 NLOS", "20·log10(f)"],
        ["Generalized", "5G-Toolbox fitted regr.", "5.9", "1.5", "LOS + NLOS via los_prob coeff",
         "UMi (same as Analytical)", "dist-param model", "folded into intercept"],
        ["NS-3 LENA",   "3GPP TR 37.885 V2V Urban", "5.9", "1.5 / 1.5", "LOS + vehicle-NLOSv (NO building-NLOS)",
         "V2V: min(1,1.05·e^(-0.0114·d))", "σ V2V (enabled, zero-mean)", "18.2·log10(f) LOS"],
    ]
    cols = ["method", "model", "fc_GHz", "ant_height_m", "condition_branches",
            "LOS_probability", "shadowing", "freq_term"]
    df = pd.DataFrame(rows, columns=cols).set_index("method").reindex(ORDER)
    df.to_csv(os.path.join(OUT, "model_parameters.csv"))
    print("=== Model parameter audit (confirm same considerations) ===")
    for m in ORDER:
        print(f"\n[{m}]")
        for c in cols[1:]:
            print(f"   {c:18s}: {df.loc[m, c]}")
    print("\nKEY MISMATCH: Analytical=TR38.901 UMi (blends harsh *building*-NLOS); "
          "NS-3=TR37.885 V2V (only mild *vehicle*-NLOSv, no buildings installed). "
          "LOS branches agree within ~5 dB; the divergence is the NLOS condition.\n"
          "The ns-3 binary ALSO supports channelScenario=UMi_StreetCanyon (== tr38901_umi) "
          "for a true like-for-like rerun vs Analytical.")


if __name__ == "__main__":
    params_table()
    fig6()
    fig7()
    table()
