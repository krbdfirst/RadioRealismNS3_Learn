#!/usr/bin/env python3
"""
Fit the NS-3 teacher signals into surrogate functions OMNeT can evaluate at
runtime, and export them in the runtime-CSV format the app already consumes.

Outputs -> results/teacher_dataset/ns3_trained/:
  bler_curve_ns3_v2v.csv        snr_db,bler         (interpolateBlerCurve format)
  collision_vs_density.csv      neighbours,collision_frac  + fitted params
  harq_retx.csv                 mean blind-ReTx multiplier (channel-occupancy inflation)
  fit_diagnostics.png

NOTE: re-run after the teacher collection completes; mid/high rates (25/50/75/100)
anchor the collision curve, low rates (01/05/10) anchor the BLER waterfall.
"""
from __future__ import annotations
import os, numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

from rr_paths import RESULTS_DIR
DS = f"{RESULTS_DIR}/teacher_dataset"
OUT = os.path.join(DS, "ns3_trained"); os.makedirs(OUT, exist_ok=True)

agg = pd.read_csv(f"{DS}/teacher_run_aggregates.csv")
bler = pd.read_csv(f"{DS}/bler_curve_ns3.csv")

# ---- 1. BLER curve (channel/PHY branch): export V2V, monotonised ----
def export_bler(scn="V2V"):
    s = bler[bler.scenario == scn].sort_values("snr_db")
    if s.empty:
        s = bler.sort_values("snr_db")
    snr = s["snr_db"].to_numpy(); dec = s["pssch_decode"].to_numpy()
    dec = np.maximum.accumulate(dec)              # enforce monotone non-decreasing decode
    out = pd.DataFrame({"snr_db": snr, "bler": np.clip(1 - dec, 0, 1)})
    out.to_csv(f"{OUT}/bler_curve_ns3_v2v.csv", index=False)
    return snr, dec

# ---- 2. collision vs neighbour density (MAC contention) ----
def logistic(x, L, k, x0):  # saturating S-curve
    return L / (1 + np.exp(-k * (x - x0)))

def fit_collision():
    d = agg.dropna(subset=["mean_neighbours", "collision_frac"]).sort_values("mean_neighbours")
    x = d["mean_neighbours"].to_numpy(); y = d["collision_frac"].to_numpy()
    p0 = [0.95, 0.05, 40.0]
    try:
        popt, _ = curve_fit(logistic, x, y, p0=p0, maxfev=20000,
                            bounds=([0.5, 1e-3, 0], [1.0, 1.0, 500]))
    except Exception as e:
        print("collision fit failed:", e); popt = p0
    pd.DataFrame({"neighbours": x, "collision_frac": y,
                  "fit": logistic(x, *popt)}).to_csv(f"{OUT}/collision_vs_density.csv", index=False)
    pd.DataFrame([dict(L=popt[0], k=popt[1], x0=popt[2],
                       form="L/(1+exp(-k*(neighbours-x0)))")]).to_csv(f"{OUT}/collision_fit_params.csv", index=False)
    return x, y, popt

def main():
    snr, dec = export_bler("V2V")
    x, y, popt = fit_collision()
    harq = agg["harq_retx_mult"].mean()
    pd.DataFrame([dict(harq_retx_mult=harq)]).to_csv(f"{OUT}/harq_retx.csv", index=False)

    fig, ax = plt.subplots(1, 3, figsize=(16, 4.5))
    ax[0].plot(snr, dec, "-o", ms=3); ax[0].axvline(12, color="r", ls="--", lw=1)
    ax[0].set(title="NS-3 V2V BLER waterfall (channel branch)", xlabel="SINR (dB)",
              ylabel="PSSCH decode prob", ylim=(0, 1.02)); ax[0].grid(alpha=.3)
    xs = np.linspace(0, max(x.max(), 200), 200)
    ax[1].scatter(x, y, c="k", zorder=5, label="NS-3 runs")
    ax[1].plot(xs, logistic(xs, *popt), "r-", label=f"logistic fit\nL={popt[0]:.2f} k={popt[1]:.3f} x0={popt[2]:.0f}")
    ax[1].set(title="Resource-collision vs neighbour density (MAC)", xlabel="neighbours in range",
              ylabel="collision fraction", ylim=(0, 1.02)); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3)
    ax[2].scatter(agg["mean_neighbours"], agg["prr300"], c="k")
    ax[2].set(title="NS-3 PRR@300 vs density (target to reproduce)", xlabel="neighbours in range",
              ylabel="PRR@300", ylim=(0, 1.02)); ax[2].grid(alpha=.3)
    fig.suptitle(f"NS-3 teacher fits (preliminary; {len(agg)} runs, HARQ ReTx×{harq:.2f})", y=1.0)
    fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(f"{OUT}/fit_diagnostics.png", dpi=150)
    print("wrote", OUT)
    print(f"collision logistic: L={popt[0]:.3f} k={popt[1]:.4f} x0={popt[2]:.1f}; HARQ ReTx mult={harq:.2f}")
    print(f"BLER curve points={len(snr)}  (SINR {snr.min():.0f}..{snr.max():.0f} dB)")

if __name__ == "__main__":
    main()
