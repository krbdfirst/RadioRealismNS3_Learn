#!/usr/bin/env python3
"""
Focused comparison: Analytical vs Generalized vs NS3-Learn, against NS-3 5G-LENA.
(Plain and Calibrated intentionally excluded.)

Four charts -> results/prop_compare_figures/
  focus1_pdr_vs_distance.png    PDR vs Tx-Rx distance (small multiples per rate) + NS-3 PRR@300 anchor
  focus2_isolated_pathloss.png  contention-free path loss (dB) vs distance (the channel models)
  focus3_coupled_total_loss.png coupled total packet loss (1-PRR@300) vs CAV adoption (full sim, incl. MAC)
  focus4_deviation_from_ns3.png deviation of each model from NS-3 (per-rate signed + MAE bars)
"""
from __future__ import annotations
import glob, os, sqlite3
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

from rr_paths import NS3_RESULTS, PROJECT_DIR
PROJ = f"{PROJECT_DIR}"
INET = os.path.join(PROJ, "results", "inet")
NS3 = f"{NS3_RESULTS}"
OUT = os.path.join(PROJ, "results", "prop_compare_figures")
os.makedirs(OUT, exist_ok=True)

RANGE_M = 300.0
RATES = [1, 5, 25, 50, 75, 100]
RTAG = {1: "CAV01", 5: "CAV05", 25: "CAV25", 50: "CAV50", 75: "CAV75", 100: "CAV100"}
METHODS = ["Analytical", "Generalized", "NS3Learn"]
COLORS = {"Analytical": "#ff7f0e", "Generalized": "#d62728", "NS3Learn": "#9467bd", "NS-3 LENA": "#000000"}
LABELS = {"Analytical": "Analytical (3GPP TR 38.901 UMi)",
          "Generalized": "Generalized (5G-Toolbox PHY)",
          "NS3Learn": "NS3-Learn (distilled from NS-3)",
          "NS-3 LENA": "NS-3 5G-LENA (reference)"}

# ---------------------------------------------------------------- data: OMNeT
def omnet_curves():
    rows, curves = [], []
    for t in METHODS:
        for r in RATES:
            for s in range(5):
                cs = sorted(glob.glob(f"{INET}/Prop_{t}_{RTAG[r]}-{s}/dist_pdr_*.csv"))
                if not cs:
                    continue
                df = pd.read_csv(cs[-1]); w = df[df.dist_bin_m < RANGE_M]
                if w.attempts.sum():
                    rows.append(dict(method=t, rate=r, prr300=w.delivered.sum() / w.attempts.sum()))
                c = df[["dist_bin_m", "attempts", "delivered"]].copy()
                c["method"] = t; c["rate"] = r
                curves.append(c)
    return pd.DataFrame(rows), pd.concat(curves, ignore_index=True)

NS3_TEACHER = NS3 + "/teacher"
NS3_SCEN = "UMi_StreetCanyon_LoS"   # LOS-only reference (forced-LOS UMi == OMNeT Analytical-LOS)
def ns3_prr():
    out = {}
    for r in RATES:
        vals = []
        for s in range(5):
            db = f"{NS3_TEACHER}/cav{r:02d}_s{s}-{NS3_SCEN}-propagation-compare.db"
            if not os.path.exists(db):
                continue
            con = sqlite3.connect(db)
            try:
                w = con.execute("SELECT SUM(avrgPrr*numNieb)*1.0/NULLIF(SUM(numNieb),0) "
                                "FROM avrgPrr WHERE range=?", (RANGE_M,)).fetchone()[0]
            finally:
                con.close()
            if w is not None:
                vals.append(w)
        out[r] = np.mean(vals) if vals else np.nan
    return pd.Series(out)

# ---------------------------------------------------------------- path-loss models
FC, H = 5.9, 1.5
lg = np.log10
# LOS-ONLY path loss (matches the losOnly OMNeT runs + NS-3 UMi_StreetCanyon_LoS reference).
def pl_analytical(d):                       # 3GPP TR 38.901 UMi LOS
    dbp = 4*H*H*(FC*1e9)/3e8; d3 = np.maximum(d, 10)
    return np.where(d <= dbp, 32.4+21*lg(d3)+20*lg(FC), 32.4+40*lg(d3)+20*lg(FC)-9.5*lg(dbp*dbp))
def pl_generalized(d):                      # 5G-Toolbox LOS (los_probability = 1)
    return 86.8235306750227 + 15.7283329114856*lg(np.maximum(d, 1)) - 37.1539578873358
def pl_ns3(d):                              # NS3-Learn / NS-3 LOS = UMi LOS (== Analytical)
    return pl_analytical(d)
PL = {"Analytical": pl_analytical, "Generalized": pl_generalized, "NS3Learn": pl_ns3}

# ---------------------------------------------------------------- figures
def fig_pdr(curves, ns3):
    fig, axes = plt.subplots(2, 3, figsize=(13, 7.5), sharex=True, sharey=True)
    for ax, r in zip(axes.ravel(), RATES):
        for t in METHODS:
            g = (curves[(curves.method == t) & (curves.rate == r)]
                 .groupby("dist_bin_m").agg(a=("attempts", "sum"), d=("delivered", "sum")).reset_index())
            g = g[g.a >= 30]
            ax.plot(g.dist_bin_m, g.d/g.a, "-", color=COLORS[t], lw=1.9, label=LABELS[t])
        if r in ns3.index:
            ax.scatter([RANGE_M], [ns3[r]], color="k", marker="s", s=55, zorder=6)
            ax.axhline(ns3[r], color="k", ls=":", lw=1, alpha=.6)
        ax.axvline(RANGE_M, color="grey", ls="--", lw=.8, alpha=.5)
        ax.set_title(f"{r}% CAV", fontsize=10); ax.set_ylim(0, 1.02); ax.set_xlim(0, 460); ax.grid(alpha=.3)
    for ax in axes[-1]: ax.set_xlabel("Tx–Rx distance (m)")
    for ax in axes[:, 0]: ax.set_ylabel("PDR")
    h, l = axes[0, 0].get_legend_handles_labels()
    h.append(plt.Line2D([], [], color="k", marker="s", ls=":", label="NS-3 LENA PRR@300 m")); l.append("NS-3 LENA PRR@300 m")
    fig.legend(h, l, loc="lower center", ncol=4, fontsize=9, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle("PDR vs distance — Analytical vs Generalized vs NS3-Learn (NS-3 PRR@300 anchored)", y=0.99)
    fig.tight_layout(rect=(0, 0.04, 1, 0.97))
    p = f"{OUT}/focus1_pdr_vs_distance.png"; fig.savefig(p, dpi=160, bbox_inches="tight"); plt.close(fig); print("wrote", p)

def fig_pathloss():
    d = np.linspace(1, 460, 600)
    fig, ax = plt.subplots(figsize=(9, 6))
    for t in METHODS:
        ax.plot(d, PL[t](d), "-", color=COLORS[t], lw=2.2, label=LABELS[t])
    ax.plot(d, pl_ns3(d), "--", color="k", lw=2.0, label=LABELS["NS-3 LENA"] + " (= NS3-Learn channel)")
    ax.axvline(300, color="grey", ls="--", lw=.8, alpha=.5); ax.text(303, 60, "300 m", fontsize=8, color="grey")
    ax.set_xlabel("Tx–Rx distance (m)"); ax.set_ylabel("Path loss (dB) — lower = stronger link")
    ax.set_title("Isolated propagation path loss (contention-free)\nAnalytical vs Generalized vs NS3-Learn vs NS-3, fc=5.9 GHz, h=1.5 m")
    ax.set_xlim(0, 460); ax.grid(alpha=.3); ax.legend(fontsize=9, loc="lower right")
    fig.tight_layout(); p = f"{OUT}/focus2_isolated_pathloss.png"; fig.savefig(p, dpi=160); plt.close(fig); print("wrote", p)

def fig_coupled(om, ns3):
    g = om.groupby(["method", "rate"]).prr300.mean().unstack(0)
    fig, ax = plt.subplots(figsize=(9, 6)); x = np.array(RATES)
    for t in METHODS:
        ax.plot(x, 1 - g[t].reindex(RATES).values, "-o", color=COLORS[t], lw=2.2, ms=6, label=LABELS[t])
    ax.plot(x, 1 - ns3.reindex(RATES).values, "--s", color="k", lw=2.6, ms=7, label=LABELS["NS-3 LENA"], zorder=5)
    ax.set_xscale("log"); ax.set_xticks(RATES); ax.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ax.set_xlabel("CAV adoption rate (%)"); ax.set_ylabel("Coupled total loss  (1 − PRR@300 m)")
    ax.set_title("Coupled total packet loss vs CAV adoption (full simulation, channel + MAC)\nhigher = worse; NS3-Learn tracks NS-3's congestion collapse")
    ax.set_ylim(0, 1.02); ax.grid(True, which="both", alpha=.3); ax.legend(fontsize=9, loc="center right")
    fig.tight_layout(); p = f"{OUT}/focus3_coupled_total_loss.png"; fig.savefig(p, dpi=160); plt.close(fig); print("wrote", p)
    return g

def fig_deviation(g, ns3):
    dev = g[METHODS].subtract(ns3, axis=0)          # signed: model PRR - NS-3 PRR
    mae = dev.abs().mean()
    fig, (axA, axB) = plt.subplots(1, 2, figsize=(13, 5.2)); x = np.array(RATES)
    for t in METHODS:
        axA.plot(x, dev[t].reindex(RATES).values, "-o", color=COLORS[t], lw=2, ms=5, label=LABELS[t])
    axA.axhline(0, color="k", lw=1.2)
    axA.set_xscale("log"); axA.set_xticks(RATES); axA.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    axA.set_xlabel("CAV adoption rate (%)"); axA.set_ylabel("PRR@300 − NS-3   (+ optimistic / − pessimistic)")
    axA.set_title("Per-rate deviation from NS-3 5G-LENA"); axA.grid(True, which="both", alpha=.3); axA.legend(fontsize=8)
    cols = [COLORS[t] for t in METHODS]
    axB.bar(METHODS, mae.reindex(METHODS).values, color=cols)
    for i, v in enumerate(mae.reindex(METHODS).values): axB.text(i, v+0.005, f"{v:.3f}", ha="center", fontsize=10)
    axB.set_ylabel("Mean |PRR@300 − NS-3| across rates"); axB.set_title("Overall fidelity to NS-3 (lower = closer)")
    axB.grid(True, axis="y", alpha=.3)
    fig.suptitle("Deviation from NS-3 5G-LENA — Analytical vs Generalized vs NS3-Learn", y=1.0, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.95)); p = f"{OUT}/focus4_deviation_from_ns3.png"; fig.savefig(p, dpi=160); plt.close(fig); print("wrote", p)
    return mae

def main():
    om, curves = omnet_curves(); ns3 = ns3_prr()
    fig_pdr(curves, ns3)
    fig_pathloss()
    g = fig_coupled(om, ns3)
    mae = fig_deviation(g, ns3)
    print("\n=== PRR@300 (mean) by rate ===")
    tbl = g[METHODS].copy(); tbl["NS-3 LENA"] = ns3; print(tbl.reindex(RATES).round(3).to_string())
    print("\n=== Fidelity to NS-3 (MAE of PRR@300) ==="); print(mae.reindex(METHODS).round(4).to_string())

if __name__ == "__main__":
    main()
