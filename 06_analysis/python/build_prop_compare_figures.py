#!/usr/bin/env python3
"""
Cross-validation figures: 4 OMNeT/Veins 5G-NR PC5 propagation treatments
(Plain, Analytical, Calibrated, Generalized) vs NS-3 5G-LENA, across CAV
adoption rates 1/5/25/50/75/100 %, 5 seeds each.

Comparable KPI = PRR within 300 m (Packet Reception Ratio):
  - NS-3 : avrgPrr table (computed @ range=300 m), weighted by neighbour count.
  - OMNeT: derived from distance-binned dist_pdr CSV -> sum(delivered)/sum(attempts)
           over all bins whose left edge < 300 m.
PRR is a normalised ratio, so the 60 s window used for cav75/cav100 (vs 200 s
for cav01-50) does not bias the comparison.

Outputs (PNG only) -> results/prop_compare_figures/
"""
from __future__ import annotations

import glob
import os
import re
import sqlite3
import sys

import numpy as np
import pandas as pd
import matplotlib
from rr_paths import NS3_RESULTS, PROJECT_DIR
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------
PROJ = f"{PROJECT_DIR}"
INET = os.path.join(PROJ, "results", "inet")
NS3 = f"{NS3_RESULTS}"
OUT = os.path.join(PROJ, "results", "prop_compare_figures")
os.makedirs(OUT, exist_ok=True)

RANGE_M = 300.0
RATES = [1, 5, 25, 50, 75, 100]
RATE_TAG = {1: "CAV01", 5: "CAV05", 25: "CAV25", 50: "CAV50", 75: "CAV75", 100: "CAV100"}
TREATMENTS = ["Plain", "Analytical", "Calibrated", "Generalized", "NS3Learn"]

# consistent, distinct colours; NS-3 = black reference
COLORS = {
    "Plain": "#1f77b4",
    "Analytical": "#ff7f0e",
    "Calibrated": "#2ca02c",
    "Generalized": "#d62728",
    "NS3Learn": "#9467bd",
    "NS-3 LENA": "#000000",
}
LABELS = {
    "Plain": "Plain (802.11 scalar, bypass)",
    "Analytical": "Analytical (TR 38.901 UMi)",
    "Calibrated": "Calibrated (current_analytical)",
    "Generalized": "Generalized (5G-Toolbox PHY)",
    "NS3Learn": "NS3-Learn (distilled from NS-3)",
    "NS-3 LENA": "NS-3 5G-LENA (reference)",
}


# ----------------------------------------------------------------------------
# OMNeT side
# ----------------------------------------------------------------------------
def load_omnet() -> pd.DataFrame:
    """One row per (treatment, rate, seed) with PRR@300m, plus the full
    distance-binned curve cached for figure 2."""
    rows = []
    curves = []  # long-form distance curves
    for treat in TREATMENTS:
        for rate in RATES:
            tag = f"Prop_{treat}_{RATE_TAG[rate]}"
            for seed in range(5):
                d = os.path.join(INET, f"{tag}-{seed}")
                csvs = sorted(glob.glob(os.path.join(d, "dist_pdr_*.csv")))
                if not csvs:
                    print(f"  WARN missing dist_pdr: {tag}-{seed}", file=sys.stderr)
                    continue
                df = pd.read_csv(csvs[-1])
                within = df[df["dist_bin_m"] < RANGE_M]
                att = within["attempts"].sum()
                prr = within["delivered"].sum() / att if att else np.nan
                rows.append(dict(treatment=treat, rate=rate, seed=seed, prr300=prr))
                # seed-level distance curve (+ drop causes for the breakdown figs)
                c = df[["dist_bin_m", "attempts", "delivered",
                        "drop_prop", "drop_sps", "drop_hd", "drop_cap"]].copy()
                c["treatment"] = treat
                c["rate"] = rate
                c["seed"] = seed
                curves.append(c)
    return pd.DataFrame(rows), pd.concat(curves, ignore_index=True)


# ----------------------------------------------------------------------------
# NS-3 side
# ----------------------------------------------------------------------------
def load_ns3() -> pd.DataFrame:
    rows = []
    for rate in RATES:
        tag = "cav%02d" % rate
        for seed in range(5):
            db = os.path.join(NS3, f"{tag}_s{seed}-V2V_Urban-propagation-compare.db")
            if not os.path.exists(db):
                print(f"  WARN missing NS-3 db: {tag}_s{seed}", file=sys.stderr)
                continue
            con = sqlite3.connect(db)
            try:
                # neighbour-count-weighted PRR@300m (NULL-safe)
                q = ("SELECT SUM(avrgPrr*numNieb)*1.0/NULLIF(SUM(numNieb),0), "
                     "AVG(avrgPrr) FROM avrgPrr WHERE range=?")
                w, simple = con.execute(q, (RANGE_M,)).fetchone()
            finally:
                con.close()
            prr = w if w is not None else simple
            rows.append(dict(rate=rate, seed=seed, prr300=prr))
    return pd.DataFrame(rows)


def agg(df: pd.DataFrame, by) -> pd.DataFrame:
    g = df.groupby(by)["prr300"]
    return pd.DataFrame({"mean": g.mean(), "std": g.std(ddof=0),
                         "lo": g.min(), "hi": g.max()}).reset_index()


# ----------------------------------------------------------------------------
# Figure 1 : PRR@300m vs adoption rate
# ----------------------------------------------------------------------------
def fig1(omnet, ns3):
    fig, ax = plt.subplots(figsize=(8, 5.5))
    x = np.array(RATES)
    for treat in TREATMENTS:
        a = agg(omnet[omnet.treatment == treat], "rate").set_index("rate").reindex(RATES)
        ax.fill_between(x, a["lo"], a["hi"], color=COLORS[treat], alpha=0.12, lw=0)
        ax.plot(x, a["mean"], "-o", color=COLORS[treat], lw=2, ms=5, label=LABELS[treat])
    n = agg(ns3, "rate").set_index("rate").reindex(RATES)
    ax.fill_between(x, n["lo"], n["hi"], color=COLORS["NS-3 LENA"], alpha=0.12, lw=0)
    ax.plot(x, n["mean"], "--s", color=COLORS["NS-3 LENA"], lw=2.5, ms=6,
            label=LABELS["NS-3 LENA"], zorder=5)

    ax.set_xscale("log")
    ax.set_xticks(RATES)
    ax.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ax.set_xlabel("CAV adoption rate (%)")
    ax.set_ylabel("PRR within 300 m")
    ax.set_ylim(0, 1.02)
    ax.set_title("Packet reception ratio vs CAV adoption\nOMNeT/Veins propagation treatments vs NS-3 5G-LENA")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8, loc="lower left", framealpha=0.95)
    fig.tight_layout()
    p = os.path.join(OUT, "fig1_prr300_vs_adoption.png")
    fig.savefig(p, dpi=160)
    plt.close(fig)
    print("wrote", p)


# ----------------------------------------------------------------------------
# Figure 2 : PDR vs distance, small multiples per rate
# ----------------------------------------------------------------------------
def fig2(curves, ns3):
    nmean = agg(ns3, "rate").set_index("rate")
    fig, axes = plt.subplots(2, 3, figsize=(13, 7.5), sharex=True, sharey=True)
    for ax, rate in zip(axes.ravel(), RATES):
        for treat in TREATMENTS:
            sub = curves[(curves.treatment == treat) & (curves.rate == rate)]
            # seed-pooled PDR per bin = sum(delivered)/sum(attempts)
            g = sub.groupby("dist_bin_m").agg(att=("attempts", "sum"),
                                              dlv=("delivered", "sum")).reset_index()
            g = g[g["att"] >= 30]  # drop sparse tail bins
            g["pdr"] = g["dlv"] / g["att"]
            ax.plot(g["dist_bin_m"], g["pdr"], "-", color=COLORS[treat], lw=1.8,
                    label=LABELS[treat])
        if rate in nmean.index:
            yv = nmean.loc[rate, "mean"]
            ax.scatter([RANGE_M], [yv], color="k", marker="s", s=55, zorder=6)
            ax.errorbar([RANGE_M], [yv],
                        yerr=[[yv - nmean.loc[rate, "lo"]], [nmean.loc[rate, "hi"] - yv]],
                        color="k", capsize=3, lw=1.5)
            ax.axhline(yv, color="k", ls=":", lw=1, alpha=0.6)
        ax.axvline(RANGE_M, color="grey", ls="--", lw=0.8, alpha=0.5)
        ax.set_title(f"{rate}% CAV", fontsize=10)
        ax.set_ylim(0, 1.02)
        ax.set_xlim(0, 460)
        ax.grid(True, alpha=0.3)
    for ax in axes[-1]:
        ax.set_xlabel("Tx–Rx distance (m)")
    for ax in axes[:, 0]:
        ax.set_ylabel("PDR")
    h, l = axes[0, 0].get_legend_handles_labels()
    h.append(plt.Line2D([], [], color="k", marker="s", ls=":", label="NS-3 LENA PRR@300 m"))
    l.append("NS-3 LENA PRR@300 m")
    fig.legend(h, l, fontsize=9, loc="lower center", ncol=5, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle("PDR vs distance by propagation treatment (NS-3 PRR@300 m anchored)", y=0.99)
    fig.tight_layout(rect=(0, 0.04, 1, 0.97))
    p = os.path.join(OUT, "fig2_pdr_vs_distance.png")
    fig.savefig(p, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


# ----------------------------------------------------------------------------
# Figure 3 : fidelity-to-NS3 ranking (mean abs error of PRR@300 across rates)
# ----------------------------------------------------------------------------
def fig3(omnet, ns3):
    om = agg(omnet, ["treatment", "rate"]).pivot(index="rate", columns="treatment", values="mean")
    n = agg(ns3, "rate").set_index("rate")["mean"]
    err = (om.subtract(n, axis=0)).abs()  # |treatment - ns3| per rate
    mae = err.mean(axis=0).reindex(TREATMENTS)
    bias = (om.subtract(n, axis=0)).mean(axis=0).reindex(TREATMENTS)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    cols = [COLORS[t] for t in TREATMENTS]
    ax1.bar(TREATMENTS, mae.values, color=cols)
    for i, v in enumerate(mae.values):
        ax1.text(i, v + 0.005, f"{v:.3f}", ha="center", fontsize=9)
    ax1.set_ylabel("Mean |PRR@300 − NS-3| across rates")
    ax1.set_title("Fidelity to NS-3 5G-LENA (lower = closer)")
    ax1.grid(True, axis="y", alpha=0.3)

    ax2.bar(TREATMENTS, bias.values, color=cols)
    ax2.axhline(0, color="k", lw=1)
    for i, v in enumerate(bias.values):
        ax2.text(i, v + (0.005 if v >= 0 else -0.02), f"{v:+.3f}", ha="center", fontsize=9)
    ax2.set_ylabel("Mean signed bias (treatment − NS-3)")
    ax2.set_title("Direction of deviation (+ optimistic / − pessimistic)")
    ax2.grid(True, axis="y", alpha=0.3)
    for ax in (ax1, ax2):
        ax.tick_params(axis="x", rotation=15)
    fig.tight_layout()
    p = os.path.join(OUT, "fig3_fidelity_ranking.png")
    fig.savefig(p, dpi=160)
    plt.close(fig)
    print("wrote", p)
    return mae, bias


# ----------------------------------------------------------------------------
# Figure 4 : loss-cause composition vs distance (where OMNeT's losses come from)
# ----------------------------------------------------------------------------
LOSS_TREATS = ["Analytical", "Calibrated", "Generalized"]  # Plain has zero loss
PANEL_RATES = [5, 50, 100]
LOSS_COLS = ["delivered", "drop_prop", "drop_sps"]  # drop_hd/cap/base are 0 in baseline
LOSS_FILL = {"delivered": "#bdbdbd", "drop_prop": "#d62728", "drop_sps": "#9467bd"}
LOSS_NAME = {"delivered": "Delivered", "drop_prop": "Lost – propagation",
             "drop_sps": "Lost – MAC (resource collision)"}


def fig4(curves, ns3):
    nmean = agg(ns3, "rate").set_index("rate")["mean"]
    fig, axes = plt.subplots(len(LOSS_TREATS), len(PANEL_RATES),
                             figsize=(12, 9), sharex=True, sharey=True)
    for i, treat in enumerate(LOSS_TREATS):
        for j, rate in enumerate(PANEL_RATES):
            ax = axes[i, j]
            sub = curves[(curves.treatment == treat) & (curves.rate == rate)]
            g = sub.groupby("dist_bin_m")[["attempts"] + LOSS_COLS].sum().reset_index()
            g = g[g["attempts"] >= 30]
            frac = g[LOSS_COLS].div(g["attempts"], axis=0)
            ax.stackplot(g["dist_bin_m"], [frac[c] for c in LOSS_COLS],
                         colors=[LOSS_FILL[c] for c in LOSS_COLS],
                         labels=[LOSS_NAME[c] for c in LOSS_COLS])
            if rate in nmean.index:  # NS-3 delivered fraction @300 m reference
                ax.scatter([RANGE_M], [nmean.loc[rate]], color="k", marker="s",
                           s=45, zorder=6)
                ax.axhline(nmean.loc[rate], color="k", ls=":", lw=1, alpha=0.7)
            ax.axvline(RANGE_M, color="white", ls="--", lw=0.8, alpha=0.7)
            ax.set_xlim(0, 460)
            ax.set_ylim(0, 1)
            if i == 0:
                ax.set_title(f"{rate}% CAV", fontsize=11)
            if j == 0:
                ax.set_ylabel(f"{treat}\nfraction of attempts", fontsize=10)
            if i == len(LOSS_TREATS) - 1:
                ax.set_xlabel("Tx–Rx distance (m)")
    h, l = axes[0, 0].get_legend_handles_labels()
    h.append(plt.Line2D([], [], color="k", marker="s", ls=":", label="NS-3 LENA delivered @300 m"))
    l.append("NS-3 LENA delivered @300 m")
    fig.legend(h, l, loc="lower center", ncol=4, fontsize=9, bbox_to_anchor=(0.5, -0.01))
    fig.suptitle("Loss-cause composition vs distance — OMNeT losses are propagation-driven, "
                 "not contention-driven\n(half-duplex / capacity drops are zero in this baseline)",
                 y=0.99, fontsize=12)
    fig.tight_layout(rect=(0, 0.04, 1, 0.96))
    p = os.path.join(OUT, "fig4_loss_composition_vs_distance.png")
    fig.savefig(p, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print("wrote", p)


# ----------------------------------------------------------------------------
# Figure 5 : MAC vs propagation loss share vs adoption (the punchline)
# ----------------------------------------------------------------------------
def fig5(curves, ns3):
    # per (treatment,rate): pooled loss fractions over ALL distances
    g = (curves.groupby(["treatment", "rate"])[["attempts", "drop_prop", "drop_sps"]]
         .sum().reset_index())
    g["prop_loss"] = g["drop_prop"] / g["attempts"]
    g["mac_loss"] = g["drop_sps"] / g["attempts"]
    nloss = 1.0 - agg(ns3, "rate").set_index("rate")["mean"]  # NS-3 total loss (1-PRR)

    fig, (axA, axB) = plt.subplots(1, 2, figsize=(13, 5.5))
    x = np.array(RATES)
    for treat in TREATMENTS:
        s = g[g.treatment == treat].set_index("rate").reindex(RATES)
        axA.plot(x, s["prop_loss"], "-o", color=COLORS[treat], lw=2, ms=4,
                 label=LABELS[treat])
        axB.plot(x, s["mac_loss"], "-o", color=COLORS[treat], lw=2, ms=4,
                 label=LABELS[treat])
    for ax in (axA, axB):
        ax.plot(x, nloss.reindex(RATES).values, "--s", color="k", lw=2.5, ms=6,
                label="NS-3 LENA total loss (1−PRR@300 m)", zorder=5)
        ax.set_xscale("log")
        ax.set_xticks(RATES)
        ax.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
        ax.set_xlabel("CAV adoption rate (%)")
        ax.grid(True, which="both", alpha=0.3)
        ax.set_ylim(0, 1.02)
    axA.set_ylabel("Loss fraction")
    axA.set_title("Propagation loss (drop_prop) vs NS-3 total loss")
    axB.set_title("MAC / resource-collision loss (drop_sps) vs NS-3 total loss")
    axA.legend(fontsize=8, loc="upper right", framealpha=0.95)
    fig.suptitle("Why the gap grows with congestion: NS-3 loss is MAC-contention-driven; "
                 "OMNeT MAC loss stays ~2 % and flat", y=1.0, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    p = os.path.join(OUT, "fig5_mac_vs_prop_loss.png")
    fig.savefig(p, dpi=160)
    plt.close(fig)
    print("wrote", p)
    return g


# ----------------------------------------------------------------------------
def main():
    print("Loading OMNeT ...")
    omnet, curves = load_omnet()
    print(f"  {len(omnet)} OMNeT runs")
    print("Loading NS-3 ...")
    ns3 = load_ns3()
    print(f"  {len(ns3)} NS-3 runs")

    fig1(omnet, ns3)
    fig2(curves, ns3)
    mae, bias = fig3(omnet, ns3)
    fig4(curves, ns3)
    loss = fig5(curves, ns3)
    loss.to_csv(os.path.join(OUT, "loss_breakdown_by_rate.csv"), index=False)

    # summary table
    tbl = agg(omnet, ["treatment", "rate"]).pivot(index="rate", columns="treatment", values="mean")
    tbl["NS-3 LENA"] = agg(ns3, "rate").set_index("rate")["mean"]
    tbl = tbl.reindex(RATES)
    tbl.to_csv(os.path.join(OUT, "prr300_mean_by_rate.csv"))
    omnet.to_csv(os.path.join(OUT, "omnet_prr300_per_run.csv"), index=False)
    ns3.to_csv(os.path.join(OUT, "ns3_prr300_per_run.csv"), index=False)

    print("\n=== PRR@300m (seed mean) by rate ===")
    print(tbl.round(3).to_string())
    print("\n=== Fidelity to NS-3 (MAE of PRR@300 across rates) ===")
    print(mae.round(4).to_string())
    print("\nBest match:", mae.idxmin(), f"(MAE={mae.min():.4f})")


if __name__ == "__main__":
    main()
