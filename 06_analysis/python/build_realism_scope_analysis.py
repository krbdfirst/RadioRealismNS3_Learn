#!/usr/bin/env python3
"""
Realism-scope analysis: what each OMNeT treatment is DESIGNED to model, and
whether that intended layer aligns with the corresponding *decoupled branch*
of NS-3 5G-LENA behaviour.

NS-3 reliability decomposes multiplicatively:
    PRR = P(reach decodable PHY slot)   # MAC: half-duplex + Mode-2 resource selection
        x P(SCI-2 control decode)       # PHY control
        x P(PSSCH data decode | SINR)   # channel + BLER  <-- the branch OMNeT models

Measured from --phyTraces re-runs (cav01/cav05, UMi_StreetCanyon = OMNeT
Analytical's exact channel).
"""
from __future__ import annotations
import os, sqlite3, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

from rr_paths import NS3_RESULTS, RESULTS_DIR
PHY = f"{NS3_RESULTS}/phy_decoupling"
OUT = f"{RESULTS_DIR}/prop_compare_figures"

def fetch(db, sql):
    con = sqlite3.connect(db)
    try: return con.execute(sql).fetchall()
    finally: con.close()

# ---- gather NS-3 decoupled branches (cav01 seeds, UMi) ----
sci, bler, prr, sinr_all = [], [], [], []
for s in range(5):
    db = f"{PHY}/cav01_s{s}-UMi_StreetCanyon-propagation-compare.db"
    if not os.path.exists(db): continue
    n, corr, scorr = fetch(db, "SELECT COUNT(*),SUM(psschCorrupt),SUM(sci2Corrupt) FROM psschRxUePhy")[0]
    bler.append(1 - corr / n); sci.append(1 - scorr / n)
    prr.append(fetch(db, "SELECT SUM(avrgPrr*numNieb)*1.0/NULLIF(SUM(numNieb),0) FROM avrgPrr WHERE range=300")[0][0])
    sinr_all.append(np.array(fetch(db, "SELECT avrgSinr,psschCorrupt FROM psschRxUePhy"), dtype=float))
P_bler = float(np.mean(bler)); P_sci = float(np.mean(sci)); P_prr = float(np.mean(prr))
P_reach = P_prr / (P_bler * P_sci)          # implied MAC reach-PHY factor
D = np.vstack(sinr_all)

# OMNeT channel-gate reception (PRR@300, same channel) measured earlier
OMNET = {"Analytical": 0.700, "Calibrated": 0.868, "Generalized": 0.750, "Plain": 1.000}

# ---------------------------------------------------------------- figure
fig, (axA, axB) = plt.subplots(1, 2, figsize=(15, 6))

# Panel A: NS-3 SINR->decode BLER waterfall + SINR histogram
edges = np.arange(-10, 35, 5)
cent = edges[:-1] + 2.5
dec = [1 - D[(D[:,0]>=lo)&(D[:,0]<lo+5), 1].mean() if ((D[:,0]>=lo)&(D[:,0]<lo+5)).sum()>20 else np.nan
       for lo in edges[:-1]]
ax2 = axA.twinx()
h,_ = np.histogram(D[:,0], bins=edges)
ax2.bar(cent, h/h.sum(), width=4.2, color="#cfe3f7", label="SINR distribution (share of receptions)")
ax2.set_ylabel("share of NS-3 receptions", color="#5a8fc7")
ax2.set_ylim(0, 0.75)
axA.plot(cent, dec, "-o", color="#000", lw=2.5, label="NS-3 PSSCH decode (BLER)")
axA.axvline(12, color="r", ls="--", lw=1); axA.text(12.3, 0.1, "~12 dB knee", color="r", fontsize=8)
axA.axhline(0.0, color="grey", lw=0.5)
axA.set_xlabel("per-reception SINR (dB)"); axA.set_ylabel("PSSCH data decode probability")
axA.set_ylim(0, 1.02); axA.set_zorder(ax2.get_zorder()+1); axA.patch.set_visible(False)
axA.set_title("(A) NS-3 channel branch = a real SINR→BLER waterfall\n"
              "most receptions sit below the ~12 dB knee → fail at the PHY")
axA.legend(loc="center right", fontsize=8); ax2.legend(loc="upper right", fontsize=8)
axA.grid(True, alpha=0.3)

# Panel B: reliability cascade — NS-3 (3 branches) vs OMNeT (1 channel gate)
stages = ["start\n(in-range\nneighbour)", "× reach PHY\n(MAC: half-duplex\n+ resource sel.)",
          "× SCI-2\ncontrol decode", "× PSSCH\nBLER decode", "= delivered\n(PRR@300)"]
ns3_vals = [1.0, P_reach, P_reach*P_sci, P_reach*P_sci*P_bler, P_prr]
x = np.arange(len(stages))
axB.plot(x, ns3_vals, "-s", color="#000", lw=2.5, ms=8, label="NS-3 5G-LENA (measured branches)")
for xi, v in zip(x, ns3_vals): axB.text(xi, v+0.03, f"{v:.2f}", ha="center", fontsize=9)
# OMNeT only has a channel gate then ~tiny SPS; mark where it acts
axB.axhline(OMNET["Analytical"], color="#ff7f0e", ls="--", lw=2, label="OMNeT Analytical channel gate (0.70)")
axB.axhline(OMNET["Generalized"], color="#d62728", ls=":", lw=2, label="OMNeT Generalized (BLER) gate (0.75)")
axB.fill_between([2.6, 3.4], 0, 1.02, color="orange", alpha=0.08)
axB.text(3.0, 0.93, "only THIS branch\nis in OMNeT's scope", ha="center", fontsize=8, color="#b25900")
axB.annotate("OMNeT models NONE\nof these two branches",
             xy=(1.5, 0.55), xytext=(1.5, 0.25), ha="center", fontsize=8, color="#444",
             arrowprops=dict(arrowstyle="->", color="#444"))
axB.set_xticks(x); axB.set_xticklabels(stages, fontsize=8)
axB.set_ylim(0, 1.05); axB.set_ylabel("cumulative reliability")
axB.set_title("(B) Reliability cascade (same UMi channel, cav01)\n"
              "OMNeT's intended layer = only the channel/BLER branch; the MAC + control branches are unmodelled")
axB.legend(loc="upper right", fontsize=8); axB.grid(True, axis="y", alpha=0.3)

fig.suptitle("What each model is FOR: OMNeT treatments are channel/PHY-reception models; "
             "NS-3's loss is mostly the Mode-2 MAC they don't model", y=1.0, fontsize=12)
fig.tight_layout(rect=(0,0,1,0.95))
p = f"{OUT}/fig9_realism_scope_cascade.png"; fig.savefig(p, dpi=160, bbox_inches="tight")
print("wrote", p)
print(f"\nNS-3 decoupled (cav01, UMi): reach-PHY≈{P_reach:.3f}  SCI≈{P_sci:.3f}  "
      f"BLER≈{P_bler:.3f}  → PRR≈{P_prr:.3f}")
print(f"OMNeT channel gate (same channel): Analytical 0.70 / Generalized 0.75 — "
      f"overshoots even NS-3's BLER branch ({P_bler:.2f}) and ignores reach-PHY+SCI")
