#!/usr/bin/env python3
"""
Python proof-of-concept for the NS-3-distilled surrogate, BEFORE wiring it into
OMNeT C++. Tests whether the layered model can reproduce NS-3 PRR@300 vs density:

  per-link deliver(d) = (1 - hd) * (1 - collision(density))
                        * sci_decode(SINR_eff) * pssch_decode(SINR_eff)
  SINR_eff(d, density) = [23 dBm - PL_v2v(d) - noiseFloor] - I(density)
  PRR300 = E_d[ deliver(d) ]  over neighbours ~ uniform in disk r<=300m (pdf 2d/300^2)

Free params fitted to NS-3 PRR(density): hd (half-duplex, const) and the
interference penalty I(density)=a*log10(1+neighbours) (the SNR->SINR term).
collision(density), sci/pssch BLER curves come straight from NS-3 (no fitting).

If this matches, the C++ wiring is just implementation.
"""
from __future__ import annotations
import os, numpy as np, pandas as pd
from scipy.optimize import curve_fit
from scipy.interpolate import interp1d
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

from rr_paths import RESULTS_DIR
DS = f"{RESULTS_DIR}/teacher_dataset"
OUT = os.path.join(DS, "ns3_trained"); os.makedirs(OUT, exist_ok=True)
FC, NF, BW, TX = 5.9, 7.0, 18.72e6, 23.0
NOISE = -174.0 + 10*np.log10(BW) + NF                      # ~-94.3 dBm
log10 = np.log10

# --- NS-3 V2V path loss (TR 37.885), LOS/NLOSv blended (mean) ---
def pl_v2v(d):
    d = np.maximum(d, 1.0)
    los = 38.77 + 16.7*log10(d) + 18.2*log10(FC)
    nlosv = los + 9.0 + np.maximum(0.0, 15*log10(d) - 41.0)
    p = np.clip(1.05*np.exp(-0.0114*d), 0, 1)
    return p*los + (1-p)*nlosv

def sinr_of_d(d):
    return TX - pl_v2v(d) - NOISE

# --- NS-3 BLER curves (decode prob vs SINR) ---
b = pd.read_csv(f"{DS}/bler_curve_ns3.csv")
bv = b[b.scenario == "V2V"].sort_values("snr_db")
pssch = interp1d(bv.snr_db, np.maximum.accumulate(bv.pssch_decode.values),
                 bounds_error=False, fill_value=(0.0, bv.pssch_decode.max()))
sci = interp1d(bv.snr_db, np.maximum.accumulate(bv.sci_decode.values),
               bounds_error=False, fill_value=(0.0, bv.sci_decode.max()))

agg = pd.read_csv(f"{DS}/teacher_run_aggregates.csv").dropna(subset=["mean_neighbours", "prr300"])
g = agg.groupby("rate").agg(nb=("mean_neighbours", "mean"), coll=("collision_frac", "mean"),
                            prr=("prr300", "mean")).reset_index().sort_values("nb")
coll_of = interp1d(g.nb, g.coll, bounds_error=False, fill_value=(g.coll.min(), g.coll.max()))

# distance grid + uniform-disk weight (r<=300)
dd = np.linspace(1, 300, 120); w = 2*dd/(300.0**2); w /= w.sum()

def predict_prr(nb, hd, a, cap):
    Ipen = a*np.log10(1+nb)                    # interference penalty (dB)
    se = sinr_of_d(dd) - Ipen
    # capture: a fraction `cap` of resource collisions still decode (strongest survives)
    reach = (1 - coll_of(nb)) + cap*coll_of(nb)
    deliver = (1-hd)*reach*sci(se)*pssch(se)
    return float(np.sum(deliver*w))

def model(nb, hd, a, cap):
    return np.array([predict_prr(x, hd, a, cap) for x in nb])

popt, _ = curve_fit(model, g.nb.values, g.prr.values, p0=[0.3, 0.0, 0.1],
                    bounds=([0, 0, 0], [0.95, 40, 1.0]), maxfev=20000)
hd, a, cap = popt
g["prr_pred"] = model(g.nb.values, *popt)
g.to_csv(f"{OUT}/surrogate_validation.csv", index=False)
pd.DataFrame([dict(hd=hd, interference_a=a, capture=cap, form="reach=(1-coll)+cap*coll; I_db=a*log10(1+n)",
                   noiseFloor_dbm=NOISE)]).to_csv(f"{OUT}/surrogate_params.csv", index=False)

rmse = np.sqrt(np.mean((g.prr-g.prr_pred)**2))
fig, ax = plt.subplots(figsize=(7.5, 5.5))
ax.plot(g.nb, g.prr, "ks-", ms=8, label="NS-3 PRR@300 (target)")
ax.plot(g.nb, g.prr_pred, "r^--", ms=8, label="surrogate prediction")
for _, r in g.iterrows(): ax.annotate(f"cav{int(r['rate']):02d}", (r.nb, r.prr), fontsize=7, xytext=(3,4), textcoords="offset points")
ax.set(xlabel="neighbours in range", ylabel="PRR@300", ylim=(0,0.5),
       title=f"Surrogate vs NS-3 (POC)  hd={hd:.2f}  cap={cap:.2f}  RMSE={rmse:.3f}")
ax.legend(); ax.grid(alpha=.3); fig.tight_layout()
fig.savefig(f"{OUT}/surrogate_validation.png", dpi=150)
print(f"fitted: half-duplex hd={hd:.3f}  interference a={a:.2f}  capture={cap:.3f}  noiseFloor={NOISE:.1f} dBm")
print(f"RMSE(PRR) = {rmse:.4f} over {len(g)} rates")
print(g[["rate","nb","coll","prr","prr_pred"]].round(3).to_string(index=False))
