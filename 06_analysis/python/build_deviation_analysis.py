#!/usr/bin/env python3
"""
Deviation analysis: Combined Analytical Reference (Prop_Analytical_M3) vs the NS-3-
distilled surrogate (Prop_NS3Learn) vs NS-3 ground truth, and WHERE the gap comes from.

FAIR attribution: the two OMNeT models route contention through different drop hooks
(M3 -> drop_sps; NS3Learn cascade -> drop_prop), so raw per-stage columns are not
comparable. We group each model's loss into two physically comparable buckets:
  - half-duplex        = drop_hd
  - contention+decode  = drop_prop + drop_sps + drop_base   (everything the medium/PHY does)
That makes the M3-vs-NS3Learn difference attributable to (a) half-duplex modeling and
(b) the joint contention/reception modeling, regardless of which hook each used.

NS-3 per-instant ground truth (all six rates) from results/deviation/ns3_groundtruth.csv.
"""
import csv, glob, os, collections
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
INET = os.path.join(HERE, "results", "inet")
OUT  = os.path.join(HERE, "results", "deviation"); os.makedirs(OUT, exist_ok=True)
RATES = [1, 5, 25, 50, 75, 100]

def agg_model(model):
    out = {}
    for rate in RATES:
        tot = collections.Counter()
        files = glob.glob(os.path.join(INET, f"Prop_{model}_CAV{rate:02d}-*", "dist_pdr_*.csv"))
        if not files:
            continue
        for f in files:
            for r in csv.DictReader(open(f)):
                for k in ["attempts", "delivered", "drop_hd", "drop_prop", "drop_sps", "drop_base"]:
                    try: tot[k] += float(r[k])
                    except (KeyError, ValueError): pass
        att = tot["attempts"]
        if att <= 0:
            continue
        out[rate] = {
            "pdr":  tot["delivered"] / att,
            "hd":   tot["drop_hd"] / att,                                   # half-duplex bucket
            "cont": (tot["drop_prop"] + tot["drop_sps"] + tot["drop_base"]) / att,  # contention+decode bucket
            "seeds": len(files),
        }
    return out

def load_ns3():
    g = {}
    p = os.path.join(OUT, "ns3_groundtruth.csv")
    if os.path.exists(p):
        for r in csv.DictReader(open(p)):
            g[int(float(r["rate"]))] = float(r["ns3_perinstant"])
    return g

m3, ns3l, ns3 = agg_model("Analytical_M3"), agg_model("NS3Learn"), load_ns3()

# ---- table ----
print(f"{'rate':>4} {'NS-3':>6} {'M3':>6} {'NS3L':>6} | {'M3-NS3':>7} {'NS3L-NS3':>8} | "
      f"{'M3 hd|cont':>13} {'NS3L hd|cont':>14}")
for rate in RATES:
    a, n = m3.get(rate), ns3l.get(rate); g = ns3.get(rate, float("nan"))
    if not a and not n: continue
    ap = a["pdr"] if a else float("nan"); npd = n["pdr"] if n else float("nan")
    m3b  = f"{a['hd']:.2f}|{a['cont']:.2f}" if a else "-"
    nsb  = f"{n['hd']:.2f}|{n['cont']:.2f}" if n else "-"
    print(f"{rate:>4} {g:>6.3f} {ap:>6.3f} {npd:>6.3f} | {ap-g:>+7.3f} {npd-g:>+8.3f} | {m3b:>13} {nsb:>14}")

# ---- figure ----
fig, ax = plt.subplots(1, 2, figsize=(13, 4.6))
rr = [r for r in RATES if r in m3 or r in ns3l]
ax[0].plot([r for r in rr if r in ns3],  [ns3[r] for r in rr if r in ns3],  "k-^", label="NS-3 (truth)")
ax[0].plot([r for r in rr if r in m3],   [m3[r]["pdr"]   for r in rr if r in m3],   "o-", color="#0072B2", label="Analytical M3")
ax[0].plot([r for r in rr if r in ns3l], [ns3l[r]["pdr"] for r in rr if r in ns3l], "s-", color="#D55E00", label="NS3Learn")
ax[0].set(title="Per-instant PDR vs adoption (6 rates)", xlabel="CAV rate (%)", ylabel="PDR", ylim=(0, 1.02)); ax[0].legend()

# fair buckets: half-duplex (solid) + contention/decode (hatched), M3 vs NS3Learn
x = np.arange(len(rr)); w = 0.38
for mod, off, lab in [(m3, -w/2, "M3"), (ns3l, +w/2, "NS3Learn")]:
    hd   = np.array([mod[r]["hd"]   if r in mod else 0 for r in rr])
    cont = np.array([mod[r]["cont"] if r in mod else 0 for r in rr])
    ax[1].bar(x+off, hd,   w, color="#56B4E9", edgecolor="white", label=(f"half-duplex" if off<0 else None))
    ax[1].bar(x+off, cont, w, bottom=hd, color="#CC79A7", edgecolor="white",
              hatch=("" if off<0 else "//"), label=(f"contention+decode" if off<0 else None))
ax[1].set(title="Loss buckets (left bar=M3, right hatched=NS3Learn)", xlabel="CAV rate (%)", ylabel="loss fraction")
ax[1].set_xticks(x); ax[1].set_xticklabels(rr); ax[1].legend()
fig.tight_layout(); fig.savefig(os.path.join(OUT, "deviation_analysis.png"), dpi=150)
print(f"\nfigure -> {os.path.join(OUT, 'deviation_analysis.png')}")
