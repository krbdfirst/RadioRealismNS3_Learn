#!/usr/bin/env python3
"""
HYBRID collision model (preserves the validated C1 baseline, adds a grounded flood term):

    collision = 1 - (1 - base(n)) * (1 - q_a(floodRate, n))^[attacker in range]

  base(n) = C1 collision(density) = sigmoid(col_c0 + col_c1*n + col_c2*n^2)   [UNCHANGED]
  q_a(rate,n) = the attacker's per-interferer collision factor, LEARNED from the flood runs.

No attacker -> collision = base(n) EXACTLY -> zero regression to the C1/C2/C3 baseline.
Under flood -> the attacker enters as ONE bounded factor (saturates in rate => concentrated
load collides less than distributed, the effect the held-out test proved).

-> results/teacher_dataset/cascade/attacker_factor_params.csv  (qa_b0..b3; OMNeT-ready)
"""
import numpy as np, pandas as pd
from scipy.optimize import minimize

from rr_paths import PROJECT_DIR
ROOT = f"{PROJECT_DIR}"
CAS = f"{ROOT}/results/teacher_dataset/cascade"
# C1 baseline collision params (from ns3learn_runtime/cascade_params.csv) — held FIXED
C0, C1c, C2 = -2.81848, 0.17125, -0.0009273
def base(n): return 1.0 / (1.0 + np.exp(-(C0 + C1c * n + C2 * n * n)))

flo = pd.read_csv(f"{CAS}/flood_validate_dataset.csv")          # pps, n_normal, flooder_in_range, collided
fit = flo.sample(n=min(300000, len(flo)), random_state=0).reset_index(drop=True)
nb = fit.n_normal.values.astype(float)
pps = fit.pps.values.astype(float)
fin = fit.flooder_in_range.values.astype(float)
y = fit.collided.values.astype(float)
base_nb = base(nb)

def qa(rate, n, b):
    lr = np.log10(np.maximum(rate, 1e-9)); ln = np.log10(n + 1.0)
    return 1.0 / (1.0 + np.exp(-(b[0] + b[1] * lr + b[2] * ln + b[3] * lr * ln)))

def predict(nb, pps, fin, base_nb, b):
    return 1.0 - (1.0 - base_nb) * np.power(np.clip(1.0 - qa(pps, nb, b), 1e-12, 1.0), fin)

def nll(b):
    p = np.clip(predict(nb, pps, fin, base_nb, b), 1e-9, 1 - 1e-9)
    return -np.mean(y * np.log(p) + (1 - y) * np.log(1 - p))

res = minimize(nll, x0=np.array([-3.0, 1.5, 0.0, 0.0]), method="Nelder-Mead",
               options={"xatol": 1e-4, "fatol": 1e-6, "maxiter": 20000})
b = res.x
print(f"q_a(rate,n) = sigmoid({b[0]:.3f} + {b[1]:.3f}*log10(rate) + {b[2]:.3f}*log10(n+1) + {b[3]:.3f}*log10(rate)*log10(n+1))")
print(f"  fit nll={res.fun:.4f}\n")

# validate: hybrid vs measured on every flood cell
flo["pred"] = predict(flo.n_normal.values.astype(float), flo.pps.values.astype(float),
                      flo.flooder_in_range.values.astype(float), base(flo.n_normal.values.astype(float)), b)
g = flo.groupby(["dens", "pps"]).agg(n=("n_normal", "mean"), f_in=("flooder_in_range", "mean"),
                                     meas=("collided", "mean"), pred=("pred", "mean"),
                                     rows=("collided", "size")).reset_index()
g["abs_err"] = (g.meas - g.pred).abs()
print("HYBRID vs flood cells:"); print(g.round(3).to_string(index=False))
print(f"\n  flood MAE = {g.abs_err.mean():.3f}")

# baseline preservation check + the attacker factor curve
print("\nbaseline preserved (no attacker => base(n) exactly): "
      f"base(7)={base(7):.3f} base(29)={base(29):.3f} base(56)={base(56):.3f}")
print("attacker factor q_a (saturates in rate => one flooder is bounded):")
for n in [7, 29, 56]:
    print("  n={:2d}: ".format(n) + "  ".join(f"q_a({r})={qa(r,n,b):.3f}" for r in [200, 500, 1000]))

pd.DataFrame([{"qa_b0": b[0], "qa_b1": b[1], "qa_b2": b[2], "qa_b3": b[3],
               "form": "collision=1-(1-base(n))*(1-qa)^attacker; qa=sigmoid(b0+b1*log10(rate)+b2*log10(n+1)+b3*log10(rate)*log10(n+1))"}]
             ).to_csv(f"{CAS}/attacker_factor_params.csv", index=False)
print(f"\n-> {CAS}/attacker_factor_params.csv")

# ───────────────────── held-out generalization (does NOT overwrite production coeffs) ─────────────────────
# The production fit above uses a 300k sample and is validated on every cell; to make the
# "held-out" accuracy claim defensible we refit on a disjoint 50% TRAIN split and score the
# untouched 50% TEST split. With 4 params over millions of events the refit lands on the same
# coefficients (stability evidence); the TEST-cell MAE is the honest generalization number.
rng = np.random.default_rng(0)
mask = rng.random(len(flo)) < 0.5
tr, te = flo[mask], flo[~mask]
trn = tr.n_normal.values.astype(float); trp = tr.pps.values.astype(float)
trf = tr.flooder_in_range.values.astype(float); try_ = tr.collided.values.astype(float)
trbase = base(trn)
def nll_tr(bb):
    p = np.clip(predict(trn, trp, trf, trbase, bb), 1e-9, 1 - 1e-9)
    return -np.mean(try_ * np.log(p) + (1 - try_) * np.log(1 - p))
res_ho = minimize(nll_tr, x0=np.array([-3.0, 1.5, 0.0, 0.0]), method="Nelder-Mead",
                  options={"xatol": 1e-4, "fatol": 1e-6, "maxiter": 20000})
bho = res_ho.x
te = te.copy()
te["pred"] = predict(te.n_normal.values.astype(float), te.pps.values.astype(float),
                     te.flooder_in_range.values.astype(float), base(te.n_normal.values.astype(float)), bho)
gte = te.groupby(["dens", "pps"]).agg(n=("n_normal", "mean"), meas=("collided", "mean"),
                                      pred=("pred", "mean"), rows=("collided", "size")).reset_index()
gte["abs_err"] = (gte.meas - gte.pred).abs()
brier_te = float(np.mean((te.collided.values - te.pred.values) ** 2))
print("\n── held-out (50/50 train/test) ──")
print(f"  refit coeffs b_ho = [{bho[0]:.3f}, {bho[1]:.3f}, {bho[2]:.3f}, {bho[3]:.3f}]  (prod = [{b[0]:.3f}, {b[1]:.3f}, {b[2]:.3f}, {b[3]:.3f}])")
print(f"  TEST cell MAE = {gte.abs_err.mean():.3f}   TEST per-event Brier = {brier_te:.4f}")

# ───────────────────── learning visual (PNG only; Okabe-Ito CB-safe; 300 dpi) ─────────────────────
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams.update({
    "font.size": 11, "axes.labelsize": 12, "axes.titlesize": 12.5, "legend.fontsize": 9,
    "savefig.dpi": 300, "figure.dpi": 150, "savefig.bbox": "tight",
    "axes.grid": True, "grid.alpha": 0.22, "grid.linewidth": 0.5, "grid.color": "#C9C9C9",
    "axes.axisbelow": True, "axes.edgecolor": "#3A3A3A"})
DENSCOL = {5: "#1F6FB2", 25: "#E69F00", 50: "#C42E2E"}   # blue / orange / vermillion
brier_fit = float(np.mean((flo.collided.values - flo.pred.values) ** 2))

fig, (a, b2, c) = plt.subplots(1, 3, figsize=(14.5, 4.4))

# (a) accuracy: predicted vs measured collision per flood cell (full fit + held-out test)
a.plot([0, 1], [0, 1], "--", color="#888888", lw=1.3, zorder=1, label="ideal ($y=x$)")
for d in sorted(g.dens.unique()):
    sub = g[g.dens == d]
    a.scatter(sub.meas, sub.pred, s=90, color=DENSCOL.get(int(d), "#444"),
              edgecolor="#222", linewidth=0.6, zorder=3, label=f"dens={int(d)} (n≈{sub.n.mean():.0f})")
gte_sub = gte
a.scatter(gte_sub.meas, gte_sub.pred, s=38, facecolor="none", edgecolor="#333",
          linewidth=1.0, marker="s", zorder=2, label="held-out test cells")
a.set_xlim(0.35, 1.0); a.set_ylim(0.35, 1.0)
a.set_xlabel("measured P(collision)  [NS-3 flood teacher]")
a.set_ylabel("predicted P(collision)  [hybrid model]")
a.set_title("(a) Held-out accuracy")
a.text(0.04, 0.96, f"full-fit cell MAE = {g.abs_err.mean():.3f}\n"
                   f"held-out cell MAE = {gte.abs_err.mean():.3f}\n"
                   f"per-event Brier = {brier_fit:.3f}",
       transform=a.transAxes, va="top", ha="left", fontsize=9,
       bbox=dict(boxstyle="round,pad=0.4", fc="white", ec="#B5B5B5", alpha=0.92))
a.legend(loc="lower right", fontsize=8)

# (b) what was learned: q_a(rate, n) — driven by attacker RATE, modulated by density (the decoupling)
rates = np.logspace(np.log10(150), np.log10(1200), 200)
for n in [7, 29, 56]:
    b2.plot(rates, qa(rates, n, b), lw=2.2, label=f"n={n} normal CAVs")
b2.set_xscale("log"); b2.set_xlabel("attacker injection rate (pps)")
b2.set_ylabel("$q_a$  (per-flooder collision factor)")
b2.set_title("(b) Learned attacker factor $q_a$(rate, $n$)")
b2.minorticks_off()
b2.set_xticks([200, 300, 500, 700, 1000])
b2.xaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{int(v)}"))
b2.set_ylim(0.40, 0.62); b2.legend(loc="upper left", fontsize=8.5)
b2.text(0.5, 0.04, "saturates in rate → one flooder is bounded;\nload decoupled from density, modulated by $n$",
        transform=b2.transAxes, ha="center", va="bottom", fontsize=8.2, style="italic", color="0.35")

# (c) composition: base(n) preserved with no attacker; bounded lift when a flooder is in range
gn = np.arange(0, 61)
c.plot(gn, base(gn), color="#1A1A1A", lw=2.4, label="no attacker → base($n$) [C1, unchanged]")
for r, ls in [(200, "--"), (1000, ":")]:
    coll = 1.0 - (1.0 - base(gn)) * (1.0 - qa(r, gn, b))
    c.plot(gn, coll, ls, color="#C42E2E", lw=2.0, label=f"flooder in range @ {r} pps")
for d in sorted(g.dens.unique()):
    sub = g[g.dens == d]
    c.scatter(sub.n, sub.meas, s=70, color=DENSCOL.get(int(d), "#444"),
              edgecolor="#222", linewidth=0.6, zorder=3)
c.set_xlabel("normal CAV density $n$ (in range)")
c.set_ylabel("P(collision)")
c.set_title("(c) Baseline preserved + bounded attacker lift")
c.set_ylim(0, 1.02); c.legend(loc="lower right", fontsize=8)
c.text(0.04, 0.96, "markers: measured flood cells\n(blend of in/out-of-range receptions)",
       transform=c.transAxes, va="top", ha="left", fontsize=8, style="italic", color="0.35")

fig.suptitle("C2 attacker-load factor — learned from NS-3 asymmetric-flood teacher, decoupled from CAV density",
             y=1.02, fontsize=12.5)
fig.tight_layout()
for outp in [f"{ROOT}/results/prop_compare_figures/learners/attacker_factor_validation.png",
             f"{ROOT}/results/paper_figures/color/F7_attacker_factor.png"]:
    os.makedirs(os.path.dirname(outp), exist_ok=True)
    fig.savefig(outp)
    print(f"-> {outp}")
plt.close(fig)

# ───────────────────── PER-EVENT (un-binned) learning visual: F7b ─────────────────────
# F7 panel (a) shows only 6 aggregated cells. This figure validates the model at the granularity
# it is actually applied: ONE prediction per individual packet (collided 0/1). Per-event accuracy
# for a probabilistic classifier is a CALIBRATION question, so we use a reliability diagram + the
# outcome-conditioned predicted-probability distributions + a fine density-resolved curve.
pe_pred = flo.pred.values.astype(float)        # per-packet predicted P(collision) [production coeffs]
pe_y = flo.collided.values.astype(float)       # per-packet actual outcome (0/1)
N = len(pe_y)

# equal-frequency (quantile) bins of the predicted probability -> observed collision frequency
nbin = 15
edges = np.unique(np.quantile(pe_pred, np.linspace(0, 1, nbin + 1)))
idx = np.clip(np.digitize(pe_pred, edges[1:-1]), 0, len(edges) - 2)
cal_pred, cal_obs, cal_w, cal_lo, cal_hi = [], [], [], [], []
for k in range(len(edges) - 1):
    m = idx == k
    cnt = int(m.sum())
    if cnt < 50:
        continue
    p_hat = pe_y[m].mean()
    se = np.sqrt(max(p_hat * (1 - p_hat), 1e-12) / cnt)   # binomial SE on the observed frequency
    cal_pred.append(pe_pred[m].mean()); cal_obs.append(p_hat); cal_w.append(cnt)
    cal_lo.append(1.96 * se); cal_hi.append(1.96 * se)
cal_pred = np.array(cal_pred); cal_obs = np.array(cal_obs); cal_w = np.array(cal_w)
ece = float(np.sum(cal_w * np.abs(cal_obs - cal_pred)) / cal_w.sum())   # expected calibration error

figb, (pa, pb, pc) = plt.subplots(1, 3, figsize=(14.5, 4.4))

# (a) reliability diagram (per-event calibration): observed vs predicted, marker size ~ #packets
pa.plot([0, 1], [0, 1], "--", color="#888888", lw=1.3, label="perfectly calibrated")
pa.errorbar(cal_pred, cal_obs, yerr=[cal_lo, cal_hi], fmt="none", ecolor="#999", elinewidth=1, zorder=2)
pa.scatter(cal_pred, cal_obs, s=20 + 240 * cal_w / cal_w.max(), color="#1F6FB2",
           edgecolor="#222", linewidth=0.6, zorder=3)
pa.set_xlim(0, 1); pa.set_ylim(0, 1)
pa.set_xlabel("mean predicted P(collision) per bin")
pa.set_ylabel("observed collision frequency")
pa.set_title("(a) Reliability diagram (per-event)")
pa.text(0.04, 0.96, f"{N:,} packets\nper-event Brier = {brier_fit:.3f}\nECE = {ece:.3f}\n"
                    f"15 equal-frequency bins\n(marker ∝ #packets, bars 95% CI)",
        transform=pa.transAxes, va="top", ha="left", fontsize=8.5,
        bbox=dict(boxstyle="round,pad=0.4", fc="white", ec="#B5B5B5", alpha=0.92))
pa.legend(loc="lower right", fontsize=8.5)

# (b) outcome-conditioned predicted-probability distributions: does the model separate the classes?
bins = np.linspace(0, 1, 41)
pb.hist(pe_pred[pe_y == 0], bins=bins, density=True, color="#1F6FB2", alpha=0.55, label="actually NO collision")
pb.hist(pe_pred[pe_y == 1], bins=bins, density=True, color="#C42E2E", alpha=0.55, label="actually collided")
pb.set_xlabel("predicted P(collision) per packet")
pb.set_ylabel("density")
pb.set_title("(b) Predicted prob. by true outcome")
pb.legend(loc="upper center", fontsize=8.5)
pb.text(0.5, 0.6, "collided mass sits at higher\npredicted prob → model discriminates",
        transform=pb.transAxes, ha="center", va="center", fontsize=8.2, style="italic", color="0.35")

# (c) fine density-resolved accuracy (flooder in range): many local-n bins, not 3 coarse cells
inr = flo[flo.flooder_in_range == 1]
for r, cc, mk in [(200, "#1F6FB2", "o"), (1000, "#C42E2E", "s")]:
    sub = inr[inr.pps == r]
    nb_max = int(np.quantile(sub.n_normal, 0.99))
    be = np.arange(0, nb_max + 4, 4)               # width-4 local-density bins
    bi = np.digitize(sub.n_normal.values, be[1:-1])
    xs, ym, yp = [], [], []
    for k in range(len(be) - 1):
        mm = bi == k
        if mm.sum() < 200:
            continue
        xs.append(sub.n_normal.values[mm].mean())
        ym.append(sub.collided.values[mm].mean())
        yp.append(sub.pred.values[mm].mean())
    xs = np.array(xs)
    pc.scatter(xs, ym, s=42, color=cc, edgecolor="#222", linewidth=0.5, zorder=3,
               marker=mk, label=f"measured @ {r} pps")
    pc.plot(xs, yp, "-", color=cc, lw=2.0, zorder=2, label=f"predicted @ {r} pps")
pc.set_xlabel("local CAV density $n$ (width-4 bins)")
pc.set_ylabel("P(collision)")
pc.set_title("(c) Per-event accuracy vs local density")
pc.set_ylim(0, 1.02); pc.legend(loc="lower right", fontsize=8)

figb.suptitle("C2 attacker-load factor — PER-EVENT (un-binned) validation: calibration, separation, density resolution",
              y=1.02, fontsize=12.5)
figb.tight_layout()
for outp in [f"{ROOT}/results/prop_compare_figures/learners/attacker_factor_perevent.png",
             f"{ROOT}/results/paper_figures/color/F7b_attacker_factor_perevent.png"]:
    os.makedirs(os.path.dirname(outp), exist_ok=True)
    figb.savefig(outp)
    print(f"-> {outp}")
plt.close(figb)
print(f"\nper-event metrics: N={N:,}  Brier={brier_fit:.4f}  ECE={ece:.4f}")
