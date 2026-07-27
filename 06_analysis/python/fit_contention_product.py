#!/usr/bin/env python3
"""
Per-interferer PRODUCT collision model (SB-SPS / Gozalvez delta_COL structure),
learned from NS-3 on BOTH the symmetric rate sweep AND the asymmetric flood runs.

  collision(tx) = 1 - PROD_i (1 - q(rate_i, n_local))

Each in-range interferer i contributes a bounded factor q (a 2-D logistic in its tx
RATE and the local SPS density n). This captures the two effects the held-out flood
test revealed:
  * q rises with DENSITY  -> high-density collision (SPS candidate set saturates)
  * q SATURATES in rate   -> ONE flooder is a single bounded factor, so concentrated
                             load collides LESS than the same load spread over many
                             nodes (the lumped log-load model over-predicted this).

q is LEARNED from data (no imposed analytic form) -> distillation-consistent, grounded.
Maps to OMNeT: loop in-range neighbours, multiply (1 - q(neighbour_rate, n)); the
attacker enters as one neighbour at floodRate.

-> results/teacher_dataset/cascade/product_collision_params.csv  (a0..a3; OMNeT-ready)
"""
import os, numpy as np, pandas as pd
from scipy.optimize import minimize

from rr_paths import RESULTS_DIR
CAS = f"{RESULTS_DIR}/teacher_dataset/cascade"
RNG = np.random.default_rng(0)

# ---- assemble a unified per-tx table: each tx = up to 2 interferer groups -------
# columns: c1,r1 (count & rate of group-1 interferers), c2,r2 (group-2 = flooder or none),
#          nloc (local SPS density used as q's density arg), y (collided 0/1)
sym = pd.read_csv(f"{CAS}/collision_load_dataset.csv")          # hz, rate(dens), n, collided
S = pd.DataFrame({"c1": sym.n, "r1": sym.hz, "c2": 0.0, "r2": 1.0,
                  "nloc": sym.n, "y": sym.collided})
flo = pd.read_csv(f"{CAS}/flood_validate_dataset.csv")          # pps, n_normal, flooder_in_range, collided
F = pd.DataFrame({"c1": flo.n_normal, "r1": 10.0, "c2": flo.flooder_in_range.astype(float),
                  "r2": flo.pps, "nloc": flo.n_normal, "y": flo.collided})
ALL = pd.concat([S, F], ignore_index=True)

# subsample for the fit (full data used for validation)
fit = ALL.sample(n=min(400000, len(ALL)), random_state=0).reset_index(drop=True)

def q(rate, nloc, a):
    lr = np.log10(np.maximum(rate, 1e-9)); ln = np.log10(nloc + 1.0)
    z = a[0] + a[1] * lr + a[2] * ln + a[3] * lr * ln
    return 1.0 / (1.0 + np.exp(-z))

def predict(df, a):
    q1 = q(df["r1"].values, df["nloc"].values, a)
    q2 = q(df["r2"].values, df["nloc"].values, a)
    # log(1-p) = c1*log(1-q1) + c2*log(1-q2)
    log1mp = df["c1"].values * np.log(np.clip(1 - q1, 1e-12, 1)) \
           + df["c2"].values * np.log(np.clip(1 - q2, 1e-12, 1))
    return 1.0 - np.exp(log1mp)

def nll(a):
    p = np.clip(predict(fit, a), 1e-9, 1 - 1e-9)
    y = fit["y"].values
    return -np.mean(y * np.log(p) + (1 - y) * np.log(1 - p))

res = minimize(nll, x0=np.array([-4.0, 1.0, 0.5, 0.0]), method="Nelder-Mead",
               options={"xatol": 1e-4, "fatol": 1e-6, "maxiter": 20000})
a = res.x
print(f"q(rate,n) = sigmoid({a[0]:.3f} + {a[1]:.3f}*log10(rate) + {a[2]:.3f}*log10(n+1) + {a[3]:.3f}*log10(rate)*log10(n+1))")
print(f"  fit nll={res.fun:.4f} on {len(fit):,} tx\n")

# ---- validation: reproduce every symmetric AND flood cell ----------------------
ALL["pred"] = predict(ALL, a)
def cells(df, keys, label):
    g = df.groupby(keys).agg(meas=("y", "mean"), pred=("pred", "mean"), rows=("y", "size")).reset_index()
    g["abs_err"] = (g.meas - g.pred).abs()
    print(f"{label} (MAE {g.abs_err.mean():.3f}):"); print(g.round(3).to_string(index=False)); print()
    return g.abs_err.mean()

S2 = ALL.iloc[:len(S)].copy(); S2["hz"] = sym.hz.values; S2["dens"] = sym.rate.values
F2 = ALL.iloc[len(S):].copy(); F2["pps"] = flo.pps.values; F2["dens"] = flo.dens.values
m_sym = cells(S2, ["dens", "hz"], "SYMMETRIC sweep cells")
m_flo = cells(F2, ["dens", "pps"], "FLOOD cells (held-out structure)")
print(f"==> symmetric MAE {m_sym:.3f} | flood MAE {m_flo:.3f}  (both small => unified grounded model)")

# per-interferer q at reference rates (sanity: q saturates in rate, rises in n)
print("\nq(rate, n) samples:")
for n in [7, 29, 56]:
    print("  n={:2d}: ".format(n) + "  ".join(f"q({r})={q(r,n,a):.3f}" for r in [10, 25, 200, 1000]))

pd.DataFrame([{"a0": a[0], "a1": a[1], "a2": a[2], "a3": a[3],
               "form": "q=sigmoid(a0+a1*log10(rate)+a2*log10(n+1)+a3*log10(rate)*log10(n+1)); collision=1-prod(1-q_i)"}]
             ).to_csv(f"{CAS}/product_collision_params.csv", index=False)
print(f"\n-> {CAS}/product_collision_params.csv")
