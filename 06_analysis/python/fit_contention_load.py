#!/usr/bin/env python3
"""
Fit the LOAD-driven collision term for the flood/CBR extension.

Tests whether per-tx collision collapses onto a single 1-D curve in OFFERED LOAD
(= local CAV density n x CAM rate Hz), and fits  collision = sigmoid(c0 + c1*log10(load)).
This is the runtime-computable quantity in OMNeT (sum of in-range neighbour rates,
incl. an attacker's floodRate), so the fit maps straight into the app.

Consistency gate: at Hz=10 the fitted load curve must reproduce collision(density),
i.e. it must not regress the C1 (uniform-10Hz) behaviour.

-> results/teacher_dataset/cascade/load_collision_params.csv  (c0, c1; OMNeT-ready)
"""
import os, numpy as np, pandas as pd
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import log_loss, roc_auc_score

from rr_paths import PROJECT_DIR
PROJ = f"{PROJECT_DIR}"
CAS = f"{PROJ}/results/teacher_dataset/cascade"
df = pd.read_csv(f"{CAS}/collision_load_dataset.csv")
df = df[df.offered_load > 0].copy()
df["log_load"] = np.log10(df.offered_load)

X = df[["log_load"]].values
y = df.collided.values
clf = LogisticRegression(max_iter=1000)
clf.fit(X, y)
c1 = float(clf.coef_[0, 0]); c0 = float(clf.intercept_[0])
p = clf.predict_proba(X)[:, 1]
print(f"collision = sigmoid({c0:.4f} + {c1:.4f}*log10(offered_load))")
print(f"  fit on {len(df):,} tx | logloss={log_loss(y,p):.4f}  AUC={roc_auc_score(y,p):.4f}")

# Collapse check: predicted vs measured collided fraction per (density, rate) cell.
def sig(z): return 1.0 / (1.0 + np.exp(-z))
cells = df.groupby(["rate", "hz"]).agg(meas=("collided", "mean"),
                                       n=("n", "mean"), load=("offered_load", "mean")).reset_index()
cells["pred"] = sig(c0 + c1 * np.log10(cells.load))
cells["abs_err"] = (cells.pred - cells.meas).abs()
print("\nCOLLAPSE CHECK (single 1-D load curve vs every density x rate cell):")
print(cells.round(3).to_string(index=False))
print(f"  cell MAE = {cells.abs_err.mean():.3f}  (small => collision really is load-driven)")

# Consistency gate: Hz=10 slice -> collision(density). Compare to the C1 cascade collision
# logistic in n if its params are available; else just report the Hz=10 predicted curve.
print("\nCONSISTENCY GATE (Hz=10 reproduces collision vs density):")
hz10 = cells[cells.hz == 10]
for _, r in hz10.iterrows():
    print(f"  n={r.n:5.1f} (cav{int(r['rate']):02d}): load-model pred={r.pred:.3f}  measured={r.meas:.3f}")

pd.DataFrame([{"c0": c0, "c1": c1, "feature": "log10(offered_load)",
               "note": "collision=sigmoid(c0+c1*log10(n*Hz)); offered_load=sum in-range neighbour rates"}]
             ).to_csv(f"{CAS}/load_collision_params.csv", index=False)
print(f"\n-> {CAS}/load_collision_params.csv")
