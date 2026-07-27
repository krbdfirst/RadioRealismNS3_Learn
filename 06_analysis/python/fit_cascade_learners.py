#!/usr/bin/env python3
"""
Three learners on the DECODE stage (SINR->decode), the data-rich cascade stage:
  1. LINEAR    regression on P(decode)   (simplest; unbounded form, baseline)
  2. LOGISTIC  regression (sigmoid in SINR; our grey-box functional form)
  3. GBM       GradientBoostingClassifier on (sinr_db, d, n)  (black box, finds residuals)
5-fold CV: log-loss, Brier, AUC, accuracy. Plus a calibration curve vs the empirical
binned decode-vs-SINR. Run for PSSCH decode (the binding waterfall) and overall decode.
-> results/prop_compare_figures/learners/decode_learner_comparison.png + metrics CSV
"""
import numpy as np, pandas as pd, os
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression, LogisticRegression
from sklearn.ensemble import GradientBoostingClassifier
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import log_loss, brier_score_loss, roc_auc_score, accuracy_score
from rr_paths import PROJECT_DIR
PROJ=f"{PROJECT_DIR}"
OUT=f"{PROJ}/results/prop_compare_figures/learners"; os.makedirs(OUT,exist_ok=True)
D=pd.read_csv(f"{PROJ}/results/teacher_dataset/cascade/decode_dataset.csv")

def evaluate(target):
    y=D[target].values.astype(int)
    Xs=D[["sinr_db"]].values                       # single physical feature
    Xg=D[["sinr_db","d","n"]].values               # GBM gets extras
    skf=StratifiedKFold(5,shuffle=True,random_state=0)
    res={m:{k:[] for k in("logloss","brier","auc","acc")} for m in("Linear","Logistic","GBM")}
    for tr,te in skf.split(Xs,y):
        # Linear regression on probability (clip to (eps,1-eps))
        lin=LinearRegression().fit(Xs[tr],y[tr]); pl=np.clip(lin.predict(Xs[te]),1e-4,1-1e-4)
        # Logistic regression (sigmoid in SINR) — add quadratic term for waterfall curvature
        Xs2=np.column_stack([Xs,Xs**2])
        log=LogisticRegression(max_iter=1000).fit(Xs2[tr],y[tr]); pg=log.predict_proba(Xs2[te])[:,1]
        # GBM
        gb=GradientBoostingClassifier(n_estimators=200,max_depth=3,learning_rate=0.05,subsample=0.8,random_state=0).fit(Xg[tr],y[tr])
        pb=gb.predict_proba(Xg[te])[:,1]
        for m,p in (("Linear",pl),("Logistic",pg),("GBM",pb)):
            res[m]["logloss"].append(log_loss(y[te],p,labels=[0,1]))
            res[m]["brier"].append(brier_score_loss(y[te],p))
            res[m]["auc"].append(roc_auc_score(y[te],p) if len(set(y[te]))>1 else np.nan)
            res[m]["acc"].append(accuracy_score(y[te],(p>0.5).astype(int)))
    rows=[]
    for m in res:
        rows.append(dict(target=target,model=m,**{k:float(np.nanmean(v)) for k,v in res[m].items()}))
    return pd.DataFrame(rows)

allres=pd.concat([evaluate("pssch_ok"),evaluate("decoded")],ignore_index=True)
allres.to_csv(f"{OUT}/decode_learner_metrics.csv",index=False)
print(allres.to_string(index=False))

# Calibration plot: predicted P(decode) vs SINR for each learner, overlaid on empirical binned
y=D["decoded"].values.astype(int); Xs=D[["sinr_db"]].values; Xg=D[["sinr_db","d","n"]].values
lin=LinearRegression().fit(Xs,y)
Xs2=np.column_stack([Xs,Xs**2]); log=LogisticRegression(max_iter=1000).fit(Xs2,y)
gb=GradientBoostingClassifier(n_estimators=200,max_depth=3,learning_rate=0.05,subsample=0.8,random_state=0).fit(Xg,y)
grid=np.linspace(-20,40,200).reshape(-1,1)
bins=np.arange(-20,42,3); D["sb"]=pd.cut(D.sinr_db,bins); emp=D.groupby("sb",observed=True).decoded.agg(["mean","count"])
ec=[(iv.left+iv.right)/2 for iv in emp.index]
fig,ax=plt.subplots(figsize=(9,6))
ax.scatter(ec,emp["mean"],s=np.clip(emp["count"]/30,8,200),c="k",alpha=.6,label="NS-3 empirical (binned)",zorder=5)
ax.plot(grid,np.clip(lin.predict(grid),0,1),"--",color="#1f77b4",lw=2,label="Linear regression")
ax.plot(grid,log.predict_proba(np.column_stack([grid,grid**2]))[:,1],"-",color="#ff7f0e",lw=2.4,label="Logistic (sigmoid, grey-box)")
gx=np.column_stack([grid,np.full((200,1),D.d.median()),np.full((200,1),D.n.median())])
ax.plot(grid,gb.predict_proba(gx)[:,1],"-.",color="#2ca02c",lw=2,label="GBM (black box)")
ax.set_xlabel("SINR (dB)"); ax.set_ylabel("P(decode)"); ax.set_ylim(-.05,1.05); ax.set_xlim(-20,40)
ax.set_title("Decode stage (SINR→PSSCH·SCI decode): three learners vs NS-3 empirical waterfall")
ax.legend(); ax.grid(alpha=.3); fig.tight_layout()
p=f"{OUT}/decode_learner_comparison.png"; fig.savefig(p,dpi=150); print("wrote",p)
