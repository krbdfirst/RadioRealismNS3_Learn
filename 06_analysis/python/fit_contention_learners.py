#!/usr/bin/env python3
"""
Three learners on the SB-SPS COLLISION stage (the dominant density-dependent contention),
then COMPOSE the full decomposed cascade and validate vs NS-3 per-instant PDR.

Collision stage: collided ~ (n, concurrency); learners linear / logistic / GBM (5-fold CV).
Cascade: PRR(d,n) = (1-hd(n)) * (1-collision(n)) * decode(SINR_noise(d)), each stage from its
own NS-3-derived labels (hd~const MAC, collision~MAC, decode~PHY waterfall). The gap to NS-3
per-instant quantifies the capture effect (overlaps that don't cause reception loss).
-> results/prop_compare_figures/learners/{collision_learner_comparison.png, cascade_validation.png}
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
DS=f"{PROJ}/results/teacher_dataset/cascade"
C=pd.read_csv(f"{DS}/collision_dataset.csv"); H=pd.read_csv(f"{DS}/halfduplex_by_rate.csv")
D=pd.read_csv(f"{DS}/decode_dataset.csv")

# ---- collision-stage 3-learner CV ----
y=C.collided.values.astype(int); X=C[["n","concurrency"]].values; Xn=C[["n"]].values
skf=StratifiedKFold(5,shuffle=True,random_state=0)
res={m:{k:[] for k in("logloss","brier","auc","acc")} for m in("Linear","Logistic","GBM")}
for tr,te in skf.split(X,y):
    lin=LinearRegression().fit(Xn[tr],y[tr]); pl=np.clip(lin.predict(Xn[te]),1e-4,1-1e-4)
    Xn2=np.column_stack([Xn,Xn**2]); lg=LogisticRegression(max_iter=1000).fit(Xn2[tr],y[tr]); pg=lg.predict_proba(Xn2[te])[:,1]
    gb=GradientBoostingClassifier(n_estimators=200,max_depth=3,learning_rate=0.05,subsample=0.8,random_state=0).fit(X[tr],y[tr]); pb=gb.predict_proba(X[te])[:,1]
    for m,p in (("Linear",pl),("Logistic",pg),("GBM",pb)):
        res[m]["logloss"].append(log_loss(y[te],p,labels=[0,1])); res[m]["brier"].append(brier_score_loss(y[te],p))
        res[m]["auc"].append(roc_auc_score(y[te],p)); res[m]["acc"].append(accuracy_score(y[te],(p>0.5).astype(int)))
met=pd.DataFrame([dict(stage="collision",model=m,**{k:float(np.mean(v)) for k,v in res[m].items()}) for m in res])
met.to_csv(f"{OUT}/collision_learner_metrics.csv",index=False); print(met.to_string(index=False))

# fit final collision learners on all data (function of n)
lin=LinearRegression().fit(Xn,y)
Xn2=np.column_stack([Xn,Xn**2]); lg=LogisticRegression(max_iter=1000).fit(Xn2,y)
gb=GradientBoostingClassifier(n_estimators=200,max_depth=3,learning_rate=0.05,subsample=0.8,random_state=0).fit(X,y)
gn=np.arange(0,70).reshape(-1,1)
emp=C.groupby("rate").agg(n=("n","mean"),col=("collided","mean"))
# representative concurrency per density n (for plotting the GBM curve)
conc_of_n=C.groupby(C.n.round().astype(int)).concurrency.mean()
conc_grid=np.array([conc_of_n.reindex(range(70)).interpolate(limit_direction="both").iloc[i] for i in range(70)])
fig,ax=plt.subplots(figsize=(8.5,5.5))
ax.scatter(emp.n,emp.col,s=90,c="k",zorder=5,label="NS-3 empirical (by rate)")
ax.plot(gn,np.clip(lin.predict(gn),0,1),"--",color="#1f77b4",lw=2,label="Linear")
ax.plot(gn,lg.predict_proba(np.column_stack([gn,gn**2]))[:,1],"-",color="#ff7f0e",lw=2.4,label="Logistic")
ax.plot(gn,gb.predict_proba(np.column_stack([gn.ravel(),conc_grid]))[:,1],"-.",color="#2ca02c",lw=2,label="GBM (n+concurrency)")
ax.set_xlabel("CAV neighbour density n"); ax.set_ylabel("P(SB-SPS collision)"); ax.set_ylim(0,1.02)
ax.set_title("SB-SPS collision stage: three learners vs NS-3 (resource-overlap fraction)")
ax.legend(); ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(f"{OUT}/collision_learner_comparison.png",dpi=150)
print("wrote collision_learner_comparison.png")

# ---- compose cascade & validate vs NS-3 per-instant ----
# decode waterfall (logistic, rate-independent)
yd=D.decoded.values.astype(int); Sd=np.column_stack([D[["sinr_db"]].values, D[["sinr_db"]].values**2])
dec=LogisticRegression(max_iter=1000).fit(Sd,yd)
def decode_p(sinr): s=np.atleast_1d(sinr); return dec.predict_proba(np.column_stack([s,s**2]))[:,1]
hd_block=H.hd_block_frac.mean()
FC,NF,BW,TX=5.9,7.0,18.72e6,23.0; NOISE=-174+10*np.log10(BW)+NF
def pl(d): d=np.maximum(d,10.0); return 32.4+21*np.log10(d)+20*np.log10(FC)
def coll_p(n,model):
    n=np.atleast_1d(n).astype(float)
    if model=="Linear": return np.clip(lin.predict(n.reshape(-1,1)),0,1)
    if model=="Logistic": return lg.predict_proba(np.column_stack([n,n**2]))[:,1]
    cc=np.array([conc_grid[int(min(max(round(v),0),69))] for v in n])
    return gb.predict_proba(np.column_stack([n,cc]))[:,1]
# NS-3 per-instant targets + densities
ref=pd.read_csv(f"{PROJ}/results/prop_compare_figures/perinstant_vs_avrgprr.csv")
densmap=C.groupby('rate').n.mean().to_dict()
print("\n=== composed cascade (no capture term) vs NS-3 per-instant ===")
print(f"{'rate':>5} {'NS-3':>6} {'cascade':>8} {'(1-hd)':>7} {'(1-coll)':>8} {'decode~':>8}")
for _,row in ref.iterrows():
    r=int(row.rate); n=densmap.get(r)
    if n is None: continue
    # decode at the per-link distances is folded into the per-instant via mean; approximate decode at typical in-range geometry
    # use NS-3 per-instant decode proxy = mean decode over uniform 0..300 weighted by area
    dd=np.linspace(5,300,60); w=dd  # area weight ~ d
    dec_mean=np.sum(w*decode_p(TX-pl(dd)-NOISE))/np.sum(w)
    cascade=(1-hd_block)*(1-coll_p(n,"Logistic"))[0]*dec_mean
    print(f"{r:>5} {row.ns3_perinstant:>6.3f} {cascade:>8.3f} {1-hd_block:>7.3f} {(1-coll_p(n,'Logistic'))[0]:>8.3f} {dec_mean:>8.3f}")
