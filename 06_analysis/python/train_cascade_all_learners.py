#!/usr/bin/env python3
"""
UNIFIED, NS-3-GROUNDED training of the decomposed PC5 cascade. Every stage is LEARNED from the
NS-3 5G-LENA teacher data — no hand-derived analytics:
  half-duplex hd(n)   <- halfduplex_dataset.csv  (MAC slot coincidence; feature = density n)
  collision  coll(n)  <- collision_dataset.csv   (MAC RB overlap;       feature = density n)
  decode     dec(SINR)<- decode_dataset.csv      (PHY SCI*PSSCH;        feature = SINR dB)
  capture    I_coll~Normal(mu,sigma) fitted to NS-3 distance-resolved PDR.
Three learners (Linear / Logistic / GBM) on the SAME feature per stage (fair form comparison),
composed PDR(d,n)=(1-hd(n))*[(1-coll(n))*dec(SINRn) + coll(n)*E_Ic dec(SINRn-Ic)], evaluated
per-instant on intersection (train) + mainstreet (validate). Logistic coeffs exported to OMNeT.
-> results/teacher_dataset/ns3learn_runtime/cascade_params.csv (logistic, adopted)
-> results/prop_compare_figures/perinstant/{learner_stage_metrics.csv, c1_bakeoff_table.csv}
"""
import sqlite3, glob, os, numpy as np, pandas as pd
from scipy.optimize import minimize
from sklearn.linear_model import LinearRegression, LogisticRegression
from sklearn.ensemble import GradientBoostingClassifier
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import log_loss, brier_score_loss, roc_auc_score
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
SCEN=os.environ.get("SCEN","V2V_Urban")
PROJ=f"{PROJECT_DIR}"
DS=f"{PROJ}/results/teacher_dataset/cascade"; B=f"{PROJ}/results/teacher_dataset/ns3learn_runtime"
OUT=f"{PROJ}/results/prop_compare_figures/perinstant"; os.makedirs(OUT,exist_ok=True)
FC,NF,BW,TX=5.9,7.0,18.72e6,23.0; NOISE=-174+10*np.log10(BW)+NF; BINS=np.arange(0,320,20)
# path loss must MATCH the OMNeT channel used at runtime. V2V_Urban -> TR 37.885 mean blend
# (identical to ns3V2vPathLossDb); UMi -> TR 38.901 UMi LOS.
def pl_umi(d): return 32.4+21*np.log10(np.maximum(d,10.0))+20*np.log10(FC)
def pl_los(d): d=np.maximum(d,1.0); return 38.77+16.7*np.log10(d)+18.2*np.log10(FC)
def pl_nlosv(d): d=np.maximum(d,1.0); return pl_los(d)+9.0+np.maximum(0.0,15.0*np.log10(d)-41.0)
def p_los(d): return np.minimum(1.0,1.05*np.exp(-0.0114*np.maximum(d,1.0)))   # TR 37.885 LOS prob
def sinr_of(plf,d): return TX-plf(d)-NOISE
gh_x,gh_w=np.polynomial.hermite_e.hermegauss(15); gh_w=gh_w/np.sqrt(2*np.pi)

def capn(df, n=120000):
    return df.sample(n=n, random_state=0).reset_index(drop=True) if len(df) > n else df
HD=capn(pd.read_csv(f"{DS}/halfduplex_dataset.csv")); CO=capn(pd.read_csv(f"{DS}/collision_dataset.csv")); DE=pd.read_csv(f"{DS}/decode_dataset.csv")
CAP=pd.read_csv(f"{DS}/capture_dataset.csv"); DNC=CAP[CAP.collided==0]; DCC=CAP[CAP.collided==1]   # collision-labeled decode
stages={"halfduplex":(HD,"n","blocked"),"collision":(CO,"n","collided"),"decode":(DE,"sinr_db","decoded"),
        "dec_nc":(DNC,"sinr_db","decoded"),"dec_cc":(DCC,"sinr_db","decoded")}
print(f"train rows: halfduplex={len(HD)} collision={len(CO)} decode={len(DE)}")

def cv_metrics(df,feat,tgt):
    x=df[[feat]].values; y=df[tgt].values.astype(int); res={}
    skf=StratifiedKFold(5,shuffle=True,random_state=0)
    for name in ("Linear","Logistic","GBM"):
        ll=[];br=[];au=[]
        for tr,te in skf.split(x,y):
            if name=="Linear":
                m=LinearRegression().fit(x[tr],y[tr]); p=np.clip(m.predict(x[te]),1e-4,1-1e-4)
            elif name=="Logistic":
                x2=np.column_stack([x,x**2]); m=LogisticRegression(max_iter=1000).fit(x2[tr],y[tr]); p=m.predict_proba(x2[te])[:,1]
            else:
                m=GradientBoostingClassifier(n_estimators=150,max_depth=3,learning_rate=0.05,subsample=0.8,random_state=0).fit(x[tr],y[tr]); p=m.predict_proba(x[te])[:,1]
            ll.append(log_loss(y[te],p,labels=[0,1])); br.append(brier_score_loss(y[te],p))
            au.append(roc_auc_score(y[te],p) if len(set(y[te]))>1 else np.nan)
        res[name]=dict(logloss=np.mean(ll),brier=np.mean(br),auc=np.nanmean(au))
    return res

print("=== per-stage learner CV (same feature per stage) ===")
mrows=[]
for st,(df,feat,tgt) in stages.items():
    r=cv_metrics(df,feat,tgt)
    for L in r: mrows.append(dict(stage=st,model=L,**r[L]))
    print(st,{L:round(r[L]["logloss"],3) for L in r})
pd.DataFrame(mrows).to_csv(f"{OUT}/learner_stage_metrics.csv",index=False)

# fit final models per learner per stage
def fit_stage(df,feat,tgt):
    x=df[[feat]].values; y=df[tgt].values.astype(int)
    lin=LinearRegression().fit(x,y)
    log=LogisticRegression(max_iter=1000).fit(np.column_stack([x,x**2]),y)
    gbm=GradientBoostingClassifier(n_estimators=150,max_depth=3,learning_rate=0.05,subsample=0.8,random_state=0).fit(x,y)
    return lin,log,gbm
fits={st:fit_stage(df,feat,tgt) for st,(df,feat,tgt) in stages.items()}
def P(st,L,xval):
    x=np.atleast_1d(xval).astype(float); lin,log,gbm=fits[st]
    if L=="Linear": return np.clip(lin.predict(x.reshape(-1,1)),0,1)
    if L=="Logistic": return log.predict_proba(np.column_stack([x,x**2]))[:,1]
    return gbm.predict_proba(x.reshape(-1,1))[:,1]

# DENSITY-AWARE decode curves (SINR, n): capture weakens with density (closer interferers).
# Learned from the multi-density PHY (cav01/05 + high-density cav25/50/100). Logistic in
# [SINR, SINR^2, n, SINR*n]. n = the rate's mean CAV density.
CAPd=pd.read_csv(f"{DS}/capture_dataset.csv")   # has PER-RECEPTION density n (matches OMNeT countNeighbours)
def feat2(s,n):
    s=np.atleast_1d(np.asarray(s,float)); n=np.full_like(s,float(n)) if np.isscalar(n) else np.asarray(n,float)
    return np.column_stack([s,s*s,n,s*n])
def fit2(g): return LogisticRegression(max_iter=1000).fit(feat2(g.sinr_db.values,g.n.values),g.decoded.values.astype(int))
dnc2=fit2(CAPd[CAPd.collided==0]); dcc2=fit2(CAPd[CAPd.collided==1])
def decode2(model,s,n): return model.predict_proba(feat2(s,n))[:,1]

# NS-3 per-instant distance distributions
def recon(meta,fcdfile,db):
    m=pd.read_csv(meta); n2v=dict(zip(m.nodeId,m.vehId)); dep=dict(zip(m.nodeId,m.departTime)); arr=dict(zip(m.nodeId,m.arriveTime))
    f=pd.read_csv(fcdfile); f=f[f.vehicle_type=="veh_av"]; pos={}
    for row in f.itertuples(index=False): pos.setdefault(row.vehicle_id,{})[round(row.timestep_time,1)]=(row.vehicle_x,row.vehicle_y)
    con=sqlite3.connect(db); tx=con.execute("SELECT nodeId,srcIp,pktSeqNum,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall()
    rx=con.execute("SELECT nodeId,srcIp,pktSeqNum FROM pktTxRx WHERE txRx='rx'").fetchall(); con.close()
    dl={};
    for rn,sip,seq in rx: dl.setdefault((sip,seq),set()).add(rn)
    nodes=list(n2v); att=np.zeros(len(BINS)); dlv=np.zeros(len(BINS)); nc=[]
    for sn,sip,seq,t in tx:
        tr=round(t,1); sp=pos.get(n2v.get(sn),{}).get(tr)
        if sp is None: continue
        got=dl.get((sip,seq),set()); inr=0
        for rn in nodes:
            if rn==sn or not(dep[rn]<=t<=arr[rn]): continue
            rp=pos.get(n2v[rn],{}).get(tr)
            if rp is None: continue
            d=np.hypot(sp[0]-rp[0],sp[1]-rp[1])
            if d>300: continue
            inr+=1; b=min(int(d//20),len(BINS)-1); att[b]+=1
            if rn in got: dlv[b]+=1
        nc.append(inr)
    return att,dlv,float(np.mean(nc))

geoms={}
for r in [5,25,50,100]:
    geoms[f"int_cav{r:02d}"]=recon(f"{NS3}/inputs/cav{r:02d}_s0_meta.csv",f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv",
                                   glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-{SCEN}-*.db")[0])
attm=np.zeros(len(BINS)); dlvm=np.zeros(len(BINS)); nmv=[]
for s in range(3):
    db=glob.glob(f"{NS3}/results/mainstreet_val/mainstreet50_s{s}-{SCEN}-*.db")
    if db:
        a,d_,n=recon(f"{NS3}/inputs/mainstreet50_s{s}_meta.csv",f"{PROJ}/ns3_mobility/mainstreet50/seed{s}/fcd.csv",db[0]); attm+=a;dlvm+=d_;nmv.append(n)
if nmv: geoms["main_50"]=(attm,dlvm,float(np.mean(nmv)))
else: print(f"(no {SCEN} mainstreet DBs — validation geometry skipped)")

# Monte-Carlo over the SAME TR 37.885 channel realisations OMNeT now samples (per-link LOS/NLOSv
# state + random NLOSv blockage log-normal + shadow fading sigma) plus the collision/capture draw.
# Fixed base randoms = common random numbers => smooth objective for the capture fit.
_NMC=40000; _rng=np.random.default_rng(0)   # large sample => robust capture fit (no overfit to CRN)
_u_state=_rng.random(_NMC); _z_shadow=_rng.standard_normal(_NMC)
_z_nlosv=_rng.standard_normal(_NMC); _u_coll=_rng.random(_NMC); _z_cap=_rng.standard_normal(_NMC)
def cascade(L,d,nval):
    # GROUNDED capture: no fitted penalty. When a collision occurs, decode follows the MEASURED
    # decode_coll(SINR) curve (capture); otherwise decode_noColl(SINR). Both learned from NS-3.
    d=np.atleast_1d(np.asarray(d,dtype=float)); lg=np.log10(np.maximum(d,1.0))
    hd=float(P("halfduplex",L,[nval])); cl=float(P("collision",L,[nval]))
    if SCEN.startswith("UMi"):
        pl=pl_umi(d)[:,None] + 4.0*_z_shadow[None,:]
    else:
        pl0=(38.77+16.7*lg+18.2*np.log10(FC))[:,None]
        los=p_los(d)[:,None] < _u_state[None,:]
        muA=9.0+np.maximum(0.0,15.0*lg-41.0); sigA=4.5
        lmu=np.log(muA*muA/np.sqrt(sigA*sigA+muA*muA))[:,None]
        lsig=np.sqrt(np.log(sigA*sigA/(muA*muA)+1.0))[:,None]
        nlosv=np.maximum(0.0,np.exp(lmu+lsig*_z_nlosv[None,:]))
        pl=pl0 + np.where(los,0.0,nlosv) + 3.0*_z_shadow[None,:]
    sinr=TX-pl-NOISE
    dnc=decode2(dnc2,sinr.ravel(),nval).reshape(sinr.shape)   # density-aware: decode|no-collision
    dcc=decode2(dcc2,sinr.ravel(),nval).reshape(sinr.shape)   # density-aware: decode|collision (capture)
    dp=(1.0-cl)*dnc + cl*dcc
    return (1.0-hd)*dp.mean(axis=1)

print("\n=== end-to-end per-instant bake-off (grounded capture, both geometries) ===")
brows=[]
for L in ["Linear","Logistic","GBM"]:
    for key,(att,dlv,nval) in geoms.items():
        ns3=dlv[att>0].sum()/att[att>0].sum(); m=att>=10; dv=BINS[m]+10
        pred=np.sum(att[m]*cascade(L,dv,nval))/att[m].sum()
        brows.append(dict(learner=L,geom=key,density=round(nval,1),ns3=round(ns3,3),pred=round(pred,3),abserr=round(abs(pred-ns3),3)))
T=pd.DataFrame(brows); T.to_csv(f"{OUT}/c1_bakeoff_table.csv",index=False)
T["grp"]=T.geom.str.startswith("int").map({True:"intersection",False:"mainstreet"})
print(T.groupby(["learner","grp"]).abserr.mean().round(3).unstack().to_string())

# export ADOPTED coefficients to OMNeT bundle. Decode curves are DENSITY-AWARE 2-D logistics:
#   decode(SINR,n) = sigmoid(d0 + d1*SINR + d2*SINR^2 + d3*n + d4*SINR*n)   [_nc no-coll, _cc capture]
log_hd=fits["halfduplex"][1]; log_co=fits["collision"][1]
def c5(m): return [round(float(m.intercept_[0]),6)]+[round(float(x),6) for x in m.coef_[0]]   # c0..c4
nc=c5(dnc2); cc=c5(dcc2)
p=dict(cascade=1.0,capture_split=1.0,decode_density=1.0,
  hd_c0=round(float(log_hd.intercept_[0]),5),hd_c1=round(float(log_hd.coef_[0,0]),5),hd_c2=round(float(log_hd.coef_[0,1]),7),
  col_c0=round(float(log_co.intercept_[0]),5),col_c1=round(float(log_co.coef_[0,0]),5),col_c2=round(float(log_co.coef_[0,1]),7),
  decnc_c0=nc[0],decnc_c1=nc[1],decnc_c2=nc[2],decnc_c3=nc[3],decnc_c4=nc[4],
  deccc_c0=cc[0],deccc_c1=cc[1],deccc_c2=cc[2],deccc_c3=cc[3],deccc_c4=cc[4],
  tx_power_dbm=23.0,noise_floor_dbm=round(NOISE,2),fc_ghz=5.9)
# PRESERVE any C2 flood-model params (qa_b*) already in the bundle. This C1 refit only
# fits the C1 stages; without this merge it would silently DROP the separately-fitted
# q_a attacker-load coefficients (fit_attacker_factor.py) and break the flood model.
_out=f"{B}/cascade_params.csv"
if os.path.exists(_out):
    _prev=pd.read_csv(_out)
    for _,_r in _prev.iterrows():
        _k=str(_r["key"])
        if _k not in p:            # keep qa_b0..b3 and anything else this refit doesn't produce
            p[_k]=_r["value"]
pd.DataFrame(list(p.items()),columns=["key","value"]).to_csv(_out,index=False)
print("\nexported logistic cascade_params.csv (grounded capture split; C2 qa_b* preserved):",{k:p[k] for k in ("hd_c0","hd_c1","col_c0","col_c1","decnc_c0","decnc_c1","deccc_c0","deccc_c1")})
