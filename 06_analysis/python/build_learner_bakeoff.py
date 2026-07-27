#!/usr/bin/env python3
"""
CONTRIBUTION-1 finaliser: end-to-end three-learner bake-off of the DECOMPOSED cascade,
evaluated on PER-INSTANT PDR, on the training geometry (intersection cav05/25/50/100) AND the
validation geometry (mainstreet 50%). Each learner (linear / logistic / GBM) fits the decode and
collision stages; capture (mu_c,sigma_c) is refit per learner; half-duplex const. Cascade composed,
aggregated over the NS-3 neighbour-distance distribution -> per-instant PRR. Compared against
NS-3 per-instant and the OMNeT Analytical / Generalized baselines.
-> results/prop_compare_figures/perinstant/{c1_bakeoff_table.csv, c1_bakeoff.png}
"""
import sqlite3, glob, os, numpy as np, pandas as pd
from scipy.optimize import minimize
from sklearn.linear_model import LinearRegression, LogisticRegression
from sklearn.ensemble import GradientBoostingClassifier
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
PROJ=f"{PROJECT_DIR}"
DS=f"{PROJ}/results/teacher_dataset/cascade"; OUT=f"{PROJ}/results/prop_compare_figures/perinstant"; os.makedirs(OUT,exist_ok=True)
FC,NF,BW,TX=5.9,7.0,18.72e6,23.0; NOISE=-174+10*np.log10(BW)+NF; BINS=np.arange(0,320,20)
def pl(d): d=np.maximum(d,10.0); return 32.4+21*np.log10(d)+20*np.log10(FC)
def sinr_n(d): return TX-pl(d)-NOISE
gh_x,gh_w=np.polynomial.hermite_e.hermegauss(15); gh_w=gh_w/np.sqrt(2*np.pi)
HD=pd.read_csv(f"{DS}/halfduplex_by_rate.csv").hd_block_frac.mean()

# ---- fit the three learners on each stage ----
D=pd.read_csv(f"{DS}/decode_dataset.csv"); C=pd.read_csv(f"{DS}/collision_dataset.csv")
yd=D.decoded.values.astype(int); sd=D[["sinr_db"]].values
yc=C.collided.values.astype(int); cn=C[["n"]].values; cnc=C[["n","concurrency"]].values
conc_of_n=C.groupby(C.n.round().astype(int)).concurrency.mean().reindex(range(80)).interpolate(limit_direction="both")
def conc(n): return np.array([conc_of_n.iloc[int(min(max(round(v),0),79))] for v in np.atleast_1d(n)])

dec_lin=LinearRegression().fit(sd,yd)
dec_log=LogisticRegression(max_iter=1000).fit(np.column_stack([sd,sd**2]),yd)
dec_gbm=GradientBoostingClassifier(n_estimators=200,max_depth=3,learning_rate=0.05,subsample=0.8,random_state=0).fit(D[["sinr_db","d","n"]].values,yd)
col_lin=LinearRegression().fit(cn,yc)
col_log=LogisticRegression(max_iter=1000).fit(np.column_stack([cn,cn**2]),yc)
col_gbm=GradientBoostingClassifier(n_estimators=200,max_depth=3,learning_rate=0.05,subsample=0.8,random_state=0).fit(cnc,yc)

def decode_p(L,sinr,nval):
    s=np.atleast_1d(sinr).astype(float)
    if L=="Linear": return np.clip(dec_lin.predict(s.reshape(-1,1)),0,1)
    if L=="Logistic": return dec_log.predict_proba(np.column_stack([s,s**2]))[:,1]
    d=np.maximum(10.0,10**((TX-NOISE-s-32.4-20*np.log10(FC))/21))  # invert SINR->d for GBM feature
    return dec_gbm.predict_proba(np.column_stack([s,d,np.full_like(s,nval)]))[:,1]
def coll_p(L,n):
    n=np.atleast_1d(n).astype(float)
    if L=="Linear": return float(np.clip(col_lin.predict([[n[0]]]),0,1))
    if L=="Logistic": return float(col_log.predict_proba([[n[0],n[0]**2]])[:,1])
    return float(col_gbm.predict_proba([[n[0],conc(n)[0]]])[:,1])

def cascade(L,d,nval,muc,sgc):
    base=decode_p(L,sinr_n(d),nval); deg=np.zeros_like(np.atleast_1d(d),dtype=float)
    for x,w in zip(gh_x,gh_w): deg+=w*decode_p(L,sinr_n(d)-(muc+sgc*x),nval)
    cl=coll_p(L,nval); return (1-HD)*((1-cl)*base+cl*deg)

# ---- NS-3 distance distributions (per-instant) ----
def recon(meta,fcdfile,db,maxd=300):
    m=pd.read_csv(meta); n2v=dict(zip(m.nodeId,m.vehId)); dep=dict(zip(m.nodeId,m.departTime)); arr=dict(zip(m.nodeId,m.arriveTime))
    f=pd.read_csv(fcdfile); f=f[f.vehicle_type=="veh_av"]; pos={}
    for row in f.itertuples(index=False): pos.setdefault(row.vehicle_id,{})[round(row.timestep_time,1)]=(row.vehicle_x,row.vehicle_y)
    con=sqlite3.connect(db); tx=con.execute("SELECT nodeId,srcIp,pktSeqNum,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall()
    rx=con.execute("SELECT nodeId,srcIp,pktSeqNum FROM pktTxRx WHERE txRx='rx'").fetchall(); con.close()
    dl={};
    for rn,sip,seq in rx: dl.setdefault((sip,seq),set()).add(rn)
    nodes=list(n2v); att=np.zeros(len(BINS)); dlv=np.zeros(len(BINS)); ncnt=[]
    for sn,sip,seq,t in tx:
        tr=round(t,1); sp=pos.get(n2v.get(sn),{}).get(tr)
        if sp is None: continue
        got=dl.get((sip,seq),set()); inr=0
        for rn in nodes:
            if rn==sn or not(dep[rn]<=t<=arr[rn]): continue
            rp=pos.get(n2v[rn],{}).get(tr)
            if rp is None: continue
            d=np.hypot(sp[0]-rp[0],sp[1]-rp[1])
            if d>maxd: continue
            inr+=1; b=min(int(d//20),len(BINS)-1); att[b]+=1
            if rn in got: dlv[b]+=1
        ncnt.append(inr)
    return att,dlv,float(np.mean(ncnt))

geoms={}
for r in [5,25,50,100]:
    att,dlv,n=recon(f"{NS3}/inputs/cav{r:02d}_s0_meta.csv",f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv",
                    glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-UMi_StreetCanyon_LoS-*.db")[0])
    geoms[f"int_cav{r:02d}"]=(att,dlv,n)
# mainstreet (pool 3 seeds)
attm=np.zeros(len(BINS)); dlvm=np.zeros(len(BINS)); nm=[]
for s in range(3):
    db=glob.glob(f"{NS3}/results/mainstreet_val/mainstreet50_s{s}-*.db")
    if not db: continue
    a,d_,n=recon(f"{NS3}/inputs/mainstreet50_s{s}_meta.csv",f"{PROJ}/ns3_mobility/mainstreet50/seed{s}/fcd.csv",db[0]); attm+=a; dlvm+=d_; nm.append(n)
geoms["main_50"]=(attm,dlvm,float(np.mean(nm)))

def perinstant(att,dlv): m=att>0; return dlv[m].sum()/att[m].sum()
def cascade_perinstant(L,att,nval,muc,sgc):
    m=att>=10; dv=BINS[m]+10; return np.sum(att[m]*cascade(L,dv,nval,muc,sgc))/att[m].sum()

# fit capture per learner on intersection distance-resolved PDR
def fit_capture(L):
    def loss(p):
        muc,sgc=p
        if sgc<0.5: return 1e9
        e=0
        for r in [5,25,50,100]:
            att,dlv,nval=geoms[f"int_cav{r:02d}"]; m=att>=30; dv=BINS[m]+10; pdr=dlv[m]/att[m]
            e+=np.sum(att[m]*(cascade(L,dv,nval,muc,sgc)-pdr)**2)
        return e
    best=None
    for m0 in (15,25,35):
        rr=minimize(loss,[m0,7],method="Nelder-Mead")
        if best is None or rr.fun<best.fun: best=rr
    return best.x

# OMNeT baselines (per-instant) from earlier
omni=pd.read_csv(f"{PROJ}/results/prop_compare_figures/perinstant_vs_avrgprr.csv").set_index("rate")
rows=[]
caps={}
for L in ["Linear","Logistic","GBM"]:
    muc,sgc=fit_capture(L); caps[L]=(muc,sgc)
    for key,(att,dlv,nval) in geoms.items():
        ns3=perinstant(att,dlv); pred=cascade_perinstant(L,att,nval,muc,sgc)
        rows.append(dict(learner=L,geom=key,density=round(nval,1),ns3=round(ns3,3),pred=round(pred,3),abserr=round(abs(pred-ns3),3)))
T=pd.DataFrame(rows); T.to_csv(f"{OUT}/c1_bakeoff_table.csv",index=False)
print("capture params:",{k:(round(v[0],1),round(v[1],1)) for k,v in caps.items()})
print(T.to_string(index=False))
print("\nMAE by learner/geometry-group:")
T["grp"]=T.geom.str.startswith("int").map({True:"intersection(train)",False:"mainstreet(valid)"})
print(T.groupby(["learner","grp"]).abserr.mean().round(3).unstack().to_string())

# figure: per-instant PRR vs density, 3 learners + NS-3 + Analytical + Generalized (intersection)
fig,(ax1,ax2)=plt.subplots(1,2,figsize=(15,6))
xr=[5,25,50,100]; ns3v=[geoms[f"int_cav{r:02d}"][1].sum()/geoms[f"int_cav{r:02d}"][0].sum() for r in xr]
ax1.plot(xr,ns3v,"ks-",lw=2.6,ms=8,label="NS-3 (per-instant)")
for L,c in zip(["Linear","Logistic","GBM"],["#1f77b4","#ff7f0e","#2ca02c"]):
    yv=[cascade_perinstant(L,geoms[f"int_cav{r:02d}"][0],geoms[f"int_cav{r:02d}"][2],*caps[L]) for r in xr]
    ax1.plot(xr,yv,"o-" if L!="Linear" else "o--",color=c,lw=2,label=f"NS3-{L}")
ax1.plot(xr,[omni.loc[r].omnet_analytical for r in xr],"^:",color="#888",lw=1.6,label="Analytical")
ax1.plot(xr,[omni.loc[r].omnet_generalized for r in xr],"v:",color="#bbb",lw=1.6,label="Generalized")
ax1.set_xscale("log"); ax1.set_xticks(xr); ax1.set_xticklabels(xr); ax1.set_xlabel("CAV %"); ax1.set_ylabel("per-instant PDR"); ax1.set_ylim(0,1.02)
ax1.set_title("Training geometry (intersection)"); ax1.legend(fontsize=8); ax1.grid(alpha=.3)
# mainstreet bars
labels=["NS-3","NS3-Lin","NS3-Log","NS3-GBM","Analytical","Generalized"]
att,dlv,nval=geoms["main_50"]; msn=perinstant(att,dlv)
vals=[msn]+[cascade_perinstant(L,att,nval,*caps[L]) for L in ["Linear","Logistic","GBM"]]+[omni.loc[50].omnet_analytical,omni.loc[50].omnet_generalized]
ax2.bar(labels,vals,color=["k","#1f77b4","#ff7f0e","#2ca02c","#888","#bbb"]); ax2.axhline(msn,color="k",ls="--",lw=1)
ax2.set_ylabel("per-instant PDR"); ax2.set_title("Validation geometry (mainstreet 50%, out-of-sample)"); ax2.set_ylim(0,1.02)
for i,v in enumerate(vals): ax2.text(i,v+.02,f"{v:.2f}",ha="center",fontsize=9)
plt.setp(ax2.get_xticklabels(),rotation=20,ha="right")
fig.suptitle("Contribution 1: decomposed-cascade learner bake-off (per-instant PDR)",y=1.01,fontsize=13)
fig.tight_layout(); fig.savefig(f"{OUT}/c1_bakeoff.png",dpi=150,bbox_inches="tight"); print("\nwrote c1_bakeoff.png")
