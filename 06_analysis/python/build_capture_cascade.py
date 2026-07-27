#!/usr/bin/env python3
"""
Complete the DECOMPOSED CASCADE with a distance-dependent CAPTURE stage.
A collision degrades SINR by an interferer penalty I_coll~Normal(mu_c,sigma_c); the packet still
decodes if SINR_noise(d)-I_coll clears the BLER knee (capture -> near survives, far fails). Cascade:
  PDR(d,n) = (1-hd)*[ (1-coll(n))*decode(SINRn(d)) + coll(n)*E_Ic[decode(SINRn(d)-Ic)] ]
hd from MAC (~const), coll(n) from MAC collision dataset, decode(SINR) from PHY waterfall (logistic).
Fit (mu_c,sigma_c) to NS-3 distance-resolved PDR(d); validate aggregate + shape vs per-instant.
-> results/prop_compare_figures/learners/cascade_capture_validation.png + params CSV
"""
import sqlite3, glob, os, numpy as np, pandas as pd
from scipy.optimize import minimize
from sklearn.linear_model import LogisticRegression
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
PROJ=f"{PROJECT_DIR}"
DS=f"{PROJ}/results/teacher_dataset/cascade"; OUT=f"{PROJ}/results/prop_compare_figures/learners"
FC,NF,BW,TX=5.9,7.0,18.72e6,23.0; NOISE=-174+10*np.log10(BW)+NF; BINS=np.arange(0,320,20)
def pl(d): d=np.maximum(d,10.0); return 32.4+21*np.log10(d)+20*np.log10(FC)
def sinr_n(d): return TX-pl(d)-NOISE

# decode(SINR) logistic from PHY waterfall
D=pd.read_csv(f"{DS}/decode_dataset.csv"); yd=D.decoded.values.astype(int)
S=np.column_stack([D[["sinr_db"]].values,D[["sinr_db"]].values**2])
dec=LogisticRegression(max_iter=1000).fit(S,yd)
def decode_p(sinr): s=np.atleast_1d(sinr); return dec.predict_proba(np.column_stack([s,s**2]))[:,1]
# collision(n) empirical per rate + half-duplex
C=pd.read_csv(f"{DS}/collision_dataset.csv"); coll_by_rate=C.groupby("rate").collided.mean().to_dict()
dens_by_rate=C.groupby("rate").n.mean().to_dict()
hd=pd.read_csv(f"{DS}/halfduplex_by_rate.csv").hd_block_frac.mean()

# NS-3 distance-resolved PDR(d) per rate (per-instant reconstruction)
def recon(r):
    me=f"{NS3}/inputs/cav{r:02d}_s0_meta.csv"; fc=f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv"
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-UMi_StreetCanyon_LoS-*.db")[0]
    meta=pd.read_csv(me); n2v=dict(zip(meta.nodeId,meta.vehId))
    dep=dict(zip(meta.nodeId,meta.departTime)); arr=dict(zip(meta.nodeId,meta.arriveTime))
    f=pd.read_csv(fc); f=f[f.vehicle_type=="veh_av"]; pos={}
    for row in f.itertuples(index=False): pos.setdefault(row.vehicle_id,{})[round(row.timestep_time,1)]=(row.vehicle_x,row.vehicle_y)
    con=sqlite3.connect(db); tx=con.execute("SELECT nodeId,srcIp,pktSeqNum,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall()
    rx=con.execute("SELECT nodeId,srcIp,pktSeqNum FROM pktTxRx WHERE txRx='rx'").fetchall(); con.close()
    dl={}
    for rn,sip,seq in rx: dl.setdefault((sip,seq),set()).add(rn)
    nodes=list(n2v); att=np.zeros(len(BINS)); dlv=np.zeros(len(BINS))
    for sn,sip,seq,t in tx:
        tr=round(t,1); sp=pos.get(n2v.get(sn),{}).get(tr)
        if sp is None: continue
        got=dl.get((sip,seq),set())
        for rn in nodes:
            if rn==sn or not(dep[rn]<=t<=arr[rn]): continue
            rp=pos.get(n2v[rn],{}).get(tr)
            if rp is None: continue
            d=np.hypot(sp[0]-rp[0],sp[1]-rp[1])
            if d>300: continue
            b=min(int(d//20),len(BINS)-1); att[b]+=1; dlv+=0
            if rn in got: dlv[b]+=1
    return np.divide(dlv,att,out=np.full(len(BINS),np.nan),where=att>=30), att

RATES=[5,25,50,100]
ns3={r:recon(r) for r in RATES}
gh_x,gh_w=np.polynomial.hermite_e.hermegauss(15); gh_w=gh_w/np.sqrt(2*np.pi)
def cascade_pdr(d,coll,muc,sgc):
    sn=sinr_n(d); base=decode_p(sn)
    deg=np.zeros_like(np.atleast_1d(d),dtype=float)
    for x,w in zip(gh_x,gh_w): deg+=w*decode_p(sn-(muc+sgc*x))
    return (1-hd)*((1-coll)*base+coll*deg)

def loss(p):
    muc,sgc=p
    if sgc<0.5: return 1e9
    e=0
    for r in RATES:
        pdr,att=ns3[r]; m=~np.isnan(pdr); dv=BINS[m]+10
        e+=np.sum(att[m]*(cascade_pdr(dv,coll_by_rate[r],muc,sgc)-pdr[m])**2)
    return e
best=None
for m0 in (10,20,30):
    rr=minimize(loss,[m0,8],method="Nelder-Mead")
    if best is None or rr.fun<best.fun: best=rr
muc,sgc=best.x
pd.DataFrame([dict(mu_c=round(muc,3),sigma_c=round(sgc,3),hd=round(hd,3))]).to_csv(f"{OUT}/cascade_capture_params.csv",index=False)
print(f"capture-collision interference: mu_c={muc:.2f} dB, sigma_c={sgc:.2f} dB, hd={hd:.3f}")

# validate: aggregate per-instant + distance shape
ref=pd.read_csv(f"{PROJ}/results/prop_compare_figures/perinstant_vs_avrgprr.csv").set_index("rate")
fig,axes=plt.subplots(1,4,figsize=(18,4.5),sharey=True)
print(f"\n{'rate':>5} {'NS-3 agg':>9} {'cascade agg':>11}")
for ax,r in zip(axes,RATES):
    pdr,att=ns3[r]; m=~np.isnan(pdr); dv=BINS[m]+10
    cas=cascade_pdr(dv,coll_by_rate[r],muc,sgc)
    agg=np.sum(att[m]*cas)/att[m].sum()
    print(f"{r:>5} {ref.loc[r].ns3_perinstant:>9.3f} {agg:>11.3f}")
    ax.plot(dv,pdr[m],"ks-",lw=2,ms=5,label="NS-3"); ax.plot(dv,cas,"-",color="#9467bd",lw=2.4,label="cascade+capture")
    ax.set_title(f"cav{r:02d} (n={dens_by_rate[r]:.0f}, coll={coll_by_rate[r]:.2f})"); ax.set_xlabel("distance (m)"); ax.grid(alpha=.3); ax.set_ylim(0,1.02)
axes[0].set_ylabel("PDR"); axes[0].legend()
fig.suptitle("Decomposed cascade WITH capture stage vs NS-3 per-instant PDR(distance)",y=1.02)
fig.tight_layout(); fig.savefig(f"{OUT}/cascade_capture_validation.png",dpi=150,bbox_inches="tight")
print("wrote cascade_capture_validation.png")
