#!/usr/bin/env python3
"""
Recalibrate I(density) against the NS-3 avrgPrr@300 KPI (the headline metric),
not the reconstructed per-link PDR. For each rate, the OMNeT distance-aware
dist_pdr gives the geometry-driven neighbour-distance distribution (attempts(d),
independent of I). Solve the SINR shift I such that the aggregate
PRR(I) = sum att(d)*decode(SINR_noise(d)-I) / sum att(d) matches NS-3 avrgPrr@300.
Then fit I = a*log10(1+density)+b and update the runtime bundle. No re-run needed
to calibrate; one confirming NS3Learn re-run applies the new coefficients.
"""
import sqlite3, glob, os, numpy as np, pandas as pd
from scipy.optimize import minimize_scalar, brentq
from scipy.interpolate import interp1d

from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3 = f"{NS3_SCRATCH}"
PROJ = f"{PROJECT_DIR}"
DS = f"{PROJ}/results/teacher_dataset"; B = f"{DS}/ns3learn_runtime"
FC, NF, BW, TX = 5.9, 7.0, 18.72e6, 23.0
NOISE = -174 + 10*np.log10(BW) + NF
lg = np.log10
def pl_umi_los(d):
    d = np.maximum(d, 10.0); return 32.4 + 21*lg(d) + 20*lg(FC)
b = pd.read_csv(f"{B}/bler_pssch.csv"); bs = pd.read_csv(f"{B}/bler_sci.csv")
pssch = interp1d(b.snr_db, 1-b.bler, bounds_error=False, fill_value=(0.0, float(1-b.bler.iloc[-1])))
sci   = interp1d(bs.snr_db, 1-bs.bler, bounds_error=False, fill_value=(0.0, float(1-bs.bler.iloc[-1])))
def decode(se): return np.clip(pssch(se)*sci(se), 0, 1)

# NS-3 avrgPrr@300 targets (headline KPI)
ns3 = pd.read_csv(f"{PROJ}/results/prop_compare_figures/ns3_prr300_per_run.csv")
tgt = ns3.groupby("rate").prr300.mean().to_dict()

# densities (recon mean in-range CAVs); reuse known, add cav01/cav75
known = {5:5.0,25:22.9,50:43.5,100:60.8}
def recon_density(r):
    me=f"{NS3}/inputs/cav{r:02d}_s0_meta.csv"; fc=f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv"
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-UMi_StreetCanyon_LoS-*.db")
    if not(db and os.path.exists(me) and os.path.exists(fc)): return np.nan
    meta=pd.read_csv(me); node2veh=dict(zip(meta.nodeId,meta.vehId))
    dep=dict(zip(meta.nodeId,meta.departTime)); arr=dict(zip(meta.nodeId,meta.arriveTime))
    fcd=pd.read_csv(fc); fcd=fcd[fcd.vehicle_type=="veh_av"]; pos={}
    for vid,t,x,y in zip(fcd.vehicle_id,fcd.timestep_time,fcd.vehicle_x,fcd.vehicle_y):
        pos.setdefault(vid,{})[round(t,1)]=(x,y)
    con=sqlite3.connect(db[0]); tx=con.execute("SELECT nodeId,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall(); con.close()
    nodes=list(node2veh); ncnt=[]
    for sn,t in tx:
        tr=round(t,1); sp=pos.get(node2veh.get(sn),{}).get(tr)
        if sp is None: continue
        inr=0
        for rn in nodes:
            if rn==sn or not(dep[rn]<=t<=arr[rn]): continue
            rp=pos.get(node2veh[rn],{}).get(tr)
            if rp is None: continue
            if np.hypot(sp[0]-rp[0],sp[1]-rp[1])<=320: inr+=1
        ncnt.append(inr)
    return float(np.mean(ncnt)) if ncnt else np.nan

def omnet_attempts(rate):
    tag={1:"CAV01",5:"CAV05",25:"CAV25",50:"CAV50",75:"CAV75",100:"CAV100"}[rate]
    cs=[]
    for d in sorted(glob.glob(f"{PROJ}/results/inet/Prop_NS3Learn_{tag}-*/")):
        f=sorted(glob.glob(d+"dist_pdr_*.csv"))
        if f: cs.append(pd.read_csv(f[-1])[["dist_bin_m","attempts"]])
    if not cs: return None
    g=pd.concat(cs).groupby("dist_bin_m").attempts.sum()
    g=g[g.index<300]
    return g.index.values+10.0, g.values.astype(float)

rows=[]
for r in [1,5,25,50,75,100]:
    dens = known.get(r) or recon_density(r)
    aw = omnet_attempts(r)
    if aw is None or not np.isfinite(dens): print(f"cav{r:02d}: missing"); continue
    dvals,att = aw
    def prr(I): return np.sum(att*decode(TX-pl_umi_los(dvals)-NOISE-I))/att.sum()
    # solve I in [0,60] for prr(I)=tgt[r]
    lo,hi=prr(60),prr(0)
    if tgt[r]>=hi: I=0.0
    elif tgt[r]<=lo: I=60.0
    else: I=brentq(lambda x: prr(x)-tgt[r], 0, 60)
    rows.append(dict(rate=r,density=dens,I_db=I,target=tgt[r],prr_check=prr(I)))
    print(f"cav{r:02d}: density={dens:5.1f}  I={I:5.1f}dB  target={tgt[r]:.3f}  modelPRR(I)={prr(I):.3f}")
R=pd.DataFrame(rows)
A=np.vstack([np.log10(1+R.density), np.ones(len(R))]).T
(a,b_),_,_,_=np.linalg.lstsq(A,R.I_db,rcond=None)
print(f"\nI(density) = {a:.2f}*log10(1+n) + {b_:.2f}")
print("fit check:", {f"cav{int(r.rate):02d}":f"{a*np.log10(1+r.density)+b_:.1f} vs {r.I_db:.1f}" for _,r in R.iterrows()})
R.to_csv(f"{B}/interference_fit_aggregate.csv",index=False)
# update bundle params
df=pd.read_csv(f"{B}/ns3learn_params.csv"); d=dict(zip(df.key,df.value))
d["intf_a"]=round(float(a),3); d["intf_b"]=round(float(b_),3)
pd.DataFrame(list(d.items()),columns=["key","value"]).to_csv(f"{B}/ns3learn_params.csv",index=False)
pd.DataFrame([dict(intf_a=round(float(a),3),intf_b=round(float(b_),3),noise_floor_dbm=round(NOISE,2))]).to_csv(f"{B}/interference_params.csv",index=False)
print("updated bundle intf_a,intf_b ->",round(float(a),3),round(float(b_),3))
