#!/usr/bin/env python3
"""
Calibrate the interference penalty I(density) for the distance-aware NS3Learn fix.
For each NS-3 run, reconstruct the true distance-resolved PDR (pktTxRx + FCD positions),
then find the SINR shift I such that BLER(SINR_noise(d) - I) best matches NS-3 PDR(d).
Fit I vs CAV-neighbour density. -> results/teacher_dataset/ns3learn_runtime/interference_fit.csv
"""
import sqlite3, glob, os, numpy as np, pandas as pd
from scipy.optimize import minimize_scalar
from scipy.interpolate import interp1d

from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3 = f"{NS3_SCRATCH}"
PROJ = f"{PROJECT_DIR}"
DS = f"{PROJ}/results/teacher_dataset"
FC, NF, BW, TX = 5.9, 7.0, 18.72e6, 23.0
NOISE = -174 + 10*np.log10(BW) + NF
lg = np.log10
BINS = np.arange(0, 320, 20)

def pl_umi_los(d):
    d = np.maximum(d, 10.0)
    return 32.4 + 21*lg(d) + 20*lg(FC)

# NS-3 V2V BLER curve (sci*pssch) from the LOS-only bundle
b = pd.read_csv(f"{DS}/ns3learn_runtime/bler_pssch.csv")
bs = pd.read_csv(f"{DS}/ns3learn_runtime/bler_sci.csv")
pssch = interp1d(b.snr_db, 1-b.bler, bounds_error=False, fill_value=(0.0, float(1-b.bler.iloc[-1])))
sci = interp1d(bs.snr_db, 1-bs.bler, bounds_error=False, fill_value=(0.0, float(1-bs.bler.iloc[-1])))
def model_pdr(d, I):
    se = TX - pl_umi_los(d) - NOISE - I
    return np.clip(pssch(se)*sci(se), 0, 1)

def recon(db, meta_csv, fcd_csv):
    meta = pd.read_csv(meta_csv); node2veh = dict(zip(meta.nodeId, meta.vehId))
    dep = dict(zip(meta.nodeId, meta.departTime)); arr = dict(zip(meta.nodeId, meta.arriveTime))
    fcd = pd.read_csv(fcd_csv); fcd = fcd[fcd.vehicle_type == "veh_av"]
    pos = {}
    for vid, t, x, y in zip(fcd.vehicle_id, fcd.timestep_time, fcd.vehicle_x, fcd.vehicle_y):
        pos.setdefault(vid, {})[round(t,1)] = (x,y)
    con = sqlite3.connect(db)
    tx = con.execute("SELECT nodeId,srcIp,pktSeqNum,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall()
    rx = con.execute("SELECT nodeId,srcIp,pktSeqNum FROM pktTxRx WHERE txRx='rx'").fetchall(); con.close()
    deliv = {}
    for rn, sip, seq in rx: deliv.setdefault((sip,seq), set()).add(rn)
    nodes = list(node2veh)
    att = np.zeros(len(BINS)); dlv = np.zeros(len(BINS)); ncnt = []
    for sn, sip, seq, t in tx:
        tr = round(t,1); sp = pos.get(node2veh.get(sn), {}).get(tr)
        if sp is None: continue
        got = deliv.get((sip,seq), set()); inr = 0
        for rn in nodes:
            if rn == sn or not (dep[rn] <= t <= arr[rn]): continue
            rp = pos.get(node2veh[rn], {}).get(tr)
            if rp is None: continue
            dd = np.hypot(sp[0]-rp[0], sp[1]-rp[1])
            if dd <= 320: inr += 1
            bb = min(int(dd//20), len(BINS)-1); att[bb]+=1
            if rn in got: dlv[bb]+=1
        ncnt.append(inr)
    pdr = np.divide(dlv, att, out=np.full(len(BINS),np.nan), where=att>=30)
    return pdr, float(np.mean(ncnt)) if ncnt else np.nan

# datasets: (label, db_glob, meta, fcd)
runs = []
for r in [5,25,50,100]:
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-UMi_StreetCanyon_LoS-*.db")
    me=f"{NS3}/inputs/cav{r:02d}_s0_meta.csv"; fc=f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv"
    if db and os.path.exists(me) and os.path.exists(fc): runs.append((f"cav{r:02d}",db[0],me,fc))
db=glob.glob(f"{NS3}/results/mainstreet_val/mainstreet50_s0-*.db")
if db: runs.append(("mainstreet50",db[0],f"{NS3}/inputs/mainstreet50_s0_meta.csv",f"{PROJ}/ns3_mobility/mainstreet50/seed0/fcd.csv"))

rows=[]
for lbl,db,me,fc in runs:
    pdr,dens = recon(db,me,fc)
    valid = ~np.isnan(pdr)
    if valid.sum()<3: print(f"{lbl}: too sparse"); continue
    dvals=BINS[valid]+10; pvals=pdr[valid]
    def err(I): return np.nansum((model_pdr(dvals,I)-pvals)**2)
    res=minimize_scalar(err,bounds=(0,50),method='bounded'); I=res.x
    rows.append(dict(label=lbl,density=dens,I_db=I))
    print(f"{lbl}: density={dens:.1f}  I={I:.1f} dB  (NS-3 PDR @ {[int(x) for x in dvals[:4]]}m = {[round(v,2) for v in pvals[:4]]})")
R=pd.DataFrame(rows)
# fit I = a*log10(1+density) + b
import numpy as np
A=np.vstack([np.log10(1+R.density), np.ones(len(R))]).T
coef,_,_,_=np.linalg.lstsq(A,R.I_db,rcond=None); a,b=coef
print(f"\nI(density) = {a:.2f}*log10(1+n) + {b:.2f}")
R.to_csv(f"{DS}/ns3learn_runtime/interference_fit.csv",index=False)
pd.DataFrame([dict(intf_a=round(a,3),intf_b=round(b,3),noise_floor_dbm=round(NOISE,2))]).to_csv(f"{DS}/ns3learn_runtime/interference_params.csv",index=False)
print("fit check:", {r.label:f"I_fit={a*np.log10(1+r.density)+b:.1f} vs {r.I_db:.1f}" for _,r in R.iterrows()})
