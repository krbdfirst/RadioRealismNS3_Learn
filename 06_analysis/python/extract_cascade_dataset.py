#!/usr/bin/env python3
"""
DECOMPOSED-CASCADE teacher dataset from NS-3 traces (cav01+cav05 only — psschRxUePhy is
disabled at higher density). nodeId=rnti-1; avrgSinr is LINEAR -> sinr_db=10*log10.

DECODE dataset (one row per psschRxUePhy reception): features sinr_db,d,n; labels sci_ok,
  pssch_ok,decoded  -> SCI/PSSCH BLER waterfall (rate-independent PHY -> valid at all density).
REACH dataset (one row per (tx,rx) pair): reach_frac = phy_receptions/intended_tx_while_inrange,
  with mean d,n -> half-duplex + resource availability (contention; low-density only).
-> results/teacher_dataset/cascade/{reach,decode}_dataset.csv
"""
import sqlite3, glob, os, numpy as np, pandas as pd
from collections import defaultdict
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
SCEN=os.environ.get("SCEN","V2V_Urban")
PROJ=f"{PROJECT_DIR}"
OUT=f"{PROJ}/results/teacher_dataset/cascade"; os.makedirs(OUT,exist_ok=True)
RANGE=300.0; RATES=[1,5]

def load(r):
    me=f"{NS3}/inputs/cav{r:02d}_s0_meta.csv"; fc=f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv"
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-{SCEN}-*.db")
    meta=pd.read_csv(me); n2v=dict(zip(meta.nodeId,meta.vehId))
    dep=dict(zip(meta.nodeId,meta.departTime)); arr=dict(zip(meta.nodeId,meta.arriveTime))
    f=pd.read_csv(fc); f=f[f.vehicle_type=="veh_av"]; pos={}
    for row in f.itertuples(index=False): pos.setdefault(row.vehicle_id,{})[round(row.timestep_time,1)]=(row.vehicle_x,row.vehicle_y)
    con=sqlite3.connect(db[0])
    tx=con.execute("SELECT nodeId,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall()
    phy=con.execute("SELECT timeMs,rnti,txRnti,avrgSinr,psschCorrupt,sci2Corrupt FROM psschRxUePhy").fetchall()
    con.close(); return n2v,dep,arr,pos,tx,phy

def dens(pos,n2v,nodes,tr,sx,sy,sn):
    c=0
    for rn in nodes:
        if rn==sn: continue
        p=pos.get(n2v[rn],{}).get(tr)
        if p is not None and np.hypot(sx-p[0],sy-p[1])<=RANGE: c+=1
    return c

dec=[]; reach=[]
for r in RATES:
    n2v,dep,arr,pos,tx,phy=load(r); nodes=list(n2v)
    # DECODE rows + per-pair reached counts
    reached=defaultdict(int)
    for timeMs,rnti,txRnti,sinr,pc,sc in phy:
        txn=txRnti-1; rxn=rnti-1; tr=round(timeMs/1000.0,1)
        reached[(txn,rxn)]+=1
        sp=pos.get(n2v.get(txn),{}).get(tr); rp=pos.get(n2v.get(rxn),{}).get(tr)
        if sp is None or rp is None: continue
        d=np.hypot(sp[0]-rp[0],sp[1]-rp[1])
        if d>RANGE: continue
        n=dens(pos,n2v,nodes,tr,sp[0],sp[1],txn)
        dec.append((r,d,n,10*np.log10(max(sinr,1e-12)),int(sc==0),int(pc==0),int(sc==0 and pc==0)))
    # REACH per-pair: intended (tx while rx in-range) + mean d,n
    intended=defaultdict(int); sumd=defaultdict(float); sumn=defaultdict(float)
    for sn,t in tx:
        tr=round(t,1); sp=pos.get(n2v.get(sn),{}).get(tr)
        if sp is None: continue
        n=dens(pos,n2v,nodes,tr,sp[0],sp[1],sn)
        for rn in nodes:
            if rn==sn or not(dep[rn]<=t<=arr[rn]): continue
            rp=pos.get(n2v[rn],{}).get(tr)
            if rp is None: continue
            d=np.hypot(sp[0]-rp[0],sp[1]-rp[1])
            if d>RANGE: continue
            k=(sn,rn); intended[k]+=1; sumd[k]+=d; sumn[k]+=n
    for k,nint in intended.items():
        if nint<5: continue
        reach.append((r,sumd[k]/nint,sumn[k]/nint,min(1.0,reached.get(k,0)/nint),nint))
    print(f"cav{r:02d}: decode+={sum(1 for x in dec if x[0]==r)}, pairs={len([1 for x in reach if x[0]==r])}")

D=pd.DataFrame(dec,columns=["rate","d","n","sinr_db","sci_ok","pssch_ok","decoded"])
R=pd.DataFrame(reach,columns=["rate","d","n","reach_frac","intended"])
D.to_csv(f"{OUT}/decode_dataset.csv",index=False); R.to_csv(f"{OUT}/reach_dataset.csv",index=False)
print(f"\nDECODE {len(D)} rows: decoded={D.decoded.mean():.3f} sci_ok={D.sci_ok.mean():.3f} pssch_ok={D.pssch_ok.mean():.3f}")
print(f"  sinr_db range [{D.sinr_db.min():.1f},{D.sinr_db.max():.1f}], median {D.sinr_db.median():.1f}")
print(f"REACH {len(R)} pairs: reach_frac mean={R.reach_frac.mean():.3f} by rate {R.groupby('rate').reach_frac.mean().round(3).to_dict()}")
