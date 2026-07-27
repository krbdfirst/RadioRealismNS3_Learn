#!/usr/bin/env python3
"""Reconstruct NS-3 per-instant in-range PDR for the two rates missing from the cache
(1 and 75), using the same method as build_paper_figure_data.py, and merge with the
cached 5/25/50/100 into results/deviation/ns3_groundtruth.csv (all six rates)."""
import sqlite3, glob, os, numpy as np, pandas as pd
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
PROJ=f"{PROJECT_DIR}"
OUT=f"{PROJ}/results/deviation"; os.makedirs(OUT, exist_ok=True)
BINS=np.arange(0,320,20)

def ns3_perinstant(meta,fcdfile,db):
    m=pd.read_csv(meta); n2v=dict(zip(m.nodeId,m.vehId)); dep=dict(zip(m.nodeId,m.departTime)); arr=dict(zip(m.nodeId,m.arriveTime))
    f=pd.read_csv(fcdfile); f=f[f.vehicle_type=="veh_av"]; pos={}
    for r in f.itertuples(index=False): pos.setdefault(r.vehicle_id,{})[round(r.timestep_time,1)]=(r.vehicle_x,r.vehicle_y)
    con=sqlite3.connect(db)
    tx=con.execute("SELECT nodeId,srcIp,pktSeqNum,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall()
    rx=con.execute("SELECT nodeId,srcIp,pktSeqNum FROM pktTxRx WHERE txRx='rx'").fetchall()
    con.close()
    dl={}
    for rn,sip,seq in rx: dl.setdefault((sip,seq),set()).add(rn)
    nodes=list(n2v); att=0; dlv=0
    for sn,sip,seq,t in tx:
        tr=round(t,1); sp=pos.get(n2v.get(sn),{}).get(tr)
        if sp is None: continue
        got=dl.get((sip,seq),set())
        for rn in nodes:
            if rn==sn or not(dep[rn]<=t<=arr[rn]): continue
            rp=pos.get(n2v[rn],{}).get(tr)
            if rp is None: continue
            if np.hypot(sp[0]-rp[0],sp[1]-rp[1])>300: continue
            att+=1
            if rn in got: dlv+=1
    return dlv/att if att else np.nan

rows=[]
for r in (1,75):
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-V2V_Urban-*.db")[0]
    pi=ns3_perinstant(f"{NS3}/inputs/cav{r:02d}_s0_meta.csv",
                      f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv", db)
    print(f"cav{r:02d}: NS-3 per-instant = {pi:.3f}")
    rows.append(dict(rate=r, ns3_perinstant=pi))

# merge with cached 5/25/50/100
cached=pd.read_csv(f"{PROJ}/results/paper_figures/data/per_instant_by_rate.csv")[["rate","ns3_perinstant"]]
allr=pd.concat([cached, pd.DataFrame(rows)]).drop_duplicates("rate").sort_values("rate")
allr.to_csv(f"{OUT}/ns3_groundtruth.csv", index=False)
print("\nAll six rates ->", f"{OUT}/ns3_groundtruth.csv")
print(allr.round(3).to_string(index=False))
