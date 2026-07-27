#!/usr/bin/env python3
"""
Per-observation HALF-DUPLEX dataset from the NS-3 MAC schedule (psschTxUeMac), so half-duplex is
LEARNED from data (function of density), never a hand-set constant. For each transmission by S in
a slot, every in-range CAV receiver R is one row: blocked=1 if R is also transmitting in that same
(frame,subFrame,slot) [R is deaf], else 0. Feature = local CAV density n at the tx time.
nodeId = rnti-1.  -> results/teacher_dataset/cascade/halfduplex_dataset.csv
"""
import sqlite3, glob, os, numpy as np, pandas as pd
from collections import defaultdict
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
SCEN=os.environ.get("SCEN","V2V_Urban")
PROJ=f"{PROJECT_DIR}"
OUT=f"{PROJ}/results/teacher_dataset/cascade"
RANGE=300.0; RATES=[5,25,50,75,100]; SAMP={5:1,25:3,50:6,75:5,100:8}

rows=[]
for r in RATES:
    me=f"{NS3}/inputs/cav{r:02d}_s0_meta.csv"; fc=f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv"
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-{SCEN}-*.db")[0]
    meta=pd.read_csv(me); n2v=dict(zip(meta.nodeId,meta.vehId)); nodes=list(n2v)
    f=pd.read_csv(fc); f=f[f.vehicle_type=="veh_av"]; pos={}
    for row in f.itertuples(index=False): pos.setdefault(row.vehicle_id,{})[round(row.timestep_time,1)]=(row.vehicle_x,row.vehicle_y)
    con=sqlite3.connect(db); tx=con.execute("SELECT timeMs,rnti,frame,subFrame,slot FROM psschTxUeMac").fetchall(); con.close()
    slots=defaultdict(set)                       # (frame,subFrame,slot) -> set of transmitting nodes
    txlist=[]
    for timeMs,rnti,fr,sf,sl in tx:
        slots[(fr,sf,sl)].add(rnti-1); txlist.append((rnti-1,(fr,sf,sl),timeMs))
    samp=SAMP[r]; i=0
    for txn,key,tms in txlist:
        i+=1
        if i%samp: continue
        tr=round(tms/1000.0,1); sp=pos.get(n2v.get(txn),{}).get(tr)
        if sp is None: continue
        slotset=slots[key]
        # density at tx time
        inr=[]
        for rn in nodes:
            if rn==txn: continue
            rp=pos.get(n2v[rn],{}).get(tr)
            if rp is not None and np.hypot(sp[0]-rp[0],sp[1]-rp[1])<=RANGE: inr.append(rn)
        n=len(inr)
        for rn in inr:
            rows.append((r,n,int(rn in slotset)))
    print(f"cav{r:02d}: rows={sum(1 for x in rows if x[0]==r)}")

H=pd.DataFrame(rows,columns=["rate","n","blocked"])
H.to_csv(f"{OUT}/halfduplex_dataset.csv",index=False)
print(f"\nHALF-DUPLEX {len(H)} rows; blocked mean={H.blocked.mean():.3f}")
print("blocked by rate:",H.groupby('rate').blocked.mean().round(3).to_dict())
print("density by rate:",H.groupby('rate').n.mean().round(1).to_dict())
