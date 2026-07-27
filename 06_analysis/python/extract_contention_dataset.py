#!/usr/bin/env python3
"""
CONTENTION-side cascade data from the MAC schedule (psschTxUeMac), available at ALL densities.
For each transmission: detect SB-SPS resource collision (another tx in the same (frame,subFrame,
slot) with overlapping RBs) and per-slot concurrency; attach local CAV density n from FCD.
Also measure half-duplex reach (fraction of in-range rx NOT transmitting in the tx's slot).
nodeId = rnti-1.
-> results/teacher_dataset/cascade/collision_dataset.csv  (per-tx: rate,n,concurrency,collided)
-> results/teacher_dataset/cascade/halfduplex_by_rate.csv  (reach_hd vs density)
"""
import sqlite3, glob, os, numpy as np, pandas as pd
from collections import defaultdict
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
SCEN=os.environ.get("SCEN","V2V_Urban")
PROJ=f"{PROJECT_DIR}"
OUT=f"{PROJ}/results/teacher_dataset/cascade"; os.makedirs(OUT,exist_ok=True)
RANGE=300.0; RATES=[5,25,50,75,100]; SAMP={5:1,25:1,50:2,75:2,100:3}

def fcd(r):
    fc=f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv"
    me=f"{NS3}/inputs/cav{r:02d}_s0_meta.csv"
    meta=pd.read_csv(me); n2v=dict(zip(meta.nodeId,meta.vehId))
    dep=dict(zip(meta.nodeId,meta.departTime)); arr=dict(zip(meta.nodeId,meta.arriveTime))
    f=pd.read_csv(fc); f=f[f.vehicle_type=="veh_av"]; pos={}
    for row in f.itertuples(index=False): pos.setdefault(row.vehicle_id,{})[round(row.timestep_time,1)]=(row.vehicle_x,row.vehicle_y)
    return n2v,dep,arr,pos

col_rows=[]; hd_rows=[]
for r in RATES:
    n2v,dep,arr,pos=fcd(r); nodes=list(n2v)
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-{SCEN}-*.db")[0]
    con=sqlite3.connect(db)
    tx=con.execute("SELECT timeMs,rnti,frame,subFrame,slot,rbStart,rbLen FROM psschTxUeMac").fetchall(); con.close()
    # group by slot key -> list of (node,rbStart,rbEnd,timeMs)
    slots=defaultdict(list)
    for timeMs,rnti,fr,sf,sl,rb0,rbl in tx:
        slots[(fr,sf,sl)].append((rnti-1,rb0,rb0+rbl,timeMs))
    def density(node,timeMs):
        tr=round(timeMs/1000.0,1); sp=pos.get(n2v.get(node),{}).get(tr)
        if sp is None: return None,None
        c=0
        for rn in nodes:
            if rn==node: continue
            p=pos.get(n2v[rn],{}).get(tr)
            if p is not None and np.hypot(sp[0]-p[0],sp[1]-p[1])<=RANGE: c+=1
        return c,(tr,sp)
    samp=SAMP[r]; i=0
    for key,lst in slots.items():
        conc=len(lst)
        for j,(node,a0,a1,tms) in enumerate(lst):
            i+=1
            if i%samp: continue
            collided=int(any(k!=j and not(a1<=b0 or b1<=a0) for k,(nn,b0,b1,_) in enumerate(lst)))
            n,_=density(node,tms)
            if n is None: continue
            col_rows.append((r,n,conc,collided))
    # half-duplex reach: sample tx packets, fraction of in-range rx NOT transmitting in same slot
    txbyslot_nodes={key:set(x[0] for x in lst) for key,lst in slots.items()}
    blocked=0; tot=0; si=0
    for (fr,sf,sl),lst in slots.items():
        slotset=txbyslot_nodes[(fr,sf,sl)]
        for node,a0,a1,tms in lst:
            si+=1
            if si%(samp*3): continue
            tr=round(tms/1000.0,1); sp=pos.get(n2v.get(node),{}).get(tr)
            if sp is None: continue
            for rn in nodes:
                if rn==node: continue
                p=pos.get(n2v[rn],{}).get(tr)
                if p is None or np.hypot(sp[0]-p[0],sp[1]-p[1])>RANGE: continue
                tot+=1; blocked+=int(rn in slotset)
    hd_rows.append((r, blocked/tot if tot else 0.0, tot))
    print(f"cav{r:02d}: collision rows={sum(1 for x in col_rows if x[0]==r)}, collided_frac={np.mean([x[3] for x in col_rows if x[0]==r]):.3f}, hd_block={blocked/tot if tot else 0:.3f}")

C=pd.DataFrame(col_rows,columns=["rate","n","concurrency","collided"])
H=pd.DataFrame(hd_rows,columns=["rate","hd_block_frac","n_pairs"])
C.to_csv(f"{OUT}/collision_dataset.csv",index=False); H.to_csv(f"{OUT}/halfduplex_by_rate.csv",index=False)
print(f"\nCOLLISION {len(C)} rows; collided by rate: {C.groupby('rate').collided.mean().round(3).to_dict()}")
print(f"density by rate: {C.groupby('rate').n.mean().round(1).to_dict()}")
print(f"HALF-DUPLEX block by rate: {dict(zip(H.rate,H.hd_block_frac.round(3)))}")
