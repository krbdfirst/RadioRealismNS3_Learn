#!/usr/bin/env python3
"""
Ground the capture term: label each NS-3 PHY reception as collided / not (via MAC RB-overlap in the
transmitter's slot), then learn TWO decode curves vs the channel (noise-limited) SINR:
  decode_noColl(SINR)  — receptions whose tx did NOT overlap another's resource
  decode_coll(SINR)    — receptions whose tx DID  (capture: near/high-SINR still decodes)
Replaces the fitted constant 26 dB penalty. PHY traces exist at cav01/05 (where collisions occur).
SINR = Tx - PL_V2V_mean(d) - N0 (distance-derived, channel-only — what OMNeT computes).
-> results/teacher_dataset/cascade/capture_dataset.csv  (sinr_db, collided, decoded)
"""
import sqlite3, glob, numpy as np, pandas as pd
from collections import defaultdict
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
PROJ=f"{PROJECT_DIR}"
FC,NF,BW,TX=5.9,7.0,18.72e6,23.0; NOISE=-174+10*np.log10(BW)+NF
def pl_v2v(d):
    d=max(d,1.0); lg=np.log10(d); plL=38.77+16.7*lg+18.2*np.log10(FC)
    nl=9+max(0,15*lg-41); pL=min(1,1.05*np.exp(-0.0114*d)); return pL*plL+(1-pL)*(plL+nl)

rows=[]
# low density: original teacher/ DBs (cav01/05); high density: new teacher_phy/ DBs (cav25/50/100)
SRC={1:"teacher",5:"teacher",25:"teacher_phy",50:"teacher_phy",100:"teacher_phy"}
for r in (1,5,25,50,100):
    me=f"{NS3}/inputs/cav{r:02d}_s0_meta.csv"; fc=f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv"
    dbg=glob.glob(f"{NS3}/results/{SRC[r]}/cav{r:02d}_s0-V2V_Urban-*.db")
    if not dbg: print(f"cav{r:02d}: no PHY DB ({SRC[r]}) — skip"); continue
    db=dbg[0]
    meta=pd.read_csv(me); n2v=dict(zip(meta.nodeId,meta.vehId))
    f=pd.read_csv(fc); f=f[f.vehicle_type=="veh_av"]; pos={}
    for row in f.itertuples(index=False): pos.setdefault(row.vehicle_id,{})[round(row.timestep_time,1)]=(row.vehicle_x,row.vehicle_y)
    con=sqlite3.connect(db)
    mac=con.execute("SELECT rnti,frame,subFrame,slot,rbStart,rbLen FROM psschTxUeMac").fetchall()
    phy=con.execute("SELECT timeMs,rnti,txRnti,frame,subFrame,slot,psschCorrupt,sci2Corrupt FROM psschRxUePhy").fetchall()
    con.close()
    # colliding (txNode, slotkey): another tx in same slot with overlapping RBs
    slots=defaultdict(list)
    for rnti,fr,sf,sl,rb0,rbl in mac: slots[(fr,sf,sl)].append((rnti-1,rb0,rb0+rbl))
    colliding=set()
    for key,lst in slots.items():
        for j,(nn,a0,a1) in enumerate(lst):
            if any(k!=j and not(a1<=b0 or b1<=a0) for k,(mm,b0,b1) in enumerate(lst)):
                colliding.add((nn,key))
    # precompute per-timestep CAV positions for density (receiver's in-range CAV count = OMNeT's countNeighbours)
    nodes=list(n2v)
    bytime=defaultdict(list)
    for vid,tm in pos.items():
        for tr_,xy in tm.items(): bytime[tr_].append(xy)
    def dens_at(tr,rx,ry):
        c=0
        for (x,y) in bytime.get(tr,()):
            if (x!=rx or y!=ry) and np.hypot(rx-x,ry-y)<=300: c+=1
        return c
    SAMP=max(1,len(phy)//120000)   # subsample huge PHY tables (density is O(nodes)/reception)
    for i,(timeMs,rnti,txRnti,fr,sf,sl,pc,sc) in enumerate(phy):
        if i%SAMP: continue
        txn=txRnti-1; rxn=rnti-1; tr=round(timeMs/1000.0,1)
        sp=pos.get(n2v.get(txn),{}).get(tr); rp=pos.get(n2v.get(rxn),{}).get(tr)
        if sp is None or rp is None: continue
        d=np.hypot(sp[0]-rp[0],sp[1]-rp[1])
        if d>300: continue
        sinr=TX-pl_v2v(d)-NOISE
        coll=int((txn,(fr,sf,sl)) in colliding)
        n=dens_at(tr,rp[0],rp[1])   # PER-RECEPTION receiver density (matches OMNeT countNeighbours)
        rows.append((r,sinr,n,coll,int(pc==0 and sc==0)))
    print(f"cav{r:02d}: rows={sum(1 for x in rows if x[0]==r)}")

D=pd.DataFrame(rows,columns=["rate","sinr_db","n","collided","decoded"])
D.to_csv(f"{PROJ}/results/teacher_dataset/cascade/capture_dataset.csv",index=False)
nc=D[D.collided==0]; cc=D[D.collided==1]
print(f"\ntotal {len(D)}: non-colliding {len(nc)} (decode {nc.decoded.mean():.3f}), colliding {len(cc)} (decode {cc.decoded.mean():.3f})")
# KEY CHECK: does decode_coll(SINR) depend on density (rate)? compare across rates
print("decode_coll(SINR) by rate (does capture weaken with density?):")
for rr in (5,25,50,100):
    g=cc[cc.rate==rr]
    if len(g)<200: continue
    print(f"  cav{rr:02d} (n={len(g)}):",{s:round(g[(g.sinr_db>=s)&(g.sinr_db<s+10)].decoded.mean(),2) for s in (10,20,30,40) if ((g.sinr_db>=s)&(g.sinr_db<s+10)).sum()>30})
