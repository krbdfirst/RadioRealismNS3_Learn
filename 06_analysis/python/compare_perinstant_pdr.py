#!/usr/bin/env python3
"""
CRITICAL methodology check. NS-3's avrgPrr table divides by numPackets*cumulativeNeighbours,
which deflates PRR ~3x at an intersection (transient neighbours). OMNeT dist_pdr is per-instant
in-range PDR. This compares APPLES-to-APPLES: NS-3 per-instant in-range PDR (raw reconstruction,
delivered/attempts <300m) vs each OMNeT model's per-instant dist_pdr, plus the deflated avrgPrr,
to see whether the 'MAC collapse' is real or a metric artefact.
"""
import sqlite3, glob, os, numpy as np, pandas as pd
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
PROJ=f"{PROJECT_DIR}"

def ns3_perinstant(r, R=300.0):
    me=f"{NS3}/inputs/cav{r:02d}_s0_meta.csv"; fc=f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv"
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-UMi_StreetCanyon_LoS-*.db")
    if not(db and os.path.exists(me) and os.path.exists(fc)): return None
    meta=pd.read_csv(me); n2v=dict(zip(meta.nodeId,meta.vehId))
    dep=dict(zip(meta.nodeId,meta.departTime)); arr=dict(zip(meta.nodeId,meta.arriveTime))
    f=pd.read_csv(fc); f=f[f.vehicle_type=="veh_av"]; pos={}
    for vid,t,x,y in zip(f.vehicle_id,f.timestep_time,f.vehicle_x,f.vehicle_y): pos.setdefault(vid,{})[round(t,1)]=(x,y)
    con=sqlite3.connect(db[0])
    tx=con.execute("SELECT nodeId,srcIp,pktSeqNum,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall()
    rx=con.execute("SELECT nodeId,srcIp,pktSeqNum FROM pktTxRx WHERE txRx='rx'").fetchall()
    prr_tbl=con.execute("SELECT avrgPrr FROM avrgPrr").fetchall(); con.close()
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
            if np.hypot(sp[0]-rp[0],sp[1]-rp[1])<=R:
                att+=1
                if rn in got: dlv+=1
    return dict(perinstant=dlv/att if att else np.nan, avrgPrr=np.mean([p[0] for p in prr_tbl]))

def omnet_perinstant(tag):
    v=[]
    for d in sorted(glob.glob(f"{PROJ}/results/inet/Prop_{tag}-*/")):
        cs=sorted(glob.glob(d+"dist_pdr_*.csv"))
        if cs:
            x=pd.read_csv(cs[-1]); w=x[x.dist_bin_m<300]
            if w.attempts.sum(): v.append(w.delivered.sum()/w.attempts.sum())
    return np.mean(v) if v else np.nan

print(f"{'rate':>5} | {'NS3 per-instant':>15} {'NS3 avrgPrr':>12} | {'OMNeT Analyt':>12} {'Generaliz':>10} {'NS3Learn':>9}")
print("-"*82)
RT={1:'CAV01',5:'CAV05',25:'CAV25',50:'CAV50',75:'CAV75',100:'CAV100'}
rows=[]
for r,tg in RT.items():
    ns=ns3_perinstant(r)
    if ns is None: continue
    a=omnet_perinstant(f"Analytical_{tg}"); g=omnet_perinstant(f"Generalized_{tg}"); n=omnet_perinstant(f"NS3Learn_{tg}")
    rows.append(dict(rate=r,ns3_perinstant=ns['perinstant'],ns3_avrgPrr=ns['avrgPrr'],omnet_analytical=a,omnet_generalized=g,omnet_ns3learn=n))
    print(f"{r:>5} | {ns['perinstant']:>15.3f} {ns['avrgPrr']:>12.3f} | {a:>12.3f} {g:>10.3f} {n:>9.3f}")
pd.DataFrame(rows).to_csv(f"{PROJ}/results/prop_compare_figures/perinstant_vs_avrgprr.csv",index=False)
print("\nIf NS3 per-instant ~ OMNeT Analytical, the 'MAC collapse' was largely an avrgPrr metric artefact (transient-neighbour deflation).")
