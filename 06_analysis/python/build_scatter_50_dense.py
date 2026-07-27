#!/usr/bin/env python3
"""
Dense per-tx-rx parity data at matched 50% load: pair NS-3 vs OMNeT per (seed x 10 m distance-bin)
instead of pooling seeds -> ~5x/3x more points + run-to-run spread. Same SUMO mobility per seed,
so NS-3 seed s <-> OMNeT seed s is a valid pairing. cav50 (5 seeds) + mainstreet50 (3 seeds), V2V.
-> results/paper_figures/data/per_txrx_50_dense.csv  (ns3,omnet,geom,seed,dist,att_ns3,att_omnet)
"""
import sqlite3, glob, numpy as np, pandas as pd
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
PROJ=f"{PROJECT_DIR}"
BW=10; BINS=np.arange(0,310,BW)

def ns3_seed(meta,fcdfile,db):
    m=pd.read_csv(meta); n2v=dict(zip(m.nodeId,m.vehId)); dep=dict(zip(m.nodeId,m.departTime)); arr=dict(zip(m.nodeId,m.arriveTime))
    f=pd.read_csv(fcdfile); f=f[f.vehicle_type=="veh_av"]; pos={}
    for r in f.itertuples(index=False): pos.setdefault(r.vehicle_id,{})[round(r.timestep_time,1)]=(r.vehicle_x,r.vehicle_y)
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
            b=min(int(d//BW),len(BINS)-1); att[b]+=1
            if rn in got: dlv[b]+=1
    return att,dlv

def omnet_seed(d):
    cs=sorted(glob.glob(d+"dist_pdr_*.csv"))
    if not cs: return None
    x=pd.read_csv(cs[-1]); return dict(zip(x.dist_bin_m,zip(x.attempts,x.delivered)))

rows=[]
def add(geom,seeds,meta_f,fcd_f,db_f,om_f):
    for s in seeds:
        db=glob.glob(db_f(s));
        if not db: continue
        og=omnet_seed(om_f(s))
        if og is None: continue
        att,dlv=ns3_seed(meta_f(s),fcd_f(s),db[0])
        for i,b in enumerate(BINS):
            if att[i]>=30 and b in og and og[b][0]>=30:
                rows.append(dict(ns3=dlv[i]/att[i],omnet=og[b][1]/og[b][0],geom=geom,seed=s,dist=int(b),
                                 att_ns3=int(att[i]),att_omnet=int(og[b][0])))
    print(f"{geom}: {sum(1 for r in rows if r['geom']==geom)} points")

add("intersection",range(5),
    lambda s:f"{NS3}/inputs/cav50_s{s}_meta.csv",
    lambda s:f"{PROJ}/ns3_mobility/cav50/seed{s}/fcd_cav50.csv",
    lambda s:f"{NS3}/results/teacher/cav50_s{s}-V2V_Urban-*.db",
    lambda s:f"{PROJ}/results/inet/Prop_NS3Learn_CAV50-{s}/")
add("mainstreet",range(3),
    lambda s:f"{NS3}/inputs/mainstreet50_s{s}_meta.csv",
    lambda s:f"{PROJ}/ns3_mobility/mainstreet50/seed{s}/fcd.csv",
    lambda s:f"{NS3}/results/mainstreet_val/mainstreet50_s{s}-V2V_Urban-*.db",
    lambda s:f"{PROJ}/results/inet/Prop_NS3Learn_MS50-{s}/")
P=pd.DataFrame(rows); P.to_csv(f"{PROJ}/results/paper_figures/data/per_txrx_50_dense.csv",index=False)
print(f"total {len(P)} points; MAE={np.mean(np.abs(P.ns3-P.omnet)):.3f} r={np.corrcoef(P.ns3,P.omnet)[0,1]:.3f}")
print(P.groupby('geom').apply(lambda d: pd.Series({'n':len(d),'MAE':np.mean(np.abs(d.ns3-d.omnet))}),include_groups=False).round(3).to_string())
