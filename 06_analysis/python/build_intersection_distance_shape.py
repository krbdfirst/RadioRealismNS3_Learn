#!/usr/bin/env python3
"""
KEY deliverable for the distance-aware refinement: on the TRAINING topology
(intersection), overlay OMNeT NS3Learn (distance-aware) per-link PDR(distance)
against NS-3 5G-LENA's reconstructed PDR(distance), for several rates.
Shows the distance SHAPE is now reproduced (steep near-field decay), not just
the aggregate -- the limitation that motivated this refinement.
-> results/prop_compare_figures/perlink/pl6_intersection_distance_shape.png
"""
import sqlite3, glob, os, numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3 = f"{NS3_SCRATCH}"
PROJ = f"{PROJECT_DIR}"
OUT = f"{PROJ}/results/prop_compare_figures/perlink"
BINS = np.arange(0, 320, 20)

def ns3_recon(r):
    me=f"{NS3}/inputs/cav{r:02d}_s0_meta.csv"; fc=f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv"
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-UMi_StreetCanyon_LoS-*.db")
    if not(db and os.path.exists(me) and os.path.exists(fc)): return None
    meta=pd.read_csv(me); n2v=dict(zip(meta.nodeId,meta.vehId))
    dep=dict(zip(meta.nodeId,meta.departTime)); arr=dict(zip(meta.nodeId,meta.arriveTime))
    fcd=pd.read_csv(fc); fcd=fcd[fcd.vehicle_type=="veh_av"]; pos={}
    for vid,t,x,y in zip(fcd.vehicle_id,fcd.timestep_time,fcd.vehicle_x,fcd.vehicle_y):
        pos.setdefault(vid,{})[round(t,1)]=(x,y)
    con=sqlite3.connect(db[0])
    tx=con.execute("SELECT nodeId,srcIp,pktSeqNum,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall()
    rx=con.execute("SELECT nodeId,srcIp,pktSeqNum FROM pktTxRx WHERE txRx='rx'").fetchall(); con.close()
    deliv={}
    for rn,sip,seq in rx: deliv.setdefault((sip,seq),set()).add(rn)
    nodes=list(n2v); att=np.zeros(len(BINS)); dlv=np.zeros(len(BINS))
    for sn,sip,seq,t in tx:
        tr=round(t,1); sp=pos.get(n2v.get(sn),{}).get(tr)
        if sp is None: continue
        got=deliv.get((sip,seq),set())
        for rn in nodes:
            if rn==sn or not(dep[rn]<=t<=arr[rn]): continue
            rp=pos.get(n2v[rn],{}).get(tr)
            if rp is None: continue
            d=np.hypot(sp[0]-rp[0],sp[1]-rp[1]); b=min(int(d//20),len(BINS)-1)
            att[b]+=1
            if rn in got: dlv[b]+=1
    return np.divide(dlv,att,out=np.full(len(BINS),np.nan),where=att>=30)

def omnet_shape(tag):
    cs=[]
    for d in sorted(glob.glob(f"{PROJ}/results/inet/Prop_NS3Learn_{tag}-*/")):
        f=sorted(glob.glob(d+"dist_pdr_*.csv"))
        if f: cs.append(pd.read_csv(f[-1])[["dist_bin_m","attempts","delivered"]])
    if not cs: return None,None
    g=pd.concat(cs).groupby("dist_bin_m").agg(a=("attempts","sum"),dl=("delivered","sum")); g=g[g.a>=30]
    return g.index.values, (g.dl/g.a).values

rates=[(5,"CAV05"),(25,"CAV25"),(50,"CAV50"),(100,"CAV100")]
fig,axes=plt.subplots(1,4,figsize=(18,4.5),sharey=True)
for ax,(r,tag) in zip(axes,rates):
    ns3=ns3_recon(r)
    if ns3 is not None: ax.plot(BINS+10,ns3,"ks--",lw=2.2,ms=5,label="NS-3 5G-LENA")
    ox,oy=omnet_shape(tag)
    if ox is not None: ax.plot(ox,oy,"-",color="#9467bd",lw=2.4,label="NS3Learn (dist-aware)")
    ax.axvline(300,color="grey",ls=":",lw=1); ax.set_xlim(0,300); ax.set_ylim(0,1.02)
    ax.set_title(f"cav{r:02d}"); ax.set_xlabel("Tx–Rx distance (m)"); ax.grid(alpha=.3)
axes[0].set_ylabel("PDR"); axes[0].legend(loc="upper right",fontsize=9)
fig.suptitle("Per-link PDR vs distance — intersection (training domain, LOS-only)\n"
             "distance-aware NS3Learn reproduces NS-3's near-field decay shape across densities",y=1.02)
fig.tight_layout()
p=f"{OUT}/pl6_intersection_distance_shape.png"; fig.savefig(p,dpi=150,bbox_inches="tight"); print("wrote",p)
