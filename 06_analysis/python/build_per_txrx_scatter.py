#!/usr/bin/env python3
"""
Per-tx-rx (per-instant) alignment scatter: OMNeT V2V-cascade emulator vs NS-3 5G-LENA (V2V_Urban).
Each point = a (scenario, distance-bin) per-instant PDR: x = NS-3 reconstructed PDR(d), y = OMNeT
dist_pdr PDR(d). Training geometry (intersection cav05/25/50/100) + validation (mainstreet 50%).
Closer to the y=x line = better per-link agreement.
-> results/prop_compare_figures/perinstant/per_txrx_scatter_v2v.png
"""
import sqlite3, glob, os, numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
PROJ=f"{PROJECT_DIR}"
OUT=f"{PROJ}/results/prop_compare_figures/perinstant"; BINS=np.arange(0,320,20)

def ns3_pdr(meta,fcdfile,db):
    m=pd.read_csv(meta); n2v=dict(zip(m.nodeId,m.vehId)); dep=dict(zip(m.nodeId,m.departTime)); arr=dict(zip(m.nodeId,m.arriveTime))
    f=pd.read_csv(fcdfile); f=f[f.vehicle_type=="veh_av"]; pos={}
    for row in f.itertuples(index=False): pos.setdefault(row.vehicle_id,{})[round(row.timestep_time,1)]=(row.vehicle_x,row.vehicle_y)
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
            b=min(int(d//20),len(BINS)-1); att[b]+=1
            if rn in got: dlv[b]+=1
    return att,dlv

def omnet_pdr(glob_pat):
    cs=[]
    for d in sorted(glob.glob(glob_pat)):
        f=sorted(glob.glob(d+"dist_pdr_*.csv"))
        if f: cs.append(pd.read_csv(f[-1])[["dist_bin_m","attempts","delivered"]])
    if not cs: return None
    g=pd.concat(cs).groupby("dist_bin_m").agg(a=("attempts","sum"),dl=("delivered","sum"))
    return g

pts=[]   # (ns3_pdr, omnet_pdr, geom, label)
# intersection
for r in [5,25,50,100]:
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-V2V_Urban-*.db")
    og=omnet_pdr(f"{PROJ}/results/inet/Prop_NS3Learn_CAV{r:02d}-0/")
    if not db or og is None: print(f"cav{r:02d}: missing (ns3={bool(db)}, omnet={og is not None})"); continue
    att,dlv=ns3_pdr(f"{NS3}/inputs/cav{r:02d}_s0_meta.csv",f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv",db[0])
    for i,b in enumerate(BINS):
        if att[i]>=30 and b in og.index and og.loc[b,"a"]>=30:
            pts.append((dlv[i]/att[i], og.loc[b,"dl"]/og.loc[b,"a"], "intersection", f"cav{r:02d}@{b}m"))
# mainstreet (validation)
attm=np.zeros(len(BINS)); dlvm=np.zeros(len(BINS))
for s in range(3):
    db=glob.glob(f"{NS3}/results/mainstreet_val/mainstreet50_s{s}-V2V_Urban-*.db")
    if db:
        a,d_=ns3_pdr(f"{NS3}/inputs/mainstreet50_s{s}_meta.csv",f"{PROJ}/ns3_mobility/mainstreet50/seed{s}/fcd.csv",db[0]); attm+=a; dlvm+=d_
ogm=omnet_pdr(f"{PROJ}/results/inet/Prop_NS3Learn_MS50-*/")
if ogm is not None and attm.sum()>0:
    for i,b in enumerate(BINS):
        if attm[i]>=30 and b in ogm.index and ogm.loc[b,"a"]>=30:
            pts.append((dlvm[i]/attm[i], ogm.loc[b,"dl"]/ogm.loc[b,"a"], "mainstreet", f"ms@{b}m"))

P=pd.DataFrame(pts,columns=["ns3","omnet","geom","label"]); P.to_csv(f"{OUT}/per_txrx_scatter_v2v.csv",index=False)
mae=np.mean(np.abs(P.ns3-P.omnet)); rmse=np.sqrt(np.mean((P.ns3-P.omnet)**2))
corr=np.corrcoef(P.ns3,P.omnet)[0,1] if len(P)>2 else float('nan')
fig,ax=plt.subplots(figsize=(7.5,7.2))
for g,c in [("intersection","#1f77b4"),("mainstreet","#d62728")]:
    s=P[P.geom==g]; ax.scatter(s.ns3,s.omnet,c=c,s=55,alpha=.75,edgecolor="k",lw=.4,label=f"{g} (n={len(s)})")
ax.plot([0,1],[0,1],"k--",lw=1.5,label="perfect (y=x)")
ax.set_xlim(0,1.02); ax.set_ylim(0,1.02); ax.set_xlabel("NS-3 5G-LENA per-instant PDR (V2V_Urban)")
ax.set_ylabel("OMNeT V2V-cascade emulator per-instant PDR")
ax.set_title(f"Per-tx-rx alignment: distilled cascade vs NS-3 5G-LENA\nMAE={mae:.3f}  RMSE={rmse:.3f}  r={corr:.3f}  (n={len(P)} distance-bins)")
ax.legend(loc="upper left"); ax.grid(alpha=.3); ax.set_aspect("equal"); fig.tight_layout()
p=f"{OUT}/per_txrx_scatter_v2v.png"; fig.savefig(p,dpi=150); print("wrote",p)
print(f"points={len(P)}  MAE={mae:.3f}  RMSE={rmse:.3f}  r={corr:.3f}")
print(P.groupby("geom").apply(lambda d: pd.Series({"MAE":np.mean(np.abs(d.ns3-d.omnet)),"n":len(d)}),include_groups=False).round(3).to_string())
