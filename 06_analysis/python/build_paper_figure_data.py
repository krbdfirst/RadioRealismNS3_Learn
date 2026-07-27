#!/usr/bin/env python3
"""
Cache the computed numbers for the Contribution-1 paper figures (expensive NS-3 reconstruction
done once; plotting reads these). All on the per-instant KPI / V2V_Urban channel.
-> results/paper_figures/data/{per_instant_by_rate,ns3_distance,omnet_distance,mainstreet}.csv
"""
import sqlite3, glob, os, numpy as np, pandas as pd
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
PROJ=f"{PROJECT_DIR}"
OUT=f"{PROJ}/results/paper_figures/data"; os.makedirs(OUT,exist_ok=True)
BINS=np.arange(0,320,20)

def ns3_recon(meta,fcdfile,db):
    m=pd.read_csv(meta); n2v=dict(zip(m.nodeId,m.vehId)); dep=dict(zip(m.nodeId,m.departTime)); arr=dict(zip(m.nodeId,m.arriveTime))
    f=pd.read_csv(fcdfile); f=f[f.vehicle_type=="veh_av"]; pos={}
    for r in f.itertuples(index=False): pos.setdefault(r.vehicle_id,{})[round(r.timestep_time,1)]=(r.vehicle_x,r.vehicle_y)
    con=sqlite3.connect(db); tx=con.execute("SELECT nodeId,srcIp,pktSeqNum,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall()
    rx=con.execute("SELECT nodeId,srcIp,pktSeqNum FROM pktTxRx WHERE txRx='rx'").fetchall()
    try: avg=np.mean([r[0] for r in con.execute("SELECT avrgPrr FROM avrgPrr").fetchall()])
    except: avg=np.nan
    con.close()
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
    pi=dlv.sum()/att.sum() if att.sum() else np.nan
    pdr=np.divide(dlv,att,out=np.full(len(BINS),np.nan),where=att>=30)
    return pi, avg, pdr, att

def omnet_agg(model,tag):
    v=[]
    for d in sorted(glob.glob(f"{PROJ}/results/inet/Prop_{model}_{tag}-*/")):
        cs=sorted(glob.glob(d+"dist_pdr_*.csv"))
        if cs:
            x=pd.read_csv(cs[-1]); w=x[x.dist_bin_m<300]
            if w.attempts.sum(): v.append(w.delivered.sum()/w.attempts.sum())
    return np.mean(v) if v else np.nan

def omnet_dist(model,tag):
    cs=[]
    for d in sorted(glob.glob(f"{PROJ}/results/inet/Prop_{model}_{tag}-*/")):
        f=sorted(glob.glob(d+"dist_pdr_*.csv"))
        if f: cs.append(pd.read_csv(f[-1])[["dist_bin_m","attempts","delivered"]])
    if not cs: return None
    g=pd.concat(cs).groupby("dist_bin_m").agg(a=("attempts","sum"),dl=("delivered","sum"))
    return g

RT=[(5,"CAV05"),(25,"CAV25"),(50,"CAV50"),(100,"CAV100")]
rows=[]; nd=[]; od=[]
for r,tag in RT:
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-V2V_Urban-*.db")[0]
    pi,avg,pdr,att=ns3_recon(f"{NS3}/inputs/cav{r:02d}_s0_meta.csv",f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv",db)
    rows.append(dict(rate=r, ns3_perinstant=pi, ns3_avrgprr=avg,
                     analytical=omnet_agg("Analytical",tag), generalized=omnet_agg("Generalized",tag),
                     ns3learn=omnet_agg("NS3Learn",tag)))
    for i,b in enumerate(BINS):
        if not np.isnan(pdr[i]): nd.append(dict(rate=r,dist=int(b),pdr=pdr[i]))
    g=omnet_dist("NS3Learn",tag)
    if g is not None:
        for b,rr in g.iterrows():
            if rr.a>=30: od.append(dict(rate=r,dist=int(b),pdr=rr.dl/rr.a))
    print(f"cav{r:02d}: NS3 per-instant={pi:.3f} avrgPrr={avg:.3f} | Analyt={rows[-1]['analytical']:.3f} Gen={rows[-1]['generalized']:.3f} NS3L={rows[-1]['ns3learn']:.3f}")
pd.DataFrame(rows).to_csv(f"{OUT}/per_instant_by_rate.csv",index=False)
pd.DataFrame(nd).to_csv(f"{OUT}/ns3_distance.csv",index=False)
pd.DataFrame(od).to_csv(f"{OUT}/omnet_distance.csv",index=False)

# mainstreet (validation) per-instant
ms=[]
attm=np.zeros(len(BINS)); dlvm=np.zeros(len(BINS))
for s in range(3):
    db=glob.glob(f"{NS3}/results/mainstreet_val/mainstreet50_s{s}-V2V_Urban-*.db")
    if db:
        pi,avg,pdr,att=ns3_recon(f"{NS3}/inputs/mainstreet50_s{s}_meta.csv",f"{PROJ}/ns3_mobility/mainstreet50/seed{s}/fcd.csv",db[0])
pd.DataFrame([dict(model="NS-3",val=np.nan)]).to_csv(f"{OUT}/_placeholder.csv",index=False)
print("cached ->",OUT)
print(pd.DataFrame(rows).round(3).to_string(index=False))
