#!/usr/bin/env python3
"""
Cascade loss attribution per CAV density (for the rebuilt F2 'reliability budget' stacked bar).
Sequential multiplicative->additive split that sums to 1.0:
  lost_hd        = hd(n)
  lost_collision = (1-hd)*E_d[ coll(n)*(1-decode(SINRn(d)-cap_mu)) ]
  lost_decode    = (1-hd)*E_d[ (1-coll(n))*(1-decode(SINRn(d))) ]
  delivered      = (1-hd)*E_d[ (1-coll)*decode(SINRn) + coll*decode(SINRn-cap_mu) ]
weighted over the NS-3 in-range distance distribution att(d). decode/coll/hd = learned cascade.
-> results/paper_figures/data/loss_breakdown.csv
"""
import sqlite3, glob, numpy as np, pandas as pd
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
PROJ=f"{PROJECT_DIR}"
OUT=f"{PROJ}/results/paper_figures/data"; BINS=np.arange(0,320,20)
FC,NF,BW,TX=5.9,7.0,18.72e6,23.0; NOISE=-174+10*np.log10(BW)+NF
p=dict(zip(pd.read_csv(f"{PROJ}/results/teacher_dataset/ns3learn_runtime/cascade_params.csv").key,
           pd.read_csv(f"{PROJ}/results/teacher_dataset/ns3learn_runtime/cascade_params.csv").value))
sig=lambda c0,c1,c2,x:1.0/(1.0+np.exp(-(c0+c1*x+c2*x*x)))
def decnc(s): return sig(p["decnc_c0"],p["decnc_c1"],p.get("decnc_c2",0),s)   # decode | no collision
def deccc(s): return sig(p["deccc_c0"],p["deccc_c1"],p.get("deccc_c2",0),s)   # decode | collision (capture)
def pl_v2v(d):  # mean-blend for the (illustrative) attribution
    d=np.maximum(d,1.0); lg=np.log10(d); plL=38.77+16.7*lg+18.2*np.log10(FC)
    nl=9+np.maximum(0,15*lg-41); pL=np.minimum(1,1.05*np.exp(-0.0114*d)); return pL*plL+(1-pL)*(plL+nl)

def att_density(meta,fcdfile,db):
    m=pd.read_csv(meta); n2v=dict(zip(m.nodeId,m.vehId)); dep=dict(zip(m.nodeId,m.departTime)); arr=dict(zip(m.nodeId,m.arriveTime))
    f=pd.read_csv(fcdfile); f=f[f.vehicle_type=="veh_av"]; pos={}
    for r in f.itertuples(index=False): pos.setdefault(r.vehicle_id,{})[round(r.timestep_time,1)]=(r.vehicle_x,r.vehicle_y)
    con=sqlite3.connect(db); tx=con.execute("SELECT nodeId,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall(); con.close()
    nodes=list(n2v); att=np.zeros(len(BINS)); nc=[]
    for sn,t in tx:
        tr=round(t,1); sp=pos.get(n2v.get(sn),{}).get(tr)
        if sp is None: continue
        inr=0
        for rn in nodes:
            if rn==sn or not(dep[rn]<=t<=arr[rn]): continue
            rp=pos.get(n2v[rn],{}).get(tr)
            if rp is None: continue
            d=np.hypot(sp[0]-rp[0],sp[1]-rp[1])
            if d>300: continue
            inr+=1; att[min(int(d//20),len(BINS)-1)]+=1
        nc.append(inr)
    return att, float(np.mean(nc))

rows=[]
for r,tag in [(5,"CAV05"),(25,"CAV25"),(50,"CAV50"),(100,"CAV100")]:
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-V2V_Urban-*.db")[0]
    att,n=att_density(f"{NS3}/inputs/cav{r:02d}_s0_meta.csv",f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv",db)
    w=att/att.sum(); dc=BINS+10
    hd=sig(p["hd_c0"],p["hd_c1"],p.get("hd_c2",0),n); cl=sig(p["col_c0"],p["col_c1"],p.get("col_c2",0),n)
    sn=TX-pl_v2v(dc)-NOISE
    d_nc=decnc(sn); d_cc=deccc(sn)   # grounded: measured decode curves, no fitted penalty
    lost_dec  =(1-hd)*np.sum(w*(1-cl)*(1-d_nc))
    lost_coll =(1-hd)*np.sum(w*cl*(1-d_cc))
    delivered =(1-hd)*np.sum(w*((1-cl)*d_nc+cl*d_cc))
    rows.append(dict(rate=r,density=round(n,1),delivered=delivered,lost_collision=lost_coll,
                     lost_halfduplex=hd,lost_decode=lost_dec))
    print(f"cav{r:02d} n={n:.0f}: deliv={delivered:.2f} coll={lost_coll:.2f} hd={hd:.2f} dec={lost_dec:.2f} sum={delivered+lost_coll+hd+lost_dec:.2f}")
pd.DataFrame(rows).to_csv(f"{OUT}/loss_breakdown.csv",index=False); print("-> loss_breakdown.csv")
