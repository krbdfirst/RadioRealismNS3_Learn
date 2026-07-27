#!/usr/bin/env python3
"""
Calibrate STOCHASTIC interference (mu(density), sigma) so NS3Learn reproduces NS-3's
GRADUAL PDR(distance) decay, not a cliff. Per reception the interference penalty is
I ~ Normal(mu(density), sigma^2); PDR(d) = E_I[ sci*pssch(SINR_noise(d) - I) ].
Fit mu,sigma per rate to the reconstructed NS-3 PDR(d), check it matches BOTH the
distance shape and the avrgPrr aggregate. If good -> implement sigma in OMNeT.
"""
import sqlite3, glob, os, numpy as np, pandas as pd
from scipy.optimize import minimize
from scipy.interpolate import interp1d
from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3=f"{NS3_SCRATCH}"
PROJ=f"{PROJECT_DIR}"
B=f"{PROJ}/results/teacher_dataset/ns3learn_runtime"
FC,NF,BW,TX=5.9,7.0,18.72e6,23.0; NOISE=-174+10*np.log10(BW)+NF; lg=np.log10
BINS=np.arange(0,320,20)
def pl(d): d=np.maximum(d,10.0); return 32.4+21*lg(d)+20*lg(FC)
bp=pd.read_csv(f"{B}/bler_pssch.csv"); bsc=pd.read_csv(f"{B}/bler_sci.csv")
pssch=interp1d(bp.snr_db,1-bp.bler,bounds_error=False,fill_value=(0.,float(1-bp.bler.iloc[-1])))
sci=interp1d(bsc.snr_db,1-bsc.bler,bounds_error=False,fill_value=(0.,float(1-bsc.bler.iloc[-1])))
# Gauss-Hermite nodes for E_I over Normal(mu,sigma)
gh_x,gh_w=np.polynomial.hermite_e.hermegauss(15); gh_w=gh_w/np.sqrt(2*np.pi)
def pdr_d(d,mu,sigma):
    sn=TX-pl(d)-NOISE
    out=np.zeros_like(np.atleast_1d(d),dtype=float)
    for x,w in zip(gh_x,gh_w):
        I=mu+sigma*x; se=sn-I; out+=w*np.clip(pssch(se)*sci(se),0,1)
    return out

def recon(r):
    me=f"{NS3}/inputs/cav{r:02d}_s0_meta.csv"; fc=f"{PROJ}/ns3_mobility/cav{r:02d}/seed0/fcd_cav{r:02d}.csv"
    db=glob.glob(f"{NS3}/results/teacher/cav{r:02d}_s0-UMi_StreetCanyon_LoS-*.db")
    if not(db and os.path.exists(me) and os.path.exists(fc)): return None,None,None
    meta=pd.read_csv(me); n2v=dict(zip(meta.nodeId,meta.vehId))
    dep=dict(zip(meta.nodeId,meta.departTime)); arr=dict(zip(meta.nodeId,meta.arriveTime))
    f=pd.read_csv(fc); f=f[f.vehicle_type=="veh_av"]; pos={}
    for vid,t,x,y in zip(f.vehicle_id,f.timestep_time,f.vehicle_x,f.vehicle_y): pos.setdefault(vid,{})[round(t,1)]=(x,y)
    con=sqlite3.connect(db[0])
    tx=con.execute("SELECT nodeId,srcIp,pktSeqNum,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall()
    rx=con.execute("SELECT nodeId,srcIp,pktSeqNum FROM pktTxRx WHERE txRx='rx'").fetchall(); con.close()
    dl={}
    for rn,sip,seq in rx: dl.setdefault((sip,seq),set()).add(rn)
    nodes=list(n2v); att=np.zeros(len(BINS)); dlv=np.zeros(len(BINS)); nc=[]
    for sn,sip,seq,t in tx:
        tr=round(t,1); sp=pos.get(n2v.get(sn),{}).get(tr)
        if sp is None: continue
        got=dl.get((sip,seq),set()); inr=0
        for rn in nodes:
            if rn==sn or not(dep[rn]<=t<=arr[rn]): continue
            rp=pos.get(n2v[rn],{}).get(tr)
            if rp is None: continue
            d=np.hypot(sp[0]-rp[0],sp[1]-rp[1])
            if d<=320: inr+=1
            b=min(int(d//20),len(BINS)-1); att[b]+=1
            if rn in got: dlv[b]+=1
        nc.append(inr)
    pdr=np.divide(dlv,att,out=np.full(len(BINS),np.nan),where=att>=30)
    return pdr,att,float(np.mean(nc))

ns3=pd.read_csv(f"{PROJ}/results/prop_compare_figures/ns3_prr300_per_run.csv"); tgt=ns3.groupby('rate').prr300.mean().to_dict()
rows=[]
for r in [5,25,50,100]:
    pdr,att,dens=recon(r)
    if pdr is None: continue
    m=~np.isnan(pdr); dv=BINS[m]+10; pv=pdr[m]; wt=att[m]
    def loss(p):
        mu,sg=p
        if sg<0.5: return 1e6
        return np.sum(wt*(pdr_d(dv,mu,sg)-pv)**2)
    best=None
    for mu0 in (15,25,35):
        rr=minimize(loss,[mu0,12],method="Nelder-Mead")
        if best is None or rr.fun<best.fun: best=rr
    mu,sg=best.x
    # aggregate over in-range attempts
    agg=np.sum(att[BINS<300]*pdr_d(BINS[BINS<300]+10,mu,sg))/att[BINS<300].sum()
    rows.append(dict(rate=r,density=dens,mu=mu,sigma=sg,agg_model=agg,agg_ns3=tgt[r]))
    shp={int(BINS[i]):round(float(pdr_d(BINS[i]+10,mu,sg)),2) for i in (0,2,4,6,8)}
    nshp={int(BINS[i]):round(float(pdr[i]),2) for i in (0,2,4,6,8)}
    print(f"cav{r:02d}: dens={dens:.1f} mu={mu:.1f} sigma={sg:.1f} | aggModel={agg:.3f} NS3={tgt[r]:.3f}")
    print(f"   shape model={shp}"); print(f"   shape NS-3 ={nshp}")
R=pd.DataFrame(rows)
A=np.vstack([np.log10(1+R.density),np.ones(len(R))]).T
(a,b_),_,_,_=np.linalg.lstsq(A,R.mu,rcond=None)
(sa,sb),_,_,_=np.linalg.lstsq(A,R.sigma,rcond=None)
print(f"\nmu(density)    = {a:.2f}*log10(1+n) + {b_:.2f}")
print(f"sigma(density) = {sa:.2f}*log10(1+n) + {sb:.2f}")
def mu(n): return a*np.log10(1+n)+b_
def sg(n): return max(0.5, sa*np.log10(1+n)+sb)
print("fit check (mu,sigma):", {f"cav{int(r.rate):02d}":f"mu {mu(r.density):.1f}/{r.mu:.1f}, sg {sg(r.density):.1f}/{r.sigma:.1f}" for _,r in R.iterrows()})
R.to_csv(f"{B}/interference_stochastic_fit.csv",index=False)
# write stochastic params into the runtime bundle
df=pd.read_csv(f"{B}/ns3learn_params.csv"); d=dict(zip(df.key,df.value))
d["intf_a"]=round(float(a),3); d["intf_b"]=round(float(b_),3)
d["intf_sigma_a"]=round(float(sa),3); d["intf_sigma_b"]=round(float(sb),3)
d["distance_aware"]=1.0; d["stochastic"]=1.0
pd.DataFrame(list(d.items()),columns=["key","value"]).to_csv(f"{B}/ns3learn_params.csv",index=False)
print("\nupdated bundle: intf_a,intf_b (mu) + intf_sigma_a,intf_sigma_b (sigma), stochastic=1")
