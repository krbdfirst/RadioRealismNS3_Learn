#!/usr/bin/env python3
"""
Network-side analysis: does the communication model (M3 vs NS3Learn) change the
TRAFFIC DYNAMICS via CAM-driven CACC<->ACC degradation? Reads the vehicle_timeseries
of the *_Net_* (enableCamCacc) runs and reports, per adoption rate, both models:
  - ACC fraction (overall, and CAV-leader-only = the isolated comm effect)
  - MRM fraction (SPaT lost near the intersection -> minimal-risk stop)
  - median time headway (CACC~0.6 vs ACC~1.4 signature) and mean CAV speed
  - throughput proxy (mean CAV speed x running fraction)
Shows the M3-vs-NS3Learn gap growing with adoption. Figure to results/deviation/.
"""
import pandas as pd, glob, numpy as np, os
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
HERE=os.path.dirname(os.path.abspath(__file__)); INET=os.path.join(HERE,"results","inet")
OUT=os.path.join(HERE,"results","deviation"); os.makedirs(OUT,exist_ok=True)
RATES=[5,25,50,75,100]

def metrics(model, rate):
    rows=[]
    for f in glob.glob(f"{INET}/Prop_{model}_Net_CAV{rate:02d}-*/vehicle_timeseries_*.csv"):
        d=pd.read_csv(f); d=d[(d.time>40)&(d.ego_is_cav==1)]
        if not len(d): continue
        mv=d[(d.speed_mps>1.0)&(d.gap_to_leader_m>=0)&(d.gap_to_leader_m<80)]
        dc=d[d.leader_is_cav==1]
        rows.append(dict(
            acc=d.cam_degraded_acc.mean(),
            acc_cavldr=(dc.cam_degraded_acc.mean() if len(dc) else np.nan),
            mrm=d.mrm_active.mean(),
            headway=(mv.gap_to_leader_m/mv.speed_mps).median() if len(mv) else np.nan,
            speed=d.speed_mps.mean(),
            running=(d.speed_mps>0.5).mean(),
            ldr_is_cav=d.leader_is_cav.mean()))
    if not rows: return None
    r=pd.DataFrame(rows); return r.mean(), r.std(), len(r)

res={m:{} for m in ["Analytical_M3","NS3Learn"]}
print(f"{'rate':>4} {'model':>10} {'ACC':>6} {'ACC|CAVldr':>10} {'mrm':>6} {'headwy':>7} {'speed':>6} {'ldrCAV':>7} {'seeds':>5}")
for rate in RATES:
    for model in ["Analytical_M3","NS3Learn"]:
        r=metrics(model,rate)
        if r is None: continue
        m,s,n=r; res[model][rate]=m
        print(f"{rate:>4} {model:>10} {m['acc']:>6.3f} {m['acc_cavldr']:>10.3f} {m['mrm']:>6.3f} "
              f"{m['headway']:>7.2f} {m['speed']:>6.2f} {m['ldr_is_cav']:>7.3f} {n:>5}")

# figure: 4 panels vs adoption, M3 vs NS3Learn
fig,ax=plt.subplots(1,4,figsize=(18,4.2))
xr=[r for r in RATES if r in res["Analytical_M3"] and r in res["NS3Learn"]]
def series(model,key): return [res[model][r][key] for r in xr]
for key,ttl,i in [("acc_cavldr","ACC frac | CAV leader (comm effect)",0),("mrm","MRM frac (SPaT-loss stop)",1),
                  ("speed","mean CAV speed (m/s)",2),("headway","median time headway (s)",3)]:
    ax[i].plot(xr,series("Analytical_M3",key),"o-",color="#0072B2",label="M3")
    ax[i].plot(xr,series("NS3Learn",key),"s-",color="#D55E00",label="NS3Learn")
    ax[i].set(title=ttl,xlabel="CAV rate (%)"); ax[i].legend()
fig.suptitle("Network-side: communication model -> traffic dynamics (CAM-driven CACC<->ACC)")
fig.tight_layout(); fig.savefig(f"{OUT}/netside_analysis.png",dpi=150)
print(f"\nfigure -> {OUT}/netside_analysis.png")
