#!/usr/bin/env python3
"""
Show BOTH NS-3 KPIs (per-instant in-range PDR vs the avrgPrr table) against the three
OMNeT models, so the avrgPrr deflation (transient-neighbour denominator) is visible and
its effect on which model 'matches' is explicit.
-> results/prop_compare_figures/focus6_metric_comparison.png
"""
import pandas as pd, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rr_paths import PROJECT_DIR
PROJ=f"{PROJECT_DIR}"
d=pd.read_csv(f"{PROJ}/results/prop_compare_figures/perinstant_vs_avrgprr.csv").sort_values("rate")
x=d.rate.values

fig,(ax1,ax2)=plt.subplots(1,2,figsize=(15,6),sharey=True)

# Panel 1: per-instant in-range PDR (3GPP-standard, apples-to-apples with OMNeT dist_pdr)
ax1.plot(x,d.ns3_perinstant,"ks-",lw=2.6,ms=7,label="NS-3 5G-LENA (per-instant)")
ax1.plot(x,d.omnet_analytical,"o-",color="#ff7f0e",lw=2,ms=6,label="OMNeT Analytical")
ax1.plot(x,d.omnet_generalized,"^-",color="#d62728",lw=2,ms=6,label="OMNeT Generalized")
ax1.plot(x,d.omnet_ns3learn,"D-",color="#9467bd",lw=2,ms=6,label="OMNeT NS3-Learn (avrgPrr-cal)")
ax1.set_title("Per-instant in-range PDR (3GPP-standard)\nNS-3 degrades 0.96→0.20 with density; Analytical ~flat misses congestion")

# Panel 2: avrgPrr KPI (deflated by transient-neighbour denominator)
ax2.plot(x,d.ns3_avrgPrr,"ks--",lw=2.6,ms=7,label="NS-3 5G-LENA (avrgPrr table)")
ax2.plot(x,d.ns3_perinstant,"-",color="grey",lw=1.4,alpha=.6,label="NS-3 per-instant (ref)")
ax2.plot(x,d.omnet_ns3learn,"D-",color="#9467bd",lw=2,ms=6,label="OMNeT NS3-Learn (avrgPrr-cal)")
ax2.set_title("avrgPrr KPI (5G-LENA table)\ndeflated ~3× by cumulative-neighbour denominator; NS3-Learn was tuned to this")

for ax in (ax1,ax2):
    ax.set_xlabel("CAV penetration (%)"); ax.set_xscale("log"); ax.set_xticks(x); ax.set_xticklabels([str(int(r)) for r in x])
    ax.set_ylim(0,1.02); ax.grid(alpha=.3); ax.legend(fontsize=9)
ax1.set_ylabel("PDR")
fig.suptitle("Two NS-3 KPIs vs OMNeT models (intersection, LOS-only) — the metric choice changes the story",y=1.01,fontsize=13)
fig.tight_layout()
p=f"{PROJ}/results/prop_compare_figures/focus6_metric_comparison.png"; fig.savefig(p,dpi=150,bbox_inches="tight"); print("wrote",p)
