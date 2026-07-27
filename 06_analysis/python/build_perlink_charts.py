#!/usr/bin/env python3
"""
Per-link / distribution charts for the LOS-only 4-way comparison
(Analytical, Generalized, NS3-Learn vs NS-3 UMi_StreetCanyon_LoS).
Richer than the mean curves — shows spread, SINR clouds, BLER, latency.

Outputs -> results/prop_compare_figures/perlink/
  pl1_sinr_cloud_bler.png      NS-3 per-reception SINR distribution + decode(SINR) waterfall
  pl2_pdr_seed_scatter.png     OMNeT PDR vs distance, every seed as a faint line + mean (per method)
  pl3_pdr_distribution.png     per-distance PDR box/violin across seeds (per method) at a mid rate
  pl4_latency_cdf.png          NS-3 per-link RLC latency CDF by rate
Re-runnable on whatever data exists.
"""
from __future__ import annotations
import glob, os, sqlite3
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

from rr_paths import NS3_RESULTS, PROJECT_DIR
PROJ = f"{PROJECT_DIR}"
INET = os.path.join(PROJ, "results", "inet")
TEACH = f"{NS3_RESULTS}/teacher"
SCEN = "UMi_StreetCanyon_LoS"
OUT = os.path.join(PROJ, "results", "prop_compare_figures", "perlink")
os.makedirs(OUT, exist_ok=True)
RATES = [1, 5, 25, 50, 75, 100]
RTAG = {1:"CAV01",5:"CAV05",25:"CAV25",50:"CAV50",75:"CAV75",100:"CAV100"}
METHODS = ["Analytical", "Generalized", "NS3Learn"]
COL = {"Analytical":"#ff7f0e","Generalized":"#d62728","NS3Learn":"#9467bd","NS-3":"#000"}

def _phy_dbs(rate):
    return sorted(glob.glob(f"{TEACH}/cav{rate:02d}_s*-{SCEN}-propagation-compare.db"))

# ---- pl1: NS-3 SINR cloud + BLER waterfall (from low-rate phyTraces) ----
def pl1():
    pts=[]
    for r in (1,5,10):
        for db in _phy_dbs(r):
            con=sqlite3.connect(db)
            try:
                if con.execute("SELECT COUNT(*) FROM psschRxUePhy").fetchone()[0]==0: continue
                pts.append(np.array(con.execute("SELECT avrgSinr,psschCorrupt FROM psschRxUePhy").fetchall(),float))
            except sqlite3.Error: pass
            finally: con.close()
    if not pts: print("pl1: no phyTraces"); return
    D=np.vstack(pts)
    fig,ax=plt.subplots(figsize=(8.5,5.5)); ax2=ax.twinx()
    edges=np.arange(-15,40,2.5); cent=edges[:-1]+1.25
    h,_=np.histogram(D[:,0],bins=edges); ax2.bar(cent,h/h.sum(),width=2.2,color="#cfe3f7",label="SINR distribution")
    dec=[1-D[(D[:,0]>=lo)&(D[:,0]<lo+2.5),1].mean() if ((D[:,0]>=lo)&(D[:,0]<lo+2.5)).sum()>30 else np.nan for lo in edges[:-1]]
    ax.plot(cent,dec,"-o",color="k",lw=2.3,ms=4,label="PSSCH decode P(SINR)")
    ax.set_xlabel("per-reception SINR (dB)"); ax.set_ylabel("decode probability"); ax.set_ylim(0,1.02)
    ax2.set_ylabel("share of receptions",color="#5a8fc7")
    ax.set_title("NS-3 UMi-LoS per-reception SINR cloud + BLER waterfall (PHY realism)")
    ax.legend(loc="center right",fontsize=9); ax2.legend(loc="upper right",fontsize=9); ax.grid(alpha=.3)
    ax.set_zorder(ax2.get_zorder()+1); ax.patch.set_visible(False)
    fig.tight_layout(); p=f"{OUT}/pl1_sinr_cloud_bler.png"; fig.savefig(p,dpi=150); plt.close(fig); print("wrote",p)

def _omnet_curves(method,rate):
    out=[]
    for d in sorted(glob.glob(f"{INET}/Prop_{method}_{RTAG[rate]}-*/")):
        cs=sorted(glob.glob(d+"dist_pdr_*.csv"))
        if cs:
            x=pd.read_csv(cs[-1]); x=x[x.attempts>=20]
            out.append(x[["dist_bin_m"]].assign(pdr=x.delivered/x.attempts))
    return out

# ---- pl2: per-seed PDR-vs-distance (spread behind the mean) ----
def pl2():
    fig,axes=plt.subplots(2,3,figsize=(13,7.5),sharex=True,sharey=True)
    for ax,r in zip(axes.ravel(),RATES):
        for m in METHODS:
            cs=_omnet_curves(m,r)
            for c in cs: ax.plot(c.dist_bin_m,c.pdr,color=COL[m],lw=0.5,alpha=0.25)
            if cs:
                allc=pd.concat(cs).groupby("dist_bin_m").pdr.mean()
                ax.plot(allc.index,allc.values,color=COL[m],lw=2,label=m)
        ax.set_title(f"{r}% CAV",fontsize=10); ax.set_ylim(0,1.02); ax.set_xlim(0,460); ax.grid(alpha=.3)
    axes[0,0].legend(fontsize=8)
    for ax in axes[-1]: ax.set_xlabel("distance (m)")
    for ax in axes[:,0]: ax.set_ylabel("PDR")
    fig.suptitle("OMNeT LOS-only PDR vs distance — per-seed spread (faint) + mean (per method)",y=0.99)
    fig.tight_layout(rect=(0,0,1,0.97)); p=f"{OUT}/pl2_pdr_seed_scatter.png"; fig.savefig(p,dpi=150); plt.close(fig); print("wrote",p)

# ---- pl3: per-distance PDR distribution (box across seeds) at cav25 ----
def pl3(rate=25):
    bins=[0,50,100,150,200,250,300]; fig,ax=plt.subplots(figsize=(11,5.5))
    width=0.22
    for i,m in enumerate(METHODS):
        cs=_omnet_curves(m,rate)
        if not cs: continue
        data=[]
        for lo,hi in zip(bins[:-1],bins[1:]):
            vals=[c[(c.dist_bin_m>=lo)&(c.dist_bin_m<hi)].pdr.mean() for c in cs]
            data.append([v for v in vals if v==v])
        pos=np.arange(len(bins)-1)+(i-1)*width
        bp=ax.boxplot(data,positions=pos,widths=width,patch_artist=True,showfliers=False)
        for b in bp["boxes"]: b.set_facecolor(COL[m]); b.set_alpha(0.6)
        ax.plot([],[],color=COL[m],lw=6,alpha=0.6,label=m)
    ax.set_xticks(np.arange(len(bins)-1)); ax.set_xticklabels([f"{lo}-{hi}" for lo,hi in zip(bins[:-1],bins[1:])])
    ax.set_xlabel("distance bin (m)"); ax.set_ylabel("PDR (per seed)"); ax.set_ylim(0,1.02)
    ax.set_title(f"PDR distribution across seeds by distance — {rate}% CAV (LOS-only)"); ax.legend(); ax.grid(axis="y",alpha=.3)
    fig.tight_layout(); p=f"{OUT}/pl3_pdr_distribution.png"; fig.savefig(p,dpi=150); plt.close(fig); print("wrote",p)

# ---- pl4: NS-3 per-link latency CDF (rlcRx, low rates have it) ----
def pl4():
    fig,ax=plt.subplots(figsize=(8.5,5.5)); any_=False
    for r in (1,5,10):
        lat=[]
        for db in _phy_dbs(r):
            con=sqlite3.connect(db)
            try:
                rows=con.execute("SELECT delayMicroSec FROM rlcRx").fetchall(); lat+=[x[0]/1000.0 for x in rows]
            except sqlite3.Error: pass
            finally: con.close()
        if len(lat)>50:
            lat=np.sort(np.array(lat)); ax.plot(lat,np.linspace(0,1,len(lat)),lw=2,label=f"cav{r:02d} (n={len(lat)})"); any_=True
    if not any_: print("pl4: no rlcRx latency"); plt.close(fig); return
    ax.set_xlabel("per-link RLC latency (ms)"); ax.set_ylabel("CDF"); ax.set_xlim(left=0)
    ax.set_title("NS-3 UMi-LoS per-link latency CDF by adoption"); ax.legend(); ax.grid(alpha=.3)
    fig.tight_layout(); p=f"{OUT}/pl4_latency_cdf.png"; fig.savefig(p,dpi=150); plt.close(fig); print("wrote",p)

if __name__=="__main__":
    pl1(); pl2(); pl3(); pl4()
