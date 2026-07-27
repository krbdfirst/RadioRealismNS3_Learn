#!/usr/bin/env python3
"""
Contribution-1 publication figure set (Q1-journal quality), rendered in BOTH a colour
(Okabe-Ito colourblind-safe) and a grayscale variant. Reads cached data from
results/paper_figures/data/ + the cascade datasets / learner CSVs.
-> results/paper_figures/{color,gray}/F*.png  (300 dpi PNG; no PDF)
"""
import os, glob, numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from rr_paths import PROJECT_DIR
PROJ=f"{PROJECT_DIR}"
D=f"{PROJ}/results/paper_figures/data"; DS=f"{PROJ}/results/teacher_dataset/cascade"
PI=f"{PROJ}/results/prop_compare_figures/perinstant"

def style():
    plt.rcParams.update({
        "font.family":"serif","font.serif":["Times New Roman","Times","DejaVu Serif"],
        "mathtext.fontset":"stix",
        "font.size":11,"axes.labelsize":12,"axes.titlesize":12.5,"legend.fontsize":9.5,
        "xtick.labelsize":10,"ytick.labelsize":10,"axes.linewidth":0.8,
        "lines.linewidth":2.0,"lines.markersize":6,"savefig.dpi":300,"figure.dpi":150,
        "savefig.bbox":"tight","axes.grid":True,"grid.alpha":0.22,"grid.linewidth":0.5,
        "grid.color":"#C9C9C9","axes.axisbelow":True,
        "axes.titleweight":"medium","figure.titleweight":"semibold",
        "axes.edgecolor":"#3A3A3A","xtick.color":"#222222","ytick.color":"#222222",
        "text.color":"#1A1A1A","axes.labelcolor":"#1A1A1A",
        "legend.frameon":True,"legend.framealpha":0.9,"legend.edgecolor":"#B5B5B5",
        "legend.fancybox":False,"legend.borderpad":0.5})

# harmonised palette: red + blue primary; black = ground truth; gray = neutral/minor
# series styles: (color, gray, marker, linestyle)
ST={"NS-3":      ("#1A1A1A","#000000","o","-"),
    "Analytical":("#C42E2E","#7f7f7f","s","--"),
    "Generalized":("#E0746B","#b0b0b0","^",":"),
    "NS3Learn":  ("#1F6FB2","#404040","D","-"),
    "avrgPrr":   ("#8C8C8C","#999999","v","--"),
    "Linear":    ("#8C8C8C","#9a9a9a","s","--"),
    "Logistic":  ("#1F6FB2","#000000","o","-"),
    "GBM":       ("#C42E2E","#5a5a5a","^","-."),
    "intersection":("#1F6FB2","#000000","o","-"),
    "mainstreet":("#C42E2E","#888888","s","-")}
def col(k,gray): return ST[k][1] if gray else ST[k][0]
def mk(k): return ST[k][2]
def ls(k): return ST[k][3]
def sig(c0,c1,c2,x): return 1.0/(1.0+np.exp(-(c0+c1*x+c2*x*x)))
def panel(ax,lab): ax.text(-0.16,1.04,lab,transform=ax.transAxes,fontweight="bold",fontsize=13,va="top")
def wilson(k,n,z=1.96):
    if n==0: return (np.nan,np.nan)
    pp=k/n; d=1+z*z/n; c=(pp+z*z/(2*n))/d; m=(z/d)*np.sqrt(pp*(1-pp)/n+z*z/(4*n*n)); return (max(0,c-m),min(1,c+m))
def binprop(xv,yv,edges):
    xv=np.asarray(xv,float); yv=np.asarray(yv,float); xc,mn,lo,hi,ct=[],[],[],[],[]
    for a,b in zip(edges[:-1],edges[1:]):
        m=(xv>=a)&(xv<b); n=int(m.sum())
        if n<30: continue
        k=int(yv[m].sum()); p=k/n; l,h=wilson(k,n)
        xc.append((a+b)/2); mn.append(p); lo.append(max(0.0,p-l)); hi.append(max(0.0,h-p)); ct.append(n)
    return np.array(xc),np.array(mn),np.array([lo,hi]),np.array(ct)

def make(gray):
    sub="gray" if gray else "color"; OUT=f"{PROJ}/results/paper_figures/{sub}"; os.makedirs(OUT,exist_ok=True)
    style()
    pir=pd.read_csv(f"{D}/per_instant_by_rate.csv"); x=pir.rate.values
    xt=[5,25,50,100]
    C=pd.read_csv(f"{DS}/collision_dataset.csv"); H=pd.read_csv(f"{DS}/halfduplex_dataset.csv")
    De=pd.read_csv(f"{DS}/decode_dataset.csv"); CAP=pd.read_csv(f"{DS}/capture_dataset.csv")

    # ---- F1: realism gap + metric correction ----
    fig,(a,b)=plt.subplots(1,2,figsize=(10,4.1))
    for k,c in [("NS-3","ns3_perinstant"),("NS3Learn","ns3learn"),("Analytical","analytical"),("Generalized","generalized")]:
        a.plot(x,pir[c],marker=mk(k),ls=ls(k),color=col(k,gray),label=("NS-3 5G-LENA" if k=="NS-3" else ("NS3-Learn" if k=="NS3Learn" else k)),
               lw=2.4 if k in("NS-3","NS3Learn") else 1.8,ms=7 if k=="NS-3" else 6,mfc="none" if k in("Analytical","Generalized") else None)
    a.set_xscale("log"); a.set_xticks(xt); a.set_xticklabels(xt); a.set_xlabel("CAV penetration (%)")
    a.set_ylabel("per-instant PDR (≤300 m)"); a.set_ylim(0,1.03); a.legend(loc="center right"); panel(a,"(a)")
    a.set_title("Realism gap under the correct KPI")
    b.plot(x,pir.ns3_perinstant,marker=mk("NS-3"),ls="-",color=col("NS-3",gray),label="per-instant PDR (3GPP-standard)",lw=2.4,ms=7)
    b.plot(x,pir.ns3_avrgprr,marker=mk("avrgPrr"),ls="--",color=col("avrgPrr",gray),label="avrgPrr (5G-LENA table)",lw=2.0)
    for xi,p,q in zip(x,pir.ns3_perinstant,pir.ns3_avrgprr):
        b.annotate("",xy=(xi,q),xytext=(xi,p),arrowprops=dict(arrowstyle="-",color="0.6",lw=0.8))
    b.set_xscale("log"); b.set_xticks(xt); b.set_xticklabels(xt); b.set_xlabel("CAV penetration (%)")
    b.set_ylabel("NS-3 PDR"); b.set_ylim(0,1.03); b.legend(loc="upper right"); panel(b,"(b)")
    b.set_title("avrgPrr deflates ~3$\\times$ (transient neighbours)")
    fig.tight_layout(); fig.savefig(f"{OUT}/F1_gap_and_metric.png"); plt.close(fig)

    # ---- F2: reliability budget (stacked) — where every transmission goes vs density ----
    LB=pd.read_csv(f"{D}/loss_breakdown.csv").sort_values("rate")
    segs=[("delivered","delivered","#1F6FB2","0.15",""),
          ("lost_decode","PHY decode loss (propagation)","#8FB8DA","0.55","//"),
          ("lost_collision","SB-SPS collision loss (contention)","#C42E2E","0.78","xx"),
          ("lost_halfduplex","half-duplex loss","#8C8C8C","0.92","..")]
    fig,ax=plt.subplots(figsize=(6.6,4.6)); xpos=np.arange(len(LB)); bottom=np.zeros(len(LB)); wbar=0.62
    for key,lab,c,g,h in segs:
        ax.bar(xpos,LB[key],wbar,bottom=bottom,label=lab,color=(g if gray else c),
               edgecolor="white",linewidth=0.8,hatch=(h if gray else None))
        bottom+=LB[key].values
    for i,dv in enumerate(LB.delivered): ax.text(i,dv/2,f"{dv:.2f}",ha="center",va="center",
                                                 color="white",fontweight="bold",fontsize=9.5)
    ax.set_xticks(xpos); ax.set_xticklabels([f"{int(r)}% CAV\n($n\\approx${int(d)})" for r,d in zip(LB.rate,LB.density)])
    ax.set_ylabel("share of transmission opportunities"); ax.set_ylim(0,1.0)
    ax.set_title("Reliability budget vs density — the collapse is MAC contention")
    ax.legend(loc="upper center",bbox_to_anchor=(0.5,-0.12),ncol=2,fontsize=8.5)
    ax.grid(False); ax.margins(x=0.04)
    fig.tight_layout(); fig.savefig(f"{OUT}/F2_reliability_budget.png"); plt.close(fig)

    # ---- F3: distilled cascade stages ----
    p=dict(zip(pd.read_csv(f"{PROJ}/results/teacher_dataset/ns3learn_runtime/cascade_params.csv").key,
               pd.read_csv(f"{PROJ}/results/teacher_dataset/ns3learn_runtime/cascade_params.csv").value))
    rng0=np.random.default_rng(2)
    def rug(ax,xv,lo,hi,xlim,n=1500):
        xv=np.asarray(xv,float); xv=xv[(xv>=xlim[0])&(xv<=xlim[1])]
        i=rng0.choice(len(xv),min(n,len(xv)),replace=False)
        ax.plot(xv[i],np.full(len(i),lo),"|",color="0.6",alpha=.25,ms=6,mew=0.6)
        ax.plot(xv[i],np.full(len(i),hi),"|",color="0.6",alpha=.25,ms=6,mew=0.6)
    fig,axs=plt.subplots(2,2,figsize=(10,8)); cc=col("Logistic",gray); gn=np.arange(0,70); gs=np.linspace(-15,40,200)
    # (a) hd(n): binned proportion + 95% CI + logistic
    ax=axs[0,0]; xc,mn,err,ct=binprop(H.n.values,H.blocked.values,np.arange(0,71,2))
    ax.errorbar(xc,mn,yerr=err,fmt="o",ms=4,color="0.35",ecolor="0.6",elinewidth=1,capsize=2,zorder=3,label="NS-3 (proportion $\\pm$95% CI)")
    ax.plot(gn,sig(p["hd_c0"],p["hd_c1"],p.get("hd_c2",0),gn),color=cc,lw=2.4,label="fitted NS-3 model: $hd(n)$")
    ax.set_xlabel("density $n$"); ax.set_ylabel("$P$(half-duplex)"); ax.set_ylim(0,.25); ax.set_xlim(0,70); ax.legend(); panel(ax,"(a)"); ax.set_title("Half-duplex")
    # (b) collision(n)
    ax=axs[0,1]; xc,mn,err,ct=binprop(C.n.values,C.collided.values,np.arange(0,71,2))
    rug(ax,C.n.values,0.0,1.0,(0,70))
    ax.errorbar(xc,mn,yerr=err,fmt="o",ms=4,color="0.35",ecolor="0.6",elinewidth=1,capsize=2,zorder=3,label="NS-3 (proportion $\\pm$95% CI)")
    ax.plot(gn,sig(p["col_c0"],p["col_c1"],p.get("col_c2",0),gn),color=cc,lw=2.4,label="fitted NS-3 model: collision$(n)$")
    ax.set_xlabel("density $n$"); ax.set_ylabel("$P$(SB-SPS collision)"); ax.set_ylim(-.04,1.04); ax.set_xlim(0,70); ax.legend(loc="lower right"); panel(ax,"(b)"); ax.set_title("SB-SPS collision")
    # density-aware decode: sigmoid(c0+c1 S+c2 S^2 + c3 n + c4 S n); show low- vs high-density curves
    def sig2(pre,s,n): return 1.0/(1.0+np.exp(-(p[pre+"_c0"]+p[pre+"_c1"]*s+p[pre+"_c2"]*s*s+p.get(pre+"_c3",0)*n+p.get(pre+"_c4",0)*s*n)))
    nc=CAP[CAP.collided==0]; ccd=CAP[CAP.collided==1]; gsc=np.linspace(-15,45,200); NLO,NHI=8,58
    # (c) decode | no collision
    ax=axs[1,0]
    for sub,nlab,clr in [(nc[nc.n<=20],"low density","0.35"),(nc[nc.n>=45],"high density","0.6")]:
        if len(sub)>200:
            xc,mn,err,ct=binprop(sub.sinr_db.values,sub.decoded.values,np.arange(-15,41,3))
            ax.errorbar(xc,mn,yerr=err,fmt="o",ms=3,color=clr,ecolor=clr,elinewidth=.8,capsize=1.5,alpha=.7,zorder=3,label=f"NS-3 {nlab}")
    ax.plot(gsc,sig2("decnc",gsc,NLO),color=cc,lw=2.2,label=f"fitted NS-3 model, low density ($n$={NLO})")
    ax.plot(gsc,sig2("decnc",gsc,NHI),color=cc,lw=2.2,ls="--",label=f"fitted NS-3 model, high density ($n$={NHI})")
    ax.set_xlabel("SINR (dB)"); ax.set_ylabel("$P$(decode)"); ax.set_ylim(-.04,1.04); ax.set_xlim(-15,40); ax.legend(loc="lower right",fontsize=7.5); panel(ax,"(c)"); ax.set_title("PHY decode | no collision")
    # (d) capture = decode | collision — weakens with density (the high-density-PHY finding)
    ax=axs[1,1]
    for sub,nlab,clr in [(ccd[ccd.n<=20],"low density","0.35"),(ccd[ccd.n>=45],"high density","0.6")]:
        if len(sub)>200:
            xc,mn,err,ct=binprop(sub.sinr_db.values,sub.decoded.values,np.arange(-15,46,4))
            ax.errorbar(xc,mn,yerr=err,fmt="s",ms=3,color=clr,ecolor=clr,elinewidth=.8,capsize=1.5,alpha=.7,zorder=3,label=f"NS-3 {nlab}")
    ax.plot(gsc,sig2("deccc",gsc,NLO),color=col("GBM",gray),lw=2.2,label=f"fitted NS-3 model, low density ($n$={NLO})")
    ax.plot(gsc,sig2("deccc",gsc,NHI),color=col("GBM",gray),lw=2.2,ls="--",label=f"fitted NS-3 model, high density ($n$={NHI})")
    ax.set_xlabel("SINR (dB)"); ax.set_ylabel("$P$(decode)"); ax.set_ylim(-.04,1.04); ax.set_xlim(-15,45); ax.legend(loc="upper left",fontsize=7.5); panel(ax,"(d)"); ax.set_title("Capture: decode | collision (weakens with density)")
    fig.suptitle("Decomposed cascade — each stage fitted to NS-3 / TR 37.885",y=1.0,fontsize=12.5)
    fig.text(0.5,-0.012,"Markers: empirical proportion $\\pm$95% Wilson CI per 2-unit ($n$) / 2-dB (SINR) bin ($\\geq$30 samples); curves: logistic fits to the full per-event data.",ha="center",fontsize=8.5,style="italic",color="0.35")
    fig.tight_layout(); fig.savefig(f"{OUT}/F3_cascade_stages.png"); plt.close(fig)

    # ---- F3b: same stages, UN-BINNED raw observations (random subsample, jittered strip) ----
    rng=np.random.default_rng(1); gn=np.arange(0,70); gs=np.linspace(-15,40,200)
    def strip(ax,xv,yv,xj,nmax=5000):
        xv=np.asarray(xv,float); yv=np.asarray(yv,float)
        i=rng.choice(len(xv),min(nmax,len(xv)),replace=False)
        ax.scatter(xv[i]+rng.uniform(-xj,xj,len(i)),yv[i]+rng.uniform(-.035,.035,len(i)),
                   s=7,alpha=.08,color="0.45",edgecolor="none",zorder=2,rasterized=True)
    fig,axs=plt.subplots(2,2,figsize=(10,8)); cc=col("Logistic",gray)
    ax=axs[0,0]; strip(ax,H.n.values,H.blocked.values,0.45)
    ax.plot(gn,sig(p["hd_c0"],p["hd_c1"],p.get("hd_c2",0),gn),color=cc,lw=2.6,label="fitted NS-3 model: $hd(n)$")
    ax.set_xlabel("density $n$"); ax.set_ylabel("half-duplex (0/1)"); ax.set_ylim(-.06,1.06); ax.set_xlim(0,70); ax.legend(loc="center right"); panel(ax,"(a)"); ax.set_title(f"Half-duplex  ($N$=10.5M, showing 5k)")
    ax=axs[0,1]; strip(ax,C.n.values,C.collided.values,0.45)
    ax.plot(gn,sig(p["col_c0"],p["col_c1"],p.get("col_c2",0),gn),color=cc,lw=2.6,label="fitted NS-3 model: collision$(n)$")
    ax.set_xlabel("density $n$"); ax.set_ylabel("collision (0/1)"); ax.set_ylim(-.06,1.06); ax.set_xlim(0,70); ax.legend(loc="center right"); panel(ax,"(b)"); ax.set_title(f"SB-SPS collision  ($N$=700k, showing 5k)")
    nc=CAP[CAP.collided==0]; ccd=CAP[CAP.collided==1]
    ax=axs[1,0]; strip(ax,nc.sinr_db.values,nc.decoded.values,0.6)
    ax.plot(gs,sig(p["decnc_c0"],p["decnc_c1"],p.get("decnc_c2",0),gs),color=cc,lw=2.6,label="fitted NS-3 model: decode$_{\\mathrm{noColl}}$")
    ax.set_xlabel("SINR (dB)"); ax.set_ylabel("decode (0/1)"); ax.set_ylim(-.06,1.06); ax.set_xlim(-15,40); ax.legend(loc="center right"); panel(ax,"(c)"); ax.set_title("PHY decode | no collision")
    ax=axs[1,1]; strip(ax,ccd.sinr_db.values,ccd.decoded.values,0.6)
    gsc=np.linspace(-15,45,200); ax.plot(gsc,sig(p["deccc_c0"],p["deccc_c1"],p.get("deccc_c2",0),gsc),color=col("GBM",gray),lw=2.6,label="fitted NS-3 model: decode$_{\\mathrm{coll}}$ (capture)")
    ax.set_xlabel("SINR (dB)"); ax.set_ylabel("decode (0/1)"); ax.set_ylim(-.06,1.06); ax.set_xlim(-15,45); ax.legend(loc="upper left"); panel(ax,"(d)"); ax.set_title("Capture: PHY decode | collision")
    fig.suptitle("Decomposed cascade — raw NS-3 observations (un-binned) + fitted-model curves",y=1.0,fontsize=12.5)
    fig.tight_layout(); fig.savefig(f"{OUT}/F3b_cascade_stages_raw.png"); plt.close(fig)

    # ---- F4: three-learner comparison ----
    M=pd.read_csv(f"{PI}/learner_stage_metrics.csv"); B=pd.read_csv(f"{PI}/c1_bakeoff_table.csv")
    fig,(a,b)=plt.subplots(1,2,figsize=(10,4.2))
    stages=["halfduplex","collision","decode"]; lr=["Linear","Logistic","GBM"]; w=0.26
    for i,L in enumerate(lr):
        vals=[M[(M.stage==s)&(M.model==L)].logloss.values[0] for s in stages]
        a.bar(np.arange(len(stages))+(i-1)*w,vals,w,color=col(L,gray),label=L,
              hatch=["","//","xx"][i] if gray else None,edgecolor="k",linewidth=0.5)
    a.set_xticks(range(len(stages))); a.set_xticklabels(["half-duplex","collision","decode"])
    a.set_ylabel("cross-validated log-loss"); a.set_ylim(0,0.40); a.legend(loc="upper right"); panel(a,"(a)"); a.set_title("Per-stage learner fit")
    B["grp"]=B.geom.str.startswith("int").map({True:"intersection",False:"mainstreet"})
    bb=B.groupby(["learner","grp"]).abserr.mean().unstack()
    geoms=["intersection","mainstreet"]
    for i,L in enumerate(lr):
        a2=[bb.loc[L,g] for g in geoms]
        b.bar(np.arange(len(geoms))+(i-1)*w,a2,w,color=col(L,gray),label=L,
              hatch=["","//","xx"][i] if gray else None,edgecolor="k",linewidth=0.5)
    b.set_xticks(range(len(geoms))); b.set_xticklabels(["intersection\n(train)","mainstreet\n(validate)"])
    b.set_ylabel("per-instant MAE vs NS-3"); b.set_ylim(0,0.13); b.legend(loc="upper left"); panel(b,"(b)"); b.set_title("End-to-end cascade")
    fig.tight_layout(); fig.savefig(f"{OUT}/F4_learner_comparison.png"); plt.close(fig)

    # ---- F4b: alternative — per-density tracking (no misleading aggregate bars) ----
    fig,(a,b)=plt.subplots(1,2,figsize=(11.2,4.5))
    for i,L in enumerate(lr):
        vals=[M[(M.stage==s)&(M.model==L)].logloss.values[0] for s in stages]
        a.bar(np.arange(len(stages))+(i-1)*w,vals,w,color=col(L,gray),label=L,
              hatch=["","//","xx"][i] if gray else None,edgecolor="k",linewidth=0.5)
    a.set_xticks(range(len(stages))); a.set_xticklabels(["half-duplex","collision","decode"])
    a.set_ylabel("cross-validated log-loss"); a.legend(loc="upper right"); panel(a,"(a)"); a.set_title("Per-stage fit")
    a.text(0,0.355,"no learner\nadvantage\n(near-constant)",ha="center",va="bottom",fontsize=7,style="italic",color="0.35")
    a.set_ylim(0,0.46)
    # (b) end-to-end per-instant PDR vs density: NS-3 truth + 3 learner cascades; mainstreet = stars
    Bi=B[B.geom.str.startswith("int")]; ns3=Bi[Bi.learner=="Logistic"].sort_values("density")
    b.plot(ns3.density,ns3.ns3,"-",marker="o",color=col("NS-3",gray),lw=2.6,ms=8,label="NS-3 5G-LENA (truth)",zorder=5)
    for L in lr:
        d=Bi[Bi.learner==L].sort_values("density")
        b.plot(d.density,d.pred,marker=mk(L),ls=ls(L),color=col(L,gray),lw=1.9,ms=6,
               mfc="none" if L=="Linear" else None,label=f"NS3-Learn [{L}]")
    Bm=B[B.geom=="main_50"]
    b.scatter(Bm[Bm.learner=="Logistic"].density,Bm[Bm.learner=="Logistic"].ns3,marker="*",s=260,
              color=col("NS-3",gray),edgecolor="k",linewidth=0.6,zorder=7,label="mainstreet (validation)")
    for L in lr:
        rr=Bm[Bm.learner==L]; b.scatter(rr.density,rr.pred,marker="*",s=150,color=col(L,gray),edgecolor="k",linewidth=0.4,zorder=7)
    b.set_xlabel("CAV neighbour density $n$"); b.set_ylabel("per-instant PDR"); b.set_ylim(0,1.0)
    b.legend(fontsize=8,loc="upper right"); panel(b,"(b)"); b.set_title("End-to-end tracking (lines=train, $\\star$=validation)")
    fig.tight_layout(); fig.savefig(f"{OUT}/F4b_learner_tracking.png"); plt.close(fig)

    # ---- F5: per-tx-rx scatter (hero) ----
    S=pd.read_csv(f"{PI}/per_txrx_scatter_v2v.csv")
    mae=np.mean(np.abs(S.ns3-S.omnet)); r=np.corrcoef(S.ns3,S.omnet)[0,1]
    fig,ax=plt.subplots(figsize=(6.0,5.8))
    for g in ["intersection","mainstreet"]:
        s=S[S.geom==g]; ax.scatter(s.ns3,s.omnet,marker=mk(g),s=58,alpha=.8,
                                   facecolor=col(g,gray) if not gray else "none",edgecolor="k" if not gray else col(g,gray),
                                   linewidth=0.6,label=f"{g} (n={len(s)})")
    ax.plot([0,1],[0,1],"k--",lw=1.4,label="perfect ($y=x$)")
    ax.set_xlim(0,1.02); ax.set_ylim(0,1.02); ax.set_aspect("equal")
    ax.set_xlabel("NS-3 5G-LENA per-instant PDR"); ax.set_ylabel("OMNeT NS3-Learn per-instant PDR")
    ax.set_title(f"Per-tx-rx validation (V2V)\nMAE={mae:.3f}, $r$={r:.3f}, $n$={len(S)} distance-bins")
    ax.legend(loc="upper left"); fig.tight_layout(); fig.savefig(f"{OUT}/F5_per_txrx_scatter.png"); plt.close(fig)

    # ---- F5b: matched-load (50% CAV) geometry comparison — fair, balanced validation ----
    S50=S[S.label.str.startswith("cav50")|(S.geom=="mainstreet")].copy()
    S50["g"]=np.where(S50.label.str.startswith("cav50"),"intersection 50% (train)","mainstreet 50% (validate)")
    mae=np.mean(np.abs(S50.ns3-S50.omnet)); r=np.corrcoef(S50.ns3,S50.omnet)[0,1]
    lim=max(S50.ns3.max(),S50.omnet.max())*1.12
    fig,ax=plt.subplots(figsize=(6.0,5.9))
    for g,key in [("intersection 50% (train)","intersection"),("mainstreet 50% (validate)","mainstreet")]:
        s=S50[S50.g==g]; ax.scatter(s.ns3,s.omnet,marker=mk(key),s=78,alpha=.82,
            facecolor=col(key,gray) if not gray else "none",edgecolor="k" if not gray else col(key,gray),
            linewidth=0.7,label=f"{g}  (n={len(s)}, MAE={np.mean(np.abs(s.ns3-s.omnet)):.3f})")
    ax.plot([0,lim],[0,lim],"k--",lw=1.4,label="perfect ($y=x$)")
    ax.set_xlim(0,lim); ax.set_ylim(0,lim); ax.set_aspect("equal")
    ax.set_xlabel("NS-3 5G-LENA per-instant PDR"); ax.set_ylabel("OMNeT NS3-Learn per-instant PDR")
    ax.set_title(f"Geometry generalization at matched 50% load\nMAE={mae:.3f}, $r$={r:.3f}, $n$={len(S50)} distance-bins")
    ax.legend(loc="upper left",fontsize=8.5); fig.tight_layout(); fig.savefig(f"{OUT}/F5b_per_txrx_50pct.png"); plt.close(fig)

    # ---- F5c: DENSE matched-50% parity (per seed x 10 m bin — no seed pooling) ----
    dpath=f"{D}/per_txrx_50_dense.csv"
    if os.path.exists(dpath):
        Sd=pd.read_csv(dpath); mae=np.mean(np.abs(Sd.ns3-Sd.omnet)); r=np.corrcoef(Sd.ns3,Sd.omnet)[0,1]
        lim=max(Sd.ns3.max(),Sd.omnet.max())*1.1
        fig,ax=plt.subplots(figsize=(6.0,5.9))
        for g in ["intersection","mainstreet"]:
            s=Sd[Sd.geom==g]; lab=f"{'intersection (train)' if g=='intersection' else 'mainstreet (validate)'}  (n={len(s)}, MAE={np.mean(np.abs(s.ns3-s.omnet)):.3f})"
            ax.scatter(s.ns3,s.omnet,marker=mk(g),s=30,alpha=.55,
                       facecolor=col(g,gray) if not gray else "none",edgecolor=col(g,gray) if gray else "k",linewidth=0.4,label=lab)
        ax.plot([0,lim],[0,lim],"k--",lw=1.4,label="perfect ($y=x$)")
        ax.set_xlim(0,lim); ax.set_ylim(0,lim); ax.set_aspect("equal")
        ax.set_xlabel("NS-3 5G-LENA per-instant PDR"); ax.set_ylabel("OMNeT NS3-Learn per-instant PDR")
        ax.set_title(f"Matched 50% load, per seed $\\times$ 10 m bin (un-pooled)\nMAE={mae:.3f}, $r$={r:.3f}, $n$={len(Sd)}")
        ax.legend(loc="upper left",fontsize=8.5); fig.tight_layout(); fig.savefig(f"{OUT}/F5c_per_txrx_50pct_dense.png"); plt.close(fig)

    # ---- F6: distance-resolved PDR per rate ----
    nd=pd.read_csv(f"{D}/ns3_distance.csv"); od=pd.read_csv(f"{D}/omnet_distance.csv")
    fig,axs=plt.subplots(1,4,figsize=(15,3.8),sharey=True)
    for ax,r in zip(axs,[5,25,50,100]):
        n=nd[nd.rate==r]; o=od[od.rate==r]
        ax.plot(n.dist+10,n.pdr,marker="o",ls="-",color=col("NS-3",gray),lw=2.2,ms=5,label="NS-3 5G-LENA")
        ax.plot(o.dist+10,o.pdr,marker="D",ls="--",color=col("NS3Learn",gray),lw=2.2,ms=5,label="NS3-Learn")
        ax.set_title(f"{r}% CAV"); ax.set_xlabel("Tx–Rx distance (m)"); ax.set_xlim(0,300); ax.set_ylim(0,1.02)
    axs[0].set_ylabel("per-instant PDR"); axs[0].legend(loc="lower left")
    fig.suptitle("Per-link PDR vs distance — NS3-Learn reproduces NS-3's profile across density",y=1.02,fontsize=12.5)
    fig.tight_layout(); fig.savefig(f"{OUT}/F6_distance_pdr.png"); plt.close(fig)

    # ---- F7: channel ablation ----
    abl=pd.DataFrame([("per-link state",0.101,0.944),("+ i.i.d. shadowing/NLOSv",0.115,0.891),
                      ("+ AR(1) (full TR 37.885)",0.116,0.918)],columns=["variant","MAE","r"])
    fig,ax=plt.subplots(figsize=(7.2,4.4)); xpos=np.arange(len(abl))
    bars=ax.bar(xpos,abl.MAE,0.5,color=[col("Logistic",gray)]*3,edgecolor="k",linewidth=0.5,alpha=.85)
    ax.set_ylabel("per-tx-rx MAE",color=col("Logistic",gray)); ax.set_ylim(0,0.16)
    ax2=ax.twinx(); ax2.plot(xpos,abl.r,"o-",color=col("GBM",gray),lw=2.2,ms=8); ax2.set_ylabel("Pearson $r$",color=col("GBM",gray)); ax2.set_ylim(0.85,0.96)
    ax.set_xticks(xpos); ax.set_xticklabels(abl.variant,rotation=12,ha="right")
    ax.set_title("Channel-model ablation (per-tx-rx vs NS-3)")
    for i,(m,rr) in enumerate(zip(abl.MAE,abl.r)): ax.text(i,m+0.004,f"{m:.3f}",ha="center",fontsize=9)
    fig.tight_layout(); fig.savefig(f"{OUT}/F7_channel_ablation.png"); plt.close(fig)
    print(f"[{sub}] wrote F1-F7 to {OUT}")

for g in (False,True): make(g)
print("done")
