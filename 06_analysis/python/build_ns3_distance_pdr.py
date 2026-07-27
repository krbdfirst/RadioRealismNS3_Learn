#!/usr/bin/env python3
"""
Reconstruct NS-3 5G-LENA's true distance-resolved PDR (same KPI as OMNeT dist_pdr)
from pktTxRx + FCD positions, for the mainstreet 50% validation. For each
transmitted CAM, every in-range CAV at TX time is an 'attempt' (binned by Tx-Rx
distance); 'delivered' if that node has an rx for (srcIp, seq). Overlays the
3 OMNeT methods so the per-link comparison includes NS-3.

-> results/prop_compare_figures/perlink/pl5_ns3_vs_omnet_distance_mainstreet.png
"""
import sqlite3, glob, csv, os
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3 = f"{NS3_SCRATCH}"
PROJ = f"{PROJECT_DIR}"
OUT = f"{PROJ}/results/prop_compare_figures/perlink"
RANGE_BINS = np.arange(0, 480, 20)

def reconstruct(seed):
    tag = f"mainstreet50_s{seed}"
    meta = pd.read_csv(f"{NS3}/inputs/{tag}_meta.csv")          # nodeId,vehId,departTime,arriveTime
    node2veh = dict(zip(meta.nodeId, meta.vehId))
    dep = dict(zip(meta.nodeId, meta.departTime)); arr = dict(zip(meta.nodeId, meta.arriveTime))
    # FCD positions: dict[vehId][t_rounded] = (x,y)
    pos = {}
    fcd = pd.read_csv(f"{PROJ}/ns3_mobility/mainstreet50/seed{seed}/fcd.csv")
    fcd = fcd[fcd.vehicle_type == "veh_av"]
    for vid, t, x, y in zip(fcd.vehicle_id, fcd.timestep_time, fcd.vehicle_x, fcd.vehicle_y):
        pos.setdefault(vid, {})[round(t, 1)] = (x, y)
    db = glob.glob(f"{NS3}/results/mainstreet_val/{tag}-UMi_StreetCanyon_LoS-*.db")[0]
    con = sqlite3.connect(db)
    tx = con.execute("SELECT nodeId,srcIp,pktSeqNum,timeSec FROM pktTxRx WHERE txRx='tx'").fetchall()
    rx = con.execute("SELECT nodeId,srcIp,pktSeqNum FROM pktTxRx WHERE txRx='rx'").fetchall()
    con.close()
    delivered = {}
    for rn, sip, seq in rx:
        delivered.setdefault((sip, seq), set()).add(rn)
    nodes = list(node2veh)
    att = np.zeros(len(RANGE_BINS)); dlv = np.zeros(len(RANGE_BINS))
    for sn, sip, seq, t in tx:
        tr = round(t, 1); sv = node2veh.get(sn)
        sp = pos.get(sv, {}).get(tr)
        if sp is None:
            continue
        got = delivered.get((sip, seq), set())
        for rn in nodes:
            if rn == sn or not (dep[rn] <= t <= arr[rn]):
                continue
            rp = pos.get(node2veh[rn], {}).get(tr)
            if rp is None:
                continue
            d = np.hypot(sp[0]-rp[0], sp[1]-rp[1])
            b = min(int(d // 20), len(RANGE_BINS)-1)
            att[b] += 1
            if rn in got:
                dlv[b] += 1
    return att, dlv

A = np.zeros(len(RANGE_BINS)); D = np.zeros(len(RANGE_BINS))
for s in range(3):
    try:
        a, d = reconstruct(s); A += a; D += d; print(f"seed {s} reconstructed")
    except Exception as e:
        print(f"seed {s}: {e}")
ns3_pdr = np.divide(D, A, out=np.full_like(D, np.nan), where=A >= 30)

COL = {"Analytical":"#ff7f0e","Generalized":"#d62728","NS3Learn":"#9467bd"}
fig, ax = plt.subplots(figsize=(9, 6))
for m in ["Analytical","Generalized","NS3Learn"]:
    cs = []
    for dd in sorted(glob.glob(f"{PROJ}/results/inet/Prop_{m}_MS50-*/")):
        f = sorted(glob.glob(dd+"dist_pdr_*.csv"))
        if f: cs.append(pd.read_csv(f[-1])[["dist_bin_m","attempts","delivered"]])
    if cs:
        g = pd.concat(cs).groupby("dist_bin_m").agg(a=("attempts","sum"), dl=("delivered","sum")); g = g[g.a >= 30]
        ax.plot(g.index, g.dl/g.a, "-", color=COL[m], lw=2, label=f"OMNeT {m}")
ax.plot(RANGE_BINS, ns3_pdr, "ks--", lw=2.5, ms=5, label="NS-3 5G-LENA (reconstructed)")
ax.axvline(300, color="grey", ls=":", lw=1); ax.set_xlim(0, 460); ax.set_ylim(0, 1.02)
ax.set_xlabel("Tx–Rx distance (m)"); ax.set_ylabel("PDR")
ax.set_title("Per-link PDR vs distance — mainstreet 50% (LOS-only)\nthree OMNeT models vs NS-3 5G-LENA (true distance-resolved PDR)")
ax.legend(); ax.grid(alpha=.3); fig.tight_layout()
p = f"{OUT}/pl5_ns3_vs_omnet_distance_mainstreet.png"; fig.savefig(p, dpi=150); print("wrote", p)
print("NS-3 reconstructed PDR @ 50/150/300/450m:",
      {int(RANGE_BINS[i]): round(ns3_pdr[i],3) for i in (2,7,15,22) if not np.isnan(ns3_pdr[i])})
