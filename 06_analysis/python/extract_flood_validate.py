#!/usr/bin/env python3
"""
HELD-OUT validation: does the load-driven collision model hold when load is
CONCENTRATED in one node (a real flood), not distributed (the symmetric sweep)?

For each NORMAL-node transmission in the flood runs, compute the LOCAL offered load
  local_load = (in-range NORMAL neighbours x 10 Hz) + (flooder's pps, if the flooder
               is within range at that moment)
and its measured collision (RB overlap in the same slot, with the flooder counted as an
interferer). Then compare against the SYMMETRIC-sweep curve
  collision_pred = sigmoid(c0 + c1*log10(local_load))   [from fit_contention_load.py]

If the flood data lands on that curve, collision is genuinely load-driven regardless of
how the load is distributed -> the OMNeT load-aware term is validated. If concentrated
load collides LESS than predicted, collision also depends on interferer COUNT -> caught
before wiring.

Reuses the existing FCD/meta (nodeId=rnti-1). -> results/teacher_dataset/cascade/flood_validate_dataset.csv
"""
import sqlite3, glob, os, re, numpy as np, pandas as pd
from collections import defaultdict

from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3 = f"{NS3_SCRATCH}"
PROJ = f"{PROJECT_DIR}"
SCEN = os.environ.get("SCEN", "V2V_Urban")
CAS = f"{PROJ}/results/teacher_dataset/cascade"
RANGE = 300.0
BASE_HZ = 10
FLOODPPS = [int(x) for x in os.environ.get("FLOODPPS", "200 1000").split()]
DENS = [int(x) for x in os.environ.get("DENS", "5 25 50").split()]
SEEDS = [int(x) for x in os.environ.get("SEEDS", "0 1 2").split()]
SAMP = {5: 1, 25: 2, 50: 3}

# symmetric-sweep fit (the curve we test against)
fp = pd.read_csv(f"{CAS}/load_collision_params.csv").iloc[0]
C0, C1 = float(fp.c0), float(fp.c1)
def sig(z): return 1.0 / (1.0 + np.exp(-z))
def pred_coll(load): return sig(C0 + C1 * np.log10(np.maximum(load, 1e-9)))
print(f"symmetric curve: collision = sigmoid({C0:.3f} + {C1:.3f}*log10(load))\n")


def fcd(dd, s):
    me = f"{NS3}/inputs/cav{dd:02d}_s{s}_meta.csv"
    fc = f"{PROJ}/ns3_mobility/cav{dd:02d}/seed{s}/fcd_cav{dd:02d}.csv"
    meta = pd.read_csv(me); n2v = dict(zip(meta.nodeId, meta.vehId))
    f = pd.read_csv(fc); f = f[f.vehicle_type == "veh_av"]; pos = {}
    for row in f.itertuples(index=False):
        pos.setdefault(row.vehicle_id, {})[round(row.timestep_time, 1)] = (row.vehicle_x, row.vehicle_y)
    return n2v, pos


def flooder_of(resfile):
    try:
        txt = open(resfile).read()
        m = re.search(r"FLOODNODE id=(\d+)", txt)
        return int(m.group(1)) if m else None
    except FileNotFoundError:
        return None


rows = []
for pps in FLOODPPS:
    for dd in DENS:
        for s in SEEDS:
            db = glob.glob(f"{NS3}/results/teacher_flood/pps{pps}/cav{dd:02d}_s{s}-{SCEN}-*.db")
            res = f"{NS3}/results/teacher_flood/pps{pps}/cav{dd:02d}_s{s}_pps{pps}.result"
            if not db:
                print(f"  MISSING pps{pps} cav{dd:02d}_s{s}"); continue
            fid = flooder_of(res)
            if fid is None:
                print(f"  no FLOODNODE id in {res}"); continue
            n2v, pos = fcd(dd, s); nodes = list(n2v); fveh = n2v.get(fid)
            con = sqlite3.connect(db[0])
            tx = con.execute("SELECT timeMs,rnti,frame,subFrame,slot,rbStart,rbLen "
                             "FROM psschTxUeMac").fetchall(); con.close()
            slots = defaultdict(list)
            for timeMs, rnti, fr, sf, sl, rb0, rbl in tx:
                slots[(fr, sf, sl)].append((rnti - 1, rb0, rb0 + rbl, timeMs))

            def local_load(node, tms):
                tr = round(tms / 1000.0, 1); sp = pos.get(n2v.get(node), {}).get(tr)
                if sp is None: return None, None, None
                n_norm = 0
                for rn in nodes:
                    if rn == node or rn == fid: continue          # exclude self + flooder from normal count
                    p = pos.get(n2v[rn], {}).get(tr)
                    if p is not None and np.hypot(sp[0] - p[0], sp[1] - p[1]) <= RANGE: n_norm += 1
                fp_ = pos.get(fveh, {}).get(tr)
                f_in = (fp_ is not None and np.hypot(sp[0] - fp_[0], sp[1] - fp_[1]) <= RANGE)
                return n_norm, int(f_in), (n_norm * BASE_HZ) + (pps if f_in else 0)

            samp = SAMP.get(dd, 1); i = 0; n0 = len(rows)
            for key, lst in slots.items():
                for j, (node, a0, a1, tms) in enumerate(lst):
                    if node == fid: continue                       # validate NORMAL nodes only
                    i += 1
                    if i % samp: continue
                    collided = int(any(k != j and not (a1 <= b0 or b1 <= a0)
                                       for k, (nn, b0, b1, _) in enumerate(lst)))
                    nn, fin, load = local_load(node, tms)
                    if nn is None: continue
                    rows.append((pps, dd, s, nn, fin, load, collided))
            sub = rows[n0:]
            cf = np.mean([x[6] for x in sub]) if sub else 0.0
            print(f"pps{pps} cav{dd:02d}_s{s}: flooder=node{fid} normal-tx rows={len(sub)} collided={cf:.3f}")

D = pd.DataFrame(rows, columns=["pps", "dens", "seed", "n_normal", "flooder_in_range", "local_load", "collided"])
D.to_csv(f"{CAS}/flood_validate_dataset.csv", index=False)
if not len(D):
    print("\nno rows yet — runs still in flight"); raise SystemExit

D["pred"] = pred_coll(D.local_load)
print(f"\nTotal {len(D):,} normal-node tx -> {CAS}/flood_validate_dataset.csv")

print("\nDECISIVE: measured collision vs SYMMETRIC-curve prediction at the same local load")
g = D.groupby(["pps", "dens"]).agg(n_norm=("n_normal", "mean"), f_in=("flooder_in_range", "mean"),
                                   load=("local_load", "mean"), meas=("collided", "mean"),
                                   pred=("pred", "mean"), rows=("collided", "size")).reset_index()
g["abs_err"] = (g.meas - g.pred).abs()
print(g.round(3).to_string(index=False))
print(f"\n  flood-vs-symmetric-curve MAE = {g.abs_err.mean():.3f}")
print("  small MAE => collision is load-driven even when load is CONCENTRATED (model validated)")
print("  meas << pred => concentrated load collides less (interferer-count effect) => model needs work")

# load-binned collapse: does flood data fall on the curve across the load range?
D["lb"] = pd.cut(np.log10(D.local_load.clip(lower=1)), bins=8)
b = D.groupby("lb", observed=True).agg(load=("local_load", "mean"), meas=("collided", "mean"),
                                       pred=("pred", "mean"), n=("collided", "size")).reset_index(drop=True)
print("\nload-binned (flood) measured vs symmetric-curve prediction:")
print(b.round(3).to_string(index=False))
