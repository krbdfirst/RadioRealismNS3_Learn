#!/usr/bin/env python3
"""
LOAD-aware contention dataset for the flood/CBR extension (closing the C2 gap).

Identical SB-SPS collision + density logic to extract_contention_dataset.py, but it
processes the RATE-SWEEP teacher DBs (results/teacher_rate/hz{HZ}/) and tags every
transmission with the run's CAM rate (Hz). This lets the fitter test the hypothesis:

    collision depends on OFFERED LOAD (= local CAV density n x CAM rate Hz),
    NOT on vehicle density n alone.

If collision(n,Hz) collapses onto a single curve vs offered_load, the flood (one node,
~100x rate) is just another way to raise offered_load -> the OMNeT collision term can be
re-grounded on load and will respond to a flood. The C1 model is the Hz=10 slice.

Reuses the EXISTING mobility/FCD (ns3_mobility/cav{dd}/seed{s}) and meta — nothing new.
nodeId = rnti-1.  -> results/teacher_dataset/cascade/collision_load_dataset.csv
"""
import sqlite3, glob, os, numpy as np, pandas as pd
from collections import defaultdict

from rr_paths import NS3_SCRATCH, PROJECT_DIR
NS3 = f"{NS3_SCRATCH}"
PROJ = f"{PROJECT_DIR}"
SCEN = os.environ.get("SCEN", "V2V_Urban")
OUT = f"{PROJ}/results/teacher_dataset/cascade"; os.makedirs(OUT, exist_ok=True)
RANGE = 300.0
RATES_HZ = [int(x) for x in os.environ.get("RATES_HZ", "10 25").split()]
DENS = [int(x) for x in os.environ.get("DENS", "5 25 50").split()]
SEEDS = [int(x) for x in os.environ.get("SEEDS", "0 1 2").split()]
SAMP = {5: 1, 25: 1, 50: 2}                      # row throttle, matches C1 extractor


def fcd(dd, s):
    """Per-(density,seed) FCD positions keyed by (rounded time)->veh->(x,y); + node->veh map."""
    fc = f"{PROJ}/ns3_mobility/cav{dd:02d}/seed{s}/fcd_cav{dd:02d}.csv"
    me = f"{NS3}/inputs/cav{dd:02d}_s{s}_meta.csv"
    meta = pd.read_csv(me); n2v = dict(zip(meta.nodeId, meta.vehId))
    f = pd.read_csv(fc); f = f[f.vehicle_type == "veh_av"]; pos = {}
    for row in f.itertuples(index=False):
        pos.setdefault(row.vehicle_id, {})[round(row.timestep_time, 1)] = (row.vehicle_x, row.vehicle_y)
    return n2v, pos


rows = []
for hz in RATES_HZ:
    for dd in DENS:
        for s in SEEDS:
            dbs = glob.glob(f"{NS3}/results/teacher_rate/hz{hz}/cav{dd:02d}_s{s}-{SCEN}-*.db")
            if not dbs:
                print(f"  MISSING hz{hz} cav{dd:02d}_s{s} (run the sweep first)"); continue
            n2v, pos = fcd(dd, s); nodes = list(n2v)
            con = sqlite3.connect(dbs[0])
            tx = con.execute("SELECT timeMs,rnti,frame,subFrame,slot,rbStart,rbLen "
                             "FROM psschTxUeMac").fetchall(); con.close()
            slots = defaultdict(list)
            for timeMs, rnti, fr, sf, sl, rb0, rbl in tx:
                slots[(fr, sf, sl)].append((rnti - 1, rb0, rb0 + rbl, timeMs))

            def density(node, timeMs):
                tr = round(timeMs / 1000.0, 1); sp = pos.get(n2v.get(node), {}).get(tr)
                if sp is None: return None
                c = 0
                for rn in nodes:
                    if rn == node: continue
                    p = pos.get(n2v[rn], {}).get(tr)
                    if p is not None and np.hypot(sp[0] - p[0], sp[1] - p[1]) <= RANGE: c += 1
                return c

            samp = SAMP.get(dd, 1); i = 0; nrows0 = len(rows)
            for key, lst in slots.items():
                conc = len(lst)
                for j, (node, a0, a1, tms) in enumerate(lst):
                    i += 1
                    if i % samp: continue
                    collided = int(any(k != j and not (a1 <= b0 or b1 <= a0)
                                       for k, (nn, b0, b1, _) in enumerate(lst)))
                    n = density(node, tms)
                    if n is None: continue
                    rows.append((hz, dd, s, n, conc, collided, n * hz))
            sub = rows[nrows0:]
            cf = np.mean([x[5] for x in sub]) if sub else 0.0
            print(f"hz{hz} cav{dd:02d}_s{s}: rows={len(sub)} collided_frac={cf:.3f}")

C = pd.DataFrame(rows, columns=["hz", "rate", "seed", "n", "concurrency", "collided", "offered_load"])
C.to_csv(f"{OUT}/collision_load_dataset.csv", index=False)
print(f"\nTotal {len(C)} rows -> {OUT}/collision_load_dataset.csv")
if len(C):
    # The decisive table: collided fraction by (density, rate). If it tracks offered_load
    # (n*Hz) and NOT n alone, e.g. cav50@10Hz ~ cav25@20Hz, the load hypothesis holds.
    piv = C.groupby(["rate", "hz"]).agg(coll=("collided", "mean"),
                                        n=("n", "mean"), load=("offered_load", "mean")).round(3)
    print("\ncollided fraction by (density rate, CAM Hz):"); print(piv)
