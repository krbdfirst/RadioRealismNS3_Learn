#!/usr/bin/env python3
"""
Extract the NS-3 teacher dataset for the OMNeT distillation. Two products:

  1. Per-run aggregates (MAC / load branch) from macSummary DBs:
     neighbour density, resource-collision fraction, HARQ ReTx multiplier, PRR@300.
     -> results/teacher_dataset/teacher_run_aggregates.csv

  2. BLER(SINR) + SCI(SINR) waterfall (channel/PHY branch) from phyRxTrace DBs:
     -> results/teacher_dataset/bler_curve_ns3.csv

Re-runnable: scans whatever DBs exist now (grows as collection completes).
"""
from __future__ import annotations
import glob, os, re, sqlite3, sys
import numpy as np, pandas as pd

from rr_paths import NS3_RESULTS, PROJECT_DIR
PROJ = f"{PROJECT_DIR}"
TEACHER = f"{NS3_RESULTS}/teacher"
PHYDIRS = [  # DBs that have RX traces (psschRxUePhy populated)
    TEACHER,
    f"{NS3_RESULTS}/phy_decoupling",
]
OUT = os.path.join(PROJ, "results", "teacher_dataset")
os.makedirs(OUT, exist_ok=True)
TAG_RE = re.compile(r"cav(\d+)_s(\d+)-")

def cols(con, table):
    try: return {r[1] for r in con.execute(f"PRAGMA table_info({table})")}
    except sqlite3.Error: return set()

def has_rows(con, table):
    try: return con.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0] > 0
    except sqlite3.Error: return False

def run_aggregates():
    rows = []
    for db in sorted(glob.glob(f"{TEACHER}/cav*-propagation-compare.db")):
        m = TAG_RE.search(os.path.basename(db));  rate, seed = (int(m.group(1)), int(m.group(2))) if m else (None, None)
        con = sqlite3.connect(db)
        try:
            if not has_rows(con, "simulPsschTx"):  # needs macSummary
                continue
            tot, nonov, ov = con.execute("SELECT totalTx,numNonOverlapping,numOverlapping FROM simulPsschTx").fetchone()
            newtb = con.execute("SELECT COUNT(*) FROM psschTxUeMac WHERE rv=0").fetchone()[0]
            nN = con.execute("SELECT AVG(numNieb),COUNT(DISTINCT nodeId) FROM avrgPrr WHERE range=300").fetchone()
            prr = con.execute("SELECT SUM(avrgPrr*numNieb)*1.0/NULLIF(SUM(numNieb),0) FROM avrgPrr WHERE range=300").fetchone()[0]
        finally:
            con.close()
        rows.append(dict(rate=rate, seed=seed, totalTx=tot,
                         collision_frac=ov/tot if tot else np.nan,
                         harq_retx_mult=tot/newtb if newtb else np.nan,
                         mean_neighbours=nN[0], numUes=nN[1], prr300=prr))
    df = pd.DataFrame(rows).sort_values(["rate", "seed"])
    df.to_csv(f"{OUT}/teacher_run_aggregates.csv", index=False)
    return df

def bler_curve():
    parts = []
    seen = set()
    for d in PHYDIRS:
        for db in sorted(glob.glob(f"{d}/cav*-propagation-compare.db")):
            key = os.path.basename(db)
            if key in seen:  # prefer first dir (teacher V2V over phy_decoupling UMi dup)
                continue
            con = sqlite3.connect(db)
            try:
                if not has_rows(con, "psschRxUePhy"):
                    continue
                arr = np.array(con.execute("SELECT avrgSinr,psschCorrupt,sci2Corrupt FROM psschRxUePhy").fetchall(), float)
            finally:
                con.close()
            if arr.size == 0:
                continue
            seen.add(key)
            scn = "UMi" if "UMi" in key else "V2V"
            for lo in range(-20, 40):
                m = (arr[:, 0] >= lo) & (arr[:, 0] < lo + 1)
                if m.sum() >= 30:
                    parts.append(dict(scenario=scn, snr_db=lo + 0.5, n=int(m.sum()),
                                      pssch_decode=1 - arr[m, 1].mean(),
                                      sci_decode=1 - arr[m, 2].mean()))
    if not parts:
        return pd.DataFrame()
    df = (pd.DataFrame(parts).groupby(["scenario", "snr_db"])
          .apply(lambda g: pd.Series({"n": g["n"].sum(),
                                      "pssch_decode": np.average(g["pssch_decode"], weights=g["n"]),
                                      "sci_decode": np.average(g["sci_decode"], weights=g["n"])}))
          .reset_index())
    df.to_csv(f"{OUT}/bler_curve_ns3.csv", index=False)
    return df

if __name__ == "__main__":
    agg = run_aggregates()
    print(f"=== run aggregates: {len(agg)} runs ===")
    if not agg.empty:
        print(agg.groupby("rate")[["numUes", "mean_neighbours", "collision_frac",
                                   "harq_retx_mult", "prr300"]].mean().round(3).to_string())
    bler = bler_curve()
    print(f"\n=== BLER curve points: {len(bler)} ===")
    if not bler.empty:
        for scn in bler.scenario.unique():
            s = bler[bler.scenario == scn]
            print(f"  [{scn}] SINR 0/5/10/15/20 dB decode:",
                  {int(r.snr_db): round(r.pssch_decode, 2) for _, r in s.iterrows()
                   if int(r.snr_db) in (0, 5, 10, 15, 20)})
