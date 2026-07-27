#!/usr/bin/env python3
"""
Consolidate the NS-3-distilled coefficients into a runtime bundle the OMNeT app
loads when realismModel="ns3learn".  -> results/teacher_dataset/ns3learn_runtime/

  bler_pssch.csv   snr_db,bler          (PSSCH data decode; interpolateBlerCurve format)
  bler_sci.csv     snr_db,bler          (SCI-2 control decode)
  ns3learn_params.csv   key,value       (half_duplex, capture, collision logistic L/k/x0,
                                          noise_figure_db, bandwidth_hz, tx_power_dbm)
Re-runnable; reads the fit outputs in ns3_trained/.
"""
import os, pandas as pd, numpy as np
from rr_paths import RESULTS_DIR
DS = f"{RESULTS_DIR}/teacher_dataset"
SRC = os.path.join(DS, "ns3_trained")
OUT = os.path.join(DS, "ns3learn_runtime"); os.makedirs(OUT, exist_ok=True)

# BLER (PSSCH) + SCI curves from the NS-3 V2V waterfall
b = pd.read_csv(f"{DS}/bler_curve_ns3.csv")
bv = b[b.scenario == "V2V"].sort_values("snr_db")
pssch = np.maximum.accumulate(bv.pssch_decode.values)
sci = np.maximum.accumulate(bv.sci_decode.values)
pd.DataFrame({"snr_db": bv.snr_db, "bler": np.clip(1 - pssch, 0, 1)}).to_csv(f"{OUT}/bler_pssch.csv", index=False)
pd.DataFrame({"snr_db": bv.snr_db, "bler": np.clip(1 - sci, 0, 1)}).to_csv(f"{OUT}/bler_sci.csv", index=False)

# scalar + collision-logistic params
sp = pd.read_csv(f"{SRC}/surrogate_params.csv").iloc[0]
cf = pd.read_csv(f"{SRC}/collision_fit_params.csv").iloc[0]
params = {
    "half_duplex": round(float(sp["hd"]), 4),
    "capture": round(float(sp["capture"]), 4),
    "collision_L": round(float(cf["L"]), 4),
    "collision_k": round(float(cf["k"]), 5),
    "collision_x0": round(float(cf["x0"]), 2),
    "noise_figure_db": 7.0,
    "bandwidth_hz": 18.72e6,
    "tx_power_dbm": 23.0,
    "fc_ghz": 5.9,
}
pd.DataFrame(list(params.items()), columns=["key", "value"]).to_csv(f"{OUT}/ns3learn_params.csv", index=False)
print("wrote runtime bundle ->", OUT)
for k, v in params.items():
    print(f"  {k:16s} = {v}")
print(f"  BLER pssch points = {len(bv)}  (SINR {bv.snr_db.min():.0f}..{bv.snr_db.max():.0f} dB)")
