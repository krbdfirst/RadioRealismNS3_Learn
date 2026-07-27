#!/usr/bin/env python3
"""
Export the adopted (logistic) decomposed-cascade coefficients to the OMNeT runtime bundle.
decodeP(SINR)   = sigmoid(dec_c0 + dec_c1*SINR + dec_c2*SINR^2)
collisionP(n)   = sigmoid(col_c0 + col_c1*n    + col_c2*n^2)
capture penalty I_coll ~ Normal(cap_mu, cap_sigma);  half_duplex hd const.
-> results/teacher_dataset/ns3learn_runtime/cascade_params.csv
"""
import numpy as np, pandas as pd
from sklearn.linear_model import LogisticRegression
from rr_paths import PROJECT_DIR
PROJ=f"{PROJECT_DIR}"
DS=f"{PROJ}/results/teacher_dataset/cascade"; B=f"{PROJ}/results/teacher_dataset/ns3learn_runtime"
D=pd.read_csv(f"{DS}/decode_dataset.csv"); C=pd.read_csv(f"{DS}/collision_dataset.csv")
hd=pd.read_csv(f"{DS}/halfduplex_by_rate.csv").hd_block_frac.mean()

dec=LogisticRegression(max_iter=1000).fit(np.column_stack([D.sinr_db,D.sinr_db**2]),D.decoded.astype(int))
col=LogisticRegression(max_iter=1000).fit(np.column_stack([C.n,C.n**2]),C.collided.astype(int))
# capture params from the logistic end-to-end fit (build_learner_bakeoff): (28.2, 6.9)
cap_mu, cap_sigma = 28.2, 6.9

p=dict(
  cascade=1.0, half_duplex=round(float(hd),4),
  dec_c0=round(float(dec.intercept_[0]),5), dec_c1=round(float(dec.coef_[0,0]),5), dec_c2=round(float(dec.coef_[0,1]),6),
  col_c0=round(float(col.intercept_[0]),5), col_c1=round(float(col.coef_[0,0]),5), col_c2=round(float(col.coef_[0,1]),6),
  cap_mu=cap_mu, cap_sigma=cap_sigma,
  nominal_rate_hz=10.0,   # calibration beacon rate; hd & collision scale with rate relative to this
  tx_power_dbm=23.0, noise_floor_dbm=round(-174+10*np.log10(18.72e6)+7.0,2), fc_ghz=5.9)
pd.DataFrame(list(p.items()),columns=["key","value"]).to_csv(f"{B}/cascade_params.csv",index=False)
for k,v in p.items(): print(f"  {k} = {v}")
# sanity: print a few decode/collision values
sig=lambda z:1/(1+np.exp(-z))
for s in (0,6,12,20): print(f"decodeP({s}dB)={sig(p['dec_c0']+p['dec_c1']*s+p['dec_c2']*s*s):.3f}")
for n in (5,25,50): print(f"collisionP(n={n})={sig(p['col_c0']+p['col_c1']*n+p['col_c2']*n*n):.3f}")
