#!/usr/bin/env python3
"""
Step 1 sanity check for the Combined Analytical Reference (paper Eq. m3):

    PDR_M3(d, n) = (1 - P_HD) * (1 - P_COL(n)) * g(SNR(d))

    Model 1 (Cao 2026)   : P_HD = t_s/T_RRI ; P_COL(n) closed-form (p_k = 0)   -> DENSITY
    Model 2 (Rehman 2023): g(SNR(d)) noise-limited decode, TR 38.901 UMi + CDL -> DISTANCE

No double counting: Model 2 uses SNR (noise only, empty interferer set); all
multi-vehicle contention lives ONLY in Model 1's P_COL; half-duplex only in Model 1.

CAVEAT — this is a FORMULA sanity check, not the authoritative comparison. The
per-rate `nb` in surrogate_validation.csv is the avrgPrr cumulative-neighbour count,
which is ~3x the TRUE per-reception in-range density (real range ~8-61). So the
absolute PRR-vs-NS-3 numbers printed here are evaluated at an inflated density and
are only indicative. The authoritative comparison is the OMNeT run, where BOTH
analytical_m3 and ns3learn read the same true countNeighboursInRange().

Everything is pinned to the NS-3 teacher config (propagation-compare.cc) so the
analytical baseline is grounded, not fitted. This script runs no OMNeT build; it
prints the numbers and writes a params CSV + a diagnostic PNG for review.
"""
import csv
import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
TEACH = os.path.join(HERE, "results", "teacher_dataset", "ns3_trained")
CDL_CURVE = os.path.join(HERE, "results", "inet",
                         "phy_model_outputs_5gtoolbox_5000", "runtime_coefficients",
                         "bler_curve_cdl_c.csv")
OUTDIR = os.path.join(HERE, "results", "analytical_m3")
os.makedirs(OUTDIR, exist_ok=True)

# ---------------------------------------------------------------- config (teacher)
FC_GHZ      = 5.9
H_TX = H_RX = 1.5          # V2V antenna heights (m)
TX_DBM      = 23.0
NOISE_DBM   = -174.0 + 10.0 * math.log10(18.72e6) + 7.0   # = -94.3 dBm
PC5_RANGE   = 300.0       # m (matches pc5Range / NS-3 RANGE)

T_RRI_MS    = 100.0       # reservation period T_RRI = 100 ms (Cao Table II)
T_SLOT_MS   = 1.0         # t_s = 1 ms, 15 kHz SCS (Cao Table II)
N_SC        = 5           # our channel N_sc (10 MHz/15 kHz); Cao demonstrated N_sc=2
SL_SLOT_FRAC = 9.0 / 12.0 # SL slots / total (slBitMap); Cao's N_r counts SL slots only
PI0         = 1.0 / 11.0  # Cao: R_c ~ U[5,15]
P_K         = 0.0         # p_k (Cao Table II range [0,0.8]); our scenario = 0
SIG_LOS = 3.0                   # TR 37.885 V2V-Urban shadowing std (dB), LOS & NLOSv
NLOSV_EXCESS_STD = 4.5          # TR 37.885 NLOSv vehicle-blockage excess std (dB)
SIG_NLOS = math.sqrt(SIG_LOS ** 2 + NLOSV_EXCESS_STD ** 2)   # NLOSv branch: shadow (+) blockage

# N_r per Cao Eq.(1): candidate SL resources = (SL slots in window) x N_sc. The 9/12 is
# the SL-slot fraction of our pool (slBitMap), NOT an extra factor: Cao counts SL slots,
# and ours is a 9/12-SL pool (his own pool is all-SL). Needed for parity with NS3Learn/NS-3.
N_R_RRI = (T_RRI_MS / T_SLOT_MS) * SL_SLOT_FRAC * N_SC    # full-RRI window (Cao default)
N_R_SEL = 33.0 * SL_SLOT_FRAC * N_SC                      # T2=33-slot selection window (our scenario)

P_HD = T_SLOT_MS / T_RRI_MS                               # = 0.01

# ---------------------------------------------------------------- TR 37.885 V2V-Urban (matches C++ v2vChannelPathLossDb)
def v2v_los_prob(d):
    return min(1.0, 1.05 * math.exp(-0.0114 * max(d, 1.0)))

def v2v_pl_db(d, los):
    lg = math.log10(max(d, 1.0))
    pl_los = 38.77 + 16.7 * lg + 18.2 * math.log10(FC_GHZ)
    if los:
        return pl_los
    nlosv_excess = 9.0 + max(0.0, 15.0 * lg - 41.0)   # NLOSv mean vehicle-blockage excess
    return pl_los + nlosv_excess

# ---------------------------------------------------------------- CDL BLER curve
def load_bler(path):
    snr, bler = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            snr.append(float(row["snr_db"])); bler.append(float(row["bler"]))
    order = np.argsort(snr)
    return np.array(snr)[order], np.array(bler)[order]

BLER_SNR, BLER_VAL = load_bler(CDL_CURVE)
def bler(snr_db):
    return float(np.interp(snr_db, BLER_SNR, BLER_VAL, left=1.0, right=0.0))

# ---------------------------------------------------------------- Model 2: g(SNR(d))
# g(d) = E_{LOS/NLOS, shadow}[ 1 - BLER(SNR(d)+shadow) ] with SNR noise-limited.
_GH_X, _GH_W = np.polynomial.hermite_e.hermegauss(21)   # Gauss-Hermite over N(0,sigma^2)
def decode_g(d):
    out = 0.0
    for los in (True, False):
        p = v2v_los_prob(d) if los else (1.0 - v2v_los_prob(d))
        if p <= 0.0:
            continue
        snr_mean = TX_DBM - v2v_pl_db(d, los) - NOISE_DBM
        sig = SIG_LOS if los else SIG_NLOS
        # E_shadow[1 - BLER] via Gauss-Hermite (probabilists' weights sum to sqrt(2pi))
        acc = sum(w * (1.0 - bler(snr_mean + sig * x)) for x, w in zip(_GH_X, _GH_W))
        out += p * acc / math.sqrt(2.0 * math.pi)
    return min(1.0, max(0.0, out))

# ---------------------------------------------------------------- Model 1: P_COL(n)
# p_k = 0  ->  denominator = 1, numerator = 1 - (1 - 2*PI0/N_a)^n
def p_col(n, n_r):
    if n <= 0:
        return 0.0
    pc = 0.0
    for _ in range(200):
        n_a = max(1.0, n_r - n + 0.5 * pc * n)
        new = 1.0 - (1.0 - 2.0 * PI0 / n_a) ** n
        if abs(new - pc) < 1e-10:
            pc = new; break
        pc = new
    return min(1.0, max(0.0, pc))

# ---------------------------------------------------------------- combined
def pdr_m3(d, n, n_r=N_R_RRI):
    return (1.0 - P_HD) * (1.0 - p_col(n, n_r)) * decode_g(d)

def aggregate_prr(n, n_r=N_R_RRI, nd=400):
    # E_d over in-range neighbours uniform in the disk r<=RANGE (pdf ~ 2d/R^2)
    ds = (np.arange(nd) + 0.5) / nd * PC5_RANGE
    w = 2.0 * ds / (PC5_RANGE ** 2)
    g = np.array([decode_g(d) for d in ds])
    edecode = np.sum(w * g) * (PC5_RANGE / nd)
    return (1.0 - P_HD) * (1.0 - p_col(n, n_r)) * edecode

# ================================================================ validation
print(f"noise floor        = {NOISE_DBM:.2f} dBm")
print(f"P_HD               = {P_HD:.4f}")
print(f"N_r (full RRI)     = {N_R_RRI:.0f}   N_r (T2 selection) = {N_R_SEL:.0f}")
print(f"CDL curve          = {os.path.basename(CDL_CURVE)}  (BLER 1->0 near "
      f"{np.interp(0.5, BLER_VAL[::-1], BLER_SNR[::-1]):.1f} dB)")
print(f"decode g(d): 10m={decode_g(10):.3f} 50m={decode_g(50):.3f} "
      f"100m={decode_g(100):.3f} 200m={decode_g(200):.3f} 300m={decode_g(300):.3f}")

# --- P_COL(n) vs NS-3 measured collision (RB-overlap) -----------------------
print("\n P_COL(n): Cao(full RRI) | Cao(T2 sel) | NS-3 RB-overlap")
coll_pts = []
if os.path.exists(os.path.join(TEACH, "collision_vs_density.csv")):
    with open(os.path.join(TEACH, "collision_vs_density.csv")) as f:
        for row in csv.DictReader(f):
            coll_pts.append((float(row["neighbours"]), float(row["collision_frac"])))
for nb in [4, 10, 20, 44, 80, 90, 136]:
    meas = np.mean([c for (x, c) in coll_pts if abs(x - nb) <= 3]) if coll_pts else float("nan")
    print(f"  n={nb:3d}:  {p_col(nb, N_R_RRI):.3f}        {p_col(nb, N_R_SEL):.3f}"
          f"        {meas:.3f}")

# --- aggregate per-instant PRR vs NS-3 --------------------------------------
print("\n aggregate PRR@range vs NS-3 (surrogate_validation.csv):")
print(" rate    n     M3(RRI)  M3(T2)   NS-3")
rows = []
sv = os.path.join(TEACH, "surrogate_validation.csv")
if os.path.exists(sv):
    with open(sv) as f:
        for row in csv.DictReader(f):
            n = float(row["nb"]); prr = float(row["prr"])
            m_rri = aggregate_prr(n, N_R_RRI); m_t2 = aggregate_prr(n, N_R_SEL)
            rows.append((float(row["rate"]), n, m_rri, m_t2, prr))
    for rate, n, m_rri, m_t2, prr in sorted(rows):
        print(f"  {rate:4.0f}  {n:5.1f}   {m_rri:.3f}   {m_t2:.3f}   {prr:.3f}")
    mae_rri = np.mean([abs(m - p) for _, _, m, _, p in rows])
    mae_t2 = np.mean([abs(m - p) for _, _, _, m, p in rows])
    print(f"\n MAE vs NS-3:  M3(full RRI) = {mae_rri:.3f}   M3(T2 selection) = {mae_t2:.3f}")

# ================================================================ figure
fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
dd = np.linspace(1, 300, 120)
ax[0].plot(dd, [decode_g(d) for d in dd], "b-")
ax[0].set(title="Model 2: g(SNR(d)) decode", xlabel="distance (m)", ylabel="P(decode)", ylim=(0, 1.02))
nn = np.arange(1, 200)
ax[1].plot(nn, [p_col(n, N_R_RRI) for n in nn], "b-", label="Cao P_COL (full RRI)")
ax[1].plot(nn, [p_col(n, N_R_SEL) for n in nn], "b--", label="Cao P_COL (T2 sel window)")
if coll_pts:
    ax[1].scatter([x for x, _ in coll_pts], [c for _, c in coll_pts], s=14, c="0.5", label="NS-3 RB-overlap")
ax[1].set(title="Model 1: collision vs density", xlabel="in-range n", ylabel="P_COL", ylim=(0, 1.02)); ax[1].legend(fontsize=8)
if rows:
    rs = sorted(rows)
    ax[2].plot([n for _, n, *_ in rs], [m for *_, m, _, _ in [(r[0], r[1], r[2], r[3], r[4]) for r in rs]], "b-o", label="M3 (full RRI)")
    ax[2].plot([n for _, n, *_ in rs], [r[3] for r in rs], "b--s", label="M3 (T2 sel)")
    ax[2].plot([n for _, n, *_ in rs], [r[4] for r in rs], "k-^", label="NS-3")
    ax[2].set(title="Aggregate PRR vs density", xlabel="in-range n", ylabel="PRR@range", ylim=(0, 1.02)); ax[2].legend(fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(OUTDIR, "analytical_m3_sanity.png"), dpi=150)
print(f"\nfigure -> {os.path.join(OUTDIR, 'analytical_m3_sanity.png')}")

# ================================================================ params CSV
with open(os.path.join(OUTDIR, "analytical_m3_params.csv"), "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["param", "value", "provenance"])
    w.writerow(["tx_power_dbm", TX_DBM, "teacher txPower"])
    w.writerow(["noise_floor_dbm", round(NOISE_DBM, 3), "-174+10log10(18.72e6)+7"])
    w.writerow(["fc_ghz", FC_GHZ, "spatCarrierFrequencyGHz"])
    w.writerow(["h_tx_m", H_TX, "V2V antenna"]); w.writerow(["h_rx_m", H_RX, "V2V antenna"])
    w.writerow(["sigma_los_db", SIG_LOS, "TR37885 V2V-Urban"])
    w.writerow(["sigma_nlosv_db", round(SIG_NLOS, 3), "TR37885 shadow (+) NLOSv excess std in quadrature"])
    w.writerow(["channel", "TR37885_V2V_Urban", "same channel as NS-3 / ns3learn (fair comparison)"])
    w.writerow(["p_hd", P_HD, "t_s/T_RRI, Cao"])
    w.writerow(["n_r", round(N_R_RRI, 1), "T_RRI*N_sc/t_s * usable, Cao Eq(nr)"])
    w.writerow(["pi0", round(PI0, 5), "Cao stationary reselect 1/11"])
    w.writerow(["p_k", P_K, "teacher slProbResourceKeep"])
    w.writerow(["pc5_range_m", PC5_RANGE, "pc5Range"])
    w.writerow(["bler_curve", os.path.relpath(CDL_CURVE, HERE), "5G-Toolbox CDL (independent)"])
print(f"params -> {os.path.join(OUTDIR, 'analytical_m3_params.csv')}")
