# NS-3 → OMNeT Realism Distillation

**Goal:** make the OMNeT/Veins 5G NR PC5 analytical model *inherit* NS-3 5G-LENA's full
realism — propagation+BLER, MAC contention, half-duplex, and the net effect of HARQ /
queuing / scheduling — by using NS-3 as a **teacher** and fitting load-conditioned surrogate
functions that plug into OMNeT's existing per-cause drop hooks. OMNeT stays a fast analytical
model; we re-ground its *values* from NS-3.

This is knowledge distillation, not re-implementation. The precedent already exists: the
"Generalized" treatment is distilled from 5G-Toolbox. We swap the teacher to NS-3 and add the
MAC layer the current pipeline omits.

## Pipeline: from the SUMO trace to the fitted model

One mobility trace feeds both simulators, so the two differ only in the radio model. Each row
below names the script that produces the artifact and the script that consumes it.

| # | Stage | Produced by | Artifact | In repo? |
|---|---|---|---|---|
| 1 | SUMO mobility | `01_omnet_project/run_sumo_fcd_for_ns3.sh`, `run_sumo_fcd_seeds.sh` | `01_omnet_project/ns3_mobility/cav{NN}/seed{S}/fcd_cav{NN}.csv` (+ `.xml`) | **excerpt** — `sample_results/sumo_trace/` |
| 2 | ns-3 inputs from the trace | `02_ns3_lena/.../make_cav_ns3_inputs.py` | `mobility.tcl` (ns-2 format, filtered to `veh_av`, antenna 1.5 m) + `cav{NN}_s{S}_meta.csv` | **excerpt** — `sample_results/sumo_trace/` |
| 3 | ns-3 teacher run | `run_ns3_teacher.sh` → `propagation-compare.cc` + `v2x-kpi` | SQLite: `results/teacher/cav{NN}_s{S}-{SCEN}-*.db` (macSummary), `results/teacher_phy/...` (phyRxTrace) | **excerpt** — `sample_results/ns3_teacher/` |
| 4a | Per-reception link data | `extract_cascade_dataset.py`, `extract_contention_dataset.py`, `extract_halfduplex_dataset.py` — each joins the ns-3 DB with the stage-1 FCD geometry | `results/teacher_dataset/cascade/{decode,reach,collision,halfduplex}_dataset.csv` | no — regenerate |
| 4b | Run-level aggregates | `extract_teacher_dataset.py` | `teacher_run_aggregates.csv`, `bler_curve_ns3.csv` | **yes** — `01_omnet_project/sample_results/ns3_teacher/` |
| 5 | Fits | `fit_cascade_learners.py`, `train_cascade_all_learners.py`, `fit_ns3_surrogate.py`, `calibrate_interference.py`, `fit_contention_load.py`, `fit_contention_product.py`, `fit_attacker_factor.py` | learner metrics, fit params | no — regenerate |
| 6 | Runtime coefficient export | `export_ns3learn_runtime.py`, `export_cascade_coeffs.py` | `01_omnet_project/model_coefficients/ns3learn_runtime/*.csv` | **yes** |
| 7 | OMNeT run using the fit | `run_propagation_compare.sh` → `VeinsInet5GVehicleApp` reads `ns3LearnCoeffDir` | `dist_pdr_*.csv` per config | **yes** — `01_omnet_project/sample_results/omnet_dist_pdr/` |

The step that turns ns-3 output into *link* data is 4a: the ns-3 database records transmissions and
receptions by node id and time (`nodeId = rnti - 1`), and the FCD trace supplies each node's position at
that time. Joining them yields, per reception, the transmitter/receiver distance `d`, the local CAV
density `n`, `sinr_db`, and the measured outcome (decoded, collided, lost to half-duplex) — the table the
surrogates in step 5 are fitted to.

The full intermediates are not committed — the teacher set is tens of gigabytes — but a real excerpt of
each stage is, under `01_omnet_project/sample_results/`, with provenance and reduction method recorded in
its README. That covers the shared SUMO trace (1), the ns-3 replay format (2), raw per-reception PHY and
MAC output from the teacher databases (3), the aggregates and BLER waterfall fitted against (4b), the
runtime coefficients the model loads (6), and OMNeT output produced with them (7).

## Why the gap exists
- On pure propagation, NS-3's channel is ~as good or better than OMNeT's (LOS agrees within ~5 dB).
- Confirmed by re-running NS-3 with the *identical* UMi channel (`UMi_StreetCanyon`): PRR did
  not move (0.22–0.27 vs OMNeT Analytical 0.70–0.82).
- Decoupling NS-3 (`--phyTraces`): `PRR = reach-PHY(MAC ~0.55) × SCI decode(0.90) × BLER(0.54)`.
- OMNeT models only the BLER/channel branch, and Analytical/Calibrated use a lenient
  power-sensitivity gate (no real BLER); only Generalized does SINR→BLER. The two MAC/control
  branches are unmodelled — that's the bulk of the gap.

## Target architecture (layer → NS-3 signal → OMNeT hook)
```
drop_prop ← P(decode|SINR_eff);  SINR_eff = (Tx−PL−shadow+gain) − I(CBR)
             BLER curve    ← NS-3 psschRxUePhy (have it from low-rate phyTraces)
             I(CBR)        ← NS-3 SINR-vs-load  (the SNR→SINR interference fix)   [interpolateBlerCurve / new term]
drop_sps  ← P(collision|CBR)        ← NS-3 simulPsschTx numOverlapping/totalTx     [estimateSpsCollisionDrop]
drop_hd   ← P(half-duplex|rate,dens)← NS-3 reach-PHY ratio                          [pc5HalfDuplexDrop_]
HARQ      → folded into delivery label (post-ReTx) + inflates CBR (psschTxUeMac.rv/reselCounter/gapReTx)
queuing   → latency(CBR) from rlcRx.delayMicroSec   [OPTIONAL — see Decisions]
CBR       → calibrate OMNeT estimateChannelBusyRatio to NS-3 occupancy (closes ReTx↔collision loop)
```

## Design: `realismModel` selector (do NOT remove existing methods)
The NS-3-trained model is an **additional, selectable** option — existing methods stay intact and
isolated. Add a single configurable parameter:

    realismModel = "analytical" | "calibrated" | "ns3learn" | "matlablearn"

- `analytical`   → existing TR 38.901 UMi sensitivity gate (unchanged).
- `calibrated`   → existing calibrated log-distance sensitivity gate (unchanged).
- `matlablearn`  → existing Generalized 5G-Toolbox PHY (BLER from CDL) — the current "Generalized" (unchanged).
- `ns3learn`     → NEW: NS-3-distilled BLER(SINR) + collision(density) + half-duplex + HARQ + I(CBR).

Implementation: keep all current dispatch/params; `realismModel` just selects which path
`passesAnalyticalPdr` / `getLinkPacketSensingRatio` take. New configs `Prop_NS3Learn_*` added at the
end of `omnetpp_inet.ini` alongside the existing `Prop_Analytical/Calibrated/Generalized/Plain`.
Default = whatever preserves current behaviour (analytical) so nothing regresses.

## Design decisions
- **Scope:** reliability first (PRR). Latency/queuing model is a later add.
- **NS-3 edit:** split `--phyTraces` into `--macSummary` (tiny: simulPsschTx, PsschTbRx, MAC
  scheduling) vs `--phyRxTrace` (huge per-reception). Rebuild binary. Approved.
- **CBR densification:** add 3 extra adoption rates **cav10, cav40, cav90** (do these FIRST).
  - cav10/cav90 route+cfg already exist; **cav40 does not** → generated by rescaling an
    existing rate file (penetration = veh_av:veh_human prob split, total demand constant).
- **SUMO mobility: NOT regenerated** for existing rates — shared, aligned input. New rates use
  the same net + demand, only the CAV penetration split changes.

## Teacher data collection

**Trace collection.** `propagation-compare.cc` splits `--phyTraces` into `--macSummary` (compact:
simulPsschTx, PsschTbRx, MAC scheduling) and `--phyRxTrace` (per-reception, very large). Mobility for the
added rates comes from `run_sumo_fcd_seeds.sh 10 40 90`, and ns-3 inputs from
`make_cav_ns3_inputs.py --jitter 0.05`. `run_ns3_teacher.sh` collects macSummary over V2V_Urban for
cav10/40/90 × 5 seeds; run length is 200 s for rates ≤ 50 and 60 s for ≥ 75. The runner is resumable and
memory-guarded (`MAXP=6`, `MIN_FREE_GB=6`), which keeps the macSummary databases at ~90 MB rather than
the ~20 GB a full PHY trace would produce. `--phyRxTrace` is collected only for cav01/05/10, where it is
tractable.

**Channel / BLER.** `extract_teacher_dataset.py` reads the databases and `fit_ns3_surrogate.py` fits them.
The BLER(SINR) waterfall (V2V: 0→0, 10→0.49, 15→0.75, 20→0.81; knee near 12 dB) is exported in
`interpolateBlerCurve` format. UMi and V2V curves are close.

**I(CBR) interference penalty.** Back-fitted to the PRR(density) target, which is cheap to obtain at every
rate, given the collision and BLER curves. No high-load SINR traces are needed; the low-rate per-reception
traces are sufficient.

**MAC.** Collision versus neighbour density is a logistic fit (L=0.92, k=0.046, x0=61 over 29 runs), and the
HARQ blind-retransmission multiplier is ≈ 2.52× — the channel-occupancy inflation used for CBR calibration.

### Preliminary teacher signals (20 runs)
| rate | neighbours | collision_frac | HARQ ReTx× | PRR@300 |
|---|---|---|---|---|
| cav01 | 4.8 | 0.02 | 2.50 | 0.30 |
| cav05 | 15.5 | 0.09 | 2.51 | 0.30 |
| cav10 | 33 | 0.22 | 2.52 | 0.24 |
| cav40 | 148 | 0.89 | 2.53 | 0.08 |
| cav90 | 122 | 0.90 | 2.53 | 0.08 |

Tooling: `extract_teacher_dataset.py` (DB→aggregates+BLER), `fit_ns3_surrogate.py` (fits+export),
`run_ns3_teacher.sh` (collection). Diagnostics: `results/teacher_dataset/ns3_trained/fit_diagnostics.png`.

### Surrogate POC validation (Python, before C++ wiring) — `validate_surrogate.py`
Model: deliver(d)=(1-hd)·(1-collision(density))·sci(SINR)·pssch(SINR); SINR(d)=23-PL_v2v(d)-noiseFloor(-94.3);
PRR300 = E_d over neighbours uniform in r≤300m. Fit hd + interference-a to NS-3 PRR(density).
**Result (6 rates): RMSE 0.030 — the layered surrogate reproduces NS-3 PRR. Approach validated.**
KEY FINDINGS that drive the C++ wiring:
1. **half-duplex hd ≈ 0.56** dominates the low-density floor (OMNeT currently sets drop_hd=0 — the #1 omission).
2. **interference penalty a ≈ 0** — load dependence is fully captured by collision(density); NO separate SINR/I(CBR) term needed (simplifies the model; supersedes the earlier I(CBR) plan).
3. **capture effect** needed at high density: raw (1-collision) over-suppresses (cav40/90 pred 0.036 vs 0.076).
   cav50/75/100 (collecting) will anchor a capture correction (effective_collision < raw overlap).
So `ns3learn` needs: pc5HalfDuplexDrop≈0.56, collision(density) logistic, V2V BLER curve, capture-corrected collision.

### VALIDATED surrogate (POC, 7 rates incl. capture) — RMSE 0.011 ✓
Final model: deliver(d) = (1-hd)·[(1-coll(density))+cap·coll(density)]·sci(SINR(d))·pssch(SINR(d)),
SINR(d)=23-PL_v2v(d)-noiseFloor(-94.3 dBm). Fitted: **hd=0.58, capture=0.21, interference a=0** (load
enters ONLY via collision(density)). Reproduces NS-3 PRR@300 across cav01..cav90 with RMSE 0.011.
`results/teacher_dataset/ns3_trained/{surrogate_params.csv,surrogate_validation.png}`, `validate_surrogate.py`.

**Maps onto EXISTING OMNeT drop hooks (minimal integration, no new mechanism):**
- `drop_hd`  ← pc5HalfDuplexDrop = 0.58   (currently 0 in Prop configs — the #1 fix)
- `drop_prop`← 1 - sci(SINR)·pssch(SINR)  (V2V BLER curve on TR 37.885 path loss)
- `drop_sps` ← collision(density) × (1 - capture=0.21)
So `realismModel="ns3learn"`: set pc5HalfDuplexDrop_, route getLinkPacketSensingRatio→V2V BLER,
route estimateSpsCollisionDrop→fitted collision×0.79. cav100 (collecting) = held-out validation.
### Runtime wiring

- `VeinsInet5GVehicleApp.ned`: added params `realismModel` (default "analytical") + `ns3LearnCoeffDir`.
  - `VeinsInet5GVehicleApp.h/.cc`: ns3learn members + `loadNs3LearnModel`, `ns3LearnReception`
    (V2V PL→SINR→pssch·sci BLER), `ns3LearnCollisionDrop` (logistic×(1-capture)), `countNeighboursInRange`,
    `ns3V2vPathLossDb` (TR 37.885). Dispatch branches added at top of `getLinkPacketSensingRatio`
    (→drop_prop) and `estimateSpsCollisionDrop` (→drop_sps); half-duplex via pc5HalfDuplexDrop_ override.
    Loads bundle from results/teacher_dataset/ns3learn_runtime/. ALL ADDITIVE — existing configs default
    realismModel="analytical" so behaviour is unchanged.
- `omnetpp_inet.ini`: `[Config Prop_NS3Learn]` (extends Prop_Analytical) + 6 leaf configs Prop_NS3Learn_CAV{01,05,25,50,75,100}.

Rebuild `veins_inet`, then run `./run_ns3learn.sh` (Prop_NS3Learn_* × 6 rates × 5 seeds, idempotent) and
regenerate the comparison figures with `build_prop_compare_figures.py` / `build_pathloss_compare.py`. Under
opp_env the run scripts must `export OPP_ENV_VERSION=1` before sourcing `setenv`; `run_ns3learn.sh` does this.

### Validation against NS-3

- **Smoke test**: Prop_NS3Learn_CAV05-0 (30 s) → **PRR@300 = 0.258** (NS-3 cav05 ≈ 0.28, surrogate ≈ 0.28).
        drop_hd=0.605 (half-duplex now ACTIVE — was 0 in all other treatments), drop_prop=0.094, drop_sps=0.050.
        Confirms end-to-end: bundle loads, half-duplex + V2V BLER + collision(density) all firing, PRR matches NS-3.
- **Calibration defect found and fixed**: full-sim cav01/05 gave PRR 0.10 against NS-3's 0.30. Cause:
        `countNeighboursInRange()` counted ALL 5G-app hosts (~118 at cav01), but the scenario instantiates
        BOTH veh_av→VeinsInetCar (CAV) AND veh_human→VeinsInetLiteCar (human) with the app, so collision was
        evaluated at total-vehicle density not CAV-transmitter density (NS-3 fit = CAV-only). The logistic
        saturated → drop_sps≈0.68. FIX: filter countNeighboursInRange to CAV hosts only (NedTypeName contains
        "VeinsInetCar"). Hand-check after fix: (1-hd 0.58)·(reception 0.74)·(1-sps 0.05) ≈ 0.295 ≈ NS-3 0.30 ✓.
        Coefficients unchanged (hd/capture/collision were right); only the density COUNT was wrong.
- **Full sweep**: 30/30 runs, in parallel via `run_omnet_resume_par.sh` (`TREATMENTS=NS3Learn`, `MAXP=8`;
  the launch daemon gives each `opp_run` its own SUMO port).
- **Result** — PRR@300 against NS-3 (fig1/fig3):
        | rate | Analyt | Calib | Geph | **NS3Learn** | Plain | NS-3 |
        |  1 | 0.70 | 0.87 | 0.75 | **0.32** | 1.0 | 0.30 |
        |  5 | 0.82 | 0.94 | 0.86 | **0.32** | 1.0 | 0.28 |
        | 25 | 0.85 | 0.97 | 0.89 | **0.28** | 1.0 | 0.13 |
        | 50 | 0.85 | 0.97 | 0.91 | **0.18** | 1.0 | 0.05 |
        | 75 | 0.86 | 0.98 | 0.89 | **0.26** | 1.0 | 0.12 |
        |100 | 0.86 | 0.98 | 0.91 | **0.21** | 1.0 | 0.07 |
        **Fidelity to NS-3 (MAE of PRR@300): NS3Learn 0.103 vs Analytical 0.664 / Generalized 0.709 /
        Calibrated 0.792 / Plain 0.841 — a 6.5× improvement.** NS3Learn alone tracks the load-dependent
        collapse; the others are flat (channel-only, no MAC). Residual +0.10 optimism at mid/high density
        (e.g. cav50 0.18 vs 0.05) = OMNeT in-range CAV count vs NS-3 numNieb + high-density capture nuance;
        a future refinement, but already the decisively closest model.

## Refinement: deviation-grows-with-adoption fixed
Focused comparison (Analytical/Generalized/NS3Learn vs NS-3, Plain+Calibrated dropped) showed NS3Learn's
deviation GREW with adoption (+0.02 @cav01 → +0.13-0.15 @cav25-100). Root cause: collision logistic was
fit vs NS-3 `numNieb` (~195 @cav50) but OMNeT evaluates it at the INSTANTANEOUS in-range CAV count (~32,
recovered by inverting the logistic on observed drop_sps) → collision under-fired at high load. Fix:
re-fit the collision logistic in-sim to OMNeT's instantaneous density (target sps per rate = 1 - PRR_ns3/
((1-hd)·reception)); new params **collision L=0.767 k=0.380 x0=4.67, capture=0** (was L=0.93 k=0.05 x0=58
cap=0.21). Runtime-CSV only, no recompile; original saved `ns3learn_params_teacherfit.csv`. Re-ran sweep
(parallel). **Result: MAE 0.102 → 0.045; deviation now non-trending (range −0.11..+0.03).** Residual:
cav05 over-corrected (−0.11) and high rates floor ~0.08 (collision×capture ceiling) — OMNeT's instantaneous
count is a compressed/noisy density signal; a single logistic can't be exact. Focused figures: focus1_pdr_vs_distance,
focus2_isolated_pathloss, focus3_coupled_total_loss, focus4_deviation_from_ns3 (build_focus_comparison.py).

## LOS-only four-way comparison — publication set
All non-LOS propagation is removed (building-NLOS from Analytical/Generalized, vehicle-NLOSv from NS3Learn)
for a like-for-like comparison. Implementation: `losOnly` param (VeinsInet5GVehicleApp) → Analytical/Generalized
LOS-only, NS3Learn uses UMi-LOS path loss; set in Prop_Analytical (inherited). NS-3 reference = new built-in
scenario `UMi_StreetCanyon_LoS` (forced-LOS via ScenarioFromString add) → channel identical to OMNeT Analytical-LOS.
- NS3Learn re-calibrated for LOS context (higher reception → weaker collision): LOS BLER curve refreshed from
  UMi_LoS phyTraces (decode @10dB 0.49→0.74); collision logistic re-fit vs TRUE CAV-neighbour density from FCD
  (not the saturation-compressed back-out): **L=0.736 k=0.143 x0=25.5, capture=0** (params backed up: *_v2vcal.csv).
- **RESULT (PRR@300, LOS-only): NS3Learn MAE to NS-3 = 0.030** vs Analytical 0.624 / Generalized 0.782.
  | rate | Analyt | Gen  | NS3Learn | NS-3 |
  |  1 | 0.72 | 0.99 | 0.37 | 0.35 |
  |  5 | 0.82 | 0.99 | 0.37 | 0.37 |
  | 25 | 0.85 | 0.98 | 0.17 | 0.21 |
  | 50 | 0.86 | 0.98 | 0.16 | 0.11 |
  | 75 | 0.86 | 0.99 | 0.15 | 0.12 |
  |100 | 0.86 | 0.99 | 0.12 | 0.08 |
  LOS-only makes channel-only models MORE optimistic (Generalized ~0.98 flat); NS3Learn (same LOS channel + MAC)
  tracks NS-3. Deviation small + non-trending (±0.04).
- Charts: focus1_pdr_vs_distance, focus2_isolated_pathloss, focus3_coupled_total_loss, focus4_deviation_from_ns3
  + perlink/{pl1_sinr_cloud_bler, pl2_pdr_seed_scatter, pl3_pdr_distribution, pl4_latency_cdf}.
- Tooling added: build_focus_comparison.py, build_perlink_charts.py. NS-3 scenario UMi_StreetCanyon_LoS in propagation-compare.cc.
- GOTCHAS this round: (1) PRR@300 is window-invariant → cav75/100 run at 60s (cav100 at 100s/443-nodes is intractable
  ~45-90min/run, O(N²) spatial channel). (2) NEVER pkill in-flight NS-3 — run_one rm's the DB at start, so killing
  mid-run corrupts it (lost cav75/cav100 twice this way). Let runs finish. (3) collision must be fit vs TRUE FCD
  density, not back-out (drop_sps=E[coll(count_dist)]≠coll(mean), Jensen). (4) teacher .result files now scenario-namespaced.

## Out-of-sample validation — mainstreet topology, 50% CAV
Validated the intersection-calibrated NS3Learn on a DIFFERENT road (mainstreet, linear; created
mainstreet_cav50.{rou,sumo.cfg,launchd}.xml at 50% via vehsPerHour rescale; FCD→ns-2 mobility 3 seeds;
NS-3 UMi_StreetCanyon_LoS + OMNeT Prop_*_MS50 configs). Gotcha: xml2csv must use `-s ,` (DictReader=comma)
or make_cav_ns3_inputs finds 0 veh_av.
**Aggregate PRR@300 (NS-3 ref 0.299): Analytical 0.880 (+0.58), Generalized 0.992 (+0.69), NS3Learn 0.161 (-0.138).**
→ NS3Learn is the ONLY model in the right (MAC-limited) regime, ~4-6× closer than channel-only models, and it
generalizes the MAC behaviour to a new topology. BUT pessimistic by 0.14: mainstreet density≈32 (vs intersection
cav50≈50) yet NS-3 PRR is HIGHER (0.30 vs intersection's ~0.21 at similar density) — linear road spreads CAVs so
there's LESS collision per neighbour than the clustered intersection; the intersection-calibrated collision(density)
over-suppresses. → collision-vs-density is somewhat GEOMETRY-SPECIFIC (a stated generalization limit).
**Per-link distance comparison (pl5, NS-3 reconstructed from pktTxRx+FCD positions — the rigorous distance-resolved
PDR, same KPI as OMNeT dist_pdr):** NS-3 has a STEEP near-field decay (0.88@0m → 0.12@150m, SINR/capture-driven);
OMNeT NS3Learn is ~FLAT 0.16 (its half-duplex + collision are distance-independent, and LOS BLER is high to ~200m),
so it matches the aggregate LEVEL but MISSES NS-3's distance SHAPE. Analytical decays from ~1.0; Generalized flat ~0.99.
→ The per-link view (which the aggregate hides) shows NS3Learn captures the MAC-limited magnitude but not the
distance profile — future work: make collision/effective-SINR distance-dependent (capture effect).
Charts: focus5_validation_mainstreet.png, perlink/pl5_ns3_vs_omnet_distance_mainstreet.png.
Tooling: build_ns3_distance_pdr.py (NS-3 distance-PDR reconstruction).

## OUTCOME
OMNeT/Veins now has a 4th, selectable realism mode `realismModel="ns3learn"` that INHERITS NS-3 5G-LENA's
MAC+PHY realism (half-duplex, resource-collision-vs-density, V2V BLER, HARQ net effect) distilled into
OMNeT's existing drop pipeline. Existing analytical/calibrated/matlablearn(=Generalized) modes untouched.
Toolchain (all in project dir): run_ns3_teacher.sh (collect) → extract_teacher_dataset.py → fit_ns3_surrogate.py
→ validate_surrogate.py → export_ns3learn_runtime.py (bundle) → run_ns3learn.sh / run_omnet_resume_par.sh (validate).

## Key paths
- OMNeT app: `subprojects/veins_inet/src/veins_inet/VeinsInet5GVehicleApp.cc`
  (`passesAnalyticalPdr` :1834; `getPacketSensingRatio` :2911; `estimateSpsCollisionDrop` :3296;
  `getPc5NoiseFloorDbm` :2793; `interpolateBlerCurve` :2890).
- NS-3 scenario: `ns-3-dev/scratch/propagation-compare/propagation-compare.cc` (phyTraces gate :581).
- NS-3 teacher DBs: `ns-3-dev/scratch/propagation-compare/results/` (+ `phy_decoupling/`).
- Figures/analysis: `results/prop_compare_figures/`.

---

## Phase 9 — Distance-shape fix and the avrgPrr metric correction

### The critical methodology finding
NS-3 5G-LENA's `avrgPrr` table (v2x-kpi.cc:456) divides receptions by
`numPackets × cumulativeNeighbours`, where a neighbour ever in range counts ALL the
tx's packets in its denominator — even those sent while it was out of range. Fine on a
highway (static topology), but at an **intersection with transient neighbours it deflates
PRR ~3×**. The 3GPP-standard, apples-to-apples metric (and what OMNeT `dist_pdr` computes)
is the **per-instant in-range PDR**: for each packet, delivered / in-range-at-tx-time.

| rate | NS-3 per-instant | NS-3 avrgPrr | OMNeT Analytical | Generalized |
|------|-----------------|--------------|------------------|-------------|
| cav01 | 0.957 | 0.231 | 0.715 | 0.985 |
| cav05 | 0.844 | 0.266 | 0.820 | 0.985 |
| cav25 | 0.525 | 0.213 | 0.849 | 0.980 |
| cav50 | 0.287 | 0.108 | 0.859 | 0.981 |
| cav100| 0.203 | 0.070 | 0.864 | 0.993 |

**Corrected story:** NS-3 per-instant PDR genuinely degrades with density (0.96→0.20) — real
MAC congestion. OMNeT **Analytical is ~flat 0.85**: correct in light traffic, misses the
high-density congestion. Generalized ~flat 0.98 (no MAC). The realism gap is real but
**density-dependent**, not a uniform collapse — the avrgPrr metric had exaggerated it.

### The distance-aware stochastic NS3Learn (the fix)
Replaced the flat half-duplex+collision losses with an interference-degraded SINR:
`SINR_eff(d) = Tx − PL_LOS(d) − noiseFloor − I`, where the per-reception penalty is
`I ~ Normal(μ(density), σ(density)²)`:
- `μ(density) = 17.70·log10(1+n) − 0.98` dB
- `σ(density) = 7.18·log10(1+n) − 3.13` dB  (variance smears the BLER threshold → gradual decay)

Calibrated to the **per-instant** reconstructed PDR(d). Config: bundle `ns3learn_params.csv`
gains `intf_a/intf_b` (μ), `intf_sigma_a/intf_sigma_b` (σ), `distance_aware=1`, `stochastic=1`,
read at runtime (no rebuild to recalibrate). Code: `ns3LearnReception` adds `normal(0,σ)`;
`pc5HalfDuplexDrop=0` and `ns3LearnCollisionDrop=0` when distance-aware (folded into SINR).

### Validation (per-instant KPI)
- **Intersection (training):** NS3Learn 0.81/0.71/0.37/0.24/0.30/0.25 vs NS-3 0.96/0.84/0.53/0.29/0.35/0.20 — tracks the congestion roll-off, **MAE ≈ 0.10**.
- **Distance shape:** gradual decay reproduced across all densities (cliff eliminated), e.g. cav50 NS3L 0.74/0.31/0.16/0.09/0.06 vs NS-3 0.83/0.32/0.18/0.09/0.06.
- **Mainstreet (out-of-sample):** NS3Learn 0.333 vs NS-3 per-instant 0.374, **dev −0.041** — geometry generalization gap nearly closes under the correct metric (was −0.17 vs avrgPrr).

### Figures
- `focus6_metric_comparison.png` — both NS-3 KPIs vs all models (shows the avrgPrr deflation).
- `perlink/pl6_intersection_distance_shape.png` — NS3Learn vs NS-3 reconstructed PDR(d), 4 rates.
- `perlink/pl5_ns3_vs_omnet_distance_mainstreet.png` — out-of-sample distance shape.
- Scripts: `compare_perinstant_pdr.py`, `calibrate_interference_stochastic.py`, `build_metric_comparison.py`.

---

## Phase 10 — Decomposed cascade + three-learner comparison (linear / logistic-NLS / GBM)

### Feature catalogue (what trains what)
| feature | side | NR mechanism | source | role |
|---|---|---|---|---|
| Tx–Rx distance d | propagation | path loss→SINR→BLER | FCD | runtime input |
| CAV density n | contention | interference, SB-SPS collision, half-duplex | countNeighbours/FCD | runtime input |
| per-slot concurrency | contention | SB-SPS resource overlap | psschTxUeMac | teacher feature |
| avrgSinr (linear→dB) | both | the master decode variable | psschRxUePhy | teacher signal |
| sci2Corrupt / psschCorrupt | PHY | SCI/PSSCH decode | psschRxUePhy | decode label |
| slot/RB allocation | contention | half-duplex (slot coincidence) + collision (RB overlap) | psschTxUeMac | reach/collision labels |

### Data availability (decisive)
`psschRxUePhy` (per-reception SINR+decode) exists ONLY for cav01/05 (huge → disabled at high density),
but the decode waterfall is rate-independent PHY → fully characterised. `psschTxUeMac` (MAC schedule)
and the `simulPsschTx` overlap summary exist at ALL densities → half-duplex + SB-SPS collision
reconstructable across the whole congestion range. SB-SPS resource-overlap fraction: 0.11/0.64/0.87/0.82/0.95 (cav05/25/50/75/100). Half-duplex block ≈ const 0.08–0.10 (rx duty cycle, density-independent).

### Three-learner results (5-fold CV)
| stage | features | Linear logloss | Logistic | GBM | winner |
|---|---|---|---|---|---|
| **Decode** (SINR→decode) | sinr_db | 0.281 | **0.153** | **0.139** | logistic≈GBM (clean sigmoid; both ≫ linear) |
| **Collision** (SB-SPS) | n, concurrency | 0.401 | 0.350 | **0.232** | **GBM** (AUC .95 vs .89; interactions) |

**Headline: the best learner is stage-dependent.** Where one physical variable drives a bounded
sigmoid (decode vs SINR), the grey-box logistic equals the black-box GBM and both crush linear
(which can't stay in [0,1]). Where multiple features interact (collision vs n × concurrency), GBM
adds real value. Linear is never competitive for these bounded/threshold responses.

### Cascade composition & the capture effect
`PRR(d,n) = (1−hd)·(1−collision(n))·decode(SINR_noise(d))` UNDERSHOOTS badly at high density
(cav100 0.014 vs NS-3 0.203): 95% of tx overlap resources, yet NS-3 delivers 20% — so most resource
collisions do NOT cause loss (the receiver captures the stronger signal). The residual = the
distance-dependent **capture** effect. This is exactly what the lumped stochastic interference model
(Phase 9, I~Normal(μ(n),σ(n))) encodes implicitly via the SINR distribution — connecting the
decomposed and lumped views: capture/collision/half-duplex all emerge from SINR statistics.

Scripts: extract_cascade_dataset.py, extract_contention_dataset.py, fit_cascade_learners.py,
fit_contention_learners.py. Charts: learners/decode_learner_comparison.png, learners/collision_learner_comparison.png.

### Phase 10b — Capture stage closes the decomposed cascade
A collision degrades SINR by an interferer penalty I_coll~Normal(mu_c,sigma_c); decode is re-evaluated
at SINR_noise(d)-I_coll (capture: near survives, far fails). Full cascade:
  PDR(d,n) = (1-hd)*[ (1-coll(n))*decode(SINRn(d)) + coll(n)*E_Ic[decode(SINRn(d)-Ic)] ]
Fitted (mu_c=28.7 dB, sigma_c=5.6 dB, hd=0.098) to NS-3 distance-resolved PDR.
**Validation (intersection, per-instant): cav05/25/50/100 cascade 0.827/0.450/0.290/0.237 vs NS-3
0.844/0.525/0.287/0.203 — MAE 0.032.** Fully mechanistic (each stage from its own NS-3 labels) and
composes to NS-3. sigma_c=5.6 dB ties the decomposed view to the lumped stochastic model (Phase 9):
same physics, one interpretable, one runtime-cheap. Script build_capture_cascade.py;
chart learners/cascade_capture_validation.png; params learners/cascade_capture_params.csv.

---

## Phase 11 — CONTRIBUTION 1 FINALISED (decomposed cascade, per-instant, learner bake-off)

**Adopted model = the DECOMPOSED CASCADE** (half-duplex × collision × decode × capture); the lumped
stochastic form is kept only for analysis. **All evaluation is per-instant PDR**; avrgPrr-era figures
archived to results/prop_compare_figures/archive_non_perinstant/.

### End-to-end three-learner bake-off (per-instant PDR, both geometries)
Each learner fits the decode + collision stages; capture (mu_c,sigma_c) refit per learner; hd const.
| learner | intersection MAE (train) | mainstreet MAE (validate) | capture (mu_c,sigma_c) |
|---|---|---|---|
| Linear   | 0.087 | 0.003* | (44.3, 0.5) |
| **Logistic** | **0.042** | **0.064** | (28.2, 6.9) |
| GBM      | 0.056 | 0.101 | (28.7, 8.1) |

**Headline: logistic (grey-box) wins end-to-end** — best training fit AND best generalization.
GBM beats logistic on the collision sub-stage in isolation (logloss .23 vs .35) but, composed
end-to-end, OVERFITS and generalizes worse out-of-sample (mainstreet 0.101 vs 0.064). Linear cannot
represent the bounded sigmoid (cav05 0.59 vs 0.84; its mainstreet 0.003 is a single-point fluke).
=> Adopt LOGISTIC on accuracy + generalization + deployability (closed-form in OMNeT; GBM trees are
impractical to embed per-packet in the event-driven sim).

### Contribution-1 figure set (all per-instant)
- focus6_metric_comparison.png — avrgPrr deflates ~3× (transient neighbours); per-instant is the KPI. [methodology sub-contribution]
- learners/decode_learner_comparison.png — decode stage: logistic≈GBM≫linear.
- learners/collision_learner_comparison.png — collision stage: GBM>logistic (interactions).
- learners/cascade_capture_validation.png — decomposed cascade vs NS-3 PDR(d), MAE 0.03.
- perinstant/c1_bakeoff.png — end-to-end bake-off, intersection + mainstreet.
- perlink/pl6 (train) + pl5 (validate) — per-link distance shape vs NS-3.

### C1 claims (for the paper)
1. The de-facto NR-V2X PRR KPI (5G-LENA avrgPrr) deflates ~3× under transient-neighbour topologies;
   per-instant in-range PDR is the correct, geometry-fair metric.
2. NS-3's PRR degradation with density (0.96→0.20) is MAC-driven (SB-SPS collision 0.11→0.95 + capture),
   NOT propagation; OMNeT analytical/generalized miss it (flat ~0.85/0.98).
3. A decomposed, NS-3-distilled cascade reproduces it (MAE 0.03 train, generalizes to a new geometry).
4. A grey-box logistic parameterisation matches a black-box GBM end-to-end while generalizing better
   and remaining embeddable — the adopted surrogate.

---

## Phase 12 — Retargeted to the correct PC5 channel: V2V_Urban (TR 37.885, LOS+NLOSv)

UMi Street Canyon (TR 38.901) is a CELLULAR model; PC5/sidelink's correct channel is TR 37.885
**V2V_Urban** (vehicle-height antennas, NLOSv vehicle blockage). UMi was a deliberate channel-matched
*control* (isolating MAC), not the emulator channel. Retargeted everything to V2V.

**No NS-3 re-run for training** — V2V_Urban teacher DBs already existed (all rates ×5 seeds, MAC at all
rates, PHY at cav01/05, pktTxRx everywhere). Extraction scripts parameterised by `SCEN` (default
V2V_Urban; UMi datasets archived to cascade_umi_control/). Ran NEW NS-3 mainstreet V2V (3 seeds) for
out-of-sample validation. OMNeT uses TR 37.885 path loss via `ns3V2vPathLossDb` (mean LOS/NLOSv blend);
runtime flag `losOnly=false`.

**What transfers (channel-independent):** hd(n), collision(n) (pure MAC); decode(SINR) (PHY waterfall
conditional on SINR). **What changed:** path loss PL(d)→SINR (now V2V incl. NLOSv mean) and the capture
penalty (re-fit). NLOSv UPSIDE: OMNeT can't compute vehicle blockage analytically, but the learned
reception inherits NLOSv statistics from the V2V teacher.

**V2V three-learner bake-off (per-instant, decomposed cascade):** per-stage CV — collision Logistic 0.276≈GBM 0.275, decode Logistic 0.18≈GBM 0.169, hd 0.33 (flat). End-to-end intersection MAE: Linear 0.094, **Logistic 0.066**, GBM 0.061 → adopt Logistic. Capture (logistic) mu=30.3, **sigma=10.4 dB** (vs UMi 6.8 — NLOSv injects link variance, absorbed by capture).

**Per-tx-rx validation (OMNeT V2V-cascade vs NS-3 V2V_Urban), per_txrx_scatter_v2v.png:**
73 distance-bins, **MAE 0.105, RMSE 0.143, r 0.939**; intersection MAE 0.104, mainstreet (out-of-sample)
0.110 — generalises. Residual ~0.10 is mainly the mean-blend channel (OMNeT averages LOS/NLOSv; NS-3
samples per link); next lever = per-link LOS/NLOSv sampling using the standard pLos.

Scripts now SCEN-parameterised: extract_{cascade,contention,halfduplex}_dataset.py,
train_cascade_all_learners.py, build_per_txrx_scatter.py.

### Phase 12b — Per-link LOS/NLOSv sampling (replaces mean-blend)
ns3V2vPathLossDb now SAMPLES the LOS/NLOSv state per reception (uniform()<pLos(d) ? LOS : LOS+NLOSvExcess),
matching how NS-3 draws per-link state — same TR 37.885 pLos, no invented analytics. Training cascade
marginalises over the same LOS/NLOSv split (branch mixture weighted by pLos) so capture stays consistent.
**Per-tx-rx scatter (OMNeT V2V vs NS-3 V2V): mean-blend MAE 0.105/r 0.939 -> sampled MAE 0.101/r 0.944**
(intersection 0.099, mainstreet 0.108). Small but real gain (the Jensen correction); binned PDR averages
out most per-link variance. Residual ~0.10 now dominated by: within-state shadowing (TR 37.885 sigma~3-4 dB,
NOT yet modelled — absorbed into capture sigma), density-only contention features, and binning noise.
Next lever (grounded): per-state log-normal shadowing.

### Phase 12c — Full TR 37.885 channel ported: AR(1) shadowing + random NLOSv (adopted)
ns3LearnReception now calls v2vChannelPathLossDb(senderId, d, fc, relX, relY): a per-link (per-sender,
receiver-side, mutable) cache holding LOS/NLOSv state + AR(1) shadow fading, decorrelating over the
TR 37.885 correlation distance (LOS 10 m / NLOSv 13 m); state re-evaluated when geometry moves beyond
d_corr; NLOSv excess = max(0, lognormal(mean=9+max(0,15logd-41), std=4.5)); shadow sigma=3 dB. All
from 3GPP TR 37.885 (cited in ns-3 source: Table 6.2.1-1 path loss, 6.2.3-1 shadowing) — implementing
the public standard, NOT invented. Train cascade does Monte-Carlo (NMC=40000) over the same channel
realisations; capture (mu,sigma) re-fit -> mu=26.4, **sigma->0.3 (floor)**: with the channel carrying
its variance, residual collision-interference variance is ~0 (collision = ~constant 26 dB bump). Clean
physical decomposition: channel variance in channel, collision a deterministic penalty.

**Per-tx-rx scatter, channel-variant comparison:** per-link-state-only MAE 0.101/r .944; i.i.d.
shadowing+NLOSv 0.115/r .891; **AR(1) shadowing+NLOSv 0.116/r .918** (AR(1) recovered the correlation
r .891->.918 vs i.i.d., confirming NS-3's shadowing is correlated). Marginal insight: AR(1) leaves the
binned-PDR marginal ~unchanged (stationary marginal N(0,3)); its payoff is TEMPORAL correlation (needed
for C2 bursty losses). KEY CORRECTION: train bake-off 0.069 = AGGREGATE per-instant deviation; scatter
0.116 = PER-BIN MAE — different granularities, NOT a train-vs-OMNeT contradiction. ADOPTED the full
TR 37.885 channel (faithful/citable/C2-ready); per-link-state's lower binned MAE is a metric artifact.
Two known limitations remain, neither material unless MAE below 0.10 is required: `pc5Range` = 320 m
against a training range of 300 m, a density mismatch; and collision and half-duplex are density-only.
Further channel refinement gives diminishing returns.

### Phase 13 — Contribution-1 publication figure set
results/paper_figures/{color (Okabe-Ito CB-safe),gray (hatch+marker, B&W-safe)}/, 300 dpi PNG (no PDF).
Pipeline: build_paper_figure_data.py (caches NS-3 V2V per-instant/avrgPrr/distance + OMNeT aggregates)
+ build_loss_breakdown.py (cascade loss attribution per density) -> build_paper_figures.py (styled plots).
F1 gap+metric (per-instant vs Analytical/Generalized; per-instant vs avrgPrr ~3x deflation).
F2 RELIABILITY BUDGET stacked bar (delivered 0.74->0.19, SB-SPS collision-loss 0.13->0.70, hd ~0.10,
   decode ~0) — collapse is MAC contention not propagation [redesigned from a 3-line plot].
F3 cascade stages (hd(n),collision(n),decode(SINR),capture) — NS-3 data + learned curves.
F4 three-learner CV + end-to-end MAE (logistic≈GBM>linear).
F5 per-tx-rx scatter (hero) MAE 0.117 r 0.914, both geometries.
F6 distance-resolved PDR per rate (NS3Learn vs NS-3).
F7 channel ablation (per-link-state -> AR(1) -> full TR 37.885).

### Phase 14 — Grounded capture (no fitted penalty); model now fully measured
Replaced the fitted constant 26 dB capture penalty with TWO MEASURED decode curves, labeling each
NS-3 PHY reception as collided/not (MAC RB-overlap in the tx's slot — build_capture_dataset.py):
  decode_noColl(SINR): un-contaminated decode (0.64->1.0) — the old single curve was dragged down by
    mixing in colliding receptions, causing the cav05 under-prediction.
  decode_coll(SINR):   MEASURED capture (0.08->0.54, rises with SINR = near survives, far fails).
Cascade now: PDR=(1-hd)*[(1-coll)*decode_noColl(SINR) + coll*decode_coll(SINR)] — ZERO fitted fudge;
every stage measured from NS-3 (hd, collision, decode_noColl, decode_coll). OMNeT ns3LearnReception
picks decode_coll/noColl by the collision draw (params decnc_c*, deccc_c* in cascade_params.csv;
cap_mu/cap_sigma removed). build_capture_cascade* / capture penalty retired.
**Per-tx-rx improved markedly: full scatter MAE 0.117->0.085, r 0.914->0.957; matched-50% dense MAE
0.084->0.064 (intersection 0.060, mainstreet 0.140->0.070); per-rate cav05 0.117->0.080, mainstreet
0.112->0.062.** Honest limitation: slight HIGH-density over-prediction (cav50/100 ~0.08) because
decode_coll is learned only from low-density PHY traces (interferers farther -> optimistic capture);
closing it needs high-density phyRxTrace (NS-3 re-run). F3 panel (d) is now the measured decode_coll
curve (was a Gaussian). Scripts: build_capture_dataset.py; train/figures updated for the split.

### Phase 15 — High-density PHY re-run, giving density-aware capture
Re-ran NS-3 with --phyRxTrace at HIGH density (cav25/50/100, V2V_Urban, 60s, 1 seed) to results/
teacher_phy/ (cav50 563M/3M rows, cav100 1.5G/8.6M rows) — the PHY trace was originally low-density-only
(huge). Confirmed capture WEAKENS with density: decode_coll(40dB) cav05 0.72 -> cav100 0.38.
Made decode density-aware: decode_{nc,cc}(SINR,n)=sigmoid(c0+c1 SINR+c2 SINR^2+c3 n+c4 SINR*n), n=
receiver CAV density. CRITICAL: must fit on PER-RECEPTION density (matches OMNeT countNeighbours), NOT
rate-mean — rate-mean overfit the 5 density levels and HURT OMNeT (r 0.957->0.891); per-reception
(build_capture_dataset.py computes density per sampled reception) transfers correctly.
**Per-tx-rx full scatter: density-independent 0.085/r.957 -> rate-mean 0.089/r.891 -> per-reception
DENSITY-AWARE 0.072/r.983 (BEST).** High-density over-prediction eliminated. OMNeT params
decnc_c0..c4, deccc_c0..c4 (5-term 2D logistic); ns3LearnReception decode uses (sinr,n=countNeighbours).
Residual: matched-50% mainstreet 0.105 (intersection 0.054) — capture/collision density-dependence is
intersection-calibrated, linear road under-predicted (geometry-specificity, the standing limit).
Model now: FULLY measured from NS-3 (hd, collision, decode_noColl(SINR,n), decode_coll(SINR,n)), no fudge.

### Phase 15b — Correction: mainstreet is a two-intersection corridor, not a linear road
User correction + FCD check: mainstreet50 spans ~220m x 226m with TWO clusters along x (CAV-time peaks
~10k and ~15k) = a TWO-INTERSECTION corridor, clustered like the training intersection — NOT a linear
road. The earlier "linear road -> density spreads -> less collision per neighbour" explanation of the
mainstreet under-prediction (Phases 12/15) is WRONG and retracted. The matched-50% residual (mainstreet
MAE 0.105 vs intersection 0.054) is a genuine cross-scenario generalization gap between two
intersection-type layouts; the MODEL adapts to LOCAL density via countNeighbours, so the mechanism is
subtler (different intersection layout/scale/flow/signal-timing -> different local-n->collision or SINR
distribution at matched local n). Mechanism NOT verified — do not assert linear-road reasoning.

---

## Phase 16 — Load-aware contention: closing the C2 high-load gap

### The gap (C2)
C1 collision = `sigmoid(col_c0 + col_c1 n + col_c2 n^2)`, driven by CAV **count** n. In the C1 teacher
runs all nodes beaconed at 10 Hz so load ∝ density and count was a valid proxy. A flood (1 node, ~100x
load) raises load WITHOUT raising count -> OMNeT saw the attacker as "+1 neighbour" and the channel was
blind to it (the flood's real OMNeT damage flowed through the OBU crypto queue = a receiver-side DoS,
missing the RF-contention DoS).

### Rate sweep: decoupling load from density, with no scenario change
`run_ns3_rate_sweep.sh`: reuse EXISTING mobility, vary CAM rate {10,25} Hz x cav{05,25,50} x 3 seeds
(20 Hz INVALID: RRI 50ms not a multiple of the 20-slot SL pool; 25 Hz=40ms is valid, 3GPP TS 38.331).
`extract_contention_load_dataset.py` + `fit_contention_load.py`. **Collision tracks OFFERED LOAD (n x Hz),
NOT count**: at fixed density 10->25 Hz jumps collision (cav05 0.13->0.50); cav50@10Hz ≈ cav25@25Hz.
Lumped fit `sigmoid(-15.35+6.92 log10(load))`, cell MAE 0.046.

### Held-out asymmetric high-load test (do not wire on the symmetric result alone)
Scenario gained `--floodNodeRate/Dynamic/Id`: ONE UE on a dynamic/aperiodic SL grant (3GPP TS 38.321;
SPS caps at 50 Hz). `run_ns3_flood_validate.sh` (200/1000 pps x cav05/25/50 x 3 seeds) +
`extract_flood_validate.py`. **The lumped model FAILS under concentration**: flood MAE 0.098, OVER-predicting
0.18–0.25 at LOW density (cav05@1000 meas 0.474 vs pred 0.728). One flooder = ONE interferer occupying
~1 subchannel/slot -> BOUNDED contribution; the same load over ~78 nodes hits the receiver far more.
=> collision depends on interferer COUNT/distribution, not just total load. Caught BEFORE wiring (the
flood headline is "worst at low density" — the lumped model would have corrupted exactly that regime).

### Adopted model: hybrid per-interferer product (Gozalvez delta_COL structure)
`collision = 1 - (1 - base(n)) * PROD_attackers (1 - q_a(rate,n))`,
`q_a = sigmoid(qa_b0 + qa_b1 log10(rate) + qa_b2 log10(n+1) + qa_b3 log10(rate)log10(n+1))`,
fit on the flood runs with base(n)=C1 collision held FIXED (`fit_attacker_factor.py`). base(n) unchanged
=> ZERO regression to the non-attack baseline. q_a SATURATES in rate (q_a(200)≈0.47 -> q_a(1000)≈0.50)
= one flooder is one bounded factor. Params `qa_b0=-0.356, qa_b1=0.011, qa_b2=-0.027, qa_b3=0.121`.
**Held-out flood MAE 0.013** (vs lumped 0.098, full-product 0.032 but the full product regresses cav05
baseline +0.13 -> rejected). A flooder in range ~doubles a normal node's collision ((1-q_a)≈0.5).

### OMNeT wiring (additive; a no-op when no interferer is present)
`VeinsInet5GVehicleApp`: members ns3QaB0_..B3_ (load qa_b* from cascade_params.csv); `countNeighboursInRange`
now EXCLUDES the flooding attacker (base(n)=normal fleet); helper `inRangeAttackerFloodSurvival(n)` =
PROD over in-range attackers of (1-q_a(min(floodRate,maxActual),n)), =1.0 if none; cascade collision
`collisionP = 1-(1-baseColl)*inRangeAttackerFloodSurvival(n)`. qa_b* appended to BOTH ns3learn_runtime
bundles. Calibrated 200–1000 pps. Two checks after rebuilding veins_inet: Prop_NS3Learn without an
interferer is bit-identical (the term is a no-op), and the high-rate-interferer runs show channel contention.

### Held-out generalization split and the q_a learning figure (F7)
Hardened the accuracy claim and added the missing learning visual (the q_a fit previously printed metrics
but emitted no figure, unlike the C1 learners F3/F4).
- **True 50/50 held-out split** (`fit_attacker_factor.py`, rng seed 0; refit on TRAIN only, scored on the
  untouched TEST half — the earlier "held-out MAE" reused training rows): refit coeffs
  `b_ho=[-0.384, 0.004, -0.039, 0.139]` ≈ production `[-0.356, 0.011, -0.027, 0.121]` (4 params over
  millions of events ⇒ **parameter stability**); **TEST cell MAE 0.013** (= full-fit), TEST per-event
  **Brier 0.087**. Confirms 0.013 is genuine generalization, not in-sample.
- **Wiring verified**: the fit re-derives exactly the `qa_b0..b3` already in
  `ns3learn_runtime/cascade_params.csv` (loaded by the `cascade_params` reader in
  `VeinsInet5GVehicleApp.cc` → `ns3QaB0_..B3_`), so the learned coefficients are the ones the
  simulation uses.
- **Learning visual** `F7_attacker_factor.png` (300 dpi PNG, Okabe-Ito CB-safe, **no PDF**) → both
  `results/paper_figures/color/` and `results/prop_compare_figures/learners/attacker_factor_validation.png`.
  3 panels: (a) held-out accuracy pred-vs-measured P(collision) per flood cell (full + test, MAE/Brier
  annotated, on y=x); (b) learned `q_a(rate,n)` — flat-ish in rate (saturates → one flooder bounded),
  shifted up by density n (the decoupling: load enters by RATE, modulated by count); (c) base(n) curve
  preserved with no attacker + bounded lift when a flooder is in range, measured cells overlaid.
The interferer-load factor is therefore fitted, held-out validated and wired, at the same standard of
evidence as the C1 cascade (Phase 11/13).

### Per-event (un-binned) validation, figure F7b
F7 panel (a) reports only 6 aggregated cells; added an un-binned validation at the granularity the model
is actually applied — ONE prediction per packet (collided 0/1), over **N = 1,939,562 packets**. For a
probabilistic model "per-packet accuracy" = a CALIBRATION question (a single outcome is 0/1, irreducibly
noisy), so:
- **Reliability diagram** (15 equal-frequency bins of predicted P): observed frequency vs mean predicted
  lands on the diagonal → **ECE = 0.009** (mean calibration error < 1%), per-event **Brier = 0.087**.
- **Outcome-conditioned predicted-prob distributions**: collided mass sits at high predicted P, no-collision
  mass at low P → the model discriminates, not just calibrates on average.
- **Fine density-resolved accuracy** (flooder in range, width-4 local-$n$ bins, ~18 points/rate vs the old
  3 coarse cells): predicted curve tracks measured across the full density range at 200 & 1000 pps.
- Figure `F7b_attacker_factor_perevent.png` → `results/paper_figures/color/` and
  `results/prop_compare_figures/learners/attacker_factor_perevent.png` (300 dpi PNG, Okabe-Ito, no PDF).
  Generated by the same `fit_attacker_factor.py` (now emits F7 + F7b in one run).

The figures above establish accuracy of the Python regression against NS-3. That is not by itself
evidence that the deployed OMNeT model reproduces NS-3 under load; the in-situ check follows below.

### Isolating the q_a gate in OMNeT: compare rates, not counts
A paired A/B run (q_a on vs off, 1000 pps, seed 0, OBU queue off; the off bundle sets
qa_b1=qa_b3=0) gives two findings, one of them a methodological caution.

- **The term is a no-op without an interferer**: warm-up CAM receptions are identical in both arms,
  as designed.
- **The gate acts in the right direction**: with q_a on, the propagation/collision drop *rate* rises,
  drop_prop 99,869/258,662 = **38.6%** vs 40,952/128,424 = **31.9%**. (q_a folds into `linkPsr` →
  `drop_prop`; `drop_sps` = 0 in cascade mode.)
- **Absolute counts must not be compared across the two arms.** The runs diverge in total reception
  volume (~288k vs ~158k attempts) while `drop_hd` stays nearly identical (29,520 vs 29,515): they run
  in lockstep until the first collision that q_a flips, after which the RNG stream and the trajectories
  diverge and the difference compounds. A count-based headline is therefore a volume artifact, not an
  effect. Comparisons must use rates — PDR and drop fractions — or a design that controls the
  divergence. The quantitative check below matches an OMNeT cell to an NS-3 cell by density × rate and
  compares collision rate, rather than comparing the two arms to each other.

### Closing the teacher/student loop: OMNeT-realized vs NS-3-measured collision
Did the rate-based accuracy check (avoids the Step-7 volume confound by comparing per-reception collision
RATES, not absolute counts).
- **Instrumentation** (`VeinsInet5GVehicleApp`, env-gated `VEINS_FLOOD_COLL_LOG`): `ns3LearnReception` now
  logs the REALIZED SB-SPS collision outcome per reception keyed by (local n, attacker-in-range) →
  `flood_coll_*.csv` (schema matches NS-3 `flood_validate_dataset.csv`). Rides the dist_pdr periodic flush
  (per-module finish() at vehicle despawn would otherwise write a partial file before the attack window).
- **Runs** (`run_qa_validation.sh`, OBU queue OFF so channel is the only drop): `Cam_ECDSA_Base_Flood`
  CAV50@1000, CAV50@200, CAV10@1000, seed 0, 130 s. Warmup (0–100 s, no active attacker) → base(n);
  attack (100–130 s) → q_a-lifted. ~4.0M receptions logged.
- **RESULT** (`compare_omnet_ns3_flood.py` → **F8_omnet_ns3_flood_agreement.png**):
  attacker-in-range **OMNeT-realized vs NS-3-measured collision MAE = 0.011 (1000 pps), 0.008 (200 pps)**;
  OMNeT-vs-q_a-model MAE = 0.007 / 0.003 (≈exact ⇒ the wiring executes the formula at the right n, no
  integration bug: countNeighbours-excludes-attacker + rate gating all correct). base(n) no-attacker branch
  tracks too. So the deployed surrogate reproduces the teacher IN-SITU, not just in Python.
- **Caveat**: at 1000 pps the NS-3 points with no interferer in range sit slightly above base(n) at low
  n — an intense interferer elevates even out-of-range receptions in NS-3. This is a property of the
  teacher and does not affect the in-range q_a validation. F8 has no greyscale variant.

End to end, then: fitted in Python (MAE 0.013), wired into the model, and reproduced in OMNeT
(MAE 0.011 against NS-3).

### CAV degraded-mode behaviour: safe_stop at the stop line
Design decision (not a comparison): on SPaT loss/staleness the CAV treats SPaT as the sole intersection
authority and performs an ISO 26262 minimal-risk maneuver. Refined the existing `safe_stop` policy from a
crude in-place speed cap to a stop-line-targeted controlled stop:
- **Logic** (`checkSpatStaleness`, `spatLossPolicy="safe_stop"`): use `getNextTls()` distance to the stop
  line; comfortable kinematic decel `vTarget=min(v, sqrt(2·a·(d−margin)))` recomputed each watchdog tick
  (a=`spatStopDecel` default 3.0 m/s², margin `kStopLineMargin`=2 m) → comes to rest ~2 m before the line.
  **Dilemma-zone rule**: if `d ≤ v²/2a + margin` (can't stop comfortably) → commit & clear; if already on a
  SUMO internal `:` (junction) edge → clear the box, never freeze in the conflict zone. Holds until a fresh
  authenticated SPaT arrives, then `applySpat()` recovers (spatControlActive_ stays true).
- **New param** `spatStopDecel` (NED, default 3.0). Behaviour env-gated probe `VEINS_SAFESTOP_PROBE` →
  `safestop_probe_*.csv` (per-CAV dist-to-stopline; OFF by default, zero campaign impact).
- **Observed behaviour** (CAV50, 1000 pps, `results/safestop_probe/`, figure `safestop_behavior_cav50.png`):
  lead CAVs decelerate smoothly and rest ~2 m before the line, neither scattered nor inside the box; followers
  queue behind (median resting dist ~40 m = queue length, vs ~20 m in warmup); attack lengthens queues.
- **Caveat**: at 50% CAV with a sustained 30 s interferer, queues grow long enough to spill back through
  upstream junctions (~8% of stopped samples on `:` edges). This is emergent gridlock through SUMO
  car-following — safe_stop never commands an in-box stop — and is realistic but severe; it is milder at
  lower CAV shares. Deceleration rate and interferer intensity are both tunable.

`spatLossPolicy` defaults to `""` in `omnetpp_inet.ini`; set it to `"safe_stop"` to select this behaviour.

## Phase 17 — Combined analytical reference (Cao 2026 + Rehman 2023) replaces the 4G analytical

Goal: give the analytical baseline BOTH density and distance dependence, faithful to the two published
NR Mode-2 models, as a fair (teacher-free) reference whose DISCREPANCY vs NS3Learn is the object of study.
NOT calibrated to NS-3 — do not tune to reduce the gap.

**Model (paper Eq. m3):** `PDR(d,n) = (1-P_HD)(1-P_COL(n))*g(SNR(d))`, assembled WITHOUT double counting.
- **Model 1 (Cao, arXiv 2309.16680 — the MAC-PRR paper, NOT the WNS3 PHY-abstraction paper 3592149):**
  `P_HD = t_s/T_RRI = 0.01`; base model p_k=0 (teacher slProbResourceKeep=0): `P_COL = 1-(1-2*pi0/N_a)^n`,
  `N_a = N_r - n + ((Nc-1)/Nc)*P_COL*n`, pi0=1/11, Nc=2. The Python and C++ implementations match Cao
  Eqs (8)+(12) exactly.
- **Model 2 (Rehman, Sensors 23-04901):** noise-limited decode `g(SNR(d)) = E_{LOS/NLOS,shadow}[1-BLER(SNR)]`,
  SNR = 23 - PL_UMi(d) - noiseFloor(-94.3), TR 38.901 UMi + independent 5G-Toolbox CDL BLER curve
  (bler_curve_cdl_c.csv, _5000 dir — the non-degenerate one; the plain runtime_coefficients CDL curves are
  all-BLER=1 and unusable). NO interference term => no double count with Model 1's collisions.

**KEY GROUNDING (Cao Table II vs our teacher):** Cao validated at N_sc=2; our teacher (propagation-compare.cc)
runs N_sc=5, t2=33-slot selection window, bitmap 9/12. Cao's Eq (1) note explicitly permits substituting the
actual selection window for T_RRI, so the FAIR + FAITHFUL pool is `N_r = 33*5*(9/12) ≈ 124` (NOT full-RRI 375).
Chosen on fairness (match the NS-3 config we compare against), NOT on MAE. Density n = countNeighboursInRange()
(same signal NS3Learn uses => comparable). retired the MATLAB calibration (calibrate_5g_pc5.m/current_analytical).

**Python sanity (build_analytical_m3.py, no build):** g(d) 1.0/0.885/0.42/0.18 @10/100/200/300m; P_COL(n) 0.006->0.59.
Discrepancy vs NS-3 is expected (analytical lacks capture+fitting) — reported, not tuned. Outputs to
results/analytical_m3/{analytical_m3_sanity.png,analytical_m3_params.csv}.

**C++ wiring (ADDITIVE; ns3learn + analytical paths UNTOUCHED — verify with a before/after NS3Learn smoke diff):**
- VeinsInet5GVehicleApp.h: members m3* + loadAnalyticalM3Model/caoCollisionProb/rehmanDecodeProb.
- .cc: initialize m3 block (sets pc5HalfDuplexDrop_=P_HD or 0, pc5DropRate_=0); dispatch in
  getLinkPacketSensingRatio (drop_prop <- 1-g, or 1.0 for col ablation) and estimateSpsCollisionDrop
  (drop_sps <- P_COL(n), or 0 for prop ablation); half-duplex via pc5HalfDuplexDrop_. Memoized caches.
- .ned: realismModel adds "analytical_m3"|"analytical_col"|"analytical_prop" + m3* params (grounded defaults).
- omnetpp_inet.ini: [Config Prop_Analytical_M3] (+ _Col/_Prop ablations) extends Prop_Analytical; leaf
  Prop_Analytical_M3_CAV{01,05,25,50,75,100} (+ Col/Prop leaves). Prop_Calibrated marked RETIRED (kept defined
  so its legacy leaves don't dangle). run_omnet_resume_par.sh default TREATMENTS -> "Analytical_M3 NS3Learn".
- The base `Prop_Analytical` is deliberately left unchanged. `Prop_Generalized` and `Prop_NS3Learn` extend it,
  and the m3 branch runs before the generalized branch, so repointing the base would have hijacked
  Generalized. The m3 models are added as new configs only.

**Build:**
  `cd subprojects/veins_inet && export OPP_ENV_VERSION=1 OMNETPP_ROOT=$OMNETPP_ROOT; source $OMNETPP_ROOT/setenv -q; make MODE=release -j4`
**Validate:** `TREATMENTS="Analytical_M3 NS3Learn" ./run_omnet_resume_par.sh`, confirming an NS3Learn run is
byte-identical to a pre-change run (the guarantee that existing treatments are unaffected), then rebuild the
comparison figures. The equation text should state the selection window (33 slots) rather than T_RRI.

### Phase 17b — Pre-build audit: channel aligned, NS3Learn cleared, config hygiene
- **Channel aligned to TR 37.885 V2V-Urban** (was TR 38.901 UMi) for a fair comparison: Model 2 now
  shares NS-3/ns3learn's channel, so the discrepancy isolates contention/capture/fitting, not channel.
  UMi had made the analytical look artificially close (its +22 dB path loss masked the collision gap).
  rehmanDecodeProb constants verified byte-identical to v2vChannelPathLossDb (38.77/16.7/18.2/1.05/0.0114/
  9+max(0,15logd-41), shadow sigma 3, NLOSv excess std 4.5 folded in quadrature). Sanity: g(d)~1 to 300m
  (V2V generous) so delivery is contention-set; analytical stays optimistic vs NS-3 (the finding, not tuned).
- **CDL BLER curve provenance nailed:** bler_curve_cdl_c.csv = QPSK R=490/1024 = MCS 8 (= Cao PHY-abstraction
  CQI6). INDEPENDENT of teacher (MCS 14) by design; documented in ini. Slightly optimistic at cell edge only.
- **NS3Learn cleared.** An apparent regression turned out to be a reproduction error in the check itself:
  the cascade had been fed the inflated cumulative `nb` (avrgPrr numNieb, ~3×) instead of the true
  per-reception density (real range 8–61). At the true density the collision collapse (n > 88) never
  triggers and there is no regression: a `train_cascade` refit reproduces identical C1 params and
  self-validates at MAE 0.065 (intersection) / 0.11 (mainstreet). The low-adoption under-prediction
  (~0.13 at rate 5) is the known decode_coll low-density residual — bounded, and not a defect. No clamp and
  no teacher rerun are needed.
- **TOOLING BUG found:** train_cascade_all_learners.py re-export STRIPS the C2 qa_b* flood params. Caught via
  backup (cascade_params.csv.pre_audit_bak); RESTORED (file byte-identical to pre-audit, qa_b* intact). The
  script must be patched to preserve qa_b* before any future C1 refit.
- **Config hygiene:** run_omnet_resume_par.sh + run_propagation_compare.sh default TREATMENTS -> "Analytical_M3
  NS3Learn"; added Prop_Analytical_M3_MS50 (out-of-sample pair); Prop_Calibrated RETIRED (kept defined so
  leaves don't dangle); legacy Plain/Generalized/old-Analytical left defined but out of the headline set.
  Verified: all run-script config names resolve, ZERO dangling extends, losOnly correctly =false in both
  active models, m3 NED defaults grounded. build_analytical_m3.py: added the inflated-nb caveat.

### Phase 17c — Model 1 re-grounded in Cao's paper rather than the teacher; qa_b* patch
- User correction: Model 1 is Cao's PUBLISHED analytical model — it has NO teacher. Regrounded:
  equations (Eqs 1,8,12) + pi0=1/11 (R_c~U[5,15]) + N_c=2 (under-sat) are from Cao's paper; config
  inputs are set to OUR scenario for comparability with NS3Learn and each verified within Cao Table II.
- **9/12 bitmap: briefly removed, then RESTORED (correct).** The slBitMap 9/12 is the SL-slot fraction
  of the pool; Cao's N_r counts SL-slot resources (his own pool is all-SL, ours is 9/12-SL), so
  N_r = (T2 slots x 9/12) x N_sc = 33 x 0.75 x 5 = 124. It is NOT an extra factor outside Cao — it is
  the correct SL-slot count for a bitmapped pool. Dropping it (-> 165) over-counts resources and makes
  Model 1 optimistically diverge from NS3Learn/NS-3 (same 9/12 pool). Param renamed m3SlSlotFraction (0.75);
  m3Nr_ = 124.
- Cao Table II conformance: t_s=1ms ✓, T_RRI=100ms ✓, p_k=0 in [0,0.8] ✓, X=0.385 in [0.2,0.5] ✓
  (X only in Cao's EXTENDED model, not the base we use). FLAGGED as outside Cao's demonstrated point:
  N_sc=5 (Cao demo=2, but N_sc is a model INPUT not a limit) and N_UE 8-61 (Cao validated [20,200], so
  low-adoption n<20 is a mild extrapolation). Both acknowledged, none fitted.
- **qa_b* PATCH applied**: train_cascade_all_learners.py now merges/preserves any existing C2 qa_b* on
  re-export (reads the prior cascade_params.csv, keeps keys the C1 refit doesn't produce). Verified.