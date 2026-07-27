# NS-3 5G-LENA replay of the OMNeT/Veins PC5 baseline

This `scratch/propagation-compare` project replays the **same SUMO vehicle
trajectories** used by the OMNeT `PropagationRealismProject` baseline in an
**NR V2X sidelink (Mode 2, out-of-coverage)** stack from 5G-LENA, so the PC5
communication behavior of the three OMNeT propagation treatments
(Plain / Analytical / Calibrated / Generalized) can be compared against a
full 3GPP NR reference.

## Files
| File | Purpose |
|---|---|
| `propagation-compare.cc` | Main NR sidelink program (trace-driven, all-CAV TX+RX). |
| `v2x-kpi.{h,cc}` | KPI helper copied from `contrib/nr/examples/nr-v2x-examples` (PRR/PIR/throughput). |
| `make_cav_ns3_inputs.py` | FCD CSV → CAV-only ns-2 mobility (`*_mobility.tcl`) + per-CAV lifetime (`*_meta.csv`) + `*_info.txt`. |
| `run_ns3_compare.sh` | Build inputs + run all rates + collect `results/summary_<scenario>.csv`. |

## Pipeline
```
PropagationRealismProject/run_sumo_fcd_for_ns3.sh   # SUMO FCD per rate (already produced)
        │  ns3_mobility/cavNN/fcd_cavNN.csv
        ▼
make_cav_ns3_inputs.py   # keep veh_av only, antenna z=1.5 m, stable node ids
        ▼
./ns3 run propagation-compare/propagation-compare --tag=cavNN ...
        ▼
results/cavNN-<scenario>-propagation-compare.db   (+ RESULTCSV summary line)
```
Quick start:
```
cd $NS3_ROOT
SIMTIME=60 ./scratch/propagation-compare/run_ns3_compare.sh 05 25
```

## Why CAV-only nodes
OMNeT instantiates a network stack **only** for `veh_av` (CAVs); `veh_human`
run as pure-SUMO `VeinsInetLiteCar` with no radio, and the channel uses
`IdealObstacleLoss` (vehicles do not block links). So the NS-3 model uses one
NR UE **per CAV** and excludes human vehicles entirely — same transmitter set,
no human-vehicle blockage — which matches OMNeT. Each CAV is both a 10 Hz CAM
**transmitter and receiver** (groupcast), gated to its SUMO depart→arrive
lifetime (mirroring the OMNeT app lifecycle).

## Parameter mapping (OMNeT → NS-3)

| Quantity | OMNeT / Veins (`omnetpp_inet.ini`, app) | NS-3 5G-LENA | Notes |
|---|---|---|---|
| Carrier frequency | `5.9 GHz` band / `spatCarrierFrequencyGHz=5.9` | `centralFrequencyBandSl = 5.9e9` | exact |
| Channel bandwidth | `radio.bandwidth = 10 MHz` | `bandwidthBandSl = 100` (×100 kHz) | exact 10 MHz |
| Numerology / SCS | (not modeled; analytical) | `numerologyBwpSl = 0` (15 kHz SCS) | standard NR-V2X @10 MHz (~52 PRB) |
| TX power | `pc5TxPowerDbm = 23` | `txPower = 23 dBm` | exact (PC5 23 dBm) |
| Noise figure | `pc5NoiseFigureDb = 7` | `NrUePhy NoiseFigure = 7 dB` | exact |
| CAM size | `VeinsInet5GMessage` chunk `B(256)` | `packetSizeBe = 256` | exact 256 B |
| CAM rate | `beaconInterval = 0.1 s` (10 Hz) | `dataRateBe = 20.48 kb/s` ⇒ 256 B @ 10 Hz; `RRI = 100 ms` | exact 10 Hz |
| First-beacon de-sync | `scheduleCam(first)`: `uniform(0,0.1 s)` | per-UE app-start jitter `Uniform(0,0.1)` | exact mechanism |
| App lifetime | app runs while vehicle bound (TraCI add→remove) | client/sink start/stop = SUMO depart/arrive (`*_meta.csv`) | exact |
| Antenna height | `spatUtHeight = 1.5 m` / radio `offsetZ = 1.5 m` | mobility `z = 1.5 m` | exact |
| V2V range (KPI) | `pc5Range = 300 m` | `kpiRange = 300 m` | exact (KPI window) |
| **Propagation — Style 1 Analytical** | `v2vPropagationModel = "tr38901_umi"` (3GPP TR 38.901 UMi) | `channelScenario = UMi_StreetCanyon` | **direct analog** of the OMNeT analytical model |
| **Propagation — V2V-native** | (intersection, urban) | `channelScenario = V2V_Urban` (3GPP TR 37.885) | NR-native V2V urban model; default; adds NLOSv vehicle blockage |
| Shadowing | calibrated `pc5ShadowingStdDb = 3.616` / TR 38.901 4–7.82 dB | `ShadowingEnabled = true` (model's own σ) | enabled; σ fixed by the 3GPP model, not separately settable |
| Resource selection | SPS w/ sensing (`pc5Sub*`, `sbspsReselectionInterval`) | Mode-2 SPS, `EnableSensing = true`, `RRI = 100 ms` | qualitatively equivalent; NR uses standardized SPS |
| Sensing threshold | `pc5SensingThresholdDbm = -94` | `SlThresPsschRsrp = -110 dBm` | **not 1:1** — OMNeT threshold is a sensing-power gate; NR uses a PSSCH-RSRP exclusion threshold. Exposed via `--slThresPsschRsrp`. |
| HARQ / retx | not modeled (analytical PDR) | `harqEnabled = true`, `slMaxTxTransNumPssch = 5` | NR adds blind retx (lower latency tail) |
| Error model / MCS | analytical PDR vs SINR | `NrEesmIrT1`, fixed `MCS = 14` | NR uses link-level BLER curves |

### Parameters intentionally neutralized for a fair channel comparison
The OMNeT baseline (`Prop_Common_Base`) already zeroes app-layer crypto
(`authDelayMean/Std = 0`), disables the background interferer, and removes OBU overload
(`obuCapacityPps = 1e9`). NS-3 LENA has no app-layer crypto or OBU queue either,
so both sides isolate **channel + MAC** behavior. The comparison metrics are
therefore PDR (PRR) and packet latency vs. distance and vs. adoption rate.

## Where exact equivalence is impossible (be explicit in the paper)
1. **MAC/PHY paradigm.** OMNeT uses an analytical/statistical PC5 PDR layer over
   a transparent INET radio; NS-3 runs a full NR Mode-2 PSCCH/PSSCH stack with
   sensing, HARQ, and BLER curves. Absolute latency will differ (NR ~5–20 ms incl.
   retx vs OMNeT's `pc5AirDelayMean ≈ 2 ms`); the **trends** vs adoption/distance
   are the comparable quantity.
2. **Channel model family.** OMNeT Style 1 = TR 38.901 UMi; NS-3 `UMi_StreetCanyon`
   is the same family (use it for the closest Style-1 match). `V2V_Urban` (TR 37.885)
   is the NR-native V2V model and adds vehicle-blockage (NLOSv) not present in the
   OMNeT UMi formula. Run both `--channelScenario` values to bracket the result.
3. **Sensing threshold semantics** differ (see table) — treat as a tuned, not
   transferred, parameter.
4. **Sub-channelization.** OMNeT abstracts resources via `pc5SubchannelsPerSubframe`;
   NS-3 uses real RB subchannels (`--slSubchannelSize`, default 10 RB ⇒ ~5 subchannels
   at 10 MHz). This governs collision behavior at high adoption.

## KPIs produced (per run, in the sqlite DB)
- `avrgPrr` — per-TX Packet Reception Ratio within the KPI range (300 m).
- `thput` — per-link `totalPktTxed` / `totalPktRxed` ⇒ **per-link PDR**.
- `avrgPir` — per-link avg Packet-Inter-Reception + `TxRxDistance` ⇒ **PDR/PIR vs distance**.
- stdout `RESULTCSV,<tag>,<scenario>,<numUes>,<activeTx>,<txPkt>,<rxPkt>,<avgLatency_ms>`
  (collected into `results/summary_<scenario>.csv`).

> Note: `v2x-kpi` range-PRR uses an **initial-position snapshot** per UE (helper
> limitation). For mobility-exact PDR-vs-distance, use the `thput`/`avrgPir`
> per-link tables (TX/RX counts are exact; distances there are also snapshot-based,
> so bin coarsely). The stdout `avgLatency_ms` is computed from per-packet
> timestamps and is mobility-exact.

## Validated
Built on ns-3.42 + nr (5G-LENA). Smoke test `cav05`, `simTime=60 s`, `V2V_Urban`:
8 concurrent CAV UEs, 2099 CAMs TX, 5613 RX (groupcast), avg latency 12.3 ms,
per-TX PRR ≈ 0.29–0.52 @ 300 m. Raise `SIMTIME` toward 200 and run higher rates
for production (cav100 ≈ 443 UEs is heavy — expect long runtimes).
