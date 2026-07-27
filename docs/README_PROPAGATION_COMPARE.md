# Propagation-Realism Comparison (vs NS-3 5G LENA)

Cross-validates OMNeT/Veins 5G NR PC5 propagation treatments against
**NS-3 5G LENA**, using a clean **baseline** (no background interferer, no
crypto-auth delay, no OBU overload) across CAV adoption rates
**1 / 5 / 25 / 50 / 75 / 100 %**.

## The four channel treatments

| Treatment | ini config prefix | network | `bypassAnalyticalChannel` | `activateGeneralizedPhyModel` | `v2vPropagationModel` |
|---|---|---|---|---|---|
| **Plain** (OMNeT+INET+Veins) | `Prop_Plain_*` | InetScalarRadioScenario | `true` | `false` | radio is the channel (Ieee80211ScalarRadio) |
| **Style 1 – Analytical** (3GPP TR 38.901 UMi, "previous research") | `Prop_Analytical_*` | IdealPc5Scenario | `false` | `false` | `tr38901_umi` |
| **Style 2 – Calibrated** (MATLAB params, `calibrate_5g_pc5.m`) | `Prop_Calibrated_*` | IdealPc5Scenario | `false` | `false` | `current_analytical` |
| **Style 3 – Generalized** (5G Toolbox fitted equations) | `Prop_Generalized_*` | IdealPc5Scenario | `false` | `true` | (any) |

Code source of truth: `subprojects/veins_inet/src/veins_inet/VeinsInet5GVehicleApp.cc`
- `~line 1834`: app-layer channel only applied when `bypassAnalyticalChannel=false`.
- `getLinkPacketSensingRatio()` dispatch order: **generalized → trace → tr38901_* → calibrated log-distance**.

## Synchronized-beaconing check (the other project's bug)

**Not present here.** `scheduleCavStartup()` (line ~1538) and `scheduleCam(first=true)`
(line ~1590) both add a per-vehicle `uniform(0, actualCamInterval)` offset to the
*first* transmission. So although SUMO injects vehicles on 0.1 s step boundaries and
the CAM interval is 0.1 s, each vehicle's CAM phase is randomly staggered in
[0.1, 0.2] s and stays staggered for all later beacons. No global 10 Hz lock-step.

## 1. NS-3 mobility ground truth (plain SUMO — priority)

```bash
./run_sumo_fcd_for_ns3.sh                  # all rates, 200 s, seed 0
END=1000 SEED=3 ./run_sumo_fcd_for_ns3.sh  # custom horizon/seed
./run_sumo_fcd_for_ns3.sh 25 100           # subset of rates
```
Per rate, under `ns3_mobility/cavNN/`:
- `fcd_cavNN.xml` / `fcd_cavNN.csv` — fields: `timestep_time, vehicle_id,
  vehicle_x, vehicle_y, vehicle_speed, vehicle_acceleration, vehicle_angle`
  (heading, deg, 0=N, CW), `vehicle_type, vehicle_lane, vehicle_slope`.
- `mobility_cavNN.tcl` — NS-2 mobility for ns-3 `Ns2MobilityHelper` (direct replay).

Filter `vehicle_type` to `veh_av` (CAV) / `veh_human` (manual); drop
`bike_human` / `ped_human` if NS-3 should model cars only.

**Requires SUMO 1.22** (`$SUMO_HOME`); the net uses vClasses
unknown to 1.11.0. The script defaults to it.

## 2. OMNeT/Veins comparison runs

Start the launch daemon in a separate terminal **with SUMO 1.22 on PATH**:
```bash
export PATH=$SUMO_HOME/bin:$PATH
$VEINS_ROOT/bin/veins_launchd -vv
```
Then:
```bash
./run_propagation_compare.sh                 # all 24, 1 seed
REPS=5 ./run_propagation_compare.sh          # 5 seeds (matches repeat=5)
./run_propagation_compare.sh Calibrated      # one treatment
./run_propagation_compare.sh CAV50           # one rate across treatments
```
Results: `results/inet/Prop_*`.

## Cross-tool reproducibility
`*.manager.seed = ${repetition}` in OMNeT; pass the same value as `SEED` to the FCD
script so NS-3 replays the identical trajectories that a given OMNeT repetition saw.

## Notes
- `authDelayMean/Std = 0` and `obuCapacityPps = 1e9` in the baseline so the only
  thing differing between treatments is the propagation model (fair channel/MAC
  comparison vs LENA, which has no app-layer crypto).
