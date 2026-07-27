# Sample data

Small excerpts of real output from each stage of the pipeline, so the formats and
the distillation chain can be inspected without rerunning anything.

Everything here is measured output, not synthetic. Each file below records where
it came from and how it was reduced. Nothing is interpolated, smoothed or
reconstructed — the reduction is always either a contiguous time window or an
every-*k*th-row systematic sample.

The full intermediate data is not committed: the ns-3 databases for one adoption
rate alone run to hundreds of megabytes, and the complete teacher set is tens of
gigabytes. Stage numbers below refer to the pipeline table in
[`docs/NS3_TO_OMNET_DISTILLATION.md`](../../docs/NS3_TO_OMNET_DISTILLATION.md).

---

## `sumo_trace/` — stage 1: the shared mobility trace

Both simulators consume this trace, which is what makes the comparison a
like-for-like one. Produced by:

```bash
cd 01_omnet_project
END=60 SEED=0 ./run_sumo_fcd_for_ns3.sh 25
```

SUMO 1.22.0, `intersection_cav25.sumo.cfg`, 60 s horizon, 0.1 s step, seed 0.
The full run wrote 31,260 rows over 135 road users; the study's own runs use
`END=200`.

| File | Contents |
|---|---|
| `fcd_cav25_t55-60.csv` | Floating-car data, the contiguous window t ∈ [55, 60) s — 4,449 rows, 50 timesteps, 94 road users (23 `veh_av`, 63 `veh_human`, 8 `bike_human`), speeds 0.00–20.28 m/s. The busiest window of the run. |
| `mobility_cav25_head.tcl` | First 200 lines of the ns-2 mobility file that ns-3's `Ns2MobilityHelper` replays. Shows the initial `set X_/Y_/Z_` block and the `$ns_ at T "$node_(N) setdest X Y SPEED"` form. |
| `activity_cav25_head.tcl` | First 40 lines of the per-node activation file: node start/stop times, each commented with the SUMO vehicle id it maps to. |

Columns in the FCD CSV: `timestep_time`, `vehicle_id`, `vehicle_x`, `vehicle_y`,
`vehicle_speed`, `vehicle_acceleration`, `vehicle_angle` (degrees, 0 = North,
clockwise), `vehicle_type`, `vehicle_lane`, `vehicle_pos`, `vehicle_slope`. The
trailing `person_*` columns are emitted by SUMO but unused here and are empty.

`vehicle_type` is the CAV/human split: `make_cav_ns3_inputs.py` filters to
`veh_av` for the ns-3 replay, because the fits are defined over CAV-transmitter
density, not total vehicle density. In this excerpt 23 of the 86 cars are
`veh_av` — 27 %, against a nominal 25 %, the sampling spread expected in a 5 s
window of one seed.

`config_cav25.tcl` is not included: `traceExporter.py` writes absolute local
paths into it. It is regenerated with the trace.

---

## `ns3_teacher/` — stages 3 and 4: what the model was fitted to

The teacher runs in two trace modes. `--macSummary` is compact and is collected
at every adoption rate; `--phyRxTrace` records every reception and is only
tractable at low rates. Both are represented here.

### Raw ns-3 output (stage 3)

Systematic samples from the databases the extractors actually read. Sampling
every *k*th row rather than taking a prefix keeps the full span of time, density
and SINR that the fits see.

| File | Source database | Reduction |
|---|---|---|
| `psschRxUePhy_cav25_s0_sample.csv` | `results/teacher_phy/cav25_s0-V2V_Urban-propagation-compare.db` | every 177th row — 4,002 of 708,510 |
| `psschTxUeMac_cav25_s0_sample.csv` | `results/teacher/cav25_s0-V2V_Urban-propagation-compare.db` | every 96th row — 3,012 of 289,236 |
| `avrgPrr_cav25_s0.csv` | `results/teacher/cav25_s0-V2V_Urban-propagation-compare.db` | complete, 96 rows |

`psschRxUePhy` is the per-reception PHY record and the source of the decode
model: one row per attempted PSSCH reception, with `avrgSinr`, `psschTbler`,
`psschCorrupt` and `sci2Corrupt`. The sample spans 1.5–61.0 s with SINR from
−49.8 to 92.6 dB (median 24.9 dB), 26.8 % of receptions PSSCH-corrupt and 11.3 %
SCI2-corrupt.

Two conventions matter when reading these:

- **`avrgSinr` is a linear ratio, not dB.** The extractors convert with
  `sinr_db = 10·log10(avrgSinr)`.
- **`nodeId = rnti − 1`.** This is how a PHY row is matched to a vehicle in the
  FCD trace.

`psschTxUeMac` is the transmit-side MAC record (resource selection, `rv`,
reselection counter) used for the SB-SPS collision and HARQ-occupancy terms.
`avrgPrr` is the built-in per-node PRR over a 300 m window, one row per node.

### Derived from the full teacher set (stage 4b)

| File | Produced by |
|---|---|
| `teacher_run_aggregates.csv` | `extract_teacher_dataset.py` — 45 runs, keyed by `rate` and `seed`: `totalTx`, `collision_frac`, `harq_retx_mult`, `mean_neighbours`, `numUes`, `prr300` |
| `bler_curve_ns3.csv` | `extract_teacher_dataset.py` — the BLER waterfall per `scenario` (`V2V` and `UMi`, 40 SNR points each): `snr_db`, sample count `n`, `pssch_decode`, `sci_decode` |

These two are aggregates over the whole teacher set, not over the single run
sampled above, so they will not reconcile row-for-row with it.

### The join that produces link data (stage 4a)

The per-reception *link* table is not committed, because it is a join of the two
sources above and is regenerated in minutes:

```bash
cd 06_analysis/python
python extract_cascade_dataset.py     # -> decode_dataset.csv, reach_dataset.csv
python extract_contention_dataset.py  # -> collision_dataset.csv
python extract_halfduplex_dataset.py  # -> halfduplex_dataset.csv
```

Each script pairs a stage-3 database with the stage-1 FCD trace: ns-3 supplies
the outcome of every reception, and the trace supplies where both vehicles were
at that moment. The result is one row per reception carrying transmitter–receiver
distance `d`, local CAV density `n`, `sinr_db`, and the labels `sci_ok`,
`pssch_ok`, `decoded`. That table is what the surrogates in stage 5 are fitted
to. Regenerating it needs the stage-1 traces for the matching rates and seeds.

---

## `omnet_dist_pdr/` — stage 7: OMNeT output using the fitted model

PDR against distance for the two headline treatments across six adoption rates,
from `Prop_Analytical_M3_*` and `Prop_NS3Learn_*`. Columns: `dist_bin_m`,
`attempts`, `delivered`, `pdr`, and the per-cause drop counters `drop_hd`,
`drop_prop`, `drop_sps`, `drop_base`, `drop_cap`.

These are the files the comparison figures are built from, so the fitted
coefficients in `../model_coefficients/ns3learn_runtime/` can be checked against
ns-3 without rerunning either simulator.
