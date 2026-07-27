# 01_omnet_project

The OMNeT++ working directory. Every run script changes into this folder first,
so all relative paths in `omnetpp_inet.ini` resolve from here.

## Contents

| Path | What it is |
|---|---|
| `omnetpp_inet.ini` | All configurations: treatments crossed with six CAV adoption rates, plus retired legacy configs |
| `IdealPc5Scenario.ned` | Network with a transparent radio medium; the analytical PC5 model acts as the channel |
| `InetScalarRadioScenario.ned` | Network where `Ieee80211ScalarRadio` acts as the channel (the `Plain` control) |
| `intersection.net.xml`, `.poly.xml` | The SUMO road network and buildings |
| `intersection_cavNN.{rou.xml,sumo.cfg,launchd.xml}` | Per-adoption-rate demand (NN = 01/05/10/25/40/50/75/90/100 % CAV) |
| `mainstreet.*`, `mainstreet_cav50.*` | A second, independent road network used for out-of-sample validation by the `*_MS50` configs |
| `config.xml`, `antenna.xml` | INET physical-environment and antenna configuration |
| `model_coefficients/` | Fitted coefficients loaded by the model at runtime |
| `sample_results/` | A small result sample for running the figure scripts before a full sweep |
| `run_*.sh`, `watch_and_resume.sh` | Run drivers (see below) |

## Runtime model coefficients

These are the fitted outputs the C++ model reads at simulation start. They are
included in the repository, so `NS3Learn` and `Analytical_M3` run without first
re-running the fitting pipeline.

| Directory | Consumed by | Produced by |
|---|---|---|
| `model_coefficients/ns3learn_runtime/` | `realismModel = "ns3learn"`: BLER(SINR) for PSSCH and SCI, collision cascade, interference and half-duplex parameters | `06_analysis/python/fit_ns3_surrogate.py`, `export_ns3learn_runtime.py` |
| `model_coefficients/phy_5gtoolbox/` | `realismModel = "matlablearn"` (generalized PHY), and the CDL-C BLER curve used by `analytical_m3` | `06_analysis/matlab/` pipeline |

The ini refers to them by relative path:

```ini
*.node[*].app[0].ns3LearnCoeffDir       = "model_coefficients/ns3learn_runtime"
*.node[*].app[0].generalizedPhyCoeffDir = "model_coefficients/phy_5gtoolbox"
*.node[*].app[0].m3BlerCurveCsv         = "model_coefficients/phy_5gtoolbox/bler_curve_cdl_c.csv"
```

Re-running the fitting scripts overwrites these files in place, which is the
intended way to regenerate them.

## Run scripts

All of them source `../env.sh` and need no editing. Every OMNeT script requires
the Veins launch daemon to be running already, with SUMO 1.22 on `PATH` (see
`../03_veins/README_STUDY.md`).

| Script | Purpose |
|---|---|
| `run_propagation_compare.sh` | Main sweep: both treatments across six rates. Accepts substring filters, e.g. `./run_propagation_compare.sh NS3Learn CAV50` |
| `run_config.sh` | Run a single named config: the ablations, the out-of-sample `*_MS50` runs, or a legacy treatment. These lie outside the treatment-by-rate grid the sweep generates. With no arguments, lists every config in the ini. |
| `run_ns3learn.sh` | The `Prop_NS3Learn_*` treatment only |
| `run_omnet_resume.sh` | Resume an interrupted sweep, serially |
| `run_omnet_resume_par.sh` | Resume in parallel, throttled on free memory |
| `run_sweep_chain.sh`, `run_sweep_supervisor.sh` | Chain and supervise long unattended sweeps |
| `watch_and_resume.sh` | Watchdog that restarts a stalled sweep |
| `run_netside_supervisor.sh` | Network-side analysis runs |
| `run_sumo_fcd_for_ns3.sh` | Export SUMO FCD + NS-2 mobility for ns-3 replay |
| `run_sumo_fcd_seeds.sh` | The same export across multiple seeds |

Common environment variables: `REPS` (repetitions per config), `RATES` (subset of
adoption rates), `SEED`, and `END` (simulation horizon for FCD export).

### Resuming

Every driver treats an existing `dist_pdr_*.csv` as a completed run, so repeating
the same command after an interruption resumes instead of restarting.

### Exit code 138

`run_propagation_compare.sh` accepts exit code 138 (`SIGBUS`) as success when
`dist_pdr_*.csv` exists. This is a seed- and traffic-dependent crash during OMNeT
module teardown, after the run has completed and flushed its results. Without the
output file, exit 138 is still reported as a failure.

## Results layout

```
results/inet/<ConfigName>-<seed>/
├── dist_pdr_<timestamp>.csv        PDR vs distance (the primary metric)
├── vehicle_timeseries_*.csv        per-vehicle time series
├── cav_pair_trace_*.csv            per-link trace
├── <ConfigName>-<seed>.sca/.vec    OMNeT scalars and vectors
└── <ConfigName>-<seed>.out         run log
```

Set by `result-dir = results/inet/${configname}-${runnumber}` in the ini.

`sample_results/omnet_dist_pdr/` holds seed-0 `dist_pdr` files for both main
treatments at all six rates, giving the analysis scripts input before a full
sweep has been run.
