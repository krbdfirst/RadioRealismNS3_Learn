# 06_analysis: the fitting and figure pipeline

The pipeline is split across two languages:

- `python/` reads ns-3 SQLite output and OMNeT CSVs, fits the surrogate
  functions, exports the runtime coefficients, and builds the figures.
- `matlab/` is the 5G Toolbox link-level pipeline. It fits the generalized PHY
  equations and the CDL BLER curves, and requires MATLAB with the 5G Toolbox. It
  is not needed to reproduce the OMNeT results, since its exported coefficients
  are included in `01_omnet_project/model_coefficients/phy_5gtoolbox/`.

## Paths

Paths are not hard-coded. `python/rr_paths.py` and `matlab/rr_paths.m` read the
same environment variables that `env.sh` sets, so one configuration covers shell,
Python and MATLAB:

```python
from rr_paths import RESULTS_DIR, NS3_RESULTS, PROJECT_DIR
```

```matlab
rrp = rr_paths();   % rrp.resultsRoot, rrp.phyOutDir, rrp.runtimeDir
```

Override with `RR_RESULTS_DIR`, `NS3_ROOT`, `RR_COEFF_DIR` and related variables;
see the Configuration section of the main `README.md`.

## Setup

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r ../requirements.txt
```

## Python pipeline, in order

### 1. Extract from the teacher

| Script | Reads | Writes |
|---|---|---|
| `extract_teacher_dataset.py` | ns-3 `macSummary` + `phyRxTrace` DBs | per-run aggregates (neighbour density, collision fraction, HARQ multiplier, PRR@300) and the BLER(SINR) / SCI(SINR) waterfall |
| `extract_cascade_dataset.py` | teacher DBs | per-link cascade training rows |
| `extract_contention_dataset.py`, `extract_contention_load_dataset.py` | teacher DBs | contention-vs-load training rows |
| `extract_halfduplex_dataset.py` | teacher DBs | half-duplex loss rows |
| `extract_flood_validate.py` | flooding-validation runs | held-out validation set |

The extractors scan whichever databases exist when they run, so they can be used
while collection is still in progress.

### 2. Fit

| Script | Fits |
|---|---|
| `fit_ns3_surrogate.py` | The core surrogate: BLER curve + collision + half-duplex, exported in the model's runtime format |
| `fit_cascade_learners.py`, `train_cascade_all_learners.py`, `build_learner_bakeoff.py` | The loss cascade; the bakeoff script compares candidate learners |
| `fit_contention_learners.py`, `fit_contention_load.py`, `fit_contention_product.py` | Collision probability vs neighbour density / channel load |
| `calibrate_interference.py`, `calibrate_interference_stochastic.py`, `recalibrate_interference_aggregate.py` | The I(CBR) interference penalty, back-fitted to the PRR-vs-density target |
| `fit_attacker_factor.py` | Load factor under the flooding validation scenario |

### 3. Export to the runtime

| Script | Produces |
|---|---|
| `export_ns3learn_runtime.py` | `01_omnet_project/model_coefficients/ns3learn_runtime/`, the coefficients `realismModel="ns3learn"` loads at startup |
| `export_cascade_coeffs.py` | Cascade coefficients, written to the same directory |

### 4. Validate

| Script | Checks |
|---|---|
| `validate_surrogate.py` | Fitted surrogate against held-out teacher runs |
| `compare_perinstant_pdr.py` | Per-instant PDR, OMNeT vs ns-3 |
| `build_deviation_analysis.py` | Where and by how much the fitted model departs from the ns-3 reference |
| `build_ns3_groundtruth_1_75.py` | Ground-truth reference across adoption rates |

### 5. Figures

`build_paper_figures.py` (the main figure set), `build_prop_compare_figures.py`,
`build_pathloss_compare.py`, `build_perlink_charts.py`, `build_per_txrx_scatter.py`,
`build_scatter_50_dense.py`, `build_loss_breakdown.py`, `build_metric_comparison.py`,
`build_focus_comparison.py`, `build_realism_scope_analysis.py`,
`build_intersection_distance_shape.py`, `build_netside_analysis.py`,
`build_ns3_distance_pdr.py`, `build_paper_figure_data.py`, `build_analytical_m3.py`,
`build_capture_cascade.py`, `build_capture_dataset.py`.

### Utility

`make_cav_rate.py` generates a route file for a new CAV adoption rate by rescaling
the `veh_av : veh_human` probability split of an existing one, holding total
demand constant. The intermediate rates (cav10, cav40, cav90) used to densify the
collision-vs-density fit were produced this way, without regenerating mobility.

## MATLAB: the 5G Toolbox PHY pipeline

Run from `matlab/`. Entry points:

| Script | Role |
|---|---|
| `run_full_phy_model_pipeline_5gtoolbox.m` | Runs the full sequence: build the model, then export coefficients |
| `build_generalized_phy_model_5gtoolbox.m` | Link-level sweep → fitted PHY equations (`*_5000.m` is the larger 5000-sample variant; `*_core.m` holds the shared implementation) |
| `export_fitted_phy_model_coeffs_5gtoolbox.m` | Export runtime coefficient CSVs + CDL BLER curves |
| `compare_channel_gain_models_5gtoolbox.m`, `diagnose_and_refit_channel_gain.m`, `refresh_generalized_phy_diagnostics_5gtoolbox.m` | Diagnostics and channel-gain refitting |
| `calibrate_5g_pc5.m` | Earlier log-distance PC5 calibration, used by the retired `Calibrated` treatment |
| `debug_bler_cdl.m`, `debug_bler_chain.m`, `debug_codec_chain.m` | Link-level debugging aids |

By default the export scripts write into
`01_omnet_project/model_coefficients/phy_5gtoolbox/`, overwriting the included
coefficients. Set `RR_COEFF_DIR` to write them elsewhere.
