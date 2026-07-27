# RadioRealismNS3_Learn

**Distilling ns-3 5G-LENA NR-V2X realism into a fast analytical PC5 model for OMNeT++ / Veins**

This repository contains the simulation code and analysis pipeline for a study of
5G NR PC5 (V2V sidelink) channel realism in vehicular network simulation.

An analytical PC5 model runs orders of magnitude faster than a full-stack ns-3
5G-LENA simulation. The study measures how much realism that speed costs, and
whether the missing realism can be recovered by fitting surrogate functions to
ns-3 output rather than re-implementing the protocol stack.

The method uses ns-3 5G-LENA as a teacher: it produces ground-truth PC5 behaviour
over a SUMO mobility trace shared with OMNeT++/Veins. Load-conditioned surrogate
functions for BLER(SINR), SB-SPS resource collision, half-duplex loss and
HARQ-inflated channel occupancy are fitted to its traces and plugged into the
analytical model's per-cause loss hooks. The resulting model (`NS3Learn`) is then
compared against a published-analytical reference (`Analytical_M3`) across six
CAV adoption rates.

Every coefficient in `01_omnet_project/model_coefficients/` is fitted from ns-3
output or exported from a MATLAB 5G Toolbox link-level simulation. The scripts
that produced them are included in `06_analysis/`.

---

## Repository layout

```
RadioRealismNS3_Learn/
├── 01_omnet_project/     ini, NED networks, SUMO scenario, run scripts,
│                         fitted coefficients, sample results
├── 02_ns3_lena/          ns-3 scenario (the teacher simulation)
├── 03_veins/             Veins modifications: the PC5 model, as an overlay
├── 04_inet/              INET version pin and install notes
├── 05_omnetpp/           OMNeT++ version pin and build notes
├── 06_analysis/          Python fitting and figure pipeline; MATLAB 5G Toolbox pipeline
├── docs/                 Methodology log and design notes
├── env.sh                Path definitions used by all run scripts
└── tools/setup.sh        Prerequisite check, scenario install, configure
```

No upstream tree is redistributed here. Every simulator is installed from its
own release, and each numbered folder holds only what this study contributed:

- `04_inet`, `05_omnetpp` — version pin and install notes; nothing was modified.
- `02_ns3_lena` — the ns-3 scenario written for this study, copied into an
  existing ns-3 tree by `tools/setup.sh`.
- `03_veins` — the 46 Veins files this study **changed or added** (14 changed,
  32 new), as an overlay copied over a pristine Veins 5.3.1 checkout. The PC5
  model under study is in that overlay. See `03_veins/README.md` for the file
  list and a description of each upstream change.
- `01_omnet_project` — the SUMO networks/routes, NED networks, ini configs and
  fitted coefficients, all authored for this study.

---

## The four software components

| Component | Version pinned | Modified? | Where |
|---|---|---|---|
| **OMNeT++** | 6.0.3 | No | install separately, see `05_omnetpp/README.md` |
| **INET** | 4.5 (`inet-4.5.4-0a1d409733`) | No | install separately, see `04_inet/README.md` |
| **Veins** | 5.3.1 | Yes, 46 files | install separately, then apply `03_veins/overlay/` |
| **ns-3 + 5G-LENA NR** | ns-3.42, `nr` v2x-1.1 | No (scenario only) | install separately, see `02_ns3_lena/README.md` |
| **SUMO** | 1.22.0 | No | install separately |

SUMO 1.22 is required: `intersection.net.xml` uses vehicle classes that older
SUMO releases reject.

---

## Quick start

```bash
git clone <this-repo> RadioRealismNS3_Learn
cd RadioRealismNS3_Learn

# 1. Record the local simulator paths
cat > env.local.sh <<'EOF'
OMNETPP_ROOT=/path/to/omnetpp-6.0.3
INET_ROOT=/path/to/inet4.5
VEINS_ROOT=/path/to/veins-veins-5.3.1
NS3_ROOT=/path/to/ns-3-dev          # the CTTC 5G-LENA fork, see 02_ns3_lena/
SUMO_HOME=/path/to/sumo-1.22.0
EOF

# 2. Check prerequisites, install the ns-3 scenario, apply the Veins
#    overlay, and configure Veins
./tools/setup.sh

# 3. Build (setup.sh prints these with the local paths filled in)
cd "$INET_ROOT"    && make makefiles && make -j8 MODE=release
cd "$VEINS_ROOT"   && make -j8 MODE=release
cd "$VEINS_ROOT/subprojects/veins_inet" && make -j8 MODE=release
cd "$NS3_ROOT"     && ./ns3 configure --enable-examples && ./ns3 build
```

### Configuration

Paths are not edited inside the scripts. `env.sh` defines them and
`env.local.sh` (git-ignored) overrides them. The same variables are read by
`06_analysis/python/rr_paths.py` and `06_analysis/matlab/rr_paths.m`, so one
setting covers shell, Python and MATLAB.

| Variable | Meaning | Default |
|---|---|---|
| `OMNETPP_ROOT` | OMNeT++ 6.0.3 install | sibling of the checkout |
| `INET_ROOT` | INET 4.5 install | sibling of the checkout |
| `NS3_ROOT` | ns-3 5G-LENA install | sibling of the checkout |
| `VEINS_ROOT` | Veins 5.3.1 install (overlay applied here) | a Veins tree containing this checkout, else sibling |
| `SUMO_HOME` | SUMO 1.22 | derived from `sumo` on `PATH` |

Check a configuration without changing anything:

```bash
./tools/setup.sh --check
```

---

## Reproducing the study

The pipeline has four stages. Stages 1 and 2 regenerate the teacher data and the
fitted coefficients. The fitted coefficients are included in the repository, so
reproducing only the OMNeT results can start at stage 3.

### Stage 1: shared mobility

Both simulators use the same vehicle trajectories. SUMO is the common source:
Veins drives it live over TraCI, and ns-3 replays an exported trace.

```bash
cd 01_omnet_project
./run_sumo_fcd_for_ns3.sh                  # all rates, 200 s, seed 0
END=1000 SEED=3 ./run_sumo_fcd_for_ns3.sh  # custom horizon / seed
```

Produces, per adoption rate, an FCD trace (`fcd_cavNN.xml/.csv`) and an NS-2
mobility file for ns-3's `Ns2MobilityHelper`. Cross-tool reproducibility comes
from `*.manager.seed = ${repetition}` in OMNeT: pass the same value as `SEED`
here and ns-3 replays the trajectories that OMNeT repetition saw.

### Stage 2: teacher collection and fitting

```bash
cd "$NS3_ROOT/scratch/propagation-compare"
./run_ns3_teacher.sh                       # macSummary: MAC scheduling + collisions
RATES="01 05 10" PHYRX=1 ./run_ns3_teacher.sh   # phyRxTrace: BLER/SINR (large)

cd 06_analysis/python
python extract_teacher_dataset.py          # ns-3 DBs -> aggregates + BLER waterfall
python fit_ns3_surrogate.py                # fit + export runtime coefficients
python export_ns3learn_runtime.py          # write the coefficient set the model loads
```

The two trace flags differ in cost: `--macSummary` is small and runs at every
adoption rate, while `--phyRxTrace` records every reception and is only tractable
at low rates. The interference penalty I(CBR) is therefore obtained by
back-fitting to the PRR-vs-density target instead of collecting high-load SINR
traces.

### Stage 3: OMNeT comparison sweep

Start the Veins launch daemon in a separate terminal, with SUMO 1.22 on `PATH`:

```bash
export PATH="$SUMO_HOME/bin:$PATH"
"$VEINS_ROOT"/bin/veins_launchd -vv
```

Then run the sweep:

```bash
cd 01_omnet_project
./run_propagation_compare.sh               # headline set, 1 seed
REPS=5 ./run_propagation_compare.sh        # 5 seeds (configs declare repeat=5)
./run_propagation_compare.sh NS3Learn      # one treatment
./run_propagation_compare.sh CAV50         # one rate across treatments
```

Results are written to `01_omnet_project/results/inet/Prop_*`. Runs that already
produced `dist_pdr_*.csv` are skipped, so an interrupted sweep resumes with the
same command.

### Stage 4: figures and analysis

```bash
cd 06_analysis/python
python build_paper_figures.py
python build_prop_compare_figures.py
python build_deviation_analysis.py
```

---

## The treatments

Each treatment is a configuration prefix in `01_omnet_project/omnetpp_inet.ini`,
crossed with six CAV adoption rates (1 / 5 / 25 / 50 / 75 / 100 %). The two main
treatments are `Analytical_M3` and `NS3Learn`.

| Treatment | Config prefix | Network | What decides packet loss |
|---|---|---|---|
| **NS3Learn** | `Prop_NS3Learn_*` | `IdealPc5Scenario` | Cascade fitted to ns-3: BLER(SINR) + resource collision(density) + half-duplex + HARQ, with I(CBR) interference |
| **Analytical_M3** | `Prop_Analytical_M3_*` | `IdealPc5Scenario` | Published analytical reference: Cao SB-SPS collision (density) × Rehman noise-limited decode (distance) |
| *Analytical_Col* | `Prop_Analytical_Col_*` | ” | Ablation: contention only |
| *Analytical_Prop* | `Prop_Analytical_Prop_*` | ” | Ablation: propagation only |
| *Plain* | `Prop_Plain_*` | `InetScalarRadioScenario` | INET 802.11 scalar radio acts as the channel (control) |
| *Analytical*, *Calibrated*, *Generalized* | `Prop_*` | ” | Earlier treatments, retired from the main comparison but still runnable |

The channel is held constant across the comparison. `Analytical_M3` uses 3GPP
TR 37.885 V2V-Urban, the same channel model as ns-3, so the measured discrepancy
reflects contention, capture and fitting behaviour rather than propagation. The
`Analytical_M3` BLER curve is a separate 5G Toolbox CDL-C curve at QPSK
R=490/1024, so the analytical baseline takes no input from the ns-3 ground
truth.

All treatments run the same baseline: no background interferer, no
crypto-authentication delay (`authDelayMean/Std = 0`) and no OBU overload
(`obuCapacityPps = 1e9`), so the only thing differing between treatments is the
channel model.

### Out-of-sample validation

The `*_MS50` configs re-run both main treatments on a second, independent road
network (`mainstreet.net.xml`, 50 % CAV) that the surrogate was not fitted on.
This tests whether the fitted model generalizes beyond its training scenario.

These configs lie outside the treatment-by-rate grid that
`run_propagation_compare.sh` generates, so run them by name:

```bash
cd 01_omnet_project
./run_config.sh Prop_NS3Learn_MS50
./run_config.sh Prop_Analytical_M3_MS50
./run_config.sh                        # no arguments: list every config
```

### Why the transparent channel

For every treatment except `Plain`, the INET radio is configured as a transparent
medium: it delivers every packet within range without PHY-layer drops or SINR
rejection. All PC5 channel behaviour then comes from the application-layer model
in `VeinsInet5GVehicleApp`. This keeps the INET 802.11 PHY out of the comparison,
so the treatments differ only in the PC5 model under test.

The dispatch logic is implemented in
`03_veins/overlay/subprojects/veins_inet/src/veins_inet/VeinsInet5GVehicleApp.cc`.

### A note on beacon phase

SUMO injects vehicles on 0.1 s step boundaries and the CAM interval is also
0.1 s, which could produce synchronized beaconing. It does not occur here:
`scheduleCavStartup()` and `scheduleCam(first=true)` each add a per-vehicle
`uniform(0, actualCamInterval)` offset to the first transmission, so CAM phase is
staggered independently per vehicle and remains staggered for later beacons.

---

## Included data

The repository contains all source and configuration needed to run every stage,
the fitted coefficients the OMNeT model loads at runtime
(`01_omnet_project/model_coefficients/`, about 100 KB), and a small sample of
results (`01_omnet_project/sample_results/`) so the figure scripts can be run
without completing a full sweep first.

The full result set is not included. The complete study generates roughly 18 GB
of OMNeT output, 12 GB of ns-3 databases and 2.9 GB of mobility traces. All of it
is regenerated by the scripts in this repository.

---

## Runtime

A full ns-3 teacher collection takes days and is memory-bound; the run scripts
throttle on available memory (`MIN_FREE_GB`). The OMNeT sweep at `REPS=5` takes
hours. Both are resumable and skip completed runs, and the supervisor scripts
(`run_sweep_supervisor.sh`, `watch_and_resume.sh`) restart them after a stall.

A seed- and traffic-dependent `SIGBUS` (exit 138) can occur during OMNeT module
teardown after a run has completed. The `dist_pdr` file is flushed during the
simulation, so the result is intact; `run_propagation_compare.sh` treats exit 138
as success when that file is present.

---

## Documentation

| File | Contents |
|---|---|
| `docs/NS3_TO_OMNET_DISTILLATION.md` | Methodology log: the source of the realism gap, the mapping from each ns-3 signal to the corresponding OMNeT hook, and the fitting decisions |
| `docs/README_PROPAGATION_COMPARE.md` | Comparison-sweep notes |
| `02_ns3_lena/scratch/propagation-compare/README_NS3_MAPPING.md` | How ns-3 scenario parameters map onto the OMNeT configuration |
| `docs/phy_pipeline_methodology.svg` | Diagram of the PHY fitting pipeline |

---

## Citing

If you use this code, please cite it using the DOI recorded in `CITATION.cff`.

## License

No simulator source is redistributed here; OMNeT++, INET, Veins, ns-3 and SUMO
are installed from their upstream sources under their own terms.

The work in this repository is released under the MIT License (`LICENSE`), with
one exception. The Veins overlay in `03_veins/overlay/` builds inside Veins: it
modifies 14 Veins source files and adds modules that include Veins headers and
link into the Veins libraries. Those files are therefore covered by Veins'
**GPL-2.0-or-later**, and the 14 modified files retain their original upstream
copyright notices and `SPDX-License-Identifier: GPL-2.0-or-later` headers. The
Veins license text ships with Veins itself (`COPYING` in a Veins checkout).

Everything outside `03_veins/overlay/` — the SUMO scenario, ini configs, ns-3
scenario, fitted coefficients, and the MATLAB/Python analysis pipeline — is MIT.
