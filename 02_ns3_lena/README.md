# 02_ns3_lena: the ns-3 5G-LENA teacher simulation

The ns-3 core and the 5G-LENA `nr` module are used unmodified. This directory
contains the scenario written for this study, arranged to mirror its position in
an ns-3 tree:

```
scratch/propagation-compare/
├── propagation-compare.cc      the scenario: NR-V2X sidelink, SUMO mobility replay
├── make_cav_ns3_inputs.py      builds per-rate ns-3 inputs from the SUMO FCD traces
├── run_ns3_teacher.sh          teacher-data collection (the distillation input)
├── run_ns3_compare.sh          comparison sweep
├── run_ns3_compare_seeds.sh    multi-seed variant
├── run_ns3_parallel.sh         RAM-throttled parallel driver
├── run_ns3_rate_sweep.sh       adoption-rate sweep
├── run_ns3_umi_confirm.sh      UMi channel control run
├── run_ns3_flood_validate.sh   flooding-load validation
└── README_NS3_MAPPING.md       how ns-3 parameters map onto the OMNeT configuration
```

## Version pin

| | |
|---|---|
| ns-3 | **3.42**, CTTC fork, branch `ns-3-dev-v2x-v1.1-branch` |
| Repository | https://gitlab.com/cttc-lena/ns-3-dev |
| Commit | `c90c13b8310a813cf4eaf67a2c90df497bbd1965` |
| `nr` module | **v2x-1.1**, commit `72ae0c3bc894bc3416dc69832ada6fcaeebdaeaa` |
| Repository | https://gitlab.com/cttc-lena/nr |
| License | GPL-2.0 |

The NR-V2X sidelink support (Mode 2 / SB-SPS) used by this study is provided by
the CTTC `v2x` line; mainline ns-3 does not include it.

### v2x-kpi is not redistributed here

`propagation-compare.cc` includes `v2x-kpi.h` for KPI collection (PRR, latency,
per-link statistics into SQLite). Those two files belong to the `nr` module and
are used unmodified, so they are not copied into this repository. `tools/setup.sh`
takes them from the local `nr` checkout:

```
$NS3_ROOT/contrib/nr/examples/nr-v2x-examples/v2x-kpi.{cc,h}
```

and places them beside the scenario in `$NS3_ROOT/scratch/propagation-compare/`,
where the scratch build expects them. To do it by hand:

```bash
cp "$NS3_ROOT"/contrib/nr/examples/nr-v2x-examples/v2x-kpi.{cc,h} \
   "$NS3_ROOT"/scratch/propagation-compare/
```

## Install

```bash
git clone https://gitlab.com/cttc-lena/ns-3-dev.git
cd ns-3-dev
git checkout ns-3-dev-v2x-v1.1-branch

git clone https://gitlab.com/cttc-lena/nr.git contrib/nr
cd contrib/nr && git checkout v2x-1.1 && cd ../..
```

Then install this scenario and build:

```bash
$RR_ROOT/tools/setup.sh           # copies the scenario into scratch/propagation-compare
./ns3 configure --enable-examples
./ns3 build
```

`setup.sh` also writes a `.rr_root` file into the installed scenario directory so
the run scripts can locate `env.sh`. If the scenario is copied manually, export
`RR_ROOT` before running them.

## The two trace modes

Teacher collection uses two trace levels, since recording every reception at
every load is not tractable:

| Flag | Cost | Use |
|---|---|---|
| `--macSummary` | small (about 90 MB per run) | MAC scheduling, SB-SPS resource collisions, HARQ retransmissions. Runs at every adoption rate. |
| `--phyRxTrace` | very large (tens of GB) | Per-reception SINR and decode outcome, giving the BLER waterfall. Tractable only at low adoption rates. |

`--phyTraces` enables both. Because high-load SINR traces are not collected, the
interference term I(CBR) is obtained by back-fitting against the PRR-vs-density
target, which is measured at all rates from `--macSummary` alone.

## Output

Runs write SQLite databases under `scratch/propagation-compare/results/`, named
by scenario so different channel models do not overwrite each other:

- `results/teacher/`: the fitting input, read by
  `06_analysis/python/extract_teacher_dataset.py`
- `results/teacher_logs/`: per-run logs, plus a `.result` marker line that makes
  collection resumable

Collection is resumable and memory-limited. A run whose `.result` marker exists
is skipped, and new runs start only while free memory stays above `MIN_FREE_GB`.
Both `MAXP` and `MIN_FREE_GB` are environment variables.
