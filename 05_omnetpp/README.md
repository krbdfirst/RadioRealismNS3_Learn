# 05_omnetpp: OMNeT++ simulation kernel

OMNeT++ is used as released; no kernel source was modified. This directory
contains the version pin, install instructions, and the build notes needed to
compile this stack on Apple Silicon.

## Version pin

| | |
|---|---|
| Release | **OMNeT++ 6.0.3** |
| Source | https://omnetpp.org/download/ (also https://github.com/omnetpp/omnetpp) |
| License | Academic Public License (free for academic/non-commercial use) |

OMNeT++ 6.x is required. The model uses the OMNeT++ 6 message compiler (`--msg6`,
set in `../03_veins/overlay/subprojects/veins_inet/src/makefrag`) and OMNeT++ 6 ini
syntax; it will not build under 5.x.

## Install

```bash
tar xzf omnetpp-6.0.3-*.tgz
cd omnetpp-6.0.3
source setenv
./configure
make -j$(getconf _NPROCESSORS_ONLN)
```

Then record the path in `../env.local.sh`:

```bash
OMNETPP_ROOT=/absolute/path/to/omnetpp-6.0.3
```

Every run script sources `$OMNETPP_ROOT/setenv` before starting a simulation, so
OMNeT++ does not need to be on `PATH` in the calling shell.

## Build notes

Two build fixes are placed in the Veins makefrags (`../03_veins/overlay/**/makefrag`)
rather than here, because makefrag is preserved when `opp_makemake` regenerates
the Makefiles:

1. **macOS relinking.** `LDFLAGS += -Wl,-headerpad_max_install_names` reserves
   Mach-O header space so `install_name_tool` can rewrite load commands after
   linking. Without it, relinking fails on Apple Silicon.

2. **Circular link dependency.** Veins core does not use veins_inet symbols, so
   `-lveins_inet` is filtered out of the core library's `LIBS`.

## Running without the IDE

All runs execute headless through `opp_run` (Cmdenv); the OMNeT++ IDE is not
required. The `image-path` setting in `omnetpp_inet.ini` affects Qtenv icons
only, and missing icons have no effect under Cmdenv.

For a GUI run, add the INET icons on the command line:

```bash
opp_run -u Qtenv --image-path="$INET_ROOT/images" -c Prop_NS3Learn_CAV50 omnetpp_inet.ini
```

## opp_env users

The run scripts are compatible with [`opp_env`](https://github.com/omnetpp/opp_env):
they export `OPP_ENV_VERSION=1` and source `$OMNETPP_ROOT/setenv`. A build inside
an opp_env workspace takes the form:

```bash
opp_env run omnetpp-6.0.3 -c 'cd $VEINS_ROOT/subprojects/veins_inet && make -j4 MODE=release'
```
