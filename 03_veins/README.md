# 03_veins: Veins modifications

Veins is used as released and is **not** vendored here. This directory contains
only the files this study changed or added, as an overlay that is copied over a
pristine Veins checkout.

46 files: **14 modifications** to upstream files and **32 new files**. Everything
else in Veins is untouched, so it is downloaded rather than redistributed.

## Version pin

| | |
|---|---|
| Release | **Veins 5.3.1** |
| Source | https://github.com/sommer/veins/releases/tag/veins-5.3.1 (also http://veins.car2x.org/) |
| License | GPL-2.0-or-later |

Veins 5.3.1 is required. `VeinsInetApplicationBase` is re-based onto
`cSimpleModule` against the 5.3.1 class layout, and the overlay's `makefrag`
files assume the 5.3.1 build system.

## Install

```bash
curl -LO https://github.com/sommer/veins/archive/refs/tags/veins-5.3.1.tar.gz
tar xzf veins-5.3.1.tar.gz          # -> veins-veins-5.3.1/
```

Record the path in `../env.local.sh`:

```bash
VEINS_ROOT=/absolute/path/to/veins-veins-5.3.1
```

Then apply the overlay and configure — `../tools/setup.sh` does both, after
checking that the checkout really is 5.3.1:

```bash
./tools/setup.sh
```

To apply the overlay by hand instead:

```bash
cp -R 03_veins/overlay/. "$VEINS_ROOT"/
cd "$VEINS_ROOT"                    && ./configure
cd "$VEINS_ROOT/subprojects/veins_inet" && ./configure --with-inet="$INET_ROOT"
```

Copying is deliberate: the overlay ships **complete files**, not context
patches, so it applies cleanly and cannot half-fail.

The same property makes the target matter. Applying the overlay replaces its 46
files outright, so a Veins tree carrying unrelated local modifications to any of
them will have that work overwritten. `tools/setup.sh` copies every differing
file into a timestamped backup directory under `$VEINS_ROOT` before writing and
prints the restore command, but the cleaner arrangement is to point `VEINS_ROOT`
at a checkout reserved for this study. The generated
`Makefile`s and `bin/veins_run` wrappers are not included — `configure` writes
absolute paths into them, so they must be produced locally.

## What the overlay adds

The 5G NR PC5 (C-V2X Mode 2) model. `VeinsInet5GVehicleApp` is the substance of
the study; it sits above the INET radio stack as an OMNeT++ application module.

| Path (under `overlay/`) | Role |
|---|---|
| `subprojects/veins_inet/src/veins_inet/VeinsInet5GVehicleApp.{cc,h,ned}` | **The PC5 model**: propagation, BLER, SB-SPS collision, half-duplex, HARQ, CBR interference, and the `realismModel` selector |
| `subprojects/veins_inet/src/veins_inet/VeinsInet5GRsuApp.{cc,h,ned}` | RSU counterpart (SPaT broadcast / response) |
| `subprojects/veins_inet/src/veins_inet/VeinsInet5GMessage.msg` | PC5 message definitions |
| `subprojects/veins_inet/src/veins_inet/VeinsInetLiteCar.ned` | Lightweight INET node used by the *Plain* control treatment |
| `src/veins/modules/application/traci/Car5G.*`, `5GManualV2VCar.*`, `ManualVehicle*.*`, `RSU5G.*`, `SpatRSU.*`, `TLS5G.*` | Veins-native (DSRC/802.11p) PC5 and SPaT apps, and the mixed human/CAV traffic apps |
| `src/veins/modules/application/traci/BeaconMessage.h`, `SpatMessage.h`, `RunMetadata.h` | CAM / SPaT payload encoding and per-run result metadata |

### The `realismModel` selector

The fitted model was added as an additional selectable option; the existing
methods are unchanged, so earlier results stay reproducible.

| `realismModel` | Loss model |
|---|---|
| `analytical` | 3GPP TR 38.901 UMi sensitivity gate |
| `calibrated` | Calibrated log-distance sensitivity gate |
| `matlablearn` | 5G Toolbox generalized PHY (BLER from CDL curves) |
| `analytical_m3` | Cao SB-SPS collision × Rehman noise-limited decode |
| `ns3learn` | **ns-3-distilled**: BLER(SINR) + collision(density) + half-duplex + HARQ + I(CBR) |

Dispatch order in `getLinkPacketSensingRatio()` is: generalized, trace,
`tr38901_*`, then calibrated log-distance. The application-layer channel applies
only when `bypassAnalyticalChannel = false`.

## What the overlay changes in upstream Veins

Full diffs are in `patches/veins-5.3.1-modified-files.diff` (review copy — the
overlay itself is what gets installed).

| File | Change | Why |
|---|---|---|
| `subprojects/veins_inet/src/veins_inet/VeinsInetApplicationBase.{cc,h}` | Re-based from `inet::ApplicationBase` to `omnetpp::cSimpleModule` | `ApplicationBase`'s lifecycle expects operational-state transitions the PC5 app does not use; inheriting `cSimpleModule` directly removes that coupling |
| `src/veins/modules/application/traci/TraCIDemo11p.{cc,h,ned}` | Extended with SPaT/CAM handling, PDR and TTC/gap logging; adds `ttcLookaheadDistance` | Base class for the Veins-native apps; provides the shared CSV result path |
| `src/veins/modules/application/ieee80211p/DemoBaseApplLayer.cc` | `ASSERT(annotations)` / `ASSERT(mac)` downgraded to warnings; stage-1 MAC setup skipped when absent | Lets application modules run on nodes without an AnnotationManager or an 802.11p MAC (the INET-based nodes) |
| `src/veins/modules/application/ieee80211p/DemoBaseApplLayer.h` | Pointer and counter members given in-class initialisers | Members were left uninitialised; a node without a MAC would otherwise read indeterminate values |
| `src/veins/modules/mobility/traci/TraCICommandInterface.{cc,h}` | Added `Vehicle::setType()` | Needed to switch a vehicle's SUMO type at runtime for the CACC/ACC fail-safe handover |
| `src/veins/modules/application/traci/TraCIDemoTrafficLightApp.cc` | `initialize()` now calls `DemoBaseApplLayer::initialize(stage)` | Upstream's override is empty, so base-class initialisation never ran |
| `subprojects/veins_inet/src/veins_inet/VeinsInetCar.ned` | `hasTcp`/`hasSctp` off, `numLoInterfaces = 0` | Only UDP multicast is used; dropping unused transports cuts host-construction cost at high vehicle counts |
| `subprojects/veins_inet/examples/veins_inet/Scenario.ned` | INET 4.5 import paths pinned (version conditionals removed) | The `//#if INET_VERSION` blocks resolve to the pre-4.3 paths on this toolchain |
| `src/makefrag` | macOS `-Wl,-headerpad_max_install_names`; filters `-lveins_inet` out of core `LIBS` | Relinking fails on Apple Silicon without header pad; the core→veins_inet link dependency is circular |
| `subprojects/veins_inet/src/makefrag` | macOS header pad; corrects an `$(INET_PROJ)/src/debug/src` include path | Same relink issue; `opp_makemake` can emit the bad include path |

`makefrag` is used for the build fixes because `opp_makemake` includes it last
and preserves it when regenerating the Makefiles.

## The launch daemon

Veins drives SUMO over TraCI through a launch daemon that must be running before
any OMNeT sweep, in its own terminal, with SUMO 1.22 on `PATH`. It ships with
upstream Veins (unmodified):

```bash
export PATH="$SUMO_HOME/bin:$PATH"
"$VEINS_ROOT"/bin/veins_launchd -vv
```

## Verifying the overlay

To confirm an installed tree matches this repository, and to see whether
anything else was changed locally:

```bash
cd 03_veins/overlay
find . -type f | while read -r f; do
    cmp -s "$f" "$VEINS_ROOT/$f" || echo "DIFFERS: $f"
done
```
