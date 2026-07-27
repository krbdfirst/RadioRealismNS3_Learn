# 04_inet: INET Framework

The INET Framework is used as released; no INET source file was modified. This
directory contains the version pin and installation instructions only.

All model code for this study is in the Veins overlay (`../03_veins/overlay/`), in
`VeinsInet5GVehicleApp`, which sits above the INET radio stack as an OMNeT++
application module.

## Version pin

| | |
|---|---|
| Release | **INET 4.5** (build string `inet-4.5.4-0a1d409733`) |
| Source | https://github.com/inet-framework/inet |
| License | LGPL-3.0-or-later |

INET 4.5 is required. `IdealPc5Scenario.ned` and
`InetScalarRadioScenario.ned` import module paths
(`inet.physicallayer.wireless.ieee80211.packetlevel.*`,
`inet.environment.common.PhysicalEnvironment`) that were reorganised between 4.4
and 4.5, and the veins_inet build links against INET 4.5 symbols.

## Install

```bash
git clone --branch v4.5.4 https://github.com/inet-framework/inet.git
cd inet
source setenv                       # after sourcing OMNeT++'s own setenv
make makefiles
make -j$(getconf _NPROCESSORS_ONLN) MODE=release
```

Then record the path in `../env.local.sh`:

```bash
INET_ROOT=/absolute/path/to/inet
```

`../tools/setup.sh` checks the version and passes this path to
`veins_inet/configure --with-inet=...`.

## What INET provides in this study

INET supplies the node model and the radio medium beneath the PC5 application
layer. Which parts are actually exercised depends on the treatment:

| Treatment | Network | Role of INET |
|---|---|---|
| **Plain** | `InetScalarRadioScenario` | `Ieee80211ScalarRadio` acts as the channel; path loss, SINR and reception are decided by INET. |
| **Analytical_M3**, **NS3Learn** | `IdealPc5Scenario` | `Ieee80211DimensionalRadioMedium` is configured as a transparent channel: it delivers every packet in range, and all PC5 channel behaviour comes from the application-layer model. |

The transparent-channel configuration keeps the INET 802.11 PHY out of the
comparison, so the compared treatments differ only in the 5G NR PC5 model under
test.
