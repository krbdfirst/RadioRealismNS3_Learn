# Combined Analytical Reference — parameter record

`analytical_m3_params.csv` records the inputs to the Combined Analytical
Reference (`realismModel = "analytical_m3"`), which multiplies Cao's SB-SPS
collision and half-duplex terms by Rehman's noise-limited decode probability:

```
PDR_M3(d, n) = (1 - P_HD) · (1 - P_COL(n)) · g(SNR(d))
```

Every row carries its provenance. The file is a record, not a runtime input: the
simulation reads these as NED parameters (`m3*` in `VeinsInet5GVehicleApp.ned`),
and the reference implementation used to derive and sanity-check them is
`06_analysis/python/build_analytical_m3.py`.

## Two things to know before using this file

**The BLER curve path is project-internal.** The `bler_curve` row records the
path the curve was read from when the parameters were produced. In this
repository that curve is `../phy_5gtoolbox/bler_curve_cdl_c.csv`, which is what
`omnetpp_inet.ini` points `m3BlerCurveCsv` at. It is a 5G-Toolbox CDL-C curve
generated at QPSK with target code rate 490/1024, and is deliberately
independent of the ns-3 teacher, so the analytical reference borrows nothing
from the ground truth it is scored against.

The teacher transmits at MCS 14, which in the NR modulation and coding table the
`nr` module implements is 16-QAM at code rate 0.54. Comparing spectral
efficiencies, 2 x 0.479 against 4 x 0.54, the curve represents a link carrying
roughly 44 % of what the teacher carries, hence a more robust one. Note that the
curve is specified by modulation and code rate rather than by an MCS index; the
nearest index, MCS 8, is QPSK at 0.59 and is not identical to it.

**`n_r = 375` is the full-RRI value, not the deployed one.** Cao's candidate
resource count can be taken over the reservation period or over the actual
selection window:

| Window | Expression | Value |
|---|---|---|
| Full RRI | `(T_RRI/t_s) · (9/12) · N_sc` = `100 · 0.75 · 5` | 375 |
| T2 selection window (deployed) | `33 · (9/12) · N_sc` = `33 · 0.75 · 5` | 123.75 |

`build_analytical_m3.py` computes both and plots them against each other, but
writes the full-RRI value into this file. The simulation uses the **selection
window**: `m3SelectionWindowSlots = 33`, matching the teacher's `t2 = 33`, giving
`N_r ≈ 124`. Results reported from `Prop_Analytical_M3_*` are therefore produced
with `N_r ≈ 124`, and `N_r` drives `P_COL` directly, so the two are not
interchangeable. The `9/12` factor is the SL-slot fraction of the resource pool
(`slBitMap`); Cao counts SL slots only, and his own pool is all-SL.

**The selection-window form is the standards-consistent one.** 3GPP TS 38.214
§8.1.4 has the UE identify candidate single-slot resources inside the selection
window $[n+T_1,\, n+T_2]$, so the count a transmitter actually chooses among is
set by that window rather than by the reservation period. The teacher inherits
the `nr` module defaults `T1 = 2` and `T2 = 33`, documented there as "the start
of the selection window in physical slots, accounting for physical layer
processing delay" and "the end of the selection window in physical slots". With
15 kHz numerology one slot is 1 ms, so a 33-slot window sits comfortably inside
the 100 ms packet delay budget.

Both simulators therefore already use the correct convention, and no re-run is
required. The full-RRI figure would apply only where the selection window spans
the entire reservation period, which is not this configuration.
