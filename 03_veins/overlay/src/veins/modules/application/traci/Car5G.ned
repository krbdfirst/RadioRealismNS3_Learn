package org.car2x.veins.modules.application.traci;

//
// CV2X 5G PC5 Sidelink Mode 4 vehicle application.
//
// Mode 4 behaviours modelled:
//   1. Air-interface latency    truncnormal(pc5AirDelayMean, pc5AirDelayStd)
//   2. SB-SPS collision model   analytical local CBR/resource pool estimate
//   3. Half-duplex collision    analytical delta_HD = lambda / 1000
//
// 3GPP References:
//   TS 36.213 §14.1.1.4  — SB-SPS resource reselection
//   TR 37.885 Table 6-1  — air-interface latency
//   TR 37.885 §6.1.2.2   — half-duplex collision probability
//
simple Car5G extends TraCIDemo11p
{
    parameters:
        @class(veins::Car5G);

        // ── Air-interface latency ─────────────────────────────
        double pc5AirDelayMean @unit(s) = default(0.002s);
            // Mean PC5 sidelink air-interface latency.
            // Source: 3GPP TR 37.885 Table 6-1, urban scenario.

        double pc5AirDelayStd  @unit(s) = default(0.0005s);
            // Standard deviation of air-interface latency.

        double pc5DropRate             = default(0.00001);
            // Base packet drop probability (channel impairment).
            // 0.00001 = 0.001% → 99.999% reliability target per TR 37.885.

        double pc5Range                = default(1000.0);
            // Effective communication range in metres (informational).
            // Actual range enforced by connectionManager.maxInterfDist.

        // ── SB-SPS analytical collision model ─────────────────
        double sbspsReselectionInterval @unit(s) = default(0.5s);
            // Mean time between resource reselections.
            // Used to estimate the average number of consecutive
            // transmissions tau under SB-SPS.

        double sbspsReselectionSilence  @unit(s) = default(0.1s);
            // Legacy parameter retained for backward compatibility.
            // The analytical collision model does not use a fixed
            // silence window directly.

        double pc5SubchannelsPerSubframe = default(4);
            // S in eq. (28) of the IEEE 2018 analytical model.

        double pc5ResourcesPerPacket = default(1);
            // Number of sidelink resource blocks/subchannels consumed
            // by one transmission. Keep at 1 for a typical single-CAM
            // reservation, or increase it to model wider allocations.

        double pc5CandidateResourceFraction = default(0.2);
            // NC / N, candidate resource fraction after Step 2+3.

        double pc5TxPowerDbm = default(23);
            // Approximate PC5 transmit power used by the PSR model.

        double pc5SensingThresholdDbm = default(-90);
            // Approximate sensing threshold used by the PSR model.

        double pc5ShadowingStdDb = default(3);
            // Shadowing sigma used by the PSR approximation.

        double pc5PathLossExponent = default(2.0);
            // Log-distance pathloss exponent for the PSR approximation.

        double pc5ReferenceLossDb = default(47.86);
            // Free-space pathloss at 1m around 5.9 GHz.

        // ── Half-duplex collision ─────────────────────────────
        double pc5HalfDuplexDrop       = default(-1);
            // If negative, Car5G computes delta_HD dynamically using
            // eq. (7): delta_HD = lambda / 1000.
            // If non-negative, this value overrides the analytical model.
}
