package org.car2x.veins.modules.application.traci;

//
// 5GManualV2VCar.ned
//
// Mixed-traffic 5G V2V + V2I vehicle application.
// Single applType for all vehicles in the simulation.
// Runtime behavior is determined by SUMO vType at initialize(stage=1).
//
// veh_human -> manual mode: no V2X, SUMO drives entirely
// cavVehicleType -> CAV mode: V2I (SPaT) + V2V (CAM) over PC5 Mode 4
//
// Authentication model:
//   - first message from a sender/TLS is fully authenticated
//   - successful authentication is cached for authCacheWindow seconds
//   - cacheWindow = 0s means continuous authentication on every message
//

simple FiveGManualV2VCar extends Car5G
{
    parameters:
        @class(veins::FiveGManualV2VCar);

        string manualVehicleType = default("veh_human");
        string cavVehicleType = default("veh_av");
        string authModeLabel = default("BASELINE_5G");
        bool logGroundtruth = default(false);

        double leaderCamMaxAge @unit(s) = default(0.5s);
        double beaconRange = default(320.0);
        bool useBeaconGap = default(true);

        double spatStaleTimeout @unit(s) = default(0.5s);

        double authDelayMean @unit(s) = default(0.0005s);
            // Mean delay for a full authentication check on a newly seen sender.
            // Applies to both V2V CAM and V2I SPaT paths.

        double authDelayStd @unit(s) = default(0.0001s);
            // Standard deviation for the shared authentication delay model.

        double authCacheWindow @unit(s) = default(20s);
            // Cache lifetime after a successful verification.
            // >0s: baseline-style trust caching.
            // 0s : re-authenticate on every message (no caching).

        double authFreshnessWindow @unit(s) = default(-1s);
            // Maximum age a CAM or SPaT may have at decision time.
            // If total age (decisionTime - timeSent) exceeds this window,
            // the message is dropped as stale and does not count as delivered.
            // <=0s disables freshness dropping.
}
