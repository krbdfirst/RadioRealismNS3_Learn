//
// ManualVehicle.ned
//
// Mixed-traffic vehicle module — handles both manual (veh_human) and
// connected autonomous (veh_av) vehicles from a SINGLE applType.
//
// At runtime, initialize() reads traciVehicle->getTypeId() and sets
// manualMode_=true if the SUMO vType matches manualVehicleType.
//
// INI USAGE (replaces the old *.node[*].applType line):
//
//   For DSRC baseline:
//     *.node[*].applType              = "ManualVehicle"
//     *.node[*].appl.manualVehicleType = "veh_human"
//
//   For 5G baseline — use ManualVehicle5G instead (extends Car5G)
//
// The manualVehicleType parameter is a single string matching the SUMO
// vType id exactly. Multiple types are not currently supported — if you
// add a third type (e.g. "veh_truck") you would need a comma-separated
// check in initialize(), or create a separate subclass.
//

package org.car2x.veins.modules.application.traci;

simple ManualVehicle extends TraCIDemo11p
{
    parameters:
        @class(veins::ManualVehicle);

        // ── Which SUMO vType is treated as manual / non-V2X ──────
        // Vehicles whose SUMO type matches this string will have
        // the V2X stack disabled at runtime.
        // Must match the vType id in your .rou.xml exactly.
        string manualVehicleType = default("veh_human");
}
