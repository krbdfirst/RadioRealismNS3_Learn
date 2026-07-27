package org.car2x.veins.modules.application.traci;

// 5G mixed-traffic vehicle — extends Car5G.
// CAV: full PC5 Mode 4 stack.  Manual: silent discard + CSV logging only.
//
// INI:  *.node[*].applType              = "ManualVehicle5G"
//       *.node[*].appl.manualVehicleType = "veh_human"

simple ManualVehicle5G extends Car5G
{
    parameters:
        @class(veins::ManualVehicle5G);
        string manualVehicleType = default("veh_human");
}
