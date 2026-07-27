package org.car2x.veins.modules.application.traci;

simple RSU5G extends SpatRSU
{
    parameters:
        @class(veins::RSU5G);

        double pc5AirDelayMean @unit(s) = default(0.002s);   // 2ms mean air latency
        double pc5AirDelayStd  @unit(s) = default(0.0005s);  // 0.5ms std dev
        double pc5Range                 = default(320.0);     // FIX 6: was 1000.0, corrected to match ini
}
