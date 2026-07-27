package org.car2x.veins.modules.application.traci;

// ─────────────────────────────────────────────────────────────
//  SpatRSU  —  BASELINE RSU
//  Broadcasts SPaT every spatInterval seconds.
//  No authentication delay of any kind.
//  Used in Baseline_SPaT config only.
// ─────────────────────────────────────────────────────────────
simple SpatRSU extends TraCIDemoRSU11p
{
    parameters:
        @class(veins::SpatRSU);

        // Must match <tlLogic id="..."> in your .net.xml
        string monitoredTlsId = default("Junc1");

        // Broadcast interval — 10 Hz is standard for SPaT
        double spatInterval @unit(s) = default(0.1s);
}
