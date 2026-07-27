#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  ManualVehicle5G.h
//
//  Mixed-traffic module for 5G scenarios — same runtime vType check as
//  ManualVehicle but inherits Car5G instead of TraCIDemo11p.
//
//  CAV path: full Car5G stack (SB-SPS drops, air delay, onPc5Delivery)
//  Manual path: silent discard of all frames, CSV with "veh_human"
//
//  MODIFIED: Added application-layer flood detection members.
//  New members:
//    stat_wsm_rx_            — total WSMs successfully processed
//    stat_wsm_flood_dropped_ — WSMs dropped due to flood rate limit
//    wsm_rx_in_window_       — message count in current sliding window
//    wsm_rx_window_start_    — start time of current sliding window
//    floodDetectedAt_        — sim time of first flood detection (-1 = never)
//    floodActive_            — true while current window exceeds threshold
//
//  INI:  *.node[*].applType              = "ManualVehicle5G"
//        *.node[*].appl.manualVehicleType = "veh_human"
// ─────────────────────────────────────────────────────────────────────────────
#include "Car5G.h"
#include <string>

namespace veins {

class VEINS_API ManualVehicle5G : public Car5G {
public:
    void initialize(int stage) override;
    void finish()              override;

protected:
    // ── Vehicle type ──────────────────────────────────────────────────────────
    bool        manualMode_        = false;
    std::string sumoVehicleType_;
    std::string manualVehicleType_;
    std::string cachedVehicleId5G_;   // separate from Car5G's cachedVehicleId_

    // ── Flood detection state ─────────────────────────────────────────────────
    long   stat_wsm_rx_            = 0;     // total WSMs processed successfully
    long   stat_wsm_flood_dropped_ = 0;     // total WSMs dropped (flood)
    int    wsm_rx_in_window_       = 0;     // messages received in current window
    double wsm_rx_window_start_    = 0.0;   // start of current sliding window (s)
    double floodDetectedAt_        = -1.0;  // sim time of first detection (-1 = never)
    bool   floodActive_            = false; // true while window exceeds threshold

    // ── Overrides ─────────────────────────────────────────────────────────────
    void onWSM(BaseFrame1609_4* frame)         override;
    void onWSA(DemoServiceAdvertisment* wsa)   override;
    void onPc5Delivery(BaseFrame1609_4* frame) override;
    void handlePositionUpdate(cObject* obj)    override;
    void handleSelfMsg(cMessage* msg)          override;
};

} // namespace veins
