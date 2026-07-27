#pragma once
//
// TLS5G.h
//
// 5G NR PC5 Traffic Light System application.
//
// In PC5 Mode 4, the TLS is a roadside infrastructure node.
// It authenticates incoming BSMs from vehicles via PC5 sidelink
// and can optionally broadcast signal phase data directly
// (in addition to the RSU doing so).
//
// For the baseline 5G scenario this is a stub — SUMO manages
// the actual light timing. This module exists so OMNeT++ can
// instantiate a TLS application module without crashing.
//
//
// FIXES vs original:
//   FIX 1 — Added finish() override declaration
//            Required to record tls_bsm_received scalar on sim end
//   FIX 2 — stat_bsm_rx_ explicitly initialised to 0 (was already 0,
//            kept for clarity and safety across compiler versions)
//

#include "veins/modules/application/traci/TraCIDemoTrafficLightApp.h"

namespace veins {

class TLS5G : public TraCIDemoTrafficLightApp {

public:
    void initialize(int stage) override;
    void finish()              override;   // FIX 1: added — records scalar
    void onBSM(DemoSafetyMessage* bsm) override;
    void handleLowerMsg(cMessage* msg) override;
    void handleMessage(cMessage* msg)  override;

protected:
    // ── Statistics ────────────────────────────────────────────
    long stat_bsm_rx_ = 0;   // total BSMs received via PC5 sidelink
};

} // namespace veins
