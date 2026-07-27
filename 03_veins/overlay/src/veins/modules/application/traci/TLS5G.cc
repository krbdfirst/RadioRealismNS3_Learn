//
// TLS5G.cc
//
// 5G NR PC5 Traffic Light System application.
//
// Role in the simulation:
//   SUMO manages all traffic light phase timing — this module
//   does NOT control the lights. It is a passive receiver that
//   counts incoming BSMs from vehicles (via PC5 sidelink) and
//   logs them. In a full implementation this would feed into
//   adaptive signal control. For this research it is a stub
//   that keeps OMNeT++ happy and counts V2I messages.
//
// FIXES vs original:
//   FIX 1 — initialize() now calls TraCIDemoTrafficLightApp::initialize(stage)
//            Skipping the parent left mobility/TraCI pointers null → c0000005
//   FIX 2 — onBSM() no longer calls delete bsm
//            The framework owns and deletes the message after onBSM() returns
//   FIX 3 — handleLowerMsg() delegates to parent instead of deleting directly
//            Direct delete bypasses OMNeT++ message ownership → double-free
//   FIX 4 — handleMessage() delegates to parent instead of deleting directly
//

#include "TLS5G.h"

using namespace veins;

Define_Module(veins::TLS5G);

// ─────────────────────────────────────────────────────────────────────────────
//  initialize
//  FIX 1: Must call parent initialize() so that mobility, TraCI, and all
//  OMNeT++ base-class pointers are properly set up before any code runs.
// ─────────────────────────────────────────────────────────────────────────────
void TLS5G::initialize(int stage)
{
    TraCIDemoTrafficLightApp::initialize(stage);

    if (stage == 0) {
        stat_bsm_rx_ = 0;
        EV_INFO << "[TLS5G] PC5 NR-V2X TLS application initialised (passive stub).\n";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  finish
// ─────────────────────────────────────────────────────────────────────────────
void TLS5G::finish()
{
    recordScalar("tls_bsm_received", stat_bsm_rx_);
    TraCIDemoTrafficLightApp::finish();
}

// ─────────────────────────────────────────────────────────────────────────────
//  onBSM
//  FIX 2: Do NOT delete bsm — the framework deletes it after this returns.
// ─────────────────────────────────────────────────────────────────────────────
void TLS5G::onBSM(DemoSafetyMessage* bsm)
{
    stat_bsm_rx_++;
    EV_DETAIL << "[TLS5G] BSM received via PC5 (total=" << stat_bsm_rx_ << ")\n";
    // FIX 2: removed delete bsm — framework owns the message
}

// ─────────────────────────────────────────────────────────────────────────────
//  handleLowerMsg
//  FIX 3: Delegate to parent — direct delete bypasses OMNeT++ ownership.
// ─────────────────────────────────────────────────────────────────────────────
void TLS5G::handleLowerMsg(cMessage* msg)
{
    DemoBaseApplLayer::handleLowerMsg(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
//  handleMessage
//  FIX 4: Delegate to parent — direct delete bypasses OMNeT++ ownership.
// ─────────────────────────────────────────────────────────────────────────────
void TLS5G::handleMessage(cMessage* msg)
{
    DemoBaseApplLayer::handleMessage(msg);
}
