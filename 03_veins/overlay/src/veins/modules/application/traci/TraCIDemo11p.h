#pragma once

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "SpatMessage.h"
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace veins {

class VEINS_API TraCIDemo11p : public DemoBaseApplLayer {
public:
    using DemoBaseApplLayer::receiveSignal;
    void initialize(int stage) override;
    void finish() override;

protected:
    simtime_t lastDroveAt;
    bool sentMessage;
    int currentSubscribedServiceId;

    // ── Acceleration tracking ──────────────────────────────────────
    double    prevSpeed;
    simtime_t prevTime;

    // ── Single consolidated CSV stream (shared across all vehicles) ─
    // Points at the module-level static ofstream. Never delete.
    std::ofstream* timeseriesFile = nullptr;

    // ── Per-vehicle braking episode state ─────────────────────────
    bool   inBrakingEpisode_     = false;
    double brakeEpisodeDist_     = 0.0;   // Euclidean dist since episode start
    double brakeEpisodeMaxDecel_ = 0.0;   // worst decel seen this episode
    double brakeLastX_           = 0.0;   // XY at previous step
    double brakeLastY_           = 0.0;
    bool   hardBrakeFired_       = false; // fired once per episode
    bool   emergencyBrakeFired_  = false; // fired once per episode

    // ── Per-vehicle collision state ────────────────────────────────
    bool      inCollision_  = false;
    simtime_t stoppedSince_ = -1;  // simtime when speed first < threshold
    bool      actualCollision_ = false;
    bool      tlsViolationLatched_ = false;
    bool      tlsViolationActive_ = false;
    bool      pedCollisionLatched_ = false;
    bool      bikeCollisionLatched_ = false;
    std::string lastRoadId_;
    std::string lastObservedTlsId_;
    char        lastObservedTlsSignal_ = 'U';
    double      lastObservedTlsDistance_ = -1.0;

    // ── SPaT speed control (baseline: instant reaction) ───────────
    char   spatPhase_         = 'U';
    bool   spatSpeedOverride_ = false;
    double spatNormalMaxSpeed_ = 0.0;
    double ttcLookaheadDistance_ = 50.0;

    // ── Lifecycle guard ───────────────────────────────────────────
    // Set false as FIRST action in finish(). Prevents TraCI calls
    // on a vehicle SUMO has already removed.
    bool   isVehicleActive_   = true;

    void applySpat(const SpatPayload& spat);

    // ── Legacy fields (kept for compatibility) ─────────────────────
    bool prevBraking;
    double prevTlsState;
    std::string prevTlsId;

    void onWSA(DemoServiceAdvertisment* wsa) override;
    void onWSM(BaseFrame1609_4* frame) override;
    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
    void receiveSignal(cComponent* source, simsignal_t signalID, bool value, cObject* details) override;

    double getLeaderGap();
    double getLeaderLookaheadDistance() const;
    std::pair<std::string, double> getLeaderInfo() const;
    double computeTTC(double speed, double leaderSpeed, double gap);
    void   initCSVFiles();
    void updateTlsObservation(int& tlsViolationEvent);
    void updateActiveTransportRisk(int& pedCollisionEvent,
                                   int& bikeCollisionEvent,
                                   int& activeTransportCollisionEvent);
};

} // namespace veins
