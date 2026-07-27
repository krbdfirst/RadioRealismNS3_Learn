#pragma once
// ═══════════════════════════════════════════════════════════════════════════
//  5GManualV2VCar.h  —  Mixed-traffic 5G V2V+V2I vehicle
//
//  SPEED CONTROL ARCHITECTURE
//  ───────────────────────────
    //  CAV vehicles (cavVehicleType) have SUMO TLS enforcement DISABLED at spawn.
//  This is done via traciVehicle->setSpeedMode(0b10100) which clears
//  bit 2 (TLS enforcement) while keeping collision avoidance and max speed.
//
//  Speed is then controlled ONLY by SPaT messages received over the network:
//
//    RED / YELLOW SPaT received:
//      → traciVehicle->setMaxSpeed(0.0)   — vehicle decelerates to stop
//        (IDM handles deceleration profile naturally)
//
//    GREEN SPaT received:
//      → traciVehicle->setMaxSpeed(normalMaxSpeed_) — restriction lifted
//        IDM then accelerates back to free-flow speed naturally
//
//    SPaT stale (no fresh SPaT for > spatStaleTimeout_):
//      → Freeze current speed: traciVehicle->setMaxSpeed(currentSpeed_)
//        If already stopped: traciVehicle->setMaxSpeed(0.0) — stay stopped
//        This models a real CAV that lost comms — it does not revert to SUMO
//        TLS, it just holds what it was doing when comms were lost.
//
//    No SPaT ever received (vehicle just spawned near intersection):
//      → normalMaxSpeed_ preserved — vehicle approaches at normal speed
//        until first SPaT arrives.
//
//  This means:
//    - Flooding that drops all SPaT → CAVs freeze at last known speed
//    - auth delay → vehicle travels extra distance before stopping
//    - Both effects appear in vehicle_timeseries.csv as real speed changes
//    - SUMO does NOT compensate — what the network does is what happens
//
//  Manual vehicles (manualVehicleType) are NOT affected — speed mode unchanged,
//  SUMO TLS drives them completely (correct real-world model).
// ═══════════════════════════════════════════════════════════════════════════

#include "Car5G.h"
#include "BeaconMessage.h"
#include "SpatMessage.h"
#include <fstream>
#include <map>
#include <string>

namespace veins {

class VEINS_API FiveGManualV2VCar : public Car5G {

public:
    void initialize(int stage) override;
    void finish()              override;

protected:
    // ════════════════════════════════════════════════════════════
    //  Manual / CAV mode
    // ════════════════════════════════════════════════════════════
    bool        manualMode_        = false;
    std::string sumoVehicleType_;
    std::string manualVehicleType_;
    std::string cavVehicleType_;
    std::string authModeLabel_;
    std::string cachedVehicleId_;
    bool logGroundtruth_ = false;
    double cachedMyZ_ = 0.0;
    double cachedMyVelX_ = 0.0;
    double cachedMyVelY_ = 0.0;
    double cachedMyVelZ_ = 0.0;
    long long nextMessageId_ = 1;

    // ════════════════════════════════════════════════════════════
    //  SPaT speed control state
    // ════════════════════════════════════════════════════════════
    double normalMaxSpeed_      = -1.0;  // vehicle's normal max speed (m/s)
                                         // captured at first position update
    double lastSpatReceivedAt_  = -1.0;  // simTime of last received SPaT
    char   lastSpatPhase_       = 'U';   // last known phase (G/R/Y/U)
    double spatStaleTimeout_    = 0.5;   // seconds before SPaT considered stale
    bool   spatControlActive_   = false; // true once first SPaT received
    double frozenSpeed_         = -1.0;  // speed frozen when SPaT went stale

    // ── SUMO speed mode bitmask ───────────────────────────────
    // Default SUMO speed mode: 0b11111 = 31 (all checks on)
    // CAV mode:                0b10100 = 20
    //   bit 0 (1): safe speed check                → OFF (CAV controls itself)
    //   bit 1 (2): max acceleration check           → OFF
    //   bit 2 (4): TLS speed enforcement            → OFF ← KEY
    //   bit 3 (8): safe following speed (Krauss)   → OFF
    //   bit 4 (16): slow down for stopped cars      → ON  (keep collision guard)
    // Reference: SUMO docs §TraCI/Change_Vehicle_State setSpeedMode
    static constexpr int CAV_SPEED_MODE = 0b10100;  // = 20

    // ════════════════════════════════════════════════════════════
    //  PC5 protocol stack hooks
    // ════════════════════════════════════════════════════════════
    void onPc5Delivery(BaseFrame1609_4* frame) override;
    void onWSM(BaseFrame1609_4* frame)         override;
    void onWSA(DemoServiceAdvertisment* wsa)   override;
    void handleSelfMsg(cMessage* msg)          override;
    void handlePositionUpdate(cObject* obj)    override;

    // ════════════════════════════════════════════════════════════
    //  SPaT speed control
    // ════════════════════════════════════════════════════════════
    SpatPayload resolveSpatForThisVehicle(const SpatPayload& spat) const;
    void applySpatSpeedControl(const SpatPayload& spat);
    void setSafeMaxSpeed(double speed);   // validated single path for all setMaxSpeed calls
    void checkSpatStaleness();   // called every position update
    double sampleAuthDelay();
    bool   isMessageStale(double timeSent, double decisionTime) const;
    bool   hasValidAuthCache(const std::string& cacheKey) const;
    void   updateAuthCache(const std::string& cacheKey, double timeActed);
    std::string camAuthCacheKey(const std::string& senderId) const;
    std::string spatAuthCacheKey(const std::string& tlsId) const;

    // ════════════════════════════════════════════════════════════
    //  V2V TX / RX
    // ════════════════════════════════════════════════════════════
    virtual void broadcastCam();
    virtual void applyBeacon(const BeaconPayload& cam,
                              double timeReceived,
                              double timeActed,
                              const std::string& reason = "BASELINE_5G_V2V");
    virtual bool isSchemaAttacker() const;
    virtual std::string camFreshAuthReason() const;
    virtual std::string camCachedAuthReason() const;
    virtual std::string spatFreshAuthReason() const;
    virtual std::string spatCachedAuthReason() const;
    int lookupLeaderIsCav(const std::string& leaderId) const;
    int getSchemaVehicleIndex() const;
    int getSchemaObuId() const;
    void logSchemaSelfState(double timeReceived);
    void logSchemaGroundTruth(const BeaconPayload& cam) const;
    void logSchemaReceivedCam(const BeaconPayload& cam, double timeReceived, double rssiWatts) const;

    // ════════════════════════════════════════════════════════════
    //  V2V parameters
    // ════════════════════════════════════════════════════════════
    double beaconInterval_;
    double leaderCamMaxAge_;
    double beaconRange_;
    bool   useBeaconGap_;
    double authDelayMean_   = 0.0005;
    double authDelayStd_    = 0.0001;
    double authCacheWindow_ = 20.0;
    double authFreshnessWindow_ = -1.0;

    // ════════════════════════════════════════════════════════════
    //  Own position cache
    // ════════════════════════════════════════════════════════════
    double cachedMyX_     = 0.0;
    double cachedMyY_     = 0.0;
    double cachedMySpeed_ = 0.0;
    double cachedMyAccel_ = 0.0;

    // ════════════════════════════════════════════════════════════
    //  Leader cache from CAMs
    // ════════════════════════════════════════════════════════════
    std::string cachedLeaderId_;
    double      cachedLeaderGap_    = -1.0;
    double      cachedLeaderSpeed_  =  0.0;
    double      cachedLeaderCamAge_ =  1e9;

    // ════════════════════════════════════════════════════════════
    //  CAM TX timer
    // ════════════════════════════════════════════════════════════
    cMessage* camTimer_ = nullptr;

    struct PendingCamAuth {
        BeaconPayload cam;
        double        timeReceived = 0.0;
        double        configuredDelay = 0.0;
        std::string   authReason;
        std::string   cacheKey;
    };

    struct PendingSpatAuth {
        SpatPayload  spat;
        double       timeReceived   = 0.0;
        double       speedAtReceipt = 0.0;
        double       configuredDelay = 0.0;
        std::string  authReason;
        std::string  cacheKey;
    };

    std::map<cMessage*, PendingCamAuth>  pendingCamAuth_;
    std::map<cMessage*, PendingSpatAuth> pendingSpatAuth_;
    std::map<std::string, double>        authCacheExpiry_;

    // ════════════════════════════════════════════════════════════
    //  Statistics
    // ════════════════════════════════════════════════════════════
    long stat_cam_tx_        = 0;
    long stat_cam_rx_        = 0;
    long stat_cam_rx_leader_ = 0;
    long stat_cam_dropped_   = 0;
    long stat_spat_rx_       = 0;
    long stat_spat_stale_    = 0;   // times SPaT went stale
    long stat_auth_full_     = 0;
    long stat_auth_cachehit_ = 0;
    long stat_auth_stale_drop_ = 0;

    // ════════════════════════════════════════════════════════════
    //  involves_cav helper and CSV
    // ════════════════════════════════════════════════════════════
    int  computeInvolvesCav(const std::string& otherId) const;
    void initCamResponseFile();
    void writeCamRow(const std::string& senderId,
                     double timeSent,
                     double timeReceived,
                     double timeActed,
                     double v2vGap,
                     int    involvesCav,
                     const std::string& reason);
};

} // namespace veins
