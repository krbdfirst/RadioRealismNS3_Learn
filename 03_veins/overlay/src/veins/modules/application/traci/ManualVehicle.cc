// ─────────────────────────────────────────────────────────────────────────────
//  ManualVehicle.cc
//
//  Single applType for mixed traffic — adapts at runtime based on SUMO vType.
//
//  Call chain for a veh_human vehicle:
//    initialize(stage=0) → TraCIDemo11p::initialize(0) → sets up CSV etc.
//    initialize(stage=1) → reads traciVehicle->getTypeId()
//                        → matches "veh_human" → manualMode_ = true
//    handlePositionUpdate() → manualMode_=true → writes "veh_human" to CSV
//                           → no SPaT, no setSpeed, no sendDown
//    onWSM()              → manualMode_=true → silent discard
//
//  Call chain for a veh_av vehicle:
//    initialize(stage=0) → TraCIDemo11p::initialize(0) — unchanged
//    initialize(stage=1) → reads getTypeId() → "veh_av" → manualMode_=false
//    handlePositionUpdate() → delegates to TraCIDemo11p (writes "veh_av")
//    onWSM()              → delegates to TraCIDemo11p (SPaT processing)
// ─────────────────────────────────────────────────────────────────────────────

#include "ManualVehicle.h"
#include "veins/modules/application/traci/RunMetadata.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include <iomanip>
#include <cmath>

using namespace veins;

Define_Module(veins::ManualVehicle);

// ─────────────────────────────────────────────────────────────────────────────
//  initialize
// ─────────────────────────────────────────────────────────────────────────────
void ManualVehicle::initialize(int stage)
{
    // Always call TraCIDemo11p first — it opens CSV, sets up state
    TraCIDemo11p::initialize(stage);

    if (stage == 0) {
        // Read ini parameter — which SUMO type = manual
        manualVehicleType_ = par("manualVehicleType").stdstringValue();
        manualMode_ = false;   // determined at stage 1 when TraCI is live
    }

    if (stage == 1) {
        // TraCI is live at stage 1 — safe to call getTypeId()
        try {
            sumoVehicleType_ = traciVehicle->getTypeId();
        } catch (...) {
            sumoVehicleType_ = "unknown";
        }

        // Check if this vehicle is a manual type
        if (sumoVehicleType_ == manualVehicleType_) {
            manualMode_ = true;

            EV_INFO << "[ManualVehicle] vType=" << sumoVehicleType_
                    << " → MANUAL MODE. V2X stack disabled.\n"
                    << "  SUMO IDM + TLS logic drives this vehicle.\n"
                    << "  SPaT ignored. No spat_response.csv rows.\n";
        } else {
            manualMode_ = false;

            EV_INFO << "[ManualVehicle] vType=" << sumoVehicleType_
                    << " → CAV MODE. Full V2X stack active.\n";
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  finish
// ─────────────────────────────────────────────────────────────────────────────
void ManualVehicle::finish()
{
    TraCIDemo11p::finish();
}

// ─────────────────────────────────────────────────────────────────────────────
//  onWSM — discard all frames when in manual mode
// ─────────────────────────────────────────────────────────────────────────────
void ManualVehicle::onWSM(BaseFrame1609_4* frame)
{
    if (manualMode_) {
        // Silent discard — manual vehicle has no V2X OBU.
        // In reality, no frame would ever reach the application layer
        // of an unequipped vehicle. We silently drop here.
        EV_DETAIL << "[ManualVehicle] Frame discarded (no V2X OBU)\n";
        return;
    }
    // CAV — process normally
    TraCIDemo11p::onWSM(frame);
}

// ─────────────────────────────────────────────────────────────────────────────
//  onWSA — discard service advertisements when in manual mode
// ─────────────────────────────────────────────────────────────────────────────
void ManualVehicle::onWSA(DemoServiceAdvertisment* wsa)
{
    if (manualMode_) return;
    TraCIDemo11p::onWSA(wsa);
}

// ─────────────────────────────────────────────────────────────────────────────
//  handleSelfMsg
// ─────────────────────────────────────────────────────────────────────────────
void ManualVehicle::handleSelfMsg(cMessage* msg)
{
    // No manual-mode self-messages are scheduled.
    // For CAV mode, delegate to TraCIDemo11p.
    // For manual mode, still delegate — base class handles framework messages.
    TraCIDemo11p::handleSelfMsg(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
//  handlePositionUpdate
//
//  Manual mode:  logs to vehicle_timeseries.csv with vehicle_type="veh_human"
//                no setSpeed, no SPaT, no sendDown
//  CAV mode:     delegates entirely to TraCIDemo11p (vehicle_type="veh_av")
// ─────────────────────────────────────────────────────────────────────────────
void ManualVehicle::handlePositionUpdate(cObject* obj)
{
    if (!manualMode_) {
        // CAV — TraCIDemo11p handles everything including CSV write
        TraCIDemo11p::handlePositionUpdate(obj);
        return;
    }

    // ── MANUAL MODE — observe and log only ───────────────────
    DemoBaseApplLayer::handlePositionUpdate(obj);

    if (!isVehicleActive_) return;

    double currentTime = simTime().dbl();

    // Cache vehicle ID
    if (cachedVehicleId_.empty()) {
        try {
            cachedVehicleId_ = mobility->getExternalId();
        } catch (...) { return; }
    }

    double speed = 0.0;
    try { speed = mobility->getSpeed(); } catch (...) { return; }

    // ── Acceleration ──────────────────────────────────────────
    double acceleration = 0.0;
    if (prevSpeed >= 0.0) {
        double dt = (simTime() - prevTime).dbl();
        if (dt > 0) acceleration = (speed - prevSpeed) / dt;
    }
    prevSpeed = speed;
    prevTime  = simTime();

    // ── Leader gap ────────────────────────────────────────────
    double      gap      = -1.0;
    std::string leaderId = "";
    try {
        auto lp  = getLeaderInfo();
        gap      = lp.second;
        leaderId = (gap >= 0) ? lp.first : "";
    } catch (...) {}

    // ── TTC ───────────────────────────────────────────────────
    double ttc = -1.0;
    if (gap >= 0) {
        double rel = speed;   // conservative — leader assumed stationary
        if (rel > 0) {
            double raw = gap / rel;
            ttc = (raw >= 999.0) ? -1.0 : raw;
        }
    }

    // ── XY for braking distance ───────────────────────────────
    double cx = 0.0, cy = 0.0;
    try {
        auto pos = mobility->getPositionAt(simTime());
        cx = pos.x; cy = pos.y;
    } catch (...) {}

    // ── Braking episode ───────────────────────────────────────
    int hardBrakeFlag = 0, emergencyBrakeFlag = 0;

    if (!inBrakingEpisode_) {
        if (acceleration <= -0.5 && speed > 0.1) {
            inBrakingEpisode_     = true;
            brakeEpisodeDist_     = 0.0;
            brakeEpisodeMaxDecel_ = acceleration;
            brakeLastX_ = cx; brakeLastY_ = cy;
            hardBrakeFired_ = false; emergencyBrakeFired_ = false;
        }
    } else {
        double dx = cx - brakeLastX_, dy = cy - brakeLastY_;
        brakeEpisodeDist_ += std::sqrt(dx*dx + dy*dy);
        brakeLastX_ = cx; brakeLastY_ = cy;
        if (acceleration < brakeEpisodeMaxDecel_)
            brakeEpisodeMaxDecel_ = acceleration;
        if (!hardBrakeFired_ && brakeEpisodeMaxDecel_ <= -4.0) {
            hardBrakeFired_ = true; hardBrakeFlag = 1;
        }
        if (!emergencyBrakeFired_ && brakeEpisodeMaxDecel_ <= -7.0) {
            emergencyBrakeFired_ = true; emergencyBrakeFlag = 1;
        }
        if (speed <= 0.1 || acceleration >= -0.2)
            inBrakingEpisode_ = false;
    }
    double brakingDist = inBrakingEpisode_ ? brakeEpisodeDist_ : 0.0;

    // ── Collision detection ───────────────────────────────────
    int collisionFlag = 0;
    if (speed <= 0.1) {
        if (stoppedSince_ < 0) stoppedSince_ = simTime();
        if ((simTime() - stoppedSince_).dbl() >= 10.0 &&
            gap >= 0.0 && gap <= 1.0) {
            inCollision_ = true; collisionFlag = 1;
        }
    } else {
        stoppedSince_ = -1; inCollision_ = false;
    }

    // ── Write CSV row with vehicle_type = "veh_human" ─────────
    // involves_cav: 1 if leader is a CAV (id contains "veh_av"), 0 = no CAV
    int egoIsCav = 0;
    int leaderIsCav = (leaderId.find("veh_av") != std::string::npos) ? 1 : 0;
    int involvesCav = leaderIsCav;
    int tlsViolationEvent = 0;
    int pedestrianCollisionEvent = 0;
    int cyclistCollisionEvent = 0;
    int activeTransportCollisionEvent = 0;

    if (timeseriesFile && timeseriesFile->is_open()) {
        *timeseriesFile << std::fixed << std::setprecision(4)
            << getCurrentRunCsvPrefix()
            << currentTime                   << ","
            << cachedVehicleId_              << ","
            << egoIsCav                      << ","
            << sumoVehicleType_              << ","
            << speed                         << ","
            << acceleration                  << ","
            << (gap < 0 ? -1.0 : gap)        << ","
            << leaderId                      << ","
            << leaderIsCav                   << ","
            << ttc                           << ","
            << brakingDist                   << ","
            << hardBrakeFlag                 << ","
            << emergencyBrakeFlag            << ","
            << collisionFlag                 << ","
            << tlsViolationEvent             << ","
            << pedestrianCollisionEvent      << ","
            << cyclistCollisionEvent         << ","
            << activeTransportCollisionEvent << ","
            << involvesCav                   << "\n";  // 1=CAV involved, 0=no CAV
        timeseriesFile->flush();
    }

    // SUMO drives this vehicle — no setSpeed, no V2X commands.
    // Update lastDroveAt for base class consistency.
    if (speed >= 1.0) lastDroveAt = simTime();
}
