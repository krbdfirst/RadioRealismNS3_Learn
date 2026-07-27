// ManualVehicle5G.cc — 5G mixed-traffic vehicle
// MODIFIED: Added application-layer flood detection and drop metrics.
//
// Changes vs original:
//   - onWSM() now tracks per-second message rate and drops messages
//     exceeding FLOOD_THRESHOLD (default 50 msg/s)
//   - New members: wsm_rx_in_window_, wsm_rx_window_start_,
//     stat_wsm_rx_, stat_wsm_flood_dropped_
//   - finish() records wsm_received and wsm_flood_dropped scalars
//   - flood_detected_at_s scalar records first detection time
//   - All other behaviour unchanged from original
//
// FIX: Removed all delete frame / delete bsm calls from onWSM().
//      Veins framework owns and deletes received frames after onWSM() returns.
//      Calling delete here caused double-free → heap corruption → c0000374.

#include "ManualVehicle5G.h"
#include "veins/modules/application/traci/RunMetadata.h"
#include <iomanip>
#include <cmath>

using namespace veins;
Define_Module(veins::ManualVehicle5G);

extern std::ofstream s_timeseriesFile;
extern bool          s_fileInitialised;

// ── Flood detection thresholds ────────────────────────────────────────────────
static constexpr double RATE_WINDOW_S   = 1.0;  // sliding window length (seconds)
static constexpr int    FLOOD_THRESHOLD = 50;   // msg/window before drop kicks in
                                                 // normal 10 Hz → 10/window; >50 = anomaly

void ManualVehicle5G::initialize(int stage)
{
    Car5G::initialize(stage);
    if (stage == 0) {
        manualVehicleType_ = par("manualVehicleType").stdstringValue();

        // Flood detection state
        stat_wsm_rx_            = 0;
        stat_wsm_flood_dropped_ = 0;
        wsm_rx_in_window_       = 0;
        wsm_rx_window_start_    = 0.0;
        floodDetectedAt_        = -1.0;
        floodActive_            = false;
    }
    if (stage == 1) {
        try { sumoVehicleType_ = traciVehicle->getTypeId(); }
        catch (...) { sumoVehicleType_ = "unknown"; }
        manualMode_ = (sumoVehicleType_ == manualVehicleType_);
        EV_INFO << "[ManualVehicle5G] vType=" << sumoVehicleType_
                << (manualMode_ ? " → MANUAL (no V2X)" : " → CAV (full PC5 stack)") << "\n";
    }
}

void ManualVehicle5G::finish()
{
    recordScalar("wsm_received",        stat_wsm_rx_);
    recordScalar("wsm_flood_dropped",   stat_wsm_flood_dropped_);
    recordScalar("flood_detected_at_s", floodDetectedAt_);

    Car5G::finish();
}

// ─────────────────────────────────────────────────────────────────────────────
//  onWSM
//  FIX: Never delete frame — framework owns it and deletes after this returns.
//  Manual vehicles simply return early (no V2X processing).
//  Flood-detected messages are counted and discarded by returning early.
// ─────────────────────────────────────────────────────────────────────────────
void ManualVehicle5G::onWSM(BaseFrame1609_4* frame)
{
    if (manualMode_) {
        // Manual (human) vehicles have no V2X stack — discard silently.
        // FIX: do NOT delete frame — framework deletes it after this returns.
        return;
    }

    // ── Sliding-window rate counter ───────────────────────────
    double now = simTime().dbl();
    if (now - wsm_rx_window_start_ >= RATE_WINDOW_S) {
        wsm_rx_window_start_ = now;
        wsm_rx_in_window_    = 0;
        floodActive_         = false;
    }
    wsm_rx_in_window_++;

    if (wsm_rx_in_window_ > FLOOD_THRESHOLD) {
        // ── Flood detected — count and discard ───────────────
        stat_wsm_flood_dropped_++;
        floodActive_ = true;

        if (floodDetectedAt_ < 0.0) {
            floodDetectedAt_ = now;
            EV_WARN << "[ManualVehicle5G] *** FLOOD DETECTED at t=" << now
                    << "s — rate=" << wsm_rx_in_window_
                    << " msg/window (threshold=" << FLOOD_THRESHOLD << ") ***\n";
        } else {
            EV_DETAIL << "[ManualVehicle5G] Flood drop #" << stat_wsm_flood_dropped_
                      << " at t=" << now
                      << " (rate=" << wsm_rx_in_window_ << "/s)\n";
        }

        // FIX: do NOT delete frame — just return, framework deletes it.
        return;
    }

    // ── Normal processing path ────────────────────────────────
    stat_wsm_rx_++;
    Car5G::onWSM(frame);
}

void ManualVehicle5G::onWSA(DemoServiceAdvertisment* wsa)
{
    if (manualMode_) return;
    Car5G::onWSA(wsa);
}

void ManualVehicle5G::onPc5Delivery(BaseFrame1609_4* frame)
{
    if (manualMode_) return;
    Car5G::onPc5Delivery(frame);
}

void ManualVehicle5G::handleSelfMsg(cMessage* msg)
{
    Car5G::handleSelfMsg(msg);
}

void ManualVehicle5G::handlePositionUpdate(cObject* obj)
{
    if (!manualMode_) {
        Car5G::handlePositionUpdate(obj);
        return;
    }

    // Manual mode — log only, identical logic to ManualVehicle.cc
    DemoBaseApplLayer::handlePositionUpdate(obj);
    if (!isVehicleActive_) return;

    double currentTime = simTime().dbl();
    if (cachedVehicleId5G_.empty()) {
        try { cachedVehicleId5G_ = mobility->getExternalId(); }
        catch (...) { return; }
    }

    double speed = 0.0;
    try { speed = mobility->getSpeed(); } catch (...) { return; }

    double acceleration = 0.0;
    if (prevSpeed >= 0.0) {
        double dt = (simTime() - prevTime).dbl();
        if (dt > 0) acceleration = (speed - prevSpeed) / dt;
    }
    prevSpeed = speed;
    prevTime  = simTime();

    double gap = -1.0; std::string leaderId;
    try {
        auto lp = getLeaderInfo();
        gap = lp.second;
        leaderId = (gap >= 0) ? lp.first : "";
    } catch (...) {}

    double ttc = -1.0;
    if (gap >= 0 && speed > 0) {
        double raw = gap / speed;
        ttc = (raw >= 999.0) ? -1.0 : raw;
    }

    double cx = 0, cy = 0;
    try { auto p = mobility->getPositionAt(simTime()); cx=p.x; cy=p.y; } catch (...) {}

    int hb=0, eb=0;
    if (!inBrakingEpisode_) {
        if (acceleration <= -0.5 && speed > 0.1) {
            inBrakingEpisode_=true; brakeEpisodeDist_=0;
            brakeEpisodeMaxDecel_=acceleration;
            brakeLastX_=cx; brakeLastY_=cy;
            hardBrakeFired_=false; emergencyBrakeFired_=false;
        }
    } else {
        double dx=cx-brakeLastX_, dy=cy-brakeLastY_;
        brakeEpisodeDist_ += std::sqrt(dx*dx+dy*dy);
        brakeLastX_=cx; brakeLastY_=cy;
        if (acceleration < brakeEpisodeMaxDecel_) brakeEpisodeMaxDecel_=acceleration;
        if (!hardBrakeFired_ && brakeEpisodeMaxDecel_ <= -4.0) { hardBrakeFired_=true; hb=1; }
        if (!emergencyBrakeFired_ && brakeEpisodeMaxDecel_ <= -7.0) { emergencyBrakeFired_=true; eb=1; }
        if (speed <= 0.1 || acceleration >= -0.2) inBrakingEpisode_=false;
    }

    int col=0;
    if (speed <= 0.1) {
        if (stoppedSince_ < 0) stoppedSince_=simTime();
        if ((simTime()-stoppedSince_).dbl()>=10.0 && gap>=0 && gap<=1.0) { inCollision_=true; col=1; }
    } else { stoppedSince_=-1; inCollision_=false; }

    if (timeseriesFile && timeseriesFile->is_open()) {
        // involves_cav: 1 if leader is a CAV (id contains "veh_av"), 0 = no CAV
        int egoIsCav = 0;
        int leaderIsCav = (leaderId.find("veh_av") != std::string::npos) ? 1 : 0;
        int involvesCav = leaderIsCav;
        int tlsViolationEvent = 0;
        int pedestrianCollisionEvent = 0;
        int cyclistCollisionEvent = 0;
        int activeTransportCollisionEvent = 0;

        *timeseriesFile << std::fixed << std::setprecision(4)
            << getCurrentRunCsvPrefix()
            << currentTime << "," << cachedVehicleId5G_ << ","
            << egoIsCav << ","
            << sumoVehicleType_ << ","
            << speed << "," << acceleration << ","
            << (gap<0?-1.0:gap) << "," << leaderId << "," << leaderIsCav << "," << ttc << ","
            << (inBrakingEpisode_?brakeEpisodeDist_:0.0) << ","
            << hb << "," << eb << "," << col << ","
            << tlsViolationEvent << ","
            << pedestrianCollisionEvent << ","
            << cyclistCollisionEvent << ","
            << activeTransportCollisionEvent << ","
            << involvesCav << "\n";  // 1=CAV involved, 0=no CAV
        timeseriesFile->flush();
    }
    if (speed >= 1.0) lastDroveAt = simTime();
}
