//
// RSU5G.cc
//
// 5G NR PC5 Sidelink RSU application.
// MODIFIED: Added per-sender rate tracking and blacklist for flood detection.
//
// Changes vs original:
//   - onWSM() now counts messages per sender per second
//   - Senders exceeding RSU_FLOOD_THRESHOLD are blacklisted
//   - Blacklisted senders have all subsequent messages dropped immediately
//   - New members: senderMsgCount_, senderWindowStart_, blacklist_
//   - New output scalars: flood_sources_detected, flood_msgs_blocked_rsu
//   - All SPaT broadcasting behaviour unchanged
//
// FIX: Removed all delete frame calls from onWSM().
//      Veins framework owns and deletes received frames after onWSM() returns.
//      Calling delete here caused double-free → heap corruption → c0000374.

#include "RSU5G.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include <iomanip>
#include <sstream>

using namespace veins;

Define_Module(veins::RSU5G);

// ── RSU flood detection threshold ─────────────────────────────────────────────
static constexpr int    RSU_FLOOD_THRESHOLD = 30;   // msg/s per sender before blacklist
static constexpr double RSU_RATE_WINDOW_S   = 1.0;

static std::string formatFixed(double value, int precision)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;
    return out.str();
}

// ─────────────────────────────────────────────────────────────────────────────
//  initialize
// ─────────────────────────────────────────────────────────────────────────────
void RSU5G::initialize(int stage)
{
    SpatRSU::initialize(stage);

    if (stage == 0) {
        pc5AirDelayMean_ = par("pc5AirDelayMean").doubleValue();
        pc5AirDelayStd_  = par("pc5AirDelayStd").doubleValue();
        pc5Range_        = par("pc5Range").doubleValue();

        stat_flood_sources_detected_ = 0;
        stat_flood_msgs_blocked_     = 0;

        EV_INFO << "[RSU5G] PC5 NR-V2X RSU initialised.\n"
                << "  Air delay      : " << pc5AirDelayMean_*1000 << "ms mean\n"
                << "  Range          : " << pc5Range_ << "m\n"
                << "  Flood threshold: " << RSU_FLOOD_THRESHOLD << " msg/s/sender\n";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  onWSM
//  FIX: Never delete frame — framework owns it and deletes after this returns.
//  Blacklisted/flood messages are discarded by returning early without delete.
// ─────────────────────────────────────────────────────────────────────────────
void RSU5G::onWSM(BaseFrame1609_4* frame)
{
    auto* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);
    int sender = wsm->getSenderAddress();
    double now = simTime().dbl();

    // ── Check blacklist first (O(1) lookup) ───────────────────
    if (blacklist_.count(sender)) {
        stat_flood_msgs_blocked_++;
        EV_DETAIL << "[RSU5G] Blocked msg from blacklisted sender "
                  << sender << " (#" << stat_flood_msgs_blocked_ << " blocked)\n";
        // FIX: do NOT delete frame — just return, framework deletes it.
        return;
    }

    // ── Sliding window rate counter per sender ────────────────
    if (now - senderWindowStart_[sender] >= RSU_RATE_WINDOW_S) {
        senderWindowStart_[sender] = now;
        senderMsgCount_[sender]    = 0;
    }
    senderMsgCount_[sender]++;

    if (senderMsgCount_[sender] > RSU_FLOOD_THRESHOLD) {
        if (!blacklist_.count(sender)) {
            blacklist_.insert(sender);
            stat_flood_sources_detected_++;
            EV_WARN << "[RSU5G] *** FLOOD SOURCE DETECTED: sender=" << sender
                    << " rate=" << senderMsgCount_[sender] << " msg/s"
                    << " (threshold=" << RSU_FLOOD_THRESHOLD << ")"
                    << " → BLACKLISTED ***\n";
        }
        stat_flood_msgs_blocked_++;
        // FIX: do NOT delete frame — just return, framework deletes it.
        return;
    }

    // ── Normal message — pass to parent ──────────────────────
    SpatRSU::onWSM(frame);
}

// ─────────────────────────────────────────────────────────────────────────────
//  broadcastSpat  — unchanged
// ─────────────────────────────────────────────────────────────────────────────
void RSU5G::sendSpatPayload(const SpatPayload& spat)
{
    TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
    populateWSM(wsm);
    wsm->setDemoData(spat.encode().c_str());
    wsm->setSenderAddress(getParentModule()->getId());
    wsm->setSerial(0);

    std::ostringstream logLine;
    logLine << "[RSU5G] PC5 broadcast: " << spat.tlsId
            << " phase=" << spat.phase
            << " state=" << spat.state
            << " ttc=" << formatFixed(spat.timeToChange, 2) << "s"
            << " [NR PC5 Mode 4]\n";
    EV_INFO << logLine.str();

    sendDown(wsm);
}

// ─────────────────────────────────────────────────────────────────────────────
//  finish
// ─────────────────────────────────────────────────────────────────────────────
void RSU5G::finish()
{
    recordScalar("flood_sources_detected", stat_flood_sources_detected_);
    recordScalar("flood_msgs_blocked_rsu", stat_flood_msgs_blocked_);

    std::ostringstream summary;
    summary << "[RSU5G] finish():"
            << " flood_sources=" << stat_flood_sources_detected_
            << " msgs_blocked="  << stat_flood_msgs_blocked_
            << " blacklist_size=" << blacklist_.size() << "\n";
    EV_INFO << summary.str();

    SpatRSU::finish();
}
