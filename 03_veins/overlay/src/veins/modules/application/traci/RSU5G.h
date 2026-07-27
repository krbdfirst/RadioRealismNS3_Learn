#pragma once
//
// RSU5G.h
//
// 5G NR PC5 Sidelink RSU — broadcasts SPaT via NR sidelink.
//
// Differences from SpatRSU (11p baseline):
//   - Inherits from SpatRSU so SPaT query/broadcast logic is reused
//   - Overrides broadcastSpat() to stamp messages with PC5 parameters
//   - Models higher-power RSU transmission (infrastructure node,
//     fixed antenna, higher EIRP than vehicle)
//   - No CCH/SCH switching — continuous broadcast on PC5 channel
//
// MODIFIED: Added per-sender flood detection and blacklist members.
//   New members:
//     senderMsgCount_              — message count per sender in current window
//     senderWindowStart_           — window start time per sender
//     blacklist_                   — set of blacklisted sender IDs
//     stat_flood_sources_detected_ — number of unique senders blacklisted
//     stat_flood_msgs_blocked_     — total messages dropped by RSU flood filter
//   New overrides:
//     onWSM()  — intercepts inbound messages for rate-limit check
//     finish() — records flood detection scalars
//

#include "SpatRSU.h"
#include <map>
#include <set>

namespace veins {

class RSU5G : public SpatRSU {

public:
    void initialize(int stage) override;
    void finish()              override;

protected:
    // ── PC5 air interface parameters ──────────────────────────────────────────
    double pc5AirDelayMean_;
    double pc5AirDelayStd_;
    double pc5Range_;

    // ── Flood detection — per-sender sliding window ───────────────────────────
    std::map<int, int>    senderMsgCount_;      // sender ID → msg count in window
    std::map<int, double> senderWindowStart_;   // sender ID → window start time (s)
    std::set<int>         blacklist_;            // blacklisted sender IDs

    // ── Flood detection statistics ────────────────────────────────────────────
    long stat_flood_sources_detected_ = 0;  // unique senders that exceeded threshold
    long stat_flood_msgs_blocked_     = 0;  // total messages dropped by flood filter

    // ── Overrides ─────────────────────────────────────────────────────────────
    void sendSpatPayload(const SpatPayload& spat) override;
    void onWSM(BaseFrame1609_4* frame) override;
};

} // namespace veins
