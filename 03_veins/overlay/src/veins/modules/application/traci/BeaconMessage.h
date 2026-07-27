#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  BeaconMessage.h
//
//  Lightweight CAM (Cooperative Awareness Message) payload for V2V.
//  Encoded as a plain string inside TraCIDemo11pMessage::demoData —
//  no new OMNeT++ .msg file needed. Detected by "CAM," prefix, exactly
//  as SpatMessage.h uses "SPAT," prefix.
//
//  Wire format:
//    "CAM,<senderId>,<messageId>,<speed_mps>,<velX_mps>,<velY_mps>,<velZ_mps>,<posX_m>,<posY_m>,<posZ_m>,<accel_mps2>,<timeSent_s>"
//
//  Fields:
//    senderId    — SUMO vehicle externalId (e.g. "vehicle.42")
//    messageId   — sender-local monotonically increasing CAM identifier
//    speed_mps   — sender speed at broadcast time (m/s)
//    velX_mps    — sender claimed x velocity at broadcast time (m/s)
//    velY_mps    — sender claimed y velocity at broadcast time (m/s)
//    velZ_mps    — sender claimed z velocity at broadcast time (m/s)
//    posX_m      — sender X position in SUMO coordinate system (m)
//    posY_m      — sender Y position in SUMO coordinate system (m)
//    posZ_m      — sender Z position in SUMO coordinate system (m)
//    accel_mps2  — sender longitudinal acceleration (m/s², negative = braking)
//    timeSent_s  — simTime().dbl() at the instant sendDown() is called
//                  allows full E2E V2V latency decomposition:
//                    prop_delay  = time_received_s - timeSent_s
//                    auth_delay  = time_acted_s    - time_received_s
//                    total_delay = time_acted_s    - timeSent_s
//
//  References:
//    ETSI EN 302 637-2 v1.4.1  — CAM generation rules (10Hz, 500ms triggered)
//    3GPP TS 23.287            — NR V2X application layer for CAM/BSM
// ─────────────────────────────────────────────────────────────────────────────

#include <string>
#include <sstream>
#include <iomanip>

namespace veins {

struct BeaconPayload {
    std::string senderId;
    long long   messageId   = 0;
    double      speed_mps   = 0.0;
    double      velX_mps    = 0.0;
    double      velY_mps    = 0.0;
    double      velZ_mps    = 0.0;
    double      posX_m      = 0.0;
    double      posY_m      = 0.0;
    double      posZ_m      = 0.0;
    double      accel_mps2  = 0.0;
    double      timeSent_s  = 0.0;

    // ── encode ────────────────────────────────────────────────
    std::string encode() const {
        std::ostringstream s;
        s << "CAM,"
          << senderId    << ","
          << messageId   << ","
          << std::fixed << std::setprecision(6)
          << speed_mps   << ","
          << velX_mps    << ","
          << velY_mps    << ","
          << velZ_mps    << ","
          << posX_m      << ","
          << posY_m      << ","
          << posZ_m      << ","
          << accel_mps2  << ","
          << timeSent_s;
        return s.str();
    }

    // ── parse ─────────────────────────────────────────────────
    // Returns false if data is not a CAM message or is malformed.
    static bool parse(const std::string& data, BeaconPayload& out) {
        if (data.size() < 4 || data.substr(0, 4) != "CAM,") return false;
        std::istringstream ss(data.substr(4));
        std::string senderId, msgId, spd, vx, vy, vz, px, py, pz, acc, ts;
        if (!std::getline(ss, senderId, ',')) return false;
        if (!std::getline(ss, msgId,    ',')) return false;
        if (!std::getline(ss, spd,      ',')) return false;
        if (!std::getline(ss, vx,       ',')) return false;
        if (!std::getline(ss, vy,       ',')) return false;
        if (!std::getline(ss, vz,       ',')) return false;
        if (!std::getline(ss, px,       ',')) return false;
        if (!std::getline(ss, py,       ',')) return false;
        if (!std::getline(ss, pz,       ',')) return false;
        if (!std::getline(ss, acc,      ',')) return false;
        if (!std::getline(ss, ts,       ',')) return false;

        if (senderId.empty()) return false;
        out.senderId = senderId;
        try {
            out.messageId  = std::stoll(msgId);
            out.speed_mps  = std::stod(spd);
            out.velX_mps   = std::stod(vx);
            out.velY_mps   = std::stod(vy);
            out.velZ_mps   = std::stod(vz);
            out.posX_m     = std::stod(px);
            out.posY_m     = std::stod(py);
            out.posZ_m     = std::stod(pz);
            out.accel_mps2 = std::stod(acc);
            out.timeSent_s = std::stod(ts);
        } catch (...) { return false; }
        return true;
    }

    // ── euclidean distance from a given position ──────────────
    double distanceTo(double x, double y) const {
        double dx = posX_m - x;
        double dy = posY_m - y;
        return std::sqrt(dx*dx + dy*dy);
    }
};

} // namespace veins
