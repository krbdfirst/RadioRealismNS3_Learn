#pragma once
// Lightweight SPaT payload encoded inside TraCIDemo11pMessage::demoData.
//
// Current wire format:
//   SPAT,<tlsId>,<phase>,<fullState>,<timeToChange_s>,<timeSent_s>
//
// Backward compatibility:
//   SPAT,<tlsId>,<phase>,<timeToChange_s>,<timeSent_s>
//   SPAT,<tlsId>,<phase>,<timeToChange_s>

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace veins {

struct SpatPayload {
    std::string tlsId;
    char        phase        = 'U';   // summary phase / fallback
    std::string state;                // full SUMO TLS state string
    double      timeToChange = 0.0;
    double      timeSent     = 0.0;

    static char normalizeSignal(char raw)
    {
        switch (raw) {
        case 'G':
        case 'g':
        case 's':
            return 'G';
        case 'Y':
        case 'y':
        case 'u':
            return 'Y';
        case 'R':
        case 'r':
            return 'R';
        default:
            return 'U';
        }
    }

    static char summarizeState(const std::string& fullState)
    {
        bool hasGreen = false;
        bool hasYellow = false;
        bool hasRed = false;

        for (char raw : fullState) {
            char signal = normalizeSignal(raw);
            if (signal == 'Y') hasYellow = true;
            else if (signal == 'G') hasGreen = true;
            else if (signal == 'R') hasRed = true;
        }

        if (hasYellow) return 'Y';
        if (hasGreen) return 'G';
        if (hasRed) return 'R';
        return 'U';
    }

    char phaseForLinkIndex(int linkIndex) const
    {
        if (linkIndex >= 0 && linkIndex < static_cast<int>(state.size())) {
            return normalizeSignal(state[static_cast<size_t>(linkIndex)]);
        }
        return normalizeSignal(phase);
    }

    bool isGreen() const { return normalizeSignal(phase) == 'G'; }
    bool isRed() const { return normalizeSignal(phase) == 'R'; }
    bool isYellow() const { return normalizeSignal(phase) == 'Y'; }

    std::string encode() const
    {
        std::ostringstream s;
        s << "SPAT," << tlsId << "," << normalizeSignal(phase) << "," << state << ","
          << std::fixed << std::setprecision(6) << timeToChange
          << "," << timeSent;
        return s.str();
    }

    static bool parse(const std::string& data, SpatPayload& out)
    {
        if (data.size() < 5 || data.substr(0, 5) != "SPAT,") return false;

        std::istringstream ss(data.substr(5));
        std::vector<std::string> fields;
        std::string field;
        while (std::getline(ss, field, ',')) fields.push_back(field);
        if (fields.size() < 3) return false;

        out.tlsId = fields[0];
        out.phase = fields[1].empty() ? 'U' : normalizeSignal(fields[1][0]);
        out.state.clear();
        out.timeSent = 0.0;

        size_t timeToChangeIndex = 2;
        size_t timeSentIndex = static_cast<size_t>(-1);

        if (fields.size() >= 5) {
            out.state = fields[2];
            timeToChangeIndex = 3;
            timeSentIndex = 4;
        }
        else if (fields.size() >= 4) {
            timeSentIndex = 3;
        }

        try {
            out.timeToChange = std::stod(fields[timeToChangeIndex]);
        }
        catch (...) {
            return false;
        }

        if (out.phase == 'U' && !out.state.empty()) {
            out.phase = summarizeState(out.state);
        }

        if (timeSentIndex < fields.size() && !fields[timeSentIndex].empty()) {
            try {
                out.timeSent = std::stod(fields[timeSentIndex]);
            }
            catch (...) {
            }
        }

        return true;
    }
};

} // namespace veins
