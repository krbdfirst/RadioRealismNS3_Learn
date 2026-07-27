//
// SpatRSU.cc
//
// BASELINE RSU — reads one or more TLS phases from SUMO every
// spatInterval seconds and broadcasts a SPaT WSM per monitored TLS.
//
// No auth delay. No pending queue.
// This is the control-group RSU used in Baseline_SPaT.
//

#include "SpatRSU.h"
#include "veins/modules/application/traci/RunMetadata.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

using namespace veins;

Define_Module(veins::SpatRSU);

extern const std::string& getRunOutputDir();

static std::ofstream s_spatBroadcastFile;
static bool s_spatBroadcastReady = false;

static std::string trimCopy(const std::string& value)
{
    const std::string whitespace = " \t\r\n";
    const auto begin = value.find_first_not_of(whitespace);
    if (begin == std::string::npos) return "";
    const auto end = value.find_last_not_of(whitespace);
    return value.substr(begin, end - begin + 1);
}

static std::vector<std::string> parseTlsIdList(const std::string& raw)
{
    std::vector<std::string> result;
    std::istringstream ss(raw);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item = trimCopy(item);
        if (!item.empty()) result.push_back(item);
    }
    return result;
}

static std::string summarizeLinkStates(const std::string& tlsState)
{
    std::ostringstream out;
    for (size_t i = 0; i < tlsState.size(); ++i) {
        if (i > 0) out << ';';
        out << i << ':' << tlsState[i];
    }
    return out.str();
}

static void initSpatBroadcastFile()
{
    if (s_spatBroadcastReady) return;
    s_spatBroadcastReady = true;

    s_spatBroadcastFile.open(getRunOutputDir() + "spat_broadcast.csv");
    if (s_spatBroadcastFile.is_open()) {
        s_spatBroadcastFile
            << "run_number,"
            << "seed_set,"
            << "run_id,"
            << "time_s,"
            << "rsu_module,"
            << "tls_id,"
            << "summary_phase,"
            << "time_to_change_s,"
            << "full_state,"
            << "link_states\n";
        s_spatBroadcastFile.flush();
    }
}

void SpatRSU::initialize(int stage)
{
    TraCIDemoRSU11p::initialize(stage);

    if (stage == 0) {
        monitoredTlsIds_ = parseTlsIdList(par("monitoredTlsId").stdstringValue());
        spatInterval_ = par("spatInterval").doubleValue();
        initSpatBroadcastFile();

        std::ostringstream tlsList;
        for (size_t i = 0; i < monitoredTlsIds_.size(); ++i) {
            if (i > 0) tlsList << ", ";
            tlsList << monitoredTlsIds_[i];
        }

        EV_INFO << "[SpatRSU] Monitoring TLS set='"
                << (tlsList.str().empty() ? std::string("<none>") : tlsList.str())
                << "' interval=" << spatInterval_ << "s  (no auth delay)\n";
    }

    if (stage == 1) {
        spatTimer_ = new cMessage("spatTimer");
        scheduleAt(simTime() + spatInterval_, spatTimer_);
    }
}

void SpatRSU::finish()
{
    if (spatTimer_) {
        cancelAndDelete(spatTimer_);
        spatTimer_ = nullptr;
    }
    TraCIDemoRSU11p::finish();
}

void SpatRSU::handleSelfMsg(cMessage* msg)
{
    if (msg == spatTimer_) {
        broadcastSpat();
        scheduleAt(simTime() + spatInterval_, spatTimer_);
        return;
    }
    TraCIDemoRSU11p::handleSelfMsg(msg);
}

void SpatRSU::onWSM(BaseFrame1609_4* frame)
{
    TraCIDemoRSU11p::onWSM(frame);
}

void SpatRSU::onWSA(DemoServiceAdvertisment* wsa)
{
    TraCIDemoRSU11p::onWSA(wsa);
}

void SpatRSU::broadcastSpat()
{
    for (const auto& tlsId : monitoredTlsIds_) {
        double timeToChange = 0.0;
        std::string tlsState;
        char phase = queryTlsPhaseFor(tlsId, timeToChange, tlsState);
        if (phase == 'U') continue;

        SpatPayload spat;
        spat.tlsId = tlsId;
        spat.phase = phase;
        spat.state = tlsState;
        spat.timeToChange = timeToChange;
        spat.timeSent = simTime().dbl();

        sendSpatPayload(spat);
        logSpatBroadcast(spat);
    }
}

void SpatRSU::sendSpatPayload(const SpatPayload& spat)
{
    auto* wsm = new TraCIDemo11pMessage();
    populateWSM(wsm);
    wsm->setDemoData(spat.encode().c_str());
    wsm->setSenderAddress(getParentModule()->getId());
    wsm->setSerial(0);

    EV_INFO << "[SpatRSU] Broadcast: " << spat.tlsId
            << " phase=" << spat.phase
            << " state=" << spat.state
            << " ttc=" << std::fixed << std::setprecision(2) << spat.timeToChange << "s\n";

    sendDown(wsm);
}

void SpatRSU::logSpatBroadcast(const SpatPayload& spat)
{
    if (!s_spatBroadcastFile.is_open()) return;

    s_spatBroadcastFile << std::fixed << std::setprecision(6)
                        << getCurrentRunCsvPrefix()
                        << simTime().dbl() << ","
                        << getParentModule()->getFullName() << ","
                        << spat.tlsId << ","
                        << spat.phase << ","
                        << spat.timeToChange << ","
                        << spat.state << ","
                        << summarizeLinkStates(spat.state) << "\n";
    s_spatBroadcastFile.flush();
}

char SpatRSU::queryTlsPhaseFor(const std::string& tlsId, double& timeToChange, std::string& tlsState)
{
    if (tlsId.empty()) return 'U';
    tlsState.clear();
    try {
        auto* mgr = FindModule<TraCIScenarioManager*>::findGlobalModule();
        if (!mgr || !mgr->isConnected()) return 'U';

        auto* ci = mgr->getCommandInterface();
        if (!ci) return 'U';

        auto tl = ci->trafficlight(tlsId);

        std::string state = tl.getCurrentState();
        simtime_t nextSwitch = tl.getAssumedNextSwitchTime();
        tlsState = state;

        timeToChange = SIMTIME_DBL(nextSwitch) - simTime().dbl();
        if (timeToChange < 0) timeToChange = 0.0;

        int nG = 0;
        int nY = 0;
        int nR = 0;
        for (char raw : state) {
            char signal = SpatPayload::normalizeSignal(raw);
            if (signal == 'G') nG++;
            else if (signal == 'Y') nY++;
            else if (signal == 'R') nR++;
        }

        if (nY > 0) return 'Y';
        if (nG > 0) return 'G';
        if (nR > 0) return 'R';
        return 'U';

    } catch (const std::exception& e) {
        EV_WARN << "[SpatRSU] queryTlsPhase(" << tlsId << "): " << e.what() << "\n";
        return 'U';
    } catch (...) {
        return 'U';
    }
}
