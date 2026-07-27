#include "veins_inet/VeinsInet5GRsuApp.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/Packet.h"
#include "inet/mobility/contract/IMobility.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"
#include "veins/base/utils/FindModule.h"
#include "veins/modules/application/traci/RunMetadata.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins_inet/VeinsInet5GMessage_m.h"

using namespace inet;

namespace {

std::ofstream s_spatBroadcastFile;
bool s_spatBroadcastReady = false;
std::string s_spatBroadcastRunId;

std::string trimCopy(const std::string& value)
{
    const std::string whitespace = " \t\r\n";
    const auto begin = value.find_first_not_of(whitespace);
    if (begin == std::string::npos) return "";
    const auto end = value.find_last_not_of(whitespace);
    return value.substr(begin, end - begin + 1);
}

std::vector<std::string> parseTlsIds(const std::string& raw)
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

char normalizeSignal(char raw)
{
    switch (raw) {
    case 'G':
    case 'g':
        return 'G';
    case 'y':
    case 'Y':
        return 'Y';
    case 'r':
    case 'R':
        return 'R';
    default:
        return 'U';
    }
}

bool hasManagedVehicles()
{
    auto* manager = veins::FindModule<veins::TraCIScenarioManager*>::findGlobalModule();
    return manager && !manager->getManagedHosts().empty();
}

std::string summarizeLinkStates(const std::string& tlsState)
{
    std::ostringstream out;
    for (size_t i = 0; i < tlsState.size(); ++i) {
        if (i > 0) out << ';';
        out << i << ':' << tlsState[i];
    }
    return out.str();
}

std::string rsuOutputDir()
{
    std::string resultDir = veins::getCurrentResultDir();
    if (!resultDir.empty() && resultDir.back() != '/') resultDir += "/";
    return resultDir.empty() ? std::string("./") : resultDir;
}

void resetSpatBroadcastFile()
{
    if (s_spatBroadcastFile.is_open()) s_spatBroadcastFile.close();
    s_spatBroadcastReady = false;
}

void ensureSpatBroadcastFile()
{
    const std::string currentRunId = veins::getCurrentRunId();
    if (currentRunId != s_spatBroadcastRunId) {
        resetSpatBroadcastFile();
        s_spatBroadcastRunId = currentRunId;
    }

    if (s_spatBroadcastReady) return;
    s_spatBroadcastReady = true;
    s_spatBroadcastFile.open(rsuOutputDir() + "spat_broadcast.csv");
    if (!s_spatBroadcastFile.is_open()) return;

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

inet::Coord currentRsuPosition(cModule* host)
{
    if (!host) return inet::Coord::ZERO;
    auto* mobility = dynamic_cast<inet::IMobility*>(host->getSubmodule("mobility"));
    return mobility ? mobility->getCurrentPosition() : inet::Coord::ZERO;
}

} // namespace

Define_Module(VeinsInet5GRsuApp);

int VeinsInet5GRsuApp::numInitStages() const
{
    return inet::NUM_INIT_STAGES;
}

void VeinsInet5GRsuApp::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL) {
        monitoredTlsIds_ = parseTlsIds(par("monitoredTlsId").stdstringValue());
        interface_ = par("interface").stdstringValue();
        port_ = par("port").intValue();
        spatInterval_ = par("spatInterval").doubleValue();
        actualSpatInterval_ = par("actualSpatInterval").doubleValue();
    }
    else if (stage == inet::INITSTAGE_LAST) {
        L3AddressResolver().tryResolve("224.0.0.1", destAddress_);
        ASSERT(!destAddress_.isUnspecified());

        socket_.setOutputGate(gate("socketOut"));
        socket_.bind(L3Address(), port_);

        IInterfaceTable* ift = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
#if INET_VERSION >= 0x0403
        NetworkInterface* ie = ift->findInterfaceByName(interface_.c_str());
#elif INET_VERSION >= 0x0402
        InterfaceEntry* ie = ift->findInterfaceByName(interface_.c_str());
#else
        InterfaceEntry* ie = ift->getInterfaceByName(interface_.c_str());
#endif
        ASSERT(ie);
        socket_.setMulticastOutputInterface(ie->getInterfaceId());
        socket_.setMulticastLoop(false);
        socket_.joinMulticastGroup(destAddress_, ie->getInterfaceId());
        socket_.setCallback(this);

        scheduleSpat();
    }
}

void VeinsInet5GRsuApp::handleMessage(cMessage* msg)
{
    if (timerManager_.handleMessage(msg)) return;
    delete msg;
}

void VeinsInet5GRsuApp::socketDataArrived(UdpSocket* socket, Packet* packet)
{
    emit(packetReceivedSignal, packet);
    delete packet;
}

void VeinsInet5GRsuApp::socketErrorArrived(UdpSocket* socket, Indication* indication)
{
    EV_WARN << "[VeinsInet5GRsuApp] ignoring UDP error " << indication->getName() << "\n";
    delete indication;
}

void VeinsInet5GRsuApp::socketClosed(UdpSocket* socket)
{
}

void VeinsInet5GRsuApp::finish()
{
    recordScalar("inet5g_spat_tx", statSpatTx_);
}

omnetpp::simtime_t VeinsInet5GRsuApp::getActualSpatInterval() const
{
    const auto configuredInterval = SimTime(actualSpatInterval_);
    const auto minimumInterval = SimTime(100, SIMTIME_MS);
    return std::max(minimumInterval, configuredInterval);
}

void VeinsInet5GRsuApp::scheduleSpat()
{
    const auto interval = getActualSpatInterval();
    auto callback = [this]() {
        if (hasManagedVehicles()) broadcastSpat();
        scheduleSpat();
    };
    // Force the RSU heartbeat to land strictly in the future. This avoids
    // pathological zero-time timer churn if the configured interval is ever
    // parsed or downcast unexpectedly during initialization.
    timerManager_.create(veins::TimerSpecification(callback).oneshotAt(simTime() + interval));
}

void VeinsInet5GRsuApp::broadcastSpat()
{
    for (const auto& tlsId : monitoredTlsIds_) {
        double timeToChange = 0.0;
        std::string state;
        char phase = queryTlsPhaseFor(tlsId, timeToChange, state);
        if (phase == 'U') continue;
        sendSpat(tlsId, phase, state, timeToChange);
    }
}

void VeinsInet5GRsuApp::sendSpat(const std::string& tlsId, char phase, const std::string& state, double timeToChange)
{
    auto payload = makeShared<VeinsInet5GMessage>();
    payload->setChunkLength(B(256));
    payload->setMessageKind("SPAT");
    payload->setSenderId(getParentModule()->getFullName());
    payload->setTargetId("");
    payload->setTlsId(tlsId.c_str());
    payload->setTlsState(state.c_str());
    payload->setAuthLabel("INET_5G_RSU");
    payload->setSenderRole("RSU");
    payload->setPurposeTag("signal_control");
    payload->setTimeSent(simTime().dbl());
    const inet::Coord pos = currentRsuPosition(getParentModule());
    payload->setPosX(pos.x);
    payload->setPosY(pos.y);
    payload->setPosZ(pos.z);
    payload->setSpeed(0.0);
    payload->setVelX(0.0);
    payload->setVelY(0.0);
    payload->setVelZ(0.0);
    payload->setAccel(0.0);
    payload->setPhase(phase);
    payload->setTimeToChange(timeToChange);

    auto packet = std::unique_ptr<Packet>(new Packet("spat"));
    packet->insertAtBack(payload);
    emit(packetSentSignal, packet.get());
    socket_.sendTo(packet.release(), destAddress_, port_);
    statSpatTx_++;

    ensureSpatBroadcastFile();
    if (s_spatBroadcastFile.is_open()) {
        s_spatBroadcastFile << std::fixed << std::setprecision(6)
                            << veins::getCurrentRunCsvPrefix()
                            << simTime().dbl() << ","
                            << getParentModule()->getFullName() << ","
                            << tlsId << ","
                            << phase << ","
                            << timeToChange << ","
                            << state << ","
                            << summarizeLinkStates(state) << "\n";
        s_spatBroadcastFile.flush();
    }
}

char VeinsInet5GRsuApp::queryTlsPhaseFor(const std::string& tlsId, double& timeToChange, std::string& tlsState) const
{
    if (tlsId.empty()) return 'U';

    try {
        auto* manager = veins::FindModule<veins::TraCIScenarioManager*>::findGlobalModule();
        if (!manager || !manager->isConnected()) return 'U';

        auto* ci = manager->getCommandInterface();
        if (!ci) return 'U';

        auto tl = ci->trafficlight(tlsId);
        tlsState = tl.getCurrentState();
        const simtime_t nextSwitch = tl.getAssumedNextSwitchTime();
        timeToChange = std::max(0.0, SIMTIME_DBL(nextSwitch) - simTime().dbl());

        int nG = 0;
        int nY = 0;
        int nR = 0;
        for (char raw : tlsState) {
            const char signal = normalizeSignal(raw);
            if (signal == 'G') nG++;
            else if (signal == 'Y') nY++;
            else if (signal == 'R') nR++;
        }

        if (nY > 0) return 'Y';
        if (nG > 0) return 'G';
        if (nR > 0) return 'R';
        return 'U';
    }
    catch (const std::exception& e) {
        EV_WARN << "[VeinsInet5GRsuApp] query TLS " << tlsId << " failed: " << e.what() << "\n";
        return 'U';
    }
    catch (...) {
        return 'U';
    }
}
