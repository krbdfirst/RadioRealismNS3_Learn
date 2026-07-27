//
// Car5G.cc
//
// CV2X 5G PC5 Sidelink Mode 4 vehicle application.
//
// This implementation now uses two paper-grounded MAC approximations:
//   1. Half-duplex loss      delta_HD = lambda / 1000
//   2. SB-SPS collision loss local CBR/resource-pool approximation
//
// Reference:
//   Gonzalez-Martin et al., "Analytical Models of the Performance of
//   C-V2X Mode 4 Vehicular Communications", IEEE, 2018.
//

#include "Car5G.h"
#include "BeaconMessage.h"
#include "SpatMessage.h"
#include "veins/base/utils/FindModule.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include <algorithm>
#include <cmath>
#include <iomanip>

using namespace veins;

extern void recordPdrAttempt(bool isV2V, double t);

namespace {

constexpr double kMsPerSecond = 1000.0;
constexpr double kReferenceDistanceMeters = 1.0;

double clamp01(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

TraCIMobility* getNodeMobility(cModule* host)
{
    if (!host) return nullptr;
    if (auto* mobility = dynamic_cast<TraCIMobility*>(host->getSubmodule("veinsmobility"))) return mobility;
    return dynamic_cast<TraCIMobility*>(host->getSubmodule("mobility"));
}

Car5G* getPc5App(cModule* host)
{
    if (!host) return nullptr;
    return dynamic_cast<Car5G*>(host->getSubmodule("appl"));
}

Coord getNodePosition(cModule* host)
{
    if (auto* mobility = getNodeMobility(host)) return mobility->getPositionAt(simTime());
    return Coord::ZERO;
}

cModule* getSenderHostFromFrame(BaseFrame1609_4* frame)
{
    auto* demoFrame = dynamic_cast<TraCIDemo11pMessage*>(frame);
    if (!demoFrame) return nullptr;
    int senderId = static_cast<int>(demoFrame->getSenderAddress());
    auto* sim = cSimulation::getActiveSimulation();
    if (!sim) return nullptr;
    return sim->getModule(senderId);
}

} // namespace

Define_Module(veins::Car5G);

double Car5G::getPacketRateHz() const
{
    double interval = 0.0;
    try {
        interval = par("beaconInterval").doubleValue();
    } catch (...) {
        interval = 0.0;
    }
    if (interval > 0.0) return 1.0 / interval;
    return 10.0;
}

double Car5G::getPacketSensingRatio(double distanceMeters) const
{
    double d = std::max(distanceMeters, kReferenceDistanceMeters);
    double pathLossDb = pc5ReferenceLossDb_ + 10.0 * pc5PathLossExponent_ * std::log10(d / kReferenceDistanceMeters);
    double numerator = pc5TxPowerDbm_ - pathLossDb - pc5SensingThresholdDbm_;

    if (pc5ShadowingStdDb_ <= 0.0) return numerator >= 0.0 ? 1.0 : 0.0;

    double denominator = pc5ShadowingStdDb_ * std::sqrt(2.0);
    return clamp01(0.5 * (1.0 + std::erf(numerator / denominator)));
}

double Car5G::getTotalResourcePool() const
{
    double lambda = std::max(1e-6, getPacketRateHz());
    double subchannelsPerPacket = std::max(1.0, pc5ResourcesPerPacket_);
    double totalResources = (kMsPerSecond * std::max(1.0, pc5SubchannelsPerSubframe_)) / lambda;
    return std::max(1.0, totalResources / subchannelsPerPacket);
}

double Car5G::estimateChannelBusyRatio(const Coord& receiverPos, cModule* selfHost) const
{
    auto* manager = FindModule<TraCIScenarioManager*>::findGlobalModule();
    if (!manager) return 0.0;

    double totalResources = getTotalResourcePool();

    double sensedVehicles = 0.0;
    for (const auto& kv : manager->getManagedHosts()) {
        cModule* host = kv.second;
        if (!host || host == selfHost) continue;
        if (!getPc5App(host)) continue;

        Coord otherPos = getNodePosition(host);
        double distance = receiverPos.distance(otherPos);
        if (pc5Range_ > 0.0 && distance > pc5Range_) continue;

        sensedVehicles += getPacketSensingRatio(distance);
    }

    double excludedResources = sensedVehicles / 2.0;
    int halfSensed = static_cast<int>(std::floor(sensedVehicles / 2.0));
    for (int k = 1; k <= halfSensed; ++k) {
        double denominator = totalResources - (sensedVehicles / 2.0);
        if (denominator <= 0.0) break;
        excludedResources += std::max(1.0 - (static_cast<double>(k) / denominator), 0.0);
    }

    return clamp01(excludedResources / totalResources);
}

double Car5G::estimateSpsCollisionDrop(BaseFrame1609_4* frame, const Coord& receiverPos, cModule* selfHost) const
{
    auto* senderHost = getSenderHostFromFrame(frame);
    if (!senderHost || senderHost == selfHost) return 0.0;

    auto* manager = FindModule<TraCIScenarioManager*>::findGlobalModule();
    if (!manager) return 0.0;

    double lambda = std::max(1e-6, getPacketRateHz());
    double totalResources = getTotalResourcePool();
    double candidateResources = std::max(1.0, pc5CandidateResourceFraction_ * totalResources);

    // Average number of transmissions kept on the same resource.
    double tau = std::max(1.0, lambda * std::max(sbspsReselectionInterval_, 1e-3));

    double cbr = estimateChannelBusyRatio(receiverPos, selfHost);
    double alpha = 0.0;
    if (cbr > 0.7) alpha = 1.0;
    else if (cbr >= 0.2) alpha = clamp01((2.0 * cbr) - 0.4);

    double sensedVehicles = 0.0;
    for (const auto& kv : manager->getManagedHosts()) {
        cModule* host = kv.second;
        if (!host || host == selfHost) continue;
        if (!getPc5App(host)) continue;

        Coord otherPos = getNodePosition(host);
        double distance = receiverPos.distance(otherPos);
        if (pc5Range_ > 0.0 && distance > pc5Range_) continue;

        sensedVehicles += getPacketSensingRatio(distance);
    }

    double excludedResources = sensedVehicles / 2.0;
    int halfSensed = static_cast<int>(std::floor(sensedVehicles / 2.0));
    for (int k = 1; k <= halfSensed; ++k) {
        double denominator = totalResources - (sensedVehicles / 2.0);
        if (denominator <= 0.0) break;
        excludedResources += std::max(1.0 - (static_cast<double>(k) / denominator), 0.0);
    }
    double assignableResources = std::max(candidateResources, totalResources - excludedResources);

    Coord senderPos = getNodePosition(senderHost);
    double senderToReceiverPsr = getPacketSensingRatio(receiverPos.distance(senderPos));
    double collisionSurvival = 1.0;

    for (const auto& kv : manager->getManagedHosts()) {
        cModule* interfererHost = kv.second;
        if (!interfererHost || interfererHost == selfHost || interfererHost == senderHost) continue;
        if (!getPc5App(interfererHost)) continue;

        Coord interfererPos = getNodePosition(interfererHost);
        double receiverDistance = receiverPos.distance(interfererPos);
        if (pc5Range_ > 0.0 && receiverDistance > pc5Range_) continue;

        double senderDistance = senderPos.distance(interfererPos);
        double ps = 1.0 - ((1.0 - (1.0 / tau)) * getPacketSensingRatio(senderDistance));

        // Approximate Step 2 and Step 3 with random selection over
        // candidate and assignable resource pools, respectively.
        double pSimStep2 = ps / candidateResources;
        double pSimStep3 = ps / assignableResources;
        double pSim = clamp01((alpha * pSimStep2) + ((1.0 - alpha) * pSimStep3));

        // Approximate pINT with distance-dependent interference
        // strength at the receiver. Nearby interferers are more
        // likely to corrupt the packet; very weak ones contribute
        // little even if they picked the same resource.
        double pInt = clamp01(getPacketSensingRatio(receiverDistance) * (1.0 - (0.5 * senderToReceiverPsr)));

        collisionSurvival *= (1.0 - clamp01(pSim * pInt));
    }

    return clamp01(1.0 - collisionSurvival);
}

// ─────────────────────────────────────────────────────────────
//  initialize
// ─────────────────────────────────────────────────────────────
void Car5G::initialize(int stage)
{
    TraCIDemo11p::initialize(stage);

    if (stage == 0) {
        pc5AirDelayMean_ = par("pc5AirDelayMean").doubleValue();
        pc5AirDelayStd_  = par("pc5AirDelayStd").doubleValue();
        pc5DropRate_     = par("pc5DropRate").doubleValue();
        pc5Range_        = par("pc5Range").doubleValue();

        sbspsReselectionInterval_   = par("sbspsReselectionInterval").doubleValue();
        sbspsReselectionSilence_    = par("sbspsReselectionSilence").doubleValue();
        pc5SubchannelsPerSubframe_  = par("pc5SubchannelsPerSubframe").doubleValue();
        pc5ResourcesPerPacket_      = par("pc5ResourcesPerPacket").doubleValue();
        pc5CandidateResourceFraction_ = par("pc5CandidateResourceFraction").doubleValue();
        pc5TxPowerDbm_              = par("pc5TxPowerDbm").doubleValue();
        pc5SensingThresholdDbm_     = par("pc5SensingThresholdDbm").doubleValue();
        pc5ShadowingStdDb_          = par("pc5ShadowingStdDb").doubleValue();
        pc5PathLossExponent_        = par("pc5PathLossExponent").doubleValue();
        pc5ReferenceLossDb_         = par("pc5ReferenceLossDb").doubleValue();

        pc5HalfDuplexDrop_ = par("pc5HalfDuplexDrop").doubleValue();
        if (pc5HalfDuplexDrop_ < 0.0) pc5HalfDuplexDrop_ = clamp01(getPacketRateHz() / kMsPerSecond);

        EV_INFO << "[Car5G] PC5 Mode 4 initialised.\n"
                << "  Air delay           : " << pc5AirDelayMean_ * 1000
                << " ± " << pc5AirDelayStd_ * 1000 << " ms\n"
                << "  Base drop rate      : " << pc5DropRate_ * 100 << "%\n"
                << "  Half-duplex (eq. 7) : " << pc5HalfDuplexDrop_ * 100 << "%\n"
                << "  Packet rate lambda  : " << getPacketRateHz() << " Hz\n"
                << "  Resource interval   : " << sbspsReselectionInterval_ << " s\n"
                << "  Legacy silence      : " << sbspsReselectionSilence_ * 1000 << " ms (unused)\n"
                << "  Subchannels/frame   : " << pc5SubchannelsPerSubframe_ << "\n"
                << "  Resources/packet    : " << pc5ResourcesPerPacket_ << "\n"
                << "  Candidate fraction  : " << pc5CandidateResourceFraction_ << "\n"
                << "  Range               : " << pc5Range_ << " m\n";
    }
}

// ─────────────────────────────────────────────────────────────
//  finish
// ─────────────────────────────────────────────────────────────
void Car5G::finish()
{
    for (auto& kv : pendingPc5_) {
        cancelAndDelete(kv.first);
        delete kv.second.frame;
    }
    pendingPc5_.clear();

    long totalDropped = stat_rx_dropped_base_
        + stat_rx_dropped_hd_
        + stat_rx_dropped_sps_;

    EV_INFO << "[Car5G] Final stats:\n"
            << "  rx total       : " << stat_rx_total_        << "\n"
            << "  delivered      : " << stat_rx_delayed_      << "\n"
            << "  dropped (base) : " << stat_rx_dropped_base_ << "\n"
            << "  dropped (HD)   : " << stat_rx_dropped_hd_   << "\n"
            << "  dropped (SPS)  : " << stat_rx_dropped_sps_  << "\n"
            << "  total dropped  : " << totalDropped          << "\n";

    TraCIDemo11p::finish();
}

// ─────────────────────────────────────────────────────────────
//  handlePositionUpdate — inherited unchanged from TraCIDemo11p
// ─────────────────────────────────────────────────────────────
void Car5G::handlePositionUpdate(cObject* obj)
{
    TraCIDemo11p::handlePositionUpdate(obj);
}

// ─────────────────────────────────────────────────────────────
//  onWSM — PC5 drop pipeline
// ─────────────────────────────────────────────────────────────
void Car5G::onWSM(BaseFrame1609_4* frame)
{
    stat_rx_total_++;
    if (auto* tmsg = dynamic_cast<TraCIDemo11pMessage*>(frame)) {
        const std::string payload = tmsg->getDemoData();
        BeaconPayload beacon;
        SpatPayload spat;
        if (BeaconPayload::parse(payload, beacon)) recordPdrAttempt(true, simTime().dbl());
        else if (SpatPayload::parse(payload, spat)) recordPdrAttempt(false, simTime().dbl());
    }

    cModule* selfHost = getParentModule();
    Coord receiverPos = getNodePosition(selfHost);

    // ── Check 1: analytical half-duplex drop ─────────────────
    if (pc5HalfDuplexDrop_ > 0.0 && uniform(0.0, 1.0) < pc5HalfDuplexDrop_) {
        stat_rx_dropped_hd_++;
        EV_DETAIL << "[Car5G] Dropped: analytical half-duplex loss\n";
        return;
    }

    // ── Check 2: analytical SB-SPS collision drop ────────────
    double spsCollisionDrop = estimateSpsCollisionDrop(frame, receiverPos, selfHost);
    if (spsCollisionDrop > 0.0 && uniform(0.0, 1.0) < spsCollisionDrop) {
        stat_rx_dropped_sps_++;
        EV_DETAIL << "[Car5G] Dropped: analytical SB-SPS collision loss p="
                  << spsCollisionDrop << "\n";
        return;
    }

    // ── Check 3: residual base drop rate ─────────────────────
    if (pc5DropRate_ > 0.0 && uniform(0.0, 1.0) < pc5DropRate_) {
        stat_rx_dropped_base_++;
        EV_DETAIL << "[Car5G] Dropped: base channel impairment\n";
        return;
    }

    // ── Queue for air-interface delay ─────────────────────────
    double airDelay = truncnormal(pc5AirDelayMean_, pc5AirDelayStd_);
    if (airDelay < 0.0005) airDelay = 0.0005;

    stat_rx_delayed_++;

    cMessage* timer = new cMessage("pc5AirTimer");
    Pc5Msg pm;
    pm.frame = frame->dup();
    pm.timeReceived = simTime().dbl();
    pendingPc5_[timer] = pm;

    scheduleAt(simTime() + airDelay, timer);
    EV_DETAIL << "[Car5G] Queued, air delay=" << airDelay * 1000 << "ms\n";
}

// ─────────────────────────────────────────────────────────────
//  onPc5Delivery — default: forward to TraCIDemo11p::onWSM
// ─────────────────────────────────────────────────────────────
void Car5G::onPc5Delivery(BaseFrame1609_4* frame)
{
    TraCIDemo11p::onWSM(frame);
}

// ─────────────────────────────────────────────────────────────
//  handleSelfMsg
// ─────────────────────────────────────────────────────────────
void Car5G::handleSelfMsg(cMessage* msg)
{
    auto it = pendingPc5_.find(msg);
    if (it != pendingPc5_.end()) {
        BaseFrame1609_4* frame = it->second.frame;
        pendingPc5_.erase(it);
        delete msg;

        onPc5Delivery(frame);
        delete frame;
        return;
    }

    TraCIDemo11p::handleSelfMsg(msg);
}
