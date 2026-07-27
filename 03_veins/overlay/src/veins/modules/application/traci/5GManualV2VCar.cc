// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  5GManualV2VCar.cc  â€”  Mixed-traffic 5G V2V+V2I vehicle (Baseline)
//
//  KEY DESIGN: CAVs respond ONLY to SPaT received over the network.
//  SUMO TLS enforcement is disabled for veh_av at spawn.
//  If SPaT is missed (flooding/drop/stale), the vehicle holds its current
//  speed â€” it does NOT fall back to SUMO's TLS logic.
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

#include "5GManualV2VCar.h"
#include "veins/modules/application/traci/RunMetadata.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include <iomanip>
#include <cmath>
#include <memory>
#include <set>
#include <tuple>

using namespace veins;

Define_Module(veins::FiveGManualV2VCar);

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  Shared cam_response.csv
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
static std::ofstream s_v2vCamFile;
static bool          s_v2vCamFileReady = false;
static std::ofstream s_groundTruthJsonFile;
static bool          s_groundTruthJsonReady = false;
static std::ofstream s_vehicleJsonMapFile;
static bool          s_vehicleJsonMapReady = false;
static std::string   s_schemaJsonRunId;
static std::map<std::string, std::unique_ptr<std::ofstream>> s_vehicleJsonFiles;
static std::set<std::string> s_vehicleJsonMapped;

extern std::ofstream s_spatResponseFile;
extern const std::string& getRunOutputDir();
extern double getLoggedStepPdr(bool isV2V, double t);
extern long getLoggedStepAttempts(bool isV2V, double t);
extern long getLoggedStepDeliveries(bool isV2V, double t);
extern void recordPdrDelivery(bool isV2V, double t);

static double lookupLeaderSpeed(TraCIMobility* mobility, const std::string& leaderId)
{
    if (!mobility || leaderId.empty()) return 0.0;

    try {
        return mobility->getCommandInterface()->vehicle(leaderId).getSpeed();
    } catch (...) {
        return 0.0;
    }
}

static char normalizeLaneSignal(char rawSignal)
{
    return SpatPayload::normalizeSignal(rawSignal);
}

static void resetSchemaJsonOutputs()
{
    if (s_groundTruthJsonFile.is_open()) s_groundTruthJsonFile.close();
    if (s_vehicleJsonMapFile.is_open()) s_vehicleJsonMapFile.close();
    for (auto& kv : s_vehicleJsonFiles) {
        if (kv.second && kv.second->is_open()) kv.second->close();
    }
    s_vehicleJsonFiles.clear();
    s_vehicleJsonMapped.clear();
    s_groundTruthJsonReady = false;
    s_vehicleJsonMapReady = false;
}

static void ensureSchemaJsonRunContext()
{
    std::string currentRunId = getCurrentRunId();
    if (currentRunId == s_schemaJsonRunId) return;

    resetSchemaJsonOutputs();
    s_schemaJsonRunId = currentRunId;
}

static void ensureGroundTruthJsonFile()
{
    ensureSchemaJsonRunContext();
    if (s_groundTruthJsonReady) return;

    s_groundTruthJsonReady = true;
    s_groundTruthJsonFile.open(getRunOutputDir() + "GroundTruthJSONlog.json");
}

static void ensureVehicleJsonMapFile()
{
    ensureSchemaJsonRunContext();
    if (s_vehicleJsonMapReady) return;

    s_vehicleJsonMapReady = true;
    s_vehicleJsonMapFile.open(getRunOutputDir() + "JSONlog_vehicle_map.csv");
    if (s_vehicleJsonMapFile.is_open()) {
        s_vehicleJsonMapFile
            << "vehicle_index,"
            << "obu_device_id,"
            << "attacker_flag,"
            << "vehicle_external_id,"
            << "vehicle_type,"
            << "node_module_id,"
            << "app_module_id\n";
        s_vehicleJsonMapFile.flush();
    }
}

static std::ofstream* ensureVehicleJsonFile(const std::string& fileName,
                                            int vehicleIndex,
                                            int obuId,
                                            int attackerFlag,
                                            const std::string& vehicleExternalId,
                                            const std::string& vehicleType,
                                            int nodeModuleId,
                                            int appModuleId)
{
    ensureSchemaJsonRunContext();
    ensureVehicleJsonMapFile();

    auto it = s_vehicleJsonFiles.find(fileName);
    if (it != s_vehicleJsonFiles.end()) return it->second.get();

    auto stream = std::make_unique<std::ofstream>(getRunOutputDir() + fileName);
    std::ofstream* raw = stream.get();
    s_vehicleJsonFiles[fileName] = std::move(stream);

    if (s_vehicleJsonMapFile.is_open() && s_vehicleJsonMapped.insert(fileName).second) {
        s_vehicleJsonMapFile
            << vehicleIndex << ","
            << obuId << ","
            << attackerFlag << ","
            << vehicleExternalId << ","
            << vehicleType << ","
            << nodeModuleId << ","
            << appModuleId << "\n";
        s_vehicleJsonMapFile.flush();
    }

    return raw;
}

static int lookupSchemaVehicleIndexByExternalId(const std::string& externalId)
{
    if (externalId.empty()) return -1;

    auto* mgr = TraCIScenarioManagerAccess().get();
    if (!mgr) return -1;

    for (const auto& kv : mgr->getManagedHosts()) {
        cModule* node = kv.second;
        if (!node) continue;

        auto* hostMobility = dynamic_cast<TraCIMobility*>(node->getSubmodule("veinsmobility"));
        if (!hostMobility) hostMobility = dynamic_cast<TraCIMobility*>(node->getSubmodule("mobility"));
        if (!hostMobility) continue;

        try {
            if (hostMobility->getExternalId() == externalId) {
                return (node->getIndex() >= 0) ? node->getIndex() : node->getId();
            }
        } catch (...) {
        }
    }

    return -1;
}

static double estimateSchemaRssiWatts(double distanceMeters,
                                      double txPowerDbm,
                                      double referenceLossDb,
                                      double pathLossExponent)
{
    double d = std::max(1.0, distanceMeters);
    double recvPowerDbm = txPowerDbm - (referenceLossDb + 10.0 * pathLossExponent * std::log10(d));
    return std::pow(10.0, recvPowerDbm / 10.0) / 1000.0;
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  initialize
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
void FiveGManualV2VCar::initialize(int stage)
{
    Car5G::initialize(stage);

    if (stage == 0) {
        manualVehicleType_ = par("manualVehicleType").stdstringValue();
        cavVehicleType_    = par("cavVehicleType").stdstringValue();
        authModeLabel_     = par("authModeLabel").stdstringValue();
        logGroundtruth_    = par("logGroundtruth").boolValue();
        if (authModeLabel_.empty()) authModeLabel_ = "BASELINE_5G";
        beaconInterval_    = par("beaconInterval").doubleValue();
        leaderCamMaxAge_   = par("leaderCamMaxAge").doubleValue();
        beaconRange_       = par("beaconRange").doubleValue();
        useBeaconGap_      = par("useBeaconGap").boolValue();
        spatStaleTimeout_  = par("spatStaleTimeout").doubleValue();
        authDelayMean_     = par("authDelayMean").doubleValue();
        authDelayStd_      = par("authDelayStd").doubleValue();
        authCacheWindow_   = par("authCacheWindow").doubleValue();
        authFreshnessWindow_ = par("authFreshnessWindow").doubleValue();
    }

    if (stage == 1) {
        try { sumoVehicleType_ = traciVehicle->getTypeId(); }
        catch (...) { sumoVehicleType_ = "unknown"; }
        manualMode_ = (sumoVehicleType_ != cavVehicleType_);

        // â”€â”€ Cache vehicle ID immediately â€” TraCI is live at stage 1 â”€â”€
        // This ensures cachedVehicleId_ is populated before the first
        // SPaT arrives (which can happen as early as t=0.1s, before
        // handlePositionUpdate fires at t=updateInterval=1s).
        try { cachedVehicleId_ = mobility->getExternalId(); }
        catch (...) { cachedVehicleId_ = "unknown"; }

        if (!manualMode_) {
            // â”€â”€ Capture road speed limit at spawn â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            // veh_av has no explicit maxSpeed in vType â€” SUMO uses the
            // road speed limit. We read it here via getMaxSpeed() which
            // at spawn reflects the vType limit or road limit (whichever
            // is lower). Clamp to [1.0, 55.0] m/s for SUMO safety.
            // 55 m/s = 198 km/h â€” well above any urban road limit.
            try {
                double raw = traciVehicle->getMaxSpeed();
                normalMaxSpeed_ = (raw > 0.5 && raw < 200.0) ? raw : 13.9;
            } catch (...) {
                normalMaxSpeed_ = 13.9;  // 50 km/h default
            }
            EV_INFO << "[5GManualV2VCar] " << cachedVehicleId_
                    << " normalMaxSpeed_=" << normalMaxSpeed_ << " m/s\n";

            // â”€â”€ Disable SUMO TLS enforcement for this CAV â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
            // CAV_SPEED_MODE = 0b10100 = 20:
            //   clears bit 2 (TLS enforcement) â€” SUMO won't stop for red lights
            //   keeps bit 4 (slow for stopped cars) â€” collision avoidance preserved
            try {
                traciVehicle->setSpeedMode(CAV_SPEED_MODE);
                EV_INFO << "[5GManualV2VCar] " << cachedVehicleId_
                        << " â€” SUMO TLS enforcement DISABLED. SPaT-only control.\n";
            } catch (...) {
                EV_WARN << "[5GManualV2VCar] WARNING: could not set speedMode\n";
            }

            initCamResponseFile();

            double offset = uniform(0.0, beaconInterval_);
            camTimer_ = new cMessage("camTimer");
            scheduleAt(simTime() + offset, camTimer_);

            EV_INFO << "[5GManualV2VCar] vType=" << sumoVehicleType_
                    << " â†’ CAV (V2I+V2V, SPaT-only speed control)\n";
        } else {
            EV_INFO << "[5GManualV2VCar] vType=" << sumoVehicleType_
                    << " â†’ MANUAL (SUMO-only, no V2X)\n";
        }
    }
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  finish
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
void FiveGManualV2VCar::finish()
{
    isVehicleActive_ = false;

    if (camTimer_) { cancelAndDelete(camTimer_); camTimer_ = nullptr; }

    for (auto& kv : pendingCamAuth_) {
        cancelAndDelete(kv.first);
    }
    pendingCamAuth_.clear();

    for (auto& kv : pendingSpatAuth_) {
        cancelAndDelete(kv.first);
    }
    pendingSpatAuth_.clear();
    authCacheExpiry_.clear();

    if (!manualMode_) {
        EV_INFO << "[5GManualV2VCar] V2V stats:"
                << " cam_tx=" << stat_cam_tx_
                << " cam_rx=" << stat_cam_rx_
                << " cam_rx_leader=" << stat_cam_rx_leader_
                << " cam_dropped=" << stat_cam_dropped_
                << " spat_rx=" << stat_spat_rx_
                << " spat_stale=" << stat_spat_stale_
                << " auth_full=" << stat_auth_full_
                << " auth_cachehit=" << stat_auth_cachehit_
                << " auth_stale_drop=" << stat_auth_stale_drop_ << "\n";
    }

    Car5G::finish();
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  initCamResponseFile
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
void FiveGManualV2VCar::initCamResponseFile()
{
    if (s_v2vCamFileReady) return;
    s_v2vCamFileReady = true;

    s_v2vCamFile.open(getRunOutputDir() + "cam_response.csv");
    if (s_v2vCamFile.is_open()) {
        s_v2vCamFile
            << "run_number,seed_set,run_id,"
            << "vehicle_id,sender_id,"
            << "time_sent_s,time_received_s,time_acted_s,"
            << "prop_delay_s,auth_delay_s,"
            << "v2v_gap_m,involves_cav,auth_reason,"
            << "network_step_v2v_attempts,"
            << "network_step_v2v_deliveries,"
            << "network_step_v2v_pdr\n";
        s_v2vCamFile.flush();
    }
}

bool FiveGManualV2VCar::isSchemaAttacker() const
{
    return false;
}

int FiveGManualV2VCar::getSchemaVehicleIndex() const
{
    cModule* node = getParentModule();
    if (!node) return -1;
    return (node->getIndex() >= 0) ? node->getIndex() : node->getId();
}

int FiveGManualV2VCar::getSchemaObuId() const
{
    return getId();
}

void FiveGManualV2VCar::logSchemaSelfState(double timeReceived)
{
    if (!logGroundtruth_) return;
    cModule* node = getParentModule();
    if (!node) return;

    int vehicleIndex = getSchemaVehicleIndex();
    int obuId = getSchemaObuId();
    int attackerFlag = isSchemaAttacker() ? 1 : 0;

    std::ostringstream fileName;
    fileName << "JSONlog-" << vehicleIndex << "-" << obuId << "-A" << attackerFlag << ".json";

    std::ofstream* file = ensureVehicleJsonFile(fileName.str(),
                                                vehicleIndex,
                                                obuId,
                                                attackerFlag,
                                                cachedVehicleId_,
                                                sumoVehicleType_,
                                                node->getId(),
                                                getId());
    if (!file || !file->is_open()) return;

    *file << std::fixed << std::setprecision(12)
          << "{\"type\":2,"
          << "\"rcvTime\":" << timeReceived << ","
          << "\"pos\":[" << cachedMyX_ << "," << cachedMyY_ << "," << cachedMyZ_ << "],"
          << "\"noise\":[0.0,0.0,0.0],"
          << "\"spd\":[" << cachedMyVelX_ << "," << cachedMyVelY_ << "," << cachedMyVelZ_ << "],"
          << "\"spd_noise\":[0.0,0.0,0.0]}\n";
    file->flush();
}

void FiveGManualV2VCar::logSchemaGroundTruth(const BeaconPayload& cam) const
{
    if (!logGroundtruth_) return;
    ensureGroundTruthJsonFile();
    if (!s_groundTruthJsonFile.is_open()) return;

    int senderIndex = getSchemaVehicleIndex();
    int attackerFlag = isSchemaAttacker() ? 1 : 0;

    s_groundTruthJsonFile << std::fixed << std::setprecision(12)
                          << "{\"type\":4,"
                          << "\"time\":" << cam.timeSent_s << ","
                          << "\"sender\":" << senderIndex << ","
                          << "\"attackerType\":" << attackerFlag << ","
                          << "\"messageID\":" << cam.messageId << ","
                          << "\"pos\":[" << cam.posX_m << "," << cam.posY_m << "," << cam.posZ_m << "],"
                          << "\"pos_noise\":[0.0,0.0,0.0],"
                          << "\"spd\":[" << cam.velX_mps << "," << cam.velY_mps << "," << cam.velZ_mps << "],"
                          << "\"spd_noise\":[0.0,0.0,0.0]}\n";
    s_groundTruthJsonFile.flush();
}

void FiveGManualV2VCar::logSchemaReceivedCam(const BeaconPayload& cam, double timeReceived, double rssiWatts) const
{
    if (!logGroundtruth_) return;
    cModule* node = getParentModule();
    if (!node) return;

    int vehicleIndex = getSchemaVehicleIndex();
    int obuId = getSchemaObuId();
    int attackerFlag = isSchemaAttacker() ? 1 : 0;
    int senderIndex = lookupSchemaVehicleIndexByExternalId(cam.senderId);

    std::ostringstream fileName;
    fileName << "JSONlog-" << vehicleIndex << "-" << obuId << "-A" << attackerFlag << ".json";

    std::ofstream* file = ensureVehicleJsonFile(fileName.str(),
                                                vehicleIndex,
                                                obuId,
                                                attackerFlag,
                                                cachedVehicleId_,
                                                sumoVehicleType_,
                                                node->getId(),
                                                getId());
    if (!file || !file->is_open()) return;

    *file << std::fixed << std::setprecision(12)
          << "{\"type\":3,"
          << "\"rcvTime\":" << timeReceived << ","
          << "\"sendTime\":" << cam.timeSent_s << ","
          << "\"sender\":" << senderIndex << ","
          << "\"messageID\":" << cam.messageId << ","
          << "\"pos\":[" << cam.posX_m << "," << cam.posY_m << "," << cam.posZ_m << "],"
          << "\"pos_noise\":[0.0,0.0,0.0],"
          << "\"spd\":[" << cam.velX_mps << "," << cam.velY_mps << "," << cam.velZ_mps << "],"
          << "\"spd_noise\":[0.0,0.0,0.0],"
          << "\"RSSI\":" << rssiWatts << "}\n";
    file->flush();
}

double FiveGManualV2VCar::sampleAuthDelay()
{
    if (authDelayMean_ <= 0.0 && authDelayStd_ <= 0.0) return 0.0;

    double delay = truncnormal(authDelayMean_, authDelayStd_);
    if (!std::isfinite(delay) || delay < 0.0) delay = 0.0;
    return delay;
}

bool FiveGManualV2VCar::isMessageStale(double timeSent, double decisionTime) const
{
    if (authFreshnessWindow_ <= 0.0) return false;
    if (!std::isfinite(timeSent) || !std::isfinite(decisionTime)) return false;
    return (decisionTime - timeSent) > authFreshnessWindow_;
}

bool FiveGManualV2VCar::hasValidAuthCache(const std::string& cacheKey) const
{
    if (authCacheWindow_ <= 0.0 || cacheKey.empty()) return false;

    auto it = authCacheExpiry_.find(cacheKey);
    if (it == authCacheExpiry_.end()) return false;
    return simTime().dbl() <= it->second;
}

void FiveGManualV2VCar::updateAuthCache(const std::string& cacheKey, double timeActed)
{
    if (authCacheWindow_ <= 0.0 || cacheKey.empty()) return;
    authCacheExpiry_[cacheKey] = timeActed + authCacheWindow_;
}

std::string FiveGManualV2VCar::camAuthCacheKey(const std::string& senderId) const
{
    return senderId.empty() ? std::string() : "CAM:" + senderId;
}

std::string FiveGManualV2VCar::spatAuthCacheKey(const std::string& tlsId) const
{
    return tlsId.empty() ? std::string() : "SPAT:" + tlsId;
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  onWSM  â€” CAVs: hand to Car5G PC5 pipeline. Manual: discard silently.
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
void FiveGManualV2VCar::onWSM(BaseFrame1609_4* frame)
{
    if (manualMode_) return;  // no V2X OBU â€” DO NOT delete, framework owns
    Car5G::onWSM(frame);
}

void FiveGManualV2VCar::onWSA(DemoServiceAdvertisment* wsa)
{
    if (manualMode_) return;
    Car5G::onWSA(wsa);
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  onPc5Delivery
//  Called by Car5G AFTER all PC5 drops + air-delay timer.
//  Routes: CAM â†’ applyBeacon(), SPaT â†’ applySpatSpeedControl()
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
void FiveGManualV2VCar::onPc5Delivery(BaseFrame1609_4* frame)
{
    if (!isVehicleActive_) return;

    auto* wsm = dynamic_cast<TraCIDemo11pMessage*>(frame);
    if (!wsm) return;

    std::string data        = wsm->getDemoData();
    double      timeReceived = simTime().dbl();

    // â”€â”€ V2V CAM path â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    BeaconPayload cam;
    if (BeaconPayload::parse(data, cam)) {
        stat_cam_rx_++;
        if (cam.senderId == cachedVehicleId_) { stat_cam_dropped_++; return; }
        double senderDistance = cam.distanceTo(cachedMyX_, cachedMyY_);
        if (senderDistance > beaconRange_)
            { stat_cam_dropped_++; return; }

        double rssiWatts = estimateSchemaRssiWatts(senderDistance,
                                                   pc5TxPowerDbm_,
                                                   pc5ReferenceLossDb_,
                                                   pc5PathLossExponent_);
        logSchemaReceivedCam(cam, timeReceived, rssiWatts);

        std::string cacheKey = camAuthCacheKey(cam.senderId);
        if (hasValidAuthCache(cacheKey)) {
            stat_auth_cachehit_++;
            if (isMessageStale(cam.timeSent_s, timeReceived)) {
                stat_auth_stale_drop_++;
                writeCamRow(cam.senderId, cam.timeSent_s, timeReceived, timeReceived,
                            -1.0, computeInvolvesCav(cam.senderId),
                            camCachedAuthReason() + "_STALE_DROP");
                return;
            }
            applyBeacon(cam, timeReceived, timeReceived, camCachedAuthReason());
            return;
        }

        stat_auth_full_++;
        double authDelay = sampleAuthDelay();
        if (authDelay <= 0.0) {
            if (isMessageStale(cam.timeSent_s, timeReceived)) {
                stat_auth_stale_drop_++;
                writeCamRow(cam.senderId, cam.timeSent_s, timeReceived, timeReceived,
                            -1.0, computeInvolvesCav(cam.senderId),
                            camFreshAuthReason() + "_STALE_DROP");
                return;
            }
            updateAuthCache(cacheKey, timeReceived);
            applyBeacon(cam, timeReceived, timeReceived, camFreshAuthReason());
            return;
        }

        cMessage* timer = new cMessage("camAuthTimer");
        PendingCamAuth pending;
        pending.cam = cam;
        pending.timeReceived = timeReceived;
        pending.configuredDelay = authDelay;
        pending.authReason = camFreshAuthReason();
        pending.cacheKey = cacheKey;
        pendingCamAuth_[timer] = pending;
        scheduleAt(simTime() + authDelay, timer);
        return;
    }

    // â”€â”€ V2I SPaT path â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    SpatPayload spat;
    if (SpatPayload::parse(data, spat)) {
        SpatPayload vehicleSpat = resolveSpatForThisVehicle(spat);
        if (vehicleSpat.phase == 'U') return;

        stat_spat_rx_++;
        double mySpeed = 0.0;
        try { mySpeed = mobility->getSpeed(); } catch (...) {}
        double timeSent = spat.timeSent;

        std::string cacheKey = spatAuthCacheKey(vehicleSpat.tlsId);
        if (hasValidAuthCache(cacheKey)) {
            stat_auth_cachehit_++;
            bool staleDrop = isMessageStale(vehicleSpat.timeSent, timeReceived);
            if (staleDrop) {
                stat_auth_stale_drop_++;
            }
            else {
                recordPdrDelivery(false, timeReceived);
                applySpatSpeedControl(vehicleSpat);
            }

            if (s_spatResponseFile.is_open()) {
                long stepAttempts = getLoggedStepAttempts(false, timeReceived);
                long stepDeliveries = getLoggedStepDeliveries(false, timeReceived);
                double stepPdr = getLoggedStepPdr(false, timeReceived);
                s_spatResponseFile << std::fixed << std::setprecision(6)
                    << getCurrentRunCsvPrefix()
                    << cachedVehicleId_ << ","
                    << vehicleSpat.tlsId << ","
                    << vehicleSpat.phase << ","
                    << vehicleSpat.timeToChange << ","
                    << timeSent         << ","
                    << timeReceived     << ","
                    << timeReceived     << ","
                    << mySpeed          << ","
                    << 0.0              << ","
                    << 0.0              << ","
                    << 0.0              << ","
                    << (staleDrop ? spatCachedAuthReason() + "_STALE_DROP"
                                  : spatCachedAuthReason()) << ","
                    << stepAttempts      << ","
                    << stepDeliveries    << ","
                    << stepPdr           << "\n";
                s_spatResponseFile.flush();
            }
            return;
        }

        stat_auth_full_++;
        double authDelay = sampleAuthDelay();
        if (authDelay <= 0.0) {
            bool staleDrop = isMessageStale(vehicleSpat.timeSent, timeReceived);
            if (staleDrop) {
                stat_auth_stale_drop_++;
            }
            else {
                updateAuthCache(cacheKey, timeReceived);
                recordPdrDelivery(false, timeReceived);
                applySpatSpeedControl(vehicleSpat);
            }

            if (s_spatResponseFile.is_open()) {
                long stepAttempts = getLoggedStepAttempts(false, timeReceived);
                long stepDeliveries = getLoggedStepDeliveries(false, timeReceived);
                double stepPdr = getLoggedStepPdr(false, timeReceived);
                s_spatResponseFile << std::fixed << std::setprecision(6)
                    << getCurrentRunCsvPrefix()
                    << cachedVehicleId_ << ","
                    << vehicleSpat.tlsId << ","
                    << vehicleSpat.phase << ","
                    << vehicleSpat.timeToChange << ","
                    << timeSent         << ","
                    << timeReceived     << ","
                    << timeReceived     << ","
                    << mySpeed          << ","
                    << 0.0              << ","
                    << 0.0              << ","
                    << 0.0              << ","
                    << (staleDrop ? spatFreshAuthReason() + "_STALE_DROP"
                                  : spatFreshAuthReason()) << ","
                    << stepAttempts      << ","
                    << stepDeliveries    << ","
                    << stepPdr           << "\n";
                s_spatResponseFile.flush();
            }
            return;
        }

        cMessage* timer = new cMessage("spatAuthTimer");
        PendingSpatAuth pending;
        pending.spat = vehicleSpat;
        pending.timeReceived = timeReceived;
        pending.speedAtReceipt = mySpeed;
        pending.configuredDelay = authDelay;
        pending.authReason = spatFreshAuthReason();
        pending.cacheKey = cacheKey;
        pendingSpatAuth_[timer] = pending;
        scheduleAt(simTime() + authDelay, timer);
        return;
    }

    EV_DETAIL << "[5GManualV2VCar] onPc5Delivery: unrecognised payload\n";
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  setSafeMaxSpeed  â€” single validated path for all setMaxSpeed calls
//
//  Validates the value, skips junction internal edges, catches cRuntimeError.
//  All setMaxSpeed calls in this module go through this function.
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
void FiveGManualV2VCar::setSafeMaxSpeed(double speed)
{
    if (!isVehicleActive_) return;

    // Clamp to SUMO-valid range: [0.01, 300] m/s
    // 0.0 is rejected on some vehicle states; negative values always rejected
    if (speed < 0.0 || std::isnan(speed) || std::isinf(speed)) speed = 0.01;
    if (speed < 0.01)  speed = 0.01;
    if (speed > 300.0) speed = normalMaxSpeed_ > 0 ? normalMaxSpeed_ : 13.9;

    // Skip if vehicle is on an internal junction edge (road id starts with ':')
    // SUMO rejects setMaxSpeed on internal edges
    try {
        std::string roadId = mobility->getRoadId();
        if (!roadId.empty() && roadId[0] == ':') {
            EV_DETAIL << "[5GManualV2VCar] setSafeMaxSpeed skipped - inside junction\n";
            return;
        }
    } catch (...) { return; }

    try {
        traciVehicle->setMaxSpeed(speed);
    } catch (const cRuntimeError& e) {
        // SUMO rejected â€” log and continue without crashing
        EV_WARN << "[5GManualV2VCar] setSafeMaxSpeed(" << speed
                << ") rejected: " << e.what() << "\n";
    } catch (...) {
        EV_WARN << "[5GManualV2VCar] setSafeMaxSpeed: TraCI call failed\n";
    }
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  applySpatSpeedControl
//
//  The ONLY place where speed commands are issued to SUMO for CAVs.
//  SUMO TLS enforcement was disabled at spawn â€” this function IS the TLS.
//
//  RED / YELLOW: setMaxSpeed(0.01)  â†’ IDM decelerates to near-stop naturally
//                0.01 m/s used instead of 0 â€” setMaxSpeed(0) is rejected by
//                SUMO in certain vehicle states (junction, lane change etc).
//
//  GREEN:        setMaxSpeed(-1)    â†’ removes override entirely
//                SUMO uses min(vType.maxSpeed, lane.speedLimit)
//                This is ALWAYS valid regardless of vehicle state.
//                IDM then accelerates naturally up to that limit.
//
//  UNKNOWN (U):  no action â€” preserve current setMaxSpeed state.
//
//  We do NOT call setSpeed() â€” only setMaxSpeed().
//  setMaxSpeed() sets a ceiling; IDM handles the trajectory profile.
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
SpatPayload FiveGManualV2VCar::resolveSpatForThisVehicle(const SpatPayload& spat) const
{
    SpatPayload resolved = spat;
    resolved.phase = 'U';

    if (!traciVehicle) return resolved;

    try {
        for (const auto& nextTls : traciVehicle->getNextTls()) {
            if (std::get<0>(nextTls) != spat.tlsId) continue;

            int linkIndex = std::get<1>(nextTls);
            char nextTlsSignal = static_cast<char>(std::get<3>(nextTls));

            if (!spat.state.empty() &&
                linkIndex >= 0 &&
                linkIndex < static_cast<int>(spat.state.size())) {
                resolved.phase = normalizeLaneSignal(spat.state[static_cast<size_t>(linkIndex)]);
            }
            else {
                resolved.phase = normalizeLaneSignal(nextTlsSignal);
            }
            return resolved;
        }
    }
    catch (...) {
    }

    return resolved;
}

void FiveGManualV2VCar::applySpatSpeedControl(const SpatPayload& spat)
{
    if (!isVehicleActive_) return;
    if (normalMaxSpeed_ <= 0.0) return;

    SpatPayload vehicleSpat = resolveSpatForThisVehicle(spat);
    if (vehicleSpat.phase == 'U') return;

    lastSpatReceivedAt_ = simTime().dbl();
    lastSpatPhase_      = vehicleSpat.phase;
    spatControlActive_  = true;
    frozenSpeed_        = -1.0;

    if (vehicleSpat.isGreen()) {
        setSafeMaxSpeed(normalMaxSpeed_);
        return;
    }

    if (vehicleSpat.isRed() || vehicleSpat.isYellow()) {
        setSafeMaxSpeed(0.01);
    }
}


// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  checkSpatStaleness
//  Called every position update for CAVs.
//
//  If no SPaT received for > spatStaleTimeout_ seconds:
//    â†’ freeze current speed via setMaxSpeed(currentSpeed_)
//    â†’ if already stopped: setMaxSpeed(0) â€” stay stopped
//    â†’ vehicle holds this speed until next SPaT arrives
//    â†’ SUMO TLS does NOT intervene (disabled at spawn)
//
//  This is the flooding consequence: loss of comms = frozen speed.
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
void FiveGManualV2VCar::checkSpatStaleness()
{
    // SPaT staleness tracking â€” log only, no TraCI speed commands.
    // setMaxSpeed (TraCI 0xc4) is rejected by SUMO when called on departing
    // vehicles in OMNeT++ release mode and cannot be caught before the
    // simulation terminates. Speed control is left entirely to SUMO.
    // The flooding safety impact is captured via spat_response.csv delivery
    // rate â€” whether the vehicle received the SPaT â€” not speed override.
    if (!spatControlActive_) return;
    if (!isVehicleActive_) return;

    double elapsed = simTime().dbl() - lastSpatReceivedAt_;
    if (elapsed > spatStaleTimeout_) {
        stat_spat_stale_++;
        EV_WARN << "[5GManualV2VCar] SPaT STALE (" << elapsed*1000
                << "ms) â€” no speed command issued (SUMO controls speed)\n";
    }
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  handleSelfMsg
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
void FiveGManualV2VCar::handleSelfMsg(cMessage* msg)
{
    {
        auto it = pendingCamAuth_.find(msg);
        if (it != pendingCamAuth_.end()) {
            PendingCamAuth pending = it->second;
            pendingCamAuth_.erase(it);

            if (!isVehicleActive_) {
                delete msg;
                return;
            }

            double timeActed = simTime().dbl();
            if (isMessageStale(pending.cam.timeSent_s, timeActed)) {
                stat_auth_stale_drop_++;
                writeCamRow(pending.cam.senderId, pending.cam.timeSent_s,
                            pending.timeReceived, timeActed, -1.0,
                            computeInvolvesCav(pending.cam.senderId),
                            pending.authReason + "_STALE_DROP");
            }
            else {
                updateAuthCache(pending.cacheKey, timeActed);
                applyBeacon(pending.cam, pending.timeReceived, timeActed, pending.authReason);
            }
            delete msg;
            return;
        }
    }

    {
        auto it = pendingSpatAuth_.find(msg);
        if (it != pendingSpatAuth_.end()) {
            PendingSpatAuth pending = it->second;
            pendingSpatAuth_.erase(it);

            if (!isVehicleActive_) {
                delete msg;
                return;
            }

            double timeActed = simTime().dbl();
            double authDelay = timeActed - pending.timeReceived;
            double extraDist = pending.speedAtReceipt * authDelay;
            bool staleDrop = isMessageStale(pending.spat.timeSent, timeActed);
            if (staleDrop) {
                stat_auth_stale_drop_++;
            }
            else {
                updateAuthCache(pending.cacheKey, timeActed);
                recordPdrDelivery(false, pending.timeReceived);
                applySpatSpeedControl(pending.spat);
            }

            if (s_spatResponseFile.is_open()) {
                long stepAttempts = getLoggedStepAttempts(false, pending.timeReceived);
                long stepDeliveries = getLoggedStepDeliveries(false, pending.timeReceived);
                double stepPdr = getLoggedStepPdr(false, pending.timeReceived);
                s_spatResponseFile << std::fixed << std::setprecision(6)
                    << getCurrentRunCsvPrefix()
                    << cachedVehicleId_ << ","
                    << pending.spat.tlsId << ","
                    << pending.spat.phase << ","
                    << pending.spat.timeToChange << ","
                    << pending.spat.timeSent << ","
                    << pending.timeReceived << ","
                    << timeActed           << ","
                    << pending.speedAtReceipt << ","
                    << pending.configuredDelay << ","
                    << authDelay           << ","
                    << extraDist           << ","
                    << (staleDrop ? pending.authReason + "_STALE_DROP"
                                  : pending.authReason) << ","
                    << stepAttempts        << ","
                    << stepDeliveries      << ","
                    << stepPdr             << "\n";
                s_spatResponseFile.flush();
            }

            delete msg;
            return;
        }
    }

    if (msg == camTimer_) {
        if (isVehicleActive_ && !manualMode_) {
            broadcastCam();
            scheduleAt(simTime() + beaconInterval_, camTimer_);
        }
        return;
    }
    Car5G::handleSelfMsg(msg);
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  handlePositionUpdate
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
void FiveGManualV2VCar::handlePositionUpdate(cObject* obj)
{
    if (manualMode_) {
        // â”€â”€ MANUAL â€” observe and log only, SUMO drives â”€â”€â”€â”€â”€â”€â”€
        DemoBaseApplLayer::handlePositionUpdate(obj);
        if (!isVehicleActive_) return;

        double t = simTime().dbl();
        if (cachedVehicleId_.empty()) {
            try { cachedVehicleId_ = mobility->getExternalId(); }
            catch (...) { return; }
        }

        double speed = 0.0;
        try { speed = mobility->getSpeed(); } catch (...) { return; }

        try {
            auto pos = mobility->getPositionAt(simTime());
            double dtVel = (simTime() - prevTime).dbl();
            if (dtVel > 0.0) {
                cachedMyVelX_ = (pos.x - cachedMyX_) / dtVel;
                cachedMyVelY_ = (pos.y - cachedMyY_) / dtVel;
                cachedMyVelZ_ = (pos.z - cachedMyZ_) / dtVel;
            }
            cachedMyX_ = pos.x;
            cachedMyY_ = pos.y;
            cachedMyZ_ = pos.z;
            cachedMySpeed_ = speed;
        } catch (...) {}

        double accel = 0.0;
        if (prevSpeed >= 0.0) {
            double dt = (simTime() - prevTime).dbl();
            if (dt > 0.0) accel = (speed - prevSpeed) / dt;
        }
        cachedMyAccel_ = accel;
        prevSpeed = speed; prevTime = simTime();

        double gap = -1.0; std::string leaderId;
        try {
            auto lp = getLeaderInfo();
            gap = lp.second;
            leaderId = (gap >= 0.0) ? lp.first : "";
        } catch (...) {}

        double leaderSpeed = lookupLeaderSpeed(mobility, leaderId);
        double ttc = -1.0;
        if (gap >= 0.0) {
            double raw = computeTTC(speed, leaderSpeed, gap);
            ttc = (raw >= 999.0) ? -1.0 : raw;
        }

        double cx = 0.0, cy = 0.0;
        try { auto p = mobility->getPositionAt(simTime()); cx=p.x; cy=p.y; }
        catch (...) {}

        int hb = 0, eb = 0;
        if (!inBrakingEpisode_) {
            if (accel <= -0.5 && speed > 0.1) {
                inBrakingEpisode_ = true; brakeEpisodeDist_ = 0.0;
                brakeEpisodeMaxDecel_ = accel;
                brakeLastX_ = cx; brakeLastY_ = cy;
                hardBrakeFired_ = false; emergencyBrakeFired_ = false;
            }
        } else {
            double dx = cx-brakeLastX_, dy = cy-brakeLastY_;
            brakeEpisodeDist_ += std::sqrt(dx*dx+dy*dy);
            brakeLastX_ = cx; brakeLastY_ = cy;
            if (accel < brakeEpisodeMaxDecel_) brakeEpisodeMaxDecel_ = accel;
            if (!hardBrakeFired_ && brakeEpisodeMaxDecel_ <= -4.0)
                { hardBrakeFired_ = true; hb = 1; }
            if (!emergencyBrakeFired_ && brakeEpisodeMaxDecel_ <= -7.0)
                { emergencyBrakeFired_ = true; eb = 1; }
            if (speed <= 0.1 || accel >= -0.2) inBrakingEpisode_ = false;
        }

        int col = 0;
        if (speed <= 0.1) {
            if (stoppedSince_ < 0) stoppedSince_ = simTime();
            if ((simTime()-stoppedSince_).dbl() >= 10.0
                && gap >= 0.0 && gap <= 1.0) { inCollision_ = true; col = 1; }
        } else { stoppedSince_ = -1; inCollision_ = false; }

        int egoIsCav = 0;
        int leaderIsCav = lookupLeaderIsCav(leaderId);
        int inv = leaderIsCav;
        int tlsViolationEvent = 0;
        int pedestrianCollisionEvent = 0;
        int cyclistCollisionEvent = 0;
        int activeTransportCollisionEvent = 0;
        updateTlsObservation(tlsViolationEvent);
        updateActiveTransportRisk(pedestrianCollisionEvent,
                                  cyclistCollisionEvent,
                                  activeTransportCollisionEvent);

        if (timeseriesFile && timeseriesFile->is_open()) {
            *timeseriesFile << std::fixed << std::setprecision(4)
                << getCurrentRunCsvPrefix()
                << t              << "," << cachedVehicleId_ << ","
                << egoIsCav       << "," << sumoVehicleType_ << ","
                << speed          << "," << accel << ","
                << (gap<0?-1.0:gap) << "," << leaderId << "," << leaderIsCav << "," << ttc << ","
                << (inBrakingEpisode_?brakeEpisodeDist_:0.0) << ","
                << hb << "," << eb << "," << col << ","
                << tlsViolationEvent << ","
                << pedestrianCollisionEvent << ","
                << cyclistCollisionEvent << ","
                << activeTransportCollisionEvent << ","
                << inv << "\n";
            timeseriesFile->flush();
        }
        logSchemaSelfState(t);
        if (speed >= 1.0) lastDroveAt = simTime();
        return;
    }

    // â”€â”€ CAV MODE â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // Guard: Veins may call handlePositionUpdate once more after finish()
    // sets isVehicleActive_=false (vehicle departing SUMO). Any TraCI call
    // at this point â€” including setMaxSpeed â€” returns status 255.
    if (!isVehicleActive_) return;

    // Cache own position for CAM TX
    try {
        double oldX = cachedMyX_;
        double oldY = cachedMyY_;
        double oldZ = cachedMyZ_;
        cachedMySpeed_ = mobility->getSpeed();
        auto pos = mobility->getPositionAt(simTime());
        cachedMyX_ = pos.x;
        cachedMyY_ = pos.y;
        cachedMyZ_ = pos.z;
        if (cachedVehicleId_.empty())
            cachedVehicleId_ = mobility->getExternalId();
        if (prevSpeed >= 0.0) {
            double dt = (simTime() - prevTime).dbl();
            if (dt > 0.0) {
                cachedMyAccel_ = (cachedMySpeed_ - prevSpeed) / dt;
                cachedMyVelX_ = (cachedMyX_ - oldX) / dt;
                cachedMyVelY_ = (cachedMyY_ - oldY) / dt;
                cachedMyVelZ_ = (cachedMyZ_ - oldZ) / dt;
            }
        }
    } catch (...) {}

    // Check SPaT staleness â€” freeze speed if comms lost
    checkSpatStaleness();

    // Update leader CAM age
    if (cachedLeaderCamAge_ < 1e9) cachedLeaderCamAge_ += 0.1;

    logSchemaSelfState(simTime().dbl());

    // Delegate to Car5G for timeseries logging and PC5 timers
    Car5G::handlePositionUpdate(obj);
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  broadcastCam
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
void FiveGManualV2VCar::broadcastCam()
{
    if (!isVehicleActive_) return;

    BeaconPayload cam;
    cam.senderId   = cachedVehicleId_;
    cam.messageId  = nextMessageId_++;
    cam.speed_mps  = cachedMySpeed_;
    cam.velX_mps   = cachedMyVelX_;
    cam.velY_mps   = cachedMyVelY_;
    cam.velZ_mps   = cachedMyVelZ_;
    cam.posX_m     = cachedMyX_;
    cam.posY_m     = cachedMyY_;
    cam.posZ_m     = cachedMyZ_;
    cam.accel_mps2 = cachedMyAccel_;
    cam.timeSent_s = simTime().dbl();

    auto* wsm = new TraCIDemo11pMessage();
    populateWSM(wsm);
    wsm->setDemoData(cam.encode().c_str());
    wsm->setSenderAddress(getParentModule()->getId());
    wsm->setSerial(0);

    sendDown(wsm);
    stat_cam_tx_++;
    logSchemaGroundTruth(cam);
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  applyBeacon
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
void FiveGManualV2VCar::applyBeacon(const BeaconPayload& cam,
                                     double timeReceived,
                                     double timeActed,
                                     const std::string& reason)
{
    if (!isVehicleActive_) return;
    recordPdrDelivery(true, timeReceived);

    std::string currentLeaderId;
    double sumoGap = -1.0;
    try {
        auto lp = getLeaderInfo();
        sumoGap = lp.second;
        currentLeaderId = (sumoGap >= 0.0) ? lp.first : "";
    } catch (...) {}

    double v2vGap = -1.0;
    if (cam.senderId == currentLeaderId && sumoGap >= 0.0) {
        double distFromCam = cam.distanceTo(cachedMyX_, cachedMyY_);
        v2vGap = std::max(0.0, distFromCam - 4.0);

        if (useBeaconGap_) {
            cachedLeaderId_     = cam.senderId;
            cachedLeaderGap_    = v2vGap;
            cachedLeaderSpeed_  = cam.speed_mps;
            cachedLeaderCamAge_ = timeActed - cam.timeSent_s;
        }
        stat_cam_rx_leader_++;
    }

    int inv = computeInvolvesCav(cam.senderId);
    writeCamRow(cam.senderId, cam.timeSent_s, timeReceived, timeActed,
                v2vGap, inv, reason);
}

std::string FiveGManualV2VCar::camFreshAuthReason() const
{
    return authModeLabel_ + "_V2V_AUTH";
}

std::string FiveGManualV2VCar::camCachedAuthReason() const
{
    return authModeLabel_ + "_V2V_CACHE_HIT";
}

std::string FiveGManualV2VCar::spatFreshAuthReason() const
{
    return authModeLabel_ + "_I2V_AUTH";
}

std::string FiveGManualV2VCar::spatCachedAuthReason() const
{
    return authModeLabel_ + "_I2V_CACHE_HIT";
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  computeInvolvesCav
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
int FiveGManualV2VCar::computeInvolvesCav(const std::string& otherId) const
{
    if (!manualMode_) return 1;
    if (!mobility || otherId.empty()) return 0;

    try {
        std::string otherType = mobility->getCommandInterface()->vehicle(otherId).getTypeId();
        return (otherType == cavVehicleType_) ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

int FiveGManualV2VCar::lookupLeaderIsCav(const std::string& leaderId) const
{
    if (!mobility || leaderId.empty()) return 0;

    try {
        std::string leaderType = mobility->getCommandInterface()->vehicle(leaderId).getTypeId();
        return (leaderType == cavVehicleType_) ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  writeCamRow
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
void FiveGManualV2VCar::writeCamRow(const std::string& senderId,
                                     double timeSent,
                                     double timeReceived,
                                     double timeActed,
                                     double v2vGap,
                                     int    involvesCav,
                                     const std::string& reason)
{
    if (!s_v2vCamFile.is_open()) return;
    long stepAttempts = getLoggedStepAttempts(true, timeReceived);
    long stepDeliveries = getLoggedStepDeliveries(true, timeReceived);
    double stepPdr = getLoggedStepPdr(true, timeReceived);

    s_v2vCamFile << std::fixed << std::setprecision(6)
        << getCurrentRunCsvPrefix()
        << cachedVehicleId_ << "," << senderId         << ","
        << timeSent         << "," << timeReceived      << ","
        << timeActed        << ","
        << (timeReceived - timeSent)   << ","  // prop_delay_s
        << (timeActed    - timeReceived) << "," // auth_delay_s
        << v2vGap           << ","
        << involvesCav      << ","
        << reason           << ","
        << stepAttempts     << ","
        << stepDeliveries   << ","
        << stepPdr          << "\n";
    s_v2vCamFile.flush();
}


