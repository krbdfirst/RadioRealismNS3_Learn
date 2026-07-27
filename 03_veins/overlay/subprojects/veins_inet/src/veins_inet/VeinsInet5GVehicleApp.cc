#include "veins_inet/VeinsInet5GVehicleApp.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <ctime>
#include <tuple>
#include <vector>
#include <string_view>
#include <sys/stat.h>
#include <limits>

#include "inet/common/packet/Packet.h"
#include "veins/base/utils/FindModule.h"
#include "veins/modules/application/traci/RunMetadata.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins_inet/VeinsInet5GMessage_m.h"

using namespace inet;

namespace {

constexpr double kMsPerSecond = 1000.0;
constexpr double kReferenceDistanceMeters = 1.0;
constexpr double kMinimumTraciStopSpeed = 0.01;
constexpr double kMaximumTraciSpeed = 55.0;
// safe_stop: come to rest this many metres before the stop line (real vehicles hold short of it).
constexpr double kStopLineMargin = 2.0;
constexpr double kMinimumFutureTimerOffsetSeconds = 0.001;
constexpr double kPdrTimeBinSeconds = 0.1;
constexpr double kBrakeStartThreshold = -0.5;
constexpr double kBrakeEndThreshold = -0.2;
constexpr double kHardBrakeThreshold = -4.0;
constexpr double kEmergencyBrakeThreshold = -7.0;
constexpr double kStopSpeedThreshold = 0.1;
// Speed below which an MRM counts as having reached the minimal risk CONDITION (standstill).
constexpr double kMrmRestSpeed = 0.5;
// Traffic conflict thresholds — Hydén (1987) / ETSI TR 103 415 v1.1.1 §6.3.2
// Each fires ONCE per episode (mirrors hard_brake_event / emergency_brake_event).
constexpr double kSeriousConflictTtcS = 1.5;  // serious conflict: TTC < 1.5 s
constexpr double kConflictTtcS        = 3.0;  // safety-critical interaction: TTC < 3.0 s
constexpr double kTlsViolationDistanceThreshold = 5.0;

std::ofstream s_groundTruthJsonFile;
bool s_groundTruthJsonReady = false;
std::ofstream s_vehicleJsonMapFile;
bool s_vehicleJsonMapReady = false;
std::string s_schemaJsonRunId;
std::map<std::string, std::unique_ptr<std::ofstream>> s_vehicleJsonFiles;
std::set<std::string> s_vehicleJsonMapped;
std::ofstream s_timeseriesFile;
bool s_timeseriesReady = false;

// ── safe_stop probe (env VEINS_SAFESTOP_PROBE): per-CAV distance-to-stop-line each tick,
// to eyeball WHERE vehicles come to rest under SPaT loss. Zero impact when env unset.
std::ofstream s_safeStopProbeFile;
bool s_safeStopProbeReady = false;
const bool s_safeStopProbeEnabled = (std::getenv("VEINS_SAFESTOP_PROBE") != nullptr);
std::ofstream s_spatResponseFile;
bool s_spatResponseReady = false;
std::ofstream s_spatAuthDecisionFile;
bool s_spatAuthDecisionReady = false;
std::ofstream s_latencyBreakdownFile;
bool s_latencyBreakdownReady = false;
std::ofstream s_cavLinkTraceFile;
bool s_cavLinkTraceReady = false;
std::set<std::tuple<long long, std::string, std::string>> s_cavLinkTraceSeen;
std::string s_csvRunId;
std::string s_runTimestamp;

struct PdrCounters {
    long attempts = 0;
    long deliveries = 0;
};

struct PdrStepCounters {
    PdrCounters v2v;
    PdrCounters v2i;
};

std::map<long long, PdrStepCounters> s_pdrSteps;

// Distance-bucketed PDR for calibration CSV output.
// Keyed by floor(distance_m / kDistBinWidthMeters) * kDistBinWidthMeters.
constexpr int kDistBinWidthMeters = 10;

struct DistPdrBin {
    long attempts  = 0;
    long delivered = 0;
    long drop_hd   = 0;
    long drop_prop = 0;
    long drop_sps  = 0;
    long drop_base = 0;
    long drop_cap  = 0;
};

std::map<int, DistPdrBin> s_distPdrBins;
bool s_distPdrWritten = false;

// ── OMNeT↔NS-3 flood-collision integration log (Step 8) ────────────────────────
// When env VEINS_FLOOD_COLL_LOG is set, ns3LearnReception records the REALIZED SB-SPS
// collision outcome per reception, keyed by (local CAV density n, attacker-in-range 0/1),
// matching NS-3's flood_validate_dataset.csv schema so the student's collision rate can be
// compared to the teacher's measured rate (not just the A/B direction).
struct FloodCollBin {
    long attempts   = 0;
    long collisions = 0;
};
std::map<std::pair<int, int>, FloodCollBin> s_floodColl;   // key {n, attackerInRange}
bool s_floodCollWritten = false;
const bool s_floodCollEnabled = (std::getenv("VEINS_FLOOD_COLL_LOG") != nullptr);
// In-sim periodic flush guard: dist_pdr is normally written in finish(), but a
// seed/traffic-dependent SIGBUS in module teardown (SUMO/INET) can abort finish()
// before non-zero seeds write their CSV. Flushing during the run keeps every seed.
omnetpp::simtime_t s_lastDistPdrFlush = SIMTIME_ZERO;

double clamp01(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

std::string normalizeProfileKey(std::string profile)
{
    std::transform(profile.begin(), profile.end(), profile.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return profile;
}

uint64_t fnv1aHash64(const std::string& text)
{
    uint64_t hash = 1469598103934665603ULL;
    for (unsigned char ch : text) {
        hash ^= static_cast<uint64_t>(ch);
        hash *= 1099511628211ULL;
    }
    return hash;
}

double uniformFromHash(uint64_t seed)
{
    constexpr double kScale = 1.0 / static_cast<double>(1ULL << 53);
    seed ^= seed >> 12;
    seed ^= seed << 25;
    seed ^= seed >> 27;
    const uint64_t mixed = seed * 2685821657736338717ULL;
    const double value = static_cast<double>((mixed >> 11) & ((1ULL << 53) - 1ULL)) * kScale;
    return std::max(1e-12, std::min(1.0 - 1e-12, value));
}

double gaussianFromHash(uint64_t seed)
{
    const double u1 = uniformFromHash(seed ^ 0x9E3779B97F4A7C15ULL);
    const double u2 = uniformFromHash(seed ^ 0xBF58476D1CE4E5B9ULL);
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * 3.14159265358979323846 * u2);
}

simtime_t safeFutureTime(simtime_t delay, simtime_t minimumDelay = SimTime(kMinimumFutureTimerOffsetSeconds))
{
    return simTime() + std::max(delay, minimumDelay);
}

simtime_t secondsToSimTime(double seconds)
{
    // Passing a double into SimTime(value, SIMTIME_S) selects the integer-unit
    // overload and truncates sub-second delays. Use the floating-point
    // constructor instead because all values here are already expressed in
    // seconds.
    return SimTime(seconds);
}

std::string ensureTrailingSlash(std::string path)
{
    if (!path.empty() && path.back() != '/') path += "/";
    return path;
}

std::string csvEscape(const std::string& value)
{
    if (value.find_first_of(",\"") == std::string::npos) return value;

    std::string escaped = "\"";
    for (char ch : value) {
        if (ch == '"') escaped += "\"\"";
        else escaped += ch;
    }
    escaped += "\"";
    return escaped;
}

std::string trimCopy(const std::string& value)
{
    const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) { return std::isspace(ch); });
    const auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) { return std::isspace(ch); }).base();
    if (begin >= end) return "";
    return std::string(begin, end);
}

std::string normalizeTraceColumnName(std::string value)
{
    value = trimCopy(value);
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        if (ch == ' ' || ch == '-') return '_';
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

bool readTraceOptionalDouble(const std::vector<std::string>& fields, int index, double& out)
{
    if (index < 0 || index >= static_cast<int>(fields.size())) return false;
    if (fields[index].empty()) return false;
    try {
        out = std::stod(fields[index]);
        return true;
    }
    catch (...) {
        return false;
    }
}

bool readTraceOptionalBool(const std::vector<std::string>& fields, int index, bool& out)
{
    if (index < 0 || index >= static_cast<int>(fields.size())) return false;
    std::string value = normalizeTraceColumnName(fields[index]);
    if (value.empty()) return false;
    if (value == "1" || value == "true" || value == "yes") {
        out = true;
        return true;
    }
    if (value == "0" || value == "false" || value == "no") {
        out = false;
        return true;
    }
    return false;
}

std::string schemaOutputDir()
{
    std::string resultDir = ensureTrailingSlash(veins::getCurrentResultDir());
    resultDir = resultDir.empty() ? std::string("./") : resultDir;

    mkdir(resultDir.c_str(), 0755);
    return resultDir;
}

std::string computeRunTimestamp()
{
    std::time_t now = std::time(nullptr);
    std::tm* timeInfo = std::localtime(&now);
    if (!timeInfo) return "0000_00_00_0000";

    std::ostringstream ss;
    ss << std::setfill('0')
       << (timeInfo->tm_year + 1900) << "_"
       << std::setw(2) << (timeInfo->tm_mon + 1) << "_"
       << std::setw(2) << timeInfo->tm_mday << "_"
       << std::setw(2) << timeInfo->tm_hour
       << std::setw(2) << timeInfo->tm_min;
    return ss.str();
}

char normalizeTlsSignal(char rawSignal)
{
    switch (rawSignal) {
    case 'G':
    case 'g':
        return 'G';
    case 'Y':
    case 'y':
        return 'Y';
    case 'R':
    case 'r':
        return 'R';
    default:
        return 'U';
    }
}

void resetCsvOutputs()
{
    if (s_timeseriesFile.is_open()) s_timeseriesFile.close();
    if (s_safeStopProbeFile.is_open()) s_safeStopProbeFile.close();
    s_safeStopProbeReady = false;
    if (s_spatResponseFile.is_open()) s_spatResponseFile.close();
    if (s_spatAuthDecisionFile.is_open()) s_spatAuthDecisionFile.close();
    if (s_latencyBreakdownFile.is_open()) s_latencyBreakdownFile.close();
    if (s_cavLinkTraceFile.is_open()) s_cavLinkTraceFile.close();
    s_timeseriesReady = false;
    s_spatResponseReady = false;
    s_spatAuthDecisionReady = false;
    s_latencyBreakdownReady = false;
    s_cavLinkTraceReady = false;
    s_cavLinkTraceSeen.clear();
    s_runTimestamp.clear();
    s_pdrSteps.clear();
    s_distPdrBins.clear();
    s_distPdrWritten = false;
    s_lastDistPdrFlush = SIMTIME_ZERO;
    s_floodColl.clear();
    s_floodCollWritten = false;
}

void ensureCsvRunContext()
{
    const std::string currentRunId = veins::getCurrentRunId();
    if (currentRunId == s_csvRunId) return;
    resetCsvOutputs();
    s_csvRunId = currentRunId;
}

const std::string& currentRunTimestamp()
{
    ensureCsvRunContext();
    if (s_runTimestamp.empty()) s_runTimestamp = computeRunTimestamp();
    return s_runTimestamp;
}

void ensureTimeseriesFile()
{
    ensureCsvRunContext();
    if (s_timeseriesReady && s_timeseriesFile.is_open()) return;

    s_timeseriesFile.open(schemaOutputDir() + "vehicle_timeseries_" + currentRunTimestamp() + ".csv");
    if (!s_timeseriesFile.is_open()) return;
    s_timeseriesReady = true;

    s_timeseriesFile
        << "run_number,"
        << "seed_set,"
        << "run_id,"
        << "time,"
        << "vehicle_id,"
        << "ego_is_cav,"
        << "vehicle_type,"
        << "pos_x,"
        << "pos_y,"
        << "speed_mps,"
        << "acceleration_mps2,"
        << "gap_to_leader_m,"
        << "leader_vehicle_id,"
        << "leader_is_cav,"
        << "ttc_s,"
        << "braking_distance_m,"
        << "hard_brake_event,"
        << "emergency_brake_event,"
        << "traffic_conflict,"
        << "safety_conflict,"
        << "tls_violation_event,"
        << "pedestrian_collision_event,"
        << "cyclist_collision_event,"
        << "active_transport_collision_event,"
        << "involves_cav,"
        << "cam_degraded_acc,"
        << "mrm_active,"
        // Signal this vehicle is currently approaching. Lets analysis scope metrics to the
        // ATTACKED junction exactly, instead of by radius: a radius wide enough to cover the
        // attacker's 300 m reach also pulls in J_3 (259 m from J_4), whose approaches carry
        // their own coverage-driven MRM and dilute the attack effect (lift 1038x at 100 m
        // falls to 6.0x at 300 m purely from that contamination).
        << "next_tls_id,"
        << "failsafe_state,"      // CACC | ACC | MRM  (MRM only once stopped)
        << "failsafe_reason,"    // V2V | V2I | V2I+V2V  (MRM implies V2I)
        << "cam_stale,"          // LINK state: -1 n/a (no leader) | 0 fresh | 1 stale
        << "spat_stale,"         // LINK state: -1 n/a (no equipped signal) | 0 fresh | 1 stale
        << "dist_to_stopline\n";  // m to the next stop line; -1 = none ahead
    s_timeseriesFile.flush();
}

void ensureSpatResponseFile()
{
    ensureCsvRunContext();
    if (s_spatResponseReady && s_spatResponseFile.is_open()) return;

    s_spatResponseFile.open(schemaOutputDir() + "spat_response.csv");
    if (!s_spatResponseFile.is_open()) return;
    s_spatResponseReady = true;

    s_spatResponseFile
        << "run_number,"
        << "seed_set,"
        << "run_id,"
        << "vehicle_id,"
        << "tls_id,"
        << "phase,"
        << "ttc_at_receipt_s,"
        << "time_sent_s,"
        << "time_received_s,"
        << "time_acted_s,"
        << "speed_at_receipt_mps,"
        << "configured_delay_s,"
        << "actual_delay_s,"
        << "extra_distance_m,"
        << "auth_reason,"
        << "network_step_v2i_attempts,"
        << "network_step_v2i_deliveries,"
        << "network_step_v2i_pdr\n";
    s_spatResponseFile.flush();
}

void ensureSpatAuthDecisionFile()
{
    ensureCsvRunContext();
    if (s_spatAuthDecisionReady && s_spatAuthDecisionFile.is_open()) return;

    s_spatAuthDecisionFile.open(schemaOutputDir() + "spat_auth_decisions.csv");
    if (!s_spatAuthDecisionFile.is_open()) return;
    s_spatAuthDecisionReady = true;

    s_spatAuthDecisionFile
        << "run_number,"
        << "seed_set,"
        << "run_id,"
        << "receiver_vehicle_id,"
        << "receiver_mode,"
        << "sender_vehicle_id,"
        << "message_kind,"
        << "message_id,"
        << "target_id,"
        << "tls_id,"
        << "phase,"
        << "sender_role,"
        << "purpose_tag,"
        << "auth_label,"
        << "cache_status,"
        << "pdp_verdict,"
        << "pdp_decision,"
        << "pep_action,"
        << "decision_reason,"
        << "standard_reference,"
        << "time_sent_s,"
        << "time_received_s,"
        << "decision_time_s,"
        << "configured_auth_delay_s,"
        << "actual_decision_delay_s,"
        << "speed_at_receipt_mps,"
        << "distance_m,"
        << "time_to_change_s,"
        << "auth_freshness_window_s,"
        << "trust_window_s\n";
    s_spatAuthDecisionFile.flush();
}

void ensureLatencyBreakdownFile()
{
    ensureCsvRunContext();
    if (s_latencyBreakdownReady && s_latencyBreakdownFile.is_open()) return;

    s_latencyBreakdownFile.open(schemaOutputDir() + "message_latency_breakdown.csv");
    if (!s_latencyBreakdownFile.is_open()) return;
    s_latencyBreakdownReady = true;

    s_latencyBreakdownFile
        << "run_number,"
        << "seed_set,"
        << "run_id,"
        << "receiver_vehicle_id,"
        << "receiver_mode,"
        << "sender_vehicle_id,"
        << "message_kind,"
        << "message_id,"
        << "target_id,"
        << "sender_role,"
        << "purpose_tag,"
        << "auth_label,"
        << "cache_status,"
        << "decision_stage,"
        << "auth_reason,"
        << "accepted,"
        << "time_sent_s,"
        << "time_received_s,"
        << "decision_time_s,"
        << "applied_time_s,"
        << "radio_delay_s,"
        << "verification_path_delay_s,"
        << "configured_auth_delay_s,"
        << "queue_wait_delay_s,"
        << "pep_to_apply_delay_s,"
        << "end_to_end_decision_delay_s,"
        << "end_to_end_apply_delay_s,"
        << "speed_at_receipt_mps,"
        << "distance_m\n";
    s_latencyBreakdownFile.flush();
}

void ensureCavLinkTraceFile()
{
    ensureCsvRunContext();
    if (s_cavLinkTraceReady && s_cavLinkTraceFile.is_open()) return;

    s_cavLinkTraceFile.open(schemaOutputDir() + "cav_pair_trace_" + currentRunTimestamp() + ".csv");
    if (!s_cavLinkTraceFile.is_open()) return;
    s_cavLinkTraceReady = true;

    s_cavLinkTraceFile
        << "time_s,"
        << "receiver_vehicle_id,"
        << "receiver_x_m,"
        << "receiver_y_m,"
        << "receiver_speed_mps,"
        << "receiver_heading_deg,"
        << "sender_vehicle_id,"
        << "sender_x_m,"
        << "sender_y_m,"
        << "sender_speed_mps,"
        << "sender_heading_deg\n";
    s_cavLinkTraceFile.flush();
}

double headingFromVelocityDegrees(double vx, double vy)
{
    if (std::abs(vx) < 1e-9 && std::abs(vy) < 1e-9) return 0.0;
    double deg = std::atan2(vy, vx) * 180.0 / 3.14159265358979323846;
    if (deg < 0.0) deg += 360.0;
    return deg;
}

std::string selectRuntimeGeneralizedCdlProfile(double distanceM, bool isLos)
{
    if (isLos) {
        return distanceM < 120.0 ? "CDL-D" : "CDL-E";
    }
    return distanceM < 150.0 ? "CDL-C" : "CDL-A";
}

bool isSpecialTraceVehicleId(const std::string& id)
{
    return id == "target_cav" || id == "attacker_car";
}

bool isTraceRelevantCavEndpoint(const std::string& id, const std::string& role)
{
    return role == "CAV" || isSpecialTraceVehicleId(id);
}

long long pdrStepKey(double t)
{
    return static_cast<long long>(std::floor((t / kPdrTimeBinSeconds) + 1e-9));
}

void recordPdrAttempt(bool isV2V, double t)
{
    ensureCsvRunContext();
    auto& bucket = s_pdrSteps[pdrStepKey(t)];
    if (isV2V) bucket.v2v.attempts++;
    else bucket.v2i.attempts++;
}

void recordPdrDelivery(bool isV2V, double t)
{
    ensureCsvRunContext();
    auto& bucket = s_pdrSteps[pdrStepKey(t)];
    if (isV2V) bucket.v2v.deliveries++;
    else bucket.v2i.deliveries++;
}

PdrCounters getPdrCounters(bool isV2V, double t)
{
    ensureCsvRunContext();
    auto it = s_pdrSteps.find(pdrStepKey(t));
    if (it == s_pdrSteps.end()) return {};
    return isV2V ? it->second.v2v : it->second.v2i;
}

long getLoggedStepAttempts(bool isV2V, double t)
{
    return getPdrCounters(isV2V, t).attempts;
}

long getLoggedStepDeliveries(bool isV2V, double t)
{
    return getPdrCounters(isV2V, t).deliveries;
}

double getLoggedStepPdr(bool isV2V, double t)
{
    const auto counters = getPdrCounters(isV2V, t);
    if (counters.attempts <= 0) return 1.0;
    return static_cast<double>(counters.deliveries) / counters.attempts;
}

veins::VeinsInetMobility* inetMobility(cModule* host)
{
    if (!host) return nullptr;
    return dynamic_cast<veins::VeinsInetMobility*>(host->getSubmodule("mobility"));
}

VeinsInet5GVehicleApp* inet5GApp(cModule* host)
{
    if (!host) return nullptr;
    return dynamic_cast<VeinsInet5GVehicleApp*>(host->getSubmodule("app", 0));
}

void resetSchemaJsonOutputs()
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

void ensureSchemaJsonRunContext()
{
    const std::string currentRunId = veins::getCurrentRunId();
    if (currentRunId == s_schemaJsonRunId) return;
    resetSchemaJsonOutputs();
    s_schemaJsonRunId = currentRunId;
}

void ensureGroundTruthJsonFile()
{
    ensureSchemaJsonRunContext();
    if (s_groundTruthJsonReady) return;
    s_groundTruthJsonReady = true;
    s_groundTruthJsonFile.open(schemaOutputDir() + "GroundTruthJSONlog.json");
}

void ensureVehicleJsonMapFile()
{
    ensureSchemaJsonRunContext();
    if (s_vehicleJsonMapReady) return;

    s_vehicleJsonMapReady = true;
    s_vehicleJsonMapFile.open(schemaOutputDir() + "JSONlog_vehicle_map.csv");
    if (!s_vehicleJsonMapFile.is_open()) return;

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

std::ofstream* ensureVehicleJsonFile(const std::string& fileName,
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

    auto stream = std::make_unique<std::ofstream>(schemaOutputDir() + fileName);
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

int lookupSchemaVehicleIndexByExternalId(const std::string& externalId)
{
    if (externalId.empty()) return -1;

    auto* manager = veins::FindModule<veins::TraCIScenarioManager*>::findGlobalModule();
    if (!manager) return -1;

    for (const auto& kv : manager->getManagedHosts()) {
        cModule* node = kv.second;
        auto* hostMobility = inetMobility(node);
        if (!node || !hostMobility) continue;
        try {
            if (hostMobility->getExternalId() == externalId) {
                return (node->getIndex() >= 0) ? node->getIndex() : node->getId();
            }
        }
        catch (...) {
        }
    }

    return -1;
}

double estimateSchemaRssiWatts(double distanceMeters,
                               double txPowerDbm,
                               double referenceLossDb,
                               double pathLossExponent)
{
    const double d = std::max(kReferenceDistanceMeters, distanceMeters);
    const double recvPowerDbm = txPowerDbm - (referenceLossDb + 10.0 * pathLossExponent * std::log10(d));
    return std::pow(10.0, recvPowerDbm / 10.0) / 1000.0;
}

} // namespace

Define_Module(VeinsInet5GVehicleApp);

bool VeinsInet5GVehicleApp::startApplication()
{
    readParameters();
    running_ = true;

    if (!initialiseVehicleState()) {
        scheduleVehicleBindRetry();
        EV_INFO << "[VeinsInet5GVehicleApp] delaying startup until TraCI binding is ready for module="
                << getParentModule()->getFullPath() << "\n";
        return true;
    }

    if (logGroundtruth_) {
        ensureGroundTruthJsonFile();
        logSchemaSelfState();
    }

    startTelemetry();

    if (!manualMode_) {
        scheduleCavStartup();
    }

    if (isAttacker_) scheduleFlood();

    EV_INFO << "[VeinsInet5GVehicleApp] started vehicle=" << selfVehicleId_
            << " type=" << sumoVehicleType_
            << " mode=" << (manualMode_ ? "human/manual" : "CAV")
            << " interferer=" << (isAttacker_ ? "true" : "false")
            << " target=" << (targetVehicleId_.empty() ? "<none>" : targetVehicleId_)
            << " OBU capacity=" << obuCapacityPps_ << " pkt/s"
            << " spatLossPolicy=" << spatLossPolicy_ << "\n";
    return true;
}

bool VeinsInet5GVehicleApp::stopApplication()
{
    running_ = false;
    return true;
}

void VeinsInet5GVehicleApp::writeDistancePdrCsv()
{
    ensureCsvRunContext();
    if (s_distPdrBins.empty()) return;

    const std::string runId  = veins::getCurrentRunId();
    const std::string runNum = veins::getCurrentRunNumberString();
    const std::string seed   = veins::getCurrentSeedSet();
    const std::string path   = schemaOutputDir() + "dist_pdr_" + currentRunTimestamp() + ".csv";

    std::ofstream f(path);
    if (!f.is_open()) {
        EV_WARN << "[VeinsInet5GVehicleApp] Cannot open dist_pdr CSV: " << path << "\n";
        return;
    }

    f << "run_number,seed_set,run_id,"
      << "dist_bin_m,attempts,delivered,pdr,"
      << "drop_hd,drop_prop,drop_sps,drop_base,drop_cap\n";

    for (const auto& kv : s_distPdrBins) {
        const DistPdrBin& b = kv.second;
        const double pdr = b.attempts > 0 ? static_cast<double>(b.delivered) / b.attempts : 0.0;
        f << runNum << ","
          << seed   << ","
          << runId  << ","
          << kv.first << ","
          << b.attempts  << ","
          << b.delivered << ","
          << std::fixed << std::setprecision(6) << pdr << ","
          << b.drop_hd   << ","
          << b.drop_prop << ","
          << b.drop_sps  << ","
          << b.drop_base << ","
          << b.drop_cap  << "\n";
    }

    f.flush();
    EV_INFO << "[VeinsInet5GVehicleApp] dist_pdr CSV written: " << path << "\n";
}

void VeinsInet5GVehicleApp::writeFloodCollCsv()
{
    if (!s_floodCollEnabled || s_floodColl.empty()) return;
    ensureCsvRunContext();
    const std::string runNum = veins::getCurrentRunNumberString();
    const std::string seed   = veins::getCurrentSeedSet();
    const std::string path   = schemaOutputDir() + "flood_coll_" + currentRunTimestamp() + ".csv";

    std::ofstream f(path);
    if (!f.is_open()) {
        EV_WARN << "[VeinsInet5GVehicleApp] Cannot open flood_coll CSV: " << path << "\n";
        return;
    }
    f << "run_number,seed_set,n,flooder_in_range,attempts,collisions,collision_rate\n";
    for (const auto& kv : s_floodColl) {
        const FloodCollBin& b = kv.second;
        const double rate = b.attempts > 0 ? static_cast<double>(b.collisions) / b.attempts : 0.0;
        f << runNum << "," << seed << ","
          << kv.first.first << "," << kv.first.second << ","
          << b.attempts << "," << b.collisions << ","
          << std::fixed << std::setprecision(6) << rate << "\n";
    }
    f.flush();
    EV_INFO << "[VeinsInet5GVehicleApp] flood_coll CSV written: " << path << "\n";
}

void VeinsInet5GVehicleApp::finish()
{
    const double pdr = stat_rx_total_ > 0 ? static_cast<double>(stat_rx_delivered_) / stat_rx_total_ : 0.0;
    const double targetPdr = stat_target_rx_total_ > 0 ? static_cast<double>(stat_target_rx_delivered_) / stat_target_rx_total_ : 0.0;

    recordScalar("inet5g_rx_total", stat_rx_total_);
    recordScalar("inet5g_rx_delivered", stat_rx_delivered_);
    recordScalar("inet5g_pdr", pdr);
    recordScalar("inet5g_drop_base", stat_drop_base_);
    recordScalar("inet5g_drop_hd", stat_drop_hd_);
    recordScalar("inet5g_drop_sps", stat_drop_sps_);
    recordScalar("inet5g_drop_capacity", stat_drop_capacity_);
    recordScalar("inet5g_drop_propagation", stat_drop_propagation_);
    recordScalar("inet5g_cam_tx", stat_cam_tx_);
    recordScalar("inet5g_cam_rx", stat_cam_rx_);
    recordScalar("inet5g_spat_rx", stat_spat_rx_);
    recordScalar("inet5g_flood_tx", stat_flood_tx_);
    recordScalar("inet5g_flood_rx", stat_flood_rx_);
    recordScalar("inet5g_target_rx_total", stat_target_rx_total_);
    recordScalar("inet5g_target_rx_delivered", stat_target_rx_delivered_);
    recordScalar("inet5g_target_pdr", targetPdr);
    recordScalar("inet5g_target_drop_capacity", stat_target_drop_capacity_);
    recordScalar("inet5g_auth_full", stat_auth_full_);
    recordScalar("inet5g_auth_cachehit", stat_auth_cachehit_);
    recordScalar("inet5g_auth_stale_drop", stat_auth_stale_drop_);
    recordScalar("inet5g_auth_queue_overflow", stat_auth_queue_overflow_);

    const auto& traceState = channelTraceState();
    recordScalar("channel_trace_configured", channelTraceCsv_.empty() ? 0 : 1);
    recordScalar("channel_trace_loaded", traceState.loaded ? 1 : 0);
    recordScalar("channel_trace_link_count", static_cast<double>(traceState.samplesByLink.size()));
    recordScalar("channel_trace_lookup_count", traceState.loggedLookupCount);
    recordScalar("inet5g_trace_configured", channelTraceCsv_.empty() ? 0 : 1);
    recordScalar("inet5g_trace_loaded", traceState.loaded ? 1 : 0);
    recordScalar("inet5g_trace_link_count", static_cast<double>(traceState.samplesByLink.size()));
    recordScalar("inet5g_trace_lookup_count", traceState.loggedLookupCount);
    const auto& generalizedState = generalizedPhyRuntimeState();
    recordScalar("inet5g_generalized_phy_configured", generalizedPhyCoeffDir_.empty() ? 0 : 1);
    recordScalar("inet5g_generalized_phy_loaded", generalizedState.loaded ? 1 : 0);
    recordScalar("inet5g_generalized_phy_active", activateGeneralizedPhyModel_ ? 1 : 0);

    EV_INFO << "[VeinsInet5GVehicleApp] Final stats vehicle=" << selfVehicleId_
            << " rx=" << stat_rx_total_
            << " delivered=" << stat_rx_delivered_
            << " pdr=" << pdr
            << " capacity_drop=" << stat_drop_capacity_
            << " target_pdr=" << targetPdr << "\n";

    // Write distance-PDR calibration CSV once per run (first node to reach finish).
    // finish() is only called after all simulation events have been processed,
    // so s_distPdrBins is fully populated when any node reaches this point.
    if (!s_distPdrWritten) {
        s_distPdrWritten = true;
        writeDistancePdrCsv();
    }
    if (!s_floodCollWritten) {
        s_floodCollWritten = true;
        writeFloodCollCsv();
    }

    veins::VeinsInetApplicationBase::finish();
}

void VeinsInet5GVehicleApp::readParameters()
{
    manualVehicleType_ = par("manualVehicleType").stdstringValue();
    cavVehicleType_ = par("cavVehicleType").stdstringValue();
    authModeLabel_ = par("authModeLabel").stdstringValue();
    v2vPropagationModel_ = par("v2vPropagationModel").stdstringValue();
    spatPropagationModel_ = par("spatPropagationModel").stdstringValue();
    enableObuProcessingQueue_ = par("enableObuProcessingQueue").boolValue();
    if (authModeLabel_.empty()) authModeLabel_ = "INET_5G";

    beaconInterval_ = par("beaconInterval").doubleValue();
    camTrustWindow_ = par("camTrustWindow").doubleValue();
    spatTrustWindow_ = par("spatTrustWindow").doubleValue();
    actualCamInterval_ = par("actualCamInterval").doubleValue();
    spatInterval_ = par("spatInterval").doubleValue();
    actualSpatInterval_ = par("actualSpatInterval").doubleValue();
    pc5AirDelayMean_ = par("pc5AirDelayMean").doubleValue();
    pc5AirDelayStd_ = par("pc5AirDelayStd").doubleValue();
    pc5DropRate_ = par("pc5DropRate").doubleValue();
    pc5Range_ = par("pc5Range").doubleValue();
    pc5HalfDuplexDrop_ = par("pc5HalfDuplexDrop").doubleValue();
    sbspsReselectionInterval_ = par("sbspsReselectionInterval").doubleValue();
    pc5SubchannelsPerSubframe_ = par("pc5SubchannelsPerSubframe").doubleValue();
    pc5ResourcesPerPacket_ = par("pc5ResourcesPerPacket").doubleValue();
    pc5CandidateResourceFraction_ = par("pc5CandidateResourceFraction").doubleValue();
    pc5TxPowerDbm_ = par("pc5TxPowerDbm").doubleValue();
    pc5SensingThresholdDbm_ = par("pc5SensingThresholdDbm").doubleValue();
    pc5ShadowingStdDb_ = par("pc5ShadowingStdDb").doubleValue();
    pc5NoiseFigureDb_ = par("pc5NoiseFigureDb").doubleValue();
    pc5BandwidthHz_ = par("pc5BandwidthHz").doubleValue();
    pc5ShadowCorrelationWindowS_ = par("pc5ShadowCorrelationWindow").doubleValue();
    pc5PathLossExponent_ = par("pc5PathLossExponent").doubleValue();
    pc5ReferenceLossDb_ = par("pc5ReferenceLossDb").doubleValue();
    channelTraceCsv_ = par("channelTraceCsv").stdstringValue();
    generalizedPhyCoeffDir_ = par("generalizedPhyCoeffDir").stdstringValue();
    spatCarrierFrequencyGHz_ = par("spatCarrierFrequencyGHz").doubleValue();
    spatBsHeight_ = par("spatBsHeight").doubleValue();
    spatUtHeight_ = par("spatUtHeight").doubleValue();

    // ── NS-3-distilled realism selector (additive; existing methods untouched) ──
    losOnly_ = par("losOnly").boolValue();
    realismModel_ = par("realismModel").stdstringValue();
    ns3LearnCoeffDir_ = par("ns3LearnCoeffDir").stdstringValue();
    ns3LearnEnabled_ = (realismModel_ == "ns3learn");
    if (ns3LearnEnabled_) {
        if (loadNs3LearnModel(ns3LearnCoeffDir_)) {
            // cascade: explicit half-duplex stage (pc5HalfDuplexDrop_=hd). distance-aware lumped:
            // folds half-duplex+collision into the SINR (hd=0). legacy flat: fitted constant hd.
            pc5HalfDuplexDrop_ = (ns3DistanceAware_ && !ns3Cascade_) ? 0.0 : ns3HalfDuplex_;
            EV_INFO << "ns3learn realism active: cascade=" << ns3Cascade_ << " distance_aware=" << ns3DistanceAware_
                    << " intf=(" << ns3IntfA_ << "*log10(1+n)+" << ns3IntfB_ << ") hd=" << ns3HalfDuplex_
                    << " capture=" << ns3Capture_ << " coll(L,k,x0)=("
                    << ns3CollL_ << "," << ns3CollK_ << "," << ns3CollX0_
                    << ") noiseFloor=" << ns3NoiseFloorDbm_ << " dBm\n";
        } else {
            EV_WARN << "ns3learn: failed to load coefficients from '" << ns3LearnCoeffDir_
                    << "'; reverting to analytical path\n";
            ns3LearnEnabled_ = false;
        }
    }

    // ── Combined Analytical Reference (Cao 2026 + Rehman 2023), paper Eq. m3 ──
    // Additive: does not touch the "analytical"/"ns3learn" paths. Routes the SAME
    // three drop hooks (hd / prop / sps) to the two published models. The three
    // modes are the combined reference and its two single-mechanism ablations.
    analyticalM3Enabled_ = (realismModel_ == "analytical_m3"
                            || realismModel_ == "analytical_col"
                            || realismModel_ == "analytical_prop");
    if (analyticalM3Enabled_) {
        m3UseCollision_ = (realismModel_ != "analytical_prop");   // Cao P_COL + P_HD
        m3UseDecode_    = (realismModel_ != "analytical_col");    // Rehman g(SNR(d))
        if (!loadAnalyticalM3Model()) {
            EV_WARN << "analytical_m3: failed to load BLER curve from '"
                    << par("m3BlerCurveCsv").stringValue() << "'; reverting to analytical path\n";
            analyticalM3Enabled_ = false;
        } else {
            // Half-duplex lives ONLY in Model 1 (P_HD); no stray base drop.
            pc5HalfDuplexDrop_ = m3UseCollision_ ? m3PHd_ : 0.0;
            pc5DropRate_ = 0.0;
            EV_INFO << "analytical_m3 active (" << realismModel_ << "): P_HD=" << m3PHd_
                    << " N_r=" << m3Nr_ << " Nc=" << m3Nc_ << " pi0=" << m3Pi0_
                    << " collision=" << m3UseCollision_ << " decode=" << m3UseDecode_
                    << " noiseFloor=" << getPc5NoiseFloorDbm() << " dBm\n";
        }
    }

    obuCapacityPps_ = par("obuCapacityPps").doubleValue();
    capacityWindow_ = par("capacityWindow").doubleValue();
    capacityDropSlope_ = par("capacityDropSlope").doubleValue();
    authDelayMean_ = par("authDelayMean").doubleValue();
    authDelayStd_ = par("authDelayStd").doubleValue();
    authCacheWindow_ = par("authCacheWindow").doubleValue();
    authFreshnessWindow_ = par("authFreshnessWindow").doubleValue();
    obuQueueMaxDepth_ = par("obuQueueMaxDepth").doubleValue();
    processorFreeAt_ = 0.0;
    spatStaleTimeout_ = par("spatStaleTimeout").doubleValue();
    spatStopDecel_ = par("spatStopDecel").doubleValue();
    mrmActivationDistance_ = par("mrmActivationDistance").doubleValue();
    spatHandoverTime_ = par("spatHandoverTime").doubleValue();
    spatCaccTypeId_ = par("spatCaccTypeId").stdstringValue();
    spatAccTypeId_ = par("spatAccTypeId").stdstringValue();
    enableSpatControl_ = par("enableSpatControl").boolValue();
    enableCamCacc_ = par("enableCamCacc").boolValue();
    camStaleTimeout_ = par("camStaleTimeout").doubleValue();
    caccLeaderRange_ = par("caccLeaderRange").doubleValue();
    enableResearchFailsafe_ = par("enableResearchFailsafe").boolValue();
    bypassAnalyticalChannel_ = par("bypassAnalyticalChannel").boolValue();
    activateChannelTrace_ = par("activateChannelTrace").boolValue();
    activateGeneralizedPhyModel_ = par("activateGeneralizedPhyModel").boolValue();

    enableFlooding_ = par("enableFlooding").boolValue();
    disguiseFloodAsCam_ = par("disguiseFloodAsCam").boolValue();
    configuredAttackerVehicleId_ = par("attackerVehicleId").stdstringValue();
    targetVehicleId_ = par("targetVehicleId").stdstringValue();
    attackDeliveryMode_ = par("attackDeliveryMode").stdstringValue();
    logGroundtruth_ = par("logGroundtruth").boolValue();
    logCavLinkTrace_ = par("logCavLinkTrace").boolValue();
    logLatencyBreakdown_ = par("logLatencyBreakdown").boolValue();
    floodRate_ = par("floodRate").doubleValue();
    maxActualFloodRate_ = par("maxActualFloodRate").doubleValue();
    attackStartTime_ = par("attackStartTime").doubleValue();
    attackDuration_ = par("attackDuration").doubleValue();
    targetRange_ = par("targetRange").doubleValue();
    openEndedAttack_ = attackDuration_ <= 0.0;

    // Which fail-safe responses this CAV may perform, e.g. "MRM,ACC" | "ACC" | "MRM" | "".
    // EMPTY MEANS NO FAIL-SAFE WAS DESIGNED: no ACC degradation on V2V loss, no stop on V2I
    // loss. That is a deliberate experimental arm (the un-mitigated CAV), not a fallback to
    // some other mechanism. spatLossPolicy below is a SEPARATE, legacy mechanism used by the
    // CV2X / Flooding / WeatherFlooding projects; it is not consulted when enableCamCacc is on.
    activateFailsafe_ = par("activateFailsafe").stdstringValue();
    {
        std::string up;
        for (char ch : activateFailsafe_) up += static_cast<char>(::toupper(ch));
        failsafeAcc_ = up.find("ACC") != std::string::npos;
        failsafeMrm_ = up.find("MRM") != std::string::npos;
        failsafeConfigured_ = failsafeAcc_ || failsafeMrm_;
        const bool junk = up.find_first_not_of(" \t,") != std::string::npos && !failsafeConfigured_;
        if (junk) {
            EV_WARN << "[VeinsInet5GVehicleApp] activateFailsafe=\"" << activateFailsafe_
                    << "\" names neither MRM nor ACC; treating as no fail-safe.\n";
        }
    }

    // SPAT-loss policy. Empty means no explicit degraded-mode recovery.
    spatLossPolicy_ = par("spatLossPolicy").stdstringValue();
    enableResearchFailsafe_ = par("enableResearchFailsafe").boolValue();

    // enableCamCacc supersedes spatLossPolicy for SPaT loss: the application-differentiated
    // ladder replaces it. Warn loudly rather than let a swept policy factor silently
    // collapse into identical arms (e.g. ${policy=sumo_fallback,safe_stop} with camCacc on).
    if (enableCamCacc_ && !spatLossPolicy_.empty()) {
        EV_WARN << "[VeinsInet5GVehicleApp] enableCamCacc=true supersedes spatLossPolicy=\""
                << spatLossPolicy_ << "\" for SPaT loss: the application-differentiated"
                << " CACC->ACC->MRM ladder is used instead. If spatLossPolicy is a swept"
                << " experimental factor, its arms are now identical — set it to \"\".\n";
    }

    if (!channelTraceCsv_.empty()) {
        ensureChannelTraceLoaded(channelTraceCsv_);
        const auto& state = channelTraceState();
        EV_WARN << "[VeinsInet5GVehicleApp] channel trace "
                << (state.loaded ? "enabled" : "not available")
                << " path=" << channelTraceCsv_
                << " links=" << state.samplesByLink.size() << "\n";
    }

    if (!generalizedPhyCoeffDir_.empty()) {
        ensureGeneralizedPhyModelLoaded(generalizedPhyCoeffDir_);
        const auto& state = generalizedPhyRuntimeState();
        EV_WARN << "[VeinsInet5GVehicleApp] generalized PHY model "
                << (state.loaded ? "enabled" : "not available")
                << " dir=" << generalizedPhyCoeffDir_ << "\n";
    }

    if (activateGeneralizedPhyModel_ && generalizedPhyCoeffDir_.empty()) {
        throw cRuntimeError("activateGeneralizedPhyModel=true but generalizedPhyCoeffDir is empty");
    }

    if (activateGeneralizedPhyModel_ && !generalizedPhyCoeffDir_.empty()) {
        const auto& state = generalizedPhyRuntimeState();
        if (!state.loaded) {
            throw cRuntimeError("activateGeneralizedPhyModel=true but generalized PHY coefficients could not be loaded from '%s'",
                                generalizedPhyCoeffDir_.c_str());
        }
    }
}

VeinsInet5GVehicleApp::ChannelTraceState& VeinsInet5GVehicleApp::channelTraceState()
{
    static ChannelTraceState state;
    return state;
}

VeinsInet5GVehicleApp::GeneralizedPhyRuntimeState& VeinsInet5GVehicleApp::generalizedPhyRuntimeState()
{
    static GeneralizedPhyRuntimeState state;
    return state;
}

std::vector<std::string> VeinsInet5GVehicleApp::splitCsvLine(const std::string& line)
{
    std::vector<std::string> fields;
    std::string current;
    bool inQuotes = false;

    for (size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (ch == '"') {
            if (inQuotes && i + 1 < line.size() && line[i + 1] == '"') {
                current += '"';
                ++i;
            }
            else {
                inQuotes = !inQuotes;
            }
        }
        else if (ch == ',' && !inQuotes) {
            fields.push_back(trimCopy(current));
            current.clear();
        }
        else {
            current += ch;
        }
    }

    fields.push_back(trimCopy(current));
    return fields;
}

bool VeinsInet5GVehicleApp::loadGeneralizedPhyLinearModel(const std::string& csvPath, GeneralizedPhyLinearModel& model)
{
    std::ifstream in(csvPath);
    if (!in.is_open()) return false;

    std::string line;
    if (!std::getline(in, line)) return false;
    const auto header = splitCsvLine(line);

    int responseIdx = -1;
    int termIdx = -1;
    int estimateIdx = -1;
    for (size_t i = 0; i < header.size(); ++i) {
        if (header[i] == "response") responseIdx = static_cast<int>(i);
        else if (header[i] == "term") termIdx = static_cast<int>(i);
        else if (header[i] == "estimate") estimateIdx = static_cast<int>(i);
    }
    if (termIdx < 0 || estimateIdx < 0) return false;

    model.coefficients.clear();
    model.responseName.clear();
    model.loaded = false;

    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const auto fields = splitCsvLine(line);
        if (termIdx >= static_cast<int>(fields.size()) || estimateIdx >= static_cast<int>(fields.size())) continue;

        try {
            const std::string term = fields[termIdx];
            const double estimate = std::stod(fields[estimateIdx]);
            model.coefficients[term] = estimate;
            if (responseIdx >= 0 && responseIdx < static_cast<int>(fields.size()) && model.responseName.empty()) {
                model.responseName = fields[responseIdx];
            }
        }
        catch (...) {
            continue;
        }
    }

    model.loaded = !model.coefficients.empty();
    return model.loaded;
}

bool VeinsInet5GVehicleApp::loadGeneralizedPhyBlerCurve(const std::string& csvPath,
                                                        const std::string& profileName,
                                                        GeneralizedPhyBlerCurve& curve)
{
    std::ifstream in(csvPath);
    if (!in.is_open()) return false;

    std::string line;
    if (!std::getline(in, line)) return false;
    const auto header = splitCsvLine(line);

    int snrIdx = -1;
    int blerIdx = -1;
    int psrIdx = -1;
    for (size_t i = 0; i < header.size(); ++i) {
        if (header[i] == "snr_db") snrIdx = static_cast<int>(i);
        else if (header[i] == "bler") blerIdx = static_cast<int>(i);
        else if (header[i] == "psr") psrIdx = static_cast<int>(i);
    }
    if (snrIdx < 0 || blerIdx < 0) return false;

    curve = GeneralizedPhyBlerCurve();
    curve.profileName = normalizeProfileKey(profileName);

    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const auto fields = splitCsvLine(line);
        if (snrIdx >= static_cast<int>(fields.size()) || blerIdx >= static_cast<int>(fields.size())) continue;
        try {
            curve.snrDb.push_back(std::stod(fields[snrIdx]));
            curve.bler.push_back(std::stod(fields[blerIdx]));
            if (psrIdx >= 0 && psrIdx < static_cast<int>(fields.size())) {
                curve.psr.push_back(std::stod(fields[psrIdx]));
            }
            else {
                curve.psr.push_back(1.0 - curve.bler.back());
            }
        }
        catch (...) {
            continue;
        }
    }

    curve.loaded = !curve.snrDb.empty() && curve.snrDb.size() == curve.bler.size();
    return curve.loaded;
}

bool VeinsInet5GVehicleApp::loadChannelGainDistParams(const std::string& csvPath,
                                                       std::map<std::string, ChannelGainDistParam>& out)
{
    // CSV columns:
    //  0:profile  1:dist_type  2:rician_k_db  3:mean_gain_db  4:std_gain_db
    //  5:mean_lin  6:std_lin  7:n_samples  8:omega_log10_intercept  9:omega_log10_slope
    std::ifstream in(csvPath);
    if (!in.is_open()) return false;

    std::string line;
    if (!std::getline(in, line)) return false;  // skip header

    bool any = false;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const auto fields = splitCsvLine(line);
        if (fields.size() < 6) continue;
        try {
            ChannelGainDistParam p;
            p.profile  = fields[0];
            p.distType = fields[1];

            // rician_k_db may be "-Inf" for Rayleigh profiles
            const std::string& kStr = fields[2];
            double kDb = -std::numeric_limits<double>::infinity();
            if (kStr != "-Inf" && kStr != "-inf" && kStr != "−Inf" && !kStr.empty()) {
                kDb = std::stod(kStr);
            }
            p.rician_k_lin = std::isfinite(kDb) ? std::pow(10.0, kDb / 10.0) : 0.0;
            p.mean_lin     = std::stod(fields[5]);  // global mean power fallback

            // Distance-conditioned mean power: Ω(d) = 10^(a + b·log10(d))
            // Columns 8 and 9 are present in the upgraded CSV.
            if (fields.size() >= 10) {
                p.omega_log10_intercept = std::stod(fields[8]);
                p.omega_log10_slope     = std::stod(fields[9]);
                p.hasDistanceModel      = true;
            }

            const std::string key = normalizeProfileKey(p.profile);
            out[key] = p;
            any = true;
        }
        catch (...) {
            continue;
        }
    }
    return any;
}

void VeinsInet5GVehicleApp::ensureGeneralizedPhyModelLoaded(const std::string& coeffDir)
{
    auto& state = generalizedPhyRuntimeState();
    if (state.loaded || state.loadAttempted || coeffDir.empty()) return;
    state.loadAttempted = true;

    const std::string base = coeffDir.back() == '/' ? coeffDir : coeffDir + "/";
    const bool okPathLoss = loadGeneralizedPhyLinearModel(base + "path_loss_coefficients.csv", state.pathLossModel);
    const bool okRssi = loadGeneralizedPhyLinearModel(base + "rssi_coefficients.csv", state.rssiModel);
    const bool okDelay = loadGeneralizedPhyLinearModel(base + "delay_spread_log10_coefficients.csv", state.delaySpreadLog10Model);
    const bool okKFactor = loadGeneralizedPhyLinearModel(base + "k_factor_coefficients.csv", state.kFactorModel);
    const bool okGainLos = loadGeneralizedPhyLinearModel(base + "channel_gain_los_coefficients.csv", state.channelGainLosModel);
    const bool okGainNlos = loadGeneralizedPhyLinearModel(base + "channel_gain_nlos_coefficients.csv", state.channelGainNlosModel);
    state.hasSplitChannelGainModel = okGainLos && okGainNlos;
    const bool okGain = state.hasSplitChannelGainModel
        ? true
        : loadGeneralizedPhyLinearModel(base + "channel_gain_coefficients.csv", state.channelGainModel);
    state.blerCurvesByProfile.clear();
    const std::vector<std::pair<std::string, std::string>> blerFiles = {
        {"CDL-A", base + "bler_curve_cdl_a.csv"},
        {"CDL-C", base + "bler_curve_cdl_c.csv"},
        {"CDL-D", base + "bler_curve_cdl_d.csv"},
        {"CDL-E", base + "bler_curve_cdl_e.csv"}
    };
    for (const auto& entry : blerFiles) {
        GeneralizedPhyBlerCurve curve;
        if (loadGeneralizedPhyBlerCurve(entry.second, entry.first, curve)) {
            state.blerCurvesByProfile[normalizeProfileKey(entry.first)] = curve;
        }
    }
    state.hasBlerCurves = !state.blerCurvesByProfile.empty();

    // Load per-profile channel gain distribution parameters (Rician / Rayleigh).
    // These replace the OLS surrogate for small-scale fading, which had R²≈0.26
    // because small-scale fading is stochastic — no geometry predictor can explain it.
    state.gainDistByProfile.clear();
    state.hasGainDistParams = loadChannelGainDistParams(
        base + "channel_gain_distribution_params.csv",
        state.gainDistByProfile);

    state.loaded = okPathLoss && okRssi && okDelay && okKFactor && okGain;
}

bool VeinsInet5GVehicleApp::parseChannelTraceRow(const std::string& line,
                                                 const ChannelTraceCsvLayout& layout,
                                                 std::string& txId,
                                                 std::string& rxId,
                                                 ChannelTraceSample& sample)
{
    const auto fields = splitCsvLine(line);
    if (layout.legacyOrder) {
        if (fields.size() < 7) return false;
        try {
            txId = fields[0];
            rxId = fields[1];
            sample.timeS = std::stod(fields[2]);
            sample.distanceM = std::stod(fields[3]);
            sample.rxPowerDb = std::stod(fields[4]);
            sample.dopplerHz = std::stod(fields[5]);
            sample.delaySpreadS = std::stod(fields[6]);
        }
        catch (...) {
            return false;
        }
        return !txId.empty() && !rxId.empty();
    }

    if (layout.txId < 0 || layout.rxId < 0 || layout.timeS < 0 || layout.distanceM < 0) return false;
    if (layout.txId >= static_cast<int>(fields.size()) || layout.rxId >= static_cast<int>(fields.size()) || layout.timeS >= static_cast<int>(fields.size())) return false;

    txId = fields[layout.txId];
    rxId = fields[layout.rxId];
    if (txId.empty() || rxId.empty()) return false;

    try {
        sample.timeS = std::stod(fields[layout.timeS]);
        sample.distanceM = std::stod(fields[layout.distanceM]);
    }
    catch (...) {
        return false;
    }

    if (!readTraceOptionalDouble(fields, layout.rxPowerDb, sample.rxPowerDb)) {
        readTraceOptionalDouble(fields, layout.rssiDbm, sample.rxPowerDb);
    }
    readTraceOptionalDouble(fields, layout.dopplerHz, sample.dopplerHz);
    readTraceOptionalDouble(fields, layout.delaySpreadS, sample.delaySpreadS);
    readTraceOptionalDouble(fields, layout.relativeSpeedMps, sample.relativeSpeedMps);
    readTraceOptionalDouble(fields, layout.radialVelocityMps, sample.radialVelocityMps);
    readTraceOptionalDouble(fields, layout.pathLossDb, sample.pathLossDb);
    readTraceOptionalDouble(fields, layout.losProbability, sample.losProbability);
    readTraceOptionalDouble(fields, layout.kFactorDb, sample.kFactorDb);
    readTraceOptionalDouble(fields, layout.shadowFadingDb, sample.shadowFadingDb);
    readTraceOptionalDouble(fields, layout.smallscaleFadingDb, sample.smallscaleFadingDb);
    readTraceOptionalDouble(fields, layout.fingerprintGainDb, sample.fingerprintGainDb);
    readTraceOptionalDouble(fields, layout.fingerprintDelayNs, sample.fingerprintDelayNs);
    readTraceOptionalDouble(fields, layout.fingerprintPhaseRad, sample.fingerprintPhaseRad);
    sample.hasLosFlag = readTraceOptionalBool(fields, layout.losFlag, sample.losFlag);
    if (layout.cdlProfile >= 0 && layout.cdlProfile < static_cast<int>(fields.size())) {
        sample.cdlProfile = fields[layout.cdlProfile];
    }

    return true;
}

void VeinsInet5GVehicleApp::ensureChannelTraceLoaded(const std::string& csvPath)
{
    auto& state = channelTraceState();
    if (state.loaded || state.loadAttempted) return;
    state.loadAttempted = true;

    std::ifstream input(csvPath);
    if (!input.is_open()) return;

    std::string line;
    bool firstLine = true;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        if (firstLine) {
            firstLine = false;
            const auto headerFields = splitCsvLine(line);
            std::map<std::string, int> columns;
            for (int i = 0; i < static_cast<int>(headerFields.size()); ++i) {
                columns[normalizeTraceColumnName(headerFields[i])] = i;
            }

            if (columns.count("tx_id") || columns.count("sender_vehicle_id")) {
                state.layout.legacyOrder = false;
                auto findIndex = [&](std::initializer_list<const char*> names) {
                    for (const char* name : names) {
                        auto it = columns.find(name);
                        if (it != columns.end()) return it->second;
                    }
                    return -1;
                };

                state.layout.txId = findIndex({"tx_id", "sender_vehicle_id"});
                state.layout.rxId = findIndex({"rx_id", "receiver_vehicle_id"});
                state.layout.timeS = findIndex({"time_s"});
                state.layout.distanceM = findIndex({"distance_m"});
                state.layout.rxPowerDb = findIndex({"rx_power_db"});
                state.layout.rssiDbm = findIndex({"rssi_dbm"});
                state.layout.dopplerHz = findIndex({"doppler_hz"});
                state.layout.delaySpreadS = findIndex({"delay_spread_s"});
                state.layout.relativeSpeedMps = findIndex({"relative_speed_mps"});
                state.layout.radialVelocityMps = findIndex({"radial_velocity_mps"});
                state.layout.pathLossDb = findIndex({"path_loss_db"});
                state.layout.losProbability = findIndex({"los_probability"});
                state.layout.losFlag = findIndex({"los_flag"});
                state.layout.cdlProfile = findIndex({"cdl_profile"});
                state.layout.kFactorDb = findIndex({"k_factor_db"});
                state.layout.shadowFadingDb = findIndex({"shadow_fading_db"});
                state.layout.smallscaleFadingDb = findIndex({"smallscale_fading_db"});
                state.layout.fingerprintGainDb = findIndex({"fingerprint_gain_db"});
                state.layout.fingerprintDelayNs = findIndex({"fingerprint_delay_ns"});
                state.layout.fingerprintPhaseRad = findIndex({"fingerprint_phase_rad"});
                continue;
            }
        }

        std::string txId;
        std::string rxId;
        ChannelTraceSample sample;
        if (!parseChannelTraceRow(line, state.layout, txId, rxId, sample)) continue;
        state.samplesByLink[{txId, rxId}].push_back(sample);
    }

    for (auto& kv : state.samplesByLink) {
        auto& rows = kv.second;
        std::sort(rows.begin(), rows.end(), [](const ChannelTraceSample& a, const ChannelTraceSample& b) {
            return a.timeS < b.timeS;
        });
    }

    state.loaded = !state.samplesByLink.empty();
}

const VeinsInet5GVehicleApp::ChannelTraceSample* VeinsInet5GVehicleApp::lookupChannelTraceSample(const std::string& txId,
                                                                                                  const std::string& rxId,
                                                                                                  double timeS) const
{
    const auto& state = channelTraceState();
    if (!state.loaded) return nullptr;

    const auto it = state.samplesByLink.find({txId, rxId});
    if (it == state.samplesByLink.end() || it->second.empty()) return nullptr;

    const auto& rows = it->second;
    auto pos = std::lower_bound(rows.begin(), rows.end(), timeS, [](const ChannelTraceSample& sample, double t) {
        return sample.timeS < t;
    });

    if (pos == rows.begin()) return &(*pos);
    if (pos == rows.end()) return &rows.back();

    const auto& after = *pos;
    const auto& before = *(pos - 1);
    return (std::abs(after.timeS - timeS) < std::abs(timeS - before.timeS)) ? &after : &before;
}

void VeinsInet5GVehicleApp::maybeLogChannelTraceSample(const std::string& txId,
                                                       const std::string& rxId,
                                                       double timeS,
                                                       const char* context) const
{
    if (channelTraceCsv_.empty()) return;

    const auto* sample = lookupChannelTraceSample(txId, rxId, timeS);
    if (!sample) return;

    auto& state = channelTraceState();
    if (!state.logHeaderPrinted) {
        state.logHeaderPrinted = true;
        EV_WARN << "[VeinsInet5GVehicleApp] channel trace lookup active for path=" << channelTraceCsv_ << "\n";
    }

    if (state.loggedLookupCount < 12) {
        state.loggedLookupCount++;
        EV_WARN << "[VeinsInet5GVehicleApp] trace(" << (context ? context : "unknown") << ") "
                << txId << "->" << rxId
                << " t=" << timeS
                << " rxPowerDb=" << sample->rxPowerDb
                << " dopplerHz=" << sample->dopplerHz
                << " delaySpreadS=" << sample->delaySpreadS
                << " distanceM=" << sample->distanceM << "\n";
    }
}

void VeinsInet5GVehicleApp::logCavLinkTrace(inet::Ptr<const VeinsInet5GMessage> payload) const
{
    if (!logCavLinkTrace_ || !payload) return;

    const std::string kind = payload->getMessageKind();
    if (kind != "CAM") return;

    const std::string senderId = payload->getSenderId();
    const std::string receiverId = selfVehicleId_;
    const std::string senderRole = payload->getSenderRole();
    const bool receiverRelevant = isTraceRelevantCavEndpoint(receiverId, senderRole_);
    const bool senderRelevant = isTraceRelevantCavEndpoint(senderId, senderRole);
    if (!receiverRelevant || !senderRelevant) return;

    const long long timeBin = static_cast<long long>(std::llround(simTime().dbl() / kPdrTimeBinSeconds));
    if (!s_cavLinkTraceSeen.emplace(timeBin, senderId, receiverId).second) return;

    ensureCavLinkTraceFile();
    if (!s_cavLinkTraceReady || !s_cavLinkTraceFile.is_open()) return;

    const double timeStep = timeBin * kPdrTimeBinSeconds;
    const inet::Coord receiverPos = currentPosition();
    const double receiverSpeed = currentSpeed();
    double receiverHeadingDeg = headingFromVelocityDegrees(mobility ? mobility->getCurrentVelocity().x : 0.0,
                                                           mobility ? mobility->getCurrentVelocity().y : 0.0);
    if (traciVehicle) {
        try { receiverHeadingDeg = traciVehicle->getAngle(); } catch (...) {}
    }

    const double senderSpeed = std::max(0.0, payload->getSpeed());
    const double senderHeadingDeg = headingFromVelocityDegrees(payload->getVelX(), payload->getVelY());

    s_cavLinkTraceFile << std::fixed << std::setprecision(4)
                       << timeStep << ","
                       << csvEscape(receiverId) << ","
                       << receiverPos.x << ","
                       << receiverPos.y << ","
                       << receiverSpeed << ","
                       << receiverHeadingDeg << ","
                       << csvEscape(senderId) << ","
                       << payload->getPosX() << ","
                       << payload->getPosY() << ","
                       << senderSpeed << ","
                       << senderHeadingDeg << "\n";
}

bool VeinsInet5GVehicleApp::initialiseVehicleState()
{
    try {
        selfVehicleId_ = mobility->getExternalId();
    }
    catch (...) {
        return false;
    }

    if (selfVehicleId_.empty()) return false;

    // Query the actual SUMO vehicle type so the app can distinguish veh_av
    // (CAV) from veh_human (manual) even when both share the same module class.
    // traciVehicle is wired via VeinsInetMobility; by the time this function
    // is reached through the bind-retry loop the TraCI channel is live.
    try {
        if (traciVehicle) {
            sumoVehicleType_ = traciVehicle->getTypeId();
        } else {
            sumoVehicleType_ = cavVehicleType_;   // fallback — should not happen
        }
    }
    catch (...) {
        sumoVehicleType_ = cavVehicleType_;       // fallback if TraCI not yet ready
    }
    // Prefix match so per-vehicle jittered human sub-types sampled from a vTypeDistribution
    // (veh_human_0, veh_human_1, ...) are all recognized as manual, not only exact "veh_human".
    // CAV types (veh_av / veh_av_acc) and veh_attacker do not share the manualVehicleType prefix,
    // so they remain non-manual. Exact single-type setups (e.g. CV2X) match as before.
    manualMode_ = (sumoVehicleType_.rfind(manualVehicleType_, 0) == 0);
    // This node is the high-rate interferer if it matches the configured
    // interferer vehicle ID and the high-rate generator is enabled.
    isAttacker_ = enableFlooding_
        && !configuredAttackerVehicleId_.empty()
        && selfVehicleId_ == configuredAttackerVehicleId_;

    senderRole_ = isAttacker_  ? "ATTACKER"
                : manualMode_  ? "MANUAL"
                :                "CAV";

    if (!manualMode_) {
        try {
            const double rawMaxSpeed = traciVehicle ? traciVehicle->getMaxSpeed() : 13.9;
            normalMaxSpeed_ = (rawMaxSpeed > 0.5 && rawMaxSpeed < 200.0) ? rawMaxSpeed : 13.9;
        }
        catch (...) {
            normalMaxSpeed_ = 13.9;
        }
    }
    else {
        normalMaxSpeed_ = 13.9;
    }
    vehicleBound_ = true;
    return true;
}

void VeinsInet5GVehicleApp::scheduleVehicleBindRetry()
{
    if (!running_ || vehicleBound_) return;

    auto callback = [this]() {
        if (!running_ || vehicleBound_) return;

        if (!initialiseVehicleState()) {
            scheduleVehicleBindRetry();
            return;
        }

        if (logGroundtruth_) {
            ensureGroundTruthJsonFile();
            logSchemaSelfState();
        }

        startTelemetry();

        if (!manualMode_) {
            scheduleCavStartup();
        }

        if (isAttacker_) scheduleFlood();

        EV_INFO << "[VeinsInet5GVehicleApp] bound vehicle=" << selfVehicleId_
                << " type=" << sumoVehicleType_
                << " mode=" << (manualMode_ ? "human/manual" : "CAV")
                << " interferer=" << (isAttacker_ ? "true" : "false")
                << " target=" << (targetVehicleId_.empty() ? "<none>" : targetVehicleId_)
                << " OBU capacity=" << obuCapacityPps_ << " pkt/s"
                << " spatLossPolicy=" << spatLossPolicy_ << "\n";
    };

    timerManager.create(veins::TimerSpecification(callback).oneshotAt(safeFutureTime(SimTime(100, SIMTIME_MS))));
}

void VeinsInet5GVehicleApp::scheduleCavStartup()
{
    if (!running_ || manualMode_) return;

    const double baseDelay = getActualCamInterval();
    const double startupDelay = baseDelay + uniform(0.0, baseDelay);
    auto callback = [this]() {
        if (!running_ || manualMode_) return;
        activateCavMode();
    };
    const simtime_t minimumDelay = secondsToSimTime(getActualCamInterval());
    const simtime_t nextAt = safeFutureTime(secondsToSimTime(startupDelay), minimumDelay);
    EV_WARN << "[VeinsInet5GVehicleApp] scheduleCavStartup vehicle=" << selfVehicleId_
            << " baseDelay=" << baseDelay
            << " startupDelay=" << startupDelay
            << " now=" << simTime()
            << " next=" << nextAt << "\n";
    timerManager.create(veins::TimerSpecification(callback).oneshotAt(nextAt));
}

void VeinsInet5GVehicleApp::activateCavMode()
{
    try {
        if (traciVehicle) traciVehicle->setSpeedMode(CAV_SPEED_MODE);
    }
    catch (...) {
    }

    scheduleCam(true);
    scheduleSpatWatchdog();
}

void VeinsInet5GVehicleApp::startTelemetry()
{
    if (telemetryStarted_) return;
    telemetryStarted_ = true;
    ensureTimeseriesFile();
    ensureSpatResponseFile();
    scheduleTelemetry();
}

void VeinsInet5GVehicleApp::scheduleTelemetry()
{
    if (!running_ || !vehicleBound_) return;

    auto callback = [this]() {
        if (!running_ || !vehicleBound_) return;
        logVehicleTimeseries();
        // Periodic in-sim flush of dist_pdr so the aggregate survives a finish()-time
        // teardown SIGBUS on non-zero seeds. s_distPdrBins is static/shared, so any one
        // node flushing writes the full aggregate; the per-run CSV is overwritten
        // idempotently and the last flush before sim end holds ~complete data.
        if (!s_distPdrBins.empty() && (simTime() - s_lastDistPdrFlush) >= SimTime(2, SIMTIME_S)) {
            s_lastDistPdrFlush = simTime();
            writeDistancePdrCsv();
            // flood_coll rides the same flush: per-module finish() at vehicle despawn would
            // otherwise let the FIRST despawn write a partial file before the attack window.
            writeFloodCollCsv();
        }
        scheduleTelemetry();
    };
    timerManager.create(veins::TimerSpecification(callback).oneshotAt(safeFutureTime(SimTime(100, SIMTIME_MS))));
}

void VeinsInet5GVehicleApp::scheduleCam(bool first)
{
    if (!running_) return;
    const double baseDelay = getActualCamInterval();
    const double delay = first ? baseDelay + uniform(0.0, baseDelay) : baseDelay;
    auto callback = [this]() {
        if (!running_ || manualMode_) return;
        sendCam();
        scheduleCam(false);
    };
    const simtime_t minimumDelay = secondsToSimTime(getActualCamInterval());
    const simtime_t nextAt = safeFutureTime(secondsToSimTime(delay), minimumDelay);
    if (stat_cam_tx_ < 3) {
        EV_WARN << "[VeinsInet5GVehicleApp] scheduleCam vehicle=" << selfVehicleId_
                << " first=" << (first ? "true" : "false")
                << " baseDelay=" << baseDelay
                << " delay=" << delay
                << " now=" << simTime()
                << " next=" << nextAt << "\n";
    }
    timerManager.create(veins::TimerSpecification(callback).oneshotAt(nextAt));
}

void VeinsInet5GVehicleApp::scheduleFlood()
{
    // Only run the high-rate interferer path if it is explicitly enabled.
    if (!running_ || !isAttacker_ || !enableFlooding_ || attackEndedLogged_) return;
    const double interval = getActualFloodInterval();
    double delay = attackActive_ ? interval : std::max(0.0, attackStartTime_ - simTime().dbl());
    if (!attackActive_ && delay <= 0.0)
        delay = interval;
    auto callback = [this]() {
        if (!running_ || !isAttacker_ || !enableFlooding_ || attackEndedLogged_) return;
        const bool targetReady = targetAvailableForFlood();
        if (!targetReady) {
            if (attackActive_ && openEndedAttack_) {
                attackEndedLogged_ = true;
                attackActive_ = false;
                return;
            }
            scheduleFlood();
            return;
        }

        if (!attackActive_) attackActive_ = true;

        if (!openEndedAttack_ && simTime().dbl() >= attackStartTime_ + attackDuration_) {
            attackEndedLogged_ = true;
            attackActive_ = false;
            return;
        }

        sendFloodCam();
        scheduleFlood();
    };
    const simtime_t minimumDelay = secondsToSimTime(getActualFloodInterval());
    timerManager.create(veins::TimerSpecification(callback).oneshotAt(safeFutureTime(secondsToSimTime(delay), minimumDelay)));
}

void VeinsInet5GVehicleApp::scheduleSpatWatchdog()
{
    if (!running_) return;
    auto callback = [this]() {
        if (!running_ || manualMode_) return;
        checkSpatStaleness();
        checkCamCaccFailsafe();
        scheduleSpatWatchdog();
    };
    timerManager.create(veins::TimerSpecification(callback).oneshotAt(safeFutureTime(SimTime(100, SIMTIME_MS))));
}

void VeinsInet5GVehicleApp::sendCam()
{
    send5GMessage("CAM");
    stat_cam_tx_++;
}

bool VeinsInet5GVehicleApp::targetAvailableForFlood() const
{
    if (targetVehicleId_.empty())
        return true;

    auto* manager = veins::FindModule<veins::TraCIScenarioManager*>::findGlobalModule();
    if (!manager)
        return false;

    const inet::Coord ownPos = currentPosition();
    for (const auto& kv : manager->getManagedHosts()) {
        auto* otherMobility = inetMobility(kv.second);
        if (!otherMobility)
            continue;
        if (otherMobility->getExternalId() != targetVehicleId_)
            continue;

        if (targetRange_ <= 0.0)
            return true;

        const double distance = ownPos.distance(otherMobility->getCurrentPosition());
        return distance <= targetRange_;
    }

    return false;
}

std::string VeinsInet5GVehicleApp::resolveAttackTargetId(const char* kind) const
{
    const std::string mode = attackDeliveryMode_;
    const std::string messageKind = kind ? kind : "";

    if (mode == "multicast")
        return "";

    if (mode == "unicast")
        return targetVehicleId_;

    // auto (default): preserve existing semantics
    //  - FLOOD_CAM targets the configured receiver
    if (messageKind == "FLOOD_CAM")
        return targetVehicleId_;

    return "";
}

void VeinsInet5GVehicleApp::sendFloodCam()
{
    if (!targetAvailableForFlood())
        return;

    send5GMessage("FLOOD_CAM", resolveAttackTargetId("FLOOD_CAM"));
    stat_flood_tx_++;
}

void VeinsInet5GVehicleApp::send5GMessage(const char* kind,
                                           const std::string& targetId,
                                           const std::string& senderIdOverride,
                                           double posXOverride,
                                           double posYOverride)
{
    auto payload = makeShared<VeinsInet5GMessage>();
    payload->setChunkLength(B(256));
    payload->setMessageKind(kind);
    payload->setMessageId(nextMessageId_++);

    const std::string senderId = senderIdOverride.empty() ? selfVehicleId_ : senderIdOverride;
    if (!targetId.empty()) {
        maybeLogChannelTraceSample(senderId, targetId, simTime().dbl(), "tx");
    }
    payload->setSenderId(senderId.c_str());
    payload->setTargetId(targetId.c_str());
    payload->setAuthLabel(authModeLabel_.c_str());

    const bool disguisedFlood = disguiseFloodAsCam_ && std::string(kind) == "FLOOD_CAM";
    // Disguised interferer traffic is labelled as ordinary CAV traffic so it is
    // indistinguishable from legitimate load at the receiver.
    const std::string visibleRole = disguisedFlood ? "CAV" : senderRole_;
    const std::string visiblePurpose = disguisedFlood                      ? "awareness"
                                     : (std::string(kind) == "FLOOD_CAM") ? "unsanctioned"
                                     : (std::string(kind) == "SPAT")      ? "signal_control"
                                     :                                       "awareness";
    payload->setSenderRole(visibleRole.c_str());
    payload->setPurposeTag(visiblePurpose.c_str());
    payload->setTimeSent(simTime().dbl());
    payload->setPhase('U');
    payload->setTimeToChange(0.0);

    const inet::Coord pos = currentPosition();
    const inet::Coord velocity = mobility ? mobility->getCurrentVelocity() : inet::Coord::ZERO;
    const double speedNow = currentSpeed();
    // Sentinel -1e38 = use the real position.
    payload->setPosX(posXOverride > -1e37 ? posXOverride : pos.x);
    payload->setPosY(posYOverride > -1e37 ? posYOverride : pos.y);
    payload->setPosZ(pos.z);
    payload->setSpeed(speedNow);
    payload->setVelX(velocity.x);
    payload->setVelY(velocity.y);
    payload->setVelZ(velocity.z);
    payload->setAccel(0.0);
    if (prevTime_ >= 0.0) {
        const double dt = simTime().dbl() - prevTime_;
        if (dt > 0.0) payload->setAccel((speedNow - prevSpeed_) / dt);
    }
    prevSpeed_ = speedNow;
    prevTime_ = simTime().dbl();

    timestampPayload(payload);
    if (logGroundtruth_) {
        logSchemaSelfState();
        logSchemaGroundTruth(payload);
    }
    auto packet = createPacket(kind);
    packet->insertAtBack(payload);
    sendPacket(std::move(packet));
}

void VeinsInet5GVehicleApp::processPacket(std::shared_ptr<inet::Packet> pk)
{
    auto payload = pk->peekAtFront<VeinsInet5GMessage>();
    if (payload->getSenderId() == selfVehicleId_) return;
    if (!isForThisVehicle(payload)) return;
    if (manualMode_ && !isAttacker_) return;

    maybeLogChannelTraceSample(payload->getSenderId(), selfVehicleId_, simTime().dbl(), "rx");

    stat_rx_total_++;
    if (isTargetVehicle()) stat_target_rx_total_++;

    const std::string kind = payload->getMessageKind();
    if (kind == "CAM" || kind == "FLOOD_CAM" || kind == "SPAT") {
        recordPdrAttempt(kind != "SPAT", simTime().dbl());
    }

    if (!passesAnalyticalPdr(payload)) return;
    logCavLinkTrace(payload);
    deliverAfterAirDelay(payload);
}

bool VeinsInet5GVehicleApp::isForThisVehicle(inet::Ptr<const VeinsInet5GMessage> payload) const
{
    const std::string target = payload->getTargetId();
    return target.empty() || target == selfVehicleId_;
}

bool VeinsInet5GVehicleApp::isTargetVehicle() const
{
    return !targetVehicleId_.empty() && selfVehicleId_ == targetVehicleId_;
}

bool VeinsInet5GVehicleApp::passesAnalyticalPdr(inet::Ptr<const VeinsInet5GMessage> payload)
{
    // Distance bin for calibration CSV — 2-D distance (ignore altitude).
    const inet::Coord rxPos = currentPosition();
    const inet::Coord txPos(payload->getPosX(), payload->getPosY(), rxPos.z);
    const double dist2d = rxPos.distance(txPos);
    const int binKey = static_cast<int>(std::floor(dist2d / kDistBinWidthMeters)) * kDistBinWidthMeters;
    DistPdrBin& bin = s_distPdrBins[binKey];
    bin.attempts++;

    // Radio-layer analytical drops: half-duplex, path loss, SPS collision, base
    // drop rate. When bypassAnalyticalChannel is true (Ieee80211 radio handles
    // the channel), these are skipped to avoid double-counting path loss.
    if (!bypassAnalyticalChannel_) {
        // Fairness/verification + drift-diagnosis instrumentation (env-gated, OFF by default):
        // VEINS_STAGE_LOG=<path> logs per-reception stage probabilities at the SAME (distance,
        // density) inputs, so M3 and NS3Learn can be compared stage-by-stage and the density
        // input confirmed identical. Sampled 1-in-10; double-computes the stages only when on.
        if (const char* slp = std::getenv("VEINS_STAGE_LOG")) {
            static std::ofstream slog(slp);
            static long sln = 0;
            static bool shdr = [&]{ if (slog.is_open()) slog << "model,kind,dist_m,n,hd,decode,collision\n"; return true; }();
            (void)shdr;
            if ((sln++ % 10) == 0 && slog.is_open()) {
                const double hdP  = ns3Cascade_ ? cascadeHalfDuplexDrop() : pc5HalfDuplexDrop_;
                const double decP = getLinkPacketSensingRatio(payload);   // reception/decode prob
                const double colP = estimateSpsCollisionDrop(payload);    // collision drop prob
                slog << realismModel_ << ',' << payload->getMessageKind() << ',' << dist2d << ','
                     << countNeighboursInRange() << ',' << hdP << ',' << decP << ',' << colP << '\n';
            }
        }
        // half-duplex stage: cascade scales hd with this node's own transmit rate (so flooding
        // self-blinds); other modes use the static configured drop.
        const double hdDrop = ns3Cascade_ ? cascadeHalfDuplexDrop() : pc5HalfDuplexDrop_;
        if (hdDrop > 0.0 && uniform(0.0, 1.0) < hdDrop) {
            stat_drop_hd_++;
            bin.drop_hd++;
            return false;
        }

        const double linkPsr = getLinkPacketSensingRatio(payload);
        if (linkPsr < 1.0 && uniform(0.0, 1.0) > linkPsr) {
            stat_drop_propagation_++;
            bin.drop_prop++;
            return false;
        }

        const double spsDrop = estimateSpsCollisionDrop(payload);
        if (spsDrop > 0.0 && uniform(0.0, 1.0) < spsDrop) {
            stat_drop_sps_++;
            bin.drop_sps++;
            return false;
        }

        if (pc5DropRate_ > 0.0 && uniform(0.0, 1.0) < pc5DropRate_) {
            stat_drop_base_++;
            bin.drop_base++;
            return false;
        }
    }

    // OBU capacity model always runs — application-layer constraint.
    const double capDrop = capacityDropProbability(getMessageLoadUnits(payload->getMessageKind()));
    if (capDrop > 0.0 && uniform(0.0, 1.0) < capDrop) {
        stat_drop_capacity_++;
        if (isTargetVehicle()) stat_target_drop_capacity_++;
        bin.drop_cap++;
        return false;
    }

    bin.delivered++;
    return true;
}

void VeinsInet5GVehicleApp::deliverAfterAirDelay(inet::Ptr<const VeinsInet5GMessage> payload)
{
    double airDelay = getTraceAdjustedAirDelay(payload);
    if (airDelay < 0.0005) airDelay = 0.0005;

    auto callback = [this, payload]() {
        if (!running_) return;
        const std::string kind = payload->getMessageKind();
        if (kind == "CAM" || kind == "FLOOD_CAM" || kind == "SPAT") {
            recordPdrDelivery(kind != "SPAT", simTime().dbl());
        }
        stat_rx_delivered_++;
        if (isTargetVehicle()) stat_target_rx_delivered_++;
        authenticateAndApplyMessage(payload, simTime().dbl(), currentSpeed());
    };
    timerManager.create(veins::TimerSpecification(callback).oneshotAt(safeFutureTime(secondsToSimTime(airDelay))));
}

void VeinsInet5GVehicleApp::authenticateAndApplyMessage(inet::Ptr<const VeinsInet5GMessage> payload, double timeReceived, double speedAtReceipt)
{
    const std::string kind = payload->getMessageKind();
    if (kind != "CAM" && kind != "FLOOD_CAM" && kind != "SPAT") {
        applyDeliveredMessage(payload, timeReceived, speedAtReceipt, 0.0, "DIRECT_DELIVERY", "DIRECT_DELIVERY", "PEP_DIRECT");
        return;
    }

    const std::string cacheKey = authCacheKeyFor(payload);
    const double timeNow = timeReceived;

    // ── 3. Cache-hit fast path ────────────────────────────────────────────
    if (hasValidAuthCache(cacheKey)) {
        stat_auth_cachehit_++;
        const auto decision = runPolicyDecisionPoint(payload, timeNow, "SESSION_CACHE_HIT", "SESSION_CACHE_HIT");
        runPolicyEnforcementPoint(payload, decision, timeReceived, timeNow, speedAtReceipt, 0.0, cacheKey);
        return;
    }

    // ── 4. Full auth — sample delay and (optionally) queue ───────────────
    stat_auth_full_++;
    const double authDelay = sampleAuthDelay();
    if (authDelay <= 0.0) {
        auto decision = runPolicyDecisionPoint(payload, timeNow, authModeLabel_, "FULL_AUTH_ZERO_DELAY");
        decision.updateAuthCache = decision.accepts();
        decision.trustWindowOverride = authTrustWindowFor(payload);
        runPolicyEnforcementPoint(payload, decision, timeReceived, timeNow, speedAtReceipt, 0.0, cacheKey);
        return;
    }

    // Queue-aware scheduling: returns relative delay accounting for processor backlog.
    // Returns -1.0 if the queue is full — message is dropped (OBU overload).
    const double scheduledDelay = scheduleAuthDelay(authDelay);
    if (scheduledDelay < 0.0) {
        PolicyDecision decision;
        decision.code = PolicyDecisionCode::RejectQueueOverflow;
        decision.authReason = "REJECT_QUEUE_OVERFLOW";
        decision.cacheStatus = "FULL_AUTH_QUEUE_OVERFLOW";
        decision.decisionStage = "PEP_RESOURCE_GUARD";
        runPolicyEnforcementPoint(payload, decision, timeReceived, timeNow, speedAtReceipt, authDelay, cacheKey);
        return;
    }

    auto callback = [this, payload, cacheKey, timeReceived, speedAtReceipt, authDelay]() {
        if (!running_) return;

        const double actedAt = simTime().dbl();
        auto decision = runPolicyDecisionPoint(payload, actedAt, authModeLabel_, "FULL_AUTH_DELAYED");
        decision.updateAuthCache = decision.accepts();
        decision.trustWindowOverride = authTrustWindowFor(payload);
        runPolicyEnforcementPoint(payload, decision, timeReceived, actedAt, speedAtReceipt, authDelay, cacheKey);
    };
    timerManager.create(veins::TimerSpecification(callback).oneshotAt(safeFutureTime(secondsToSimTime(scheduledDelay))));
}

void VeinsInet5GVehicleApp::applyDeliveredMessage(inet::Ptr<const VeinsInet5GMessage> payload,
                                                  double timeReceived,
                                                  double speedAtReceipt,
                                                  double configuredDelay,
                                                  const std::string& authReason,
                                                  const std::string& cacheStatus,
                                                  const std::string& decisionStage)
{
    const double appliedTime = simTime().dbl();
    logLatencyBreakdown(payload,
                        true,
                        timeReceived,
                        appliedTime,
                        appliedTime,
                        speedAtReceipt,
                        configuredDelay,
                        authReason,
                        cacheStatus,
                        decisionStage);

    const std::string kind = payload->getMessageKind();

    // Leader-CAM freshness for the car-following app. Recorded HERE, on the accepted-
    // delivery path, not at air-delay delivery: a CAM that survives the channel but is
    // then discarded by the OBU processing queue (flooding-induced overload) or rejected
    // rejected by the auth gate never reaches the CACC controller, so it must not count as fresh.
    // FLOOD_CAM counts when accepted: a flood disguised as a CAM is indistinguishable to
    // the receiver, so if the attacker happens to be the ego's leader its traffic does
    // sustain the link — that is the modelled reality, not an artefact.
    if (enableCamCacc_ && (kind == "CAM" || kind == "FLOOD_CAM")) {
        lastCamFromVeh_[payload->getSenderId()] = simTime().dbl();
    }

    if (kind == "CAM") {
        stat_cam_rx_++;
        if (logGroundtruth_) {
            logSchemaSelfState();
            logSchemaReceived(payload);
        }
        return;
    }
    if (kind == "FLOOD_CAM") {
        stat_flood_rx_++;
        if (logGroundtruth_) {
            logSchemaSelfState();
            logSchemaReceived(payload);
        }
        return;
    }
    if (kind == "SPAT") {
        stat_spat_rx_++;
        // Multi-junction networks: a vehicle hears SPaT from several signals within range.
        // Only the SPaT for the signal it is actually approaching (its next TLS, tracked in
        // lastObservedTlsId_) is authoritative for control and fail-safe freshness; SPaT for
        // other junctions is logged but must not drive this vehicle. (Single-intersection
        // scenarios match trivially, so behavior there is unchanged.)
        const std::string msgTlsId = payload->getTlsId();
        const bool relevantSpat = !lastObservedTlsId_.empty() &&
                                  (msgTlsId.empty() || msgTlsId == lastObservedTlsId_);
        // Any SPaT heard for a signal proves that signal is V2I-equipped. Recorded in the
        // shared, run-scoped registry even when the message is for a junction this vehicle is
        // not currently approaching, so equipage is learned once for the whole fleet rather
        // than rediscovered per vehicle per approach.
        if (!msgTlsId.empty()) spatEquippedTls().insert(msgTlsId);

        if (relevantSpat) {
            lastSpatReceivedAt_ = simTime().dbl();
            // Record freshness against the specific signal, so the fail-safe can tell
            // "this junction's SPaT went silent" from "this junction never had SPaT".
            const std::string tls = msgTlsId.empty() ? lastObservedTlsId_ : msgTlsId;
            lastSpatFromTls_[tls] = lastSpatReceivedAt_;
            if (!tls.empty()) spatEquippedTls().insert(tls);
            spatControlActive_ = true;
            recoverFromAccFallback();   // cooperative link restored -> return CACC before reasserting control
            if (enableSpatControl_) applySpat(payload->getPhase());
        }
        logSpatResponse(payload, timeReceived, simTime().dbl(), speedAtReceipt, configuredDelay, authReason);
    }
}

void VeinsInet5GVehicleApp::applySpat(char phase)
{
    if (phase == 'G') {
        setSafeMaxSpeed(normalMaxSpeed_ > 0.0 ? normalMaxSpeed_ : 13.9);
        frozenSpeed_ = -1.0;
        return;
    }
    if (phase != 'R' && phase != 'Y') return;

    // Red/yellow: shape a decel-limited approach so the CAV comes to rest AT the stop line.
    //
    // This previously clamped the vehicle's max speed to kMinimumTraciStopSpeed on ANY red,
    // at ANY distance, held until a green SPaT arrived — so a CAV 200 m from a red light was
    // brought to a standstill. Measured effect: 24.9% of CAV samples at v<0.1 m/s in the
    // no-attack baseline (and 12.6% of humans, queued behind them). Because losing SPaT
    // RELEASES the cap, the flood then looked beneficial: attacked runs flowed 13% faster
    // than the baseline, inverting the sign of the whole experiment. The clamp was also
    // redundant — SUMO enforces red lights natively, so it added nothing to compliance.
    const double vTarget = stopLineApproachSpeed();
    if (vTarget < 0.0) {
        // inside the junction box, past the line, or committed in the dilemma zone -> clear it
        restoreSumoBehavior();
        return;
    }
    setSafeMaxSpeed(vTarget);
    frozenSpeed_ = vTarget;
}

double VeinsInet5GVehicleApp::stopLineApproachSpeed(double* distanceOut) const
{
    if (distanceOut) *distanceOut = -1.0;
    // Never hold a vehicle inside the conflict zone.
    std::string roadId;
    try { roadId = traciVehicle ? traciVehicle->getRoadId() : ""; } catch (...) {}
    if (!roadId.empty() && roadId[0] == ':') return -2.0;

    double dLine = -1.0;
    try {
        for (const auto& t : traciVehicle->getNextTls()) {
            const double dd = std::get<2>(t);
            if (dd >= 0.0) { dLine = dd; break; }
        }
    } catch (...) {}
    if (distanceOut) *distanceOut = dLine;
    if (dLine < 0.0) return -2.0;          // stop line already behind us

    const double v = std::max(0.0, currentSpeed());
    const double a = std::max(0.5, spatStopDecel_);
    const double dNeed = (v * v) / (2.0 * a);
    if (dLine <= dNeed + kStopLineMargin) return -1.0;   // dilemma zone: commit and clear

    // Cap so we come to rest kStopLineMargin short of the line. Recomputed every watchdog
    // tick, which yields a smooth ~a m/s^2 profile rather than a step to zero.
    const double dRem = std::max(0.0, dLine - kStopLineMargin);
    double vTarget = std::min(v, std::sqrt(2.0 * a * dRem));
    if (vTarget < kMinimumTraciStopSpeed) vTarget = kMinimumTraciStopSpeed;
    return vTarget;
}

std::set<std::string>& VeinsInet5GVehicleApp::spatEquippedTls()
{
    static std::set<std::string> equipped;
    return equipped;
}

void VeinsInet5GVehicleApp::checkSpatStaleness()
{
    spatLinkState_ = -1;          // default: no SPaT-equipped signal ahead -> not applicable
    if (!enableSpatControl_) return;
    // No signal ahead (passed all TLS, or next-TLS not yet acquired) -> nothing to obey:
    // release any degraded control and idle the watchdog until a new next-TLS appears.
    if (lastObservedTlsId_.empty()) {
        recoverFromAccFallback();
        restoreSumoBehavior();
        spatControlActive_ = false;
        return;
    }
    // Note the moment this signal became the next-TLS, so an equipped junction we have heard
    // NOTHING from still has a freshness reference (see spatEquippedTls()).
    if (lastObservedTlsId_ != tlsAcquiredId_) {
        tlsAcquiredId_ = lastObservedTlsId_;
        tlsAcquiredAt_ = simTime().dbl();
    }

    // Is this junction SPaT-equipped at all? Infrastructure equipage is static and map-known,
    // so it is judged against the shared registry, NOT against this vehicle's own reception
    // history. An unequipped junction is not an app failure — there is nothing to fall back
    // from, so it is approached conventionally; without this, every unequipped junction in a
    // partly equipped network fires a spurious MRM on every approach. Using per-vehicle
    // history here instead would let a jammer suppress the fallback entirely by denying every
    // SPaT, which is the opposite of the intended behaviour.
    if (!spatEquippedTls().count(lastObservedTlsId_)) {
        recoverFromAccFallback();
        restoreSumoBehavior();
        return;
    }

    // Freshness for THIS signal. If the CAV has never personally received SPaT from it (the
    // jamming case), the clock runs from when the junction came into view.
    const auto itSpat = lastSpatFromTls_.find(lastObservedTlsId_);
    const double reference = (itSpat != lastSpatFromTls_.end()) ? itSpat->second : tlsAcquiredAt_;
    if (reference < 0.0) return;
    const double spatAge = simTime().dbl() - reference;
    spatLinkState_ = (spatAge <= getEffectiveSpatStaleTimeout()) ? 0 : 1;
    if (spatLinkState_ == 0) return;

    // ── Application-differentiated mode: SPaT loss disables the intersection-assist
    // app. The response is the DDT-fallback ladder CACC -> ACC -> MRM, run below.
    // The car-following app (and hence CACC<->ACC on leader-CAM freshness) is handled
    // independently by checkCamCaccFailsafe(); spatLossPolicy is not consulted here.
    if (enableCamCacc_) { runIntersectionAssistFallback(spatAge); return; }

    // Empty policy reproduces the pre-failsafe behavior: keep the last
    // SPAT-commanded TraCI speed limit latched until a fresh SPAT arrives.
    if (spatLossPolicy_.empty()) return;

    // ── "sumo_fallback": CACC -> ACC graceful degradation (SAE J3161/1 C-V2X / SR-CACC).
    // On loss of the cooperative SPaT link the CAV drops to onboard-sensor ACC and stays
    // there until a fresh SPaT restores cooperation. spatControlActive_ stays true so the
    // watchdog keeps firing and recoverFromAccFallback() returns CACC on recovery.
    if (spatLossPolicy_ == "sumo_fallback") {
        degradeToAcc();
        return;
    }

    // ── "safe_stop": SAE J3016 DDT fallback -> minimal-risk maneuver (MRM). Same ladder
    // as the application-differentiated mode above (ACC during the handover window, then
    // the decel-limited stop at the stop line), so both share one implementation.
    // spatControlActive_ stays true: the watchdog keeps firing and recovers on fresh SPaT.
    if (spatLossPolicy_ == "safe_stop") {
        runIntersectionAssistFallback(spatAge);
        return;
    }

    // ── "freeze_speed": hold last commanded speed ceiling ────────────────────────
    if (frozenSpeed_ < 0.0 || !std::isfinite(frozenSpeed_)) {
        const double speedNow = currentSpeed();
        frozenSpeed_ = std::isfinite(speedNow) ? std::max(kMinimumTraciStopSpeed, speedNow) : kMinimumTraciStopSpeed;
    }
    setSafeMaxSpeed(frozenSpeed_);
}

void VeinsInet5GVehicleApp::restoreSumoBehavior()
{
    frozenSpeed_ = -1.0;
    setSafeMaxSpeed(-1.0);
}

double VeinsInet5GVehicleApp::getMrmTriggerTimeout() const
{
    // Time after the last SPaT before the MRM commits: one message interval (loss
    // detection) + the SAE J3016 DDT-fallback handover window. Matches the design
    // formula "frequency + handover" (e.g. 0.1 s + 1 s with the default handover).
    // Never below the loss-detection gate, or the MRM would be unreachable: the caller only
    // gets here once spatAge already exceeds getEffectiveSpatStaleTimeout().
    return std::max(getEffectiveSpatStaleTimeout(),
                    std::max(0.1, actualSpatInterval_) + std::max(0.0, spatHandoverTime_));
}

std::string VeinsInet5GVehicleApp::failsafeState() const
{
    if (mrmStopped_) return "MRM";
    if (accReasonCam_ || accReasonSpat_) return "ACC";
    return "CACC";
}

std::string VeinsInet5GVehicleApp::failsafeReason() const
{
    const bool v2i = accReasonSpat_ || spatLossActive_;
    if (accReasonCam_ && v2i) return "V2I+V2V";
    if (v2i) return "V2I";                // SPaT lost -> intersection assist; the only MRM cause
    if (accReasonCam_) return "V2V";      // leader CAM lost -> cooperative car-following only
    return "";
}

void VeinsInet5GVehicleApp::applyCarFollowingMode()
{
    // Sole owner of the CACC/ACC vType. ACC is held while EITHER degradation reason
    // stands (leader-CAM loss or SPaT loss), so the two watchdogs cannot fight over
    // setType(): each only sets its own reason flag and calls this arbiter. Idempotent —
    // TraCI is touched only when the resolved mode differs from what is applied.
    if (!traciVehicle || manualMode_) return;
    // In the application-differentiated model the car-following mode answers to the CAM link
    // alone; SPaT loss arms the MRM instead of switching the vType. The legacy spatLossPolicy
    // path still uses accReasonSpat_ to drive the switch.
    const bool wantAcc = enableCamCacc_ ? accReasonCam_ : (accReasonCam_ || accReasonSpat_);
    if (wantAcc == accModeApplied_) return;
    const std::string& target = wantAcc ? spatAccTypeId_ : spatCaccTypeId_;
    if (target.empty()) return;
    try { traciVehicle->setType(target); } catch (...) { return; }
    accModeApplied_ = wantAcc;
    // Keep the logged vType in step with the live one: it is captured once at bind time,
    // so without this the timeseries would report CACC throughout and any headway analysis
    // split by vehicle_type would silently compare the wrong populations.
    sumoVehicleType_ = target;
}

void VeinsInet5GVehicleApp::degradeToAcc()
{
    // SAE J3161/1 C-V2X / SR-CACC graceful degradation: on loss of the cooperative
    // SPaT link the CAV falls back to onboard-sensor ACC car-following.
    // recoverFromAccFallback() reverses it on fresh SPaT. Runs once per loss episode:
    // the cap must not be released again on later ticks or it would fight the MRM, which
    // re-applies its own cap every tick once the handover window has elapsed.
    if (accReasonSpat_) return;
    accReasonSpat_ = true;
    applyCarFollowingMode();
    restoreSumoBehavior();   // release the SPaT speed cap so the ACC model drives
}

void VeinsInet5GVehicleApp::recoverFromAccFallback()
{
    // Fresh authenticated SPaT restored the cooperative link: clear the SPaT reason and
    // end any MRM. The vType only returns to CACC if the leader-CAM reason is also clear.
    mrmEngaged_ = false;
    mrmStopped_ = false;
    spatLossActive_ = false;
    if (!accReasonSpat_) return;
    accReasonSpat_ = false;
    applyCarFollowingMode();
}

void VeinsInet5GVehicleApp::runIntersectionAssistFallback(double spatAge)
{
    // Intersection-assist failure response. Responses map one-to-one onto applications:
    //   ACC <- leader-CAM (V2V) loss, handled in checkCamCaccFailsafe()
    //   MRM <- SPaT (V2I) loss, handled here
    // There is NO ACC handover stage on this path. The loss-confirmation window is itself the
    // grace period: five consecutive missed SPaT at 10 Hz (getEffectiveSpatStaleTimeout) is
    // enough evidence that the intersection-assist application is gone, after which the CAV
    // decelerates to a halt. Removing the handover also removes most of the coupling between
    // the two responses - previously the SPaT path forced a vType switch, so enabling or
    // disabling ACC changed how fast a vehicle traversed the approach and hence whether the
    // MRM had time to commit at all.
    if (!failsafeMrm_) {
        spatLossActive_ = false;
        restoreSumoBehavior();
        return;
    }

    spatLossActive_ = true;    // arms the MRM and marks V2I in failsafe_reason; no vType change
    if (spatAge <= getMrmTriggerTimeout()) return;
    runMinimalRiskManeuver();
}

void VeinsInet5GVehicleApp::runMinimalRiskManeuver()
{
    // Decel-limited stop AT the stop line, with the two "keep going" exemptions:
    // already inside the junction box, or the stop line is already behind us.
    // Never a speed clamp: the target speed is recomputed each 100 ms watchdog tick from
    // the remaining distance, which yields a ~spatStopDecel profile instead of the
    // emergency braking a hard setMaxSpeed(0) would provoke from SUMO.

    // Shares stopLineApproachSpeed() with the SPaT red/yellow response, so the MRM and normal
    // signal compliance produce the same decel-limited profile and differ only in WHY they run.
    // Negative sentinels mean "release": inside the junction box, the stop line already behind
    // us (continue in ACC rather than stopping past the line), or the dilemma zone.
    double dLine = -1.0;
    const double vTarget = stopLineApproachSpeed(&dLine);
    distToStopLine_ = dLine;
    if (vTarget < 0.0) {
        mrmEngaged_ = false;
        mrmStopped_ = false;
        restoreSumoBehavior();
        return;
    }
    // Outside the activation zone the manoeuvre would command a cap far above free-flow speed
    // and change nothing, so it is not engaged and the vehicle is not labelled MRM.
    if (dLine > mrmActivationDistance_) {
        mrmEngaged_ = false;
        mrmStopped_ = false;
        return;
    }
    mrmEngaged_ = true;
    // Reported state flips to MRM only at standstill; until then the vehicle is rolling down
    // the decel-limited approach on onboard sensors, which IS ACC.
    mrmStopped_ = (currentSpeed() <= kMrmRestSpeed);
    setSafeMaxSpeed(vTarget);
    frozenSpeed_ = vTarget;
}

void VeinsInet5GVehicleApp::checkCamCaccFailsafe()
{
    // Car-following / forward-collision-warning app, fed by the leader's V2V CAM.
    // Loss of that CAM costs the CAV its cooperative capability only: it degrades to
    // onboard-sensor ACC and keeps driving, still seeing the vehicle ahead. This is
    // location independent — the intersection-assist app (SPaT) and its MRM are handled
    // separately in checkSpatStaleness/runIntersectionAssistFallback.
    if (!enableCamCacc_ || manualMode_ || !running_ || !traciVehicle) return;
    const double now = simTime().dbl();

    // Leader directly ahead + the freshness of ITS CAM. A non-CAV leader never appears
    // in lastCamFromVeh_, so it reads as stale -> CACC requires a connected leader.
    std::string leaderId;
    try {
        auto ld = traciVehicle->getLeader(caccLeaderRange_);
        leaderId = (ld.second >= 0.0) ? ld.first : std::string();
    } catch (...) { leaderId.clear(); }
    bool camFresh;
    if (leaderId.empty()) {
        camFresh = true;                     // no one ahead -> nominal CACC (free driving)
        camLinkState_ = -1;                  // not applicable, NOT "healthy"
    } else {
        auto it = lastCamFromVeh_.find(leaderId);
        camFresh = (it != lastCamFromVeh_.end() && (now - it->second) <= camStaleTimeout_);
        camLinkState_ = camFresh ? 0 : 1;     // observed regardless of activateFailsafe
    }

    // Car-following app: the leader's CAM freshness is this watchdog's only output. It
    // records its reason and defers the vType to the arbiter, so a concurrent SPaT-side
    // degradation cannot be silently undone here (and vice versa).
    // `activateFailsafe` without ACC means V2V loss costs nothing observable: the CAV keeps
    // its cooperative car-following model even with a stale leader CAM.
    accReasonCam_ = failsafeAcc_ && !camFresh;
    applyCarFollowingMode();

    // No stop logic here. Losing the car-following app away from an intersection must not
    // stop the vehicle: ACC still tracks the leader, so it keeps cruising. Stopping is the
    // intersection-assist failure response and lives in runIntersectionAssistFallback(),
    // which reaches it on SPaT loss alone and therefore also covers the platoon leader
    // (no vehicle ahead), which the previous combined-blind rule could never trigger for.
}

void VeinsInet5GVehicleApp::logVehicleTimeseries()
{
    if (!running_ || !vehicleBound_) return;

    // Next-TLS observation FIRST: lastObservedTlsId_/lastObservedTlsDistance_ gate the
    // SPaT fail-safe, so they must refresh every tick even if the CSV cannot be opened.
    // Any violation detected is latched and consumed by the row writer below.
    {
        int tlsViolationNow = 0;
        updateTlsObservation(tlsViolationNow);
        if (tlsViolationNow) pendingTlsViolation_ = 1;
    }

    ensureTimeseriesFile();
    if (!s_timeseriesFile.is_open()) return;

    const double currentTime = simTime().dbl();
    const double speed = currentSpeed();
    double acceleration = 0.0;
    if (timeseriesPrevSpeed_ >= 0.0 && timeseriesPrevTime_ >= 0.0) {
        const double dt = currentTime - timeseriesPrevTime_;
        if (dt > 0.0) acceleration = (speed - timeseriesPrevSpeed_) / dt;
    }
    timeseriesPrevSpeed_ = speed;
    timeseriesPrevTime_ = currentTime;

    std::string leaderId;
    double gap = -1.0;
    try {
        if (traciVehicle) {
            auto leaderInfo = traciVehicle->getLeader(50.0);
            gap = leaderInfo.second;
            leaderId = gap >= 0.0 ? leaderInfo.first : "";
        }
    }
    catch (...) {
        gap = -1.0;
        leaderId.clear();
    }

    const int egoIsCav = manualMode_ ? 0 : 1;
    const int leaderIsCav = lookupLeaderIsCav(leaderId);
    const int involvesCav = egoIsCav ? 1 : leaderIsCav;
    const double leaderSpeed = lookupLeaderSpeed(leaderId);
    double ttc = -1.0;
    if (gap >= 0.0) {
        const double relativeSpeed = speed - leaderSpeed;
        if (relativeSpeed > 0.0) ttc = gap / relativeSpeed;
    }

    const inet::Coord pos = currentPosition();
    const double cx = pos.x;
    const double cy = pos.y;

    int hardBrakeFlag = 0;
    int emergencyBrakeFlag = 0;
    if (!inBrakingEpisode_) {
        if (acceleration <= kBrakeStartThreshold && speed > kStopSpeedThreshold) {
            inBrakingEpisode_ = true;
            brakeEpisodeDist_ = 0.0;
            brakeEpisodeMaxDecel_ = acceleration;
            brakeLastX_ = cx;
            brakeLastY_ = cy;
            hardBrakeFired_ = false;
            emergencyBrakeFired_ = false;
        }
    }
    else {
        const double dx = cx - brakeLastX_;
        const double dy = cy - brakeLastY_;
        brakeEpisodeDist_ += std::sqrt((dx * dx) + (dy * dy));
        brakeLastX_ = cx;
        brakeLastY_ = cy;
        if (acceleration < brakeEpisodeMaxDecel_) brakeEpisodeMaxDecel_ = acceleration;

        if (!hardBrakeFired_ && brakeEpisodeMaxDecel_ <= kHardBrakeThreshold) {
            hardBrakeFired_ = true;
            hardBrakeFlag = 1;
        }
        if (!emergencyBrakeFired_ && brakeEpisodeMaxDecel_ <= kEmergencyBrakeThreshold) {
            emergencyBrakeFired_ = true;
            emergencyBrakeFlag = 1;
        }

        if (speed <= kStopSpeedThreshold || acceleration >= kBrakeEndThreshold) {
            inBrakingEpisode_ = false;
        }
    }
    const double brakingDistance = inBrakingEpisode_ ? brakeEpisodeDist_ : 0.0;

    // ── Traffic conflict detection (Hydén 1987 / ETSI TR 103 415 §6.3.2) ────────
    // Each flag fires ONCE at the start of a new episode, identical pattern to
    // hard_brake_event / emergency_brake_event. Both reset when TTC recovers.
    //
    //  traffic_conflict  (1 per episode) — TTC drops below 1.5 s (serious conflict)
    //  safety_conflict   (1 per episode) — TTC drops below 3.0 s (any conflict)
    //
    // Previous "collision" metric (vehicle stopped >10 s within 1 m gap) fired
    // every timestep and counted normal queue behaviour as crashes — removed.
    int trafficConflictFlag = 0;  // serious conflict: TTC < 1.5 s
    int safetyConflictFlag  = 0;  // any conflict:    TTC < 3.0 s

    const bool hasTtc = (ttc > 0.0 && std::isfinite(ttc));

    if (hasTtc && ttc <= kConflictTtcS) {
        if (!inConflictEpisode_) {
            // Entering conflict zone — start new episode, fire safety_conflict once.
            inConflictEpisode_ = true;
            seriousConflictFiredInEpisode_ = false;
            safetyConflictFlag = 1;
        }
        if (!seriousConflictFiredInEpisode_ && ttc <= kSeriousConflictTtcS) {
            // TTC has crossed into serious range for first time this episode.
            seriousConflictFiredInEpisode_ = true;
            trafficConflictFlag = 1;
        }
    }
    else {
        // TTC recovered (or no leader) — close episode so next drop starts fresh.
        inConflictEpisode_ = false;
        seriousConflictFiredInEpisode_ = false;
    }

    const int tlsViolationEvent = pendingTlsViolation_;
    pendingTlsViolation_ = 0;

    s_timeseriesFile << std::fixed << std::setprecision(4)
                     << veins::getCurrentRunCsvPrefix()
                     << currentTime << ","
                     << csvEscape(selfVehicleId_) << ","
                     << egoIsCav << ","
                     << csvEscape(sumoVehicleType_) << ","
                     << cx << ","
                     << cy << ","
                     << speed << ","
                     << acceleration << ","
                     << (gap < 0.0 ? -1.0 : gap) << ","
                     << csvEscape(leaderId) << ","
                     << leaderIsCav << ","
                     << ttc << ","
                     << brakingDistance << ","
                     << hardBrakeFlag << ","
                     << emergencyBrakeFlag << ","
                     << trafficConflictFlag << ","
                     << safetyConflictFlag << ","
                     << tlsViolationEvent << ","
                     << 0 << ","
                     << 0 << ","
                     << 0 << ","
                     << involvesCav << ","
                     << (accReasonCam_ ? 1 : 0) << ","
                     << (mrmStopped_ ? 1 : 0) << ","
                     << csvEscape(lastObservedTlsId_) << ","
                     << failsafeState() << ","
                     << failsafeReason() << ","
                     << camLinkState_ << ","
                     << spatLinkState_ << ","
                     << distToStopLine_ << "\n";
    s_timeseriesFile.flush();

    // safe_stop probe: record each CAV's distance to the next stop line + speed, to eyeball
    // where vehicles come to rest under SPaT loss. Gated by VEINS_SAFESTOP_PROBE (off by default).
    if (s_safeStopProbeEnabled && egoIsCav && traciVehicle) {
        if (!s_safeStopProbeReady) {
            s_safeStopProbeFile.open(schemaOutputDir() + "safestop_probe_" + currentRunTimestamp() + ".csv");
            if (s_safeStopProbeFile.is_open()) {
                s_safeStopProbeReady = true;
                s_safeStopProbeFile << "time,vehicle_id,speed,dist_to_stopline,road_id\n";
            }
        }
        if (s_safeStopProbeReady) {
            double dLine = -1.0; std::string roadId;
            try {
                roadId = traciVehicle->getRoadId();
                for (const auto& t : traciVehicle->getNextTls()) {
                    const double dd = std::get<2>(t);
                    if (dd >= 0.0) { dLine = dd; break; }
                }
            } catch (...) {}
            s_safeStopProbeFile << std::fixed << std::setprecision(3)
                                << currentTime << "," << csvEscape(selfVehicleId_) << ","
                                << speed << "," << dLine << "," << csvEscape(roadId) << "\n";
            s_safeStopProbeFile.flush();
        }
    }
}

void VeinsInet5GVehicleApp::updateTlsObservation(int& tlsViolationEvent)
{
    tlsViolationEvent = 0;
    if (!running_ || !vehicleBound_ || !traciVehicle || !mobility) return;

    std::string currentRoadId;
    try {
        currentRoadId = traciVehicle->getRoadId();
    }
    catch (...) {
        return;
    }

    const bool onInternalEdge = !currentRoadId.empty() && currentRoadId[0] == ':';
    const bool wasOnInternalEdge = !lastRoadId_.empty() && lastRoadId_[0] == ':';

    if (!onInternalEdge) {
        tlsViolationLatched_ = false;
        lastObservedTlsId_.clear();
        lastObservedTlsSignal_ = 'U';
        lastObservedTlsDistance_ = -1.0;

        try {
            for (const auto& nextTls : traciVehicle->getNextTls()) {
                const std::string tlsId = std::get<0>(nextTls);
                const double distanceToTls = std::get<2>(nextTls);
                const char signal = normalizeTlsSignal(static_cast<char>(std::get<3>(nextTls)));
                if (tlsId.empty() || distanceToTls < 0.0) continue;
                lastObservedTlsId_ = tlsId;
                lastObservedTlsSignal_ = signal;
                lastObservedTlsDistance_ = distanceToTls;
                break;
            }
        }
        catch (...) {
        }
    }
    else if (!wasOnInternalEdge) {
        const bool enteredOnRed = (lastObservedTlsSignal_ == 'R' || lastObservedTlsSignal_ == 'Y');
        const bool closeEnough = (lastObservedTlsDistance_ >= 0.0 &&
                                  lastObservedTlsDistance_ <= kTlsViolationDistanceThreshold);
        if (enteredOnRed && closeEnough && !tlsViolationLatched_) {
            tlsViolationEvent = 1;
            tlsViolationLatched_ = true;
        }
    }

    lastRoadId_ = currentRoadId;
}

int VeinsInet5GVehicleApp::lookupLeaderIsCav(const std::string& leaderId) const
{
    if (leaderId.empty() || !traci) return 0;

    try {
        const std::string lt = traci->vehicle(leaderId).getTypeId();
        return (lt == cavVehicleType_ || lt == spatAccTypeId_) ? 1 : 0;   // CACC or degraded-ACC CAV
    }
    catch (...) {
        return 0;
    }
}

double VeinsInet5GVehicleApp::lookupLeaderSpeed(const std::string& leaderId) const
{
    if (leaderId.empty() || !traci) return 0.0;

    try {
        return traci->vehicle(leaderId).getSpeed();
    }
    catch (...) {
        return 0.0;
    }
}

void VeinsInet5GVehicleApp::logSpatResponse(inet::Ptr<const VeinsInet5GMessage> payload,
                                            double timeReceived,
                                            double timeActed,
                                            double speedAtReceipt,
                                            double configuredDelay,
                                            const std::string& authReason) const
{
    ensureSpatResponseFile();
    if (!s_spatResponseFile.is_open()) return;

    const double actualDelay = std::max(0.0, timeActed - timeReceived);
    const double extraDistance = std::max(0.0, speedAtReceipt) * actualDelay;
    const long stepAttempts = getLoggedStepAttempts(false, timeReceived);
    const long stepDeliveries = getLoggedStepDeliveries(false, timeReceived);
    const double stepPdr = getLoggedStepPdr(false, timeReceived);
    const std::string resolvedAuthReason = authReason.empty() ? authModeLabel_ : authReason;

    s_spatResponseFile << std::fixed << std::setprecision(6)
                       << veins::getCurrentRunCsvPrefix()
                       << csvEscape(selfVehicleId_) << ","
                       << csvEscape(payload->getTlsId()) << ","
                       << payload->getPhase() << ","
                       << payload->getTimeToChange() << ","
                       << payload->getTimeSent() << ","
                       << timeReceived << ","
                       << timeActed << ","
                       << speedAtReceipt << ","
                       << configuredDelay << ","
                       << actualDelay << ","
                       << extraDistance << ","
                       << csvEscape(resolvedAuthReason) << ","
                       << stepAttempts << ","
                       << stepDeliveries << ","
                       << stepPdr << "\n";
    s_spatResponseFile.flush();
}

void VeinsInet5GVehicleApp::logSpatAuthDecision(inet::Ptr<const VeinsInet5GMessage> payload,
                                                const PolicyDecision& decision,
                                                double timeReceived,
                                                double decisionTime,
                                                double speedAtReceipt,
                                                double configuredDelay) const
{
    if (std::string(payload->getMessageKind()) != "SPAT") return;

    ensureSpatAuthDecisionFile();
    if (!s_spatAuthDecisionFile.is_open()) return;

    const inet::Coord ownPos = currentPosition();
    const inet::Coord senderPos(payload->getPosX(), payload->getPosY(), payload->getPosZ());
    const double dx = ownPos.x - senderPos.x;
    const double dy = ownPos.y - senderPos.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double actualDecisionDelay = std::max(0.0, decisionTime - timeReceived);

    const char* pdpDecision = decision.accepts() ? "ACCEPT" : "REJECT";
    const char* pepAction = decision.accepts() ? "DELIVER" : "DROP";
    const char* decisionReason = "UNKNOWN";
    const char* standardRef = "IEEE 1609.2; ETSI TS 103 097";

    switch (decision.code) {
    case PolicyDecisionCode::Accept:
        decisionReason = "ALLOW";
        standardRef = "IEEE 1609.2; ETSI TS 103 097";
        break;
    case PolicyDecisionCode::RejectQueueOverflow:
        decisionReason = "QUEUE_OVERFLOW";
        standardRef = "OBU processing-queue overflow";
        break;
    case PolicyDecisionCode::RejectStale:
        decisionReason = "STALE_MESSAGE";
        standardRef = "IEEE 1609.2 §6.3.9; ETSI TS 103 097";
        break;
    }

    s_spatAuthDecisionFile << std::fixed << std::setprecision(6)
                           << veins::getCurrentRunCsvPrefix()
                           << csvEscape(selfVehicleId_) << ","
                           << csvEscape(manualMode_ ? "MANUAL" : senderRole_) << ","
                           << csvEscape(payload->getSenderId()) << ","
                           << csvEscape(payload->getMessageKind()) << ","
                           << payload->getMessageId() << ","
                           << csvEscape(payload->getTargetId()) << ","
                           << csvEscape(payload->getTlsId()) << ","
                           << payload->getPhase() << ","
                           << csvEscape(payload->getSenderRole()) << ","
                           << csvEscape(payload->getPurposeTag()) << ","
                           << csvEscape(payload->getAuthLabel()) << ","
                           << csvEscape(decision.cacheStatus) << ","
                           << (decision.accepts() ? 1 : 0) << ","
                           << pdpDecision << ","
                           << pepAction << ","
                           << csvEscape(decisionReason) << ","
                           << csvEscape(standardRef) << ","
                           << payload->getTimeSent() << ","
                           << timeReceived << ","
                           << decisionTime << ","
                           << configuredDelay << ","
                           << actualDecisionDelay << ","
                           << speedAtReceipt << ","
                           << distance << ","
                           << payload->getTimeToChange() << ","
                           << authFreshnessWindow_ << ","
                           << authTrustWindowFor(payload) << "\n";
    s_spatAuthDecisionFile.flush();
}

void VeinsInet5GVehicleApp::logLatencyBreakdown(inet::Ptr<const VeinsInet5GMessage> payload,
                                                bool accepted,
                                                double timeReceived,
                                                double decisionTime,
                                                double appliedTime,
                                                double speedAtReceipt,
                                                double configuredDelay,
                                                const std::string& authReason,
                                                const std::string& cacheStatus,
                                                const std::string& decisionStage) const
{
    if (!logLatencyBreakdown_ || !payload) return;

    ensureLatencyBreakdownFile();
    if (!s_latencyBreakdownFile.is_open()) return;

    const inet::Coord ownPos = currentPosition();
    const inet::Coord senderPos(payload->getPosX(), payload->getPosY(), payload->getPosZ());
    const double dx = ownPos.x - senderPos.x;
    const double dy = ownPos.y - senderPos.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    const double timeSent = payload->getTimeSent();
    const double radioDelay = std::max(0.0, timeReceived - timeSent);
    const double verificationPathDelay = std::max(0.0, decisionTime - timeReceived);
    const double queueWaitDelay = std::max(0.0, verificationPathDelay - std::max(0.0, configuredDelay));
    const double pepToApplyDelay = std::max(0.0, appliedTime - decisionTime);
    const double endToEndDecisionDelay = std::max(0.0, decisionTime - timeSent);
    const double endToEndApplyDelay = std::max(0.0, appliedTime - timeSent);

    s_latencyBreakdownFile << std::fixed << std::setprecision(6)
                           << veins::getCurrentRunCsvPrefix()
                           << csvEscape(selfVehicleId_) << ","
                           << csvEscape(manualMode_ ? "MANUAL" : senderRole_) << ","
                           << csvEscape(payload->getSenderId()) << ","
                           << csvEscape(payload->getMessageKind()) << ","
                           << payload->getMessageId() << ","
                           << csvEscape(payload->getTargetId()) << ","
                           << csvEscape(payload->getSenderRole()) << ","
                           << csvEscape(payload->getPurposeTag()) << ","
                           << csvEscape(payload->getAuthLabel()) << ","
                           << csvEscape(cacheStatus) << ","
                           << csvEscape(decisionStage) << ","
                           << csvEscape(authReason) << ","
                           << (accepted ? 1 : 0) << ","
                           << timeSent << ","
                           << timeReceived << ","
                           << decisionTime << ","
                           << appliedTime << ","
                           << radioDelay << ","
                           << verificationPathDelay << ","
                           << std::max(0.0, configuredDelay) << ","
                           << queueWaitDelay << ","
                           << pepToApplyDelay << ","
                           << endToEndDecisionDelay << ","
                           << endToEndApplyDelay << ","
                           << speedAtReceipt << ","
                           << distance << "\n";
    s_latencyBreakdownFile.flush();
}

VeinsInet5GVehicleApp::PolicyDecision VeinsInet5GVehicleApp::runPolicyDecisionPoint(inet::Ptr<const VeinsInet5GMessage> payload,
                                                                                     double decisionTime,
                                                                                     const std::string& authReason,
                                                                                     const std::string& cacheStatus)
{
    PolicyDecision decision;
    decision.code = PolicyDecisionCode::Accept;
    decision.authReason = authReason;
    decision.cacheStatus = cacheStatus;
    decision.decisionStage = "PDP";

    // Message freshness (IEEE 1609.2 §6.3.9 / ETSI TS 103 097): a message whose
    // end-to-end latency exceeds authFreshnessWindow_ is too old to act on.
    if (isMessageStale(payload->getTimeSent(), decisionTime)) {
        decision.code = PolicyDecisionCode::RejectStale;
        decision.authReason = "REJECT_STALE";
        return decision;
    }

    return decision;
}

bool VeinsInet5GVehicleApp::runPolicyEnforcementPoint(inet::Ptr<const VeinsInet5GMessage> payload,
                                                      const PolicyDecision& decision,
                                                      double timeReceived,
                                                      double decisionTime,
                                                      double speedAtReceipt,
                                                      double configuredDelay,
                                                      const std::string& cacheKey)
{
    logSpatAuthDecision(payload, decision, timeReceived, decisionTime, speedAtReceipt, configuredDelay);

    if (!decision.accepts()) {
        logLatencyBreakdown(payload,
                            false,
                            timeReceived,
                            decisionTime,
                            decisionTime,
                            speedAtReceipt,
                            configuredDelay,
                            decision.authReason,
                            decision.cacheStatus,
                            decision.decisionStage);
        recordPolicyDecisionCounters(decision);
        return false;
    }

    if (decision.updateAuthCache && !cacheKey.empty()) {
        updateAuthCache(cacheKey, decisionTime, decision.trustWindowOverride);
    }

    applyDeliveredMessage(payload,
                          timeReceived,
                          speedAtReceipt,
                          configuredDelay,
                          decision.authReason,
                          decision.cacheStatus,
                          decision.decisionStage);
    return true;
}

void VeinsInet5GVehicleApp::recordPolicyDecisionCounters(const PolicyDecision& decision)
{
    switch (decision.code) {
    case PolicyDecisionCode::Accept:
        return;
    case PolicyDecisionCode::RejectQueueOverflow:
        stat_auth_queue_overflow_++;
        return;
    case PolicyDecisionCode::RejectStale:
        stat_auth_stale_drop_++;
        return;
    }
}

void VeinsInet5GVehicleApp::setSafeMaxSpeed(double speed)
{
    try {
        if (!traciVehicle || manualMode_) return;

        std::string roadId = traciVehicle ? traciVehicle->getRoadId() : "";
        if (!roadId.empty() && roadId[0] == ':') return;

        double safeSpeed = speed;
        if (!std::isfinite(safeSpeed) || safeSpeed < 0.0) safeSpeed = normalMaxSpeed_ > 0.0 ? normalMaxSpeed_ : 13.9;
        if (safeSpeed >= 0.0) safeSpeed = std::max(kMinimumTraciStopSpeed, std::min(safeSpeed, kMaximumTraciSpeed));

        traciVehicle->setMaxSpeed(safeSpeed);
    }
    catch (const omnetpp::cRuntimeError& e) {
        EV_WARN << "[VeinsInet5GVehicleApp] setSafeMaxSpeed(" << speed
                << ") rejected for vehicle " << selfVehicleId_
                << ": " << e.what() << "\n";
    }
    catch (...) {
        EV_WARN << "[VeinsInet5GVehicleApp] setSafeMaxSpeed(" << speed
                << ") failed for vehicle " << selfVehicleId_ << "\n";
    }
}

double VeinsInet5GVehicleApp::getPacketRateHz() const
{
    if (beaconInterval_ > 0.0) return 1.0 / beaconInterval_;
    return 10.0;
}

double VeinsInet5GVehicleApp::getActualCamInterval() const
{
    return std::max(0.05, actualCamInterval_);
}

double VeinsInet5GVehicleApp::getActualFloodInterval() const
{
    const double limitedRate = std::max(0.1, std::min(floodRate_, maxActualFloodRate_));
    return 1.0 / limitedRate;
}

double VeinsInet5GVehicleApp::getEffectiveSpatStaleTimeout() const
{
    return std::max(spatStaleTimeout_, 1.5 * std::max(0.1, actualSpatInterval_));
}

double VeinsInet5GVehicleApp::getMessageLoadUnits(const std::string& /*kind*/) const
{
    // Each received message represents exactly one ECDSA verification operation
    // regardless of message type (IEEE 1609.2 §6.3 — per-message certificate check).
    // obuCapacityPps is therefore directly interpretable as crypto-ops/second,
    // calibrated to the Cohda Wireless MK6C ECDSA-P256 path (~600 ops/s).
    return 1.0;
}

double VeinsInet5GVehicleApp::getPc5NoiseFloorDbm() const
{
    const double bandwidthHz = std::max(1.0, pc5BandwidthHz_);
    return -174.0 + (10.0 * std::log10(bandwidthHz)) + pc5NoiseFigureDb_;
}

double VeinsInet5GVehicleApp::sampleRuntimeShadowFadingDb(const std::string& txId,
                                                          const std::string& rxId,
                                                          double timeS,
                                                          bool losFlag) const
{
    const double sigmaDb = losFlag ? 4.0 : 7.82;
    const double epochS = std::max(0.1, pc5ShadowCorrelationWindowS_);
    const long long epoch = static_cast<long long>(std::floor((timeS / epochS) + 1e-9));
    const std::string a = txId < rxId ? txId : rxId;
    const std::string b = txId < rxId ? rxId : txId;
    const std::string key = a + "|" + b + "|" + std::to_string(epoch) + "|" + (losFlag ? "LOS" : "NLOS");
    return sigmaDb * gaussianFromHash(fnv1aHash64(key));
}

double VeinsInet5GVehicleApp::sampleFadingGainDb(const std::string& cdlProfile,
                                                    const std::string& txId,
                                                    const std::string& rxId,
                                                    double timeS,
                                                    double distanceM) const
{
    const auto& state = generalizedPhyRuntimeState();

    // Coherence time for small-scale fading at 5.9 GHz, typical V2X speed (60 km/h):
    //   f_D ≈ 328 Hz  →  T_c ≈ 1/(2·f_D) ≈ 1.5 ms.  We use 2 ms so adjacent slots
    //   share the same draw without updating too aggressively.
    constexpr double kFadingCoherenceS = 0.002;
    const long long epoch = static_cast<long long>(std::floor(timeS / kFadingCoherenceS + 1e-9));
    const std::string& a = txId < rxId ? txId : rxId;
    const std::string& b = txId < rxId ? rxId : txId;
    const std::string normProfile = normalizeProfileKey(cdlProfile);
    const std::string hashKey = a + "|" + b + "|" + std::to_string(epoch) + "|FADING|" + normProfile;
    const uint64_t seed = fnv1aHash64(hashKey);

    // Look up Toolbox-calibrated distribution parameters.
    ChannelGainDistParam params;
    auto it = state.gainDistByProfile.find(normProfile);
    if (it != state.gainDistByProfile.end()) {
        params = it->second;
    }
    else if (normProfile == "CDL-E") {
        // CDL-E rarely appears in training data; use 3GPP TR 38.901 CDL-E spec values:
        //   K ≈ 22 dB, Ω ≈ CDL-D mean power.
        params.distType             = "rician";
        params.rician_k_lin         = std::pow(10.0, 22.0 / 10.0);  // ~158
        params.mean_lin             = 1.547;
        params.omega_log10_intercept = 0.173;   // CDL-D fitted intercept (best available)
        params.omega_log10_slope    = -0.001;
        params.hasDistanceModel     = true;
    }
    else {
        // Unknown profile: return 0 dB (no small-scale correction).
        return 0.0;
    }

    // Compute distance-conditioned mean power Ω(d) = 10^(a + b·log10(d)).
    // If no distance model is available (old CSV), fall back to global mean.
    double omega;
    if (params.hasDistanceModel && std::isfinite(distanceM) && distanceM >= 1.0) {
        const double log10d = std::log10(distanceM);
        omega = std::pow(10.0, params.omega_log10_intercept + params.omega_log10_slope * log10d);
    }
    else {
        omega = params.mean_lin;
    }
    omega = std::max(omega, 1e-15);

    double powerLin;
    if (params.distType == "rician") {
        // Rician: X = (ν + σ·G₁)² + (σ·G₂)²
        //   σ² = Ω(d)/(2(K+1)),  ν = √(K·Ω(d)/(K+1))
        const double K      = std::max(0.0, params.rician_k_lin);
        const double sigma2 = omega / (2.0 * (K + 1.0));
        const double sigma  = std::sqrt(sigma2);
        const double nu     = std::sqrt(K * omega / (K + 1.0));
        const double g1     = gaussianFromHash(seed ^ 0xA3B5C7D9E1F20348ULL);
        const double g2     = gaussianFromHash(seed ^ 0x1234567890ABCDEFULL);
        const double comp_a = nu + sigma * g1;
        const double comp_b = sigma * g2;
        powerLin = comp_a * comp_a + comp_b * comp_b;
    }
    else {
        // Rayleigh: power ~ Exponential(mean = Ω(d)).
        // Inverse CDF: P = -Ω(d)·ln(u),  u ~ Uniform(0,1).
        const double u = uniformFromHash(seed);
        powerLin = -omega * std::log(u);
    }

    powerLin = std::max(powerLin, 1e-15);
    return 10.0 * std::log10(powerLin);
}

double VeinsInet5GVehicleApp::interpolateBlerCurve(const GeneralizedPhyBlerCurve& curve, double snrDb)
{
    if (!curve.loaded || curve.snrDb.empty()) return std::numeric_limits<double>::quiet_NaN();
    if (snrDb <= curve.snrDb.front()) return curve.bler.front();
    if (snrDb >= curve.snrDb.back()) return curve.bler.back();

    auto upper = std::lower_bound(curve.snrDb.begin(), curve.snrDb.end(), snrDb);
    if (upper == curve.snrDb.begin()) return curve.bler.front();
    if (upper == curve.snrDb.end()) return curve.bler.back();

    const size_t hi = static_cast<size_t>(std::distance(curve.snrDb.begin(), upper));
    const size_t lo = hi - 1;
    const double x0 = curve.snrDb[lo];
    const double x1 = curve.snrDb[hi];
    const double y0 = curve.bler[lo];
    const double y1 = curve.bler[hi];
    if (std::abs(x1 - x0) <= 1e-12) return y0;
    const double w = (snrDb - x0) / (x1 - x0);
    return y0 + (w * (y1 - y0));
}

double VeinsInet5GVehicleApp::getPacketSensingRatio(double distanceMeters) const
{
    const std::string_view model(v2vPropagationModel_);
    const double d = std::max(distanceMeters, kReferenceDistanceMeters);
    if (model == "tr38901_umi" || model == "tr38901_uma") {
        const bool uma = model == "tr38901_uma";
        const double pLos = get3gppUrbanLosProbability(d, uma);
        const double plLos = get3gppUrbanPathLossDb(d, d, uma, true, spatUtHeight_, spatUtHeight_);
        const double plNlos = get3gppUrbanPathLossDb(d, d, uma, false, spatUtHeight_, spatUtHeight_);
        const double sigmaLos = 4.0;
        const double sigmaNlos = uma ? 6.0 : 7.82;
        const auto psrFor = [&](double pathLossDb, double sigmaDb) {
            const double numerator = pc5TxPowerDbm_ - pathLossDb - pc5SensingThresholdDbm_;
            if (sigmaDb <= 0.0) return numerator >= 0.0 ? 1.0 : 0.0;
            return clamp01(0.5 * (1.0 + std::erf(numerator / (sigmaDb * std::sqrt(2.0)))));
        };
        if (losOnly_) return psrFor(plLos, sigmaLos);   // LOS-only: drop building-NLOS branch
        return clamp01((pLos * psrFor(plLos, sigmaLos)) + ((1.0 - pLos) * psrFor(plNlos, sigmaNlos)));
    }

    const double pathLossDb = pc5ReferenceLossDb_ + 10.0 * pc5PathLossExponent_ * std::log10(d / kReferenceDistanceMeters);
    const double numerator = pc5TxPowerDbm_ - pathLossDb - pc5SensingThresholdDbm_;
    if (pc5ShadowingStdDb_ <= 0.0) return numerator >= 0.0 ? 1.0 : 0.0;
    return clamp01(0.5 * (1.0 + std::erf(numerator / (pc5ShadowingStdDb_ * std::sqrt(2.0)))));
}

double VeinsInet5GVehicleApp::getLinkPacketSensingRatio(inet::Ptr<const VeinsInet5GMessage> payload) const
{
    if (ns3LearnEnabled_) {
        const inet::Coord rxPos = currentPosition();
        const inet::Coord txPos(payload->getPosX(), payload->getPosY(), payload->getPosZ());
        return ns3LearnReception(std::max(rxPos.distance(txPos), kReferenceDistanceMeters),
                                 payload->getSenderId(), txPos.x - rxPos.x, txPos.y - rxPos.y);
    }

    // Combined Analytical Reference: drop_prop = 1 - g(SNR(d)) (Rehman, distance).
    // For the contention-only ablation, Model 2 is disabled (no propagation loss).
    if (analyticalM3Enabled_) {
        if (!m3UseDecode_) return 1.0;
        const inet::Coord rxPos = currentPosition();
        const inet::Coord txPos(payload->getPosX(), payload->getPosY(), payload->getPosZ());
        return rehmanDecodeProb(rxPos.distance(txPos));
    }

    if (activateGeneralizedPhyModel_ && !generalizedPhyCoeffDir_.empty()) {
        ChannelTraceSample generalizedSample;
        if (buildGeneralizedPhySample(payload, generalizedSample)) {
            return getTraceAdjustedPacketSensingRatio(generalizedSample);
        }
    }

    if (activateChannelTrace_ && !channelTraceCsv_.empty()) {
        if (const auto* sample = lookupChannelTraceSample(payload->getSenderId(), selfVehicleId_, simTime().dbl())) {
            return getTraceAdjustedPacketSensingRatio(*sample);
        }
    }

    const inet::Coord receiverPos = currentPosition();
    const inet::Coord senderPos(payload->getPosX(), payload->getPosY(), payload->getPosZ());
    const double distance2d = std::max(receiverPos.distance(inet::Coord(senderPos.x, senderPos.y, receiverPos.z)), kReferenceDistanceMeters);
    const double distance3d = std::max(receiverPos.distance(senderPos), kReferenceDistanceMeters);

    const bool isSpat = std::string(payload->getMessageKind()) == "SPAT";
    const std::string_view model(isSpat ? spatPropagationModel_ : v2vPropagationModel_);
    if (model != "tr38901_umi" && model != "tr38901_uma") {
        return getPacketSensingRatio(distance3d);
    }

    const bool uma = model == "tr38901_uma";
    const double txHeight = isSpat ? spatBsHeight_ : spatUtHeight_;
    const double rxHeight = spatUtHeight_;
    const double pLos = get3gppUrbanLosProbability(distance2d, uma);
    const double plLos = get3gppUrbanPathLossDb(distance2d, distance3d, uma, true, txHeight, rxHeight);
    const double plNlos = get3gppUrbanPathLossDb(distance2d, distance3d, uma, false, txHeight, rxHeight);
    const double sigmaLos = 4.0;
    const double sigmaNlos = uma ? 6.0 : 7.82;

    const auto psrFor = [&](double pathLossDb, double sigmaDb) {
        const double numerator = pc5TxPowerDbm_ - pathLossDb - pc5SensingThresholdDbm_;
        if (sigmaDb <= 0.0) return numerator >= 0.0 ? 1.0 : 0.0;
        return clamp01(0.5 * (1.0 + std::erf(numerator / (sigmaDb * std::sqrt(2.0)))));
    };

    const double psrLos = psrFor(plLos, sigmaLos);
    const double psrNlos = psrFor(plNlos, sigmaNlos);
    return clamp01((pLos * psrLos) + ((1.0 - pLos) * psrNlos));
}

VeinsInet5GVehicleApp::GeneralizedPhyPredictors VeinsInet5GVehicleApp::deriveGeneralizedPhyPredictors(inet::Ptr<const VeinsInet5GMessage> payload) const
{
    GeneralizedPhyPredictors predictors;

    const inet::Coord receiverPos = currentPosition();
    const inet::Coord senderPos(payload->getPosX(), payload->getPosY(), receiverPos.z);
    const double dx = receiverPos.x - senderPos.x;
    const double dy = receiverPos.y - senderPos.y;
    const double distance2d = std::max(std::sqrt((dx * dx) + (dy * dy)), kReferenceDistanceMeters);
    predictors.log10DistanceM = std::log10(distance2d);

    const inet::Coord receiverVel = mobility ? mobility->getCurrentVelocity() : inet::Coord::ZERO;
    const double senderVx = payload->getVelX();
    const double senderVy = payload->getVelY();
    const double relVx = senderVx - receiverVel.x;
    const double relVy = senderVy - receiverVel.y;
    predictors.relativeSpeedMps = std::sqrt((relVx * relVx) + (relVy * relVy));

    const double ux = dx / distance2d;
    const double uy = dy / distance2d;
    predictors.radialVelocityMps = (relVx * ux) + (relVy * uy);
    predictors.absRadialVelocityMps = std::abs(predictors.radialVelocityMps);

    double receiverHeadingDeg = headingFromVelocityDegrees(receiverVel.x, receiverVel.y);
    if (traciVehicle) {
        try { receiverHeadingDeg = traciVehicle->getAngle(); } catch (...) {}
    }
    const double senderHeadingDeg = headingFromVelocityDegrees(senderVx, senderVy);
    predictors.headingAlignmentCos = std::cos((senderHeadingDeg - receiverHeadingDeg) * 3.14159265358979323846 / 180.0);

    const bool uma = v2vPropagationModel_ == "tr38901_uma";
    predictors.losProbability = losOnly_ ? 1.0 : get3gppUrbanLosProbability(distance2d, uma);   // LOS-only: force LOS channel-gain branch
    return predictors;
}

double VeinsInet5GVehicleApp::evaluateGeneralizedPhyModel(const GeneralizedPhyLinearModel& model,
                                                          const GeneralizedPhyPredictors& predictors) const
{
    if (!model.loaded) return std::numeric_limits<double>::quiet_NaN();

    double value = 0.0;
    for (const auto& entry : model.coefficients) {
        const auto& term = entry.first;
        const double coeff = entry.second;
        if (term == "(Intercept)") value += coeff;
        else if (term == "log10_distance_m") value += coeff * predictors.log10DistanceM;
        else if (term == "relative_speed_mps") value += coeff * predictors.relativeSpeedMps;
        else if (term == "radial_velocity_mps") value += coeff * predictors.radialVelocityMps;
        else if (term == "abs_radial_velocity_mps") value += coeff * predictors.absRadialVelocityMps;
        else if (term == "heading_alignment_cos") value += coeff * predictors.headingAlignmentCos;
        else if (term == "los_probability") value += coeff * predictors.losProbability;
    }
    return value;
}

bool VeinsInet5GVehicleApp::buildGeneralizedPhySample(inet::Ptr<const VeinsInet5GMessage> payload, ChannelTraceSample& sample) const
{
    const auto& state = generalizedPhyRuntimeState();
    if (!state.loaded) return false;

    const auto predictors = deriveGeneralizedPhyPredictors(payload);
    const double distanceM = std::pow(10.0, predictors.log10DistanceM);
    const bool losFlag = predictors.losProbability >= 0.5;
    const double pathLossDb = evaluateGeneralizedPhyModel(state.pathLossModel, predictors);
    const double rssiDbm = evaluateGeneralizedPhyModel(state.rssiModel, predictors);
    const double delayLog10Ns = evaluateGeneralizedPhyModel(state.delaySpreadLog10Model, predictors);
    const double kFactorDb = evaluateGeneralizedPhyModel(state.kFactorModel, predictors);

    // Small-scale fading: draw from the Toolbox-calibrated per-profile distribution
    // (Rician for LOS profiles CDL-D/E, exponential/Rayleigh for NLOS CDL-A/C).
    // This replaces the OLS surrogate, which had R²≈0.26 because small-scale fading
    // is inherently stochastic and cannot be predicted from geometry alone.
    const std::string cdlProfileForFading = selectRuntimeGeneralizedCdlProfile(
        std::pow(10.0, predictors.log10DistanceM), losFlag);
    const double channelGainDb = state.hasGainDistParams
        ? sampleFadingGainDb(cdlProfileForFading, payload->getSenderId(), selfVehicleId_, simTime().dbl(), distanceM)
        : evaluateGeneralizedPhyModel(
              state.hasSplitChannelGainModel
                  ? (losFlag ? state.channelGainLosModel : state.channelGainNlosModel)
                  : state.channelGainModel,
              predictors);

    const double shadowFadingDb = sampleRuntimeShadowFadingDb(payload->getSenderId(), selfVehicleId_, simTime().dbl(), losFlag);
    double effectiveRxPowerDb = std::numeric_limits<double>::quiet_NaN();
    if (std::isfinite(pathLossDb) && std::isfinite(channelGainDb)) {
        effectiveRxPowerDb = pc5TxPowerDbm_ - pathLossDb - shadowFadingDb + channelGainDb;
    }
    else {
        effectiveRxPowerDb = rssiDbm;
    }

    sample.timeS = simTime().dbl();
    sample.distanceM = distanceM;
    sample.relativeSpeedMps = predictors.relativeSpeedMps;
    sample.radialVelocityMps = predictors.radialVelocityMps;
    sample.dopplerHz = (predictors.radialVelocityMps / 299792458.0) * (spatCarrierFrequencyGHz_ * 1e9);
    sample.pathLossDb = pathLossDb;
    sample.rxPowerDb = effectiveRxPowerDb;
    sample.delaySpreadS = std::pow(10.0, delayLog10Ns) * 1e-9;
    sample.losProbability = predictors.losProbability;
    sample.hasLosFlag = true;
    sample.losFlag = losFlag;
    sample.kFactorDb = kFactorDb;
    sample.shadowFadingDb = shadowFadingDb;
    sample.smallscaleFadingDb = channelGainDb;
    sample.cdlProfile = selectRuntimeGeneralizedCdlProfile(distanceM, sample.losFlag);
    return std::isfinite(sample.rxPowerDb) && std::isfinite(sample.delaySpreadS);
}

double VeinsInet5GVehicleApp::getTraceAdjustedPacketSensingRatio(const ChannelTraceSample& sample) const
{
    double effectiveRxPowerDb = sample.rxPowerDb;
    if (!std::isfinite(effectiveRxPowerDb) && std::isfinite(sample.pathLossDb)) {
        effectiveRxPowerDb = pc5TxPowerDbm_ - sample.pathLossDb;
        if (std::isfinite(sample.shadowFadingDb)) effectiveRxPowerDb -= sample.shadowFadingDb;
        if (std::isfinite(sample.smallscaleFadingDb)) effectiveRxPowerDb += sample.smallscaleFadingDb;
        if (std::isfinite(sample.fingerprintGainDb)) effectiveRxPowerDb += sample.fingerprintGainDb;
    }
    if (!std::isfinite(effectiveRxPowerDb)) {
        return getPacketSensingRatio(std::isfinite(sample.distanceM) ? sample.distanceM : pc5Range_);
    }

    const auto& state = generalizedPhyRuntimeState();
    const std::string profileKey = normalizeProfileKey(sample.cdlProfile);
    const auto curveIt = state.blerCurvesByProfile.find(profileKey);
    if (state.hasBlerCurves && curveIt != state.blerCurvesByProfile.end()) {
        const double sinrDb = effectiveRxPowerDb - getPc5NoiseFloorDbm();
        const double bler = interpolateBlerCurve(curveIt->second, sinrDb);
        if (std::isfinite(bler)) {
            return clamp01(1.0 - bler);
        }
    }

    const double numerator = effectiveRxPowerDb - pc5SensingThresholdDbm_;
    double sigmaDb = std::max(0.5, pc5ShadowingStdDb_);
    if (std::isfinite(sample.smallscaleFadingDb)) sigmaDb += std::min(1.5, std::abs(sample.smallscaleFadingDb) * 0.10);
    if (sigmaDb <= 0.0) return numerator >= 0.0 ? 1.0 : 0.0;
    return clamp01(0.5 * (1.0 + std::erf(numerator / (sigmaDb * std::sqrt(2.0)))));
}

double VeinsInet5GVehicleApp::getTraceAdjustedAirDelay(inet::Ptr<const VeinsInet5GMessage> payload) const
{
    double meanDelay = pc5AirDelayMean_;
    double stdDelay = std::max(0.0, pc5AirDelayStd_);

    if (activateGeneralizedPhyModel_ && !generalizedPhyCoeffDir_.empty()) {
        ChannelTraceSample generalizedSample;
        if (buildGeneralizedPhySample(payload, generalizedSample)) {
            const double dopplerFactor = std::isfinite(generalizedSample.dopplerHz) ? std::min(2.0, std::abs(generalizedSample.dopplerHz) / 400.0) : 0.0;
            const double delayFactor = std::isfinite(generalizedSample.delaySpreadS) ? std::min(2.0, std::max(0.0, generalizedSample.delaySpreadS) / 2.5e-7) : 0.0;
            const double nlosPenalty = generalizedSample.hasLosFlag && !generalizedSample.losFlag ? 1.0 : 0.0;
            double cdlPenalty = 0.0;
            if (generalizedSample.cdlProfile == "CDL-E") cdlPenalty = 0.75;
            else if (generalizedSample.cdlProfile == "CDL-C") cdlPenalty = 0.35;
            else if (generalizedSample.cdlProfile == "CDL-D") cdlPenalty = 0.20;

            meanDelay += 0.00015 * dopplerFactor;
            meanDelay += 0.00025 * delayFactor;
            meanDelay += 0.00015 * nlosPenalty;
            meanDelay += 0.00010 * cdlPenalty;
            stdDelay = std::max(stdDelay, 0.0002);
            stdDelay *= (1.0 + (0.25 * dopplerFactor) + (0.50 * delayFactor) + (0.25 * nlosPenalty) + (0.20 * cdlPenalty));
        }
    }
    else if (activateChannelTrace_ && !channelTraceCsv_.empty()) {
        if (const auto* sample = lookupChannelTraceSample(payload->getSenderId(), selfVehicleId_, simTime().dbl())) {
            const double dopplerFactor = std::isfinite(sample->dopplerHz) ? std::min(2.0, std::abs(sample->dopplerHz) / 400.0) : 0.0;
            const double delayFactor = std::isfinite(sample->delaySpreadS) ? std::min(2.0, std::max(0.0, sample->delaySpreadS) / 2.5e-7) : 0.0;
            const double nlosPenalty = sample->hasLosFlag && !sample->losFlag ? 1.0 : 0.0;
            double cdlPenalty = 0.0;
            if (sample->cdlProfile == "CDL-E") cdlPenalty = 0.75;
            else if (sample->cdlProfile == "CDL-C") cdlPenalty = 0.35;
            else if (sample->cdlProfile == "CDL-D") cdlPenalty = 0.20;

            meanDelay += 0.00015 * dopplerFactor;
            meanDelay += 0.00025 * delayFactor;
            meanDelay += 0.00015 * nlosPenalty;
            meanDelay += 0.00010 * cdlPenalty;
            stdDelay = std::max(stdDelay, 0.0002);
            stdDelay *= (1.0 + (0.25 * dopplerFactor) + (0.50 * delayFactor) + (0.25 * nlosPenalty) + (0.20 * cdlPenalty));
        }
    }

    double airDelay = truncnormal(meanDelay, stdDelay);
    if (!std::isfinite(airDelay)) airDelay = meanDelay;
    return std::max(airDelay, 0.0005);
}

double VeinsInet5GVehicleApp::get3gppUrbanLosProbability(double distanceMeters, bool uma) const
{
    const double d = std::max(distanceMeters, 1.0);
    if (d <= 18.0) return 1.0;
    const double decay = uma ? 63.0 : 36.0;
    return clamp01((18.0 / d) + std::exp(-d / decay) * (1.0 - (18.0 / d)));
}

double VeinsInet5GVehicleApp::get3gppUrbanPathLossDb(double distance2dMeters, double distance3dMeters, bool uma, bool los, double txHeightMeters, double rxHeightMeters) const
{
    const double d2d = std::max(distance2dMeters, 10.0);
    const double d3d = std::max(distance3dMeters, 10.0);
    const double fc = std::max(0.5, spatCarrierFrequencyGHz_);
    const double hBs = std::max(1.0, txHeightMeters);
    const double hUt = std::max(1.0, rxHeightMeters);
    const double c = 3.0e8;
    const double dBp = (4.0 * hBs * hUt * (fc * 1.0e9)) / c;

    double plLos = 0.0;
    if (uma) {
        if (d2d <= dBp) plLos = 28.0 + 22.0 * std::log10(d3d) + 20.0 * std::log10(fc);
        else plLos = 28.0 + 40.0 * std::log10(d3d) + 20.0 * std::log10(fc) - 9.0 * std::log10((dBp * dBp) + ((hBs - hUt) * (hBs - hUt)));
        if (los) return plLos;
        const double plNlos = 13.54 + 39.08 * std::log10(d3d) + 20.0 * std::log10(fc) - 0.6 * (hUt - 1.5);
        return std::max(plLos, plNlos);
    }

    if (d2d <= dBp) plLos = 32.4 + 21.0 * std::log10(d3d) + 20.0 * std::log10(fc);
    else plLos = 32.4 + 40.0 * std::log10(d3d) + 20.0 * std::log10(fc) - 9.5 * std::log10((dBp * dBp) + ((hBs - hUt) * (hBs - hUt)));
    if (los) return plLos;
    const double plNlos = 22.4 + 35.3 * std::log10(d3d) + 21.3 * std::log10(fc) - 0.3 * (hUt - 1.5);
    return std::max(plLos, plNlos);
}

double VeinsInet5GVehicleApp::getTotalResourcePool() const
{
    const double lambda = std::max(1e-6, getPacketRateHz());
    const double totalResources = (kMsPerSecond * std::max(1.0, pc5SubchannelsPerSubframe_)) / lambda;
    return std::max(1.0, totalResources / std::max(1.0, pc5ResourcesPerPacket_));
}

double VeinsInet5GVehicleApp::estimateChannelBusyRatio() const
{
    auto* manager = veins::FindModule<veins::TraCIScenarioManager*>::findGlobalModule();
    if (!manager) return 0.0;

    const inet::Coord receiverPos = currentPosition();
    double sensedVehicles = 0.0;
    for (const auto& kv : manager->getManagedHosts()) {
        cModule* host = kv.second;
        if (!host || host == getParentModule() || !inet5GApp(host)) continue;
        auto* otherMobility = inetMobility(host);
        if (!otherMobility) continue;

        const double distance = receiverPos.distance(otherMobility->getCurrentPosition());
        if (pc5Range_ > 0.0 && distance > pc5Range_) continue;
        sensedVehicles += getPacketSensingRatio(distance);
    }

    const double totalResources = getTotalResourcePool();
    double excludedResources = sensedVehicles / 2.0;
    const int halfSensed = static_cast<int>(std::floor(sensedVehicles / 2.0));
    for (int k = 1; k <= halfSensed; ++k) {
        const double denominator = totalResources - (sensedVehicles / 2.0);
        if (denominator <= 0.0) break;
        excludedResources += std::max(1.0 - (static_cast<double>(k) / denominator), 0.0);
    }
    return clamp01(excludedResources / totalResources);
}

double VeinsInet5GVehicleApp::estimateSpsCollisionDrop(inet::Ptr<const VeinsInet5GMessage> payload) const
{
    if (ns3LearnEnabled_) return ns3LearnCollisionDrop();

    // Combined Analytical Reference: drop_sps = P_COL(n) (Cao, density). For the
    // propagation-only ablation, Model 1's collision is disabled.
    if (analyticalM3Enabled_) {
        return m3UseCollision_ ? caoCollisionProb(countNeighboursInRange()) : 0.0;
    }

    auto* manager = veins::FindModule<veins::TraCIScenarioManager*>::findGlobalModule();
    if (!manager) return 0.0;

    const double lambda = std::max(1e-6, getPacketRateHz());
    const double totalResources = getTotalResourcePool();
    const double candidateResources = std::max(1.0, pc5CandidateResourceFraction_ * totalResources);
    const double tau = std::max(1.0, lambda * std::max(sbspsReselectionInterval_, 1e-3));
    const double cbr = estimateChannelBusyRatio();
    const double alpha = cbr > 0.7 ? 1.0 : (cbr >= 0.2 ? clamp01((2.0 * cbr) - 0.4) : 0.0);

    const inet::Coord receiverPos = currentPosition();
    const inet::Coord senderPos(payload->getPosX(), payload->getPosY(), 0.0);
    const double senderToReceiverPsr = getPacketSensingRatio(receiverPos.distance(senderPos));
    double collisionSurvival = 1.0;

    for (const auto& kv : manager->getManagedHosts()) {
        cModule* host = kv.second;
        if (!host || host == getParentModule() || !inet5GApp(host)) continue;
        auto* otherMobility = inetMobility(host);
        if (!otherMobility || otherMobility->getExternalId() == payload->getSenderId()) continue;

        const inet::Coord interfererPos = otherMobility->getCurrentPosition();
        const double receiverDistance = receiverPos.distance(interfererPos);
        if (pc5Range_ > 0.0 && receiverDistance > pc5Range_) continue;

        const double senderDistance = senderPos.distance(interfererPos);
        const double ps = 1.0 - ((1.0 - (1.0 / tau)) * getPacketSensingRatio(senderDistance));
        const double pSimStep2 = ps / candidateResources;
        const double pSimStep3 = ps / std::max(1.0, totalResources * (1.0 - cbr));
        const double pSim = clamp01((alpha * pSimStep2) + ((1.0 - alpha) * pSimStep3));
        const double pInt = clamp01(getPacketSensingRatio(receiverDistance) * (1.0 - (0.5 * senderToReceiverPsr)));
        collisionSurvival *= (1.0 - clamp01(pSim * pInt));
    }

    return clamp01(1.0 - collisionSurvival);
}

// ───────────── Combined Analytical Reference (Cao 2026 + Rehman 2023) ─────────────
// Faithful, teacher-free baseline for NR PC5 Mode-2 delivery (paper Eq. m3):
//   PDR(d,n) = (1 - P_HD) * (1 - P_COL(n)) * g(SNR(d)).
// All constants are pinned to the NS-3 teacher config / 3GPP standards; nothing is
// fitted to the ground truth. The point of the reference is to expose the analytical
// vs distilled discrepancy, so it is NOT calibrated to reduce that gap.
bool VeinsInet5GVehicleApp::loadAnalyticalM3Model()
{
    // Model 1 = Cao's PUBLISHED analytical model (arXiv 2309.16680); it has no teacher.
    // Its equations and fixed constants come from the paper; its config inputs are set to
    // our scenario for comparability with NS3Learn and verified to lie within Cao Table II:
    //   N_r = (SL slots in the selection window) x N_sc              [Cao Eq.(1)]. Cao counts
    //          candidate *SL-slot* resources; his own pool is all-SL, but ours has a 9/12 SL
    //          bitmap (slBitMap 1x9,0x3), so the SL slots in the T2 = 33-slot window are
    //          33 x 9/12 ~ 25. Dropping the 9/12 would over-count resources and make Model 1
    //          optimistically diverge from NS3Learn/NS-3 (which use the same 9/12 pool).
    //   t_s = 1 ms, T_RRI = 100 ms, p_k = 0 in [0,0.8]        (Cao Table II tunables).
    //   N_sc = 5 is OUR channel's subchannel count (Cao demonstrated N_sc=2, but N_sc is a
    //          documented model input, not a fitted constant; 5 keeps us under-saturated).
    //   pi0 = 1/11 (R_c ~ U[5,15]) and N_c = 2 (under-saturation) are Cao's derivations.
    const double selSlots   = par("m3SelectionWindowSlots").doubleValue(); // scenario T2 = 33 slots
    const double nSc        = par("m3NumSubchannels").doubleValue();       // our channel = 5 subchannels
    const double slSlotFrac = par("m3SlSlotFraction").doubleValue();       // SL slots / total = 9/12 (bitmap)
    const double slotMs     = par("m3SlotDurationMs").doubleValue();       // t_s = 1 ms  (Cao Table II)
    const double rriMs      = par("m3RriMs").doubleValue();                // T_RRI = 100 ms (Cao Table II)
    m3Nc_  = par("m3Nc").doubleValue();                                    // N_c = 2 (Cao under-saturation)
    m3Nr_  = std::max(1.0, selSlots * slSlotFrac * nSc);                   // Cao Eq.(1), SL slots only
    m3PHd_ = clamp01(slotMs / std::max(slotMs, rriMs));                    // P_HD = t_s/T_RRI (Cao)
    m3Pi0_ = 1.0 / 11.0;                                                   // Cao: R_c ~ U[5,15]

    // Model 2 (Rehman) decode reads PER(x) from an INDEPENDENT 5G-Toolbox CDL BLER
    // curve (not the NS-3 teacher's curve) so the baseline borrows nothing from the
    // ground truth it is compared against.
    const std::string blerCsv = par("m3BlerCurveCsv").stdstringValue();
    if (blerCsv.empty()) return false;
    if (!loadGeneralizedPhyBlerCurve(blerCsv, "cdl", m3BlerCurve_)) return false;
    m3BlerCurve_.loaded = !m3BlerCurve_.snrDb.empty();
    m3DecodeCache_.clear();
    m3CollCache_.clear();
    return m3BlerCurve_.loaded;
}

// Model 1: Cao steady-state MAC collision probability for p_k = 0 (teacher
// slProbResourceKeep = 0). Base model, Eqs. (8)+(12) solved simultaneously with Nc=2:
//   P_COL = 1 - (1 - 2*pi0/N_a)^n,   N_a = N_r - n + ((Nc-1)/Nc)*P_COL*n.
double VeinsInet5GVehicleApp::caoCollisionProb(int n) const
{
    if (n <= 0) return 0.0;
    auto it = m3CollCache_.find(n);
    if (it != m3CollCache_.end()) return it->second;
    double pc = 0.0;
    for (int iter = 0; iter < 200; ++iter) {
        const double na = std::max(1.0, m3Nr_ - n + ((m3Nc_ - 1.0) / m3Nc_) * pc * n);
        const double next = 1.0 - std::pow(std::max(0.0, 1.0 - (2.0 * m3Pi0_ / na)),
                                           static_cast<double>(n));
        if (std::abs(next - pc) < 1e-10) { pc = next; break; }
        pc = next;
    }
    pc = clamp01(pc);
    m3CollCache_[n] = pc;
    return pc;
}

// Model 2: Rehman noise-limited packet-success probability at distance d.
//   SNR(d) = P_tx - PL(d) - noiseFloor   (noise only; empty interferer set -> no
//   double counting with Model 1's collisions). g = E over the LOS/NLOSv state and
//   log-normal shadowing of [1 - BLER(SNR)], integrated against the SNR density
//   exactly as Rehman integrates PER against the SINR density. The path loss is
//   3GPP TR 37.885 V2V-Urban (the same standard NS-3 5G-LENA and the ns3learn
//   surrogate use), so the two models share the propagation channel and the
//   comparison isolates the contention / capture / fitting differences, not the
//   channel. Rehman's law is generic in the path-loss exponent, so this is a valid
//   instantiation rather than a departure from the published model.
double VeinsInet5GVehicleApp::rehmanDecodeProb(double distanceM) const
{
    const double d = std::max(distanceM, kReferenceDistanceMeters);
    const int key = static_cast<int>(std::lround(d));
    auto it = m3DecodeCache_.find(key);
    if (it != m3DecodeCache_.end()) return it->second;

    const double lg    = std::log10(d);
    const double fc    = std::max(0.5, spatCarrierFrequencyGHz_);
    const double noise = getPc5NoiseFloorDbm();
    const double plLos = 38.77 + (16.7 * lg) + (18.2 * std::log10(fc));   // TR 37.885 V2V LOS
    const double pLos  = std::min(1.0, 1.05 * std::exp(-0.0114 * d));     // TR 37.885 LOS prob
    const double nlosvExcess = 9.0 + std::max(0.0, (15.0 * lg) - 41.0);   // NLOSv mean excess (dB)
    const int NG = 41;                                                    // shadow grid over +/-4 sigma

    double g = 0.0;
    for (int branch = 0; branch < 2; ++branch) {
        const bool los = (branch == 0);
        const double p = los ? pLos : (1.0 - pLos);
        if (p <= 0.0) continue;
        // NLOSv adds a mean vehicle-blockage excess; its variance folds into the
        // branch shadowing in quadrature (shadow sigma (+) NLOSv excess std).
        const double plMean = plLos + (los ? 0.0 : nlosvExcess);
        const double sig    = los ? m3SigmaLosDb_
                                  : std::sqrt((m3SigmaNlosDb_ * m3SigmaNlosDb_)
                                              + (m3NlosvExcessStdDb_ * m3NlosvExcessStdDb_));
        const double snrMean = pc5TxPowerDbm_ - plMean - noise;
        double acc = 0.0, wsum = 0.0;
        for (int k = 0; k < NG; ++k) {
            const double z = -4.0 + (8.0 * k / (NG - 1));        // sigma units
            const double w = std::exp(-0.5 * z * z);             // Gaussian weight
            const double bler = interpolateBlerCurve(m3BlerCurve_, snrMean + (sig * z));
            acc += w * (1.0 - (std::isfinite(bler) ? bler : 1.0));
            wsum += w;
        }
        g += p * (wsum > 0.0 ? acc / wsum : 0.0);
    }
    g = clamp01(g);
    m3DecodeCache_[key] = g;
    return g;
}

// ───────────────────── NS-3-distilled ("ns3learn") surrogate ─────────────────────
// NS-3 V2V Urban path loss (3GPP TR 37.885), LOS / vehicle-NLOSv blended (mean),
// no buildings — matches the teacher scenario.
double VeinsInet5GVehicleApp::v2vChannelPathLossDb(const std::string& senderId, double distance3dMeters,
                                                   double fcGhz, double relX, double relY) const
{
    // Faithful TR 37.885 V2V-Urban channel with SPATIAL COHERENCE (matching NS-3): the LOS/NLOSv
    // state and the shadow fading are held per link and evolve with the Tx-Rx relative geometry.
    // Shadowing is AR(1): s = R*s_prev + sqrt(1-R^2)*N(0,sigma), R = exp(-displacement/d_corr),
    // d_corr = 10 m (LOS) / 13 m (NLOSv); re-initialised when the state changes.
    const double d = std::max(distance3dMeters, 1.0);
    const double lg = std::log10(d);
    const double plLos = 38.77 + (16.7 * lg) + (18.2 * std::log10(fcGhz));
    const double pLos = std::min(1.0, 1.05 * std::exp(-0.0114 * d));
    const double sigSF = 3.0;   // shadow-fading std, LOS & NLOSv (TR 37.885 Table 6.2.3-1)

    V2vChanState& s = v2vChan_[senderId];
    const double dispLen = s.init ? std::hypot(relX - s.lastRelX, relY - s.lastRelY) : 1e18;
    bool condChanged = false;
    if (!s.init) {
        s.los = (uniform(0.0, 1.0) < pLos); condChanged = true;
    } else if (dispLen > (s.los ? 10.0 : 13.0)) {
        // geometry has moved beyond the correlation distance -> re-evaluate the LOS/NLOSv state
        const bool nl = (uniform(0.0, 1.0) < pLos);
        condChanged = (nl != s.los); s.los = nl;
    }
    if (condChanged) {
        s.nlosvExtra = 0.0;
        if (!s.los) {
            const double muA = 9.0 + std::max(0.0, (15.0 * lg) - 41.0);
            const double sigA = 4.5;
            const double lmu = std::log((muA * muA) / std::sqrt((sigA * sigA) + (muA * muA)));
            const double lsig = std::sqrt(std::log(((sigA * sigA) / (muA * muA)) + 1.0));
            s.nlosvExtra = std::max(0.0, std::exp(normal(lmu, lsig)));
        }
        s.shadow = normal(0.0, sigSF);                       // fresh draw on state change
    } else {
        const double R = std::exp(-dispLen / (s.los ? 10.0 : 13.0));
        s.shadow = (R * s.shadow) + (std::sqrt(std::max(0.0, 1.0 - (R * R))) * normal(0.0, sigSF));
    }
    s.lastRelX = relX; s.lastRelY = relY; s.init = true;
    return plLos + (s.los ? 0.0 : s.nlosvExtra) + s.shadow;
}

double VeinsInet5GVehicleApp::ns3V2vPathLossDb(double distance3dMeters, double fcGhz) const
{
    // Faithful port of 3GPP TR 37.885 V2V-Urban (the same standard NS-3's
    // ThreeGppV2vUrbanPropagationLossModel implements), sampled per link:
    //   (a) LOS prob pLos(d) -> per-link LOS/NLOSv state (Table 6.2.1-1);
    //   (b) NLOSv: random blockage loss max(0, lognormal(mean=mu_a, std=sigma_a)) [TR 37.885 v15.2.0];
    //   (c) shadow fading: zero-mean log-normal, sigma per state (Table 6.2.3-1).
    const double d = std::max(distance3dMeters, 1.0);
    const double lg = std::log10(d);
    const double plLos = 38.77 + (16.7 * lg) + (18.2 * std::log10(fcGhz));
    const double pLos = std::min(1.0, 1.05 * std::exp(-0.0114 * d));
    const bool los = (uniform(0.0, 1.0) < pLos);

    double pl = plLos;
    if (!los) {
        // NLOSv additional vehicle-blockage loss. With both V2V antennas (~1.5 m) below typical
        // blocker heights (car 1.6 m / truck 3.0 m) the "both-below" regime applies: mean
        // mu_a = 9 + max(0,15 log10 d - 41) dB, std sigma_a = 4.5 dB, drawn log-normal, max(0,.).
        const double muA  = 9.0 + std::max(0.0, (15.0 * lg) - 41.0);
        const double sigA = 4.5;
        const double lmu  = std::log((muA * muA) / std::sqrt((sigA * sigA) + (muA * muA)));
        const double lsig = std::sqrt(std::log(((sigA * sigA) / (muA * muA)) + 1.0));
        pl += std::max(0.0, std::exp(normal(lmu, lsig)));
    }
    // Shadow fading (TR 37.885 Table 6.2.3-1): sigma = 3 dB for LOS & NLOSv (4 dB for building-NLOS,
    // not reached without buildings). NOTE: the per-link spatial AR(1) correlation is omitted — it
    // only correlates consecutive packets in time and leaves the marginal per-link PDR unchanged.
    pl += normal(0.0, los ? 3.0 : 3.0);
    return pl;
}

// Per-link reception probability (channel/PHY branch) = PSSCH decode × SCI decode,
// from the NS-3-distilled BLER curves evaluated at the noise-limited SINR. Applied
// by passesAnalyticalPdr as the drop_prop gate; half-duplex (drop_hd) and capture-
// corrected collision (drop_sps) are applied separately.
double VeinsInet5GVehicleApp::ns3LearnReception(double distance3dMeters, const std::string& senderId,
                                                double relX, double relY) const
{
    const double fc = std::max(0.5, spatCarrierFrequencyGHz_);
    // LOS-only: 3GPP UMi LOS path loss (control). Otherwise the TR 37.885 V2V channel with per-link
    // spatially-coherent LOS/NLOSv state + AR(1) shadow fading (v2vChannelPathLossDb).
    const double plDb = losOnly_
        ? get3gppUrbanPathLossDb(distance3dMeters, distance3dMeters, false, true, spatUtHeight_, spatUtHeight_)
        : v2vChannelPathLossDb(senderId, distance3dMeters, fc, relX, relY);

    // DECOMPOSED CASCADE (adopted model): each stage is its own formula, drawn per packet.
    //   (1) half-duplex: handled upstream by pc5HalfDuplexDrop_.
    //   (2) SB-SPS collision: drawn with prob collisionP(n)=sigmoid(c0+c1 n+c2 n^2).
    //   (3) decode/BLER: GROUNDED capture — use the MEASURED decode curve for the drawn collision
    //       state: decode_coll(SINR) if it collided (capture: near/high-SINR still decodes), else
    //       decode_noColl(SINR). Both learned from NS-3 (no fitted penalty). The pipeline's Bernoulli
    //       (uniform()>linkPsr) then performs the decode draw.
    if (ns3Cascade_) {
        const double sinr = pc5TxPowerDbm_ - plDb - ns3NoiseFloorDbm_;
        const double n = static_cast<double>(countNeighboursInRange());   // learned collision driver: NORMAL CAV density
        const double baseColl = 1.0 / (1.0 + std::exp(-(ns3ColC0_ + (ns3ColC1_ * n) + (ns3ColC2_ * n * n))));
        // Flood/load-aware: combine the baseline collision with each in-range attacker's bounded
        // per-interferer factor. No attacker in range => survival=1 => collisionP=baseColl (no change).
        const double survival = inRangeAttackerFloodSurvival(n);
        const double collisionP = 1.0 - ((1.0 - baseColl) * survival);
        const bool collided = (uniform(0.0, 1.0) < collisionP);
        if (s_floodCollEnabled) {
            // realized collision outcome by (local n, attacker-in-range): survival<1 => an active
            // flooder is in range. Mirrors NS-3 flood_validate_dataset.csv (n_normal, flooder_in_range, collided).
            FloodCollBin& fb = s_floodColl[{static_cast<int>(n), survival < 1.0 ? 1 : 0}];
            fb.attempts++;
            if (collided) fb.collisions++;
        }
        // density-aware decode: capture (decode|collision) and decode|no-collision both weaken with n
        const double c0 = collided ? ns3DecCcC0_ : ns3DecNcC0_;
        const double c1 = collided ? ns3DecCcC1_ : ns3DecNcC1_;
        const double c2 = collided ? ns3DecCcC2_ : ns3DecNcC2_;
        const double c3 = collided ? ns3DecCcC3_ : ns3DecNcC3_;
        const double c4 = collided ? ns3DecCcC4_ : ns3DecNcC4_;
        const double z = c0 + (c1 * sinr) + (c2 * sinr * sinr) + (c3 * n) + (c4 * sinr * n);
        return clamp01(1.0 / (1.0 + std::exp(-z)));
    }

    // Distance-aware model: degrade SINR by an interference floor I(density) that rises with the
    // local CAV-transmitter count. The reception then decays with distance via PL(d) (capture effect:
    // near=high SINR survives collisions, far=low SINR fails) AND with load via I(density). This
    // replaces the flat half-duplex+collision losses and generalises across topologies.
    double interfDb = 0.0;
    if (ns3DistanceAware_) {
        const int nbr = countNeighboursInRange();
        const double logN = std::log10(1.0 + static_cast<double>(nbr));
        interfDb = (ns3IntfA_ * logN) + ns3IntfB_;                 // mean interference mu(density)
        if (ns3Stochastic_) {
            // add per-reception interference variance sigma(density): colliders' count/proximity
            // vary per packet, which smears the decode threshold into NS-3's gradual distance decay.
            const double sigma = std::max(0.5, (ns3IntfSigmaA_ * logN) + ns3IntfSigmaB_);
            interfDb += normal(0.0, sigma);
        }
    }
    const double sinrDb = pc5TxPowerDbm_ - plDb - ns3NoiseFloorDbm_ - interfDb;
    const double psschBler = interpolateBlerCurve(ns3BlerPssch_, sinrDb);
    const double sciBler = interpolateBlerCurve(ns3BlerSci_, sinrDb);
    const double pssch = std::isfinite(psschBler) ? (1.0 - psschBler) : 1.0;
    const double sci = std::isfinite(sciBler) ? (1.0 - sciBler) : 1.0;
    return clamp01(pssch * sci);
}

int VeinsInet5GVehicleApp::countNeighboursInRange() const
{
    auto* manager = veins::FindModule<veins::TraCIScenarioManager*>::findGlobalModule();
    if (!manager) return 0;
    const inet::Coord pos = currentPosition();
    int n = 0;
    for (const auto& kv : manager->getManagedHosts()) {
        cModule* host = kv.second;
        if (!host || host == getParentModule() || !inet5GApp(host)) continue;
        // Count CAV transmitters only (veh_av -> VeinsInetCar). The scenario also
        // instantiates human vehicles (veh_human -> VeinsInetLiteCar) as 5G-app
        // hosts, but the NS-3 collision curve was fitted against CAV-neighbour
        // density (NS-3 modelled CAV-only TX), so non-CAVs must be excluded here.
        if (std::string(host->getNedTypeName()).find("VeinsInetCar") == std::string::npos) continue;
        // Exclude a flooding attacker from the baseline density: the NS-3 collision curve
        // base(n) was fitted on the NORMAL 10 Hz fleet; the attacker enters separately as the
        // bounded q_a factor (inRangeAttackerFloodSurvival). Avoids double-counting its load.
        VeinsInet5GVehicleApp* oa = inet5GApp(host);
        if (oa && oa->isAttacker_ && oa->enableFlooding_) continue;
        auto* m = inetMobility(host);
        if (!m) continue;
        if (pc5Range_ > 0.0 && pos.distance(m->getCurrentPosition()) > pc5Range_) continue;
        ++n;
    }
    return n;
}

// Flood/load-aware contention: each in-range flooding attacker degrades a normal node's
// reception by a bounded per-interferer collision factor q_a(floodRate, n), distilled from the
// NS-3 asymmetric-flood teacher runs (held-out validation MAE 0.013). Returns PROD (1 - q_a);
// 1.0 when no attacker is in range (so the non-attack baseline is exactly base(n) — no regression).
double VeinsInet5GVehicleApp::inRangeAttackerFloodSurvival(double n) const
{
    if (ns3QaB1_ == 0.0 && ns3QaB3_ == 0.0) return 1.0;   // attacker factor not in bundle -> no-op
    auto* manager = veins::FindModule<veins::TraCIScenarioManager*>::findGlobalModule();
    if (!manager) return 1.0;
    const inet::Coord pos = currentPosition();
    const double ln = std::log10(n + 1.0);
    double survival = 1.0;
    for (const auto& kv : manager->getManagedHosts()) {
        cModule* host = kv.second;
        if (!host || host == getParentModule()) continue;
        VeinsInet5GVehicleApp* a = inet5GApp(host);
        // Only an ACTIVELY-flooding attacker occupies channel resources: gate on attackActive_
        // (true only within [attackStartTime, +attackDuration]) so q_a fires during the attack
        // window, not the attacker's whole presence. enableFlooding_ restricts to the
        // high-rate-interferer vector that q_a was fitted on.
        if (!a || !a->isAttacker_ || !a->enableFlooding_ || !a->attackActive_) continue;
        auto* m = inetMobility(host);
        if (!m) continue;
        if (pc5Range_ > 0.0 && pos.distance(m->getCurrentPosition()) > pc5Range_) continue;
        // attacker's actual injection rate = min(floodRate, maxActualFloodRate); calibrated 200-1000 pps.
        const double rate = std::max(1.0, std::min(a->floodRate_, a->maxActualFloodRate_));
        const double lr = std::log10(rate);
        const double q = 1.0 / (1.0 + std::exp(-(ns3QaB0_ + (ns3QaB1_ * lr) + (ns3QaB2_ * ln) + (ns3QaB3_ * lr * ln))));
        survival *= (1.0 - clamp01(q));
    }
    return survival;
}

// Half-duplex drop for the cascade: LEARNED logistic in local CAV density n (fitted to the NS-3
// MAC slot-coincidence data), not a constant and not an invented rate scaling.
double VeinsInet5GVehicleApp::cascadeHalfDuplexDrop() const
{
    const double n = static_cast<double>(countNeighboursInRange());
    const double z = ns3HdC0_ + (ns3HdC1_ * n) + (ns3HdC2_ * n * n);
    return clamp01(1.0 / (1.0 + std::exp(-z)));
}

// Resource-collision drop (drop_sps) = collision(neighbour density) × (1 − capture),
// distilled from NS-3 simulPsschTx overlap vs density.
double VeinsInet5GVehicleApp::ns3LearnCollisionDrop() const
{
    // Distance-aware lumped model and the decomposed cascade both handle collisions inside
    // ns3LearnReception (interference-degraded SINR / explicit collision stage), so the separate
    // collision drop is disabled.
    if (ns3DistanceAware_ || ns3Cascade_) return 0.0;
    const double n = static_cast<double>(countNeighboursInRange());
    const double coll = ns3CollL_ / (1.0 + std::exp(-ns3CollK_ * (n - ns3CollX0_)));
    return clamp01(coll * (1.0 - ns3Capture_));
}

bool VeinsInet5GVehicleApp::loadNs3LearnModel(const std::string& dir)
{
    if (dir.empty()) return false;
    std::string base = dir;
    if (base.back() != '/') base += '/';
    if (!loadGeneralizedPhyBlerCurve(base + "bler_pssch.csv", "ns3_pssch", ns3BlerPssch_)) return false;
    if (!loadGeneralizedPhyBlerCurve(base + "bler_sci.csv", "ns3_sci", ns3BlerSci_)) return false;

    std::ifstream f(base + "ns3learn_params.csv");
    if (!f.is_open()) return false;
    std::string line;
    double nf = 7.0, bw = 18.72e6;
    std::getline(f, line);   // header: key,value
    while (std::getline(f, line)) {
        const auto c = line.find(',');
        if (c == std::string::npos) continue;
        const std::string k = line.substr(0, c);
        double v = 0.0;
        try { v = std::stod(line.substr(c + 1)); } catch (...) { continue; }
        if (k == "half_duplex") ns3HalfDuplex_ = v;
        else if (k == "capture") ns3Capture_ = v;
        else if (k == "collision_L") ns3CollL_ = v;
        else if (k == "collision_k") ns3CollK_ = v;
        else if (k == "collision_x0") ns3CollX0_ = v;
        else if (k == "intf_a") ns3IntfA_ = v;
        else if (k == "intf_b") ns3IntfB_ = v;
        else if (k == "intf_sigma_a") ns3IntfSigmaA_ = v;
        else if (k == "intf_sigma_b") ns3IntfSigmaB_ = v;
        else if (k == "stochastic") ns3Stochastic_ = (v > 0.5);
        else if (k == "distance_aware") ns3DistanceAware_ = (v > 0.5);
        else if (k == "noise_figure_db") nf = v;
        else if (k == "bandwidth_hz") bw = v;
    }
    ns3NoiseFloorDbm_ = -174.0 + (10.0 * std::log10(std::max(1.0, bw))) + nf;

    // Optional decomposed-cascade coefficients (adopted model). If present, override the
    // lumped path: half-duplex + collisionP(n) logistic + capture penalty + decodeP(SINR) logistic.
    std::ifstream cf(base + "cascade_params.csv");
    if (cf.is_open()) {
        std::string cl; std::getline(cf, cl);   // header
        while (std::getline(cf, cl)) {
            const auto c = cl.find(',');
            if (c == std::string::npos) continue;
            const std::string k = cl.substr(0, c);
            double v = 0.0;
            try { v = std::stod(cl.substr(c + 1)); } catch (...) { continue; }
            if (k == "cascade") ns3Cascade_ = (v > 0.5);
            else if (k == "half_duplex") ns3HalfDuplex_ = v;
            else if (k == "dec_c0") ns3DecC0_ = v;
            else if (k == "dec_c1") ns3DecC1_ = v;
            else if (k == "dec_c2") ns3DecC2_ = v;
            else if (k == "decnc_c0") ns3DecNcC0_ = v;
            else if (k == "decnc_c1") ns3DecNcC1_ = v;
            else if (k == "decnc_c2") ns3DecNcC2_ = v;
            else if (k == "decnc_c3") ns3DecNcC3_ = v;
            else if (k == "decnc_c4") ns3DecNcC4_ = v;
            else if (k == "deccc_c0") ns3DecCcC0_ = v;
            else if (k == "deccc_c1") ns3DecCcC1_ = v;
            else if (k == "deccc_c2") ns3DecCcC2_ = v;
            else if (k == "deccc_c3") ns3DecCcC3_ = v;
            else if (k == "deccc_c4") ns3DecCcC4_ = v;
            else if (k == "col_c0") ns3ColC0_ = v;
            else if (k == "col_c1") ns3ColC1_ = v;
            else if (k == "col_c2") ns3ColC2_ = v;
            else if (k == "qa_b0") ns3QaB0_ = v;
            else if (k == "qa_b1") ns3QaB1_ = v;
            else if (k == "qa_b2") ns3QaB2_ = v;
            else if (k == "qa_b3") ns3QaB3_ = v;
            else if (k == "cap_mu") ns3CapMu_ = v;
            else if (k == "cap_sigma") ns3CapSigma_ = v;
            else if (k == "hd_c0") ns3HdC0_ = v;
            else if (k == "hd_c1") ns3HdC1_ = v;
            else if (k == "hd_c2") ns3HdC2_ = v;
            else if (k == "noise_floor_dbm") ns3NoiseFloorDbm_ = v;
        }
    }
    // cascade needs only the two logistic curves; lumped needs the collision logistic params.
    return ns3BlerPssch_.loaded && ns3BlerSci_.loaded && (ns3Cascade_ || ns3CollL_ > 0.0);
}

double VeinsInet5GVehicleApp::capacityDropProbability(double loadUnits)
{
    const double now = simTime().dbl();
    const double weight = std::max(1.0, loadUnits);
    rxWindow_.push_back({now, weight});
    rxWindowLoad_ += weight;
    while (!rxWindow_.empty() && now - rxWindow_.front().first > capacityWindow_) {
        rxWindowLoad_ -= rxWindow_.front().second;
        rxWindow_.pop_front();
    }

    const double capacity = std::max(1.0, obuCapacityPps_ * std::max(0.001, capacityWindow_));
    if (rxWindowLoad_ <= capacity) return 0.0;

    const double overload = (rxWindowLoad_ - capacity) / capacity;
    return clamp01(overload * capacityDropSlope_);
}

double VeinsInet5GVehicleApp::sampleAuthDelay() const
{
    if (authDelayMean_ <= 0.0 && authDelayStd_ <= 0.0) return 0.0;

    double delay = truncnormal(authDelayMean_, authDelayStd_);
    if (!std::isfinite(delay) || delay < 0.0) delay = 0.0;
    return delay;
}

bool VeinsInet5GVehicleApp::isMessageStale(double timeSent, double decisionTime) const
{
    if (authFreshnessWindow_ <= 0.0) return false;
    if (!std::isfinite(timeSent) || !std::isfinite(decisionTime)) return false;
    return (decisionTime - timeSent) > authFreshnessWindow_;
}

bool VeinsInet5GVehicleApp::hasValidAuthCache(const std::string& cacheKey) const
{
    if (authCacheWindow_ <= 0.0 || cacheKey.empty()) return false;

    auto it = authCacheExpiry_.find(cacheKey);
    if (it == authCacheExpiry_.end()) return false;
    return simTime().dbl() <= it->second;
}

void VeinsInet5GVehicleApp::updateAuthCache(const std::string& cacheKey, double timeActed, double trustWindowOverride)
{
    const double trustWindow = trustWindowOverride >= 0.0 ? trustWindowOverride : authCacheWindow_;
    if (trustWindow <= 0.0 || cacheKey.empty()) return;
    authCacheExpiry_[cacheKey] = timeActed + trustWindow;
}

double VeinsInet5GVehicleApp::authTrustWindowFor(inet::Ptr<const VeinsInet5GMessage> payload) const
{
    const std::string kind = payload->getMessageKind();
    if (kind == "SPAT") return spatTrustWindow_ >= 0.0 ? spatTrustWindow_ : authCacheWindow_;
    if (kind == "CAM" || kind == "FLOOD_CAM") return camTrustWindow_ >= 0.0 ? camTrustWindow_ : authCacheWindow_;
    return authCacheWindow_;
}

std::string VeinsInet5GVehicleApp::authCacheKeyFor(inet::Ptr<const VeinsInet5GMessage> payload) const
{
    const std::string kind = payload->getMessageKind();
    if (kind == "CAM" || kind == "FLOOD_CAM") {
        const std::string sender = payload->getSenderId();
        return sender.empty() ? std::string() : "CAM:" + sender;
    }
    if (kind == "SPAT") {
        const std::string tlsId = payload->getTlsId();
        return tlsId.empty() ? std::string() : "SPAT:" + tlsId;
    }
    return "";
}

inet::Coord VeinsInet5GVehicleApp::currentPosition() const
{
    return mobility ? mobility->getCurrentPosition() : inet::Coord::ZERO;
}

double VeinsInet5GVehicleApp::currentSpeed() const
{
    try {
        return traciVehicle ? traciVehicle->getSpeed() : mobility->getCurrentVelocity().length();
    }
    catch (...) {
        return mobility ? mobility->getCurrentVelocity().length() : 0.0;
    }
}

int VeinsInet5GVehicleApp::getSchemaVehicleIndex() const
{
    cModule* node = getParentModule();
    if (!node) return -1;
    return (node->getIndex() >= 0) ? node->getIndex() : node->getId();
}

int VeinsInet5GVehicleApp::getSchemaObuId() const
{
    return getId();
}

bool VeinsInet5GVehicleApp::isSchemaAttacker() const
{
    return isAttacker_;
}

void VeinsInet5GVehicleApp::logSchemaSelfState() const
{
    if (!logGroundtruth_) return;

    cModule* node = getParentModule();
    if (!node) return;

    const int vehicleIndex = getSchemaVehicleIndex();
    const int obuId = getSchemaObuId();
    const int attackerFlag = isSchemaAttacker() ? 1 : 0;
    const inet::Coord pos = currentPosition();
    const inet::Coord velocity = mobility ? mobility->getCurrentVelocity() : inet::Coord::ZERO;

    std::ostringstream fileName;
    fileName << "JSONlog-" << vehicleIndex << "-" << obuId << "-A" << attackerFlag << ".json";

    std::ofstream* file = ensureVehicleJsonFile(fileName.str(),
                                                vehicleIndex,
                                                obuId,
                                                attackerFlag,
                                                selfVehicleId_,
                                                sumoVehicleType_,
                                                node->getId(),
                                                getId());
    if (!file || !file->is_open()) return;

    *file << std::fixed << std::setprecision(12)
          << "{\"type\":2,"
          << "\"rcvTime\":" << simTime().dbl() << ","
          << "\"pos\":[" << pos.x << "," << pos.y << "," << pos.z << "],"
          << "\"noise\":[0.0,0.0,0.0],"
          << "\"spd\":[" << velocity.x << "," << velocity.y << "," << velocity.z << "],"
          << "\"spd_noise\":[0.0,0.0,0.0]}\n";
    file->flush();
}

void VeinsInet5GVehicleApp::logSchemaGroundTruth(inet::Ptr<const VeinsInet5GMessage> payload) const
{
    if (!logGroundtruth_) return;
    ensureGroundTruthJsonFile();
    if (!s_groundTruthJsonFile.is_open()) return;

    const int senderIndex = getSchemaVehicleIndex();
    const int attackerFlag = isSchemaAttacker() ? 1 : 0;

    s_groundTruthJsonFile << std::fixed << std::setprecision(12)
                          << "{\"type\":4,"
                          << "\"time\":" << payload->getTimeSent() << ","
                          << "\"sender\":" << senderIndex << ","
                          << "\"attackerType\":" << attackerFlag << ","
                          << "\"messageID\":" << payload->getMessageId() << ","
                          << "\"pos\":[" << payload->getPosX() << "," << payload->getPosY() << "," << payload->getPosZ() << "],"
                          << "\"pos_noise\":[0.0,0.0,0.0],"
                          << "\"spd\":[" << payload->getVelX() << "," << payload->getVelY() << "," << payload->getVelZ() << "],"
                          << "\"spd_noise\":[0.0,0.0,0.0]}\n";
    s_groundTruthJsonFile.flush();
}

void VeinsInet5GVehicleApp::logSchemaReceived(inet::Ptr<const VeinsInet5GMessage> payload) const
{
    if (!logGroundtruth_) return;

    cModule* node = getParentModule();
    if (!node) return;

    const int vehicleIndex = getSchemaVehicleIndex();
    const int obuId = getSchemaObuId();
    const int attackerFlag = isSchemaAttacker() ? 1 : 0;
    const int senderIndex = lookupSchemaVehicleIndexByExternalId(payload->getSenderId());
    const inet::Coord ownPos = currentPosition();
    const inet::Coord senderPos(payload->getPosX(), payload->getPosY(), payload->getPosZ());
    const double rssiWatts = estimateSchemaRssiWatts(ownPos.distance(senderPos),
                                                     pc5TxPowerDbm_,
                                                     pc5ReferenceLossDb_,
                                                     pc5PathLossExponent_);

    std::ostringstream fileName;
    fileName << "JSONlog-" << vehicleIndex << "-" << obuId << "-A" << attackerFlag << ".json";

    std::ofstream* file = ensureVehicleJsonFile(fileName.str(),
                                                vehicleIndex,
                                                obuId,
                                                attackerFlag,
                                                selfVehicleId_,
                                                sumoVehicleType_,
                                                node->getId(),
                                                getId());
    if (!file || !file->is_open()) return;

    *file << std::fixed << std::setprecision(12)
          << "{\"type\":3,"
          << "\"rcvTime\":" << simTime().dbl() << ","
          << "\"sendTime\":" << payload->getTimeSent() << ","
          << "\"sender\":" << senderIndex << ","
          << "\"messageID\":" << payload->getMessageId() << ","
          << "\"pos\":[" << payload->getPosX() << "," << payload->getPosY() << "," << payload->getPosZ() << "],"
          << "\"pos_noise\":[0.0,0.0,0.0],"
          << "\"spd\":[" << payload->getVelX() << "," << payload->getVelY() << "," << payload->getVelZ() << "],"
          << "\"spd_noise\":[0.0,0.0,0.0],"
          << "\"RSSI\":" << rssiWatts << "}\n";
    file->flush();
}

// ─── OBU Processing Queue (M/D/1 approximation) ───────────────────────────────
// Models the OBU as a single-threaded crypto engine.  Each auth job occupies the
// processor for exactly serviceTime seconds.  If the queue backlog exceeds
// obuQueueMaxDepth_ (default 0.5 s), the message is dropped (buffer overflow).
//
// With N neighbours at 10 Hz: arrival rate λ = 10N, service rate μ = 1/authDelay.
// Queue utilisation ρ = λ/μ.  For ρ ≥ 1 (TESLA/Stress in dense traffic) the
// backlog will grow until obuQueueMaxDepth_ is hit and messages are dropped.
//
// Returns the relative delay (from now) at which the auth callback should fire.
// Returns -1.0 if the queue is full (caller should drop the message).

double VeinsInet5GVehicleApp::scheduleAuthDelay(double serviceTime)
{
    if (!enableObuProcessingQueue_) return serviceTime;

    const double now = simTime().dbl();
    const double queueBacklog = std::max(0.0, processorFreeAt_ - now);

    if (queueBacklog > obuQueueMaxDepth_) {
        return -1.0; // queue overflow — drop
    }

    const double startAt = std::max(now, processorFreeAt_);
    processorFreeAt_ = startAt + serviceTime;
    return processorFreeAt_ - now; // relative delay from now
}

