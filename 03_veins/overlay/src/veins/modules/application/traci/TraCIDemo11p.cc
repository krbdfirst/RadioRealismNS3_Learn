//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//

#include "veins/modules/application/traci/TraCIDemo11p.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include "veins/modules/application/traci/RunMetadata.h"
#include "veins/modules/mobility/traci/TraCIConstants.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <ctime>
#include <vector>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

using namespace veins;

Define_Module(veins::TraCIDemo11p);

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  Output base directory
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
static const std::string BASE_RESULTS_DIR = "results/";

static void makedir(const std::string& path)
{
#ifdef _WIN32
    _mkdir(path.c_str());
#else
    mkdir(path.c_str(), 0755);
#endif
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  getRunOutputDir
//  Non-static - shared via extern with subclasses.
//  Reuses OMNeT++ result-dir when configured so custom CSV files land beside
//  .sca/.vec outputs. Falls back to a timestamped folder if result-dir is unset.
//  Fallback folder format:  YYYY-MM-DD_HH-MM-SS
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
const std::string& getRunOutputDir()
{
    static std::string dir = "";
    if (!dir.empty()) return dir;

    std::string resultDir = getCurrentResultDir();
    if (!resultDir.empty()) {
        if (resultDir.back() != '/' && resultDir.back() != '\\') resultDir += "/";
        makedir(BASE_RESULTS_DIR);
        makedir(resultDir);
        dir = resultDir;
        return dir;
    }

    std::time_t now = std::time(nullptr);
    std::tm*    t   = std::localtime(&now);

    std::ostringstream folder;
    folder << std::setfill('0')
           << (t->tm_year + 1900) << "-"
           << std::setw(2) << (t->tm_mon + 1) << "-"
           << std::setw(2) <<  t->tm_mday     << "_"
           << std::setw(2) <<  t->tm_hour     << "-"
           << std::setw(2) <<  t->tm_min      << "-"
           << std::setw(2) <<  t->tm_sec;

    makedir(BASE_RESULTS_DIR);
    dir = BASE_RESULTS_DIR + folder.str() + "/";
    makedir(dir);
    return dir;
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  getRunTimestamp
//  File-name stamp: YYYY_MM_DD_HHMM
//  e.g. vehicle_timeseries_2026_02_28_1509.csv
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
static const std::string& getRunTimestamp()
{
    static std::string ts = "";
    if (!ts.empty()) return ts;

    std::time_t now = std::time(nullptr);
    std::tm*    t   = std::localtime(&now);

    std::ostringstream s;
    s << std::setfill('0')
      << (t->tm_year + 1900) << "_"
      << std::setw(2) << (t->tm_mon + 1) << "_"
      << std::setw(2) <<  t->tm_mday     << "_"
      << std::setw(2) <<  t->tm_hour
      << std::setw(2) <<  t->tm_min;
    ts = s.str();
    return ts;
}

#define OUTPUT_DIR (getRunOutputDir())

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  Thresholds
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
static const double HARD_BRAKE_THRESHOLD      = -4.0;  // m/sÂ² hard braking
static const double EMERGENCY_BRAKE_THRESHOLD = -7.0;  // m/sÂ² emergency braking
static const double BRAKE_START_THRESHOLD     = -0.5;  // m/sÂ² episode begins
static const double BRAKE_END_THRESHOLD       = -0.2;  // m/sÂ² episode ends
static const double SPEED_STOP_THRESHOLD      =  0.1;  // m/s  considered stopped
static const double COLLISION_GAP_THRESHOLD   =  1.0;  // m    gap â‰¤ this â†’ collision
static const double COLLISION_STOP_SECONDS    = 10.0;  // s    must be stopped this long
static const double TTC_INFINITY              = 999.0; // sentinel "no TTC risk"
static const double ACTIVE_TRANSPORT_COLLISION_DISTANCE = 1.5; // m
static const double TLS_VIOLATION_DISTANCE_THRESHOLD    = 8.0; // m
static const double PDR_TIME_BIN_SECONDS                = 0.1; // s

static std::string csvEscape(const std::string& s) {
    if (s.find(',') != std::string::npos) return "\"" + s + "\"";
    return s;
}

static int isCavVehicleType(const std::string& vehicleType)
{
    return (vehicleType == "veh_av" || vehicleType == "veh_cav") ? 1 : 0;
}

static int lookupLeaderIsCav(TraCIMobility* mobility, const std::string& leaderId)
{
    if (!mobility || leaderId.empty()) return 0;

    try {
        std::string leaderType = mobility->getCommandInterface()->vehicle(leaderId).getTypeId();
        return isCavVehicleType(leaderType);
    } catch (...) {
        return 0;
    }
}

static double lookupLeaderSpeed(TraCIMobility* mobility, const std::string& leaderId)
{
    if (!mobility || leaderId.empty()) return 0.0;

    try {
        return mobility->getCommandInterface()->vehicle(leaderId).getSpeed();
    } catch (...) {
        return 0.0;
    }
}

static char normalizeTlsSignal(char rawSignal)
{
    return SpatPayload::normalizeSignal(rawSignal);
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  Single static stream shared across ALL
//  vehicle instances in one simulation run.
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
static std::ofstream s_timeseriesFile;
static bool          s_fileInitialised = false;

// Shared spat_response.csv - written by TraCIDemo11p (baseline) and
// any subclass. Opened here so the baseline run always generates
// the file.
std::ofstream s_spatResponseFile;
static bool          s_spatResponseReady = false;
static std::ofstream s_networkPdrFile;
static bool          s_networkPdrReady = false;
static bool          s_networkPdrWritten = false;
static std::string   s_networkPdrRunId;
static long          s_activeAppInstances = 0;

struct ActiveTransportSnapshot {
    double time = -1.0;
    std::vector<std::pair<std::string, Coord>> pedestrians;
    std::vector<std::pair<std::string, Coord>> cyclists;
};

struct PdrCounters {
    long attempts = 0;
    long deliveries = 0;
};

struct PdrStepCounters {
    PdrCounters v2v;
    PdrCounters v2i;
};

static ActiveTransportSnapshot s_activeTransportSnapshot;
static std::map<long long, PdrStepCounters> s_pdrSteps;

static long long pdrStepKey(double t)
{
    return static_cast<long long>(std::floor((t / PDR_TIME_BIN_SECONDS) + 1e-9));
}

static double pdrStepTime(long long key)
{
    return key * PDR_TIME_BIN_SECONDS;
}

static void initNetworkPdrFile()
{
    if (s_networkPdrReady) return;
    s_networkPdrReady = true;
    s_networkPdrFile.open(getRunOutputDir() + "network_pdr.csv");
    if (s_networkPdrFile.is_open()) {
        s_networkPdrFile
            << "run_number,"
            << "seed_set,"
            << "run_id,"
            << "time_s,"
            << "v2v_attempts,"
            << "v2v_deliveries,"
            << "v2v_pdr,"
            << "v2i_attempts,"
            << "v2i_deliveries,"
            << "v2i_pdr\n";
        s_networkPdrFile.flush();
    }
}

static void prepareNetworkPdrForCurrentRun()
{
    std::string currentRunId = getCurrentRunId();
    if (s_networkPdrRunId == currentRunId) return;

    s_networkPdrRunId = currentRunId;
    s_networkPdrWritten = false;
    s_pdrSteps.clear();
}

static void notePdrAttempt(bool isV2V, double t)
{
    auto& bucket = s_pdrSteps[pdrStepKey(t)];
    if (isV2V) bucket.v2v.attempts++;
    else bucket.v2i.attempts++;
}

static void notePdrDelivery(bool isV2V, double t)
{
    auto& bucket = s_pdrSteps[pdrStepKey(t)];
    if (isV2V) bucket.v2v.deliveries++;
    else bucket.v2i.deliveries++;
}

static PdrCounters getPdrCounters(bool isV2V, double t)
{
    auto it = s_pdrSteps.find(pdrStepKey(t));
    if (it == s_pdrSteps.end()) return {};
    return isV2V ? it->second.v2v : it->second.v2i;
}

double getLoggedStepPdr(bool isV2V, double t)
{
    PdrCounters counters = getPdrCounters(isV2V, t);
    if (counters.attempts <= 0) return 1.0;
    return static_cast<double>(counters.deliveries) / counters.attempts;
}

long getLoggedStepAttempts(bool isV2V, double t)
{
    return getPdrCounters(isV2V, t).attempts;
}

long getLoggedStepDeliveries(bool isV2V, double t)
{
    return getPdrCounters(isV2V, t).deliveries;
}

void recordPdrAttempt(bool isV2V, double t)
{
    notePdrAttempt(isV2V, t);
}

void recordPdrDelivery(bool isV2V, double t)
{
    notePdrDelivery(isV2V, t);
}

static void writeNetworkPdrFileOnce()
{
    if (s_networkPdrWritten || !s_networkPdrFile.is_open()) return;
    s_networkPdrWritten = true;

    for (const auto& kv : s_pdrSteps) {
        const auto& step = kv.second;
        double v2vPdr = (step.v2v.attempts > 0)
            ? static_cast<double>(step.v2v.deliveries) / step.v2v.attempts
            : 1.0;
        double v2iPdr = (step.v2i.attempts > 0)
            ? static_cast<double>(step.v2i.deliveries) / step.v2i.attempts
            : 1.0;

        s_networkPdrFile << std::fixed << std::setprecision(4)
            << getCurrentRunCsvPrefix()
            << pdrStepTime(kv.first) << ","
            << step.v2v.attempts << ","
            << step.v2v.deliveries << ","
            << v2vPdr << ","
            << step.v2i.attempts << ","
            << step.v2i.deliveries << ","
            << v2iPdr << "\n";
    }
    s_networkPdrFile.flush();
}

static void refreshActiveTransportSnapshot(TraCICommandInterface* ci, double currentTime)
{
    if (!ci) return;
    if (std::fabs(s_activeTransportSnapshot.time - currentTime) < 1e-9) return;

    s_activeTransportSnapshot = {};
    s_activeTransportSnapshot.time = currentTime;

    auto* mgr = TraCIScenarioManagerAccess().get();
    if (!mgr) return;

    for (const auto& kv : mgr->getManagedHosts()) {
        cModule* mod = kv.second;
        if (!mod) continue;

        auto* hostMobility = dynamic_cast<TraCIMobility*>(mod->getSubmodule("mobility"));
        if (!hostMobility) continue;

        try {
            std::string externalId = hostMobility->getExternalId();
            Coord pos = hostMobility->getPositionAt(simTime());
            std::string typeId = ci->vehicle(externalId).getTypeId();

            if (typeId == "bike_human") {
                s_activeTransportSnapshot.cyclists.push_back({externalId, pos});
            }
            else if (typeId == "ped_human") {
                s_activeTransportSnapshot.pedestrians.push_back({externalId, pos});
            }
        } catch (...) {
        }
    }
}

static void initSpatResponseFile()
{
    if (s_spatResponseReady) return;
    s_spatResponseReady = true;
    s_spatResponseFile.open(getRunOutputDir() + "spat_response.csv");
    if (s_spatResponseFile.is_open()) {
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
}

void TraCIDemo11p::initCSVFiles()
{
    if (s_fileInitialised) return;      // first vehicle only
    s_fileInitialised = true;

    std::string dir  = OUTPUT_DIR;      // creates folder on first call
    std::string name = "vehicle_timeseries_" + getRunTimestamp() + ".csv";

    s_timeseriesFile.open(dir + name);
    if (!s_timeseriesFile.is_open())
        throw cRuntimeError("TraCIDemo11p: cannot open %s in %s",
                            name.c_str(), dir.c_str());

    s_timeseriesFile
        << "run_number,"
        << "seed_set,"
        << "run_id,"
        << "time,"
        << "vehicle_id,"
        << "ego_is_cav,"
        << "vehicle_type,"           // "veh_av" for CAVs, "veh_human" for ManualVehicle
        << "speed_mps,"
        << "acceleration_mps2,"
        << "gap_to_leader_m,"
        << "leader_vehicle_id,"
        << "leader_is_cav,"
        << "ttc_s,"
        << "braking_distance_m,"
        << "hard_brake_event,"       // 1 on first step where decel â‰¤ -4 m/sÂ²
        << "emergency_brake_event,"  // 1 on first step where decel â‰¤ -7 m/sÂ²
        << "collision,"              // 1 while stopped â‰¥10 s with gap â‰¤1 m
        << "tls_violation_event,"
        << "pedestrian_collision_event,"
        << "cyclist_collision_event,"
        << "active_transport_collision_event,"
        << "involves_cav\n";         // 1 = CAV involved, 0 = no CAV in this interaction
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  initialize
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void TraCIDemo11p::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        ++s_activeAppInstances;
        sentMessage  = false;
        lastDroveAt  = simTime();
        currentSubscribedServiceId = -1;

        prevSpeed    = -1.0;
        prevTime     = simTime();
        prevBraking  = false;
        prevTlsId    = "";
        prevTlsState = -1.0;

        // Per-vehicle SPaT state
        spatPhase_          = 'U';
        spatSpeedOverride_  = false;
        spatNormalMaxSpeed_ = 0.0;
        ttcLookaheadDistance_ = std::max(0.0, par("ttcLookaheadDistance").doubleValue());
        isVehicleActive_    = true;

        // Per-vehicle braking state
        inBrakingEpisode_     = false;
        brakeEpisodeDist_     = 0.0;
        brakeEpisodeMaxDecel_ = 0.0;
        brakeLastX_           = 0.0;
        brakeLastY_           = 0.0;
        hardBrakeFired_       = false;
        emergencyBrakeFired_  = false;

        // Per-vehicle collision state
        inCollision_  = false;
        stoppedSince_ = -1;
        actualCollision_ = false;
        tlsViolationLatched_ = false;
        tlsViolationActive_ = false;
        pedCollisionLatched_ = false;
        bikeCollisionLatched_ = false;
        lastRoadId_.clear();
        lastObservedTlsId_.clear();
        lastObservedTlsSignal_ = 'U';
        lastObservedTlsDistance_ = -1.0;
        if (mobility) mobility->subscribe(TraCIMobility::collisionSignal, this);

        initCSVFiles();                      // guarded â€” only first vehicle opens
        initSpatResponseFile();               // opens spat_response.csv for baseline
        initNetworkPdrFile();
        prepareNetworkPdrForCurrentRun();
        timeseriesFile = &s_timeseriesFile;  // all vehicles share one stream
    }
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  finish  â€“  flush shared stream
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void TraCIDemo11p::finish()
{
    // FIRST: mark inactive â€” prevents any further TraCI calls
    isVehicleActive_ = false;

    // Do NOT call setSpeed here â€” vehicle is being removed from SUMO.
    // SUMO handles speed constraint cleanup automatically on vehicle removal.
    // Attempting setSpeed on a departing vehicle causes status 255.
    spatSpeedOverride_ = false;

    DemoBaseApplLayer::finish();
    if (s_timeseriesFile.is_open()) s_timeseriesFile.flush();

    if (s_activeAppInstances > 0) --s_activeAppInstances;
    if (s_activeAppInstances == 0) {
        writeNetworkPdrFileOnce();
    }
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  getLeaderGap
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
double TraCIDemo11p::getLeaderGap()
{
    auto leader = getLeaderInfo();
    return leader.second;   // -1 when no leader within lookahead
}

double TraCIDemo11p::getLeaderLookaheadDistance() const
{
    return ttcLookaheadDistance_;
}

std::pair<std::string, double> TraCIDemo11p::getLeaderInfo() const
{
    if (!traciVehicle) return {"", -1.0};
    return traciVehicle->getLeader(getLeaderLookaheadDistance());
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  computeTTC
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
double TraCIDemo11p::computeTTC(double speed, double leaderSpeed, double gap)
{
    if (gap <= 0)              return 0.0;
    double rel = speed - leaderSpeed;
    if (rel <= 0)              return TTC_INFINITY;
    return gap / rel;
}

void TraCIDemo11p::receiveSignal(cComponent* source, simsignal_t signalID, bool value, cObject* details)
{
    Enter_Method_Silent();
    if (signalID == TraCIMobility::collisionSignal) {
        actualCollision_ = value;
        return;
    }
    cListener::receiveSignal(source, signalID, value, details);
}

void TraCIDemo11p::updateTlsObservation(int& tlsViolationEvent)
{
    tlsViolationEvent = 0;
    if (!isVehicleActive_ || !mobility || !traciVehicle) return;

    std::string currentRoadId;
    try {
        currentRoadId = mobility->getRoadId();
    } catch (...) {
        return;
    }

    bool onInternalEdge = !currentRoadId.empty() && currentRoadId[0] == ':';
    bool wasOnInternalEdge = !lastRoadId_.empty() && lastRoadId_[0] == ':';

    if (!onInternalEdge) {
        tlsViolationLatched_ = false;
        tlsViolationActive_ = false;
        lastObservedTlsId_.clear();
        lastObservedTlsSignal_ = 'U';
        lastObservedTlsDistance_ = -1.0;

        try {
            for (const auto& nextTls : traciVehicle->getNextTls()) {
                std::string tlsId = std::get<0>(nextTls);
                double distanceToTls = std::get<2>(nextTls);
                char signal = normalizeTlsSignal(static_cast<char>(std::get<3>(nextTls)));
                if (tlsId.empty() || distanceToTls < 0.0) continue;
                lastObservedTlsId_ = tlsId;
                lastObservedTlsSignal_ = signal;
                lastObservedTlsDistance_ = distanceToTls;
                break;
            }
        } catch (...) {
        }
    }
    else if (!wasOnInternalEdge) {
        bool enteredOnRed = (lastObservedTlsSignal_ == 'R' || lastObservedTlsSignal_ == 'Y');
        bool closeEnough = (lastObservedTlsDistance_ >= 0.0 &&
                            lastObservedTlsDistance_ <= TLS_VIOLATION_DISTANCE_THRESHOLD);
        if (enteredOnRed && closeEnough && !tlsViolationLatched_) {
            tlsViolationEvent = 1;
            tlsViolationLatched_ = true;
            tlsViolationActive_ = true;
        }
    }

    lastRoadId_ = currentRoadId;
}

void TraCIDemo11p::updateActiveTransportRisk(int& pedCollisionEvent,
                                             int& bikeCollisionEvent,
                                             int& activeTransportCollisionEvent)
{
    pedCollisionEvent = 0;
    bikeCollisionEvent = 0;
    activeTransportCollisionEvent = 0;
    if (!isVehicleActive_ || !mobility) return;

    TraCICommandInterface* ci = nullptr;
    try {
        ci = mobility->getCommandInterface();
    } catch (...) {
        return;
    }
    refreshActiveTransportSnapshot(ci, simTime().dbl());

    Coord myPos;
    std::string myId;
    try {
        myPos = mobility->getPositionAt(simTime());
        myId = mobility->getExternalId();
    } catch (...) {
        return;
    }

    const double collisionDistance =
        actualCollision_ ? (ACTIVE_TRANSPORT_COLLISION_DISTANCE + 1.0)
                         : ACTIVE_TRANSPORT_COLLISION_DISTANCE;

    bool pedHitNow = false;
    for (const auto& ped : s_activeTransportSnapshot.pedestrians) {
        if (myPos.distance(ped.second) <= collisionDistance) {
            pedHitNow = true;
            break;
        }
    }

    bool bikeHitNow = false;
    for (const auto& bike : s_activeTransportSnapshot.cyclists) {
        if (bike.first == myId) continue;
        if (myPos.distance(bike.second) <= collisionDistance) {
            bikeHitNow = true;
            break;
        }
    }

    if (pedHitNow && !pedCollisionLatched_) {
        pedCollisionEvent = 1;
        pedCollisionLatched_ = true;
    }
    else if (!pedHitNow) {
        pedCollisionLatched_ = false;
    }

    if (bikeHitNow && !bikeCollisionLatched_) {
        bikeCollisionEvent = 1;
        bikeCollisionLatched_ = true;
    }
    else if (!bikeHitNow) {
        bikeCollisionLatched_ = false;
    }

    if (pedCollisionEvent || bikeCollisionEvent) {
        activeTransportCollisionEvent = 1;
    }

    actualCollision_ = false;
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  handlePositionUpdate  â€“  called every step
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void TraCIDemo11p::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);

    // â”€â”€ Basic telemetry â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    double      currentTime = simTime().dbl();
    std::string vehicleId   = mobility->getExternalId();
    double      speed       = mobility->getSpeed();         // m/s

    // â”€â”€ Acceleration (numerical derivative) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    double acceleration = 0.0;
    if (prevSpeed >= 0.0) {
        double dt = (simTime() - prevTime).dbl();
        if (dt > 0) acceleration = (speed - prevSpeed) / dt;
    }
    prevSpeed = speed;
    prevTime  = simTime();

    // â”€â”€ Leader, gap, leader ID â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    auto        leaderPair  = getLeaderInfo();
    double      gap         = leaderPair.second;             // -1 = no leader
    std::string leaderId    = (gap >= 0) ? leaderPair.first : "";
    int         egoIsCav    = 1;
    int         leaderIsCav = lookupLeaderIsCav(mobility, leaderId);
    double      leaderSpeed = lookupLeaderSpeed(mobility, leaderId);

    // â”€â”€ TTC â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // -1 reported when no leader or vehicle is not closing on leader.
    double ttc = -1.0;
    if (gap >= 0) {
        double raw = computeTTC(speed, leaderSpeed, gap);
        ttc = (raw >= TTC_INFINITY) ? -1.0 : raw;
    }

    // â”€â”€ Current XY for braking distance â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    double cx = mobility->getPositionAt(simTime()).x;
    double cy = mobility->getPositionAt(simTime()).y;

    // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //  Braking episode tracking (per vehicle)
    //
    //  Episode begins: accel â‰¤ -0.5 m/sÂ² while moving
    //  Episode ends:   accel recovers â‰¥ -0.2 m/sÂ² OR vehicle stops
    //
    //  hard_brake_event      : fires once per episode when max decel â‰¤ -4
    //  emergency_brake_event : fires once per episode when max decel â‰¤ -7
    //  braking_distance_m    : Euclidean distance accumulated since episode
    //                          start; 0 when not in an episode
    // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    int hardBrakeFlag      = 0;
    int emergencyBrakeFlag = 0;

    if (!inBrakingEpisode_) {
        if (acceleration <= BRAKE_START_THRESHOLD &&
            speed > SPEED_STOP_THRESHOLD) {
            // Start new episode
            inBrakingEpisode_     = true;
            brakeEpisodeDist_     = 0.0;
            brakeEpisodeMaxDecel_ = acceleration;
            brakeLastX_           = cx;
            brakeLastY_           = cy;
            hardBrakeFired_       = false;
            emergencyBrakeFired_  = false;
        }
    } else {
        // Accumulate distance step-by-step
        double dx = cx - brakeLastX_;
        double dy = cy - brakeLastY_;
        brakeEpisodeDist_ += std::sqrt(dx*dx + dy*dy);
        brakeLastX_ = cx;
        brakeLastY_ = cy;

        // Track worst deceleration
        if (acceleration < brakeEpisodeMaxDecel_)
            brakeEpisodeMaxDecel_ = acceleration;

        // Fire hard_brake_event once per episode
        if (!hardBrakeFired_ &&
            brakeEpisodeMaxDecel_ <= HARD_BRAKE_THRESHOLD) {
            hardBrakeFired_ = true;
            hardBrakeFlag   = 1;
        }

        // Fire emergency_brake_event once per episode
        if (!emergencyBrakeFired_ &&
            brakeEpisodeMaxDecel_ <= EMERGENCY_BRAKE_THRESHOLD) {
            emergencyBrakeFired_ = true;
            emergencyBrakeFlag   = 1;
        }

        // Episode ends when vehicle stops OR accel recovers
        if (speed <= SPEED_STOP_THRESHOLD ||
            acceleration >= BRAKE_END_THRESHOLD) {
            inBrakingEpisode_ = false;
        }
    }

    double brakingDist = inBrakingEpisode_ ? brakeEpisodeDist_ : 0.0;

    // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //  Collision detection (per vehicle)
    //  Condition: stopped â‰¥ 10 s AND gap â‰¤ 1 m
    //  Flag stays 1 until vehicle starts moving again.
    // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    int collisionFlag = 0;
    if (speed <= SPEED_STOP_THRESHOLD) {
        if (stoppedSince_ < 0) stoppedSince_ = simTime();
        double stoppedFor = (simTime() - stoppedSince_).dbl();
        if (stoppedFor >= COLLISION_STOP_SECONDS &&
            gap >= 0.0 && gap <= COLLISION_GAP_THRESHOLD) {
            inCollision_  = true;
            collisionFlag = 1;
        }
    } else {
        stoppedSince_ = -1;
        inCollision_  = false;
    }

    int tlsViolationEvent = 0;
    int pedestrianCollisionEvent = 0;
    int cyclistCollisionEvent = 0;
    int activeTransportCollisionEvent = 0;
    updateTlsObservation(tlsViolationEvent);
    updateActiveTransportRisk(pedestrianCollisionEvent,
                              cyclistCollisionEvent,
                              activeTransportCollisionEvent);

    // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //  Write one consolidated row per vehicle per step
    // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (timeseriesFile && timeseriesFile->is_open()) {
        *timeseriesFile
            << std::fixed << std::setprecision(4)
            << getCurrentRunCsvPrefix()
            << currentTime              << ","
            << csvEscape(vehicleId)     << ","
            << egoIsCav                 << ","
            << "veh_av"                 << ","   // vehicle_type â€” CAV
            << speed                    << ","
            << acceleration             << ","
            << (gap < 0 ? -1.0 : gap)  << ","   // -1 = no leader
            << csvEscape(leaderId)      << ","   // empty = no leader
            << leaderIsCav              << ","
            << ttc                      << ","   // -1 = no TTC risk
            << brakingDist              << ","
            << hardBrakeFlag            << ","
            << emergencyBrakeFlag       << ","
            << collisionFlag            << ","
            << tlsViolationEvent        << ","
            << pedestrianCollisionEvent << ","
            << cyclistCollisionEvent    << ","
            << activeTransportCollisionEvent << ","
            << 1                              // involves_cav=1: this vehicle IS a CAV
            << "\n";
        timeseriesFile->flush();
    }

    // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    //  Original WSM crash-alert logic (unchanged)
    // â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    if (speed < 1) {
        if (simTime() - lastDroveAt >= 10 && !sentMessage) {
            findHost()->getDisplayString().setTagArg("i", 1, "red");
            sentMessage = true;
            TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
            populateWSM(wsm);
            wsm->setDemoData(mobility->getRoadId().c_str());
            if (dataOnSch) {
                startService(Channel::sch2, 42, "Traffic Information Service");
                scheduleAt(computeAsynchronousSendingTime(1, ChannelType::service), wsm);
            } else {
                sendDown(wsm);
            }
        }
    } else {
        lastDroveAt = simTime();
    }
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  onWSA
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void TraCIDemo11p::onWSA(DemoServiceAdvertisment* wsa)
{
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(static_cast<Channel>(wsa->getTargetChannel()));
        currentSubscribedServiceId = wsa->getPsid();
        if (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService(static_cast<Channel>(wsa->getTargetChannel()),
                         wsa->getPsid(), "Mirrored Traffic Service");
        }
    }
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  onWSM
//  BASELINE behaviour: react to ALL messages
//  immediately â€” zero authentication delay.
//  SPaT messages â†’ applySpat() called right now.
//  Rerouting WSMs â†’ changeRoute() called right now.
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void TraCIDemo11p::onWSM(BaseFrame1609_4* frame)
{
    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);
    std::string data = wsm->getDemoData();

    try {
        // â”€â”€ SPaT message? â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        SpatPayload spat;
        if (SpatPayload::parse(data, spat)) {
            applySpat(spat);
            return;
        }

        // â”€â”€ Regular rerouting WSM â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
        findHost()->getDisplayString().setTagArg("i", 1, "green");
        if (mobility->getRoadId()[0] != ':')
            traciVehicle->changeRoute(data, 9999);
        if (!sentMessage) {
            sentMessage = true;
            wsm->setSenderAddress(myId);
            wsm->setSerial(3);
            scheduleAt(simTime() + 2 + uniform(0.01, 0.2), wsm->dup());
        }
    } catch (const cRuntimeError& e) {
        EV_WARN << "[TraCIDemo11p] Vehicle left simulation â€” discarding WSM. ("
                << e.what() << ")\n";
    } catch (const std::exception& e) {
        EV_WARN << "[TraCIDemo11p] Discarding WSM for departed vehicle. ("
                << e.what() << ")\n";
    } catch (...) {
        EV_WARN << "[TraCIDemo11p] Discarding WSM (unknown error)\n";
    }
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  applySpat
//  Issues speed commands to SUMO in response to
//  a SPaT message.  Called immediately in baseline
//  (TraCIDemo11p); subclasses that model verification
//  overhead call it after their delay - that difference is the metric.
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void TraCIDemo11p::applySpat(const SpatPayload& spat)
{
    if (!isVehicleActive_) return;

    recordPdrDelivery(false, simTime().dbl());

    spatPhase_ = spat.phase;
    double mySpeed = 0.0;
    try { mySpeed = traciVehicle->getSpeed(); } catch (...) { return; }

    std::string myId;
    try { myId = mobility->getExternalId(); } catch (...) { return; }

    // SUMO already stops vehicles at red lights via its own TLS logic.
    // We do NOT call setSpeed() â€” it corrupts activeVehicleCount.
    // Baseline delay is 0 â€” vehicle acts on SPaT instantly on receipt.
    // extra_distance_m = 0 in baseline (no auth overhead).
    double extraDist = 0.0;  // baseline: zero delay, zero extra distance

    EV_INFO << "[TraCIDemo11p] SPaT phase=" << spat.phase
            << " acknowledged instantly by " << myId << " (baseline)\n";

    // Write to spat_response.csv - same columns for every variant
    // configured_delay=0, actual_delay=0, extra_distance=0 for baseline
    if (s_spatResponseFile.is_open()) {
        long stepAttempts = getLoggedStepAttempts(false, simTime().dbl());
        long stepDeliveries = getLoggedStepDeliveries(false, simTime().dbl());
        double stepPdr = getLoggedStepPdr(false, simTime().dbl());
        s_spatResponseFile << std::fixed << std::setprecision(6)
            << getCurrentRunCsvPrefix()
            << myId              << ","
            << spat.tlsId        << ","
            << spat.phase        << ","
            << spat.timeToChange << ","
            << spat.timeSent     << ","   // time_sent_s â€” stamped at RSU sendDown()
            << simTime().dbl()   << ","   // time_received = time_acted (no delay)
            << simTime().dbl()   << ","   // time_acted
            << mySpeed           << ","
            << 0.0               << ","   // configured_delay = 0
            << 0.0               << ","   // actual_delay = 0
            << extraDist         << ","   // extra_distance_m = 0
            << "BASELINE"        << ","
            << stepAttempts      << ","
            << stepDeliveries    << ","
            << stepPdr           << "\n";
        s_spatResponseFile.flush();
    }
}

// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//  handleSelfMsg
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
void TraCIDemo11p::handleSelfMsg(cMessage* msg)
{
    if (TraCIDemo11pMessage* wsm = dynamic_cast<TraCIDemo11pMessage*>(msg)) {
        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() + 1);
        if (wsm->getSerial() >= 3) {
            stopService();
            delete wsm;
        } else {
            scheduleAt(simTime() + 1, wsm);
        }
    } else {
        DemoBaseApplLayer::handleSelfMsg(msg);
    }
}


