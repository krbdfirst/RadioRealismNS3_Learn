#pragma once

#include <cstdlib>
#include <string>
#include <omnetpp.h>

namespace veins {

inline std::string getCurrentConfigVariable(const char* name)
{
    auto* sim = omnetpp::cSimulation::getActiveSimulation();
    if (!sim) return "";

    auto* envir = sim->getEnvir();
    if (!envir) return "";

    auto* config = envir->getConfigEx();
    if (!config) return "";

    const char* value = config->getVariable(name);
    return value ? value : "";
}

inline std::string getCurrentRunId()
{
    return getCurrentConfigVariable(CFGVAR_RUNID);
}

inline std::string getCurrentConfigName()
{
    return getCurrentConfigVariable(CFGVAR_CONFIGNAME);
}

inline std::string getCurrentResultDir()
{
    return getCurrentConfigVariable(CFGVAR_RESULTDIR);
}

inline std::string getCurrentSeedSet()
{
    return getCurrentConfigVariable(CFGVAR_SEEDSET);
}

inline std::string getCurrentRunNumberString()
{
    return getCurrentConfigVariable(CFGVAR_RUNNUMBER);
}

inline std::string getCurrentRunCsvPrefix()
{
    return getCurrentRunNumberString() + "," + getCurrentSeedSet() + "," + getCurrentRunId() + ",";
}

inline long getCurrentRunNumber()
{
    std::string value = getCurrentRunNumberString();
    if (value.empty()) return 0;

    char* end = nullptr;
    long parsed = std::strtol(value.c_str(), &end, 10);
    return (end && *end == '\0') ? parsed : 0;
}

} // namespace veins
