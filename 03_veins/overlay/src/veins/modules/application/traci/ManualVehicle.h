#pragma once
// ─────────────────────────────────────────────────────────────────────────────
//  ManualVehicle.h
//
//  NOTE ON DESIGN — why this is NOT a separate OMNeT++ module type
//  ───────────────────────────────────────────────────────────────
//  In Veins, *.node[*].applType assigns ONE application type to ALL vehicles.
//  The SUMO vType (veh_human / veh_av) is only available at runtime via
//  traciVehicle->getTypeId() — not at ini-parse time.
//
//  The *.node[type=veh_human] syntax does NOT work in Veins ini files.
//  The *.manager.moduleType parameter controls the outer compound node
//  module (Car/VNode), not the inner applType — and requires separate
//  compound NED files for each type, which would break your existing setup.
//
//  CORRECT APPROACH: ManualVehicle is a thin subclass of TraCIDemo11p.
//  It overrides initialize() to call traciVehicle->getTypeId() at stage 1,
//  and sets manualMode_ = true if the vehicle is "veh_human".
//  All V2X methods (onWSM, applySpat, spat_response.csv) become no-ops.
//  CSV logging still works — vehicle_type column = "veh_human".
//
//  HOW TO USE IN INI:
//  ──────────────────
//  Replace:   *.node[*].applType = "Car5G"
//  With:      *.node[*].applType = "ManualVehicle"
//
//  Then add:  *.node[*].appl.manualVehicleType = "veh_human"
//
//  ManualVehicle checks getTypeId() at runtime.
//  If it matches manualVehicleType → V2X disabled, logs as "veh_human".
//  If it does NOT match → full V2X stack active, logs as "veh_av".
//
//  This means ManualVehicle is the SINGLE applType for all vehicles.
//  It adapts at runtime based on the SUMO vType. No second module needed.
//
//  For 5G scenarios:   *.node[*].applType = "ManualVehicle5G"
//  (subclasses provided — same pattern, different V2X base)
// ─────────────────────────────────────────────────────────────────────────────

#include "veins/modules/application/traci/TraCIDemo11p.h"
#include <string>

namespace veins {

class VEINS_API ManualVehicle : public TraCIDemo11p {

public:
    void initialize(int stage) override;
    void finish()              override;

protected:
    // ── Runtime mode flag ─────────────────────────────────────
    // Set in initialize() stage 1 by reading traciVehicle->getTypeId().
    // true  → this vehicle is veh_human: V2X stack disabled
    // false → this vehicle is veh_av:    full V2X stack via TraCIDemo11p
    bool manualMode_ = false;

    // SUMO vehicle type string as read at init (e.g. "veh_human")
    std::string sumoVehicleType_;

    // Which SUMO type(s) should be treated as manual (ini parameter)
    // Default "veh_human" — can be overridden per scenario
    std::string manualVehicleType_;

    // Cached vehicle ID — populated on first handlePositionUpdate().
    // Avoids calling mobility->getExternalId() after vehicle is removed.
    std::string cachedVehicleId_;

    // ── V2X overrides — disabled when manualMode_ == true ─────
    void onWSM(BaseFrame1609_4* frame)        override;
    void onWSA(DemoServiceAdvertisment* wsa)  override;
    void handlePositionUpdate(cObject* obj)   override;
    void handleSelfMsg(cMessage* msg)         override;
};

} // namespace veins
