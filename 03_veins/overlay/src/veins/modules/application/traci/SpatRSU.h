#pragma once
// ─────────────────────────────────────────────────────────────
//  SpatRSU.h
//
//  BASELINE RSU — broadcasts SPaT with ZERO authentication
//  overhead. Used in Baseline_SPaT config only.
//
//  Inherits TraCIDemoRSU11p directly. No pending queue and
//  no auth delay of any kind.
//  Every SPaT message is sent immediately when the timer fires.
// ─────────────────────────────────────────────────────────────

#include "veins/modules/application/traci/TraCIDemoRSU11p.h"
#include "SpatMessage.h"
#include <string>
#include <vector>

namespace veins {

class VEINS_API SpatRSU : public TraCIDemoRSU11p {
public:
    void initialize(int stage) override;
    void finish()              override;

protected:
    void handleSelfMsg(cMessage* msg)        override;
    void onWSM(BaseFrame1609_4* frame)       override;
    void onWSA(DemoServiceAdvertisment* wsa) override;

protected:
    std::vector<std::string> monitoredTlsIds_;
    double                   spatInterval_ = 0.1;
    cMessage*                spatTimer_ = nullptr;

    virtual void broadcastSpat();
    virtual void sendSpatPayload(const SpatPayload& spat);
    char queryTlsPhaseFor(const std::string& tlsId, double& timeToChange, std::string& tlsState);
    void logSpatBroadcast(const SpatPayload& spat);
};

} // namespace veins
