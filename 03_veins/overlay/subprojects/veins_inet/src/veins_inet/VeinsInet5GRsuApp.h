#pragma once

#include <string>
#include <vector>

#include "inet/transportlayer/contract/udp/UdpSocket.h"
#include "veins/modules/utility/TimerManager.h"
#include "veins_inet/veins_inet.h"

class VEINS_INET_API VeinsInet5GRsuApp : public omnetpp::cSimpleModule, public inet::UdpSocket::ICallback {
protected:
    int numInitStages() const override;
    void initialize(int stage) override;
    void handleMessage(omnetpp::cMessage* msg) override;
    void socketDataArrived(inet::UdpSocket* socket, inet::Packet* packet) override;
    void socketErrorArrived(inet::UdpSocket* socket, inet::Indication* indication) override;
    void socketClosed(inet::UdpSocket* socket) override;
    void finish() override;

private:
    inet::UdpSocket socket_;
    veins::TimerManager timerManager_{this};
    inet::L3Address destAddress_;

    std::vector<std::string> monitoredTlsIds_;
    std::string interface_;
    int port_ = 9001;
    double spatInterval_ = 0.1;
    double actualSpatInterval_ = 1.0;
    long statSpatTx_ = 0;

    omnetpp::simtime_t getActualSpatInterval() const;
    void scheduleSpat();
    void broadcastSpat();
    void sendSpat(const std::string& tlsId, char phase, const std::string& state, double timeToChange);
    char queryTlsPhaseFor(const std::string& tlsId, double& timeToChange, std::string& tlsState) const;
};
