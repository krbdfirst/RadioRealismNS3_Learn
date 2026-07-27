//
// Copyright (C) 2018 Christoph Sommer <sommer@ccs-labs.org>
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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

//
// Veins "application layer" module for the INET Framework
// Simplified for dynamically spawned TraCI hosts: avoid ApplicationBase
// lifecycle interactions and bring the UDP socket up explicitly at the
// final initialization stage.
//

#include "veins_inet/VeinsInetApplicationBase.h"

#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/TagBase_m.h"
#include "inet/common/TimeTag_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/transportlayer/contract/udp/UdpControlInfo_m.h"

namespace veins {

using namespace inet;

Define_Module(VeinsInetApplicationBase);

VeinsInetApplicationBase::VeinsInetApplicationBase()
{
}

int VeinsInetApplicationBase::numInitStages() const
{
    return inet::NUM_INIT_STAGES;
}

void VeinsInetApplicationBase::initialize(int stage)
{
    if (stage != inet::INITSTAGE_LAST) return;

    mobility = veins::VeinsInetMobilityAccess().get(getParentModule());
    traci = mobility ? mobility->getCommandInterface() : nullptr;
    traciVehicle = mobility ? mobility->getVehicleCommandInterface() : nullptr;

    L3AddressResolver().tryResolve("224.0.0.1", destAddress);
    ASSERT(!destAddress.isUnspecified());

    socket.setOutputGate(gate("socketOut"));
    socket.bind(L3Address(), portNumber);

    const char* interface = par("interface");
    ASSERT(interface[0]);
    IInterfaceTable* ift = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this);
#if INET_VERSION >= 0x0403
    NetworkInterface* ie = ift->findInterfaceByName(interface);
#elif INET_VERSION >= 0x0402
    InterfaceEntry* ie = ift->findInterfaceByName(interface);
#else
    InterfaceEntry* ie = ift->getInterfaceByName(interface);
#endif
    ASSERT(ie);
    socket.setMulticastOutputInterface(ie->getInterfaceId());
    socket.setMulticastLoop(false);
    socket.joinMulticastGroup(destAddress, ie->getInterfaceId());
    socket.setCallback(this);

    started_ = startApplication();
    ASSERT(started_);
}

bool VeinsInetApplicationBase::startApplication()
{
    return true;
}

bool VeinsInetApplicationBase::stopApplication()
{
    return true;
}

void VeinsInetApplicationBase::finish()
{
    if (started_) stopApplication();
}

VeinsInetApplicationBase::~VeinsInetApplicationBase()
{
}

void VeinsInetApplicationBase::refreshDisplay() const
{
    char buf[100];
    sprintf(buf, "okay");
    getDisplayString().setTagArg("t", 0, buf);
}

void VeinsInetApplicationBase::handleMessage(cMessage* msg)
{
    if (timerManager.handleMessage(msg)) return;

    socket.processMessage(msg);
}

void VeinsInetApplicationBase::socketDataArrived(UdpSocket* socket, Packet* packet)
{
    auto pk = std::shared_ptr<inet::Packet>(packet);

    // ignore local echoes
    auto srcAddr = pk->getTag<L3AddressInd>()->getSrcAddress();
    if (srcAddr == Ipv4Address::LOOPBACK_ADDRESS) {
        EV_DEBUG << "Ignored local echo: " << pk.get() << endl;
        return;
    }

    // statistics
    emit(packetReceivedSignal, pk.get());

    // process incoming packet
    processPacket(pk);
}

void VeinsInetApplicationBase::socketErrorArrived(UdpSocket* socket, Indication* indication)
{
    EV_WARN << "Ignoring UDP error report " << indication->getName() << endl;
    delete indication;
}

void VeinsInetApplicationBase::socketClosed(UdpSocket* socket)
{
}

void VeinsInetApplicationBase::timestampPayload(inet::Ptr<inet::Chunk> payload)
{
    payload->removeTagIfPresent<CreationTimeTag>(b(0), b(-1));
    auto creationTimeTag = payload->addTag<CreationTimeTag>();
    creationTimeTag->setCreationTime(simTime());
}

void VeinsInetApplicationBase::sendPacket(std::unique_ptr<inet::Packet> pk)
{
    emit(packetSentSignal, pk.get());
    socket.sendTo(pk.release(), destAddress, portNumber);
}

std::unique_ptr<inet::Packet> VeinsInetApplicationBase::createPacket(std::string name)
{
    return std::unique_ptr<Packet>(new Packet(name.c_str()));
}

void VeinsInetApplicationBase::processPacket(std::shared_ptr<inet::Packet> pk)
{
}

} // namespace veins
