package org.car2x.veins.subprojects.veins_inet;

import inet.applications.contract.IApp;
import inet.common.MessageDispatcher;
import inet.linklayer.ieee80211.Ieee80211Interface;
import inet.networklayer.common.InterfaceTable;
import inet.networklayer.ipv4.Ipv4NetworkLayer;
import inet.transportlayer.udp.Udp;

//
// Lightweight INET host for SUMO/TraCI-spawned CAV nodes.
//
// The default AdhocHost stack brings in a lot of generic machinery that this
// project does not use (TCP, SCTP, loopback, multi-dispatcher glue, etc.).
// This trimmed host keeps only the pieces needed for:
// - VeinsInetMobility / TraCI position updates
// - one 802.11p-style wireless interface
// - IPv4 + UDP multicast delivery
// - the custom VeinsInet5GVehicleApp
//
module VeinsInetLiteCar
{
    parameters:
        @display("i=device/cellphone");
        @networkNode();
        app[*].interfaceTableModule = default("^.interfaceTable");
        ipv4.interfaceTableModule = default("^.interfaceTable");
        udp.interfaceTableModule = default("^.interfaceTable");
        wlan[*].interfaceTableModule = default("^.interfaceTable");
        wlan[*].radio.antenna.mobilityModule = default("^.^.^.^.mobility");
    gates:
        input radioIn[1] @directIn;
    submodules:
        mobility: VeinsInetMobility {
            @display("p=90,120");
        }
        interfaceTable: InterfaceTable {
            @display("p=90,210;is=s");
        }
        app[1]: <default("org.car2x.veins.subprojects.veins_inet.VeinsInet5GVehicleApp")> like IApp {
            @display("p=90,40");
        }
        udp: Udp {
            @display("p=250,40");
        }
        ipv4: Ipv4NetworkLayer {
            @display("p=250,130;q=queue");
        }
        nl: MessageDispatcher {
            @display("p=250,185;b=220,5");
        }
        wlan[1]: Ieee80211Interface {
            @display("p=250,235;q=queue");
        }
    connections allowunconnected:
        radioIn[0] --> { @display("m=s"); } --> wlan[0].radioIn;

        app[0].socketOut --> udp.appIn;
        app[0].socketIn <-- udp.appOut;

        udp.ipOut --> ipv4.transportIn;
        udp.ipIn <-- ipv4.transportOut;

        ipv4.ifOut --> nl.in++;
        ipv4.ifIn <-- nl.out++;

        nl.out++ --> wlan[0].upperLayerIn;
        nl.in++ <-- wlan[0].upperLayerOut;
}
