package org.car2x.veins.subprojects.veins_inet;

import inet.applications.contract.IApp;

simple VeinsInet5GRsuApp like IApp
{
    parameters:
        @class(VeinsInet5GRsuApp);
        string interfaceTableModule;
        string interface = default("wlan0");
        int port = default(9001);
        string monitoredTlsId = default("");
        double spatInterval @unit(s) = default(0.1s);
        double actualSpatInterval @unit(s) = default(1s);
        @display("i=block/app");
        @signal[packetSent](type=inet::Packet);
        @signal[packetReceived](type=inet::Packet);
        @statistic[packetSent](title="packets sent"; source=packetSent; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
        @statistic[packetReceived](title="packets received"; source=packetReceived; record=count,"sum(packetBytes)","vector(packetBytes)"; interpolationmode=none);
    gates:
        input socketIn @labels(UdpControlInfo/up);
        output socketOut @labels(UdpControlInfo/down);
}
