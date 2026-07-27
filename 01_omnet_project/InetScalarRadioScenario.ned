//
// InetScalarRadioScenario — control topology for the propagation comparison.
//
// Uses Ieee80211ScalarRadioMedium, so the stock INET 802.11 scalar radio is the
// ONLY channel model: path loss and SINR rejection happen in the radio layer and
// the analytical PC5 model in VeinsInet5GVehicleApp is bypassed
// (bypassAnalyticalChannel=true). This is the "Plain" control treatment that the
// calibrated PC5 models are measured against.
//
// Vehicles are lightweight INET nodes (VeinsInetLiteCar) driven by SUMO/TraCI
// through veins_inet.
//

import inet.environment.common.PhysicalEnvironment;
import inet.node.inet.AdhocHost;
import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.common.IntegratedVisualizer;
import org.car2x.veins.subprojects.veins_inet.VeinsInetLiteCar;
import org.car2x.veins.subprojects.veins_inet.VeinsInetManager;
import org.car2x.veins.visualizer.roads.RoadsCanvasVisualizer;

network InetScalarRadioScenario
{
    parameters:
        bool useOsg = default(false);
        bool enableVisualizers = default(false);
        @display("bgb=600,500");

    submodules:
        radioMedium: Ieee80211ScalarRadioMedium {
            @display("p=80,120");
        }

        manager: VeinsInetManager {
            @display("p=220,120");
        }

        visualizer: IntegratedVisualizer if enableVisualizers {
            @display("p=80,220");
        }

        physicalEnvironment: PhysicalEnvironment {
            @display("p=220,220");
        }

        roadsCanvasVisualizer: RoadsCanvasVisualizer if enableVisualizers {
            @display("p=80,320");
        }

        rsu[1]: AdhocHost {
            @display("p=480,220;i=veins/sign/yellowdiamond;is=vs");
        }

        // VeinsInetManager expects a resizable "node" vector on the parent
        // network and will grow/shrink it as SUMO vehicles appear/disappear.
        node[0]: VeinsInetLiteCar;
}
