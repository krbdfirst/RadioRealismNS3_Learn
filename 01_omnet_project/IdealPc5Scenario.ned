//
// IdealPc5Scenario — transparent radio testbed for calibrated 5G NR PC5 runs.
//
// Uses Ieee80211DimensionalRadioMedium configured as a transparent channel:
// the radio layer delivers all packets within range without any PHY-layer
// path-loss drops or SINR rejection. All PC5 channel characteristics (path
// loss, half-duplex, SPS collision, OBU capacity) come exclusively from the
// analytical model in VeinsInet5GVehicleApp (bypassAnalyticalChannel=false).
// This is the recommended topology for MATLAB-calibrated parameter sweeps.
//

import inet.environment.common.PhysicalEnvironment;
import inet.node.inet.AdhocHost;
import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211DimensionalRadioMedium;
import inet.visualizer.common.IntegratedVisualizer;
import org.car2x.veins.subprojects.veins_inet.VeinsInetCar;
import org.car2x.veins.subprojects.veins_inet.VeinsInetManager;
import org.car2x.veins.visualizer.roads.RoadsCanvasVisualizer;

network IdealPc5Scenario
{
    parameters:
        bool useOsg = default(false);
        bool enableVisualizers = default(false);
        @display("bgb=600,500");

    submodules:
        radioMedium: Ieee80211DimensionalRadioMedium {
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

        node[0]: VeinsInetCar;
}
