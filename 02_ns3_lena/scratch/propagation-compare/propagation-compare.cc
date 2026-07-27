/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
//
// propagation-compare.cc
//
// NR V2X (5G-LENA) sidelink Mode-2 replay of SUMO/Veins CAV mobility, built to
// mirror the OMNeT/Veins "PropagationRealismProject" baseline as closely as the
// two stacks allow, so PC5 communication behavior can be compared against the
// three OMNeT propagation treatments across CAV adoption rates.
//
// Mobility is replayed from a CAV-only ns-2 trace produced by
// make_cav_ns3_inputs.py (filtered to veh_av, antenna height 1.5 m). Every CAV
// is both a transmitter and receiver of 10 Hz, 256-byte CAMs (groupcast), which
// matches the OMNeT app (beaconInterval=0.1s, VeinsInet5GMessage chunk=B(256)).
//
// Built from contrib/nr/examples/nr-v2x-examples/nr-v2x-west-to-east-highway.cc.
//
// OMNeT  ->  NS-3 parameter map  (see README_NS3_MAPPING.md for the full table)
//   5.9 GHz band ............ centralFrequencyBandSl = 5.9e9
//   radio.bandwidth 10 MHz .. bandwidthBandSl = 100  (x100 kHz)
//   pc5TxPowerDbm 23 ........ txPower = 23 dBm
//   pc5NoiseFigureDb 7 ...... NrUePhy NoiseFigure = 7 dB
//   CAM 10 Hz / 256 B ....... dataRateBe=20.48 kb/s, packetSizeBe=256, RRI=100 ms
//   first-beacon jitter ..... per-UE uniform(0,0.1s) app start jitter
//   pc5Range 300 m ... V2xKpi range = 300 m
//   v2vPropagationModel ..... channelScenario: V2V_Urban (TR 37.885) default,
//                             or UMi_StreetCanyon (direct analog of tr38901_umi)
//   spatUtHeight 1.5 m ...... antenna z = 1.5 m (in the ns-2 trace)
//

#include "v2x-kpi.h"

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/stats-module.h"

#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("PropagationCompare");

// ---- global counters / accumulators -----------------------------------------
uint64_t g_txPkt = 0;
uint64_t g_rxPkt = 0;
double g_latencySum = 0.0; // seconds
uint64_t g_latencyCnt = 0;

// ---- DB trace sinks (same as the nr v2x examples) ---------------------------
void
NotifySlPscchScheduling(UeMacPscchTxOutputStats* s, const SlPscchUeMacStatParameters p)
{
    s->Save(p);
}

void
NotifySlPsschScheduling(UeMacPsschTxOutputStats* s, const SlPsschUeMacStatParameters p)
{
    s->Save(p);
}

void
NotifySlPscchRx(UePhyPscchRxOutputStats* s, const SlRxCtrlPacketTraceParams p)
{
    s->Save(p);
}

void
NotifySlPsschRx(UePhyPsschRxOutputStats* s, const SlRxDataPacketTraceParams p)
{
    s->Save(p);
}

void
NotifySlRlcPduRx(UeRlcRxOutputStats* stats,
                 uint64_t imsi,
                 uint16_t rnti,
                 uint16_t txRnti,
                 uint8_t lcid,
                 uint32_t rxPduSize,
                 double delay)
{
    stats->Save(imsi, rnti, txRnti, lcid, rxPduSize, delay);
}

// Application TX/RX trace -> DB + global counters/latency
void
UePacketTraceDb(UeToUePktTxRxOutputStats* stats,
                Ptr<Node> node,
                const Address& localAddrs,
                std::string txRx,
                Ptr<const Packet> p,
                const Address& srcAddrs,
                const Address& dstAddrs,
                const SeqTsSizeHeader& seqTsSizeHeader)
{
    uint32_t nodeId = node->GetId();
    uint64_t imsi = node->GetDevice(0)->GetObject<NrUeNetDevice>()->GetImsi();
    uint32_t seq = seqTsSizeHeader.GetSeq();
    uint32_t pktSize = p->GetSize() + seqTsSizeHeader.GetSerializedSize();
    stats->Save(txRx, localAddrs, nodeId, imsi, pktSize, srcAddrs, dstAddrs, seq);

    if (txRx == "tx")
    {
        ++g_txPkt;
    }
    else if (txRx == "rx")
    {
        ++g_rxPkt;
        double lat = (Simulator::Now() - seqTsSizeHeader.GetTs()).GetSeconds();
        if (lat >= 0.0)
        {
            g_latencySum += lat;
            ++g_latencyCnt;
        }
    }
}

void
SavePositionPerIP(V2xKpi* v2xKpi)
{
    for (auto it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        for (int j = 0; j < node->GetNDevices(); j++)
        {
            Ptr<NrUeNetDevice> uedev = node->GetDevice(j)->GetObject<NrUeNetDevice>();
            if (uedev)
            {
                Ptr<Ipv4L3Protocol> ipv4 = node->GetObject<Ipv4L3Protocol>();
                std::ostringstream ip;
                ip << ipv4->GetAddress(1, 0).GetLocal();
                v2xKpi->FillPosPerIpMap(ip.str(), node->GetObject<MobilityModel>()->GetPosition());
            }
        }
    }
}

// Map a channel-scenario string to the nr BandwidthPartInfo enum.
static BandwidthPartInfo::Scenario
ScenarioFromString(const std::string& s)
{
    if (s == "V2V_Urban")
    {
        return BandwidthPartInfo::V2V_Urban;
    }
    if (s == "V2V_Highway")
    {
        return BandwidthPartInfo::V2V_Highway;
    }
    if (s == "UMi_StreetCanyon")
    {
        return BandwidthPartInfo::UMi_StreetCanyon;
    }
    if (s == "UMi_StreetCanyon_LoS")
    {
        // Forced-LOS UMi (AlwaysLosChannelConditionModel) — no NLOS of any kind.
        // Used for the LOS-only fair comparison: matches OMNeT Analytical-LOS exactly.
        return BandwidthPartInfo::UMi_StreetCanyon_LoS;
    }
    NS_ABORT_MSG("Unknown channelScenario '" << s
                                             << "'. Use V2V_Urban | V2V_Highway | UMi_StreetCanyon | UMi_StreetCanyon_LoS");
}

// Read "key=value" lines from the <tag>_info.txt sidecar.
static std::map<std::string, std::string>
ReadInfo(const std::string& path)
{
    std::map<std::string, std::string> kv;
    std::ifstream f(path);
    NS_ABORT_MSG_IF(!f.is_open(), "Cannot open info file: " << path);
    std::string line;
    while (std::getline(f, line))
    {
        auto eq = line.find('=');
        if (eq != std::string::npos)
        {
            kv[line.substr(0, eq)] = line.substr(eq + 1);
        }
    }
    return kv;
}

// Read <tag>_meta.csv -> per-node (depart, arrive) seconds, indexed by nodeId.
static void
ReadMeta(const std::string& path, std::vector<double>& depart, std::vector<double>& arrive)
{
    std::ifstream f(path);
    NS_ABORT_MSG_IF(!f.is_open(), "Cannot open meta file: " << path);
    std::string line;
    std::getline(f, line); // header
    while (std::getline(f, line))
    {
        std::stringstream ss(line);
        std::string nid, vid, dep, arr;
        std::getline(ss, nid, ',');
        std::getline(ss, vid, ',');
        std::getline(ss, dep, ',');
        std::getline(ss, arr, ',');
        if (nid.empty())
        {
            continue;
        }
        uint32_t i = std::stoul(nid);
        if (depart.size() <= i)
        {
            depart.resize(i + 1, 0.0);
            arrive.resize(i + 1, 0.0);
        }
        depart[i] = std::stod(dep);
        arrive[i] = std::stod(arr);
    }
}

int
main(int argc, char* argv[])
{
    // ---- inputs (mobility produced by make_cav_ns3_inputs.py) ----------------
    std::string inputsDir = "./inputs";
    std::string tag = "cav25";

    // ---- OMNeT-matched defaults ----------------------------------------------
    double centralFrequencyBandSl = 5.9e9; // OMNeT 5.9 GHz band
    uint16_t bandwidthBandSl = 100;        // x100 kHz => 10 MHz (radio.bandwidth)
    uint16_t numerologyBwpSl = 0;          // 15 kHz SCS @ 10 MHz
    double txPower = 23;                    // pc5TxPowerDbm
    double noiseFigure = 7.0;               // pc5NoiseFigureDb
    uint32_t udpPacketSizeBe = 256;         // VeinsInet5GMessage chunk = B(256)
    double dataRateBe = 20.48;              // kb/s => 256B @ 10 Hz (CAM rate)
    uint16_t reservationPeriod = 100;       // ms (10 Hz CAM RRI)
    std::string channelScenario = "V2V_Urban"; // or UMi_StreetCanyon (== tr38901_umi)
    bool enableShadowing = true;            // OMNeT analytical model includes shadowing

    // NR sidelink resource pool / sensing
    bool enableSensing = true;              // OMNeT SPS uses sensing-based selection
    int slThresPsschRsrp = -110;            // NR sensing RSRP threshold (dBm); cf. pc5SensingThresholdDbm=-94
    uint16_t slSubchannelSize = 10;         // RBs per subchannel (10 MHz -> ~5 subchannels)
    uint16_t slSensingWindow = 100;         // T0 (ms)
    uint16_t slSelectionWindow = 5;         // T2min factor
    uint16_t slMaxNumPerReserve = 3;
    double slProbResourceKeep = 0.0;
    uint16_t slMaxTxTransNumPssch = 5;
    uint16_t t1 = 2;
    uint16_t t2 = 33;
    uint8_t mcs = 14;
    bool harqEnabled = true;
    // Optional ASYMMETRIC flood node (held-out validation of the load-driven contention
    // model): ONE UE injects high-rate sidelink traffic so load is CONCENTRATED, not
    // distributed. NR-V2X Mode 2 supports dynamic (aperiodic) scheduling (3GPP TS 38.321)
    // for bursty traffic; SPS is capped at 50 Hz (RRI >= 20-slot pool) so a real flood uses
    // a dynamic grant. floodNodeRate=0 => disabled (normal sweep, behaviour byte-identical).
    double floodNodeRate = 0.0;     // flood packets/s on ONE node (0 = off)
    bool floodNodeDynamic = true;   // dynamic/aperiodic SL grant for the flood flow
    int floodNodeId = -1;           // which UE floods (-1 = auto: the longest-present UE)
    std::string tddPattern = "DL|DL|DL|F|UL|UL|UL|UL|UL|UL|";
    std::string slBitMap = "1|1|1|1|1|1|0|0|0|1|1|1";
    uint16_t channelUpdatePeriod = 100; // ms

    // simulation
    double simTime = 20.0;                  // seconds of CAM exchange (raise toward 200 to match OMNeT)
    double slBearersActivationTime = 1.0;   // s
    uint16_t kpiRange = 300;                // m (pc5Range)
    std::string outputDir = "./";
    std::string simTag = "";
    bool logging = false;
    // Per-slot PHY/MAC trace tables (pscch/pssch/rlc) are HUGE at high adoption
    // (one psschRxUePhy row per neighbor per TX -> multi-GB DB + crashes). They
    // are NOT needed for PRR/PDR/PIR/latency (those come from the app-level
    // pktTxRx table), so they are OFF by default. Enable only for PHY debugging.
    bool phyTraces = false;
    // Split of phyTraces (added for NS-3->OMNeT distillation teacher-data collection):
    //   macSummary  = TX-side scheduling traces only (pscch/psschTxUeMac). One row per TX,
    //                 cheap even at cav100. Feeds simulPsschTx (resource collisions) and the
    //                 HARQ fields (rv/reselCounter/gapReTx). Safe to run on ALL rates.
    //   phyRxTrace  = RX-side per-reception traces (pscch/psschRxUePhy + rlcRx). HUGE at high
    //                 load. Feeds PsschTbRx (decode), per-reception SINR, latency. Low rate only.
    // --phyTraces stays as a master switch enabling BOTH (back-compat with existing scripts).
    bool macSummary = false;
    bool phyRxTrace = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("inputsDir", "Directory holding <tag>_mobility.tcl/_meta.csv/_info.txt", inputsDir);
    cmd.AddValue("tag", "Input basename, e.g. cav25", tag);
    cmd.AddValue("simTime", "Seconds of CAM exchange to simulate", simTime);
    cmd.AddValue("channelScenario",
                 "V2V_Urban | V2V_Highway | UMi_StreetCanyon (UMi == OMNeT tr38901_umi)",
                 channelScenario);
    cmd.AddValue("centralFrequencyBandSl", "Carrier frequency (Hz)", centralFrequencyBandSl);
    cmd.AddValue("bandwidthBandSl", "System bandwidth in multiples of 100 kHz", bandwidthBandSl);
    cmd.AddValue("numerologyBwpSl", "SL numerology", numerologyBwpSl);
    cmd.AddValue("txPower", "TX power (dBm)", txPower);
    cmd.AddValue("noiseFigure", "UE noise figure (dB)", noiseFigure);
    cmd.AddValue("packetSizeBe", "CAM payload bytes", udpPacketSizeBe);
    cmd.AddValue("dataRateBe", "CAM data rate (kb/s)", dataRateBe);
    cmd.AddValue("reservationPeriod", "SPS reservation period / RRI (ms)", reservationPeriod);
    cmd.AddValue("floodNodeRate", "Asymmetric flood: packets/s on ONE node (0=off)", floodNodeRate);
    cmd.AddValue("floodNodeDynamic", "Flood flow uses dynamic (aperiodic) SL grant (TS 38.321)", floodNodeDynamic);
    cmd.AddValue("floodNodeId", "UE id to flood (-1 = auto: longest-present UE)", floodNodeId);
    cmd.AddValue("enableSensing", "Sensing-based resource selection", enableSensing);
    cmd.AddValue("slThresPsschRsrp", "Sensing RSRP threshold (dBm)", slThresPsschRsrp);
    cmd.AddValue("slSubchannelSize", "Subchannel size (RBs)", slSubchannelSize);
    cmd.AddValue("enableShadowing", "Enable 3GPP shadowing", enableShadowing);
    cmd.AddValue("mcs", "Sidelink MCS", mcs);
    cmd.AddValue("kpiRange", "Range (m) for V2X PRR KPI", kpiRange);
    cmd.AddValue("outputDir", "Output directory", outputDir);
    cmd.AddValue("simTag", "Tag appended to output filenames", simTag);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("phyTraces",
                 "Master switch: enable BOTH macSummary and phyRxTrace (huge; off by default)",
                 phyTraces);
    cmd.AddValue("macSummary",
                 "TX scheduling traces only (collisions+HARQ; cheap, safe on all rates)",
                 macSummary);
    cmd.AddValue("phyRxTrace",
                 "RX per-reception traces (decode/SINR/latency; HUGE at high load)",
                 phyRxTrace);
    cmd.Parse(argc, argv);
    if (phyTraces) { macSummary = true; phyRxTrace = true; }

    if (simTag.empty())
    {
        simTag = tag + "-" + channelScenario;
    }

    NS_ABORT_IF(centralFrequencyBandSl > 6e9);

    if (logging)
    {
        LogLevel l = (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL);
        LogComponentEnable("PropagationCompare", l);
    }

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // ---- load CAV mobility inputs --------------------------------------------
    std::string mobFile = inputsDir + "/" + tag + "_mobility.tcl";
    std::string metaFile = inputsDir + "/" + tag + "_meta.csv";
    std::string infoFile = inputsDir + "/" + tag + "_info.txt";
    auto info = ReadInfo(infoFile);
    uint32_t numUes = std::stoul(info.at("numUes"));
    std::vector<double> depart;
    std::vector<double> arrive;
    ReadMeta(metaFile, depart, arrive);
    NS_ABORT_MSG_IF(depart.size() != numUes, "meta/info UE count mismatch");
    std::cout << "tag=" << tag << " numUes(CAV)=" << numUes << " scenario=" << channelScenario
              << " simTime=" << simTime << "s" << std::endl;

    // ---- create UE nodes FIRST so ns-2 $node_(i) maps to NodeList[i] ---------
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    Ns2MobilityHelper ns2(mobFile);
    ns2.Install(); // maps $node_(i) -> NodeList id i (only UE nodes exist now)
    // The 3GPP V2V_Urban / UMi channel-condition models require MobilityBuildingInfo
    // on every node. We install it with NO Building objects, so LOS/NLOS stays
    // probabilistic (matching OMNeT tr38901_umi's statistical LOS, and OMNeT's
    // IdealObstacleLoss = no deterministic building blockage).
    BuildingsHelper::Install(ueNodes);

    // ---- NR stack ------------------------------------------------------------
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetEpcHelper(epcHelper);

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    CcBwpCreator::SimpleOperationBandConf bandConfSl(centralFrequencyBandSl,
                                                     bandwidthBandSl,
                                                     numCcPerBand,
                                                     ScenarioFromString(channelScenario));
    OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc(bandConfSl);

    if (enableShadowing)
    {
        Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                           TimeValue(MilliSeconds(channelUpdatePeriod)));
        nrHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                    TimeValue(MilliSeconds(channelUpdatePeriod)));
        nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    }
    else
    {
        Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
        nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
        nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    }

    nrHelper->InitializeOperationBand(&bandSl);
    allBwps = CcBwpCreator::GetAllBwps({bandSl});

    Packet::EnableChecking();
    Packet::EnablePrinting();
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // Quasi-omnidirectional UE antenna (no beamforming in SL)
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(txPower));
    nrHelper->SetUePhyAttribute("NoiseFigure", DoubleValue(noiseFigure));

    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId());
    nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(enableSensing));
    nrHelper->SetUeMacAttribute("T1", UintegerValue(static_cast<uint8_t>(t1)));
    nrHelper->SetUeMacAttribute("T2", UintegerValue(t2));
    nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));
    nrHelper->SetUeMacAttribute("SlThresPsschRsrp", IntegerValue(slThresPsschRsrp));

    uint8_t bwpIdForGbrMcptt = 0;
    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpIdForGbrMcptt));
    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdForGbrMcptt);

    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);
    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // ---- Sidelink configuration ----------------------------------------------
    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    nrSlHelper->SetEpcHelper(epcHelper);
    nrSlHelper->SetSlErrorModel("ns3::NrEesmIrT1");
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(mcs));
    nrSlHelper->PrepareUeForSidelink(ueNetDev, bwpIdContainer);

    LteRrcSap::SlResourcePoolNr slResourcePoolNr;
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();
    std::vector<std::bitset<1>> slBitMapVector;
    {
        std::stringstream ss(slBitMap);
        std::string tok;
        while (std::getline(ss, tok, '|'))
        {
            slBitMapVector.emplace_back(std::stoi(tok) & 0x01);
        }
    }
    ptrFactory->SetSlTimeResources(slBitMapVector);
    ptrFactory->SetSlSensingWindow(slSensingWindow);
    ptrFactory->SetSlSelectionWindow(slSelectionWindow);
    ptrFactory->SetSlFreqResourcePscch(10);
    ptrFactory->SetSlSubchannelSize(slSubchannelSize);
    ptrFactory->SetSlMaxNumPerReserve(slMaxNumPerReserve);
    std::list<uint16_t> resourceReservePeriodList = {0, reservationPeriod};
    ptrFactory->SetSlResourceReservePeriodList(resourceReservePeriodList);
    LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool();
    slResourcePoolNr = pool;

    LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
    slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
    LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
    slResourcePoolIdNr.id = 0;
    slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
    slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

    LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
    slBwpPoolConfigCommonNr.slTxPoolSelectedNormal[slResourcePoolIdNr.id] = slresoPoolConfigNr;

    LteRrcSap::Bwp bwp;
    bwp.numerology = numerologyBwpSl;
    bwp.symbolsPerSlots = 14;
    bwp.rbPerRbg = 1;
    bwp.bandwidth = bandwidthBandSl;

    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp = bwp;
    slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
    slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum(0);

    LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
    slBwpConfigCommonNr.haveSlBwpGeneric = true;
    slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
    slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
    slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

    LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
    for (const auto& it : bwpIdContainer)
    {
        slFreConfigCommonNr.slBwpList[it] = slBwpConfigCommonNr;
    }

    LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
    tddUlDlConfigCommon.tddPattern = tddPattern;

    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
    slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

    LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
    slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = static_cast<uint8_t>(slMaxTxTransNumPssch);
    LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
    pscchTxConfigList.slPsschTxParameters[0] = psschParams;
    slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

    LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
    slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
    slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
    slPreConfigNr.slPreconfigFreqInfoList[0] = slFreConfigCommonNr;
    nrSlHelper->InstallNrSlPreConfiguration(ueNetDev, slPreConfigNr);

    int64_t stream = 1;
    stream += nrHelper->AssignStreams(ueNetDev, stream);
    stream += nrSlHelper->AssignStreams(ueNetDev, stream);

    // ---- IP + sidelink bearer (all UEs both TX and RX) -----------------------
    InternetStackHelper internet;
    internet.Install(ueNodes);
    stream += internet.AssignStreams(ueNodes, stream);

    uint32_t dstL2Id = 255;
    Ipv4Address groupAddress4("225.0.0.0");
    uint16_t port = 8000;
    SidelinkInfo slInfo;
    slInfo.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id = dstL2Id;
    slInfo.m_rri = MilliSeconds(reservationPeriod);
    slInfo.m_dynamic = false;
    slInfo.m_pdb = Seconds(0);
    slInfo.m_harqEnabled = harqEnabled;

    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueNetDev);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> r =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        r->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }
    Address remoteAddress = InetSocketAddress(groupAddress4, port);
    Address localAddress = InetSocketAddress(Ipv4Address::GetAny(), port);

    Ptr<LteSlTft> tft =
        Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress4, slInfo);
    nrSlHelper->ActivateNrSlBearer(Seconds(slBearersActivationTime), ueNetDev, tft);

    // ---- applications: every CAV is a 10 Hz CAM source + sink ----------------
    // Per-UE app start/stop follows the SUMO depart/arrive (vehicle lifetime),
    // mirroring OMNeT where the app exists only while the vehicle is in the net.
    Ptr<UniformRandomVariable> jitter = CreateObject<UniformRandomVariable>();
    jitter->SetStream(stream++);
    jitter->SetAttribute("Min", DoubleValue(0.0));
    jitter->SetAttribute("Max", DoubleValue(0.10)); // first-beacon de-sync (OMNeT uniform(0,0.1s))

    OnOffHelper client("ns3::UdpSocketFactory", remoteAddress);
    client.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    std::string drStr = std::to_string(dataRateBe) + "kb/s";
    client.SetConstantRate(DataRate(drStr), udpPacketSizeBe);

    PacketSinkHelper sink("ns3::UdpSocketFactory", localAddress);
    sink.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));

    double simStop = slBearersActivationTime + simTime;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        // window the app to [max(activation, depart), min(arrive, simStop)]
        double start = std::max(slBearersActivationTime, depart[i]) + jitter->GetValue();
        double stop = std::min(arrive[i], simStop);
        if (stop <= start)
        {
            continue; // CAV not present during the simulated window
        }
        ApplicationContainer c = client.Install(ueNodes.Get(i));
        c.Get(0)->SetStartTime(Seconds(start));
        c.Get(0)->SetStopTime(Seconds(stop));
        clientApps.Add(c);

        ApplicationContainer srv = sink.Install(ueNodes.Get(i));
        srv.Get(0)->SetStartTime(Seconds(std::max(slBearersActivationTime, depart[i])));
        srv.Get(0)->SetStopTime(Seconds(stop));
        serverApps.Add(srv);
    }
    std::cout << "Active client apps=" << clientApps.GetN() << "/" << numUes << std::endl;

    // ---- optional asymmetric FLOOD node: concentrate load on ONE UE -----------
    // Separate flow + bearer (distinct dstL2Id/group) but the SAME resource pool, so the
    // flood contends for the same time-frequency resources -> raises occupancy/collision for
    // all UEs. Dynamic (aperiodic) grant per 3GPP TS 38.321 (SPS fallback if !dynamic).
    if (floodNodeRate > 0.0)
    {
        uint32_t fid = (floodNodeId >= 0) ? static_cast<uint32_t>(floodNodeId) : 0;
        if (floodNodeId < 0)
        {
            double best = -1.0;
            for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
            {
                double life = std::min(arrive[i], simStop) - std::max(slBearersActivationTime, depart[i]);
                if (life > best) { best = life; fid = i; }
            }
        }
        SidelinkInfo fInfo;
        fInfo.m_castType = SidelinkInfo::CastType::Groupcast;
        fInfo.m_dstL2Id = 254;
        fInfo.m_rri = floodNodeDynamic ? MilliSeconds(0)
                                       : MilliSeconds(static_cast<uint16_t>(1000.0 / floodNodeRate));
        fInfo.m_dynamic = floodNodeDynamic;
        fInfo.m_pdb = Seconds(0);
        fInfo.m_harqEnabled = harqEnabled;
        Ipv4Address floodGroup("225.0.0.1");
        Ptr<LteSlTft> ftft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, floodGroup, fInfo);
        NetDeviceContainer floodDev;
        floodDev.Add(ueNetDev.Get(fid));
        nrSlHelper->ActivateNrSlBearer(Seconds(slBearersActivationTime), floodDev, ftft);

        double frKbps = floodNodeRate * udpPacketSizeBe * 8.0 / 1000.0;
        OnOffHelper flood("ns3::UdpSocketFactory", InetSocketAddress(floodGroup, port));
        flood.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
        flood.SetConstantRate(DataRate(std::to_string(frKbps) + "kb/s"), udpPacketSizeBe);
        double fstart = std::max(slBearersActivationTime, depart[fid]);
        double fstop = std::min(arrive[fid], simStop);
        ApplicationContainer fc = flood.Install(ueNodes.Get(fid));
        fc.Get(0)->SetStartTime(Seconds(fstart));
        fc.Get(0)->SetStopTime(Seconds(fstop));
        clientApps.Add(fc);
        std::cout << "FLOODNODE id=" << fid << " rate=" << floodNodeRate << "pps dynamic="
                  << floodNodeDynamic << " window=[" << fstart << "," << fstop << "]s" << std::endl;
    }

    // ---- traces + DB ---------------------------------------------------------
    std::string exampleName = simTag + "-propagation-compare";
    SQLiteOutput db(outputDir + exampleName + ".db");

    // SetDb always runs (creates empty tables so v2x-kpi's ComputePssch* don't
    // hit a missing table). The trace *connections* are gated by --phyTraces,
    // because the RX PHY tables explode to multi-GB at high adoption.
    UeMacPscchTxOutputStats pscchStats;
    pscchStats.SetDb(&db, "pscchTxUeMac");
    UeMacPsschTxOutputStats psschStats;
    psschStats.SetDb(&db, "psschTxUeMac");
    UePhyPscchRxOutputStats pscchPhyStats;
    pscchPhyStats.SetDb(&db, "pscchRxUePhy");
    UePhyPsschRxOutputStats psschPhyStats;
    psschPhyStats.SetDb(&db, "psschRxUePhy");
    UeRlcRxOutputStats ueRlcRxStats;
    ueRlcRxStats.SetDb(&db, "rlcRx");
    if (macSummary)
    {
        // TX-side scheduling only — cheap (one row per TX), feeds simulPsschTx (collisions)
        // and HARQ fields. Safe to enable on every adoption rate.
        Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                      "ComponentCarrierMapUe/*/NrUeMac/SlPscchScheduling",
                                      MakeBoundCallback(&NotifySlPscchScheduling, &pscchStats));
        Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                      "ComponentCarrierMapUe/*/NrUeMac/SlPsschScheduling",
                                      MakeBoundCallback(&NotifySlPsschScheduling, &psschStats));
    }
    if (phyRxTrace)
    {
        // RX-side per-reception — HUGE at high adoption (one row per neighbour per TX).
        // Feeds PsschTbRx (decode), per-reception SINR, and rlcRx latency. Low rate only.
        Config::ConnectWithoutContext(
            "/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
            "SpectrumPhy/RxPscchTraceUe",
            MakeBoundCallback(&NotifySlPscchRx, &pscchPhyStats));
        Config::ConnectWithoutContext(
            "/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
            "SpectrumPhy/RxPsschTraceUe",
            MakeBoundCallback(&NotifySlPsschRx, &psschPhyStats));
        Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                      "ComponentCarrierMapUe/*/NrUeMac/RxRlcPduWithTxRnti",
                                      MakeBoundCallback(&NotifySlRlcPduRx, &ueRlcRxStats));
    }

    UeToUePktTxRxOutputStats pktStats;
    pktStats.SetDb(&db, "pktTxRx");
    for (uint32_t ac = 0; ac < clientApps.GetN(); ac++)
    {
        Ptr<Node> n = clientApps.Get(ac)->GetNode();
        Ipv4Address la = n->GetObject<Ipv4L3Protocol>()->GetAddress(1, 0).GetLocal();
        clientApps.Get(ac)->TraceConnect(
            "TxWithSeqTsSize",
            "tx",
            MakeBoundCallback(&UePacketTraceDb, &pktStats, n, la));
    }
    for (uint32_t ac = 0; ac < serverApps.GetN(); ac++)
    {
        Ptr<Node> n = serverApps.Get(ac)->GetNode();
        Ipv4Address la = n->GetObject<Ipv4L3Protocol>()->GetAddress(1, 0).GetLocal();
        serverApps.Get(ac)->TraceConnect(
            "RxWithSeqTsSize",
            "rx",
            MakeBoundCallback(&UePacketTraceDb, &pktStats, n, la));
    }

    V2xKpi v2xKpi;
    v2xKpi.SetDbPath(outputDir + exampleName);
    v2xKpi.SetTxAppDuration(simTime);
    SavePositionPerIP(&v2xKpi);
    v2xKpi.SetRangeForV2xKpis(kpiRange);

    Simulator::Stop(Seconds(simStop));
    Simulator::Run();

    pktStats.EmptyCache();
    pscchStats.EmptyCache();
    psschStats.EmptyCache();
    pscchPhyStats.EmptyCache();
    psschPhyStats.EmptyCache();
    ueRlcRxStats.EmptyCache();
    v2xKpi.WriteKpis();

    double avgLat = g_latencyCnt ? (g_latencySum / g_latencyCnt) : 0.0;
    std::cout << "==== propagation-compare summary ====" << std::endl;
    std::cout << "tag=" << tag << " scenario=" << channelScenario << " numUes=" << numUes
              << " activeTx=" << clientApps.GetN() << std::endl;
    std::cout << "txPkt=" << g_txPkt << " rxPkt=" << g_rxPkt
              << " avgLatency_ms=" << avgLat * 1000.0 << std::endl;
    // machine-readable line for the run script to grep:
    std::cout << "RESULTCSV," << tag << "," << channelScenario << "," << numUes << ","
              << clientApps.GetN() << "," << g_txPkt << "," << g_rxPkt << ","
              << std::fixed << std::setprecision(4) << avgLat * 1000.0 << std::endl;

    Simulator::Destroy();
    return 0;
}
