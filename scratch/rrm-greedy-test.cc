/*
 * Copyright (c) 2005,2006,2007 INRIA
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#include "ns3/core-module.h"
#include "ns3/athstats-helper.h"
#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-socket-address.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-mac.h"
#include "ns3/ap-wifi-mac.h"
#include "ns3/internet-module.h"
#include "ns3/udp-echo-helper.h"

#include "ns3/wifi-net-device.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/net-device.h"
#include "ns3/rrm.h"
#include <cassert>
// #include <random>
#include <iomanip>
#include "ns3/netanim-module.h"
#include "ns3/simulator-impl.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
// #include "ns3/visualizer.h"
// #include "ns3/visual-simulator-impl.h"


using namespace ns3;
using std::cout, std::endl, std::setw, std::setprecision,
      std::vector, std::string, std::map, std::set, std::shared_ptr;
NS_LOG_COMPONENT_DEFINE("rrm-greedy-test");


#define LOG_LOGIC(x) do { NS_LOG_LOGIC(std::setprecision(7) << Simulator::Now().GetSeconds() << "s: " << x ); } while(0)
#define LOG_DEBUG(x) do { NS_LOG_DEBUG(std::setprecision(7) << Simulator::Now().GetSeconds() << "s: " << x ); } while(0)

/// True for verbose output.
static bool g_verbose = false;
static bool g_debug = false;
static bool g_logic = false;

vector<std::shared_ptr<Scanner>>
doScanning(vector<uint16_t>& apChannelAllocation,
           vector<uint16_t>& apStaAllocation,
           vector<uint16_t> channelsToScan={1, 6, 11}) {
    // Packet::EnablePrinting();
    RngSeedManager::SetSeed(2);
    // LogComponentEnable("WifiPhy", LOG_LEVEL_DEBUG);
    if (g_debug) {
        LogComponentEnable("rrm-greedy-test", LOG_LEVEL_DEBUG);
        LogComponentEnable("StaWifiMac", LOG_LEVEL_DEBUG);
    } else if (g_logic) {
        LogComponentEnable("StaWifiMac", LogLevel(LOG_LEVEL_LOGIC & (~LOG_FUNCTION)));
    }
    // our modules have logic level logging by default
    LogComponentEnable("rrm-greedy-test", LOG_LEVEL_LOGIC);
    LogComponentEnable("rrm", LOG_LEVEL_LOGIC);
    // LogComponentEnable("WifiDefaultAssocManager", LOG_LEVEL_LO);
    //
    const int n_aps = apChannelAllocation.size();

    WifiHelper wifi;
    MobilityHelper mobility;
    vector<NodeContainer> staNodes(n_aps);
    NodeContainer apNodes;
    // NodeContainer scanAp;

    vector<NetDeviceContainer> staDevs(n_aps);
    NetDeviceContainer apDevs;
    PacketSocketHelper packetSocket;

    const uint16_t initialWidth = 20;
    const WifiPhyBand initialBand = WIFI_PHY_BAND_2_4GHZ;

    // give packet socket powers to nodes.
    for (int i = 0; i < n_aps; i++) {
        staNodes[i].Create(apStaAllocation[i]);
        packetSocket.Install(staNodes[i]);
    }
    apNodes.Create(n_aps);
    packetSocket.Install(apNodes);

    // setup mobility for APs
    vector<Vector> apPosVectors = {
        {0.0, 0.0, 0.0},
        {4.0, 0.0, 0.0},
        {0.0, 7.0, 0.0},
        {5.0, 5.0, 0.0},
    };
    Ptr<ListPositionAllocator> listPos = CreateObject<ListPositionAllocator>();
    for (auto& v : apPosVectors) {
        listPos->Add(v);
    }
    mobility.SetPositionAllocator(listPos);
    // mobility.SetPositionAllocator(
    //     "ns3::GridPositionAllocator",
    //     "MinX", DoubleValue(0.0),
    //     "MinY", DoubleValue(0.0),
    //     "DeltaX", DoubleValue(5.0),
    //     "DeltaY", DoubleValue(5.0),
    //     "GridWidth", UintegerValue(2),
    //     "LayoutType", StringValue("RowFirst")
    // );
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(apNodes);

    // setup mobility for STAs: distribute them randomly around each AP
    for (int i = 0; i < n_aps; i++) {
        std::string x = std::to_string(apPosVectors[i].x);
        std::string y = std::to_string(apPosVectors[i].y);
        std::string z = std::to_string(apPosVectors[i].z);
        // std::string rho = "7.0";
        mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                      "X", StringValue(x),
                                      "Y", StringValue(y),
                                      "Z", StringValue(z),
                                      "Rho", StringValue("ns3::UniformRandomVariable[Min=1|Max=1.5]") //
        );
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(staNodes[i]);

    }

    // setup mobility for STAs

    // setup wifi
    wifi.SetStandard(WIFI_STANDARD_80211n);
    // uint32_t rtsThreshold = 65535;
    // std::string staManager = "ns3::MinstrelHtWifiManager";
    // std::string apManager = "ns3::MinstrelHtWifiManager";
    // wifi.SetRemoteStationManager(apManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));

    WifiMacHelper wifiMac;
    // setup wifi channel helper
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    // below code is not needed: Default() applies these settings already.
    // double exponent = 3.0;
    // wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    // wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
    //                            "Exponent",
    //                            DoubleValue(PathLossExponent));
    wifiPhy.SetChannel(wifiChannel.Create());

    InternetStackHelper stack;
    Ipv4AddressHelper address;
    address.SetBase("1.1.1.0", "255.255.255.0");
    vector<Ipv4InterfaceContainer> staInterfaces(n_aps);

    double initialApTxPower_dbm = 20.0;
    // setup APs
    auto setupAp = [&](Ptr<Node> apNode_i, NetDeviceContainer& apDev_i, string ssid) {
        wifiMac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(Ssid(ssid)),
                "QosSupported", BooleanValue(false)
        );
        wifiPhy.Set("TxPowerStart", DoubleValue(initialApTxPower_dbm));
        wifiPhy.Set("TxPowerEnd", DoubleValue(initialApTxPower_dbm));
        wifiPhy.Set("TxPowerLevels", UintegerValue(1));
        apDev_i = wifi.Install(wifiPhy, wifiMac, apNode_i);
    };

    // setup animation
    AnimationInterface anim("rrmgreedy.xml");
    auto setupApAnim = [&anim](Ptr<Node> apNode, int i) {
        anim.UpdateNodeColor(apNode, (50 + (20*i)) % 256, 0, 0); // red
        // anim.UpdateNodeSize(apNode->GetId(), 1.0, 1.0);
        anim.UpdateNodeDescription(apNode, "AP-" + std::to_string(i));
    };


    for (int i = 0; i < n_aps; i++) {
        setupAp(apNodes.Get(i), apDevs, "ssid-" + std::to_string(i));
        setupApAnim(apNodes.Get(i), i);
    }

    do {
        // mobility.Install(apNodes);
        stack.Install(apNodes);
        Ipv4InterfaceContainer apInterfaces = address.Assign(apDevs);
    } while(false);


    constexpr double simulationStartTime = 0.0;
    constexpr double simulationEndTime = 10.0;
    constexpr double udpStartTime = simulationStartTime + 0.5;
    constexpr double udpEndTime = simulationEndTime;
    auto setupUdpEchoClientServer = [](Ptr<Node> nodeServer,
                                        Ptr<Node> nodeClient,
                                        Ipv4InterfaceContainer& staInterfaces,
                                        uint16_t echoPort=9) {
        UdpEchoServerHelper echoServer(echoPort);
        ApplicationContainer serverApps = echoServer.Install(nodeServer);

        UdpEchoClientHelper echoClient(staInterfaces.GetAddress(0), echoPort);
        double maxPackets = 0; // 0 means unlimited
        double packetInterval = 0.0005; // 0.005;
        int packetSize = 1024;
        echoClient.SetAttribute("MaxPackets", UintegerValue(maxPackets));
        echoClient.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
        echoClient.SetAttribute("PacketSize", UintegerValue(packetSize));
        NS_LOG_LOGIC("UDP packets rate: " << ((packetSize*8 / packetInterval) / 1000 / 1000) << " Mbps");
        ApplicationContainer clientApps = echoClient.Install(nodeClient);

        serverApps.Start(Seconds(udpStartTime));
        clientApps.Start(Seconds(udpStartTime));
        serverApps.Stop(Seconds(udpEndTime));
        clientApps.Stop(Seconds(udpEndTime));
        return std::make_pair(serverApps, clientApps);
    };


    auto setupSta = [&](NodeContainer& sta_i_nodes, NetDeviceContainer& sta_i_devs, string ssid) {
        int initialChannel = 1;
        TupleValue<UintegerValue, UintegerValue, EnumValue<WifiPhyBand>, UintegerValue> channelValue;
        channelValue.Set(WifiPhy::ChannelTuple {initialChannel, initialWidth, initialBand, 0});
        wifiPhy.Set("ChannelSettings", channelValue);
        wifiMac.SetType("ns3::StaWifiMac",
                        "ActiveProbing", BooleanValue(true),
                        "Ssid", SsidValue(Ssid(ssid)),
                        "QosSupported", BooleanValue(false)
        );
        sta_i_devs = wifi.Install(wifiPhy, wifiMac, sta_i_nodes);
    };
    // setup stas.

    auto setupSTAAnimation = [&anim, &staInterfaces](NodeContainer staNodes_ap_i, int i) {
        for (size_t k = 0; k < staNodes_ap_i.GetN(); k++) {
            anim.UpdateNodeColor(staNodes_ap_i.Get(k), 0, (150 + 20*(i)) %  256, 0); // green
            // anim.UpdateNodeSize(staNodes_ap_i.Get(k)->GetId(), 0.4, 0.4);
            auto [ipv4, _] = staInterfaces[i].Get(k);
            std::stringstream staname;
            staname << "STA " << i << "-" << k << "\n" << "(" << ipv4->GetAddress(1, 0).GetLocal() << ")";
            anim.UpdateNodeDescription(staNodes_ap_i.Get(k), staname.str());

        }
    };

    for (int i = 0; i < n_aps; i++) {
        setupSta(staNodes[i], staDevs[i], "ssid-" + std::to_string(i));
        // mobility.Install(staNodes[i]);
        stack.Install(staNodes[i]);
        staInterfaces[i] = address.Assign(staDevs[i]);
        setupUdpEchoClientServer(staNodes[i].Get(0), staNodes[i].Get(1), staInterfaces[i]);
        setupSTAAnimation(staNodes[i], i);
    }

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();


    /* Populate routing table */
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();


    // constexpr uint16_t echoPort = 9;
    // UdpEchoServerHelper echoServer(echoPort);
    // ApplicationContainer serverApps = echoServer.Install(staNodes.Get(0));
    //
    // UdpEchoClientHelper echoClient(staInterfaces.GetAddress(0), echoPort);
    // double maxPackets = 0; // 0 means unlimited
    // double packetInterval = 0.005;
    // int packetSize = 1024;
    // echoClient.SetAttribute("MaxPackets", UintegerValue(maxPackets));
    // echoClient.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
    // echoClient.SetAttribute("PacketSize", UintegerValue(1024));
    // NS_LOG_LOGIC("Packets rate: " << ((packetSize / packetInterval) / 1024) * 8 << " kbps");
    // ApplicationContainer clientApps = echoClient.Install(staNodes.Get(1));

    // simulation
    Simulator::Stop(Seconds(udpEndTime));
    for (auto it = apNodes.Begin(); it != apNodes.End(); ++it) {
        int i = it - apNodes.Begin();
        SIM_LOG_LOGIC("AP " << i
             << ",channel: " << +getWifiNd(*it)->GetPhy()->GetOperatingChannel().GetNumber()
             << ",ssid: " << getWifiNd(*it)->GetMac()->GetSsid().PeekString()
             << ",bssid: " << (*it)->GetDevice(0)->GetAddress()
        );
    }

    for (const auto& staNodes_ap_i : staNodes) {
        for (auto it = staNodes_ap_i.Begin(); it != staNodes_ap_i.End(); ++it) {
            int i = it - staNodes_ap_i.Begin();
            SIM_LOG_LOGIC("STA " << i
                 << ",ssid: " << getWifiNd(*it)->GetMac()->GetSsid().PeekString()
                 << ",channel: " << +getWifiNd(*it)->GetPhy()->GetOperatingChannel().GetNumber()
                 << ",mac: " << (*it)->GetDevice(0)->GetAddress()
            );
        }
    }

    vector<std::shared_ptr<Scanner>> scanners;

    // Initialize a random number generator with a given seed
    // constexpr int SEED = 1;
    // constexpr double min = 0.1;
    // constexpr double max = 5.0;
    // std::mt19937 rng(SEED);
    // // Generate a random number
    // std::uniform_real_distribution<double> uni(min, max); // inclusive min, exclusive max
    // auto random_double = uni(rng);
    //
    std::shared_ptr<RRMGreedyAlgo> rrmgreedy = std::make_shared<RRMGreedyAlgo>(channelsToScan);

    for (size_t i = 0; i < apNodes.GetN(); i++) {
        auto apNode = apNodes.Get(i);
        std::shared_ptr<Scanner> scanner = CreateScannerForNode(apNode, channelsToScan, "AP-" + std::to_string(i));
        scanner->setAfterScanCallback<void, RRMGreedyAlgo*, Scanner*>(
                std::function<void(RRMGreedyAlgo*, Scanner*)>(
                    RRMGreedyAlgo::AddApScandata_s
                ),
                &(*rrmgreedy),
                &(*scanner)
        );
        const double apScanStart_s = 2.5 + (0.01*i);
        Simulator::Schedule(Seconds(apScanStart_s), &Scanner::Scan, &(*scanner));
        scanners.push_back(scanner);
    }
    rrmgreedy->AddDevices(scanners);


    Simulator::Schedule(Seconds(7.0), [&rrmgreedy](){rrmgreedy->Decide();});

    anim.EnablePacketMetadata(true);
    anim.EnableIpv4L3ProtocolCounters(Seconds(0), Seconds(10)); // Optional
    anim.EnableWifiMacCounters(Seconds(0), Seconds(10)); // Optional
    anim.EnableWifiPhyCounters(Seconds(0), Seconds(10)); // Optional
    anim.EnableIpv4RouteTracking("routingtable-rrmgreedy.xml",
                                 Seconds(0),
                                 Seconds(10),
                                 Seconds(0.25));         // Optional
    Simulator::Run();

    // print metrics
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
    for (auto i = stats.begin(); i != stats.end(); ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);

        std::cout << "Flow " << i->first << " (" << t.sourceAddress << " -> "
                  << t.destinationAddress << ")\n";
        std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
        std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
        std::cout << "  TxOffered:  "
                  << i->second.txBytes * 8.0 / (udpEndTime - udpStartTime) / 1000 / 1000
                  << " Mbps\n";
        std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
        std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        std::cout << "  Throughput: "
                  << i->second.rxBytes * 8.0 / (udpEndTime - udpStartTime) / 1000 / 1000
                  << " Mbps\n";
    }
    return scanners;
}

int main(int argc, char* argv[]) {
    CommandLine cmd(__FILE__);
    // { PyViz v; }
    cmd.AddValue("verbose", "Print trace information if true", g_verbose);
    cmd.AddValue("debug", "Print debug information if true", g_debug);
    cmd.AddValue("logic", "Enable info debug level", g_logic);
    cmd.Parse(argc, argv);

    const int NUM_APS = 4;
    vector<shared_ptr<Scanner>> scanners(NUM_APS);

    vector<uint16_t> apChannelAllocation = {
        1,
        1,
        1,
        1
    };

    assert(apChannelAllocation.size() == NUM_APS &&
            std::string(
                "expected NUM_APS=" + std::to_string(NUM_APS) + ", got " + std::to_string(apChannelAllocation.size()) + " channel allocations"
                ).c_str());

    vector<uint16_t> apStaAllocation = {
        3,
        2,
        2,
        2
    };

    assert(apStaAllocation.size() == NUM_APS &&
            std::string(
                "expected NUM_APS=" + std::to_string(NUM_APS) + ", got " + std::to_string(apStaAllocation.size()) + " channel allocations"
                ).c_str());

    scanners = doScanning(apChannelAllocation, apStaAllocation);
    Simulator::Destroy();
    return 0;
}

