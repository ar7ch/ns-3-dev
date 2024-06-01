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

#define XML_NEWLINE ""

void printRssiRecords() {
    cout << "====================================== RSSI records ==============================================" << endl;
    std::cout << std::left << std::setw(20) << "mac"
              << std::setw(10) << "avgRSSI"
              << std::setw(10) << "avgNoise"
              << std::setw(10) << "avgSNR" << endl;
    for (auto [mac, rxPhy] : getRssiRecords()) {
        std::stringstream macStr; macStr << mac;
        std::cout << std::left << std::setw(20) << macStr.str()
                  << std::setw(10) << rxPhy.rssi / rxPhy.n
                  << std::setw(10) << rxPhy.noise / rxPhy.n
                  << std::setw(10) << rxPhy.snr / rxPhy.n
                  << endl;
    }
    cout << "==================================================================================================" << endl;
}

// double
// printThroughputResults(Ptr<FlowMonitor> monitor, FlowMonitorHelper& flowmon,
//         double udpStartTime, double udpEndTime,
//         map<Ipv4Address, Mac48Address>& ip2mac) {
//     double totalRxBytes = 0.0;
//     Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
//     FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
//     std::set<Ipv4Address> visitedIps;
//     cout << "====================================== Throughput results ==============================================" << endl;
//     std::cout << std::left << std::setw(10) << "MAC1"
//               << std::setw(10) << "MAC2"
//               << std::setw(10) << "TxPkt"
//               << std::setw(15) << "TxBytes"
//               << std::setw(20) << "TxOffered (Mbps)"
//               << std::setw(10) << "RxPkt"
//               << std::setw(15) << "RxBytes"
//               << std::setw(15) << "Thrpt (Mbps)"
//               << std::endl;
//     for (auto [flowId, flowStats] : stats)
//     {
//         Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
//         Ipv4Address srcAddr = t.sourceAddress;
//         Ipv4Address dstAddr = t.destinationAddress;
//         totalRxBytes += flowStats.rxBytes;
//
//         if (visitedIps.count(srcAddr)) {
//             continue;
//         }
//         visitedIps.insert(dstAddr);
//         // Mac48Address mac1 = ip2mac.at(t.sourceAddress);
//         // Mac48Address mac2 = ip2mac.at(t.destinationAddress);
//         auto printStats =  [&](Mac48Address& mac1, Mac48Address& mac2) {
//             double txOffered = (flowStats.rxBytes * 8 / 1000.0 / 1000.0) / (udpEndTime - udpStartTime);
//             double throughput = (flowStats.rxBytes * 8 / 1000.0 / 1000.0) / (udpEndTime - udpStartTime);
//
//             std::stringstream mac1Str; mac1Str << mac1; string mac1Str_s = mac1Str.str().substr(14);
//             std::stringstream mac2Str; mac2Str << mac2; string mac2Str_s = mac2Str.str().substr(14);
//             std::cout << std::left << std::setw(10) << mac1Str_s
//                 << std::setw(10) << mac2Str_s
//                 << std::setw(10) << flowStats.txPackets
//                 << std::setw(15) << flowStats.txBytes
//                 << std::setw(20) << std::fixed << std::setprecision(4) << txOffered
//                 << std::setw(10) << flowStats.rxPackets
//                 << std::setw(15) << flowStats.rxBytes
//                 << std::setw(15) << std::fixed << std::setprecision(4) << throughput
//                 << std::endl;
//         };
//         printStats(ip2mac.at(srcAddr), ip2mac.at(dstAddr));
//         printStats(ip2mac.at(dstAddr), ip2mac.at(srcAddr));
//     }
//     cout << "==================================================================================================" << endl;
//     return (totalRxBytes * 8 / 1000.0 / 1000.0) / (udpEndTime - udpStartTime);
// }

// void printSimulationParams(NodeContainer& apNodes, vector<NodeContainer>& staNodes) {
//     cout << "============== Simulation parameters ==============================" << endl;
//     cout << "=================== APs ===========================================" << endl;
//     std::cout << std::left
//         << std::setw(10) << "AP"
//         << std::setw(10) << "Channel"
//         << std::setw(25) << "SSID"
//         << std::setw(20) << "BSSID"
//         << std::endl;
//
//     // Print data
//     for (auto it = apNodes.Begin(); it != apNodes.End(); ++it) {
//         int i = it - apNodes.Begin();
//         std::stringstream bssidStr; bssidStr << getWifiNd(*it)->GetMac()->GetAddress(); string bssidStr_s = bssidStr.str();
//         std::cout << std::left
//             << std::setw(10) << i
//             << std::setw(10) << +getWifiNd(*it)->GetPhy()->GetOperatingChannel().GetNumber()
//             << std::setw(25) << getWifiNd(*it)->GetMac()->GetSsid().PeekString()
//             << std::setw(20) << bssidStr_s
//             << std::endl;
//     }
//     cout << "=================== STAs ===========================================" << endl;
//     // Print headers
//     std::cout << std::left
//               << std::setw(10) << "STA"
//               << std::setw(25) << "SSID"
//               << std::setw(10) << "Channel"
//               << std::setw(20) << "MAC"
//               << std::endl;
//
//     // Print data
//     for (const auto& staNodes_ap_i : staNodes) {
//         for (auto it = staNodes_ap_i.Begin(); it != staNodes_ap_i.End(); ++it) {
//             int i = (it - staNodes_ap_i.Begin()) + staNodes_ap_i.GetN();
//             std::stringstream bssidStr; bssidStr << getWifiNd(*it)->GetMac()->GetAddress(); string bssidStr_s = bssidStr.str();
//             std::cout << std::left
//                       << std::setw(10) << i
//                       << std::setw(25) << getWifiNd(*it)->GetMac()->GetSsid().PeekString()
//                       << std::setw(10) << +getWifiNd(*it)->GetPhy()->GetOperatingChannel().GetNumber()
//                       << std::setw(20) << bssidStr_s
//                       << std::endl;
//         }
//     }
//     cout << "====================================================================" << endl;
// }

// std::shared_ptr<RRMGreedyAlgo>
// setupApScanners(NodeContainer& apNodes,
//         vector<std::shared_ptr<Scanner>>& scanners,
//         const vector<uint16_t>& channelsToScan) {
//     std::shared_ptr<RRMGreedyAlgo> rrmgreedy = std::make_shared<RRMGreedyAlgo>(channelsToScan);
//     for (size_t i = 0; i < apNodes.GetN(); i++) { auto apNode = apNodes.Get(i);
//         std::shared_ptr<Scanner> scanner = CreateScannerForNode(apNode, channelsToScan, "AP-" + std::to_string(i));
//         scanner->setAfterScanCallback<void, RRMGreedyAlgo*, Scanner*>(
//                 std::function<void(RRMGreedyAlgo*, Scanner*)>(
//                     RRMGreedyAlgo::AddApScandata_s
//                 ),
//                 &(*rrmgreedy),
//                 &(*scanner)
//         );
//         const double apScanStart_s = 2.5 + (0.01*i);
//         Simulator::Schedule(Seconds(apScanStart_s), &Scanner::Scan, &(*scanner));
//         scanners.push_back(scanner);
//     }
//     rrmgreedy->AddDevices(scanners);
//     return rrmgreedy;
// }





std::string
getWifiMacStr(Ptr<Node> node) {
    auto wifiNd = getWifiNd(node);
    auto wifiMac = wifiNd->GetMac();
    std::stringstream macStr; macStr << wifiMac->GetAddress();
    return macStr.str();
}

std::string
getWifiMacStr(Mac48Address mac) {
    std::stringstream macStr; macStr << mac;
    return macStr.str();
}

// void setupStaTrafficFlow(NodeContainer& staNodes, Ipv4InterfaceContainer& staInterfaces,
//         const double trafficStartTime, const double trafficEndTime) {
//     auto setupUdpEchoClientServer = [](Ptr<Node> nodeServer,
//                                         Ptr<Node> nodeClient,
//                                         Ipv4InterfaceContainer& staInterfaces,
//                                         const double trafficStartTime,
//                                         const double trafficEndTime,
//                                         const uint16_t echoPort=9,
//                                         double maxPackets = 0, // 0 means unlimited
//                                         double packetInterval = 0.005,
//                                         int packetSize = 1024) {
//         UdpEchoServerHelper echoServer(echoPort);
//         ApplicationContainer serverApps = echoServer.Install(nodeServer);
//
//         UdpEchoClientHelper echoClient(staInterfaces.GetAddress(0), echoPort);
//         echoClient.SetAttribute("MaxPackets", UintegerValue(maxPackets));
//         echoClient.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
//         echoClient.SetAttribute("PacketSize", UintegerValue(packetSize));
//         NS_LOG_LOGIC("UDP packets rate: " << ((packetSize*8 / packetInterval) / 1000 / 1000) << " Mbps");
//         ApplicationContainer clientApps = echoClient.Install(nodeClient);
//
//         serverApps.Start(Seconds(trafficStartTime));
//         clientApps.Start(Seconds(trafficStartTime));
//         serverApps.Stop(Seconds(trafficEndTime));
//         clientApps.Stop(Seconds(trafficEndTime));
//         return std::make_pair(serverApps, clientApps);
//     };
//     if (staNodes.GetN() < 2) {
//         return;
//     }
//     for (size_t i = 0; i < staNodes.GetN()-1; i++) {
//         Ptr<Node> nodeServer = staNodes.Get(i);
//         Ptr<Node> nodeClient = staNodes.Get(i+1);
//         setupUdpEchoClientServer(staNodes.Get(i), staNodes.Get(i+1), staInterfaces, trafficStartTime, trafficEndTime);
//         cout << setw(20) << getWifiMacStr(nodeServer) << setw(20) << getWifiMacStr(nodeClient) << setw(20) << endl;
//     }
// }
//
// void
// setupStas(vector<NodeContainer>& staNodes, vector<NetDeviceContainer>& staDevs, vector<Ipv4InterfaceContainer>& staInterfaces,
//         vector<uint16_t>& apChannelAllocation,
//         double trafficStartTime, double trafficEndTime,
//         map<Ipv4Address, Mac48Address>& ip2mac, map<Mac48Address, Ipv4Address>& mac2ip,
//         WifiHelper& wifi, WifiPhyHelper& wifiPhy, WifiMacHelper& wifiMac, MobilityHelper& mobility,
//             InternetStackHelper& stack, AnimationInterface& anim, Ipv4AddressHelper& addressHelper,
//         const WifiPhyBand initialBand = WIFI_PHY_BAND_2_4GHZ, const uint16_t initialWidth = 20
//         ) {
//     auto setupSTAAnimation = [&anim, &staInterfaces, &ip2mac, &mac2ip](NodeContainer staNodes_ap_i, int i) {
//         for (size_t k = 0; k < staNodes_ap_i.GetN(); k++) {
//             Ptr<Node> sta_i_k = staNodes_ap_i.Get(k);
//             anim.UpdateNodeColor(sta_i_k, 0, (100 + 50*(i)) %  256, 0); // green
//             // anim.UpdateNodeSize(staNodes_ap_i.Get(k)->GetId(), 0.4, 0.4);
//             auto [ipv4, _] = staInterfaces[i].Get(k);
//             std::stringstream staname;
//             Ipv4Address staIp = ipv4->GetAddress(1, 0).GetLocal();
//             Mac48Address staMac = getWifiNd(sta_i_k)->GetMac()->GetAddress();
//             ip2mac[staIp] = staMac;
//             mac2ip[staMac] = staIp;
//             std::stringstream staMacStr_ss;
//             staMacStr_ss << staMac;
//             string staMacStr = staMacStr_ss.str().substr(14);
//             staname << "STA " << i << "-" << k << XML_NEWLINE
//                 << "(" << staIp << ")" << staMacStr << XML_NEWLINE;
//
//             anim.UpdateNodeDescription(sta_i_k, staname.str());
//
//         }
//     };
//     cout << setw(20) << "SRV" << setw(20) << "CLI" << setw(20) << endl;;
//     for (int i = 0; i < staNodes.size(); i++) {
//         packetSocket.Install(staNodes[i]);
//         std::string ssid = "ssid-" + std::to_string(i);
//         switchChannel_attr(wifiPhy, apChannelAllocation[i]);
//         wifiMac.SetType("ns3::StaWifiMac",
//                         "ActiveProbing", BooleanValue(true),
//                         "Ssid", SsidValue(Ssid(ssid)),
//                         "QosSupported", BooleanValue(false)
//         );
//         staDevs[i] = wifi.Install(wifiPhy, wifiMac, staNodes[i]);
//         stack.Install(staNodes[i]);
//         staInterfaces[i] = addressHelper.Assign(staDevs[i]);
//         setupStaTrafficFlow(staNodes[i], staInterfaces[i], trafficStartTime, trafficEndTime);
//         setupSTAAnimation(staNodes[i], i);
//         for (size_t k = 0; k < staNodes[i].GetN(); k++) {
//             CreateScannerForStaNode(staNodes[i].Get(k));
//         }
//     }
// }


// std::pair<
//     vector<std::shared_ptr<Scanner>>,
//     std::shared_ptr<RRMGreedyAlgo>
// >
// doInitialSim(vector<uint16_t>& apChannelAllocation,
//            vector<uint16_t>& apStaAllocation,
//            vector<uint16_t> channelsToScan={1, 6, 11},
//            uint16_t initialWidth = 20,
//            WifiPhyBand initialBand = WIFI_PHY_BAND_2_4GHZ) {
//     // Packet::EnablePrinting();
//     RngSeedManager::SetSeed(2);
//     const int n_aps = apChannelAllocation.size();
//     WifiHelper wifi;
//     MobilityHelper mobility;
//     vector<NodeContainer> staNodes(n_aps);
//     NodeContainer apNodes;
//
//     vector<NetDeviceContainer> staDevs(n_aps);
//     NetDeviceContainer apDevs;
//     PacketSocketHelper packetSocket;
//
//     // give packet socket powers to nodes.
//     for (int i = 0; i < n_aps; i++) {
//         staNodes[i].Create(apStaAllocation[i]);
//         packetSocket.Install(staNodes[i]);
//     }
//     apNodes.Create(n_aps);
//     packetSocket.Install(apNodes);
//
//     Vector startPos{3.0, 3.0, 0.0};
//     // setup mobility for APs
//     vector<Vector> apPosRelativeVectors = {
//         {0.0, 0.0, 0.0},
//         {4.0, 0.0, 0.0},
//         {0.0, 7.0, 0.0},
//         {5.0, 5.0, 0.0},
//     };
//
//     for (auto& v : apPosRelativeVectors) {
//         v = v+startPos;
//     }
//
//     Ptr<ListPositionAllocator> listPos = CreateObject<ListPositionAllocator>();
//     for (auto& v : apPosRelativeVectors) {
//         listPos->Add(v);
//     }
//     mobility.SetPositionAllocator(listPos);
//     // mobility.SetPositionAllocator(
//     //     "ns3::GridPositionAllocator",
//     //     "MinX", DoubleValue(0.0),
//     //     "MinY", DoubleValue(0.0),
//     //     "DeltaX", DoubleValue(5.0),
//     //     "DeltaY", DoubleValue(5.0),
//     //     "GridWidth", UintegerValue(2),
//     //     "LayoutType", StringValue("RowFirst")
//     // );
//     mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
//     mobility.Install(apNodes);
//
//     // setup mobility for STAs: distribute them randomly around each AP
//     for (int i = 0; i < n_aps; i++) {
//         std::string x = std::to_string(apPosRelativeVectors[i].x);
//         std::string y = std::to_string(apPosRelativeVectors[i].y);
//         std::string z = std::to_string(apPosRelativeVectors[i].z);
//         // std::string rho = "7.0";
//         mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
//                                       "X", StringValue(x),
//                                       "Y", StringValue(y),
//                                       "Z", StringValue(z),
//                                       "Rho", StringValue("ns3::UniformRandomVariable[Min=1|Max=1.5]") //
//         );
//         mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
//         mobility.Install(staNodes[i]);
//     }
//
//     // setup wifi
//     wifi.SetStandard(WIFI_STANDARD_80211n);
//     uint32_t rtsThreshold = 65535;
//     // // std::string staManager = "ns3::MinstrelHtWifiManager";
//     // std::string apManager = "ns3::MinstrelHtWifiManager";
//     // wifi.SetRemoteStationManager(apManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));
//     // std::ostringstream ossControlMode;
//     // ossControlMode << "OfdmRate" << "18" << "Mbps";
//     //
//     // std::ostringstream ossDataMode;
//     // ossDataMode << "OfdmRate" << "54" << "Mbps";
//     wifi.SetRemoteStationManager("ns3::IdealWifiManager",
//             "RtsCtsThreshold", UintegerValue(rtsThreshold)
//             // ,
//             // "DataMode",
//             // StringValue(ossDataMode.str()),
//             // "ControlMode",
//             // StringValue(ossControlMode.str())
//     );
//
//     // Set guard interval
//     wifi.ConfigHtOptions("ShortGuardIntervalSupported", BooleanValue(true));
//
//
//     WifiMacHelper wifiMac;
//     // setup wifi channel helper
//     YansWifiPhyHelper wifiPhy;
//     YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
//     wifiPhy.SetChannel(wifiChannel.Create());
//
//     InternetStackHelper stack;
//     Ipv4AddressHelper address;
//     address.SetBase("1.1.1.0", "255.255.255.0");
//     vector<Ipv4InterfaceContainer> staInterfaces(n_aps);
//
//     double initialApTxPower_dbm = 20.0;
//     // setup APs
//
//     // setup animation
//     AnimationInterface anim("rrmgreedy-before.xml");
//     auto setupApAnim = [&anim](Ptr<Node> apNode_i, int i) {
//         anim.UpdateNodeColor(apNode_i, (50 + (20*i)) % 256, 0, 0); // red
//         // anim.UpdateNodeSize(apNode->GetId(), 1.0, 1.0);
//         std::stringstream apName;
//         Mac48Address bssid = getWifiNd(apNode_i)->GetMac()->GetAddress();
//         std::stringstream bssidStr;
//         bssidStr << bssid;
//         string bssidStr_s = bssidStr.str().substr(14);
//         apName << "AP-" << i << " " << bssidStr_s << XML_NEWLINE
//             << "CH: " << +getWifiNd(apNode_i)->GetPhy()->GetOperatingChannel().GetNumber()
//             << " txp: " << getWifiNd(apNode_i)->GetPhy()->GetTxPowerStart();
//         anim.UpdateNodeDescription(apNode_i, apName.str());
//     };
//
//
//     for (int i = 0; i < n_aps; i++) {
//         setupAp(apNodes.Get(i), apDevs, "ssid-" + std::to_string(i));
//         setupApAnim(apNodes.Get(i), i);
//     }
//
//     do {
//         // mobility.Install(apNodes);
//         stack.Install(apNodes);
//         Ipv4InterfaceContainer apInterfaces = address.Assign(apDevs);
//     } while(false);
//
//     map<Ipv4Address, Mac48Address> ip2mac;
//     map<Mac48Address, Ipv4Address> mac2ip;
//
//     constexpr double simulationStartTime = 0.0;
//     constexpr double simulationEndTime = 10.0;
//     constexpr double udpStartTime = simulationStartTime + 0.5;
//     constexpr double udpEndTime = simulationEndTime;
//     auto setupUdpEchoClientServer = [](Ptr<Node> nodeServer,
//                                         Ptr<Node> nodeClient,
//                                         Ipv4InterfaceContainer& staInterfaces,
//                                         uint16_t echoPort=9) {
//         UdpEchoServerHelper echoServer(echoPort);
//         ApplicationContainer serverApps = echoServer.Install(nodeServer);
//
//         UdpEchoClientHelper echoClient(staInterfaces.GetAddress(0), echoPort);
//         double maxPackets = 0; // 0 means unlimited
//         double packetInterval = 0.005; // 0.005;
//         int packetSize = 1024;
//         echoClient.SetAttribute("MaxPackets", UintegerValue(maxPackets));
//         echoClient.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
//         echoClient.SetAttribute("PacketSize", UintegerValue(packetSize));
//         NS_LOG_LOGIC("UDP packets rate: " << ((packetSize*8 / packetInterval) / 1000 / 1000) << " Mbps");
//         ApplicationContainer clientApps = echoClient.Install(nodeClient);
//
//         serverApps.Start(Seconds(udpStartTime));
//         clientApps.Start(Seconds(udpStartTime));
//         serverApps.Stop(Seconds(udpEndTime));
//         clientApps.Stop(Seconds(udpEndTime));
//         return std::make_pair(serverApps, clientApps);
//     };
//
//
//     auto setupSta = [&](NodeContainer& sta_i_nodes, NetDeviceContainer& sta_i_devs, string ssid) {
//         int initialChannel = 1;
//         TupleValue<UintegerValue, UintegerValue, EnumValue<WifiPhyBand>, UintegerValue> channelValue;
//         channelValue.Set(WifiPhy::ChannelTuple {initialChannel, initialWidth, initialBand, 0});
//         wifiPhy.Set("ChannelSettings", channelValue);
//         wifiMac.SetType("ns3::StaWifiMac",
//                         "ActiveProbing", BooleanValue(true),
//                         "Ssid", SsidValue(Ssid(ssid)),
//                         "QosSupported", BooleanValue(false)
//         );
//         sta_i_devs = wifi.Install(wifiPhy, wifiMac, sta_i_nodes);
//     };
//     // setup stas.
//
//     auto setupSTAAnimation = [&anim, &staInterfaces, &ip2mac, &mac2ip](NodeContainer staNodes_ap_i, int i) {
//         for (size_t k = 0; k < staNodes_ap_i.GetN(); k++) {
//             Ptr<Node> sta_i_k = staNodes_ap_i.Get(k);
//             anim.UpdateNodeColor(sta_i_k, 0, (100 + 50*(i)) %  256, 0); // green
//             // anim.UpdateNodeSize(staNodes_ap_i.Get(k)->GetId(), 0.4, 0.4);
//             auto [ipv4, _] = staInterfaces[i].Get(k);
//             std::stringstream staname;
//             Ipv4Address staIp = ipv4->GetAddress(1, 0).GetLocal();
//             Mac48Address staMac = getWifiNd(sta_i_k)->GetMac()->GetAddress();
//             ip2mac[staIp] = staMac;
//             mac2ip[staMac] = staIp;
//             std::stringstream staMacStr_ss;
//             staMacStr_ss << staMac;
//             string staMacStr = staMacStr_ss.str().substr(14);
//             staname << "STA " << i << "-" << k << XML_NEWLINE
//                 << "(" << staIp << ")" << staMacStr << XML_NEWLINE;
//
//             anim.UpdateNodeDescription(sta_i_k, staname.str());
//
//         }
//     };
//
//     eraseRssiRecords();
//     for (int i = 0; i < n_aps; i++) {
//
//         setupSta(staNodes[i], staDevs[i], "ssid-" + std::to_string(i));
//         // mobility.Install(staNodes[i]);
//         stack.Install(staNodes[i]);
//         staInterfaces[i] = address.Assign(staDevs[i]);
//         setupUdpEchoClientServer(staNodes[i].Get(0), staNodes[i].Get(1), staInterfaces[i]);
//         setupSTAAnimation(staNodes[i], i);
//         for (size_t k = 0; k < staNodes[i].GetN(); k++) {
//             CreateScannerForStaNode(staNodes[i].Get(k));
//         }
//     }
//
//     FlowMonitorHelper flowmon;
//     Ptr<FlowMonitor> monitor = flowmon.InstallAll();
//
//
//     /* Populate routing table */
//     Ipv4GlobalRoutingHelper::PopulateRoutingTables();
//
//     // simulation
//     Simulator::Stop(Seconds(udpEndTime));
//
//     printSimulationParams(apNodes, staNodes);
//     vector<std::shared_ptr<Scanner>> scanners;
//     std::shared_ptr<RRMGreedyAlgo> rrmgreedy = std::make_shared<RRMGreedyAlgo>(channelsToScan);
//
//     for (size_t i = 0; i < apNodes.GetN(); i++) {
//         auto apNode = apNodes.Get(i);
//         std::shared_ptr<Scanner> scanner = CreateScannerForNode(apNode, channelsToScan, "AP-" + std::to_string(i));
//         // std::shared_ptr<Scanner> scanner;
//         scanner->setAfterScanCallback<void, RRMGreedyAlgo*, Scanner*>(
//                 std::function<void(RRMGreedyAlgo*, Scanner*)>(
//                     RRMGreedyAlgo::AddApScandata_s
//                 ),
//                 &(*rrmgreedy),
//                 &(*scanner)
//         );
//         const double apScanStart_s = 2.5 + (0.01*i);
//         Simulator::Schedule(Seconds(apScanStart_s), &Scanner::Scan, &(*scanner));
//         scanners.push_back(scanner);
//     }
//     rrmgreedy->AddDevices(scanners);
//
//
//     Simulator::Schedule(Seconds(7.0), [&rrmgreedy](){rrmgreedy->Decide();});
//
//     anim.EnablePacketMetadata(true);
//     anim.EnableIpv4L3ProtocolCounters(Seconds(0), Seconds(10)); // Optional
//     anim.EnableWifiMacCounters(Seconds(0), Seconds(10)); // Optional
//     anim.EnableWifiPhyCounters(Seconds(0), Seconds(10)); // Optional
//     anim.EnableIpv4RouteTracking("routingtable-rrmgreedy.xml",
//                                  Seconds(0),
//                                  Seconds(10),
//                                  Seconds(0.25));         // Optional
//     Simulator::Run();
//
//     // print metrics
//     printRssiRecords();
//     monitor->CheckForLostPackets();
//     double totalThroughput = printThroughputResults(monitor, flowmon, udpStartTime, udpEndTime, ip2mac);
//     std::cout << "Total group throughput before RRM: " << totalThroughput << "Mbps" << std::endl;
//     return std::make_pair(scanners, rrmgreedy);
// }
//
// vector<std::shared_ptr<Scanner>>
// doRrmImprovedSim(
//            vector<uint16_t>& apChannelAllocation,
//            vector<uint16_t>& apTxpAllocationDbm,
//            vector<uint16_t>& apStaAllocation,
//            RrmResults& rrmResults,
//            vector<uint16_t> channelsToScan={1, 6, 11}) {
//     const int n_aps = apChannelAllocation.size();
//
//     WifiHelper wifi;
//     MobilityHelper mobility;
//     vector<NodeContainer> staNodes(n_aps);
//     NodeContainer apNodes;
//     // NodeContainer scanAp;
//
//     vector<NetDeviceContainer> staDevs(n_aps);
//     NetDeviceContainer apDevs;
//     PacketSocketHelper packetSocket;
//
//     const uint16_t initialWidth = 20;
//     const WifiPhyBand initialBand = WIFI_PHY_BAND_2_4GHZ;
//
//     // give packet socket powers to nodes.
//     for (int i = 0; i < n_aps; i++) {
//         staNodes[i].Create(apStaAllocation[i]);
//         packetSocket.Install(staNodes[i]);
//     }
//     apNodes.Create(n_aps);
//     packetSocket.Install(apNodes);
//
//     Vector startPos{3.0, 3.0, 0.0};
//     // setup mobility for APs
//     vector<Vector> apPosRelativeVectors = {
//         {0.0, 0.0, 0.0},
//         {4.0, 0.0, 0.0},
//         {0.0, 7.0, 0.0},
//         {5.0, 5.0, 0.0},
//     };
//
//     for (auto& v : apPosRelativeVectors) {
//         v = v+startPos;
//     }
//
//     Ptr<ListPositionAllocator> listPos = CreateObject<ListPositionAllocator>();
//     for (auto& v : apPosRelativeVectors) {
//         listPos->Add(v);
//     }
//     mobility.SetPositionAllocator(listPos);
//     // mobility.SetPositionAllocator(
//     //     "ns3::GridPositionAllocator",
//     //     "MinX", DoubleValue(0.0),
//     //     "MinY", DoubleValue(0.0),
//     //     "DeltaX", DoubleValue(5.0),
//     //     "DeltaY", DoubleValue(5.0),
//     //     "GridWidth", UintegerValue(2),
//     //     "LayoutType", StringValue("RowFirst")
//     // );
//     mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
//     mobility.Install(apNodes);
//
//     // setup mobility for STAs: distribute them randomly around each AP
//     for (int i = 0; i < n_aps; i++) {
//         std::string x = std::to_string(apPosRelativeVectors[i].x);
//         std::string y = std::to_string(apPosRelativeVectors[i].y);
//         std::string z = std::to_string(apPosRelativeVectors[i].z);
//         // std::string rho = "7.0";
//         mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
//                                       "X", StringValue(x),
//                                       "Y", StringValue(y),
//                                       "Z", StringValue(z),
//                                       "Rho", StringValue("ns3::UniformRandomVariable[Min=1|Max=1.5]") //
//         );
//         mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
//         // mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
//         //                           "Bounds",
//         //                           RectangleValue(Rectangle(x+2, 50, -25, 50)));
//         mobility.Install(staNodes[i]);
//
//     }
//
//     // setup mobility for STAs
//
//     // setup wifi
//     wifi.SetStandard(WIFI_STANDARD_80211n);
//     uint32_t rtsThreshold = 65535;
//     // std::string staManager = "ns3::MinstrelHtWifiManager";
//     // std::string apManager = "ns3::MinstrelHtWifiManager";
//     // wifi.SetRemoteStationManager(apManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));
//     wifi.SetRemoteStationManager("ns3::IdealWifiManager",
//             "RtsCtsThreshold", UintegerValue(rtsThreshold)
//             // ,
//             // "DataMode",
//             // StringValue(ossDataMode.str()),
//             // "ControlMode",
//             // StringValue(ossControlMode.str())
//             // "RtsCtsThreshold", UintegerValue(rtsThreshold)
//     );
//     // std::ostringstream ossControlMode; ossControlMode << "OfdmRate" << "18" << "Mbps";
//     //
//     // std::ostringstream ossDataMode; ossDataMode << "OfdmRate" << "54" << "Mbps";
//     // wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
//     //         "DataMode",
//     //         StringValue(ossDataMode.str()),
//     //         "ControlMode",
//     //         StringValue(ossControlMode.str())
//     // );
//
//     // Set guard interval
//     wifi.ConfigHtOptions("ShortGuardIntervalSupported", BooleanValue(true));
//
//     WifiMacHelper wifiMac;
//     // setup wifi channel helper
//     YansWifiPhyHelper wifiPhy;
//     YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
//     wifiPhy.SetChannel(wifiChannel.Create());
//
//     InternetStackHelper stack;
//     Ipv4AddressHelper address;
//     address.SetBase("1.1.1.0", "255.255.255.0");
//     vector<Ipv4InterfaceContainer> staInterfaces(n_aps);
//
//     double initialApTxPower_dbm = 20.0;
//     // setup APs
//     auto setupAp = [&](Ptr<Node> apNode_i, NetDeviceContainer& apDev_i, string ssid) {
//         wifiMac.SetType("ns3::ApWifiMac",
//                 "Ssid", SsidValue(Ssid(ssid)),
//                 "QosSupported", BooleanValue(false)
//         );
//         wifiPhy.Set("TxPowerStart", DoubleValue(initialApTxPower_dbm));
//         wifiPhy.Set("TxPowerEnd", DoubleValue(initialApTxPower_dbm));
//         wifiPhy.Set("TxPowerLevels", UintegerValue(1));
//         apDev_i = wifi.Install(wifiPhy, wifiMac, apNode_i);
//         Mac48Address bssid = getWifiNd(apNode_i)->GetMac()->GetAddress();
//         auto [chan_i, txp_i] = rrmResults.at(bssid);
//         NS_LOG_DEBUG("AP " << bssid << " switched to RrmResult channel " << chan_i << " and txp " << txp_i);
//         Ptr<WifiNetDevice> wifiNd_i = getWifiNd(apNode_i);
//         switchChannel_attr(wifiNd_i, chan_i, initialBand, initialWidth);
//         setTxPower_attr(wifiNd_i, txp_i);
//
//     };
//
//     // setup animation
//     AnimationInterface anim("rrmgreedy-after.xml");
//     auto setupApAnim = [&anim](Ptr<Node> apNode_i, int i) {
//         anim.UpdateNodeColor(apNode_i, (50 + (20*i)) % 256, 0, 0); // red
//         // anim.UpdateNodeSize(apNode->GetId(), 1.0, 1.0);
//         std::stringstream apName;
//         Mac48Address bssid = getWifiNd(apNode_i)->GetMac()->GetAddress();
//         std::stringstream bssidStr;
//         bssidStr << bssid;
//         string bssidStr_s = bssidStr.str().substr(14);
//         apName << "AP-" << i << " " << bssidStr_s << XML_NEWLINE
//             << " CH=" << +getWifiNd(apNode_i)->GetPhy()->GetOperatingChannel().GetNumber()
//             << " txp=" << getWifiNd(apNode_i)->GetPhy()->GetTxPowerStart();
//         anim.UpdateNodeDescription(apNode_i, apName.str());
//     };
//
//
//     for (int i = 0; i < n_aps; i++) {
//         setupAp(apNodes.Get(i), apDevs, "ssid-" + std::to_string(i));
//         setupApAnim(apNodes.Get(i), i);
//     }
//
//     do {
//         // mobility.Install(apNodes);
//         stack.Install(apNodes);
//         Ipv4InterfaceContainer apInterfaces = address.Assign(apDevs);
//     } while(false);
//
//     map<Ipv4Address, Mac48Address> ip2mac;
//     map<Mac48Address, Ipv4Address> mac2ip;
//
//     constexpr double simulationStartTime = 0.0;
//     constexpr double simulationEndTime = 10.0;
//     constexpr double udpStartTime = simulationStartTime + 0.5;
//     constexpr double udpEndTime = simulationEndTime;
//     auto setupUdpEchoClientServer = [](Ptr<Node> nodeServer,
//                                         Ptr<Node> nodeClient,
//                                         Ipv4InterfaceContainer& staInterfaces,
//                                         uint16_t echoPort=9) {
//         UdpEchoServerHelper echoServer(echoPort);
//         ApplicationContainer serverApps = echoServer.Install(nodeServer);
//
//         UdpEchoClientHelper echoClient(staInterfaces.GetAddress(0), echoPort);
//         double maxPackets = 0; // 0 means unlimited
//         double packetInterval = 0.005; // 0.005;
//         int packetSize = 1024;
//         echoClient.SetAttribute("MaxPackets", UintegerValue(maxPackets));
//         echoClient.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
//         echoClient.SetAttribute("PacketSize", UintegerValue(packetSize));
//         NS_LOG_LOGIC("UDP packets rate: " << ((packetSize*8 / packetInterval) / 1000 / 1000) << " Mbps");
//         ApplicationContainer clientApps = echoClient.Install(nodeClient);
//
//         serverApps.Start(Seconds(udpStartTime));
//         clientApps.Start(Seconds(udpStartTime));
//         serverApps.Stop(Seconds(udpEndTime));
//         clientApps.Stop(Seconds(udpEndTime));
//         return std::make_pair(serverApps, clientApps);
//     };
//
//
//     auto setupSta = [&](NodeContainer& sta_i_nodes, NetDeviceContainer& sta_i_devs, string ssid) {
//         int initialChannel = 1;
//         TupleValue<UintegerValue, UintegerValue, EnumValue<WifiPhyBand>, UintegerValue> channelValue;
//         channelValue.Set(WifiPhy::ChannelTuple {initialChannel, initialWidth, initialBand, 0});
//         wifiPhy.Set("ChannelSettings", channelValue);
//         wifiMac.SetType("ns3::StaWifiMac",
//                         "ActiveProbing", BooleanValue(true),
//                         "Ssid", SsidValue(Ssid(ssid)),
//                         "QosSupported", BooleanValue(false)
//         );
//         sta_i_devs = wifi.Install(wifiPhy, wifiMac, sta_i_nodes);
//     };
//     // setup stas.
//
//     auto setupSTAAnimation = [&anim, &staInterfaces, &ip2mac, &mac2ip](NodeContainer staNodes_ap_i, int i) {
//         for (size_t k = 0; k < staNodes_ap_i.GetN(); k++) {
//             Ptr<Node> sta_i_k = staNodes_ap_i.Get(k);
//             anim.UpdateNodeColor(sta_i_k, 0, (100 + 50*(i)) %  256, 0); // green
//             // anim.UpdateNodeSize(staNodes_ap_i.Get(k)->GetId(), 0.4, 0.4);
//             auto [ipv4, _] = staInterfaces[i].Get(k);
//             std::stringstream staname;
//             Ipv4Address staIp = ipv4->GetAddress(1, 0).GetLocal();
//             Mac48Address staMac = getWifiNd(sta_i_k)->GetMac()->GetAddress();
//             ip2mac[staIp] = staMac;
//             mac2ip[staMac] = staIp;
//             std::stringstream staMacStr_ss;
//             staMacStr_ss << staMac;
//             string staMacStr = staMacStr_ss.str().substr(14);
//             staname << "STA " << i << "-" << k << XML_NEWLINE
//                 << "(" << staIp << ")" << staMacStr;
//
//             anim.UpdateNodeDescription(sta_i_k, staname.str());
//
//         }
//     };
//
//     eraseRssiRecords();
//     for (int i = 0; i < n_aps; i++) {
//         setupSta(staNodes[i], staDevs[i], "ssid-" + std::to_string(i));
//         // mobility.Install(staNodes[i]);
//         stack.Install(staNodes[i]);
//         staInterfaces[i] = address.Assign(staDevs[i]);
//         setupUdpEchoClientServer(staNodes[i].Get(0), staNodes[i].Get(1), staInterfaces[i]);
//         setupSTAAnimation(staNodes[i], i);
//         for (size_t k = 0; k < staNodes[i].GetN(); k++) {
//             CreateScannerForStaNode(staNodes[i].Get(k));
//         }
//     }
//
//     FlowMonitorHelper flowmon;
//     Ptr<FlowMonitor> monitor = flowmon.InstallAll();
//
//     /* Populate routing table */
//     Ipv4GlobalRoutingHelper::PopulateRoutingTables();
//
//     // simulation
//     Simulator::Stop(Seconds(udpEndTime));
//     printSimulationParams(apNodes, staNodes);
//
//     vector<std::shared_ptr<Scanner>> scanners;
//
//     std::shared_ptr<RRMGreedyAlgo> rrmgreedy = std::make_shared<RRMGreedyAlgo>(channelsToScan);
//
//     for (size_t i = 0; i < apNodes.GetN(); i++) {
//         auto apNode = apNodes.Get(i);
//         std::shared_ptr<Scanner> scanner = CreateScannerForNode(apNode, channelsToScan, "AP-" + std::to_string(i));
//         // std::shared_ptr<Scanner> scanner;
//         scanner->setAfterScanCallback<void, RRMGreedyAlgo*, Scanner*>(
//                 std::function<void(RRMGreedyAlgo*, Scanner*)>(
//                     RRMGreedyAlgo::AddApScandata_s
//                 ),
//                 &(*rrmgreedy),
//                 &(*scanner)
//         );
//         const double apScanStart_s = 2.5 + (0.01*i);
//         Simulator::Schedule(Seconds(apScanStart_s), &Scanner::Scan, &(*scanner));
//         scanners.push_back(scanner);
//     }
//     rrmgreedy->AddDevices(scanners);
//
//     // Simulator::Schedule(Seconds(7.0), [&rrmgreedy](){rrmgreedy->Decide();});
//
//     anim.EnablePacketMetadata(true);
//     anim.EnableIpv4L3ProtocolCounters(Seconds(0), Seconds(10)); // Optional
//     anim.EnableWifiMacCounters(Seconds(0), Seconds(10)); // Optional
//     anim.EnableWifiPhyCounters(Seconds(0), Seconds(10)); // Optional
//     anim.EnableIpv4RouteTracking("routingtable-rrmgreedy.xml",
//                                  Seconds(0),
//                                  Seconds(10),
//                                  Seconds(0.25));         // Optional
//     Simulator::Run();
//
//     monitor->CheckForLostPackets();
//     // print metrics
//     printRssiRecords();
//     double totalThroughput = printThroughputResults(monitor, flowmon, udpStartTime, udpEndTime, ip2mac);
//     std::cout << "Total group throughput after RRM: " << totalThroughput << "Mbps" << std::endl;
//     return scanners;
// }

class SimulationCase {
public:
    // nodes
    NodeContainer apNodes;
    vector<NodeContainer> staNodes;
    // network devices
    NetDeviceContainer apDevs;
    vector<NetDeviceContainer> staDevs;
    // ipv4 interfaces
    vector<Ipv4InterfaceContainer> staInterfaces;
    // RRM
    vector<std::shared_ptr<Scanner>> scanners;
    std::shared_ptr<RRMGreedyAlgo> rrmgreedy;
    vector<uint16_t> apChannelAllocation;
    vector<double> apTxpAllocationDbm;
    vector<uint16_t> apStaAllocation;
    vector<uint16_t> channelsToScan;
    // simulator settings
    double simulationStartTime;
    double simulationEndTime;
    double trafficStartTime;
    double trafficEndTime;
    // useful data structures
    map<Ipv4Address, Mac48Address> ip2mac;
    map<Mac48Address, Ipv4Address> mac2ip;
    // animation
    std::unique_ptr<AnimationInterface> anim;
    // metrics capture
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor;
    // mobility
    MobilityHelper mobility;
    Vector startPos{3.0, 3.0, 0.0};
    vector<Vector> apPosRelativeVectors = {
        {0.0, 0.0, 0.0},
        {4.0, 0.0, 0.0},
        {0.0, 7.0, 0.0},
        {5.0, 5.0, 0.0},
    };
    // network helpers
    PacketSocketHelper packetSocket;
    WifiHelper wifi;
    YansWifiPhyHelper wifiPhy;
    WifiMacHelper wifiMac;
    InternetStackHelper stack;
    Ipv4AddressHelper addressHelper;

    const size_t n_aps;

    SimulationCase(
            const vector<uint16_t>& apChannelAllocation,
            const vector<uint16_t>& apStaAllocation,
            const vector<double>& apTxpAllocationDbm,
            const vector<uint16_t>& channelsToScan,
            const double simulationStartTime,
            const double simulationEndTime,
            const double trafficStartTime,
            const double trafficEndTime,
            const std::string animFileName
    ) : apChannelAllocation(apChannelAllocation),
        apTxpAllocationDbm(apTxpAllocationDbm),
        apStaAllocation(apStaAllocation),
        channelsToScan(channelsToScan),
        simulationStartTime(simulationStartTime),
        simulationEndTime(simulationEndTime),
        trafficStartTime(trafficStartTime),
        trafficEndTime(trafficEndTime),
        n_aps(apChannelAllocation.size())
    {
            wifi.SetStandard(WIFI_STANDARD_80211n);
            uint32_t rtsThreshold = 65535;
            std::string manager = "ns3::MinstrelHtWifiManager";
            wifi.SetRemoteStationManager(manager, "RtsCtsThreshold", UintegerValue(rtsThreshold));
            YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
            wifiPhy.SetChannel(wifiChannel.Create());
            addressHelper.SetBase("1.1.1.0", "255.255.255.0");

            setupAps();
            setupStas();
            eraseRssiRecords();
            Ipv4GlobalRoutingHelper::PopulateRoutingTables();
            anim = std::make_unique<AnimationInterface>(animFileName);
            setupAnim();
            monitor = flowmon.InstallAll();
            Simulator::Stop(Seconds(trafficEndTime));
        }

    struct SimulationCaseResults {
    public:
        double totalThroughput;
        struct RrmResults {
            vector<uint16_t> apChannelAllocation;
            vector<double> apTxpAllocationDbm;

            RrmResults(
                    const vector<uint16_t>& apChannelAllocation,
                    const vector<double>& apTxpAllocationDbm
            ) : apChannelAllocation(apChannelAllocation),
                apTxpAllocationDbm(apTxpAllocationDbm)
            {}
        };
        const RrmResults newRrmAlloc;
        SimulationCaseResults(double totalThroughput, const RrmResults& rrmResults) :
            totalThroughput(totalThroughput),
            newRrmAlloc(rrmResults)
        {}
    };

    SimulationCaseResults
    runSimulation(bool doRrm) {
        printSimulationParams();
        if (doRrm) {
            Simulator::Schedule(Seconds(7.0), [&](){rrmgreedy->Decide();});
        }
        vector<uint16_t> apChannelAllocation(n_aps, 0);
        vector<double> apTxpAllocationDbm(n_aps, 0);
        Simulator::Run();
        if (doRrm) {
            ApsRrmAssignments rrmResults = rrmgreedy->GetRrmResults();
            for (size_t i = 0; i < n_aps; i++) {
                auto apNode = apNodes.Get(i);
                Mac48Address bssid = getWifiNd(apNode)->GetMac()->GetAddress();
                auto [chan_i, txp_i] = rrmResults.at(bssid);
                apChannelAllocation[i] = chan_i;
                apTxpAllocationDbm[i] = txp_i * 1.0;
                // NS_LOG_DEBUG("AP " << bssid << " switched to RrmResult channel " << chan_i << " and txp " << txp_i);
            }
        }
        double totalThroughput = printMetrics();
        return SimulationCaseResults(totalThroughput, {apChannelAllocation, apTxpAllocationDbm});
    }

    double
    printMetrics() {
        printRssiRecords();
        monitor->CheckForLostPackets();
        double totalThroughput = printThroughputResults();
        std::cout << "Total group throughput: " << totalThroughput << "Mbps" << std::endl;
        return totalThroughput;
    }

    void setupAnim() {
        setupApsAnim();
        setupStasAnim();
        anim->EnablePacketMetadata(true);
        anim->EnableIpv4L3ProtocolCounters(Seconds(trafficStartTime), Seconds(trafficEndTime)); // Optional
        anim->EnableWifiMacCounters(Seconds(simulationStartTime), Seconds(simulationEndTime)); // Optional
        anim->EnableWifiPhyCounters(Seconds(simulationStartTime), Seconds(simulationEndTime)); // Optional
    }

    void setupApsMobility() {

        for (auto& v : apPosRelativeVectors) {
            v = v+startPos;
        }

        // Ptr<ListPositionAllocator> listPos = CreateObject<ListPositionAllocator>();
        // for (auto& v : apPosRelativeVectors) {
        //     listPos->Add(v);
        //}
        mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                      "MinX",
                                      DoubleValue(5.0),
                                      "MinY",
                                      DoubleValue(5.0),
                                      "DeltaX",
                                      DoubleValue(5.0),
                                      "DeltaY",
                                      DoubleValue(5.0),
                                      "GridWidth",
                                      UintegerValue(2),
                                      "LayoutType",
                                      StringValue("RowFirst"));
        // mobility.SetPositionAllocator(listPos);
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(apNodes);
    }

    void
    setupAps() {
        NS_LOG_DEBUG("Start APs setup");
        apNodes.Create(n_aps);
        packetSocket.Install(apNodes);
        setupApsMobility();
        assert(apNodes.GetN() == n_aps && n_aps == apChannelAllocation.size() && n_aps == apTxpAllocationDbm.size());
        for (size_t i = 0; i < apNodes.GetN(); i++) {
            Ptr<Node> apNode_i = apNodes.Get(i);
            std::string ssid = "ssid-" + std::to_string(i);
            wifiMac.SetType("ns3::ApWifiMac",
                    "Ssid", SsidValue(Ssid(ssid)),
                    "QosSupported", BooleanValue(false)
            );
            apDevs.Add(wifi.Install(wifiPhy, wifiMac, apNode_i));
            NS_LOG_DEBUG("ap dev mac: " << apNode_i->GetDevice(0)->GetAddress());
        }
        stack.Install(apNodes);
        addressHelper.Assign(apDevs);
        setupApScanners();
        NS_LOG_DEBUG("APs setup done");
    }

    void setupApAnim (Ptr<Node> apNode_i, int i) const {
        anim->UpdateNodeColor(apNode_i, (50 + (20*i)) % 256, 0, 0); // red
        Ptr<WifiNetDevice> wifiNd_i = getWifiNd(apNode_i);
        string bssidStr_s = getWifiMacStr(apNode_i);
        auto wifiPhy_i = wifiNd_i->GetPhy();
        std::stringstream apName;
        apName <<
            "AP-" << i << " " << bssidStr_s <<
            "CH: "   << +wifiPhy_i->GetOperatingChannel().GetNumber() <<
            " txp: " << wifiPhy_i->GetTxPowerStart();
        anim->UpdateNodeDescription(apNode_i, apName.str());
    }

    void setupApsAnim() const {
        for (size_t i = 0; i < apNodes.GetN(); i++) {
            setupApAnim(apNodes.Get(i), i);
        }
    }

    void setupApScanners() {
        rrmgreedy = std::make_shared<RRMGreedyAlgo>(channelsToScan);
        for (size_t i = 0; i < apNodes.GetN(); i++) { auto apNode = apNodes.Get(i);
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
    }

    void setupUdpEchoClientServer (Ptr<Node> nodeServer, Ptr<Node> nodeClient, Ipv4InterfaceContainer& staInterfaces_i,
            const uint16_t echoPort=9,
            double maxPackets = 0, // 0 means unlimited
            double packetInterval = 0.05,
            int packetSize = 1024) const {
        UdpEchoServerHelper echoServer(echoPort);
        ApplicationContainer serverApps = echoServer.Install(nodeServer);
        UdpEchoClientHelper echoClient(staInterfaces_i.GetAddress(0), echoPort);
        echoClient.SetAttribute("MaxPackets", UintegerValue(maxPackets));
        echoClient.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
        echoClient.SetAttribute("PacketSize", UintegerValue(packetSize));
        NS_LOG_LOGIC("UDP packets rate: " << ((packetSize*8 / packetInterval) / 1000 / 1000) << " Mbps");
        ApplicationContainer clientApps = echoClient.Install(nodeClient);

        serverApps.Start(Seconds(trafficStartTime));
        clientApps.Start(Seconds(trafficStartTime));
        serverApps.Stop(Seconds(trafficEndTime));
        clientApps.Stop(Seconds(trafficEndTime));
    }

    void setupStaTrafficFlow(NodeContainer& staNodes_i, Ipv4InterfaceContainer& staInterfaces_i) const {
        if (staNodes_i.GetN() < 2) {
            return;
        }
        for (size_t i = 0; i < staNodes_i.GetN()-1; i++) {
            Ptr<Node> nodeServer = staNodes_i.Get(i);
            Ptr<Node> nodeClient = staNodes_i.Get(i+1);
            setupUdpEchoClientServer(nodeServer, nodeClient, staInterfaces_i);
            cout << setw(20) << getWifiMacStr(nodeServer) << setw(20) << getWifiMacStr(nodeClient) << setw(20) << endl;
        }
    }

    void setupStaAnim(NodeContainer staNodes_ap_i, int i) {
        for (size_t k = 0; k < staNodes_ap_i.GetN(); k++) {
            Ptr<Node> sta_i_k = staNodes_ap_i.Get(k);
            anim->UpdateNodeColor(sta_i_k, 0, (100 + 50*(i)) %  256, 0); // green
                                                                        // anim.UpdateNodeSize(staNodes_ap_i.Get(k)->GetId(), 0.4, 0.4);
            auto [ipv4, _] = staInterfaces[i].Get(k);
            std::stringstream staname;
            Ipv4Address staIp = ipv4->GetAddress(1, 0).GetLocal();
            Mac48Address staMac = getWifiNd(sta_i_k)->GetMac()->GetAddress();
            ip2mac[staIp] = staMac;
            mac2ip[staMac] = staIp;
            std::stringstream staMacStr_ss;
            staMacStr_ss << staMac;
            string staMacStr = staMacStr_ss.str().substr(14);
            staname << "STA " << i << "-" << k << XML_NEWLINE
                << "(" << staIp << ")" << staMacStr << XML_NEWLINE;

            anim->UpdateNodeDescription(sta_i_k, staname.str());

        }
    }

    void setupStasAnim() {
        for (size_t i = 0; i < staNodes.size(); i++) {
            setupStaAnim(staNodes[i], i);
        }
    }

    void setupStas(const WifiPhyBand initialBand = WIFI_PHY_BAND_2_4GHZ, const uint16_t initialWidth = 20) {
        cout << setw(20) << "SRV" << setw(20) << "CLI" << setw(20) << endl;;
        for (size_t i = 0; i < n_aps; i++) {
            staNodes.emplace_back();
            staNodes[i].Create(apStaAllocation[i]);
            staDevs.emplace_back();
            staInterfaces.emplace_back();
        }
        setupStasMobility();
        for (size_t i = 0; i < staNodes.size(); i++) {
            packetSocket.Install(staNodes[i]);
            std::string ssid = "ssid-" + std::to_string(i);
            switchChannel_attr(wifiPhy, apChannelAllocation[i]);
            wifiMac.SetType("ns3::StaWifiMac",
                    "ActiveProbing", BooleanValue(true),
                    "Ssid", SsidValue(Ssid(ssid)),
                    "QosSupported", BooleanValue(false)
                    );
            staDevs[i] = wifi.Install(wifiPhy, wifiMac, staNodes[i]);
            stack.Install(staNodes[i]);
            staInterfaces[i] = addressHelper.Assign(staDevs[i]);
            setupStaTrafficFlow(staNodes[i], staInterfaces[i]);
            for (size_t k = 0; k < staNodes[i].GetN(); k++) {
                CreateScannerForStaNode(staNodes[i].Get(k));
            }
        }
    }

    void setupStasMobility() {
        for (size_t i = 0; i < n_aps; i++) {
            std::string x = std::to_string(apPosRelativeVectors[i].x);
            std::string y = std::to_string(apPosRelativeVectors[i].y);
            std::string z = std::to_string(apPosRelativeVectors[i].z);
            mobility.SetPositionAllocator("ns3::RandomDiscPositionAllocator",
                                          "X", StringValue(x),
                                          "Y", StringValue(y),
                                          "Z", StringValue(z),
                                          "Rho", StringValue("ns3::UniformRandomVariable[Min=1|Max=1.5]") //
            );
            mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
            mobility.Install(staNodes[i]);
        }
    }

    void printSimulationParams() {
        cout << "============== Simulation parameters ==============================" << endl;
        cout << "=================== APs ===========================================" << endl;
        std::cout << std::left
            << std::setw(10) << "AP"
            << std::setw(10) << "Channel"
            << std::setw(25) << "SSID"
            << std::setw(20) << "BSSID"
            << std::endl;

        // Print data
        for (auto it = apNodes.Begin(); it != apNodes.End(); ++it) {
            int i = it - apNodes.Begin();
            std::stringstream bssidStr; bssidStr << getWifiNd(*it)->GetMac()->GetAddress(); string bssidStr_s = bssidStr.str();
            std::cout << std::left
                << std::setw(10) << i
                << std::setw(10) << +getWifiNd(*it)->GetPhy()->GetOperatingChannel().GetNumber()
                << std::setw(25) << getWifiNd(*it)->GetMac()->GetSsid().PeekString()
                << std::setw(20) << bssidStr_s
                << std::endl;
        }
        cout << "=================== STAs ===========================================" << endl;
        // Print headers
        std::cout << std::left
            << std::setw(10) << "STA"
            << std::setw(25) << "SSID"
            << std::setw(10) << "Channel"
            << std::setw(20) << "MAC"
            << std::endl;

        // Print data
        for (const auto& staNodes_ap_i : staNodes) {
            for (auto it = staNodes_ap_i.Begin(); it != staNodes_ap_i.End(); ++it) {
                int i = (it - staNodes_ap_i.Begin()) + staNodes_ap_i.GetN();
                std::stringstream bssidStr; bssidStr << getWifiNd(*it)->GetMac()->GetAddress(); string bssidStr_s = bssidStr.str();
                std::cout << std::left
                    << std::setw(10) << i
                    << std::setw(25) << getWifiNd(*it)->GetMac()->GetSsid().PeekString()
                    << std::setw(10) << +getWifiNd(*it)->GetPhy()->GetOperatingChannel().GetNumber()
                    << std::setw(20) << bssidStr_s
                    << std::endl;
            }
        }
        cout << "====================================================================" << endl;
    }

    double printThroughputResults() {
        double totalRxBytes = 0.0;
        Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
        FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
        std::set<Ipv4Address> visitedIps;
        cout << "====================================== Throughput results ==============================================" << endl;
        std::cout << std::left << std::setw(10) << "MAC1"
            << std::setw(10) << "MAC2"
            << std::setw(10) << "TxPkt"
            << std::setw(15) << "TxBytes"
            << std::setw(20) << "TxOffered (Mbps)"
            << std::setw(10) << "RxPkt"
            << std::setw(15) << "RxBytes"
            << std::setw(15) << "Thrpt (Mbps)"
            << std::endl;
        for (auto [flowId, flowStats] : stats)
        {
            Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
            Ipv4Address srcAddr = t.sourceAddress;
            Ipv4Address dstAddr = t.destinationAddress;
            totalRxBytes += flowStats.rxBytes;

            if (visitedIps.count(srcAddr)) {
                continue;
            }
            visitedIps.insert(dstAddr);
            // Mac48Address mac1 = ip2mac.at(t.sourceAddress);
            // Mac48Address mac2 = ip2mac.at(t.destinationAddress);
            auto printStats =  [&](Mac48Address& mac1, Mac48Address& mac2) {
                double txOffered = (flowStats.rxBytes * 8 / 1000.0 / 1000.0) / (trafficEndTime - trafficStartTime);
                double throughput = (flowStats.rxBytes * 8 / 1000.0 / 1000.0) / (trafficEndTime - trafficStartTime);

                std::stringstream mac1Str; mac1Str << mac1; string mac1Str_s = mac1Str.str().substr(14);
                std::stringstream mac2Str; mac2Str << mac2; string mac2Str_s = mac2Str.str().substr(14);
                std::cout << std::left << std::setw(10) << mac1Str_s
                    << std::setw(10) << mac2Str_s
                    << std::setw(10) << flowStats.txPackets
                    << std::setw(15) << flowStats.txBytes
                    << std::setw(20) << std::fixed << std::setprecision(4) << txOffered
                    << std::setw(10) << flowStats.rxPackets
                    << std::setw(15) << flowStats.rxBytes
                    << std::setw(15) << std::fixed << std::setprecision(4) << throughput
                    << std::endl;
            };
            printStats(ip2mac.at(srcAddr), ip2mac.at(dstAddr));
            printStats(ip2mac.at(dstAddr), ip2mac.at(srcAddr));
        }
        cout << "==================================================================================================" << endl;
        return (totalRxBytes * 8 / 1000.0 / 1000.0) / (trafficEndTime - trafficStartTime);
    }

    ~SimulationCase() {
        Simulator::Destroy();
    }
};

int main(int argc, char* argv[]) {
    CommandLine cmd(__FILE__);
    // { PyViz v; }
    cmd.AddValue("verbose", "Print trace information if true", g_verbose);
    cmd.AddValue("debug", "Print debug information if true", g_debug);
    cmd.AddValue("logic", "Enable info debug level", g_logic);
    cmd.Parse(argc, argv);
    if (g_debug) {
        LogComponentEnable("rrm-greedy-test", LOG_LEVEL_DEBUG);
        LogComponentEnable("StaWifiMac", LOG_LEVEL_DEBUG);
    } else if (g_logic) {
        LogComponentEnable("StaWifiMac", LogLevel(LOG_LEVEL_LOGIC & (~LOG_FUNCTION)));
    }
    // our modules have logic level logging by default
    LogComponentEnable("rrm-greedy-test", LOG_LEVEL_LOGIC);
    LogComponentEnable("rrm", LOG_LEVEL_LOGIC);
    LogComponentEnable("WifiMac", LOG_LEVEL_ERROR);
    LogComponentEnable("WifiHelper", LOG_LEVEL_ERROR);
    RngSeedManager::SetSeed(2);

    constexpr int NUM_APS = 4;
    // vector<shared_ptr<Scanner>> scanners(NUM_APS);

    vector<uint16_t> apChannelAllocation = {
        1,
        1,
        1,
        1
    };

    vector<double> apTxpAllocationDbm = {
        20,
        20,
        20,
        20
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

    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~ Initial Simulation ~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    auto initialSettingsCase = new SimulationCase(
            apChannelAllocation,
            apStaAllocation,
            apTxpAllocationDbm,
            {1, 6, 11},
            0.0,
            10.0,
            0.5,
            10.0,
            "rrmgreedy-before.xml"
    );
    SimulationCase::SimulationCaseResults initialResults = initialSettingsCase->runSimulation(true);
    delete initialSettingsCase;

    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~ With RRM Simulation ~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    auto withRrmCase = new SimulationCase(
            initialResults.newRrmAlloc.apChannelAllocation,
            apStaAllocation,
            initialResults.newRrmAlloc.apTxpAllocationDbm,
            {1, 6, 11},
            0.0,
            10.0,
            0.5,
            10.0,
            "rrmgreedy-before.xml"
    );
    // SimulationCase::SimulationCaseResults withRrm = withRrmCase->runSimulation(false);
    withRrmCase->runSimulation(false);
    delete withRrmCase;
    return 0;
}

class ApSettings {
public:
    uint16_t channel;
    double txp;
    ApSettings(uint16_t channel, double txp) : channel(channel), txp(txp) {}
};



