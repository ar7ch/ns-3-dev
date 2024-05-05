/*
 * Copyright (c) 2005,2006,2007 INRIA
 *
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
#include <iomanip>

using namespace ns3;
using std::cout, std::endl, std::setw, std::setprecision, std::vector, std::string, std::map, std::set;
NS_LOG_COMPONENT_DEFINE("wifi-monitor-mode");

#define LOG_LOGIC(x) do { NS_LOG_LOGIC(std::setprecision(7) << Simulator::Now().GetSeconds() << "s: " << x ); } while(0)
#define LOG_DEBUG(x) do { NS_LOG_DEBUG(std::setprecision(7) << Simulator::Now().GetSeconds() << "s: " << x ); } while(0)

/// True for verbose output.
static bool g_verbose = false;
static bool g_debug = false;
static bool g_logic = false;

std::shared_ptr<Scanner>
doScanning(int scanningApIdx) {
    Packet::EnablePrinting();
    // LogComponentEnable("WifiPhy", LOG_LEVEL_DEBUG);
    // if
    if (g_debug) {
        LogComponentEnable("wifi-monitor-mode", LOG_LEVEL_DEBUG);
        LogComponentEnable("StaWifiMac", LOG_LEVEL_DEBUG);
    } else if (g_logic) {
        LogComponentEnable("StaWifiMac", LogLevel(LOG_LEVEL_LOGIC & (~LOG_FUNCTION)));
    }
    LogComponentEnable("wifi-monitor-mode", LOG_LEVEL_LOGIC);
    LogComponentEnable("rrm", LOG_LEVEL_LOGIC);
    // LogComponentEnable("WifiDefaultAssocManager", LOG_LEVEL_LO);

    WifiHelper wifi;
    MobilityHelper mobility;
    NodeContainer staNodes;
    NodeContainer apNodes;
    // NodeContainer scanAp;

    NetDeviceContainer staDevs;
    NetDeviceContainer apDevs;
    PacketSocketHelper packetSocket;

    const int n_stas = 2;
    const int n_aps = 3;
    const uint16_t initialChannel = 1;
    const uint16_t initialWidth = 20;
    const WifiPhyBand initialBand = WIFI_PHY_BAND_2_4GHZ;

    staNodes.Create(n_stas);
    apNodes.Create(n_aps);
    // scanAp.Create(1);

    // give packet socket powers to nodes.
    packetSocket.Install(staNodes);
    packetSocket.Install(apNodes);
    // packetSocket.Install(scanAp);

    // setup wifi
    wifi.SetStandard(WIFI_STANDARD_80211n);

    WifiMacHelper wifiMac;
    YansWifiPhyHelper wifiPhy;

    // setup wifi channel helper
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    // initial channel settings
    TupleValue<UintegerValue, UintegerValue, EnumValue<WifiPhyBand>, UintegerValue> channelValue;
    channelValue.Set(WifiPhy::ChannelTuple {initialChannel, initialWidth, initialBand, 0});
    wifiPhy.Set("ChannelSettings", channelValue);

    Ssid ssid = Ssid("wifi-default");

    auto setupSta = [&]() {
        wifiMac.SetType("ns3::StaWifiMac",
                        "ActiveProbing",
                        BooleanValue(true),
                        "Ssid",
                        SsidValue(ssid),
                        "QosSupported",
                        BooleanValue(false)
        );
        staDevs = wifi.Install(wifiPhy, wifiMac, staNodes);
    };
    // setup stas.
    setupSta();

    wifiMac.SetType("ns3::ApWifiMac",
            "Ssid",
            SsidValue(ssid),
            "QosSupported",
            BooleanValue(false)
            );
    apDevs.Add(wifi.Install(wifiPhy, wifiMac, apNodes.Get(0)));

    wifiMac.SetType("ns3::ApWifiMac",
            "Ssid",
            SsidValue(Ssid("ssid-2")),
            "QosSupported",
            BooleanValue(false)
            );
    apDevs.Add(wifi.Install(wifiPhy, wifiMac, apNodes.Get(1)));

    // setup scanning ap
    wifiMac.SetType("ns3::ApWifiMac",
            "Ssid", SsidValue(Ssid("ssid-3")),
            "BeaconGeneration", BooleanValue(true)
    );
    wifi.Install(wifiPhy, wifiMac, apNodes.Get(2));

    // mobility.
    mobility.Install(staNodes);
    mobility.Install(apNodes);

    // Simulator::Schedule(Seconds(1.0), &AdvancePosition, ap.Get(0));

    // -------------------

    //
    switchChannel(getWifiNd(apNodes.Get(1)), 11);

    InternetStackHelper stack;
    stack.Install(staNodes);
    stack.Install(apNodes);

    Ipv4AddressHelper address;
    address.SetBase("1.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevs);
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevs);


    constexpr uint16_t echoPort = 9;
    UdpEchoServerHelper echoServer(echoPort);
    ApplicationContainer serverApps = echoServer.Install(staNodes.Get(0));

    UdpEchoClientHelper echoClient(staInterfaces.GetAddress(0), echoPort);
    double maxPackets = 0; // 0 means unlimited
    double packetInterval = 0.005;
    int packetSize = 1024;
    echoClient.SetAttribute("MaxPackets", UintegerValue(maxPackets));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));
    NS_LOG_LOGIC("Packets rate: " << ((packetSize / packetInterval) / 1024) * 8 << " kbps");
    ApplicationContainer clientApps = echoClient.Install(staNodes.Get(1));

    /* Populate routing table */
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    constexpr double onOffStartTime = 0.5;
    constexpr double onOffStopTime = 10.0;

    serverApps.Start(Seconds(onOffStartTime));
    serverApps.Stop(Seconds(onOffStopTime));
    clientApps.Start(Seconds(onOffStartTime));
    clientApps.Stop(Seconds(onOffStopTime));

    // simulation

    Simulator::Stop(Seconds(onOffStopTime));
    // cout << "AP1 bssid: " << apNodes.Get(0)->GetDevice(0)->GetAddress() << endl;
    // cout << "AP2 bssid: " << apNodes.Get(1)->GetDevice(0)->GetAddress() << endl;
    // cout << "AP 3 bssid: " << apNodes.Get(2)->GetDevice(0)->GetAddress() << endl;
    // cout << "STA1 mac: " << staNodes.Get(0)->GetDevice(0)->GetAddress() << endl;
    // cout << "STA2 mac: " << staNodes.Get(1)->GetDevice(0)->GetAddress() << endl;

    for (auto it = apNodes.Begin(); it != apNodes.End(); ++it) {
        int i = it - apNodes.Begin();
        SIM_LOG_LOGIC("AP " << i
             << ",channel: " << +getWifiNd(*it)->GetPhy()->GetOperatingChannel().GetNumber()
             << ",ssid: " << getWifiNd(*it)->GetMac()->GetSsid().PeekString()
             << ",bssid: " << (*it)->GetDevice(0)->GetAddress()
        );
    }

    for (auto it = staNodes.Begin(); it != staNodes.End(); ++it) {
        int i = it - staNodes.Begin();
        SIM_LOG_LOGIC("STA " << i
             << ",ssid: " << getWifiNd(*it)->GetMac()->GetSsid().PeekString()
             << ",channel: " << +getWifiNd(*it)->GetPhy()->GetOperatingChannel().GetNumber()
             << ",mac: " << (*it)->GetDevice(0)->GetAddress()
        );
    }

    Ptr<Node> scannerWifiNode = apNodes.Get(scanningApIdx);
    vector<uint16_t> operatingChannels{1, 6, 11};
    auto scanner = CreateScannerForNode(scannerWifiNode, operatingChannels);

    switchChannel(getWifiNd(apNodes.Get(2)), 1);

    Simulator::Schedule(Seconds(2.5), &Scanner::Scan, &(*scanner));
    Simulator::Run();
    scanner->PrintScanResults();
    return scanner;
}

int main(int argc, char* argv[]) {
    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Print trace information if true", g_verbose);
    cmd.AddValue("debug", "Print debug information if true", g_debug);
    cmd.AddValue("logic", "Enable info debug level", g_logic);
    cmd.Parse(argc, argv);

    auto scanner = doScanning(0);
    assert(!scanner->knownAps.empty());
    assert(scanner->getOperatingChannel() == 6);
    Simulator::Destroy();

    scanner = doScanning(1);
    assert(!scanner->knownAps.empty());
    assert(scanner->getOperatingChannel() == 6);
    Simulator::Destroy();

    scanner = doScanning(2);
    assert(!scanner->knownAps.empty());
    assert(scanner->getOperatingChannel() == 6);
    Simulator::Destroy();
    return 0;
}

