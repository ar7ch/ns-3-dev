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

/**
 * Move a node position by 5m on the x axis every second, up to 210m.
 *
 * \param node The node.
 */
// static void
// AdvancePosition(Ptr<Node> node)
// {
//     Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
//     Vector pos = mobility->GetPosition();
//     pos.x += 5.0;
//     if (pos.x >= 210.0)
//     {
//         return;
//     }
//     mobility->SetPosition(pos);
//
//     Simulator::Schedule(Seconds(1.0), &AdvancePosition, node);
// }


string assembleChannelSettings(uint16_t channel, uint16_t width, string band) {
    assert(band == "BAND_2_4GHZ" || band == "BAND_5GHZ");
    std::stringstream ss;
    ss << "{"
        << std::to_string(channel) << ", "
        << std::to_string(width) << ", "
        << band
        << ", " << "0" << "}";
    return ss.str();
}

void switchChannelv2(Ptr<WifiNetDevice> dev, uint16_t newOperatingChannel, WifiPhyBand band=WIFI_PHY_BAND_2_4GHZ, uint16_t width=20) {
    Ptr<WifiPhy> phy = dev->GetPhy();
    if (phy->IsStateSleep())
    {
        phy->ResumeFromSleep();
    }
    if (phy->IsStateSwitching()){
        NS_LOG_DEBUG(Simulator::Now().GetSeconds() << "s: AP channel switch is in progress, postpone the switch");
        Simulator::Schedule(Seconds(0.1), &switchChannelv2, dev, newOperatingChannel, band, width);
    }


    WifiPhyOperatingChannel channelToSwitch = WifiPhyOperatingChannel(WifiPhyOperatingChannel::FindFirst(
                    newOperatingChannel,
                    0,
                    20,
                    phy->GetStandard(),
                    band
                    )
                );
    NS_LOG_DEBUG(Simulator::Now().GetSeconds() <<
            "s: switch AP channel: " << +phy->GetOperatingChannel().GetNumber() <<
            "->" << +channelToSwitch.GetNumber());
    WifiPhy::ChannelTuple chTuple{channelToSwitch.GetNumber(),
        channelToSwitch.GetWidth(),
        channelToSwitch.GetPhyBand(),
        channelToSwitch.GetPrimaryChannelIndex(20)};
    phy->SetOperatingChannel(chTuple);
}

void switchChannel(Ptr<WifiNetDevice> dev, uint16_t operatingChannel, WifiPhyBand band=WIFI_PHY_BAND_2_4GHZ, uint16_t width=20) {
    string bandStr;
    switch(band) {
        case WIFI_PHY_BAND_2_4GHZ:
            assert(operatingChannel >= 1 && operatingChannel <= 14);
            bandStr = "BAND_2_4GHZ";
            break;
        case WIFI_PHY_BAND_5GHZ:
            assert(operatingChannel >= 36 && operatingChannel <= 165);
            bandStr = "BAND_5GHZ";
            break;
        default:
            assert(false);
    }
    Ptr<WifiPhy> phy = dev->GetPhy();
    phy->SetAttribute("ChannelSettings", StringValue(
        assembleChannelSettings(operatingChannel, width, bandStr))
    );
    // assert(phy->GetOperatingChannel().GetNumber() == operatingChannel);
}

// class Scanner {
// public:
//     Scanner() = delete;
//     virtual ~Scanner() {};
//     virtual void Scan() = 0;
//     virtual void PrintScanResults() = 0;
//     virtual void addAp(uint16_t channel, Mac48Address bssid) = 0;
//
//     uint16_t getOperatingChannel();
// };

class Scanner {
public:
    struct ScanData {
        uint16_t channel = 0;
        uint16_t width = 0;
        double snr = 0.0;
        double rssi = 0.0;
        set<Mac48Address> clients;

        ScanData() {}

        ScanData(uint16_t channel, uint16_t width, double snr, double rssi) :
            channel(channel), width(width), snr(snr), rssi(rssi) {}
    };

    Scanner(Ptr<WifiNetDevice> wifidev,
            vector<uint16_t> opChannelsList) : dev(wifidev),
                                                channelsToScan(opChannelsList),
                                                operatingChannelsList(opChannelsList) {}
    ~Scanner() {}

    void Scan() {
        Ptr<WifiPhy> phy = dev->GetPhy();
        dataChannel = phy->GetChannelNumber();
        Simulator::ScheduleNow(&Scanner::scanChannel, this, channelsToScan.begin());
    }

    void PrintScanResults() {
        cout << endl << "Scan data:" << endl;
        cout << "BSSID" << "\t\t\t" << "Channel" << "\t" << "SNR" << "\t\t" << "RSSI" << endl;
        for (auto& [bssid, scanData] : knownAps) {
            cout << bssid << "\t" << scanData.channel << "\t" << scanData.snr << "\t" << scanData.rssi << endl;
        }
    }

    // void addAp(uint16_t channel, Mac48Address bssid) {
    //     knownAps.insert(bssid);
    //     apMap[channel].insert(bssid);
    // }

    uint16_t getOperatingChannel() {
        return dev->GetPhy()->GetChannelNumber();
    }
    std::map<Mac48Address, ScanData> knownAps;
private:
    double channelDwellTime_s = 0.2;
    double scanInterval_s = 1.0;
    int dataChannel;
    Ptr<WifiNetDevice> dev;
    vector<uint16_t> channelsToScan;
    vector<uint16_t> operatingChannelsList;

    void scanChannel(std::vector<uint16_t>::iterator nextChanIt) {
        if (nextChanIt == channelsToScan.end()) {
            Simulator::ScheduleNow(&Scanner::lccsDecide, this);
            return;
        }
        uint16_t channel = *nextChanIt;
        switchChannel(dev, channel);
        LOG_LOGIC("Scanning channel " << +channel);
        nextChanIt++;
        Simulator::Schedule(Seconds(channelDwellTime_s), &Scanner::returnToDataChannel, this, nextChanIt);
    }

    void returnToDataChannel(std::vector<uint16_t>::iterator nextChanIt) {
        switchChannel(dev, dataChannel);
        LOG_LOGIC("Returning to data channel " << +dataChannel);
        if (nextChanIt != channelsToScan.end() && *nextChanIt == dataChannel) {
            LOG_LOGIC("Skipping scan for data channel");
            nextChanIt++;
        }
        Simulator::Schedule(Seconds(scanInterval_s), &Scanner::scanChannel, this, nextChanIt);
    }

    void lccsDecide() {
        // least congested channel selection
        // for each possible channel, calculate channel metric: number of APs + number of clients
        // choose the channel with the lowest metric
        // if the metric is the same, choose the channel with the lowest number
        cout << "Running LCCS" << endl;
        cout << "Gathered scan data:" << endl;
        for (auto& [bssid, scanData] : knownAps) {
            cout << "BSSID: " << bssid
                << ", channel: " << scanData.channel
                << ", clients: " << scanData.clients.size()
                << ", SNR: " << scanData.snr
                << ", RSSI: " << scanData.rssi << endl;
        }
        const int NO_DATA_METRIC = 0;
        std::map<int, int> metric;
        for (auto& channel : operatingChannelsList) {
            metric[channel] = NO_DATA_METRIC;
        }
        for (auto& [bssid, scanData] : knownAps) {
            if (metric[scanData.channel] == NO_DATA_METRIC) {
                metric[scanData.channel] = 0; // start with neutral metric value
            }
            metric[scanData.channel]++;
            metric[scanData.channel] += 10*scanData.clients.size();
        }

        // find the channel with the lowest metric
        int minMetric = INT_MAX;
        int newChannel = 0;
        for (int ch_i : operatingChannelsList) {
            cout << "channel " << ch_i << " metric: " << metric[ch_i];
            if (metric[ch_i] < minMetric) {
                if (metric[ch_i] == NO_DATA_METRIC &&
                        std::find(channelsToScan.begin(), channelsToScan.end(),
                            ch_i) == channelsToScan.end() ) {
                    cout << " (no scan data for this channel)" << endl;
                    assert(false);
                    continue;
                }
                minMetric = metric[ch_i];
                newChannel = ch_i;
            }
            cout << endl;
        }
        LOG_LOGIC("LCCS: switching to channel " << newChannel);
        switchChannel(dev, newChannel);
    }
};

// since no additional parameters can be passed to the callback, we maintain
// a global map that Scanners can be accessed by the trace context
// (that contains info about device and corresponding scanner)
static map<string, std::shared_ptr<Scanner>> scannerByTraceContext;

map<uint16_t, uint16_t> ofdmFreqToChanNumber = {
    {2412, 1},
    {2417, 2},
    {2422, 3},
    {2427, 4},
    {2432, 5},
    {2437, 6},
    {2442, 7},
    {2447, 8},
    {2452, 9},
    {2457, 10},
    {2462, 11},
    {2467, 12},
    {2472, 13},
    {2484, 14},
    {5180, 36},
    {5200, 40},
    {5220, 44},
    {5240, 48},
    {5260, 52},
    {5280, 56},
    {5300, 60},
    {5320, 64},
    {5500, 100},
    {5520, 104},
    {5540, 108},
    {5560, 112},
    {5580, 116},
    {5600, 120},
    {5620, 124},
    {5640, 128},
    {5660, 132},
    {5680, 136},
    {5700, 140},
    {5745, 149},
    {5765, 153},
    {5785, 157},
    {5805, 161},
    {5825, 165},
};

void monitorSniffer(
        std::string context, Ptr<const Packet> p,
        uint16_t channelFreqMhz,
        WifiTxVector txVector,
        MpduInfo aMpdu,
        SignalNoiseDbm signalNoise,
        uint16_t staId) {

    Ptr<Packet> packet = p->Copy();

    WifiMacHeader hdr;
    packet->RemoveHeader(hdr);
    if (g_verbose || g_debug) {
        cout << setw(7) << std::fixed << std::setprecision(5) << Simulator::Now().GetSeconds() << "s: ";
        hdr.Print(cout);
        cout << endl;
        if (hdr.IsQosData()) {
            cout << "QoS data! "
                << "RA: " << hdr.GetAddr1()
                << " TA: " << hdr.GetAddr2()
                << " DA: " << hdr.GetAddr3()
                << " SA: " << hdr.GetAddr4()
                << " FromDS: " << hdr.IsFromDs()
                << " ToDS: " << hdr.IsToDs()
                << " Retry: " << hdr.IsRetry()
                << endl;
        }
    }
    std::shared_ptr<Scanner> scanner = scannerByTraceContext[context];

    if (hdr.IsBeacon() || (hdr.IsData() && hdr.IsFromDs())) { // access point
        uint16_t channel = scanner->getOperatingChannel();
        // check consistency of channel usage
        assert(ofdmFreqToChanNumber.find(channelFreqMhz) != ofdmFreqToChanNumber.end());
        assert(channel == ofdmFreqToChanNumber[channelFreqMhz]);

        double snr = signalNoise.signal / signalNoise.noise;
        Mac48Address bssid = hdr.GetAddr2();

        if (scanner->knownAps.count(bssid)) {
            scanner->knownAps[bssid].snr = snr;
            scanner->knownAps[bssid].rssi = signalNoise.signal;
        } else {
            Scanner::ScanData scanData{channel, 20, snr, signalNoise.signal};
            scanner->knownAps[bssid] = scanData;
        }
        NS_LOG_DEBUG("Beacon from " << bssid << " on channel " << channel);
    } else if (hdr.IsData() && hdr.IsToDs()) { // client
        Mac48Address bssid = hdr.GetAddr1();
        Mac48Address client = hdr.GetAddr2();

        if (scanner->knownAps.count(bssid)) {
            scanner->knownAps[bssid].clients.insert(client);
        }
        NS_LOG_DEBUG("Data from " << client
                        << " to " << bssid
                        << " on channel " << scanner->getOperatingChannel());
    }
}

Ptr<WifiNetDevice> getWifiNd (Ptr<Node> node, int idx=0) {
    return DynamicCast<WifiNetDevice>(node->GetDevice(idx));
};


std::shared_ptr<Scanner>
CreateScannerForNode(Ptr<Node> scannerWifiNode, vector<uint16_t> operatingChannels) {

    Ptr<WifiNetDevice> scannerWifiNetDev = getWifiNd(scannerWifiNode);
    std::stringstream ss;
    ss << "/NodeList/" << scannerWifiNode->GetId()
        << "/DeviceList/" << scannerWifiNetDev->GetIfIndex()
        << "/$ns3::WifiNetDevice/Phy/MonitorSnifferRx";
    string scanApTraceStr = ss.str();
    std::shared_ptr<Scanner> scanner = std::make_shared<Scanner>(scannerWifiNetDev, operatingChannels);
    scannerByTraceContext[scanApTraceStr] = scanner;
    Config::Connect(scanApTraceStr, MakeCallback(&monitorSniffer));
    return scanner;
}

int main(int argc, char* argv[]) {
    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Print trace information if true", g_verbose);
    cmd.AddValue("debug", "Print debug information if true", g_debug);
    cmd.AddValue("logic", "Enable info debug level", g_logic);
    cmd.Parse(argc, argv);

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
    // LogComponentEnable("WifiDefaultAssocManager", LOG_LEVEL_LO);

    WifiHelper wifi;
    MobilityHelper mobility;
    NodeContainer stas;
    NodeContainer ap;
    NodeContainer scanAp;

    NetDeviceContainer staDevs;
    NetDeviceContainer apDevs;
    PacketSocketHelper packetSocket;

    stas.Create(2);
    ap.Create(2);
    scanAp.Create(1);

    // give packet socket powers to nodes.
    packetSocket.Install(stas);
    packetSocket.Install(ap);
    packetSocket.Install(scanAp);

    // setup wifi
    wifi.SetStandard(WIFI_STANDARD_80211n);

    WifiMacHelper wifiMac;
    YansWifiPhyHelper wifiPhy;

    // setup wifi channel helper
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    // initial channel settings
    TupleValue<UintegerValue, UintegerValue, EnumValue<WifiPhyBand>, UintegerValue> channelValue;
    channelValue.Set(WifiPhy::ChannelTuple {1, 20, WIFI_PHY_BAND_2_4GHZ, 0});
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
        staDevs = wifi.Install(wifiPhy, wifiMac, stas);
    };
    // setup stas.
    setupSta();

    wifiMac.SetType("ns3::ApWifiMac",
            "Ssid",
            SsidValue(ssid),
            "QosSupported",
            BooleanValue(false)
            );
    apDevs.Add(wifi.Install(wifiPhy, wifiMac, ap.Get(0)));

    wifiMac.SetType("ns3::ApWifiMac",
            "Ssid",
            SsidValue(Ssid("ssid-2")),
            "QosSupported",
            BooleanValue(false)
            );
    apDevs.Add(wifi.Install(wifiPhy, wifiMac, ap.Get(1)));

    // setup scanning ap
    wifiMac.SetType("ns3::ApWifiMac",
            "Ssid", SsidValue(Ssid("scan-ap-ssid")),
            "BeaconGeneration", BooleanValue(true)
    );
    wifi.Install(wifiPhy, wifiMac, scanAp);

    // mobility.
    mobility.Install(stas);
    mobility.Install(ap);
    mobility.Install(scanAp);

    // Simulator::Schedule(Seconds(1.0), &AdvancePosition, ap.Get(0));

    // -------------------

    //
    Ptr<WifiNetDevice> scanApNetDev = getWifiNd(scanAp.Get(0));
    Ptr<WifiNetDevice> apNetDev = getWifiNd(ap.Get(0));
    Ptr<WifiNetDevice> ap2NetDev = getWifiNd(ap.Get(1));
    switchChannel(ap2NetDev, 1);

    InternetStackHelper stack;
    stack.Install(stas);
    stack.Install(ap);

    Ipv4AddressHelper address;
    address.SetBase("1.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevs);
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevs);


    constexpr uint16_t echoPort = 9;
    UdpEchoServerHelper echoServer(echoPort);
    ApplicationContainer serverApps = echoServer.Install(stas.Get(0));

    UdpEchoClientHelper echoClient(staInterfaces.GetAddress(0), echoPort);
    double maxPackets = 0; // 0 means unlimited
    double packetInterval = 0.005;
    int packetSize = 1024;
    echoClient.SetAttribute("MaxPackets", UintegerValue(maxPackets));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(packetInterval)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));
    NS_LOG_LOGIC("Packets rate: " << ((packetSize / packetInterval) / 1024) * 8 << " kbps");
    ApplicationContainer clientApps = echoClient.Install(stas.Get(1));

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

    cout << "AP1 bssid: " << ap.Get(0)->GetDevice(0)->GetAddress() << endl;
    cout << "AP2 bssid: " << ap.Get(1)->GetDevice(0)->GetAddress() << endl;
    cout << "STA1 mac: " << stas.Get(0)->GetDevice(0)->GetAddress() << endl;
    cout << "STA2 mac: " << stas.Get(1)->GetDevice(0)->GetAddress() << endl;
    cout << "scanning ap bssid: " << scanAp.Get(0)->GetDevice(0)->GetAddress() << endl;

    Ptr<Node> scannerWifiNode= ap.Get(0);
    vector<uint16_t> operatingChannels{1, 6, 11};
    auto scanner = CreateScannerForNode(scannerWifiNode, operatingChannels);

    switchChannel(scanApNetDev, 11);

    Simulator::Schedule(Seconds(2.5), &Scanner::Scan, &(*scanner));
    Simulator::Run();
    Simulator::Destroy();

    scanner->PrintScanResults();
    return 0;
}

