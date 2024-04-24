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

#include "ns3/wifi-net-device.h"
#include "ns3/net-device.h"
#include <cassert>
#include <iomanip>

using namespace ns3;
using std::cout, std::endl, std::setw, std::setprecision, std::vector, std::string, std::map, std::set;
NS_LOG_COMPONENT_DEFINE("wifi-monitor-mode");

/// True for verbose output.
static bool g_verbose = true;

/**
 * Move a node position by 5m on the x axis every second, up to 210m.
 *
 * \param node The node.
 */
static void
AdvancePosition(Ptr<Node> node)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    Vector pos = mobility->GetPosition();
    pos.x += 5.0;
    if (pos.x >= 210.0)
    {
        return;
    }
    mobility->SetPosition(pos);

    Simulator::Schedule(Seconds(1.0), &AdvancePosition, node);
}


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
    assert(phy->GetOperatingChannel().GetNumber() == operatingChannel);
}

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

    Scanner(Ptr<WifiNetDevice> wifidev, vector<uint16_t> scanlist, vector<uint16_t> opChannelsList) : channelsToScan(scanlist), operatingChannelsList(opChannelsList), dev(wifidev) {}
    ~Scanner() {}

    void Scan() {
        Ptr<WifiPhy> phy = dev->GetPhy();
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
    // std::map<uint16_t, std::set<Mac48Address>> apMap;
    std::map<Mac48Address, ScanData> knownAps;
    // std::set<Mac48Address> tempChannelScanResults;
    // std::set<Mac48Address> knownAps;
private:
    std::vector<uint16_t> channelsToScan;
    std::vector<uint16_t> operatingChannelsList;
    Ptr<WifiNetDevice> dev;
    /*
     * Initial idea: for each channel, scan it for 0.5s sequentially. This sequential scan is launched once in a while by a scheduled Scan callback.
     * As an idea for next iteration, implement real-life behavior: AP switches to channel to scan, then returns back to operating channel, then switches to the next channel to scan
     */
    void scanChannel(std::vector<uint16_t>::iterator chanIt) {
        if (chanIt == channelsToScan.end()) {
            Simulator::ScheduleNow(&Scanner::lccsDecide, this);
            return;
        }
        uint16_t channel = *chanIt;
        switchChannel(dev, channel);
        cout << setprecision(7) << Simulator::Now().GetSeconds() << "s: " <<
            "scanning channel " << channel << endl;
        chanIt++;
        Simulator::Schedule(Seconds(0.2), &Scanner::scanChannel, this, chanIt);
    }

    void lccsDecide() {
        // least congested channel selection
        // for each possible channel, calculate channel metric: number of APs + number of clients
        // choose the channel with the lowest metric
        // if the metric is the same, choose the channel with the lowest number
        cout << "Running LCCS" << endl;
        cout << "Gathered scan data:" << endl;
        for (auto& [bssid, scanData] : knownAps) {
            cout << "BSSID: " << bssid << ", channel: " << scanData.channel << ", clients: " << scanData.clients.size() << ", SNR: " << scanData.snr << ", RSSI: " << scanData.rssi << endl;
        }
        vector<int> metric(operatingChannelsList.size(), 0);
        for (auto& [bssid, scanData] : knownAps) {
            metric[scanData.channel]++;
            metric[scanData.channel] += 10*scanData.clients.size();
        }
        int minMetric = INT_MAX;
        size_t minChannelIdx = 0;
        for (size_t i = 0; i < metric.size(); i++) {
            cout << "channel " << operatingChannelsList[i] << " metric: " << metric[i] << endl;
            if (metric[i] < minMetric) {
                minMetric = metric[i];
                minChannelIdx = i;
            }
        }
        uint16_t newChannel = operatingChannelsList[minChannelIdx];
        cout << "switching to channel " << newChannel << endl;
        switchChannel(dev, newChannel);
    }
};

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
    // if (g_verbose) {
    //     cout << setw(7) << std::fixed << std::setprecision(5) << Simulator::Now().GetSeconds() << "s: ";
    //     cout << "context: " << context << ", ";
    //     packet->Print(cout);
    //     cout << endl;
    // }

    WifiMacHeader hdr;
    packet->RemoveHeader(hdr);
    if (g_verbose) {
        cout << setw(7) << std::fixed << std::setprecision(5) << Simulator::Now().GetSeconds() << "s: ";
        hdr.Print(cout);
        cout << endl;
        if (hdr.IsQosData()) {
            cout << "QoS data! " << "RA: " << hdr.GetAddr1() << " TA: " << hdr.GetAddr2() << " DA: " << hdr.GetAddr3() << " SA: " << hdr.GetAddr4() << " FromDS: " << hdr.IsFromDs() << " ToDS: " << hdr.IsToDs() << endl;
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
        // code below only complicates the matter. commented for now
        // else {
        //     double snr = -1;
        //     double rssi = -1;
        //     Scanner::ScanData scanData{0, 20, rssi, snr};
        //     scanData.clients.insert(client);
        //     scanner->knownAps[bssid] = scanData;
        // }
        NS_LOG_DEBUG("Data from " << client << " to " << bssid << " on channel " << scanner->getOperatingChannel());
    }
}

int main(int argc, char* argv[]) {
    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Print trace information if true", g_verbose);
    cmd.Parse(argc, argv);

    Packet::EnablePrinting();
    // LogComponentEnable("WifiPhy", LOG_LEVEL_DEBUG);

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

    WifiMacHelper wifiMac;
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    TupleValue<UintegerValue, UintegerValue, EnumValue, UintegerValue> value;
    value.Set(WifiPhy::ChannelTuple {1, 20, WIFI_PHY_BAND_2_4GHZ, 0});
    wifiPhy.Set("ChannelSettings", value);
    Ssid ssid = Ssid("wifi-default");
    // setup stas.
    wifiMac.SetType("ns3::StaWifiMac",
                    "ActiveProbing",
                    BooleanValue(false),
                    "Ssid",
                    SsidValue(ssid),
                    "QosSupported",
                    BooleanValue(false)
    );
    wifi.SetStandard(WIFI_STANDARD_80211n);
    staDevs = wifi.Install(wifiPhy, wifiMac, stas);
    // setup ap.
    wifiMac.SetType("ns3::ApWifiMac",
            "Ssid", SsidValue(ssid),
            "QosSupported", BooleanValue(false)
    );
    wifi.Install(wifiPhy, wifiMac, ap);
    wifiMac.SetType("ns3::ApWifiMac",
            "Ssid", SsidValue(Ssid("scan-ap-ssid")),
            "BeaconGeneration", BooleanValue(false)
    );
    wifi.Install(wifiPhy, wifiMac, scanAp);

    // mobility.
    mobility.Install(stas);
    mobility.Install(ap);
    mobility.Install(scanAp);

    // why not work???
    Ptr<WifiNetDevice> scanApNetDev = DynamicCast<WifiNetDevice>(scanAp.Get(0)->GetDevice(0));
    Ptr<WifiNetDevice> apNetDev = DynamicCast<WifiNetDevice>(ap.Get(0)->GetDevice(0));
    Ptr<WifiNetDevice> ap2NetDev = DynamicCast<WifiNetDevice>(ap.Get(1)->GetDevice(0));
    switchChannel(ap2NetDev, 6);

    Simulator::Schedule(Seconds(1.0), &AdvancePosition, ap.Get(0));
    // Simulator::Schedule(Seconds(1.25), &switchChannel, apNetDev, 6);

    PacketSocketAddress socket;
    socket.SetSingleDevice(staDevs.Get(0)->GetIfIndex());
    socket.SetPhysicalAddress(staDevs.Get(1)->GetAddress());
    socket.SetProtocol(1);

    OnOffHelper onoff("ns3::PacketSocketFactory", Address(socket));
    onoff.SetConstantRate(DataRate("500kb/s"));

    ApplicationContainer apps = onoff.Install(stas.Get(0));

    double onOffStartTime = 0.5;
    double onOffStopTime = 10.0;
    apps.Start(Seconds(onOffStartTime));
    apps.Stop(Seconds(onOffStopTime));

    Simulator::Stop(Seconds(onOffStopTime + 1.0));

    cout << "AP1 bssid: " << ap.Get(0)->GetDevice(0)->GetAddress() << endl;
    cout << "AP2 bssid: " << ap.Get(1)->GetDevice(0)->GetAddress() << endl;
    cout << "STA1 mac: " << stas.Get(0)->GetDevice(0)->GetAddress() << endl;
    cout << "STA2 mac: " << stas.Get(1)->GetDevice(0)->GetAddress() << endl;
    cout << "scanning ap bssid: " << scanAp.Get(0)->GetDevice(0)->GetAddress() << endl;

    std::stringstream ss;
    ss << "/NodeList/" << scanAp.Get(0)->GetId() << "/DeviceList/" << scanAp.Get(0)->GetDevice(0)->GetIfIndex() << "/$ns3::WifiNetDevice/Phy/MonitorSnifferRx";
    string scanApTraceStr = ss.str();
    vector<uint16_t> scanlist{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    vector<uint16_t> availableChannels{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    std::shared_ptr<Scanner> scanner = std::make_shared<Scanner>(scanApNetDev, scanlist, availableChannels);
    scannerByTraceContext[scanApTraceStr] = scanner;
    Config::Connect(scanApTraceStr, MakeCallback(&monitorSniffer));

    Simulator::Schedule(Seconds(0.5), &Scanner::Scan, &(*scanner));
    Simulator::Run();
    Simulator::Destroy();

    scanner->PrintScanResults();
    return 0;
}
