#include "ns3/core-module.h"
#include "ns3/ssid.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-mac.h"
#include "ns3/ap-wifi-mac.h"

#include "ns3/wifi-net-device.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/net-device.h"
#include <cassert>
#include <iomanip>
#include <limits>
#include "ns3/rrm.h"


using std::cout, std::endl, std::setw, std::setprecision, std::vector, std::string, std::map, std::set;
NS_LOG_COMPONENT_DEFINE("rrm");

namespace ns3 {

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

void switchChannelv2(Ptr<WifiNetDevice> dev, uint16_t newOperatingChannel, WifiPhyBand band, uint16_t width) {
    Ptr<WifiPhy> phy = dev->GetPhy();
    if (phy->IsStateSleep())
    {
        phy->ResumeFromSleep();
    }
    if (phy->IsStateSwitching()){
        SIM_LOG_DEBUG("AP channel switch is in progress, postpone the switch");
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
    SIM_LOG_DEBUG("switch AP channel: " << +phy->GetOperatingChannel().GetNumber() <<
            "->" << +channelToSwitch.GetNumber());
    WifiPhy::ChannelTuple chTuple{channelToSwitch.GetNumber(),
        channelToSwitch.GetWidth(),
        channelToSwitch.GetPhyBand(),
        channelToSwitch.GetPrimaryChannelIndex(20)};
    phy->SetOperatingChannel(chTuple);
}

void switchChannel(Ptr<WifiNetDevice> dev, uint16_t operatingChannel, WifiPhyBand band, uint16_t width) {
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



void
Scanner::scanChannel(std::vector<uint16_t>::iterator nextChanIt) {
    if (nextChanIt == channelsToScan.end()) {
        Simulator::ScheduleNow(&LCCSAlgo::Decide, this);
        return;
    }
    uint16_t channel = *nextChanIt;
    switchChannel(dev, channel);
    SIM_LOG_LOGIC("Scanning channel " << +channel);
    nextChanIt++;
    Simulator::Schedule(Seconds(channelDwellTime_s), &Scanner::returnToDataChannel, this, nextChanIt);
}

void
Scanner::returnToDataChannel(std::vector<uint16_t>::iterator nextChanIt) {
    switchChannel(dev, dataChannel);
    SIM_LOG_LOGIC("Returning to data channel " << +dataChannel);
    if (nextChanIt != channelsToScan.end() && *nextChanIt == dataChannel) {
        SIM_LOG_LOGIC("Skipping scan for data channel");
        nextChanIt++;
    }
    Simulator::Schedule(Seconds(scanInterval_s), &Scanner::scanChannel, this, nextChanIt);
}

void
Scanner::Scan() {
    Ptr<WifiPhy> phy = dev->GetPhy();
    dataChannel = phy->GetChannelNumber();
    Simulator::ScheduleNow(&Scanner::scanChannel, this, channelsToScan.begin());
}

const Ptr<WifiNetDevice>
Scanner::GetDevice() const {
    return dev;
}

const map<Mac48Address, Scanner::ScanData>&
Scanner::GetKnownAps() const {
    return knownAps;
}

const vector<uint16_t>&
Scanner::GetOperatingChannelsList() const {
    return operatingChannelsList;
}

const vector<uint16_t>&
Scanner::GetChannelsToScan() const {
    return channelsToScan;
}

void
Scanner::PrintScanResults() {
    cout << endl << "Scan data:" << endl;
    cout << "BSSID" << "\t\t\t" << "Channel" << "\t" << "SNR" << "\t\t" << "RSSI" << endl;
    for (auto& [bssid, scanData] : knownAps) {
        cout << bssid << "\t" << scanData.channel << "\t" << scanData.snr << "\t" << scanData.rssi << endl;
    }
}

uint16_t Scanner::getOperatingChannel() {
    return dev->GetPhy()->GetChannelNumber();
}

void LCCSAlgo::Decide(const Scanner* const scanner) {
    // least congested channel selection
    // for each possible channel, calculate channel metric: number of APs + number of clients
    // choose the channel with the lowest metric
    // if the metric is the same, choose the channel with the lowest number

    auto& knownAps = scanner->GetKnownAps();
    auto& operatingChannelsList = scanner->GetOperatingChannelsList();
    auto& channelsToScan = scanner->GetChannelsToScan();
    auto& dev = scanner->GetDevice();
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
    SIM_LOG_LOGIC("LCCS: switching to channel " << newChannel);
    switchChannel(dev, newChannel);
}

// since no additional parameters can be passed to the callback, we maintain
// a global map that Scanners can be accessed by the trace context
// (that contains info about device and corresponding scanner)
static map<string, std::shared_ptr<Scanner>> scannerByTraceContext;


class RRMGreedyAlgo {

    const double MinSignal = -100;
    const double MaxSignal = -25;

    std::set<Mac48Address> rrmGroup;
    std::map<Mac48Address, std::shared_ptr<Scanner>> devices;

    vector<uint16_t> channelsList;
    RRMGreedyAlgo(std::vector<std::shared_ptr<Scanner>> devs) {
        for (auto& dev : devs) {
            Mac48Address bssid = dev->GetDevice()->GetMac()->GetAddress();
            rrmGroup.insert(bssid);
            devices[bssid] = dev;
            channelsList = dev->GetOperatingChannelsList();
        }
    }
    std::map<Mac48Address, Scanner::ScanDataTable> scandata;
    using ScanData_t = std::map<Mac48Address, Scanner::ScanData>;

    struct IfaceAirData {
        Scanner::ScanDataTable signals;
        uint16_t channel = -1;
        double txDiff = 0.0;
        uint16_t width = 20;
    };

    using GroupState = std::map<Mac48Address, IfaceAirData>;

    GroupState PreprocessScanData() {
        GroupState groupState;
        for (auto& [bssid, scanData] : scandata) {
            IfaceAirData ifaceData;
            auto cpe = devices[bssid];
            ifaceData.channel = cpe->getOperatingChannel();
            ifaceData.signals = scanData;
            groupState[bssid] = ifaceData;
        }
        return groupState;
    }

    void RequestScandata() {
        for (auto& [bssid, dev] : devices) {
            dev->Scan();
            scandata[bssid] = dev->GetKnownAps();
        }
    }

    double ChannelInterference(uint16_t ch1, uint16_t ch2, int width=20) {
        // provides a theoretical measure of how much channels overlap,
        // with no respect to actual RF situation
        // 0 - no overlap, 1 - full overlap
        //
        double interf = 0.0;
        double diff = std::abs(ch1 - ch2);
        double add = std::max(ch1, ch2) < 36;
        if (diff >= ((double)width/5)+ add) {
            interf = 0.0;
        } else {
            interf = 1.0;
        }
        return interf;
    }

    double OnIfaceInterference(const Mac48Address& bssid, GroupState& groupState, uint16_t ifaceChannel) {
        double cumInterf = 0.0;
        auto iface = devices[bssid];


        for (auto& [otherBssid, scandataEntry] : groupState[bssid].signals) {
            if (otherBssid == bssid) {
                continue;
            }
            uint16_t otherChannel = scandataEntry.channel;
            double otherTxDiff = 0.0;
            if (groupState.count(otherBssid)) { // other AP belongs to RRM group
                otherTxDiff = groupState[otherBssid].txDiff;
                otherChannel = groupState[otherBssid].channel;
            }
            double ciScore = ChannelInterference(
                ifaceChannel,
                otherChannel
            );
            if (ciScore == 0.0) {
                continue;
            }
            double signal = scandataEntry.rssi + otherTxDiff;
            signal = std::min(signal, MaxSignal);
            signal = std::max(signal, MinSignal);
            signal = (signal - MinSignal) / (MaxSignal - MinSignal);
            // TODO: is it equivalent to initial RRMGreedy implementation?
            cumInterf += ciScore * signal;
        }
        // double interf = cumInnerInterf + cumOuterInterf;
        assert (cumInterf > 0.0);
        return cumInterf;
    }


    double GroupInterference(GroupState& groupState) {
        double totalInterf = 0.0;
        for (auto& [bssid, ifaceState] : groupState) {
            double onIfaceInterf = OnIfaceInterference(bssid, groupState,
                    ifaceState.channel);
            totalInterf += onIfaceInterf;
        }
        return totalInterf;
    }

    void updateAPsConfig(GroupState& groupState) {
        for (auto& [bssid, ifaceState] : groupState) {
            auto iface = devices[bssid];
            switchChannelv2(iface->GetDevice(), ifaceState.channel);
        }
    }

    void Decide() {
        RequestScandata();
        GroupState groupState = PreprocessScanData();
        double prevGroupInterf = 0;
        double groupInterf = GroupInterference(groupState);
        const double groupInterfEps = 0.001; // TODO elaborate the value

        // channel selection
        do {
            /*
             * for each device in the group, calculate the interference
             * caused by the group
             * choose the device with the lowest interference
             * if the interference is the same,
             * choose the device with the lowest number
            */
            for (auto& [bssid, ifaceState] : groupState) {
                double ifaceInterf = std::numeric_limits<double>::max();
                uint16_t minInterfChannel = ifaceState.channel;
                for (auto ch_i : channelsList) {
                    double interf = OnIfaceInterference(bssid, groupState, ch_i);
                    if (interf < ifaceInterf) {
                        ifaceInterf = interf;
                        minInterfChannel = ch_i;
                    }

                }
                ifaceState.channel = minInterfChannel;
            }
            // switch channel for the device with the lowest interference
            // and update the scan data
            prevGroupInterf = groupInterf;
            groupInterf = GroupInterference(groupState);
        } while ((groupInterf - prevGroupInterf) > groupInterfEps);
    }

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
    std::stringstream headerSs;
    hdr.Print(headerSs);
    if (hdr.IsQosData()) {
        headerSs << "QoS data! "
            << "RA: " << hdr.GetAddr1()
            << " TA: " << hdr.GetAddr2()
            << " DA: " << hdr.GetAddr3()
            << " SA: " << hdr.GetAddr4()
            << " FromDS: " << hdr.IsFromDs()
            << " ToDS: " << hdr.IsToDs()
            << " Retry: " << hdr.IsRetry()
            ;
    }
    SIM_LOG_DEBUG("Received frame: " << headerSs.str());
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
        SIM_LOG_DEBUG("Beacon from " << bssid << " on channel " << channel);
    } else if (hdr.IsData() && hdr.IsToDs()) { // client
        Mac48Address bssid = hdr.GetAddr1();
        Mac48Address client = hdr.GetAddr2();

        if (scanner->knownAps.count(bssid)) {
            scanner->knownAps[bssid].clients.insert(client);
        }
        SIM_LOG_DEBUG("Data from " << client
                        << " to " << bssid
                        << " on channel " << scanner->getOperatingChannel());
    }
}

Ptr<WifiNetDevice> getWifiNd (Ptr<Node> node, int idx) {
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

} // ns3 namespace

