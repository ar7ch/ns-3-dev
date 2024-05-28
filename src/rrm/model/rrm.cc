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
#include "rrm.h"


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

string
assembleChannelSettings(uint16_t channel, uint16_t width, string band) {
    assert(band == "BAND_2_4GHZ" || band == "BAND_5GHZ");
    std::stringstream ss;
    ss << "{"
        << std::to_string(channel) << ", "
        << std::to_string(width) << ", "
        << band
        << ", " << "0" << "}";
    return ss.str();
}

void
setTxPower_attr(Ptr<WifiNetDevice> dev, double txPowerDbm) {
    Ptr<WifiPhy> phy = dev->GetPhy();
    phy->SetTxPowerStart(txPowerDbm);
    phy->SetTxPowerEnd(txPowerDbm);
}

void
switchChannel_event(Ptr<WifiNetDevice> dev, uint16_t newOperatingChannel, WifiPhyBand band, uint16_t width) {
    Ptr<WifiPhy> phy = dev->GetPhy();
    if (phy->IsStateSleep())
    {
        phy->ResumeFromSleep();
    }
    if (phy->IsStateSwitching()){
        SIM_LOG_DEBUG("AP channel switch is in progress, postpone the switch");
        Simulator::Schedule(Seconds(0.1), &switchChannel_event, dev, newOperatingChannel, band, width);
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

void
switchChannel_attr(Ptr<WifiNetDevice> dev, uint16_t operatingChannel, WifiPhyBand band, uint16_t width) {
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

Ptr<WifiNetDevice>
getWifiNd (Ptr<Node> node, int idx) {
    return DynamicCast<WifiNetDevice>(node->GetDevice(idx));
};

void Scanner::endScan() {
    state = ScanState::AP_MODE_SCAN_COMPLETE;
    scanDataTimestamp = Simulator::Now().GetSeconds();
    // Simulator::ScheduleNow(&LCCSAlgo::Decide, this);
    if (afterScanCallback_) {
        afterScanCallback_->invoke();
    }
}

void
Scanner::scanChannel(std::vector<uint16_t>::iterator nextChanIt) {
    if (nextChanIt == channelsToScan.end()) {
        SIM_LOG_LOGIC(id_ << ": " << "Scan complete");
        endScan();
        return;
    }
    uint16_t channel = *nextChanIt;
    switchChannel_attr(dev, channel);
    state = ScanState::SCAN_IN_PROGRESS_MON_MODE;
    SIM_LOG_LOGIC(id_ << ": " << "Scanning channel " << +channel);
    nextChanIt++;
    Simulator::Schedule(Seconds(channelDwellTime_s), &Scanner::returnToDataChannel, this, nextChanIt);
}

void
Scanner::returnToDataChannel(std::vector<uint16_t>::iterator nextChanIt) {
    switchChannel_attr(dev, dataChannel);
    state = ScanState::SCAN_IN_PROCESSS_AP_MODE;
    SIM_LOG_LOGIC(id_ << ": " << "Returning to data channel " << +dataChannel);
    if (nextChanIt != channelsToScan.end() && *nextChanIt == dataChannel) {
        SIM_LOG_LOGIC(id_ << ": " << "Skipping scan for data channel");
        nextChanIt++;
    }
    Simulator::Schedule(Seconds(scanInterval_s), &Scanner::scanChannel, this, nextChanIt);
}

void
Scanner::Scan() {
    assert(state == ScanState::AP_MODE_NO_SCANDATA || state == ScanState::AP_MODE_SCAN_COMPLETE);
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
    cout << endl << id_ << ": " << "Scan data:" << endl;
    cout << "BSSID" << "\t\t\t" << "Channel" << "\t" << "SNR" << "\t\t" << "RSSI" << "\t" << "Clients" << endl;
    for (auto& [bssid, scanData] : knownAps) {
        cout << bssid << "\t"
             << scanData.channel << "\t"
             << scanData.snr << "\t"
             << scanData.rssi << "\t"
             << scanData.clients.size()
             << endl;
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
    switchChannel_attr(dev, newChannel);
}

// since no additional parameters can be passed to the callback, we maintain
// a global map that Scanners can be accessed by the trace context
// (that contains info about device and corresponding scanner)
static map<string, std::shared_ptr<Scanner>> scannerByTraceContext;



void
monitorSniffer(
        std::string context, Ptr<const Packet> p,
        uint16_t channelFreqMhz,
        WifiTxVector txVector,
        MpduInfo aMpdu,
        SignalNoiseDbm signalNoise,
        uint16_t staId) {

    std::shared_ptr<Scanner> scanner = scannerByTraceContext[context];
    if (!scanner->inMonitorMode()) {
        SIM_LOG_DEBUG("Not in monitor mode, ignoring the frame");
        return;
    } else {
        SIM_LOG_DEBUG("Received frame in monitor mode");
    }
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

std::shared_ptr<Scanner>
CreateScannerForNode(Ptr<Node> scannerWifiNode, vector<uint16_t> operatingChannels, std::string id) {

    Ptr<WifiNetDevice> scannerWifiNetDev = getWifiNd(scannerWifiNode);
    std::stringstream ss;
    ss << "/NodeList/" << scannerWifiNode->GetId()
        << "/DeviceList/" << scannerWifiNetDev->GetIfIndex()
        << "/$ns3::WifiNetDevice/Phy/MonitorSnifferRx";
    string scanApTraceStr = ss.str();
    std::shared_ptr<Scanner> scanner = std::make_shared<Scanner>(scannerWifiNetDev, operatingChannels, id);
    scannerByTraceContext[scanApTraceStr] = scanner;
    Config::Connect(scanApTraceStr, MakeCallback(&monitorSniffer));
    return scanner;
}

template<typename ReturnType, typename... Args>
std::shared_ptr<Scanner>
CreateScannerForNode(Ptr<Node> scannerWifiNode, vector<uint16_t> operatingChannels,
        std::function<ReturnType(Args...)> callback, Args... args) {

    Ptr<WifiNetDevice> scannerWifiNetDev = getWifiNd(scannerWifiNode);
    std::stringstream ss;
    ss << "/NodeList/" << scannerWifiNode->GetId()
        << "/DeviceList/" << scannerWifiNetDev->GetIfIndex()
        << "/$ns3::WifiNetDevice/Phy/MonitorSnifferRx";
    string scanApTraceStr = ss.str();
    std::shared_ptr<Scanner> scanner = std::make_shared<Scanner>(scannerWifiNetDev, operatingChannels, callback, args...);
    scannerByTraceContext[scanApTraceStr] = scanner;
    Config::Connect(scanApTraceStr, MakeCallback(&monitorSniffer));
    return scanner;
}

RRMGreedyAlgo::GroupState
RRMGreedyAlgo::PreprocessScanData() {
    RRMGreedyAlgo::GroupState groupState;
    for (auto& [bssid, scanData] : scandata) {
        RRMGreedyAlgo::IfaceAirData ifaceData;
        if (rrmGroup.count(bssid) == 0) {
            NS_LOG_ERROR("bssid " << bssid << " is not in the RRM group");
            assert(false);
        }
        auto cpe = devices[bssid];
        ifaceData.txPowerDbm = cpe->GetDevice()->GetPhy()->GetTxPowerEnd();
        ifaceData.txDiff = MaxTxPower_dbm - ifaceData.txPowerDbm;
        assert(
            cpe->GetDevice()->GetPhy()->GetTxPowerEnd() == cpe->GetDevice()->GetPhy()->GetTxPowerStart()
        );
        ifaceData.channel = cpe->getOperatingChannel();
        ifaceData.signals = scanData;
        groupState[bssid] = ifaceData;
    }
    return groupState;
}

void
RRMGreedyAlgo::RequestScandata() {
    for (auto& [bssid, dev] : devices) {
        if (dev->state == Scanner::ScanState::AP_MODE_SCAN_COMPLETE) {
            if (scandataTimestamp.count(bssid) == 0) { // no scandata uploaded to RRMGreedy so far
                SIM_LOG_LOGIC("bssid " << bssid << ": no scandata yet");
                assert(false);
            }
            if (!isScanDataStale(bssid)) {
                SIM_LOG_LOGIC("bssid " << bssid << ": using earlier reported scandata, with timestamp "
                        << dev->scanDataTimestamp);
            } else if (isScanDataStale(bssid) && !isScanDataStale(dev->scanDataTimestamp)) {
                SIM_LOG_LOGIC("bssid " << bssid << ": using newer scandata from the device, with timestamp "
                        << dev->scanDataTimestamp);
                scandata[bssid] = dev->GetKnownAps();
                scandataTimestamp[bssid] = dev->scanDataTimestamp;
            } else {
                SIM_LOG_LOGIC("bssid " << bssid << ": scandata is too old");
                assert(false && "scandata is too old");
            }
        } else {
            assert(false && "scandata is not ready");
        }
    }
}

double
RRMGreedyAlgo::ChannelInterference(uint16_t ch1, uint16_t ch2, int width) {
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

double
RRMGreedyAlgo::OnIfaceInterference(const Mac48Address& bssid, GroupState& groupState, uint16_t ifaceChannel) {
    double cumInterf = 0.0;
    auto iface = devices[bssid];
    NS_LOG_DEBUG("-- Calculating onInterf for " << bssid << " on channel " << ifaceChannel);

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
        // NS_LOG_LOGIC(" --- Channel interference score: " << ciScore);
        if (ciScore == 0.0) {
            continue;
        }
        double signal = scandataEntry.rssi + otherTxDiff;
        signal = std::min(signal, MaxRSSI_dbm);
        signal = std::max(signal, MinRSSI_dbm);
        signal = (signal - MinRSSI_dbm) / (MaxRSSI_dbm - MinRSSI_dbm);
        // NS_LOG_LOGIC(" ------- Normalized RSSI: " << signal);
        // TODO: is it equivalent to initial RRMGreedy implementation?
        cumInterf += ciScore * signal;
        NS_LOG_DEBUG("--- Scandata entry: " << otherBssid
                << ", ch: " << scandataEntry.channel
                << ", rssi: " << scandataEntry.rssi << " (norm=" << signal << ")"
                << ", type: " << (groupState.count(otherBssid) ? "Inner" : "Outer")
                << ", ciScore: " << ciScore
                );
    }
    // double interf = cumInnerInterf + cumOuterInterf;
    // assert (cumInterf > 0.0);
    NS_LOG_DEBUG("-- Total onInterf for " << bssid << ": " << cumInterf);
    return cumInterf;
}

std::pair<double, double>
RRMGreedyAlgo::FromIfaceInterference(const Mac48Address& bssid, GroupState& groupState) {

    double cumInterf = 0.0;
    double maxSignal = MinRSSI_dbm;
    for (auto& [otherBssid, otherIfaceData] : groupState) {
        if (rrmGroup.count(otherBssid) == 0) {
            NS_LOG_ERROR("bssid " << otherBssid << " is not in the RRM group");
            assert(false);
        }
        if (otherBssid == bssid) {
            continue;
        }
        if (otherIfaceData.signals.count(bssid) == 0) {
            continue;
        }
        auto entryAboutIface = otherIfaceData.signals[bssid];
        double ciScore = ChannelInterference(
                otherIfaceData.channel,
                groupState[bssid].channel
                );
        if (ciScore == 0.0) {
            continue;
        }
        double signal = entryAboutIface.rssi;
        maxSignal = std::max(maxSignal, signal);
        signal += groupState[bssid].txDiff;
        signal = std::min(signal, MaxRSSI_dbm);
        signal = std::max(signal, MinRSSI_dbm);
        signal = (signal - MinRSSI_dbm) / (MaxRSSI_dbm - MinRSSI_dbm);
        NS_LOG_LOGIC(" --- Signal from " << bssid << " at " << otherBssid << ": " << signal);
        // there should be a bug from the original RRMGreedy, when signals from the same APs get added like they are distinct. currently, Scanner implementation only remembers signal measurement data from the last frame received by an AP
        cumInterf += ciScore * signal;
    }
    return std::make_pair(cumInterf, maxSignal);
}


double
RRMGreedyAlgo::GroupInterference(GroupState& groupState) {
    double totalInterf = 0.0;
    for (auto& [bssid, ifaceState] : groupState) {
        double onIfaceInterf = OnIfaceInterference(bssid, groupState,
                ifaceState.channel);
        totalInterf += onIfaceInterf;
    }
    return totalInterf;
}

void
RRMGreedyAlgo::updateAPsConfig(GroupState& groupState) {
    for (auto& [bssid, ifaceState] : groupState) {
        auto iface = devices[bssid];
        switchChannel_event(iface->GetDevice(), ifaceState.channel);
        // iface->GetDevice()->GetPhy()->SetTxPowerStart(ifaceState.txPowerDbm + ifaceState.txDiff);
        // iface->GetDevice()->GetPhy()->SetTxPowerEnd(ifaceState.txPowerDbm + ifaceState.txDiff);
    }
}

void
RRMGreedyAlgo::updateRrmResults(GroupState& groupState) {
    for (auto& [bssid, ifaceData] : groupState) {
        rrmResults[bssid] = {ifaceData.channel, ifaceData.txPowerDbm + ifaceData.txDiff};
        NS_LOG_LOGIC("RRM result for " << bssid << ": ch=" << rrmResults[bssid].first << " txp=" << rrmResults[bssid].second);
    }
}

void
RRMGreedyAlgo::Decide() {
    SIM_LOG_LOGIC("=== Running RRMGreedy algorithm ===");

    SIM_LOG_LOGIC("1. Requesting scan data");
    RequestScandata();

    SIM_LOG_LOGIC("2. Preprocessing scan data");
    GroupState groupState = PreprocessScanData();
    SIM_LOG_LOGIC("Initial group state:");
    PrintGroupState(groupState);

    SIM_LOG_LOGIC("3. Calculating initial group interference");
    double prevGroupInterf = 0;
    double groupInterf = GroupInterference(groupState);
    const double groupInterfEps = 0.001; // TODO elaborate the value

    SIM_LOG_LOGIC("- Initial group interference: " << groupInterf);

    SIM_LOG_LOGIC("4. Starting iterative greedy channel selection");
    // channel selection
    // while ((groupInterf - prevGroupInterf) > groupInterfEps);
    constexpr int MAX_ITER = 1000;
    for (int i = 1; i < MAX_ITER; i++) {
        /*
         * for each device in the group, calculate the interference
         * caused by the group
         * choose the device with the lowest interference
         * if the interference is the same,
         * choose the device with the lowest number
         */
        for (auto& [bssid, ifaceState] : groupState) {
            NS_LOG_LOGIC("- Calculating interference for " << bssid);
            double ifaceInterf = std::numeric_limits<double>::max();
            uint16_t ifaceInitialChannel = ifaceState.channel;
            uint16_t minInterfChannel = ifaceInitialChannel;
            for (auto ch_i : channelsList) {
                double interf = OnIfaceInterference(bssid, groupState, ch_i);
                NS_LOG_LOGIC("- Channel " << ch_i << " interference: " << interf);
                if (interf < ifaceInterf) {
                    ifaceInterf = interf;
                    minInterfChannel = ch_i;
                }

            }
            NS_LOG_LOGIC(
                    "- " << bssid
                    << ": minimal interference: "
                    << ifaceInterf
                    << " on channel " << minInterfChannel
                    << endl);
            ifaceState.channel = minInterfChannel;
            NS_LOG_LOGIC(bssid << ": channel switch "
                    << ifaceInitialChannel << " -> " << minInterfChannel);
        }
        // switch channel for the device with the lowest interference
        // and update the scan data
        prevGroupInterf = groupInterf;
        groupInterf = GroupInterference(groupState);
        NS_LOG_LOGIC("== ACS iteration=" << std::to_string(i)
                << ", group interference: " << prevGroupInterf << " -> " << groupInterf
                << " ==");
        if ((prevGroupInterf - groupInterf) < groupInterfEps) {
            NS_LOG_LOGIC("ACS: no significant improvement, stopping");
            break;
        }
    }
    // TPC
    bool managePower = true;
    bool alreadyPrintedIad = false;
    prevGroupInterf = groupInterf;
    if (managePower) {
        NS_LOG_LOGIC("5. Starting TPC");
        for (int i = 0; managePower && i < MAX_ITER; i++) {
            Mac48Address worstIface;
            double worstInterf = 0.0;
            double worstSignal = 0.0;
            // find the worst interface
            for (auto& [bssid, ifaceState] : groupState) {
                if (!alreadyPrintedIad) {
                    PrintIfaceAirData(groupState, bssid);
                    alreadyPrintedIad = true;
                }
                auto [ifaceInterf, ifaceSignal] = FromIfaceInterference(bssid, groupState);
                if (ifaceInterf > worstInterf) {
                    worstInterf = ifaceInterf;
                    worstSignal = ifaceSignal;
                    worstIface = bssid;
                }
            }
            if (worstInterf == 0.0) {
                NS_LOG_LOGIC("TPC: no interference, stopping");
                break;
            }
            NS_LOG_LOGIC("TPC: worst interface: " << worstIface <<
                    " interference=" << worstInterf <<
                    ", signal=" << worstSignal);
            // reduce power of the worst interface
            double oldTxDiff = groupState[worstIface].txDiff;
            double needDiff = MinRSSI_dbm - worstSignal;
            double needPower = groupState[worstIface].txPowerDbm + needDiff/2.0;
            double newTxDiff = needPower - groupState[worstIface].txPowerDbm;
            if (newTxDiff + groupState[worstIface].txPowerDbm < MinTxPower_dbm) { // FIXME
                NS_LOG_LOGIC("TPC: worst txPower is minimal");
                newTxDiff = MinTxPower_dbm - groupState[worstIface].txPowerDbm;
            }
            groupState[worstIface].txDiff = newTxDiff;
            NS_LOG_LOGIC("TPC: " << "txdiff " << oldTxDiff << " -> " << newTxDiff
                    << " (txpower "
                    << groupState[worstIface].txPowerDbm + oldTxDiff << " -> " << groupState[worstIface].txPowerDbm + newTxDiff << ")"
                    );
            groupInterf = GroupInterference(groupState);
            NS_LOG_LOGIC("== TPC " << "iteration " << i << ": group interference "
                    << prevGroupInterf << " -> " << groupInterf);
            if ((prevGroupInterf - groupInterf) < groupInterfEps) {
                NS_LOG_LOGIC("TPC: no significant improvement, stopping");
                break;
            }
            prevGroupInterf = groupInterf;
        }
    }

    NS_LOG_LOGIC(
            "=====" << endl
            << "Ending with group interference=" << groupInterf
            << " prevGroupInterf=" << prevGroupInterf
            << " eps=" << groupInterfEps);
    NS_LOG_LOGIC("New group state:");
    PrintGroupState(groupState);
    NS_LOG_LOGIC("=====");
    NS_LOG_LOGIC("6. Updating APs configuration DISABLED, NO CHANGES APPLIED");
    // updateAPsConfig(groupState);
    updateRrmResults(groupState);
}

void RRMGreedyAlgo::AddApScandata(const Scanner *scanner) {
    Mac48Address bssid = scanner->GetDevice()->GetMac()->GetAddress();
    auto dev = devices[bssid];
    if (!isScanDataStale(dev->scanDataTimestamp)) {
        SIM_LOG_LOGIC(
                "bssid " << bssid <<
                ": using scandata from timestamp " << dev->scanDataTimestamp);
        scandata[bssid] = dev->GetKnownAps();
        scandataTimestamp[bssid] = dev->scanDataTimestamp;
    } else {
        SIM_LOG_LOGIC("bssid " << bssid << ": scandata is too old");
        assert(false && "scandata is too old");
    }
    scandata[bssid] = scanner->GetKnownAps();
    scandataTimestamp[bssid] = scanner->scanDataTimestamp;
}

void RRMGreedyAlgo::AddApScandata_s(RRMGreedyAlgo *rrmgreedy, Scanner *scanner) {
    assert (rrmgreedy != nullptr && "rrmgreedy is nullptr");
    assert (scanner != nullptr && "scanner is nullptr");
    Mac48Address bssid = scanner->GetDevice()->GetMac()->GetAddress();
    assert (rrmgreedy->devices.count(bssid) > 0 && "bssid not found in devices");
    auto dev = rrmgreedy->devices[bssid];
    if (!rrmgreedy->isScanDataStale(dev->scanDataTimestamp)) {
        SIM_LOG_LOGIC(
                "bssid " << bssid <<
                ": using scandata from timestamp " << dev->scanDataTimestamp);
        rrmgreedy->scandata[bssid] = dev->GetKnownAps();
        rrmgreedy->scandataTimestamp[bssid] = dev->scanDataTimestamp;
    } else {
        SIM_LOG_LOGIC("bssid " << bssid << ": scandata is too old");
        assert(false && "scandata is too old");
    }
    rrmgreedy->scandata[bssid] = scanner->GetKnownAps();
    rrmgreedy->scandataTimestamp[bssid] = scanner->scanDataTimestamp;
}

RRMGreedyAlgo::RRMGreedyAlgo(std::vector<std::shared_ptr<Scanner>>& devs, vector<uint16_t> channels) : channelsList(channels) {
    for (auto& dev : devs) {
        Mac48Address bssid = dev->GetDevice()->GetMac()->GetAddress();
        rrmGroup.insert(bssid);
        devices[bssid] = dev;
        // channelsList = dev->GetOperatingChannelsList();
    }
}

void
RRMGreedyAlgo::PrintGroupState(GroupState& groupState) {
    NS_LOG_LOGIC("BSSID\t\t\tChannel\t\tTxPower (dBm)");
    for (auto& [bssid, ifaceData] : groupState) {
        NS_LOG_LOGIC(bssid
                << "\t" << ifaceData.channel
                << "\t\t" << ifaceData.txPowerDbm + ifaceData.txDiff);
    }
}

void
RRMGreedyAlgo::PrintIfaceAirData(GroupState& groupState, Mac48Address bssid) {

    IfaceAirData& iad = groupState.at(bssid);
    NS_LOG_LOGIC("InterfaceAirData for " << bssid << " ("
            << "ch=" << iad.channel << ", "
            << "txp=" << iad.txPowerDbm << "dBm" << ", "
            << "txdiff=" << iad.txDiff
            << ")"
            );
    NS_LOG_LOGIC("BSSID\t\t\tCH\tRSSI\t\tSNR");
    NS_LOG_LOGIC("----------------------------------------------------------");
    for (auto& [otherBssid, otherEnt] : iad.signals) {
        int otherCh = otherEnt.channel;
        if (rrmGroup.count(otherBssid) > 0) {
            otherCh = groupState[otherBssid].channel;
        }
        NS_LOG_LOGIC(
                otherBssid
                << "\t"
                << (otherCh == otherEnt.channel ?
                    std::to_string(otherCh) : std::to_string(otherEnt.channel) + "->" + std::to_string(otherCh))
                << "\t"
                << otherEnt.rssi
                << "\t"
                << otherEnt.snr
                );
    }
    NS_LOG_LOGIC("==========================================================");
}

} // ns3 namespace

