#ifndef RRM_H
#define RRM_H

#include "ns3/core-module.h"
#include "ns3/node.h"
#include "ns3/wifi-net-device.h"
#include <iomanip>

#define SIM_LOG_LOGIC(x) do { NS_LOG_LOGIC(std::setprecision(7) << Simulator::Now().GetSeconds() << "s: " << x ); } while(0)
#define SIM_LOG_DEBUG(x) do { NS_LOG_DEBUG(std::setprecision(7) << Simulator::Now().GetSeconds() << "s: " << x ); } while(0)


namespace ns3 {
using std::vector, std::map, std::set, std::cout, std::endl, std::string;


// inline Ptr<WifiNetDevice> getWifiNd (Ptr<Node> node, int idx=0) {
//     return DynamicCast<WifiNetDevice>(node->GetDevice(idx));
// };

Ptr<WifiNetDevice> getWifiNd (Ptr<Node> node, int idx=0);

string
assembleChannelSettings(uint16_t channel, uint16_t width, string band);

void
switchChannelv2(Ptr<WifiNetDevice> dev, uint16_t newOperatingChannel,
        WifiPhyBand band=WIFI_PHY_BAND_2_4GHZ, uint16_t width=20);

void
switchChannel(Ptr<WifiNetDevice> dev, uint16_t operatingChannel,
        WifiPhyBand band=WIFI_PHY_BAND_2_4GHZ, uint16_t width=20);


class Scanner {
private:
    const double channelDwellTime_s = 0.2;
    const double scanInterval_s = 1.0;
    int dataChannel;
    Ptr<WifiNetDevice> dev;
    vector<uint16_t> channelsToScan;
    vector<uint16_t> operatingChannelsList;
    // bool isScanning = false;

    void scanChannel(std::vector<uint16_t>::iterator nextChanIt);
    void returnToDataChannel(std::vector<uint16_t>::iterator nextChanIt);
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

    using ScanDataTable = std::map<Mac48Address, ScanData>;
    ScanDataTable knownAps;

    Scanner(Ptr<WifiNetDevice> wifidev,
            vector<uint16_t> opChannelsList) : dev(wifidev),
                                                channelsToScan(opChannelsList),
                                                operatingChannelsList(opChannelsList) {}
    ~Scanner() {}

    void Scan();

    const Ptr<WifiNetDevice> GetDevice() const;

    const map<Mac48Address, ScanData>& GetKnownAps() const;

    const vector<uint16_t>& GetOperatingChannelsList() const;

    const vector<uint16_t>& GetChannelsToScan() const;

    void PrintScanResults();
    uint16_t getOperatingChannel();
};

std::shared_ptr<Scanner>
CreateScannerForNode(Ptr<Node> scannerWifiNode, vector<uint16_t> operatingChannels);


class LCCSAlgo {
public:
    LCCSAlgo() = delete;
    static void Decide(const Scanner* const scanner);
};


} // namespace ns3

#endif // RRM_H
