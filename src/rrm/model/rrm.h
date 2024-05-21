#ifndef RRM_H
#define RRM_H

#include "ns3/core-module.h"
#include "ns3/node.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac.h"
#include <iostream>
#include <functional>
#include <tuple>
#include <iomanip>

#define SIM_LOG_LOGIC(x) do { NS_LOG_LOGIC(std::fixed << std::setprecision(7) << Simulator::Now().GetSeconds() << "s: " << x ); } while(0)
#define SIM_LOG_DEBUG(x) do { NS_LOG_DEBUG(std::fixed << std::setprecision(7) << Simulator::Now().GetSeconds() << "s: " << x ); } while(0)



namespace ns3 {


#define IN_MAP(_map, _key) (_map.count(_key) > 0)

using std::vector, std::map, std::set, std::cout, std::endl, std::string;

class ICallbackHolder {
public:
    virtual ~ICallbackHolder() = default;
    virtual void invoke() = 0;
};

template<typename ReturnType, typename... Args>
class CallbackHolder : public ICallbackHolder {
private:
    std::function<ReturnType(Args...)> callback_;
    std::tuple<Args...> args_;

public:
    CallbackHolder(std::function<ReturnType(Args...)> callback, Args... args)
    : callback_(std::move(callback)), args_(std::make_tuple(std::forward<Args>(args)...)) {}

    void invoke() override {
        std::apply(callback_, args_);
    }
};


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

    void scanChannel(std::vector<uint16_t>::iterator nextChanIt);
    void endScan();
    void returnToDataChannel(std::vector<uint16_t>::iterator nextChanIt);
    std::unique_ptr<ICallbackHolder> afterScanCallback_ = nullptr;

public:
    enum class ScanState {
        AP_MODE_NO_SCANDATA,            // No scan yet performed
        SCAN_IN_PROGRESS_MON_MODE,
        SCAN_IN_PROCESSS_AP_MODE,
        AP_MODE_SCAN_COMPLETE,
    };
    ScanState state = ScanState::AP_MODE_NO_SCANDATA;

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
    double scanDataTimestamp = 0.0;
    std::string id_;

    Scanner(Ptr<WifiNetDevice> wifidev, vector<uint16_t> opChannelsList, std::string id="") :
        dev(wifidev),
        channelsToScan(opChannelsList),
        operatingChannelsList(opChannelsList),
        id_(id)
    {
        auto defaultCallback = [this]() {
            this->PrintScanResults();
        };
        afterScanCallback_ = std::make_unique<CallbackHolder<void>>(
                defaultCallback);
    }

    template<typename ReturnType, typename... Args>
    Scanner(Ptr<WifiNetDevice> wifidev,
            vector<uint16_t> opChannelsList,
            std::function<ReturnType(Args...)> callback,
            Args... args) : dev(wifidev),
                            channelsToScan(opChannelsList),
                            operatingChannelsList(opChannelsList)
    {
        afterScanCallback_ =
            std::make_unique<CallbackHolder<ReturnType, Args...>>(
                    callback, args...);
    }
    ~Scanner() {}

    void Scan();

    bool inMonitorMode() const {
        return state == ScanState::SCAN_IN_PROGRESS_MON_MODE;
    }

    bool hasScanData() const {
        return state == ScanState::AP_MODE_SCAN_COMPLETE;
    }

    template<typename ReturnType, typename... Args>
    void setAfterScanCallback(std::function<ReturnType(Args...)> callback,
            Args... args) {
        afterScanCallback_ = std::make_unique<CallbackHolder<ReturnType, Args...>>(
                callback, args...);
    }

    const Ptr<WifiNetDevice> GetDevice() const;

    const map<Mac48Address, ScanData>& GetKnownAps() const;

    const vector<uint16_t>& GetOperatingChannelsList() const;

    const vector<uint16_t>& GetChannelsToScan() const;

    void PrintScanResults();
    uint16_t getOperatingChannel();
};

std::shared_ptr<Scanner>
CreateScannerForNode(Ptr<Node> scannerWifiNode, vector<uint16_t> operatingChannels, std::string id="");


class LCCSAlgo {
public:
    LCCSAlgo() = delete;
    static void Decide(const Scanner* const scanner);
};

class RRMGreedyAlgo {

    const double MinRSSI_dbm = -100;
    const double MaxRSSI_dbm = -25;
    const double scandataStaleTime_s = 5.0;
    const double MaxTxPower_dbm = 30.0;
    const double MinTxPower_dbm = 10.0;

    std::vector<uint16_t> channelsList; // assumed to be the same in the group

    std::set<Mac48Address> rrmGroup;
    std::map<Mac48Address, std::shared_ptr<Scanner>> devices;

    std::map<Mac48Address, Scanner::ScanDataTable> scandata;
    std::map<Mac48Address, double> scandataTimestamp;
    using ScanData_t = std::map<Mac48Address, Scanner::ScanData>;

    struct IfaceAirData {
        Scanner::ScanDataTable signals;
        uint16_t channel = -1;
        double txPowerDbm = 0.0;
        double txDiff = 0.0;
        uint16_t width = 20;
    };

    using GroupState = std::map<Mac48Address, IfaceAirData>;

    GroupState PreprocessScanData();

    void RequestScandata();

    double ChannelInterference(uint16_t ch1, uint16_t ch2, int width=20);

    double OnIfaceInterference(const Mac48Address& bssid, GroupState& groupState,
            uint16_t ifaceChannel);

    std::pair<double, double> FromIfaceInterference(const Mac48Address& bssid,
            GroupState& groupState);

    double GroupInterference(GroupState& groupState);

    void updateAPsConfig(GroupState& groupState);

    bool isScanDataStale(const Mac48Address& bssid) const {
        return isScanDataStale(scandataTimestamp.at(bssid));
    }

    bool isScanDataStale(double scandataTimestamp) const {
        return (Simulator::Now().GetSeconds() - scandataTimestamp) > scandataStaleTime_s;
    }
    void PrintGroupState(GroupState& groupState);
    // void PrintScanData(Scanner::ScanData& scandata);
    void PrintIfaceAirData(GroupState& groupState, Mac48Address bssid);

    public:

    void Decide();

    void AddApScandata(const Scanner *scanner);
    static void AddApScandata_s(RRMGreedyAlgo *rrmgreedy, Scanner *scanner);

    void AddDevice(std::shared_ptr<Scanner> dev) {
        Mac48Address bssid = dev->GetDevice()->GetMac()->GetAddress();
        rrmGroup.insert(bssid);
        devices[bssid] = dev;
    }

    void AddDevices(vector<std::shared_ptr<Scanner>>& dev) {
        for (auto& d : dev) {
            AddDevice(d);
        }
    }

    RRMGreedyAlgo(std::vector<std::shared_ptr<Scanner>>& devs, vector<uint16_t> channels);
    RRMGreedyAlgo(vector<uint16_t> channels) : channelsList(channels) {}
};


} // namespace ns3

#endif // RRM_H
