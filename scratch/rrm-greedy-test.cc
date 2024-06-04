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
#include "ns3/gnuplot-helper.h"

#include "ns3/wifi-net-device.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/net-device.h"
#include "ns3/rrm.h"
#include <cassert>
// #include <random>
#include <iomanip>
#include <string>
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

static const uint32_t g_packetSize = 1024;
static const double g_packetInterval = 0.005;


class NodeStatistics
{
  public:
    /**
     * \brief Constructor.
     *
     * \param aps Access points
     * \param stas WiFi Stations.
     */
    NodeStatistics(NetDeviceContainer aps, NetDeviceContainer stas);

    /**
     * \brief Collects the statistics at a given time.
     *
     * \param time Time at which the statistics are collected.
     */
    void CheckStatistics(double time);

    /**
     * \brief Callback called by WifiNetDevice/Phy/PhyTxBegin.
     *
     * \param path The trace path.
     * \param packet The sent packet.
     * \param powerW The Tx power.
     */
    void PhyCallback(std::string path, Ptr<const Packet> packet, double powerW);
    /**
     * \brief Callback called by PacketSink/Rx.
     *
     * \param path The trace path.
     * \param packet The received packet.
     * \param from The sender address.
     */
    void RxCallback(std::string path, Ptr<const Packet> packet, const Address& from);
    /**
     * \brief Callback called by WifiNetDevice/RemoteStationManager/x/PowerChange.
     *
     * \param path The trace path.
     * \param oldPower Old Tx power.
     * \param newPower Actual Tx power.
     * \param dest Destination of the transmission.
     */
    void PowerCallback(std::string path, double oldPower, double newPower, Mac48Address dest);
    /**
     * \brief Callback called by WifiNetDevice/RemoteStationManager/x/RateChange.
     *
     * \param path The trace path.
     * \param oldRate Old rate.
     * \param newRate Actual rate.
     * \param dest Destination of the transmission.
     */
    void RateCallback(std::string path, DataRate oldRate, DataRate newRate, Mac48Address dest);
    /**
     * \brief Callback called by YansWifiPhy/State/State.
     *
     * \param path The trace path.
     * \param init Time when the state started.
     * \param duration Amount of time we've been in (or will be in) the state.
     * \param state The state.
     */
    void StateCallback(std::string path, Time init, Time duration, WifiPhyState state);

    /**
     * \brief Get the Throughput output data
     *
     * \return the Throughput output data.
     */
    Gnuplot2dDataset GetDatafile();
    /**
     * \brief Get the Power output data.
     *
     * \return the Power output data.
     */
    Gnuplot2dDataset GetPowerDatafile();
    /**
     * \brief Get the IDLE state output data.
     *
     * \return the IDLE state output data.
     */
    Gnuplot2dDataset GetIdleDatafile();
    /**
     * \brief Get the BUSY state output data.
     *
     * \return the BUSY state output data.
     */
    Gnuplot2dDataset GetBusyDatafile();
    /**
     * \brief Get the TX state output data.
     *
     * \return the TX state output data.
     */
    Gnuplot2dDataset GetTxDatafile();
    /**
     * \brief Get the RX state output data.
     *
     * \return the RX state output data.
     */
    Gnuplot2dDataset GetRxDatafile();

    /**
     * \brief Get the Busy time.
     *
     * \return the busy time.
     */
    double GetBusyTime() const;

  private:
    /// Time, DataRate pair vector.
    typedef std::vector<std::pair<Time, DataRate>> TxTime;
    /**
     * \brief Setup the WifiPhy object.
     *
     * \param phy The WifiPhy to setup.
     */
    void SetupPhy(Ptr<WifiPhy> phy);
    /**
     * \brief Get the time at which a given datarate has been recorded.
     *
     * \param rate The datarate to search.
     * \return the time.
     */
    Time GetCalcTxTime(DataRate rate);

    std::map<Mac48Address, double> m_currentPower;  //!< Current Tx power for each sender.
    std::map<Mac48Address, DataRate> m_currentRate; //!< Current Tx rate for each sender.
    uint32_t m_bytesTotal;                          //!< Number of received bytes.
    double m_totalEnergy;                           //!< Energy used.
    double m_totalTime;                             //!< Time spent.
    double busyTime;                                //!< BUSY time.
    double idleTime;                                //!< IDLE time.
    double txTime;                                  //!< TX time.
    double rxTime;                                  //!< RX time.
    double m_totalBusyTime;                         //!< Total time in BUSY state.
    double m_totalIdleTime;                         //!< Total time in IDLE state.
    double m_totalTxTime;                           //!< Total time in TX state.
    double m_totalRxTime;                           //!< Total time in RX state.
    TxTime m_timeTable;                             //!< Time, DataRate table.
    Gnuplot2dDataset m_output;                      //!< Throughput output data.
    Gnuplot2dDataset m_output_power;                //!< Power output data.
    Gnuplot2dDataset m_output_idle;                 //!< IDLE output data.
    Gnuplot2dDataset m_output_busy;                 //!< BUSY output data.
    Gnuplot2dDataset m_output_rx;                   //!< RX output data.
    Gnuplot2dDataset m_output_tx;                   //!< TX output data.
};

NodeStatistics::NodeStatistics(NetDeviceContainer aps, NetDeviceContainer stas)
{
    Ptr<NetDevice> device = aps.Get(0);
    Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice>(device);
    Ptr<WifiPhy> phy = wifiDevice->GetPhy();
    SetupPhy(phy);
    DataRate dataRate = DataRate(phy->GetDefaultMode().GetDataRate(phy->GetChannelWidth()));
    double power = phy->GetTxPowerEnd();
    for (uint32_t j = 0; j < stas.GetN(); j++)
    {
        Ptr<NetDevice> staDevice = stas.Get(j);
        Ptr<WifiNetDevice> wifiStaDevice = DynamicCast<WifiNetDevice>(staDevice);
        Mac48Address addr = wifiStaDevice->GetMac()->GetAddress();
        m_currentPower[addr] = power;
        m_currentRate[addr] = dataRate;
    }
    m_currentRate[Mac48Address("ff:ff:ff:ff:ff:ff")] = dataRate;
    m_totalEnergy = 0;
    m_totalTime = 0;
    busyTime = 0;
    idleTime = 0;
    txTime = 0;
    rxTime = 0;
    m_totalBusyTime = 0;
    m_totalIdleTime = 0;
    m_totalTxTime = 0;
    m_totalRxTime = 0;
    m_bytesTotal = 0;
    m_output.SetTitle("Throughput Mbits/s");
    m_output_idle.SetTitle("Idle Time");
    m_output_busy.SetTitle("Busy Time");
    m_output_rx.SetTitle("RX Time");
    m_output_tx.SetTitle("TX Time");
}

void
NodeStatistics::SetupPhy(Ptr<WifiPhy> phy)
{
    for (const auto& mode : phy->GetModeList())
    {
        WifiTxVector txVector;
        txVector.SetMode(mode);
        txVector.SetPreambleType(WIFI_PREAMBLE_LONG);
        txVector.SetChannelWidth(phy->GetChannelWidth());
        DataRate dataRate(mode.GetDataRate(phy->GetChannelWidth()));
        Time time = phy->CalculateTxDuration(g_packetSize, txVector, phy->GetPhyBand());
        NS_LOG_DEBUG(mode.GetUniqueName() << " " << time.GetSeconds() << " " << dataRate);
        m_timeTable.emplace_back(time, dataRate);
    }
}

Time
NodeStatistics::GetCalcTxTime(DataRate rate)
{
    for (auto i = m_timeTable.begin(); i != m_timeTable.end(); i++)
    {
        if (rate == i->second)
        {
            return i->first;
        }
    }
    NS_ASSERT(false);
    return Seconds(0);
}

void
NodeStatistics::PhyCallback(std::string path, Ptr<const Packet> packet, double powerW)
{
    WifiMacHeader head;
    packet->PeekHeader(head);
    Mac48Address dest = head.GetAddr1();

    if (head.GetType() == WIFI_MAC_DATA)
    {
        m_totalEnergy += pow(10.0, m_currentPower[dest] / 10.0) *
                         GetCalcTxTime(m_currentRate[dest]).GetSeconds();
        m_totalTime += GetCalcTxTime(m_currentRate[dest]).GetSeconds();
    }
}

void
NodeStatistics::PowerCallback(std::string path, double oldPower, double newPower, Mac48Address dest)
{
    m_currentPower[dest] = newPower;
}

void
NodeStatistics::RateCallback(std::string path,
                             DataRate oldRate,
                             DataRate newRate,
                             Mac48Address dest)
{
    m_currentRate[dest] = newRate;
}

void
NodeStatistics::StateCallback(std::string path, Time init, Time duration, WifiPhyState state)
{
    if (state == WifiPhyState::CCA_BUSY)
    {
        busyTime += duration.GetSeconds();
        m_totalBusyTime += duration.GetSeconds();
    }
    else if (state == WifiPhyState::IDLE)
    {
        idleTime += duration.GetSeconds();
        m_totalIdleTime += duration.GetSeconds();
    }
    else if (state == WifiPhyState::TX)
    {
        txTime += duration.GetSeconds();
        m_totalTxTime += duration.GetSeconds();
    }
    else if (state == WifiPhyState::RX)
    {
        rxTime += duration.GetSeconds();
        m_totalRxTime += duration.GetSeconds();
    }
}

void
NodeStatistics::RxCallback(std::string path, Ptr<const Packet> packet, const Address& from)
{
    m_bytesTotal += packet->GetSize();
}

void
NodeStatistics::CheckStatistics(double time)
{
    double mbs = ((m_bytesTotal * 8.0) / (1000000 * time));
    m_bytesTotal = 0;
    double atp = m_totalEnergy / time;
    m_totalEnergy = 0;
    m_totalTime = 0;
    m_output_power.Add((Simulator::Now()).GetSeconds(), atp);
    m_output.Add((Simulator::Now()).GetSeconds(), mbs);

    m_output_idle.Add((Simulator::Now()).GetSeconds(), idleTime * 100);
    m_output_busy.Add((Simulator::Now()).GetSeconds(), busyTime * 100);
    m_output_tx.Add((Simulator::Now()).GetSeconds(), txTime * 100);
    m_output_rx.Add((Simulator::Now()).GetSeconds(), rxTime * 100);
    busyTime = 0;
    idleTime = 0;
    txTime = 0;
    rxTime = 0;

    Simulator::Schedule(Seconds(time), &NodeStatistics::CheckStatistics, this, time);
}

Gnuplot2dDataset
NodeStatistics::GetDatafile()
{
    return m_output;
}

Gnuplot2dDataset
NodeStatistics::GetPowerDatafile()
{
    return m_output_power;
}

Gnuplot2dDataset
NodeStatistics::GetIdleDatafile()
{
    return m_output_idle;
}

Gnuplot2dDataset
NodeStatistics::GetBusyDatafile()
{
    return m_output_busy;
}

Gnuplot2dDataset
NodeStatistics::GetRxDatafile()
{
    return m_output_rx;
}

Gnuplot2dDataset
NodeStatistics::GetTxDatafile()
{
    return m_output_tx;
}

double
NodeStatistics::GetBusyTime() const
{
    return m_totalBusyTime + m_totalRxTime;
}

/**
 * Callback called by WifiNetDevice/RemoteStationManager/x/PowerChange.
 *
 * \param path The trace path.
 * \param oldPower Old Tx power.
 * \param newPower Actual Tx power.
 * \param dest Destination of the transmission.
 */
void
PowerCallback(std::string path, double oldPower, double newPower, Mac48Address dest)
{
    NS_LOG_INFO((Simulator::Now()).GetSeconds()
                << " " << dest << " Old power=" << oldPower << " New power=" << newPower);
}

/**
 * \brief Callback called by WifiNetDevice/RemoteStationManager/x/RateChange.
 *
 * \param path The trace path.
 * \param oldRate Old rate.
 * \param newRate Actual rate.
 * \param dest Destination of the transmission.
 */
void
RateCallback(std::string path, DataRate oldRate, DataRate newRate, Mac48Address dest)
{
    NS_LOG_INFO((Simulator::Now()).GetSeconds()
                << " " << dest << " Old rate=" << oldRate << " New rate=" << newRate);
}

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
    std::shared_ptr<RRMGreedyAlgo> rrmalgo;
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
    map<Mac48Address, set<Mac48Address>> bssid2stas;
    map<Mac48Address, Mac48Address> sta2bssid;
    // animation
    std::unique_ptr<AnimationInterface> anim;
    // metrics capture
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor;
    std::unique_ptr<NodeStatistics> nodeStats;
    // plotting
    GnuplotHelper plotHelper;
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

    static map<string, Mac48Address> traceStr2mac;
    static map<Mac48Address, RxPhyInfo> mac2SignalStats;
    static SimulationCase* currentInst;


    SimulationCase(
            const vector<uint16_t>& apChannelAllocation,
            const vector<uint16_t>& apStaAllocation,
            const vector<double>& apTxpAllocationDbm,
            const vector<uint16_t>& channelsToScan,
            const double simulationStartTime,
            const double simulationEndTime,
            const double trafficStartTime,
            const double trafficEndTime,
            const std::string simCaseName,
            std::shared_ptr<RRMGreedyAlgo> rrmalgo) :
        apChannelAllocation(apChannelAllocation),
        apTxpAllocationDbm(apTxpAllocationDbm),
        apStaAllocation(apStaAllocation),
        channelsToScan(channelsToScan),
        simulationStartTime(simulationStartTime),
        simulationEndTime(simulationEndTime),
        trafficStartTime(trafficStartTime),
        trafficEndTime(trafficEndTime),
        n_aps(apChannelAllocation.size()) {
            this->rrmalgo = rrmalgo;
            wifi.SetStandard(WIFI_STANDARD_80211n);
            // std::string phyMode = "ErpOfdmRate54Mbps";        ///< the constant PHY mode string used to transmit frames
            uint32_t rtsThreshold = 65535;
            std::string manager = "ns3::MinstrelHtWifiManager";
            wifi.SetRemoteStationManager(manager, "RtsCtsThreshold", UintegerValue(rtsThreshold));
            // wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
            //                              "DataMode", StringValue(phyMode),
            //                              "ControlMode", StringValue(phyMode));
            YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
            wifiPhy.SetChannel(wifiChannel.Create());
            addressHelper.SetBase("1.1.1.0", "255.255.255.0");

            traceStr2mac.clear();
            mac2SignalStats.clear();
            currentInst = this;
            sta2bssid.clear();

            setupAps();
            setupStas();

            NetDeviceContainer allStaDevs;
            for (auto& staDevs_i : staDevs) {
                allStaDevs.Add(staDevs_i);
            }

            nodeStats = std::make_unique<NodeStatistics>(apDevs, allStaDevs);

            do {
                // Config::Connect("/NodeList/4/ApplicationList/*/$ns3::UdpEchoClient/Rx", MakeCallback(&NodeStatistics::RxCallback, &(*nodeStats)));
                // Config::Connect("/NodeList/5/ApplicationList/*/$ns3::UdpEchoServer/Rx",
                //         MakeCallback(&NodeStatistics::RxCallback, &(*nodeStats)));
                // Register power and rate changes to calculate the Average Transmit Power
                for (size_t i = 0; i < n_aps; i++) {
                    uint32_t apNodeId = apNodes.Get(i)->GetId();
                    auto connectNode = [&](uint32_t nodeId) {
                        Config::Connect("/NodeList/" + std::to_string(nodeId) + "/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                                MakeCallback(&NodeStatistics::PhyCallback, &(*nodeStats)));
                        // Register States
                        Config::Connect(
                                "/NodeList/" + std::to_string(nodeId) + "/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/State",
                                MakeCallback(&NodeStatistics::StateCallback, &(*nodeStats)));
                    };
                    connectNode(apNodeId);
                    for (int k = 0; k < apStaAllocation[i]; k++) {
                        auto staNode = staNodes[i].Get(k);
                        connectNode(staNode->GetId());
                    }
                }
                nodeStats->CheckStatistics(1);
            } while(false);
            Ipv4GlobalRoutingHelper::PopulateRoutingTables();
            anim = std::make_unique<AnimationInterface>(simCaseName + ".xml");
            setupAnim();
            monitor = flowmon.InstallAll();
            Simulator::Stop(Seconds(trafficEndTime));
        }

    struct SimSignalResults {
        using snr_t = double;
        map<Mac48Address, snr_t> avgSnrBySta;
        snr_t avgSnr;
        map<Mac48Address, snr_t> avgSnrByAps;

        SimSignalResults(map<Mac48Address, snr_t> signalResults,
                snr_t avgSnr,
                map<Mac48Address, snr_t> avgSnrByAps) :
            avgSnrBySta(signalResults),
            avgSnr(avgSnr),
            avgSnrByAps(avgSnrByAps) {}
    };

    struct SimThroughputResults {
        double totalThroughput;
        double averageDelay;
        // double avgThroughput;
        // double avgThroughputByAps;
        // double avgThroughputByStas;

        SimThroughputResults(
                double totalThroughput,
                double averageDelay
                //,
                // double avgThroughput,
                // double avgThroughputByAps,
                // double avgThroughputByStas,
                ) :
            totalThroughput(totalThroughput),
            averageDelay(averageDelay)
            // avgThroughput(avgThroughput),
            // avgThroughputByAps(avgThroughputByAps),
            // avgThroughputByStas(avgThroughputByStas)
        {}
    };


    struct SimulationCaseResults {
    public:
        SimThroughputResults throughputResults;
        SimSignalResults signalResults;

        // RRM results from this simulation run
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
        SimulationCaseResults(
                SimThroughputResults throughputResults,
                SimSignalResults signalResults,
                const RrmResults& rrmResults) :
            throughputResults(throughputResults),
            signalResults(signalResults),
            newRrmAlloc(rrmResults)
        {}
    };


    Mac48Address getApBssid(Mac48Address staMac) {
        return sta2bssid.at(staMac);
    }

    static void
    staSniffer(
            std::string context, Ptr<const Packet> p,
            uint16_t channelFreqMhz,
            WifiTxVector txVector,
            MpduInfo aMpdu,
            SignalNoiseDbm signalNoise,
            uint16_t staId) {
        Ptr<Packet> packet = p->Copy();
        WifiMacHeader hdr;
        packet->RemoveHeader(hdr);
        if (hdr.GetAddr1().IsBroadcast() || !hdr.IsFromDs()) {
            return;
        }
        if (traceStr2mac.find(context) == traceStr2mac.end()) {
            cout << "context \'" + context + "\' not found in traceStr2mac" << endl;
        }
        Mac48Address rxMac = traceStr2mac.at(context);
        Mac48Address ra = hdr.GetAddr1();
        Mac48Address ta = hdr.GetAddr2();
        auto signalStats = mac2SignalStats[rxMac];
        if ((ra == rxMac) && (ta == currentInst->getApBssid(rxMac))) {
            signalStats.rssi += signalNoise.signal;
            signalStats.noise += signalNoise.noise;
            signalStats.snr += (double) signalNoise.signal - signalNoise.noise;
            signalStats.n += 1;
        }
        mac2SignalStats[rxMac] = signalStats;
    }


    void
    createScannerForStaNode(Ptr<Node> staWifiNode) {

        Ptr<WifiNetDevice> staWifiNetDev = getWifiNd(staWifiNode);
        std::stringstream ss;
        ss << "/NodeList/" << staWifiNode->GetId()
            << "/DeviceList/" << staWifiNetDev->GetIfIndex()
            << "/$ns3::WifiNetDevice/Phy/MonitorSnifferRx";
        string scanApTraceStr = ss.str();
        // cout << scanApTraceStr << "->" << getWifiMacStr(staWifiNetDev->GetMac()->GetAddress()) << endl;
        traceStr2mac[scanApTraceStr] = staWifiNetDev->GetMac()->GetAddress();
        // scannerByTraceContext[scanApTraceStr] = scanner;
        Config::Connect(scanApTraceStr, MakeCallback(&staSniffer));
    }

    SimulationCaseResults
    runSimulation(bool doRrm) {
        printSimulationParams();
        if (doRrm) {
            Simulator::Schedule(Seconds(7.0), [&](){rrmalgo->Decide();});
        }
        vector<uint16_t> apChannelAllocation(n_aps, 0);
        vector<double> apTxpAllocationDbm(n_aps, 0);
        Simulator::Run();
        monitor->CheckForLostPackets();
        if (doRrm) {
            ApsRrmAssignments rrmResults = rrmalgo->GetRrmResults();
            for (size_t i = 0; i < n_aps; i++) {
                auto apNode = apNodes.Get(i);
                Mac48Address bssid = getWifiNd(apNode)->GetMac()->GetAddress();
                auto [chan_i, txp_i] = rrmResults.at(bssid);
                apChannelAllocation[i] = chan_i;
                apTxpAllocationDbm[i] = txp_i * 1.0;
                // NS_LOG_DEBUG("AP " << bssid << " switched to RrmResult channel " << chan_i << " and txp " << txp_i);
            }
        }
        SimThroughputResults throughputResults = saveThroughputResults();
        SimSignalResults signalResults = saveSignalResults();

        SimulationCaseResults simResults(throughputResults, signalResults,
                {apChannelAllocation, apTxpAllocationDbm});
        return simResults;
    }

    // double
    // printMetrics() {
    //     saveSignalResults();
    //     double totalThroughput = saveThroughputResults();
    //     std::cout << "Total group throughput: " << totalThroughput << "Mbps" << std::endl;
    //     return totalThroughput;
    // }

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
            switchChannel_attr(wifiPhy, apChannelAllocation[i]);
            setTxPower_attr(wifiPhy, apTxpAllocationDbm[i]);
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
        string bssidStr_s = getWifiMacStr(apNode_i).substr(14);
        auto wifiPhy_i = wifiNd_i->GetPhy();
        std::stringstream apName;
        apName <<
            "AP-" << i << " " << bssidStr_s <<
            " CH: "   << +wifiPhy_i->GetOperatingChannel().GetNumber() <<
            " txp: " << wifiPhy_i->GetTxPowerStart();
        anim->UpdateNodeDescription(apNode_i, apName.str());
    }

    void setupApsAnim() const {
        for (size_t i = 0; i < apNodes.GetN(); i++) {
            setupApAnim(apNodes.Get(i), i);
        }
    }

    void setupApScanners() {
        // rrmalgo = std::make_shared<RRMGreedyAlgo>(channelsToScan);
        for (size_t i = 0; i < apNodes.GetN(); i++) { auto apNode = apNodes.Get(i);
            std::shared_ptr<Scanner> scanner = CreateScannerForNode(apNode, channelsToScan, "AP-" + std::to_string(i));
            scanner->setAfterScanCallback<void, RRMGreedyAlgo*, Scanner*>(
                    std::function<void(RRMGreedyAlgo*, Scanner*)>(
                        RRMGreedyAlgo::AddApScandata_s
                        ),
                    &(*rrmalgo),
                    &(*scanner)
                    );
            const double apScanStart_s = 2.5 + (0.01*i);
            Simulator::Schedule(Seconds(apScanStart_s), &Scanner::Scan, &(*scanner));
            scanners.push_back(scanner);
        }
        rrmalgo->AddDevices(scanners);
    }

    void setupUdpEchoClientServer (Ptr<Node> nodeServer, Ptr<Node> nodeClient, Ipv4InterfaceContainer& staInterfaces_i,
            const uint16_t echoPort=9,
            double maxPackets = 0, // 0 means unlimited
            double packetInterval = g_packetInterval,
            int packetSize = g_packetSize) const {
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
            Mac48Address bssid = getWifiNd(apNodes.Get(i))->GetMac()->GetAddress();
            for (size_t k = 0; k < staNodes[i].GetN(); k++) {
                Mac48Address staMac = getWifiNd(staNodes[i].Get(k))->GetMac()->GetAddress();
                sta2bssid[staMac] = bssid;
                bssid2stas[bssid].insert(staMac);
                createScannerForStaNode(staNodes[i].Get(k));
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
        for (size_t i = 0; i < n_aps; i++) {
            NodeContainer staNodes_ap_i = staNodes[i];
            for (auto it = staNodes_ap_i.Begin(); it != staNodes_ap_i.End(); ++it) {
                int k = (it - staNodes_ap_i.Begin());
                std::stringstream bssidStr; bssidStr << getWifiNd(*it)->GetMac()->GetAddress(); string bssidStr_s = bssidStr.str();
                std::cout << std::left
                    << std::setw(10) << (std::to_string(i) + "-" + std::to_string(k))
                    << std::setw(25) << getWifiNd(*it)->GetMac()->GetSsid().PeekString()
                    << std::setw(10) << +getWifiNd(*it)->GetPhy()->GetOperatingChannel().GetNumber()
                    << std::setw(20) << bssidStr_s
                    << std::endl;
            }
        }
        cout << "====================================================================" << endl;
    }

    SimThroughputResults
    saveThroughputResults() {
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
            << std::setw(15) << "Mean delay (s)"
            << std::endl;
        double timeDiff = (trafficEndTime - trafficStartTime);
        double averageDelay = 0.0;
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
            timeDiff = (flowStats.timeLastRxPacket.GetSeconds() - flowStats.timeFirstTxPacket.GetSeconds());

            auto printStats =  [&](Mac48Address& mac1, Mac48Address& mac2) {
                // double t = (trafficEndTime - trafficStartTime);
                double txOffered = (flowStats.rxBytes * 8 / 1000.0 / 1000.0) / timeDiff;
                double throughput = (flowStats.rxBytes * 8 / 1000.0 / 1000.0) / timeDiff;

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
                    << std::setw(15) << flowStats.delaySum.GetSeconds() / flowStats.rxPackets
                    << std::endl;
            };
            averageDelay += flowStats.delaySum.GetSeconds() / flowStats.rxPackets;
            printStats(ip2mac.at(srcAddr), ip2mac.at(dstAddr));
            printStats(ip2mac.at(dstAddr), ip2mac.at(srcAddr));
        }
        cout << "Total busy time: " << nodeStats->GetBusyTime() / (simulationEndTime - simulationStartTime) << endl;
        cout << "==================================================================================================" << endl;
        averageDelay = averageDelay / stats.size();
        double totalThroughput = (totalRxBytes * 8 / 1000.0 / 1000.0) / timeDiff;
        return {totalThroughput, averageDelay};
    }

    SimSignalResults
    saveSignalResults() {
        cout << "====================================== RSSI records ==============================================" << endl;
        std::cout << std::left << std::setw(20) << "mac"
            << std::setw(10) << "avgRSSI"
            << std::setw(10) << "avgNoise"
            << std::setw(10) << "avgSNR" << endl;
        for (auto [mac, rxPhy] : mac2SignalStats) {
            std::stringstream macStr; macStr << mac;
            std::cout << std::left << std::setw(20) << macStr.str()
                << std::setw(10) << rxPhy.rssi / rxPhy.n
                << std::setw(10) << rxPhy.noise / rxPhy.n
                << std::setw(10) << rxPhy.snr / rxPhy.n
                << endl;
        }
        cout << "==================================================================================================" << endl;


        double avgSnr = 0.0;
        map<Mac48Address, double> signalStats;
        map<Mac48Address, double> avgSnrByAps;
        for (auto [mac, rxPhy] : mac2SignalStats) {
            rxPhy.snr = rxPhy.snr / rxPhy.n;
            rxPhy.rssi = rxPhy.rssi / rxPhy.n;
            rxPhy.noise = rxPhy.noise / rxPhy.n;

            signalStats[mac] = rxPhy.snr;
            avgSnr += rxPhy.snr;
        }
        for (auto [bssid, stas] : bssid2stas) {
            double avgSnrAp = 0.0;
            for (auto sta : stas) {
                avgSnrAp += signalStats[sta];
            }
            avgSnrAp = avgSnrAp / stas.size();
            avgSnrByAps[bssid] = avgSnrAp;
        }
        avgSnr = avgSnr / mac2SignalStats.size();
        SimSignalResults signalResults(signalStats, avgSnr, avgSnrByAps);
        return signalResults;
    }

   static void PrintThroughputComparison(vector<string>& caseNames, vector<SimulationCaseResults> results) {
       cout << "=================== Throughput Comparison =======================" << endl;
       cout << setw(15) << "Scenario" << setw(20) << "Throughput(Mbit/s)" << setw(20) << "Average delay (s)" << endl;

       for (size_t i = 0; i < caseNames.size(); i++) {
           auto result = results[i];
           auto name = caseNames[i];
           cout << setw(15) << name << setw(20) << result.throughputResults.totalThroughput
               << setw(20) << result.throughputResults.averageDelay << endl;
       }
   }

   static void PrintSignalComparison(vector<string>& caseNames, vector<SimulationCaseResults>& results) {
       cout << "=================== Signal Comparison (Total AvgSNR) =======================" << endl;
       cout << setw(15) << "Scenario" << setw(20) << "AvgSNR" << endl;
       size_t n_cases = caseNames.size();
       for (size_t i = 0; i < n_cases; i++) {
           auto result = results[i];
           auto name = caseNames[i];
           cout << setw(15) << name << setw(20) << result.signalResults.avgSnr << endl;
       }
       cout << "=================== Signal Comparison (AvgSNR by AP) =======================" << endl;
       cout << setw(15) << "Scenario";
       for (size_t i = 0; i < results[i].signalResults.avgSnrByAps.size(); i++) {
           cout << setw(20) << "AvgSNR@AP" + std::to_string(i);
       }
       cout << endl;

       for (size_t i = 0; i < n_cases; i++) {
           cout << setw(15) << caseNames[i];
           for(auto& [mac, snr] : results[i].signalResults.avgSnrByAps) {
               cout << setw(20) << snr;
           }
           cout << endl;
       }
       cout << "=================== Signal Comparison (AvgSNR by STA) =======================" << endl;
       cout << setw(15) << "Scenario";
       for (size_t i = 0; i < results[i].signalResults.avgSnrBySta.size(); i++) {
           cout << setw(15) << "AvgSNR@STA" + std::to_string(i);
       }
       cout << endl;

       for (size_t i = 0; i < n_cases; i++) {
           cout << setw(15) << caseNames[i];
           for(auto& [mac, snr] : results[i].signalResults.avgSnrBySta) {
               cout << setw(15) << snr;
           }
           cout << endl;
       }
   }

   static void PrintComparison(vector<string>& caseNames, vector<SimulationCaseResults>& results) {
       PrintThroughputComparison(caseNames, results);
       PrintSignalComparison(caseNames, results);
       cout << "================================================================" << endl;
   }

    ~SimulationCase() {
        Simulator::Destroy();
    }
};



map<string, Mac48Address> SimulationCase::traceStr2mac;
map<Mac48Address, RxPhyInfo> SimulationCase::mac2SignalStats;
SimulationCase* SimulationCase::currentInst;

std::pair<SimulationCase::SimulationCaseResults, SimulationCase::SimulationCaseResults>
evaluateAlgo(
            const vector<uint16_t>& apChannelAllocation,
            const vector<uint16_t>& apStaAllocation,
            const vector<double>& apTxpAllocationDbm,
            const vector<uint16_t>& channelsToScan,
            const double simulationStartTime,
            const double simulationEndTime,
            const double trafficStartTime,
            const double trafficEndTime,
            const std::string simCaseName,
            std::shared_ptr<RRMGreedyAlgo> rrmalgo) {
    // baseline simulation stage
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~ NoRRM Simulation ~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    auto noRrmCase = new SimulationCase(
            apChannelAllocation,
            apStaAllocation,
            apTxpAllocationDbm,
            channelsToScan,
            simulationStartTime,
            simulationEndTime,
            trafficStartTime,
            trafficEndTime,
            simCaseName,
            rrmalgo
    );
    SimulationCase::SimulationCaseResults noRrmResults = noRrmCase->runSimulation(true);
    delete noRrmCase;

    // rrm simulation stage
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~ " << simCaseName << " Simulation ~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    auto withRrmGreedyCase = new SimulationCase(
            noRrmResults.newRrmAlloc.apChannelAllocation,
            apStaAllocation,
            noRrmResults.newRrmAlloc.apTxpAllocationDbm,
            channelsToScan,
            simulationStartTime,
            simulationEndTime,
            trafficStartTime,
            trafficEndTime,
            simCaseName,
            rrmalgo
    );
    SimulationCase::SimulationCaseResults withRrmGreedyResults = withRrmGreedyCase->runSimulation(false);
    delete withRrmGreedyCase;
    return {noRrmResults, withRrmGreedyResults};
}

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

    vector<uint16_t> channelsToScan = {1, 6, 11};

    assert(apStaAllocation.size() == NUM_APS &&
            std::string(
                "expected NUM_APS=" + std::to_string(NUM_APS) + ", got " + std::to_string(apStaAllocation.size()) + " channel allocations"
                ).c_str());

    std::shared_ptr<RRMGreedyAlgo> rrmgreedy = std::make_shared<RRMGreedyAlgo>(channelsToScan);
    auto [baselineResults, withRrmGreedyResults] = evaluateAlgo(
            apChannelAllocation,
            apStaAllocation,
            apTxpAllocationDbm,
            channelsToScan,
            0.0,
            10.0,
            0.5,
            10.0,
            "rrmgreedy",
            rrmgreedy
    );
    cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~ With RRMGreedy Simulation ~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
    // RRMGreedyPlusPlusAlgo rrmgRMGreedyPlusPlusAlgo(channelsToScan);
    std::shared_ptr<RRMGreedyAlgo> rrmgreedyplusplus = std::make_shared<RRMGreedyPlusPlusAlgo>(channelsToScan);
    auto [baselineResults2, withRrmGreedyPlusPlusResults] = evaluateAlgo(
            apChannelAllocation,
            apStaAllocation,
            apTxpAllocationDbm,
            channelsToScan,
            0.0,
            10.0,
            0.5,
            10.0,
            "rrmgreedy++",
            rrmgreedyplusplus
    );

    // print metrics and benchmark results

    vector<string> caseNames = {"Initial", "RRMGreedy", "RRMGreedy++"};
    vector<SimulationCase::SimulationCaseResults> results = {baselineResults, withRrmGreedyResults, withRrmGreedyPlusPlusResults};
    SimulationCase::PrintComparison(caseNames, results);

    return 0;
}

class ApSettings {
public:
    uint16_t channel;
    double txp;
    ApSettings(uint16_t channel, double txp) : channel(channel), txp(txp) {}
};

