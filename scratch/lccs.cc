
/**
 * 					__________________Server_________
 * 					|			|		|           |
 * 				   AP1         AP2     AP3          AP4
 * 		       _____|_____
 * 	           |         |
 *          Client0   Client1
 *
 *


 There are 2 Clients , 4 Aps and a server.
 The 4 Aps and server are connected by p2p link.
 2 clients, 4 Aps and the server are stationary.


 Aps Send beacons at different time intervals.

 Aps are deployed on Channel 36 each AP is assigned a unique ssid
 Ap1  --> ssid AP1
 Ap2  --> ssid AP2
 Ap3  --> ssid AP3
 Ap4  --> ssid AP4


 Two NetDevices are installed on the client node.
 The RRI object/Measurement MAC is created and is installed on the second device
 The second net device is not connected to any AP

 The Measurement MAC object has  logic to make the measurements and scan the channels

 1)From the script : Change the association of the client to AP with maximum snr

 Create function to associate with new AP based on snr :
 AssociateWithBestSNR() is scheduled to be called  at some time (50th second ) from the  script


 Each AP is assigned a unique ssid value. This information is stored in a datastructure.
 Change in association is based on the ssid of the AP.

 Data traffic flows from the server to both the clients


 Standard Used: IEEE 802.11n_5Ghz

 802.11n configuration parameters:

 short_guard enabled  enabled
 Data Rate - 6 Mbps

 Get inputs from the user:

 1. Start time of RRM logic
 2. Scan Duration: Period for which the Device with RRM logic should remain in a particular channel
 while scanning
 3. Enter the channels to scan ( Currenlty 3 channles are go as input)
 4. Get snrThreshold as input from user
 *
 AssociatewithBestSnr() is triggered for client 1 at 50s

*/

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/gnuplot.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/olsr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/rri-module.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-phy.h"

#include <iostream>
#include <map>
#include <iomanip>
// #include <random>
// #include <iterator>

using namespace ns3;
using namespace std;




NS_LOG_COMPONENT_DEFINE("twoRadio");

// static void
// TraceAssoc(std::string context, Mac48Address bssid)
// {
//     NS_LOG_INFO("Association at  " << Simulator::Now().GetSeconds() << " to " << bssid);
// }
//
// static void
// TraceDeassoc(std::string context, Mac48Address bssid)
// {
//     NS_LOG_INFO("Deassociation at " << Simulator::Now().GetSeconds() << " from " << bssid << context);
// }

// Function to display the map containing average SNR values of last 5 beacons.
// We display both MAC of Aps and the average snr values.
// Also display the Aps and the channel number of the Aps
void
rriModStaSnrData(Ptr<RriModuleMac> rriMod)
{
    int chNo;
    Ssid apSsid;
    char* ssidVal;

    cout << endl;

    std::map<Mac48Address, std::pair<double, Ssid>> map_ApSnrSsid = rriMod->map_ApSnrSsid;

    /* Test in APChnl Map */
    std::map<Mac48Address, int> map_ap_channel = rriMod->map_ap_channel;

    // cout << "AP Mac Address   \tChannel #" << endl;
    // cout << "-------------------------------------------------------------" << endl;
    // for (auto it = map_ap_channel.begin(); it != map_ap_channel.end(); ++it)
    // {
    //     cout << setw(20) << it->first << setw(10) << it->second << "\n";
    // }

    cout << endl;
    cout << "Scan results for " << rriMod->cpeId << " (" << rriMod->GetAddress() << ")" << endl;
    cout << "Entries: " << map_ApSnrSsid.size() << endl;
    cout << "BSSID   \t   Avg SNR \t Channel# \t  SSID" << endl;
    cout << "------------------------------------------------ --------" << endl;
    for (auto it = map_ApSnrSsid.begin(); it != map_ApSnrSsid.end(); ++it)
    {
        chNo = map_ap_channel[it->first];

        apSsid = it->second.second;
        ssidVal = apSsid.PeekString();

        /* Output first , macid , snr value , channel no, ssid */
        cout << setw(15) << it->first << setw(10) << it->second.first << " \t " << chNo
             << "\t\t" << ssidVal << "\n";
    }

    cout << endl; /* Print a new line */
}
//
// // Function to Associate with the AP with best snr value
// void
// AssociateWithBestSNR(Ptr<StaWifiMac> staMac1, Ptr<RriModuleMac> rriMod)
// {
//     // std::cout << "\n";
//     // std::cout << "*************************** ";
//     // std::cout << "\n";
//     // Get access to the mapapSnrSsid data structure of the client from the new mac
//     std::map<Mac48Address, std::pair<double, Ssid>> map_Ap_SnrSsid = rriMod->map_ApSnrSsid;
//     std::map<Mac48Address, int> mapAPchn = rriMod->map_ap_channel;
//
//     Mac48Address temp_add;
//     double max = 0;
//     for (auto it1 = map_Ap_SnrSsid.begin(); it1 != map_Ap_SnrSsid.end(); ++it1)
//     {
//         NS_LOG_DEBUG("BSSID: " << it1->first << " ; "
//                   << "SINR Value: " << it1->second.first << std::endl);
//         // Check the SNR value
//         if ((it1->second.first) > max)
//         {
//             temp_add = it1->first;
//             max = it1->second.first;
//         }
//     }
//     /* from artem: GetBssid API changed since this code was written;
//      * now it expects link_id.
//      * given that our stations keeps strictly 1 link (to an AP) at a time, I can fetch it from the GetLinkIds map */
//     auto& m_link_ids = staMac1->GetLinkIds();
//     assert(m_link_ids.size() == 1);
//     uint8_t link_id = *m_link_ids.begin();
//
//     if (temp_add != staMac1->GetBssid(link_id))
//     {
//         Ssid apSsid = map_Ap_SnrSsid[temp_add].second;
//
//         int ch = mapAPchn[temp_add];
//         /* from artem: simply using channel number as integer is not possible anymore */
//         // WifiPhyOperatingChannel channel;
//         constexpr uint16_t width = 20;
//         constexpr uint16_t primary20idx = 0;
//         constexpr WifiPhyBand band = WIFI_PHY_BAND_5GHZ;
//         // channel.Set(ch, chan_freq, width, USED_80211_STANDARD, band);
//
//         WifiPhy::ChannelTuple channel = std::make_tuple(ch, width, band, primary20idx);
//         std::cout << rriMod->cpeId << ": setting channel to " << ch << endl;
//         staMac1->GetWifiPhy()->SetOperatingChannel(channel);
//         // phy.SetChannel("{36, 20, BAND_5GHZ, 0}"); // this only works on helper
//         // Associate to the AP using ssid
//         std::cout << rriMod->cpeId << ": assoc to SSID=" << apSsid.PeekString() << "(BSSID=" << temp_add << "), " << "with max SNR=" << max << "\n";
//
//         Ptr<RegularWifiMac> regStmac = DynamicCast<RegularWifiMac>(staMac1);
//         regStmac->SetSsid(apSsid);
//     }
//     else
//     {
//         std::cout << "Client already associated to the AP with MAC address: " << temp_add << "\n";
//         std::cout << "\n";
//     }
// }


// using ChannelTuple =
//     std::tuple<uint8_t /* channel number */,
//                uint16_t /* channel width */,
//                int /* WifiPhyBand */,
//                uint8_t /* primary20 index*/>; //!< Tuple identifying an operating channel
map<chNum_t, WifiPhy::ChannelTuple> map_chanToChanTuple {
    {36, std::make_tuple(36, 20, WIFI_PHY_BAND_5GHZ, 0)},
    {40, std::make_tuple(40, 20, WIFI_PHY_BAND_5GHZ, 0)},
    {44, std::make_tuple(44, 20, WIFI_PHY_BAND_5GHZ, 0)},
    {48, std::make_tuple(48, 20, WIFI_PHY_BAND_5GHZ, 0)},
    {52, std::make_tuple(52, 20, WIFI_PHY_BAND_5GHZ, 0)},
    {56, std::make_tuple(56, 20, WIFI_PHY_BAND_5GHZ, 0)},
    {60, std::make_tuple(60, 20, WIFI_PHY_BAND_5GHZ, 0)},
    {64, std::make_tuple(64, 20, WIFI_PHY_BAND_5GHZ, 0)},
};

list<chNum_t> avail_channels = {36, 40, 44, 48, 52, 56};


void setChannel(Ptr<RegularWifiMac> apMac, chNum_t ch)
{
    uint8_t link_id = *apMac->GetLinkIds().begin();
    auto& chanTuple = map_chanToChanTuple[ch];
    apMac->GetWifiPhy(link_id)->SetOperatingChannel(chanTuple);
}

void traceRx(std::string ctx, Ptr<const Packet> p)
{
    std::cout << "Rx: " << ctx << std::endl;
}

//void setChannelEvent(Ptr<RegularWifiMac> apMac, Ptr<RriModuleMac> rriMod, Ptr<Node> node, chNum_t ch, AnimationInterface& anim)
void setChannelEvent(Ptr<RegularWifiMac> apMac, Ptr<RriModuleMac> rriMod, Ptr<Node> node, chNum_t ch)
{
    setChannel(apMac, ch);
    std::cout << rriMod->cpeId << " switched to channel " << +ch << endl;
    //anim.UpdateNodeDescription(node, rriMod->cpeId + "\nCH: " + std::to_string(ch));
}

void decideChanSwitch(Ptr<ApWifiMac> apMac, Ptr<RriModuleMac> rriMod, Ptr<Node> node)
{
    // Get access to the map_ApSnrSsid data structure of the AP
    // std::map<Mac48Address, std::pair<double, Ssid>>& map_Ap_SnrSsid = rriMod->map_ApSnrSsid;
    // std::map<Mac48Address, int>& mapAPchn = rriMod->map_ap_channel;
    chNum_t cur_ch = apMac->GetWifiPhy()->GetChannelNumber();
    chNum_t new_ch = cur_ch;
    using chanLoad_t = int;
    chanLoad_t min_load = rriMod->map_ChanLoad[cur_ch];

    // using map_ChanLoad, print the table of channel loads
    // print header
    std::cout << rriMod->cpeId << " is deciding on channel switch. Current Channel: " << +cur_ch << endl;
    std::cout << "Channel\tLoad" << endl;
    std::cout << "-------------" << endl;

    for (chNum_t ch_i : avail_channels) {
        std::cout << (unsigned int) ch_i << "\t" << rriMod->map_ChanLoad[ch_i] << endl;
        chanLoad_t load_i = 0;
        if (rriMod->map_ChanLoad.count(ch_i)) {
            load_i = rriMod->map_ChanLoad[ch_i];
        }
        if (load_i < min_load) {
            min_load = load_i;
            new_ch = ch_i;
        }
    }
    std::cout << "-------------" << endl;

    if (new_ch != cur_ch) {
        // NS_LOG_INFO(rriMod->cpeId << ": setting channel to " << +new_ch << endl);
        setChannelEvent(apMac, rriMod, node, new_ch);
    } else {
        NS_LOG_INFO(rriMod->cpeId << " decided to stay on channel " << +cur_ch);
    }
}

int
main(int argc, char* argv[])
{
    LogComponentEnable("twoRadio", LOG_LEVEL_INFO);
    LogComponentEnable("RriModuleMac", LOG_LEVEL_INFO);
    LogComponentEnable("WifiPhy", LOG_LEVEL_DEBUG);

    double start_scanning = 2.0;
    double scan_duration = 1.0;

    CommandLine cmd;
    cmd.AddValue("StartRRM", "Start Time of RRI Module (in seconds)", start_scanning);
    cmd.AddValue("ScanDuration", "Duration to stay in channel (in seconds)", scan_duration);
    cmd.Parse(argc, argv);
    vector<chNum_t> rrm_channels_to_scan{36, 40, 44, 48};
    // double snrThreshold = 60;

    const double txgain = 0.0;
    const double PathLossExponent = 3.5;
    const double CCAThreshold = -85.0;

    // constexpr int SERVER_COUNT = 1;
    // constexpr int STA_COUNT = 2;
    // constexpr int STA_NETDEVS_COUNT = STA_COUNT;
    constexpr int AP_COUNT = 4;
    constexpr int AP_NETDEVS_COUNT = AP_COUNT*2;

    NS_LOG_INFO("Create 4 nodes.");

    // NodeContainer ServerNode(SERVER_COUNT);
    NodeContainer wifiAPnodes(AP_COUNT);
    // NodeContainer wifiSTAnodes(STA_COUNT);

    // Create p2p links from each AP to Server
    // NodeContainer n0n1 = NodeContainer(ServerNode.Get(0), wifiAPnodes.Get(0)); // server and Ap1
    // NodeContainer n0n2 = NodeContainer(ServerNode.Get(0), wifiAPnodes.Get(1)); // server and Ap2
    // NodeContainer n0n3 = NodeContainer(ServerNode.Get(0), wifiAPnodes.Get(2)); // server and Ap3
    // NodeContainer n0n4 = NodeContainer(ServerNode.Get(0), wifiAPnodes.Get(3)); // server and Ap4

    // PointToPointHelper p2p;
    // p2p.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
    // p2p.SetChannelAttribute("Delay", StringValue("1us"));
    //
    // // Install P2P links
    // NetDeviceContainer p2pDevices1 = p2p.Install(n0n1);
    // NetDeviceContainer p2pDevices2 = p2p.Install(n0n2);
    // NetDeviceContainer p2pDevices3 = p2p.Install(n0n3);
    // NetDeviceContainer p2pDevices4 = p2p.Install(n0n4);

    // Enable the beacon jitter for the MAC of APs
    Config::SetDefault("ns3::ApWifiMac::EnableBeaconJitter", BooleanValue(true));

    // Create wireless channels
    NS_LOG_INFO("Create channels.");

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                               "Exponent",
                               DoubleValue(PathLossExponent));

    YansWifiPhyHelper wiPhyHelper = YansWifiPhyHelper();
    wiPhyHelper.Set("TxGain", DoubleValue(txgain));
    wiPhyHelper.Set("RxGain", DoubleValue(0));
    wiPhyHelper.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
    wiPhyHelper.Set("CcaEdThreshold", DoubleValue(CCAThreshold));
    // phy.Set("ShortGuardEnabled", BooleanValue(true)); // deprecated

    // configure operating channel
    wiPhyHelper.SetChannel(channel.Create());
    TupleValue<UintegerValue, UintegerValue, EnumValue, UintegerValue> value;
    value.Set(WifiPhy::ChannelTuple {36, 20, WIFI_PHY_BAND_5GHZ, 0});
    wiPhyHelper.Set("ChannelSettings", value);

    // Set Wifi Standard
    WifiHelper wifiHelper;
    wifiHelper.SetStandard(WIFI_STANDARD_80211ac);

    StringValue phyRate("VhtMcs0");

    // use same transmission rate for all the stations
    wifiHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue(phyRate),
                                 "ControlMode",
                                 StringValue(phyRate));

    // Set HT Wi-Fi MAC

    WifiMacHelper wifiMac;
    WifiMacHelper scanMac;

    // NetDeviceContainer staNetDev[STA_NETDEVS_COUNT];

    Ssid ssids[] = {Ssid("AP1"), Ssid("AP2"), Ssid("AP3"), Ssid("AP4")};

    // auto setupStaMainMac = [&](int sta_i, Ssid ssid) {
    //     char macAddr[18] = {0};
    //     sprintf(macAddr, "00:55:55:55:%02x:%02x", sta_i / 256, sta_i % 256);
    //     wifiMac.SetType("ns3::StaWifiMac",
    //                     "Ssid",
    //                     SsidValue(ssid),
    //                     "ActiveProbing",
    //                     BooleanValue(false),
    //                     "QosSupported",
    //                     BooleanValue(true));
    //     wiPhy.Set("TxPowerStart", DoubleValue(17.0));
    //     wiPhy.Set("TxPowerEnd", DoubleValue(17.0));
    //     staNetDev[sta_i] = wifi.Install(wiPhy, wifiMac, wifiSTAnodes.Get(sta_i));
    //     // Ptr<StaWifiMac> apMac = DynamicCast<StaWifiMac>((apNetDev->GetMac()));
    // };
    //
    // /* setup STAs */
    // NS_LOG_INFO("Setup STAs.");
    // for (int i = 0; i < STA_COUNT; i++) {
    //     setupStaMainMac(i, ssids[i]);
    // }

    /* Setting Attributes for AP */
    NS_LOG_INFO("Setup APs.");
    NetDeviceContainer apNetDevices[AP_NETDEVS_COUNT];
    wiPhyHelper.Set("TxPowerStart", DoubleValue(23.0));
    wiPhyHelper.Set("TxPowerEnd", DoubleValue(23.0));

    auto setupAp = [&](int ap_i, Ssid ssid, unsigned long beaconInterval=102400) {
        wifiMac.SetType("ns3::ApWifiMac",
                        "Ssid",
                        SsidValue(ssid),
                        "BeaconGeneration",
                        BooleanValue(true),
                        "BeaconInterval",
                        TimeValue(MicroSeconds(beaconInterval)),
                        "QosSupported",
                        BooleanValue(true)
                        );
        apNetDevices[2*ap_i] = wifiHelper.Install(wiPhyHelper, wifiMac, wifiAPnodes.Get(ap_i));
        Ptr<WifiNetDevice> ap_i_NetDev = DynamicCast<WifiNetDevice>(apNetDevices[2*ap_i].Get(0));
        Ptr<RegularWifiMac> apMac = DynamicCast<RegularWifiMac>(DynamicCast<ApWifiMac>(ap_i_NetDev->GetMac()));
        apMac->SetSsid(ssid);
    };

    auto setupApScanMac = [&](int ap_i) {
        wiPhyHelper.SetChannel(channel.Create());
        TupleValue<UintegerValue, UintegerValue, EnumValue, UintegerValue> value;
        uint8_t chan = 36; // rrm_channels_to_scan[ap_i];
        value.Set(WifiPhy::ChannelTuple {chan, 20, WIFI_PHY_BAND_5GHZ, 0});
        wiPhyHelper.Set("ChannelSettings", value);
        scanMac.SetType("ns3::RriModuleMac",
                        "GetLoadUpdates",
                        BooleanValue(false),
                        "StartTime",
                        TimeValue(Seconds(start_scanning)),
                        "ScanDuration",
                        TimeValue(Seconds(scan_duration)));
        // Install the Measurement MAC on STA Node
        apNetDevices[2*ap_i + 1] = wifiHelper.Install(wiPhyHelper, scanMac, wifiAPnodes.Get(ap_i));
        Ptr<WifiNetDevice> wifiNetDev = DynamicCast<WifiNetDevice>(apNetDevices[2*ap_i + 1].Get(0));
        // Set the netdev of the measurement mac to promisc mode to receive data packets meant to device 1
        // wifiNetDev->SetStandard(WIFI_STANDARD_80211ac);
        wifiNetDev->GetMac()->SetPromisc();
        // Pass to the scanning module the list of channels to scan
        Ptr<RriModuleMac> rriMod = DynamicCast<RriModuleMac>(wifiNetDev->GetMac());
        rriMod->setChannelsToScan(rrm_channels_to_scan);
        rriMod->cpeId = "AP " + std::to_string(ap_i);
    };

    // all start on same channel

    for (int i = 0; i < AP_COUNT; i++) {
        setupAp(i, ssids[i]);
        // setupApChannel(i, apStartingChannels[i]);
        setupApScanMac(i);
    }

    NS_LOG_INFO("Init mobility model.");

    // Init mobility model. All nodes are stationary
    MobilityHelper mobility1;
    mobility1.SetMobilityModel(
        "ns3::ConstantPositionMobilityModel"); // For stationary APs Aand client
    Ptr<ListPositionAllocator> positionAlloc1 = CreateObject<ListPositionAllocator>();

    Vector positions[] = {
        // Vector(100.0, 20.0, 0.0), // Server position
        Vector(0.0, 10.0, 0.0),   // AP1
        Vector(25.0, 10.0, 0.0),  // AP2
        Vector(50.0, 10.0, 0.0),  // AP3
        Vector(75.0, 10.0, 0.0),  // AP4
        // Vector(37.0, 15.0, 0.0),  // STA1
        // Vector(40.0, 15.0, 0.0)   // STA2
    };

    for (auto& pos : positions) {
        positionAlloc1->Add(pos);
    }

    mobility1.SetPositionAllocator(positionAlloc1);

    // mobility1.Install(ServerNode);
    mobility1.Install(wifiAPnodes);
    // mobility1.Install(wifiSTAnodes);
    // TODO: make station roam between APs

    // Using OLSR routing protocol
    // OlsrHelper olsr;
    // Ipv4StaticRoutingHelper staticRouting;
    // Ipv4ListRoutingHelper list;
    // list.Add(staticRouting, 0);
    // list.Add(olsr, 10);
    //
    // // Installing internet stack
    InternetStackHelper stack;
    // stack.SetRoutingHelper(list);

    // stack.Install(ServerNode);
    // stack.Install(wifiSTAnodes);
    stack.Install(wifiAPnodes);

    NS_LOG_INFO("Assign IP Addresses.");

    Ipv4AddressHelper address;

    // Assign IP address to the 2 interfaces on station nodes
    address.SetBase("10.1.5.0", "255.255.255.0");
    // Ipv4InterfaceContainer StaInterface[STA_NETDEVS_COUNT];
    // for (int i = 0; i < STA_NETDEVS_COUNT; i++) {
    //     StaInterface[i] = address.Assign(staNetDev[i]);
    // }

    Ipv4InterfaceContainer ApInterface[AP_COUNT];
    for (int i = 0; i < AP_COUNT; i++) {
        ApInterface[i] = address.Assign(apNetDevices[i]);
    }

    // Data connections from Server to client
    // NS_LOG_INFO("Create Applications.");
    // uint16_t port = 9; // Discard port (RFC 863)
    //
    // // Install the ON/OFF application downlink
    // ApplicationContainer sourceApp1;
    // ApplicationContainer sourceApp2;
    //
    // BulkSendHelper onoff("ns3::TcpSocketFactory", Address());
    // onoff.SetAttribute("MaxBytes", UintegerValue(0));

    // onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
    // onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
    // onoff.SetAttribute("DataRate", StringValue("54Kbps"));

    // install source for the client 1 on Server
    // AddressValue remoteAddress1;
    // remoteAddress1 = AddressValue(InetSocketAddress(StaInterface[0].GetAddress(0), port));
    // onoff.SetAttribute("Remote", remoteAddress1);
    // sourceApp1.Add(onoff.Install(ServerNode.Get(0)));
    // sourceApp1.Start(Seconds(15.0));
    // sourceApp1.Stop(Seconds(17.0));

    // install source for the client 2 on Server
    // AddressValue remoteAddress2;
    // remoteAddress2 = AddressValue(InetSocketAddress(StaInterface[2].GetAddress(0), port));
    // onoff.SetAttribute("Remote", remoteAddress2);
    // sourceApp2.Add(onoff.Install(ServerNode.Get(0)));
    // sourceApp2.Start(Seconds(15.1));
    // sourceApp2.Stop(Seconds(17.0));
    //
    // // Create an optional packet sink to receive these packets on
    // ApplicationContainer sink1;
    // ApplicationContainer sink2;
    //
    // AddressValue localaddress;
    //
    // PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", Address());
    // localaddress = Address(InetSocketAddress(Ipv4Address::GetAny(), port));
    // packetSinkHelper.SetAttribute("Local", localaddress);
    // sink1.Add(packetSinkHelper.Install(wifiSTAnodes.Get(0))); // Install sink on STA1
    // sink1.Start(Seconds(15.0));
    // sink1.Stop(Seconds(17.0));
    //
    // sink2.Add(packetSinkHelper.Install(wifiSTAnodes.Get(1))); // Install sink on STA2
    // sink2.Start(Seconds(15.1));
    // sink2.Stop(Seconds(17.0));

    const string scriptName = "lccs";
    AnimationInterface anim(scriptName + ".xml");
    // Tracing, .pcap and .xml files generation
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream(scriptName + "tr");
    // stack.EnableAsciiIpv4All(stream);

    wiPhyHelper.EnablePcapAll(scriptName);

    NS_LOG_INFO("Setup animation.");
    auto setupAPAnimation = [&](int i) {
        anim.UpdateNodeColor(wifiAPnodes.Get(i), 255, 0, 0); // red
        anim.UpdateNodeSize(wifiAPnodes.Get(i)->GetId(), 5.0, 5.0);
        anim.UpdateNodeDescription(wifiAPnodes.Get(i), "AP" + std::to_string(i + 1));
    };

    for (int i = 0; i < AP_COUNT; i++) {
        setupAPAnimation(i);
    }

    // auto setupSTAAnimation = [&](int i) {
    //     anim.UpdateNodeColor(wifiSTAnodes.Get(i), 0, 255, 0); // green
    //     anim.UpdateNodeSize(wifiSTAnodes.Get(i)->GetId(), 5.0, 5.0);
    //     anim.UpdateNodeDescription(wifiSTAnodes.Get(i), "STA" + std::to_string(i + 1));
    // };

    // for (int i = 0; i < STA_COUNT; i++) {
    //     setupSTAAnimation(i);
    // }

    // auto scheduleClients = [&](int sta_i, double reassocPeriod, double delta) {
    //     // Get pointer to the original station mac object for client
    //     Ptr<WifiNetDevice> staNetDevMain = DynamicCast<WifiNetDevice>(staNetDev[2*sta_i].Get(0));
    //     Ptr<StaWifiMac> staMacMain = DynamicCast<StaWifiMac>(staNetDevMain->GetMac());
    //
    //     // Get pointer to the measurement station mac object for client
    //     Ptr<WifiNetDevice> staNetDevScan = DynamicCast<WifiNetDevice>(staNetDev[2*sta_i + 1].Get(0));
    //     Ptr<RriModuleMac> rriMac = DynamicCast<RriModuleMac>(staNetDevScan->GetMac());
    //
    //     Simulator::Schedule(Seconds(reassocPeriod - delta), &rriModStaSnrData, rriMac); // measure snr before reassoc attempt
    //     Simulator::Schedule(Seconds(reassocPeriod), &AssociateWithBestSNR, staMacMain, rriMac); // reassociate
    //     Simulator::Schedule(Seconds(reassocPeriod + delta), &rriModStaSnrData, rriMac); // measure snr after reassoc attempt
    // };

    auto scheduleAp = [&](int ap_i) {
        // Get pointer to the original station mac object for client
        Ptr<WifiNetDevice> apNetDevMain = DynamicCast<WifiNetDevice>(apNetDevices[2*ap_i].Get(0));
        Ptr<ApWifiMac> apMacMain = DynamicCast<ApWifiMac>(apNetDevMain->GetMac());

        // Get pointer to the measurement station mac object for client
        Ptr<WifiNetDevice> apNetDevScan = DynamicCast<WifiNetDevice>(apNetDevices[2*ap_i + 1].Get(0));
        Ptr<RriModuleMac> rriMac = DynamicCast<RriModuleMac>(apNetDevScan->GetMac());
        Simulator::Schedule(Seconds(2.0), &RriModuleMac::Scan, rriMac);
    };

    // auto scheduleOtherAps = [&](int except_i) {
    //     std::random_device rd;
    //     std::mt19937 rng(rd());
    //     std::uniform_int_distribution<> uniform_dis(0, avail_channels.size() - 1);
    //     auto chan_it = avail_channels.begin();
    //     for (int i = 0; i < AP_COUNT; i++) {
    //         if (i != except_i) {
    //             double channelSwitchPeriod = i+2;
    //             // pick random channel to use
    //             std::advance(chan_it, uniform_dis(rng));
    //
    //             Ptr<WifiNetDevice> apNetDevMain = DynamicCast<WifiNetDevice>(apNetDevices[2*i].Get(0));
    //             Ptr<ApWifiMac> apMacMain = DynamicCast<ApWifiMac>(apNetDevMain->GetMac());
    //             Ptr<WifiNetDevice> apNetDevScan = DynamicCast<WifiNetDevice>(apNetDevices[2*i + 1].Get(0));
    //             Ptr<RriModuleMac> rriMac = DynamicCast<RriModuleMac>(apNetDevScan->GetMac());
    //             uint8_t chan = *chan_it;
    //             Simulator::Schedule(Seconds(channelSwitchPeriod), &setChannelEvent, apMacMain, rriMac, wifiAPnodes.Get(i), chan);
    //         }
    //     }
    // };

    NS_LOG_INFO("Scheduling AP 0 to use Least Congested Channel Scan.");
    scheduleAp(0);
    // NS_LOG_INFO("Scheduling other APs to switch to random channels.");
    // scheduleOtherAps(0);

    // scheduleClients(0, 5.0, 0.1);
    // scheduleClients(1, 5.2, 0.1);

    Simulator::Stop(Seconds(20.0));

    // Enable tracing for assoc / deassoc
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc",
    //                 MakeCallback(&TraceAssoc));
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/DeAssoc",
    //                 MakeCallback(&TraceDeassoc));
    //
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::ApWifiMac/MacRx", MakeCallback(&traceRx));

    // Start the simulation
    NS_LOG_INFO("Starting simulation.");
    // Get pointer to the original station mac object for client
    // Get pointer to the measurement station mac object for client
    Ptr<WifiNetDevice> apNetDevScan = DynamicCast<WifiNetDevice>(apNetDevices[1].Get(0));
    Ptr<RriModuleMac> rriMac = DynamicCast<RriModuleMac>(apNetDevScan->GetMac());
    Simulator::Run();
    rriModStaSnrData(rriMac);

    // // Calculating throughput on each client
    // auto printClientThroughput = [](ApplicationContainer sink, int client_name) {
    //     cout << "----- Client " << client_name << " -----" << endl;
    //     double totalPacketsThrough = DynamicCast<PacketSink>(sink.Get(0))->GetTotalRx();
    //     cout << "Total Packets received = " << totalPacketsThrough << endl;
    //     double thrpt = totalPacketsThrough * 8 / (2 * 1000.0);
    //     cout << "Thrpt Kbps =  " << thrpt << endl;
    // };
    // printClientThroughput(sink1, 1);
    // printClientThroughput(sink2, 2);
    // cout << "-------------------" << endl;


    Simulator::Destroy();
    // NS_LOG_INFO("Done.");
}
