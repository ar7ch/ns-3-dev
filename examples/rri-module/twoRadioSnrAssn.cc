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


 To Run: ./waf --run scratch/twoRadioSnrAssn

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

#include <cassert>
#include <iomanip>
#include <iostream>
#include <map>

using namespace ns3;
using namespace std;

const WifiStandard USED_80211_STANDARD = WIFI_STANDARD_80211ac;

NS_LOG_COMPONENT_DEFINE("twoRadio");

static void
TraceAssoc(std::string context, Mac48Address bssid)
{
    NS_LOG_INFO("Association at  " << Simulator::Now().GetSeconds() << " to " << bssid);
}

static void
TraceDeassoc(std::string context, Mac48Address bssid)
{
    NS_LOG_INFO("Deassociation at " << Simulator::Now().GetSeconds() << " from " << bssid);
}

// Function to display the map containing average SNR values of last 5 beacons.
// We display both MAC of Aps and the average snr values.
// Also display the Aps and the channel number of the Aps
void
displayStnSnr(Ptr<RriModuleMac> rriMod)
{
    int chNo;
    Ssid apSsid;
    char* ssidVal;

    cout << endl;
    cout << "For Client scanned Radio Mac Address = " << rriMod->GetAddress() << endl;
    cout << "-------------------------------------------------------------" << endl;

    std::map<Mac48Address, std::pair<double, Ssid>> mapapSnrSsid = rriMod->mapapSnrSsid;

    /* Create an iterator, much like vector iterators */
    std::map<Mac48Address, std::pair<double, Ssid>>::iterator it1;
    std::map<Mac48Address, int>::iterator it2;

    /* Test in APChnl Map */
    std::map<Mac48Address, int> mapAPchn = rriMod->mapAPchn;

    cout << "AP Mac Address : \t  Channel No  " << endl;
    cout << "---------------------------------" << endl;
    for (it2 = mapAPchn.begin(); it2 != mapAPchn.end(); ++it2)
    {
        cout << setw(20) << it2->first << setw(10) << it2->second << "\n";
    }

    cout << endl;
    cout << "\tTotal size of Mapbssnr : " << mapapSnrSsid.size() << endl; /* Output the size */

    cout << "AP Mac Address : \t  Av Snr \t Channel No \t  Ssid" << endl;
    cout << "------------------------------------------------------- --------" << endl;
    cout << endl;
    for (it1 = mapapSnrSsid.begin(); it1 != mapapSnrSsid.end(); ++it1)
    {
        chNo = mapAPchn[it1->first];

        apSsid = it1->second.second;
        ssidVal = apSsid.PeekString();

        /* Output first , macid , snr value , channel no, ssid */
        cout << setw(20) << it1->first << "\t" << setw(10) << it1->second.first << " \t \t" << chNo
             << "\t" << ssidVal << "\n";
    }

    cout << endl; /* Print a new line */
}

// Function to Associate with the AP with best snr value
void
AssociateWithBestSNR(Ptr<StaWifiMac> staMac1, Ptr<RriModuleMac> rriMod)
{
    std::cout << "\n";
    std::cout << "Inside Associate with Best: (" << staMac1->GetAddress() << ")" << std::endl;
    std::cout << "*************************** ";
    std::cout << "\n";

    // Get access to the mapapSnrSsid data structure of the client from the new mac
    std::map<Mac48Address, std::pair<double, Ssid>> mapapSnrSsid = rriMod->mapapSnrSsid;
    std::map<Mac48Address, int> mapAPchn = rriMod->mapAPchn;

    std::map<Mac48Address, std::pair<double, Ssid>>::iterator it1;

    Mac48Address temp_add;
    double max = 0;
    for (it1 = mapapSnrSsid.begin(); it1 != mapapSnrSsid.end(); ++it1)
    {
        std::cout << "MAC Address Of AP: " << it1->first << " ; "
                  << "SINR Value: " << it1->second.first << std::endl;
        // Check the SNR value
        if ((it1->second.first) > max)
        {
            temp_add = it1->first;
            max = it1->second.first;
        }
    }
    std::cout << "\n";
    /* from artem: GetBssid API changed since this code was written;
     * now it expects link_id.
     * given that our stations keeps strictly 1 link (to an AP) at a time, I can fetch it from the GetLinkIds map */
    auto& m_link_ids = staMac1->GetLinkIds();
    assert(m_link_ids.size() == 1);
    uint8_t link_id = *m_link_ids.begin();

    if (temp_add != staMac1->GetBssid(link_id))
    {
        Ssid apSsid = mapapSnrSsid[temp_add].second;

        int ch = mapAPchn[temp_add];
        /* from artem: simply using channel number as integer is not possible anymore */
        WifiPhyOperatingChannel channel;
        constexpr uint16_t chan_freq = 0;
        constexpr uint16_t width = 20;
        constexpr WifiPhyBand band = WIFI_PHY_BAND_5GHZ;
        channel.Set(ch, chan_freq, width, USED_80211_STANDARD, band);
        std::cout << "Setting channel to " << ch << endl;
        staMac1->GetWifiPhy()->SetOperatingChannel(channel);

        // Associate to the AP using ssid

        std::cout << "Client associating to AP with maximum SNR: " << max << "\n";
        std::cout << "****************************************************" << std::endl;

        std::cout << " MAC address \t Channel No, \t SSid: "
                  << "\n";

        std::cout << temp_add << "\t" << ch << "\t" << apSsid.PeekString() << "\n" << std::endl;

        Ptr<RegularWifiMac> regStmac = DynamicCast<RegularWifiMac>(staMac1);
        regStmac->SetSsid(apSsid);
    }
    else
    {
        std::cout << "Client already associated to the AP with MAC address: " << temp_add << "\n";
        std::cout << "\n";
    }
}

int
main(int argc, char* argv[])
{
    LogComponentEnable("twoRadio", LOG_LEVEL_INFO);

    double start_scanning = 2.0;
    double scan_duration = 1.0;
    int rrm_channels_to_scan[3] = {36, 36, 36};
    // double snrThreshold = 60;

    double txgain = 0.0;
    double PathLossExponent = 3.5;
    double CCAThreshold = -85.0;

    CommandLine cmd;
    cmd.AddValue("StartRRM", "Start Time of RRI Module", start_scanning);
    cmd.AddValue("ScanDurtion", "Duration to stay in channel", scan_duration);
    cmd.Parse(argc, argv);

    NS_LOG_INFO("Create 4 nodes.");

    // server node
    NodeContainer ServerNode;
    ServerNode.Create(1);

    // Create Wi-Fi APs and stations
    NodeContainer wifiApNode; // Container for 4 APs
    wifiApNode.Create(4);

    NodeContainer wifiStaNode;
    wifiStaNode.Create(2); // 2 STAs

    // Create p2p links from each AP to Server
    NodeContainer n0n1 = NodeContainer(ServerNode.Get(0), wifiApNode.Get(0)); // server and Ap1
    NodeContainer n0n2 = NodeContainer(ServerNode.Get(0), wifiApNode.Get(1)); // server and Ap2
    NodeContainer n0n3 = NodeContainer(ServerNode.Get(0), wifiApNode.Get(2)); // server and Ap3
    NodeContainer n0n4 = NodeContainer(ServerNode.Get(0), wifiApNode.Get(3)); // server and Ap4

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
    p2p.SetChannelAttribute("Delay", StringValue("1us"));

    // Install P2P links
    NetDeviceContainer p2pDevices1 = p2p.Install(n0n1);
    NetDeviceContainer p2pDevices2 = p2p.Install(n0n2);
    NetDeviceContainer p2pDevices3 = p2p.Install(n0n3);
    NetDeviceContainer p2pDevices4 = p2p.Install(n0n4);

    // Enable the beacon jitter for the MAC of APs
    Config::SetDefault("ns3::ApWifiMac::EnableBeaconJitter", BooleanValue(true));

    // Create wireless channels
    NS_LOG_INFO("Create channels.");

    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                               "Exponent",
                               DoubleValue(PathLossExponent));

    YansWifiPhyHelper phy = YansWifiPhyHelper();
    phy.Set("TxGain", DoubleValue(txgain));
    phy.Set("RxGain", DoubleValue(0));
    phy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
    // phy.Set("ShortGuardEnabled", BooleanValue(true)); // deprecated
    phy.SetChannel("{36, 20, BAND_5GHZ, 0}");
    phy.Set("CcaEdThreshold", DoubleValue(CCAThreshold));
    // create the channel object and associate to PHY layer object
    phy.SetChannel(channel.Create());

    // Set Wifi Standard
    WifiHelper wifi;
    wifi.SetStandard(USED_80211_STANDARD);

    StringValue phyRate;
    phyRate = StringValue("VhtMcs0");

    // use same transmission rate for all the stations
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue(phyRate),
                                 "ControlMode",
                                 StringValue(phyRate));

    // Set HT Wi-Fi MAC

    WifiMacHelper mac = WifiMacHelper();
    WifiMacHelper msrmac = WifiMacHelper();

    NetDeviceContainer wifistaDevice[4];
    Ssid ssid1 = Ssid("AP1");
    Ssid ssid2 = Ssid("AP2");
    Ssid ssid3 = Ssid("AP3");
    Ssid ssid4 = Ssid("AP4");
    Ssid ssid5 = Ssid("invalid");

    /******** Client 1 in ssid AP3 ************/

    mac.SetType("ns3::StaWifiMac",
                "Ssid",
                SsidValue(ssid5),
                "ActiveProbing",
                BooleanValue(false),
                "QosSupported",
                BooleanValue(true));

    phy.Set("TxPowerStart", DoubleValue(17.0));
    phy.Set("TxPowerEnd", DoubleValue(17.0));
    // Install Wi-Fi device on the STA Node
    wifistaDevice[0] = wifi.Install(phy, mac, wifiStaNode.Get(0)); // Orig Mac

    msrmac.SetType("ns3::RriModuleMac",
                   "GetLoadUpdates",
                   BooleanValue(false),
                   "StartTime",
                   TimeValue(Seconds(start_scanning)),
                   "ScanDuration",
                   TimeValue(Seconds(scan_duration)));
    // Install the Measurement MAC on STA Node
    wifistaDevice[1] = wifi.Install(phy, msrmac, wifiStaNode.Get(0)); // Measurement MAC

    // Set the NetDevice of the measurement mac to promiscous mode to receive data packets meant to device 1
    Ptr<WifiNetDevice> wifiNet = DynamicCast<WifiNetDevice>(wifistaDevice[1].Get(0));
    wifiNet->GetMac()->SetPromisc();

    Ptr<RriModuleMac> rriMod = DynamicCast<RriModuleMac>(wifiNet->GetMac());

    // Call function on the scanned mac radio to pass list of channels to scan
    rriMod->setChanneltoScan(rrm_channels_to_scan);

    /****** end setup for STA 1 ***********/

    /******** start setup for STA 2  ************/

    mac.SetType("ns3::StaWifiMac",
                "Ssid",
                SsidValue(ssid1),
                "ActiveProbing",
                BooleanValue(false),
                "QosSupported",
                BooleanValue(true));

    phy.Set("TxPowerStart", DoubleValue(17.0));
    phy.Set("TxPowerEnd", DoubleValue(17.0));
    wifistaDevice[2] = wifi.Install(phy, mac, wifiStaNode.Get(1)); // Orig Mac

    /// Install the Measuremnt Mac  on  client Node

    msrmac.SetType("ns3::RriModuleMac",
                   "GetLoadUpdates",
                   BooleanValue(false),
                   "StartTime",
                   TimeValue(Seconds(start_scanning)),
                   "ScanDuration",
                   TimeValue(Seconds(scan_duration)));
    wifistaDevice[3] = wifi.Install(phy, msrmac, wifiStaNode.Get(1)); // Msr Mac

    // Set the NetDevice of the msr mac to promiscous mode to receive data packets meant to device 1
    wifiNet = DynamicCast<WifiNetDevice>(wifistaDevice[3].Get(0));
    wifiNet->GetMac()->SetPromisc();

    rriMod = DynamicCast<RriModuleMac>(wifiNet->GetMac());

    // Call function on the scanned mac radio to pass list of channels to scan
    rriMod->setChanneltoScan(rrm_channels_to_scan);

    /****** end setup STA 2 ***********/

    /* Setting Attributes for AP */

    mac.SetType("ns3::ApWifiMac",
                "Ssid",
                SsidValue(ssid1),
                "BeaconGeneration",
                BooleanValue(true),
                "BeaconInterval",
                TimeValue(MicroSeconds(102400)),
                "QosSupported",
                BooleanValue(true));

    NetDeviceContainer wifiapDevice[4];

    // APs 1,2,3,4 are in channels 36,36,40 and 44 respectively
    phy.Set("TxPowerStart", DoubleValue(23.0));
    phy.Set("TxPowerEnd", DoubleValue(23.0));
    wifiapDevice[0] = wifi.Install(phy, mac, wifiApNode.Get(0)); // Add the mobile devices for AP1
    Ptr<WifiNetDevice> apNetDev1 = DynamicCast<WifiNetDevice>(wifiapDevice[0].Get(0));
    Ptr<RegularWifiMac> apMac1 =
        DynamicCast<RegularWifiMac>(DynamicCast<ApWifiMac>(apNetDev1->GetMac()));
    apMac1->SetSsid(ssid1);

    wifiapDevice[1] = wifi.Install(phy, mac, wifiApNode.Get(1)); // Add the mobile devices for AP2
    Ptr<WifiNetDevice> apNetDev2 = DynamicCast<WifiNetDevice>(wifiapDevice[1].Get(0));
    Ptr<RegularWifiMac> apMac2 =
        DynamicCast<RegularWifiMac>(DynamicCast<ApWifiMac>(apNetDev2->GetMac()));
    apMac2->SetSsid(ssid2);

    wifiapDevice[2] = wifi.Install(phy, mac, wifiApNode.Get(2)); // Add the mobile devices for AP3
    Ptr<WifiNetDevice> apNetDev3 = DynamicCast<WifiNetDevice>(wifiapDevice[2].Get(0));
    Ptr<RegularWifiMac> apMac3 =
        DynamicCast<RegularWifiMac>(DynamicCast<ApWifiMac>(apNetDev3->GetMac()));
    apMac3->SetSsid(ssid3);

    wifiapDevice[3] = wifi.Install(phy, mac, wifiApNode.Get(3)); // Add the mobile devices for AP4
    Ptr<WifiNetDevice> apNetDev4 = DynamicCast<WifiNetDevice>(wifiapDevice[3].Get(0));
    Ptr<RegularWifiMac> apMac4 =
        DynamicCast<RegularWifiMac>(DynamicCast<ApWifiMac>(apNetDev4->GetMac()));
    apMac4->SetSsid(ssid4);

    // Init mobility model. All nodes are stationary
    MobilityHelper mobility1;
    Ptr<ListPositionAllocator> positionAlloc1 = CreateObject<ListPositionAllocator>();

    positionAlloc1->Add(Vector(100.0, 20.0, 0.0)); // Server position
    positionAlloc1->Add(Vector(0.0, 10.0, 0.0));   // AP1
    positionAlloc1->Add(Vector(25.0, 10.0, 0.0));  // AP2
    positionAlloc1->Add(Vector(50.0, 10.0, 0.0));  // AP3
    positionAlloc1->Add(Vector(75.0, 10.0, 0.0));  // AP4

    positionAlloc1->Add(Vector(37.0, 15.0, 0.0)); // STA1
    positionAlloc1->Add(Vector(40.0, 15.0, 0.0)); // STA1
    mobility1.SetPositionAllocator(positionAlloc1);

    mobility1.SetMobilityModel(
        "ns3::ConstantPositionMobilityModel"); // For stationary APs Aand client
    mobility1.Install(ServerNode);
    mobility1.Install(wifiApNode);
    mobility1.Install(wifiStaNode.Get(0)); // 1 Client and 3 Aps are stationary

    mobility1.Install(wifiStaNode.Get(1)); // 2 Client

    // Using OLSR routing protocol
    OlsrHelper olsr;
    Ipv4StaticRoutingHelper staticRouting;
    Ipv4ListRoutingHelper list;
    list.Add(staticRouting, 0);
    list.Add(olsr, 10);

    // Installing internet stack
    InternetStackHelper stack;
    stack.SetRoutingHelper(list);

    stack.Install(ServerNode);
    stack.Install(wifiStaNode);
    stack.Install(wifiApNode);

    NS_LOG_INFO("Assign IP Addresses.");

    Ipv4AddressHelper address;

    // ipaddress for p2p interfaces
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer p2pInterfaces1;
    p2pInterfaces1 = address.Assign(p2pDevices1);

    address.SetBase("10.1.2.0", "255.255.255.0");
    Ipv4InterfaceContainer p2pInterfaces2 = address.Assign(p2pDevices2);

    address.SetBase("10.1.3.0", "255.255.255.0");
    Ipv4InterfaceContainer p2pInterfaces3 = address.Assign(p2pDevices3);

    address.SetBase("10.1.4.0", "255.255.255.0");
    Ipv4InterfaceContainer p2pInterfaces4 = address.Assign(p2pDevices4);

    // Assign IP address to the 2 interfaces on station nodes
    address.SetBase("10.1.5.0", "255.255.255.0");
    Ipv4InterfaceContainer StaInterface[4];
    StaInterface[0] = address.Assign(wifistaDevice[0]); // Orig Mac
    StaInterface[1] = address.Assign(wifistaDevice[1]); // Msr Mac
    StaInterface[2] = address.Assign(wifistaDevice[2]); // Orig Mac
    StaInterface[3] = address.Assign(wifistaDevice[3]); // Msr Mac

    Ipv4InterfaceContainer ApInterface[4];
    ApInterface[0] = address.Assign(wifiapDevice[0]);
    ApInterface[1] = address.Assign(wifiapDevice[1]);
    ApInterface[2] = address.Assign(wifiapDevice[2]);
    ApInterface[3] = address.Assign(wifiapDevice[3]);

    // Data connections from Server to client
    NS_LOG_INFO("Create Applications.");
    uint16_t port = 9; // Discard port (RFC 863)

    // Install the ON/OFF application downlink
    ApplicationContainer sourceApp1;
    ApplicationContainer sourceApp2;
    // OnOffHelper onoff("ns3::UdpSocketFactory", Address());

    BulkSendHelper onoff("ns3::TcpSocketFactory", Address());
    onoff.SetAttribute("MaxBytes", UintegerValue(0));

    // onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
    // onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
    // onoff.SetAttribute("DataRate", StringValue("54Kbps"));

    // install source for the client 1 on Server
    AddressValue remoteAddress1;
    remoteAddress1 = AddressValue(InetSocketAddress(StaInterface[0].GetAddress(0), port));
    onoff.SetAttribute("Remote", remoteAddress1);
    sourceApp1.Add(onoff.Install(ServerNode.Get(0)));
    sourceApp1.Start(Seconds(15.0));
    sourceApp1.Stop(Seconds(17.0));

    // install source for the client 2 on Server
    AddressValue remoteAddress2;
    remoteAddress2 = AddressValue(InetSocketAddress(StaInterface[2].GetAddress(0), port));
    onoff.SetAttribute("Remote", remoteAddress2);
    sourceApp2.Add(onoff.Install(ServerNode.Get(0)));
    sourceApp2.Start(Seconds(15.1));
    sourceApp2.Stop(Seconds(17.0));

    // Create an optional packet sink to receive these packets on

    ApplicationContainer sink1;
    ApplicationContainer sink2;

    // PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", Address ());
    AddressValue localaddress;

    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory", Address());
    localaddress = Address(InetSocketAddress(Ipv4Address::GetAny(), port));
    packetSinkHelper.SetAttribute("Local", localaddress);
    sink1.Add(packetSinkHelper.Install(wifiStaNode.Get(0))); // Install sinks on clients
    sink1.Start(Seconds(15.0));
    sink1.Stop(Seconds(17.0));

    sink2.Add(packetSinkHelper.Install(wifiStaNode.Get(1))); // Install sinks on clients
    sink2.Start(Seconds(15.1));
    sink2.Stop(Seconds(17.0));

    // Tracing, .pcap and .xml files generation
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream("twoRadio.tr");
    stack.EnableAsciiIpv4All(stream);

    phy.EnablePcapAll("twoRadio");
    AnimationInterface anim("twoRadio.xml");

    /******* Client 1 *******************/
    // Get pointer to the original station mac object for client 1
    Ptr<WifiNetDevice> stNetDev1 = DynamicCast<WifiNetDevice>(wifistaDevice[0].Get(0));
    Ptr<StaWifiMac> staMac1 = DynamicCast<StaWifiMac>(stNetDev1->GetMac());

    // Get pointer to the measurement station mac object for client 1
    Ptr<WifiNetDevice> stNetDev2 = DynamicCast<WifiNetDevice>(wifistaDevice[1].Get(0));
    Ptr<RriModuleMac> rriMod1 = DynamicCast<RriModuleMac>(stNetDev2->GetMac());

    // Display the measurement
    Simulator::Schedule(Seconds(4.9), &displayStnSnr, rriMod1);
    // Associate with AP with best Snr
    Simulator::Schedule(Seconds(5), &AssociateWithBestSNR, staMac1, rriMod1);

    Simulator::Schedule(Seconds(5.1), &displayStnSnr, rriMod1);

    /******* end Client 1 *******************/
    Simulator::Stop(Seconds(50.0));

    // This is to use the assoc trace source
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc",
                    MakeCallback(&TraceAssoc));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/DeAssoc",
                    MakeCallback(&TraceDeassoc));

    Simulator::Run();

    // Calculating throughput on each client
    double thrpt;

    uint32_t totalPacketsThrough = DynamicCast<PacketSink>(sink1.Get(0))->GetTotalRx();
    std::cout << "Total Packets received on client 1 = " << totalPacketsThrough << endl;
    thrpt = totalPacketsThrough * 8 / (2 * 1000.0);
    std::cout << " Thrpt of client 1  "
              << " Kbps =  " << thrpt << endl;

    totalPacketsThrough = DynamicCast<PacketSink>(sink2.Get(0))->GetTotalRx();
    std::cout << "Total Packets received on client 2 = " << totalPacketsThrough << endl;
    thrpt = totalPacketsThrough * 8 / (2 * 1000.0);
    std::cout << " Thrpt of client 2  "
              << " Kbps =  " << thrpt << endl;

    Simulator::Destroy();
    NS_LOG_INFO("Done.");
}
