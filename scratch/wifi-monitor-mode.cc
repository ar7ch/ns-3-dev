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
using std::cout, std::endl, std::setw;

/// True for verbose output.
static bool g_verbose = true;

/**
 * MAC-level TX trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
DevTxTrace(std::string context, Ptr<const Packet> p)
{
    if (g_verbose)
    {
        std::cout << " TX p: " << *p << std::endl;
    }
}

/**
 * MAC-level RX trace.
 *
 * \param context The context.
 * \param p The packet.
 */
void
DevRxTrace(std::string context, Ptr<const Packet> p)
{
    if (g_verbose)
    {
        std::cout << " RX p: " << *p << std::endl;
    }
}

/**
 * PHY-level RX OK trace
 *
 * \param context The context.
 * \param packet The packet.
 * \param snr The SNR.
 * \param mode The wifi mode.
 * \param preamble The preamble.
 */
void
PhyRxOkTrace(std::string context,
             Ptr<const Packet> packet,
             double snr,
             WifiMode mode,
             WifiPreamble preamble)
{
    if (g_verbose)
    {
        std::cout << "PHYRXOK mode=" << mode << " snr=" << snr << " " << *packet << std::endl;
    }
}

/**
 * PHY-level RX error trace
 *
 * \param context The context.
 * \param packet The packet.
 * \param snr The SNR.
 */
void
PhyRxErrorTrace(std::string context, Ptr<const Packet> packet, double snr)
{
    if (g_verbose)
    {
        std::cout << "PHYRXERROR snr=" << snr << " " << *packet << std::endl;
    }
}

/**
 * PHY-level TX trace.
 *
 * \param context The context.
 * \param packet The packet.
 * \param mode The wifi mode.
 * \param preamble The preamble.
 * \param txPower The TX power.
 */
void
PhyTxTrace(std::string context,
           Ptr<const Packet> packet,
           WifiMode mode,
           WifiPreamble preamble,
           uint8_t txPower)
{
    if (g_verbose)
    {
        std::cout << "PHYTX mode=" << mode << " " << *packet << std::endl;
    }
}

/**
 * PHY state trace.
 *
 * \param context The context.
 * \param start Start time of the state.
 * \param duration Duration of the state.
 * \param state The state.
 */
void
PhyStateTrace(std::string context, Time start, Time duration, WifiPhyState state)
{
    if (g_verbose)
    {
        std::cout << " state=" << state << " start=" << start << " duration=" << duration
                  << std::endl;
    }
}

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

/*
     * \param device a pointer to the net device which is calling this callback
     * \param packet the packet received
     * \param protocol the 16 bit protocol number associated with this packet.
     *        This protocol number is expected to be the same protocol number
     *        given to the Send method by the user on the sender side.
     * \param sender the address of the sender
     * \param receiver the address of the receiver
     * \param packetType type of packet received (broadcast/multicast/unicast/otherhost)
     * \returns true if the callback could handle the packet successfully, false
     *          otherwise.
     *
     **/
bool MonitoringModeRxCallback(
                     Ptr<NetDevice> device,
                     Ptr<const Packet> packet,
                     uint16_t protocol,
                     const Address& sender,
                     const Address& receiver,
                     WifiNetDevice::PacketType packetType)
{
    cout << "received packet from " << sender << " to " << receiver << endl;
    return true;
}

void switchChannel(Ptr<WifiNetDevice> dev, uint16_t operatingChannel)
{
    Ptr<WifiPhy> phy = dev->GetPhy();
    std::stringstream ss;
    ss << "{" << std::to_string(operatingChannel) << ", 20, BAND_5GHZ, 0}";
    phy->SetAttribute("ChannelSettings", StringValue(ss.str()));
    assert(phy->GetOperatingChannel().GetNumber() == operatingChannel);
    cout << "switched channel to " << operatingChannel << endl;
}

void
monitorSniffer(
        std::string context,
        Ptr<const Packet> p,
        uint16_t channelFreqMhz,
        WifiTxVector txVector,
        MpduInfo aMpdu,
        SignalNoiseDbm signalNoise,
        uint16_t staId)
{
    Ptr<Packet> packet = p->Copy();
    WifiMacHeader hdr;
    packet->RemoveHeader(hdr);
    cout << setw(7) << Simulator::Now().GetSeconds() << "s: " << hdr.GetAddr2() << " -> " << hdr.GetAddr1() <<
        " CH=" << channelFreqMhz << " SNR=" << signalNoise.signal / signalNoise.noise <<
        " fromDS=" << hdr.IsFromDs() << " beacon=" << hdr.IsBeacon() <<
        " retry=" << hdr.IsRetry()
        << endl;
}

int
main(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Print trace information if true", g_verbose);
    cmd.Parse(argc, argv);

    Packet::EnablePrinting();

    WifiHelper wifi;
    MobilityHelper mobility;
    NodeContainer stas;
    NodeContainer ap;
    NodeContainer scan_ap;
    NetDeviceContainer staDevs;
    NetDeviceContainer apDevs;
    PacketSocketHelper packetSocket;

    stas.Create(2);
    ap.Create(1);
    scan_ap.Create(1);

    // give packet socket powers to nodes.
    packetSocket.Install(stas);
    packetSocket.Install(ap);
    packetSocket.Install(scan_ap);

    wifi.SetStandard(WIFI_STANDARD_80211ac);
    WifiMacHelper wifiMac;
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiPhy.SetChannel(wifiChannel.Create());
    Ssid ssid = Ssid("wifi-default");
    // setup stas.
    wifiMac.SetType("ns3::StaWifiMac",
                    "ActiveProbing",
                    BooleanValue(false),
                    "Ssid",
                    SsidValue(ssid));
    staDevs = wifi.Install(wifiPhy, wifiMac, stas);
    // setup ap.
    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    wifi.Install(wifiPhy, wifiMac, ap);
    wifiMac.SetType("ns3::ApWifiMac",
            "Ssid", SsidValue(Ssid("scan-ap-ssid")),
            "BeaconGeneration", BooleanValue(false)
    );
    wifi.Install(wifiPhy, wifiMac, scan_ap);

    // mobility.
    mobility.Install(stas);
    mobility.Install(ap);
    mobility.Install(scan_ap);

    // why not work???
    Ptr<WifiNetDevice> apNetDev1 = DynamicCast<WifiNetDevice>(scan_ap.Get(0)->GetDevice(0));
    Ptr<WifiNetDevice> apNetDev = DynamicCast<WifiNetDevice>(ap.Get(0)->GetDevice(0));
    apNetDev1->SetPromiscReceiveCallback(MonitoringModeRxCallback);
    // apNetDev1->SetAddress(Mac48Address("00:00:dd:dd:dd:01"));
    // why not work???

    Simulator::Schedule(Seconds(1.0), &AdvancePosition, ap.Get(0));
    Simulator::Schedule(Seconds(1.25), &switchChannel, apNetDev, 40);

    PacketSocketAddress socket;
    socket.SetSingleDevice(staDevs.Get(0)->GetIfIndex());
    socket.SetPhysicalAddress(staDevs.Get(1)->GetAddress());
    socket.SetProtocol(1);

    OnOffHelper onoff("ns3::PacketSocketFactory", Address(socket));
    onoff.SetConstantRate(DataRate("500kb/s"));

    ApplicationContainer apps = onoff.Install(stas.Get(0));
    apps.Start(Seconds(0.5));
    apps.Stop(Seconds(1.5));

    Simulator::Stop(Seconds(1.5));

    // Config::Connect("/NodeList/*/DeviceList/*/Mac/MacTx", MakeCallback(&DevTxTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/Mac/MacRx", MakeCallback(&DevRxTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/Phy/State/RxOk", MakeCallback(&PhyRxOkTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/Phy/State/RxError", MakeCallback(&PhyRxErrorTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/Phy/State/Tx", MakeCallback(&PhyTxTrace));
    // Config::Connect("/NodeList/*/DeviceList/*/Phy/State/State", MakeCallback(&PhyStateTrace));
    std::stringstream ss;
    ss << "/NodeList/" << "*" << "/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx";
    Config::Connect(ss.str(), MakeCallback(&monitorSniffer));
    // Config::Connect(
    //     "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/MonitorSnifferRx",
    //     MakeCallback(&Bug2470TestCase::RxCallback, this));


    Simulator::Run();

    Simulator::Destroy();

    return 0;
}
