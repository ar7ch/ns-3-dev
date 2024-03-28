/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 *
 * Authors:	S.Kalpalatha
 * 			Krishna
 *
 */


#ifndef RRI_MODULE_H
#define RRI_MODULE_H

#include "ns3/wifi-mac.h"
#include "ns3/event-id.h"
#include "ns3/packet.h"
#include "ns3/capability-information.h"

using RegularWifiMac = ns3::WifiMac;

namespace ns3  {


/**
 * \ingroup wifi
 *
 * The Wifi MAC high model for a non-AP STA in a BSS.
 */

// Kalp Modified
class RriModuleMac : public RegularWifiMac
{
public:
  static TypeId GetTypeId (void);

  RriModuleMac ();
  ~RriModuleMac () override;

  /*
   * Below functions are pure virtual functions defined in ns3::WifiMac. Hence this class has to implement those functions
   */

  /**
   * Return true if packets can be forwarded to the given destination,
   * false otherwise.
   *
   * \param to the address to which the packet should be sent
   * \return whether packets can be forwarded to the given destination
   */
  bool CanForwardPacketsTo(Mac48Address to) const override;

  /**
   * \param packet the packet to send.
   * \param to the address to which the packet should be sent.
   * \param from the address from which the packet should be sent.
   *
   * The packet should be enqueued in a TX queue, and should be
   * dequeued as soon as the DCF function determines that
   * access it granted to this MAC. The extra parameter "from" allows
   * this device to operate in a bridged mode, forwarding received
   * frames without altering the source address.
   */
  void Enqueue(Ptr<Packet> packet, Mac48Address to, Mac48Address from) override;

  /**
   * \param packet the packet to send.
   * \param to the address to which the packet should be sent.
   *
   * The packet should be enqueued in a TX queue, and should be
   * dequeued as soon as the DCF function determines that
   * access it granted to this MAC.
   */
  void Enqueue(Ptr<Packet> packet, Mac48Address to) override;


  /// Added by Kalpa
   void Scan(); // Added Code -Scanning- Function to make client scan channels
   void setChanneltoScan(int *chnlNos);
   void ScheduleEvent(bool enable);

   // For UA
   std::map <Mac48Address, std::pair<double,Ssid>> mapapSnrSsid;//Added Code -UA- Map to store average snr values from each AP and SSid
   std::map <Mac48Address,std::list<double> > snrlist;//Added Code -UA- Map to store SNR values of last 5 beacons for each AP


   // For AP Scanning
   void UpdateChannelLoad();
   std::map <Mac48Address,int> mapAPchn; //Added Code -Channel Selection- Map to store the channel number of each AP
   std::map <int,int> mapchnload; //Added Code -Channel Selection- Map to store number of APs in each channel
  /// Till here

  ///Added by KRISHNA
  std::map <Mac48Address, std::pair <Mac48Address,int >> client_ap_channel;
  std::map <Mac48Address, std::pair <int, int>> ap_channel_load;
  void UpdateLoad ();

   std::map <Mac48Address, double> mac_rssi; // For TPC
   ///Upto here


private:
   //virtual void Receive (Ptr<Packet> packet, const WifiMacHeader *hdr);
   void Receive(Ptr<const WifiMpdu> mpdu, uint8_t linkId) override;

  /// Added by Kalpa
  EventId ScanEvent;//Added Code-scanning- Channel scanning
  EventId UpdtChnlApLoadEvent;// Added Code- AP Scanning - For updating load information

  Time m_startscan;//Added code - Attribute for  Triggering the scan function at specific time
  Time m_scanduration;//Added Code - Attribute for duration for scanning each channel
  int choice; //Added Code-Scanning - To choose the  channels to scan
  int  *chnl; // Added Code - To get the channel number from the array
  /// Till here

   ///Added by KRISHNA
  Mac48Address ap_address, client_address;
  int channel, load;
  EventId ShowLoadTable;
  bool Update;
///Upto Here


};

} //namespace ns3

#endif /* RRI_MODULE_H */
