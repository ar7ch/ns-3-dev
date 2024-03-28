/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

/*
  Authors: S.Kalpalatha
           Krishna
              
 
*/

//Kalpa Modified to include new header file
#include "rri-module.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/boolean.h"
#include "mac-low.h"
#include "wifi-mac-header.h"
#include "ns3/tag.h" //Added Code -UA
#include "ns3/snr-tag.h" //Added Code -UA


namespace ns3 {

//Kalp Modified
NS_LOG_COMPONENT_DEFINE ("RriModule");

NS_OBJECT_ENSURE_REGISTERED (RriModule);

//Kalp Modified
TypeId
RriModule::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RriModule")
    .SetParent<RegularWifiMac> ()
    .SetGroupName ("Wifi")
    .AddConstructor<RriModule> ()
                     
   					
     ///Added by Kalpa			
	 //Added Code -Specify the start time for the scan logic
	.AddAttribute ("StartTime",
				   "For Triggering the scan function ",
					TimeValue (Seconds (0.0)),
					MakeTimeAccessor (&RriModule::m_startscan),
					MakeTimeChecker ()) 
					
	 ///Added Code - Attribute for setting the duration of scan time
	.AddAttribute ("ScanDuration",
				   "For setting the duration of scan time ",
					TimeValue (Seconds (0.0)),
					MakeTimeAccessor (&RriModule::m_scanduration),
					MakeTimeChecker ()) 
					
	/// Added Code- Attribute set to true always. Call the SceduleEvent function				
	.AddAttribute ("ScheduleAll",
				   "To schedule all functions",
				   BooleanValue (true),
				   MakeBooleanAccessor (&RriModule::ScheduleEvent),
				   MakeBooleanChecker ())
	///Till here
					
	 ///Added by KRISHNA    
 	.AddAttribute ("GetLoadUpdates",
				   "If this is enabled then a table is dispalyed which shows"
				   "the no.of clients communicating with their own AP's"
				   "and the channel they are present",
					BooleanValue (false),
					MakeBooleanAccessor (&RriModule::Update),
					MakeBooleanChecker ()) 
	///Upto Here
	    
  ;
  return tid;
}

RriModule::RriModule ()
  : choice(0) // Kalpa - To choose the channel to scan
   
{
  NS_LOG_FUNCTION (this);
     
}

RriModule::~RriModule ()
{
  NS_LOG_FUNCTION (this);
}

void
RriModule::ScheduleEvent(bool enable)
{
    ///Added by Kalpa
    //std::cout<<"Scheduling Scan for the first time" << std::endl;
    //std::cout<<"Start Time enterd is: "<<m_startscan<<std::endl;
    //std::cout<<"Scan Time entered is: "<<m_scanduration<<std::endl;
    
   // For scanning the different channels
   ScanEvent = Simulator::Schedule (m_startscan,&RriModule::Scan,this);
   
   // To update the channel load information
   UpdtChnlApLoadEvent = Simulator::Schedule (Time ("2s"),&RriModule::UpdateChannelLoad,this); 
  /// Till here
  
    ///Added by KRISHNA
     // To update the load information on the AP into the datastructure
   ShowLoadTable = Simulator::Schedule (Seconds(8),&RriModule::UpdateLoad,this);
   ///Upto Here
	
}
void
RriModule::Enqueue (Ptr<const Packet> packet, Mac48Address to)
{
  NS_LOG_FUNCTION (this << packet << to);
 
}


void
RriModule::Receive (Ptr<Packet> packet, const WifiMacHeader *hdr)
{
  NS_LOG_FUNCTION (this << packet << hdr);
  NS_ASSERT (!hdr->IsCtl ());
  
       
  ///Added by Kalpa  
  Mac48Address temp_address;//Added Code- UA
  temp_address=hdr->GetAddr2();//Added Code -UA- to get the source address from hdr
  
  //Added Code -UA
  SnrTag tag;
  double snrValue = 0.0, SNRValueindB = 0.0;

  //To retrieve the snr tag. Header files tag.h and snr-tag.h also added
  if (packet->PeekPacketTag(tag))
  {
     NS_LOG_DEBUG ("Received Packet with SNR = " << tag.Get());
     snrValue = tag.Get();
	 SNRValueindB = (10 * log10 (snrValue));
  }
  ///Till here
  
  
  if (hdr->GetAddr3 () == GetAddress ())
    {
      NS_LOG_LOGIC ("packet sent by us.");
      return;
    }
    
    ///Commented by KRISHNA: To make the Measruremnt MAC receive data sent to any other MAc
 /** else if (hdr->GetAddr1 () != GetAddress ()
           && !hdr->GetAddr1 ().IsGroup ())
    {
      NS_LOG_LOGIC ("packet is not for us");
      std::cout<<"Dropping packets  hdr->GetAddr1 () "<< "\t "<<hdr->GetAddr1() <<" hdr->GetAddr2" << "\t" << hdr->GetAddr2 ()<< "\n";
      NotifyRxDrop (packet);
      return;
    }*/
    
  else if (hdr->IsData ())
    {
		///Added by KRISHNA
		
		/// For finding the load on AP. Client Logic
		
		/// Whenever the client receives or sends data, it indicates that there is a communication between
		/// Client and AP. Store the ,channel number, mac address of client and AP in a datastructure
		
		/// This condition is to eliminate olsr packets sent from AP or sent by Client. We do not want those packets
		/// as those olsr packets are also treated as data
		
		if( hdr->GetAddr1 () != "ff:ff:ff:ff:ff:ff" && hdr->GetAddr3 () != "ff:ff:ff:ff:ff:ff")
		{
			///std::cout << "From SCAN MAC: Channel = " << GetWifiPhy ()->GetChannelNumber () << "   from header source Address = " << hdr->GetAddr2 () << "  from header destination Address = " << hdr->GetAddr1 () << "BSSID=  " << hdr->GetAddr3 () << std::endl;
			channel = GetWifiPhy ()->GetChannelNumber ();
			///A condition to check who is the SENDER(whether AP or Client)
			if(hdr->IsFromDs ())          ///SENDER IS AP
			{
				ap_address = hdr->GetAddr2 ();
				client_address = hdr->GetAddr1 ();
			}
			else                         ///SENDER IS CLIENT
			{
				ap_address = hdr->GetAddr1 ();
				client_address = hdr->GetAddr2 ();				
			}
			
			if (client_ap_channel.count (client_address))
			{
				client_ap_channel.erase (client_ap_channel.find (client_address));
				client_ap_channel[client_address].first = ap_address;
				client_ap_channel[client_address].second = channel;			
			}
			else
			{
				client_ap_channel[client_address].first = ap_address;
				client_ap_channel[client_address].second = channel;			
			}
		}
		///Upto Here
				   
         return;
    } 
 
  else if (hdr->IsBeacon ())
  {
      MgtBeaconHeader beacon;
      packet->RemoveHeader (beacon);
      
     ///Added by Kalpa
      ///Added Code - for AP Channel Selection. Receive beacons from all Aps. Store the AP Mac and the Channel No
      Mac48Address from = hdr->GetAddr2 ();
      if(from != GetAddress()) //If a beacon is from another AP
	  {
		  
		  if(mapAPchn.count(from))
		  {
			mapAPchn.erase(mapAPchn.find(from));
			mapAPchn[from] = GetWifiPhy()->GetChannelNumber();
		  }
		  else
		  mapAPchn[from] = GetWifiPhy()->GetChannelNumber();
	  }  
	/// Till here

      
      ///Added by Kalpa - For Snr based UA  
	  double sum_of_elems = 0,snr_average = 0;
      if (snrlist.count(temp_address))
      {
          std::list<double> presentlist;
          presentlist=snrlist[temp_address];
          if (presentlist.size()>=5)  // To store only 5 values in the list
          {
             presentlist.pop_front();
             presentlist.push_back(SNRValueindB);
             for(std::list<double>::iterator j=presentlist.begin();j!=presentlist.end();++j)
             {
                sum_of_elems = sum_of_elems + *j; //To get average
             }
          }
          else // If count is less than 5, simply push
          {
              presentlist.push_back(SNRValueindB);
              for(std::list<double>::iterator j=presentlist.begin();j!=presentlist.end();++j)
              {
                 sum_of_elems = sum_of_elems + *j;
              }
          }

          snrlist.erase (snrlist.find(temp_address));
          snrlist[temp_address] = presentlist;
          snr_average=sum_of_elems/presentlist.size();
      }
      else // Create new entry if not present in list already
      {
		 std::list<double> presentlist;
         presentlist.push_back(SNRValueindB);
         snrlist[temp_address] = presentlist;
         snr_average=SNRValueindB;
      }

          
      //The final average value of last 5 beacons from snrlist map is added to mapbsssnr. 
      // AP address is not present in list, new entry is created. Also store the SSid of AP
      
      Ssid apSsid;
      apSsid = beacon.GetSsid ();
      if (mapapSnrSsid.count(temp_address))
      {
		mapapSnrSsid.erase (mapapSnrSsid.find(temp_address));
        mapapSnrSsid[temp_address].first = snr_average;
        mapapSnrSsid[temp_address].second = apSsid;
      }
      else 
      {
        mapapSnrSsid[temp_address].first = snr_average;
        mapapSnrSsid[temp_address].second = apSsid;
      }
      /// Till here
      
       ///Added by KRISHNA: For TPC Logic
      double noise = -93.966;
      double m_RSSI = snr_average + noise;
      
      if (mac_rssi.count (temp_address))
      {
		  mac_rssi.erase (mac_rssi.find (temp_address));
		  mac_rssi[temp_address] = m_RSSI;
	  }
	  else
	  {
		  mac_rssi[temp_address] = m_RSSI;
	  }
      ///Upto here by KRISHNA

                  
       return;
    }
   
  //Invoke the receive handler of our parent class to deal with any
  //other frames. Specifically, this will handle Block Ack-related
  //Management Action frames.
  RegularWifiMac::Receive (packet, hdr);
}

/// Added By Kalpa
//Added code -Scanning- for channel scanning by the Device
void
RriModule::Scan()
{
	
	Time t = Simulator::Now();
	//std::cout<<"\nAt "<<t<<"	Scanning Channel ";
	
	// Currenly 3 channles are scanned. So after 3 channels are scanned, return to channel 1
	if ( choice == 3 )
		choice = 0;
	
	if(choice == 0)
		GetWifiPhy ()->SetChannelNumber(chnl[0]);
	if(choice == 1)
		GetWifiPhy ()->SetChannelNumber(chnl[1]);
	if(choice == 2)
		GetWifiPhy ()->SetChannelNumber(chnl[2]);
			

	//std::cout<<GetWifiPhy ()->GetChannelNumber()<<std::endl;
	//std::cout<<"..................."<<std::endl;
	choice++;

	//ScanEvent = Simulator::Schedule (Seconds(5), &StaWifiMacMsr::Scan,this);
	ScanEvent = Simulator::Schedule (m_scanduration, &RriModule::Scan,this);

}
//Added code -Scanning- to get the list of channels to scan 
void RriModule::setChanneltoScan(int *chnlNos)
{
	//for (int i=0; i < size; i++)
	chnl = chnlNos;
	//std::cout<< "From Setchannel " << chnl[0] << "\t"  << chnl[1] << "\t"  << chnl[2] << "\t" << "\n";
}	

//Added code -To update the channel load information into mapchnload datastructure
// No of Aps in each channel
void RriModule::UpdateChannelLoad()
{
	//std::cout << "UpdateChannelLoad " << "\n";
    //Get access to the mapchnload data structure of the AP
	std::map <Mac48Address, int>::iterator it;
	int cnt=0;
	
		
	mapchnload.clear();
	
	//mapchnload is updated with the help of mapAPchn by counting the number of APs in a channel
	for (it = mapAPchn.begin(); it != mapAPchn.end(); ++it)
	{
		//std::cout<<"\n"<<it->first<<" "<<it->second;
		if(mapchnload.count(it->second))
		{
			cnt = mapchnload[it->second];
			mapchnload.erase(mapchnload.find(it->second));
			cnt = cnt +1;
			mapchnload[it->second] = cnt;
			
		}
		else
		{
			mapchnload[it->second] = 1;
		}
	}
	
	UpdtChnlApLoadEvent = Simulator::Schedule (Time ("2s"), &RriModule::UpdateChannelLoad,this);
}
/// Till here

///Added by KRISHNA
void
RriModule::UpdateLoad ()
{
	if (Update)
	{
		int load = 0;
		std::map<Mac48Address,std::pair<Mac48Address, int>>::iterator it1;
		std::map<Mac48Address,std::pair<Mac48Address, int>>::iterator it2;
		std::map<Mac48Address, std::pair<int, int>>::iterator it3;
		ap_channel_load.clear();
		
		for (it1 = client_ap_channel.begin (); it1 != client_ap_channel.end (); it1++)
		{
			for (it2 = client_ap_channel.begin (); it2 != client_ap_channel.end (); it2++)
			{
				if (it1->second.first == it2->second.first)
					load++;
				else
					continue;
			}
			ap_channel_load[it1->second.first].first = it1->second.second;
			ap_channel_load[it1->second.first].second = load;
			load=0;
		}
		
		std::cout << "*********************Load Update*********************" << std::endl;
		std::cout << "Channel            AP Mac Address          Load on AP" << std::endl;
		std::cout << "-----------------------------------------------------" << std::endl;
		for (it3 = ap_channel_load.begin ();it3 != ap_channel_load.end (); it3++)
		std::cout << it3->second.first << "\t\t" << it3->first << "\t\t" << it3->second.second << std::endl;
		std::cout << "*****************************************************" << std::endl;
		std::cout << "*********************CLIENT AP***********************" << std::endl;
		std::cout << "AP Mac Address     Client Mac Address" << std::endl;
		std::cout << "-----------------------------------------------------" << std::endl;
		for (it2 = client_ap_channel.begin ();it2 != client_ap_channel.end (); it2++)
		std::cout << it2->second.first << "\t" << it2->first << std::endl;
		std::cout << "*****************************************************" << std::endl;
		
		ShowLoadTable = Simulator::Schedule (Seconds (5), &RriModule::UpdateLoad, this);
	}
}
///Upto here



} //namespace ns3
