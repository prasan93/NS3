#include <stdio.h>      
#include <stdlib.h> 
#include <cstdlib>     
#include <time.h>      
#include <ctime> 
#include <math.h>
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/olsr-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/seq-ts-header.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhocGrid");
double  senttime,delay,rcv,sqhd,bytesTotal,packetsReceived,totalDelay,nwTime;

void ReceivePacket (Ptr<Socket> socket)
{

 Ptr<Packet> pkt ;
 while (pkt = socket->Recv())
    {
 
      rcv=0;
      sqhd=0;
	  SeqTsHeader hdr = SeqTsHeader();
	  pkt->RemoveHeader (hdr);
      bytesTotal += pkt->GetSize();
      packetsReceived += 1;
      rcv = Simulator::Now().GetMilliSeconds();
      sqhd = hdr.GetTs().GetMilliSeconds();
      delay = (rcv - sqhd); //delay calculation
      nwTime = rcv/1000;
      totalDelay=totalDelay+delay/1000;
      NS_LOG_UNCOND ("Received one packet at time T = " << Simulator::Now()<<" :::  Delay = " << delay/1000 << " ms"); 
        NS_LOG_UNCOND ("Total delay " << totalDelay ); 
    }
       
}
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
	  Ptr<Packet> pkt = Create<Packet> (pktSize);
	  SeqTsHeader hdr = SeqTsHeader();
	  pkt->AddHeader(hdr);
	  socket->Send (pkt);
      if(pktCount == 1)
      {
        senttime = Simulator::Now().GetMilliSeconds();
      }
      Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
    }

  else
    {
      socket->Close ();
    }
}

int main (int argc, char *argv[])
{

  std::string phyMode ("DsssRate1Mbps");
  double distance = 500;  // m
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 1;
  int numNodes = 8;  // by default, 5x5
  int sourceNode = 8;// change source node
  int ct=1;

  double interval = 1.0; // seconds
  bool verbose = false;
  bool tracing = false;
  int sinkNode = 0;
  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("distance", "distance (m)", distance);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("tracing", "turn on ascii and pcap tracing", tracing);
  cmd.AddValue ("numNodes", "number of nodes", numNodes);
  cmd.AddValue ("sinkNode", "Receiver node number", sinkNode);
  cmd.AddValue ("sourceNode", "Sender node number", sourceNode);

  cmd.Parse (argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));

  NodeContainer c;
  c.Create (numNodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (-10) );
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add an upper mac and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));
  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, c);

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (distance),
                                 "DeltaY", DoubleValue (distance),
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  // Enable OLSR
  OlsrHelper olsr;
  Ipv4StaticRoutingHelper staticRouting;

  Ipv4ListRoutingHelper list;
  list.Add (staticRouting, 0);
  list.Add (olsr, 10);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list); // has effect on the next Install ()
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  ////////////////////////
 Ptr<Socket> recvSink[9][9];
 Ptr<Socket> source[9][9];

 for (int k=0;k<numNodes;k++){
  for (int j=0;j<numNodes;j++){

  recvSink[k][j] = Socket::CreateSocket (c.Get (j), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink[k][j]->Bind (local);
  recvSink[k][j]->SetRecvCallback (MakeCallback (&ReceivePacket));

  source[k][j] = Socket::CreateSocket (c.Get (k), tid);
  InetSocketAddress remote = InetSocketAddress (i.GetAddress (j, 0), 80);
  source[k][j]->Connect (remote);
  NS_LOG_UNCOND ("Testing from node " << k << " to " << j << " sending at" << Simulator::Now()); 
  Simulator::Schedule (Seconds (60*ct), &GenerateTraffic,
                       source[k][j], packetSize, numPackets, interPacketInterval);;
    ct=ct+1;
    }
}
  if (tracing == true)
    {
      AsciiTraceHelper ascii;
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("wifi-simple-adhoc-grid.tr"));
      wifiPhy.EnablePcap ("wifi-simple-adhoc-grid", devices);
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("wifi-simple-adhoc-grid.routes", std::ios::out);
      olsr.PrintRoutingTableAllEvery (Seconds (2), routingStream);
      Ptr<OutputStreamWrapper> neighborStream = Create<OutputStreamWrapper> ("wifi-simple-adhoc-grid.neighbors", std::ios::out);
      olsr.PrintNeighborCacheAllEvery (Seconds (2), neighborStream);
    }

  AnimationInterface anim("AGM.xml");
  anim.SetConstantPosition(c.Get(0),1000,1000);
  anim.SetConstantPosition(c.Get(1),1200,892);
  anim.SetConstantPosition(c.Get(2),1250,1433);
  anim.SetConstantPosition(c.Get(3),520,1023);
  anim.SetConstantPosition(c.Get(4),1012,600);
  anim.SetConstantPosition(c.Get(5),1100,797);
  anim.SetConstantPosition(c.Get(6),1100,391);
  anim.SetConstantPosition(c.Get(7),1375,200);
  //anim.SetConstantPosition(c.Get(7),400,1000);

  Simulator::Stop (Seconds (1950.0));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}

