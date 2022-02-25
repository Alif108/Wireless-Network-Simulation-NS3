/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 University of Kansas
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
 * Author: Justin Rohrer <rohrej@ittc.ku.edu>
 *
 * James P.G. Sterbenz <jpgs@ittc.ku.edu>, director
 * ResiliNets Research Group  http://wiki.ittc.ku.edu/resilinets
 * Information and Telecommunication Technology Center (ITTC)
 * and Department of Electrical Engineering and Computer Science
 * The University of Kansas Lawrence, KS USA.
 *
 * Work supported in part by NSF FIND (Future Internet Design) Program
 * under grant CNS-0626918 (Postmodern Internet Architecture),
 * NSF grant CNS-1050226 (Multilayer Network Resilience Analysis and Experimentation on GENI),
 * US Department of Defense (DoD), and ITTC at The University of Kansas.
 */

/*
 * This example program allows one to run ns-3 DSDV, AODV, or OLSR under
 * a typical random waypoint mobility model.
 *
 * By default, the simulation runs for 200 simulated seconds, of which
 * the first 50 are used for start-up time.  The number of nodes is 50.
 * Nodes move according to RandomWaypointMobilityModel with a speed of
 * 20 m/s and no pause time within a 300x1500 m region.  The WiFi is
 * in ad hoc mode with a 2 Mb/s rate (802.11b) and a Friis loss model.
 * The transmit power is set to 7.5 dBm.
 *
 * It is possible to change the mobility and density of the network by
 * directly modifying the speed and the number of nodes.  It is also
 * possible to change the characteristics of the network by changing
 * the transmit power (as power increases, the impact of mobility
 * decreases and the effective density increases).
 *
 * By default, OLSR is used, but specifying a value of 2 for the protocol
 * will cause AODV to be used, and specifying a value of 3 will cause
 * DSDV to be used.
 *
 * By default, there are 10 source/sink data pairs sending UDP data
 * at an application rate of 2.048 Kb/s each.    This is typically done
 * at a rate of 4 64-byte packets per second.  Application data is
 * started at a random time between 50 and 51 seconds and continues
 * to the end of the simulation.
 *
 * The program outputs a few items:
 * - packet receptions are notified to stdout such as:
 *   <timestamp> <node-id> received one packet from <src-address>
 * - each second, the data reception statistics are tabulated and output
 *   to a comma-separated value (csv) file
 * - some tracing and flow monitor configuration that used to work is
 *   left commented inline in the program
 */

#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/netanim-module.h"
#include<string>  
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"

#include "ns3/flow-monitor-module.h"     

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("my_network3");


Ptr<PacketSink> sink;                         /* Pointer to the packet sink application */
uint64_t lastTotalRx = 0;                     /* The value of the last total received bytes */
AsciiTraceHelper asciiTraceHelper;
Ptr<OutputStreamWrapper> throughput_stream;


// ------------------------- Custom Application Class Starts -------------------- //

class MyApp : public Application 
{
public:

  MyApp ();
  virtual ~MyApp();

  void Setup (Ptr<Socket> socket, Address sinkAddress, Address srcAddress, uint32_t packetSize, uint32_t nPackets, DataRate dataRate);

private:
  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void ScheduleTx (void);
  void SendPacket (void);

  Ptr<Socket>     m_socket;
  Address         m_self;
  Address         m_peer;
  uint32_t        m_packetSize;
  uint32_t        m_nPackets;
  DataRate        m_dataRate;
  EventId         m_sendEvent;
  bool            m_running;
  uint32_t        m_packetsSent;
};

MyApp::MyApp ()
  : m_socket (0),
    m_self (), 
    m_peer (), 
    m_packetSize (0), 
    m_nPackets (0), 
    m_dataRate (0), 
    m_sendEvent (), 
    m_running (false), 
    m_packetsSent (0)
{
}

MyApp::~MyApp()
{
  m_socket = 0;
}

void
MyApp::Setup (Ptr<Socket> socket, Address sinkAddress, Address srcAddress, uint32_t packetSize, uint32_t nPackets, DataRate dataRate)
{
  m_socket = socket;
  m_peer = sinkAddress;
  m_self = srcAddress;
  m_packetSize = packetSize;
  m_nPackets = nPackets;
  m_dataRate = dataRate;
}

void
MyApp::StartApplication (void)
{
  m_running = true;
  m_packetsSent = 0;
  m_socket->Bind ();
  m_socket->Connect (m_peer);
  SendPacket ();
}

void 
MyApp::StopApplication (void)
{
  m_running = false;

  if (m_sendEvent.IsRunning ())
    {
      Simulator::Cancel (m_sendEvent);
    }

  if (m_socket)
    {
      m_socket->Close ();
    }
}

void 
MyApp::SendPacket (void)
{
  Ptr<Packet> packet = Create<Packet> (m_packetSize);
  m_socket->Send (packet);

  // InetSocketAddress sinkAddr = InetSocketAddress::ConvertFrom (m_peer);
  // InetSocketAddress srcAddr = InetSocketAddress::ConvertFrom (m_self);

  // NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << "\t" << srcAddr.GetIpv4 ()<< "--->"<< sinkAddr.GetIpv4 ());

  if (++m_packetsSent < m_nPackets)
    {
      ScheduleTx ();
    }
}

void 
MyApp::ScheduleTx (void)
{
  if (m_running)
    {
      Time tNext (Seconds (m_packetSize * 8 / static_cast<double> (m_dataRate.GetBitRate ())));
      m_sendEvent = Simulator::Schedule (tNext, &MyApp::SendPacket, this);
    }
}

// ------------------------- Custom Application Class Ends -------------------- //

int
main (int argc, char *argv[])
{
  uint32_t port = 9;                      // receiver port
  int nWifi = 10;
  int nSinks = 2;
  int nPackets = 500;
  int nPackets_per_second = 100;
  int packetSize = 1024;                  // in bytes
  double TotalTime = 50.0;
  double m_txp = 7.5; 
  double range = 7;                       // in meters
  double distance = 5;
  uint16_t nodes_in_a_row = 2;
  double errorRate = 0.0001;
  // int nodeSpeed = 5;                   // in m/s
  int nodeToTrace = 0;
  bool vegasW = false;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("nWifi", "number of nodes", nWifi);
  cmd.AddValue ("nSinks", "number of sinks/sources", nSinks);
  cmd.AddValue ("TotalTime", "total simulation time", TotalTime);
  cmd.AddValue ("range", "transmission range of a node", range);
  cmd.AddValue ("nPackets", "total number of packets to be sent by a node", nPackets);
  cmd.AddValue ("nPackets_per_second", "number of packets sent per second", nPackets_per_second);
  cmd.AddValue ("distance", "distance between the nodes", distance);
  cmd.AddValue ("nodes_in_a_row", "how many nodes will remain in a row", nodes_in_a_row);
  cmd.AddValue ("errorRate", "error rate of wifi physical", errorRate);
  // cmd.AddValue ("nodeSpeed", "speed of nodes", nodeSpeed);
  cmd.AddValue ("nodeToTrace", "node to trace congestion window", nodeToTrace);
  cmd.AddValue ("vegasW", "apply vegasW", vegasW);
  cmd.Parse (argc, argv);

  if(nSinks > nWifi/2)
  {
    std::cout<<"nSinks should be less than nWifi/2"<<std::endl;
    exit(1);
  }


  std::string tcpVariant;
  if(vegasW)
    tcpVariant = "VegasW";
  else
    tcpVariant = "Vegas";

  std::string dataRate;
  dataRate = std::to_string(nPackets_per_second * packetSize * 8/(std::pow(2,20))) + "Mbps";


  // ---------------------- Configure Network ---------------- //

  Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TypeId::LookupByName ("ns3::Tcp" + tcpVariant)));  
  Config::SetDefault ("ns3::RangePropagationLossModel::MaxRange", DoubleValue (range));


  // -------------- create directory for output files ------------ //
  std::string dir = "TaskA_High_Rate_Output/";
  std::string dirToSave = "mkdir -p " + dir;
  if (system (dirToSave.c_str ()) == -1)
  {
    std::cout<<"Could Not Create Output Directory"<<std::endl;
    exit (1);
  }
  std::string filePrefix = "TaskA_High";


  // --------------- Create Nodes ------------ //

  NodeContainer adhocNodes;
  adhocNodes.Create (nWifi);


  // ------------ setting up wifi phy and channel using helpers ------------ //

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel");
  wifiPhy.SetChannel (wifiChannel.Create ());



  //  ---------------- Add a mac ---------------- //
  
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::AarfWifiManager");

  wifiPhy.Set ("TxPowerStart",DoubleValue (m_txp));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txp));


  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);



  // ------------- Error Model -------------- //

  wifiPhy.SetErrorRateModel ("ns3::YansErrorRateModel");
  Ptr<RateErrorModel> em = CreateObject<RateErrorModel> ();
  em->SetAttribute ("ErrorRate", DoubleValue(errorRate));

  for(int i=0; i<nWifi; i++)
  {
    Config::Set("/NodeList/" + std::__cxx11::to_string(i) + "/DeviceList/0/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/PostReceptionErrorModel", PointerValue(em));
  }
  


  // --------------- Position and Mobility --------------- //

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (distance),
                                 "DeltaY", DoubleValue (distance),
                                 "GridWidth", UintegerValue (nodes_in_a_row),
                                 "LayoutType", StringValue ("RowFirst"));
  
  // std::stringstream ssSpeed;
  // ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";

  // mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
  //                            "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)),
  //                            "Speed", StringValue (ssSpeed.str ()));
  
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (adhocNodes);



  // ------------- Routing ------------ //

  AodvHelper aodv;
  Ipv4ListRoutingHelper list;
  InternetStackHelper internet;

  list.Add (aodv, 100);

  internet.SetRoutingHelper (list);
  internet.Install (adhocNodes);
  


  // --------------- assigning ip address ---------------- //

  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = addressAdhoc.Assign (adhocDevices);


  // Here, the same node acts as source and sink
  // So, setup is done in two steps
  // nSinks has to be less than nWifi/2
  // otherwise, two application will try to install on same port
  // and throw an error 
  for (int i = 0; i < nSinks; i++)
    {
      // ---------- sink ------------ //
      
      Ptr<Node> sinkNode1 = adhocNodes.Get (i);
      Address sinkAddress1, anyAddress1;
      sinkAddress1 = InetSocketAddress (adhocInterfaces.GetAddress (i), port);
      anyAddress1 = InetSocketAddress (Ipv4Address::GetAny (), port);

      PacketSinkHelper sinkHelper1 ("ns3::TcpSocketFactory", anyAddress1);
      ApplicationContainer sinkApp1 = sinkHelper1.Install (sinkNode1);

      sinkApp1.Start (Seconds (10.));
      sinkApp1.Stop (Seconds (TotalTime));


      // ---------- source --------- // 

      Ptr<Node> srcNode1 = adhocNodes.Get(nWifi - i - 1);
      Ptr<Ipv4> ipv4_1 = srcNode1->GetObject<Ipv4> ();                 // Get Ipv4 instance of the node
      Ipv4Address addr1 = ipv4_1->GetAddress (1, 0).GetLocal ();       // Get Ipv4InterfaceAddress of 1th interface.
      Address srcAddress1 = InetSocketAddress(addr1);

      Ptr<MyApp> app1 = CreateObject<MyApp> ();
      Ptr<Socket> ns3TcpSocket1 = Socket::CreateSocket (srcNode1, TcpSocketFactory::GetTypeId ());
      ns3TcpSocket1->Bind();
      
      app1->Setup (ns3TcpSocket1, sinkAddress1, srcAddress1, packetSize, nPackets, DataRate (dataRate));
      srcNode1->AddApplication(app1);
      Ptr<UniformRandomVariable> var1 = CreateObject<UniformRandomVariable> ();
      app1->SetStartTime (Seconds (var1->GetValue (10.0, 11.0)));
      app1->SetStopTime (Seconds (TotalTime));


      // ------------------- tracing congestion window ------------------ //
      // cwnd is traced on every source node 

      // Ptr<OutputStreamWrapper> stream1 = asciiTraceHelper.CreateFileStream (dir + filePrefix + "_" + std::to_string(nWifi-1-i) + ".cwnd");
      // ns3TcpSocket1->TraceConnectWithoutContext ("CongestionWindow", MakeBoundCallback (&CwndChange, stream1));
      

      // ************************ From the other way round ************************** //

      // ---------- sink ------------ //
      
      Ptr<Node> sinkNode2 = adhocNodes.Get (nWifi - i - 1);
      Address sinkAddress2, anyAddress2;
      sinkAddress2 = InetSocketAddress (adhocInterfaces.GetAddress (nWifi - i - 1), port);
      anyAddress2 = InetSocketAddress (Ipv4Address::GetAny (), port);

      PacketSinkHelper sinkHelper2 ("ns3::TcpSocketFactory", anyAddress2);
      ApplicationContainer sinkApp2 = sinkHelper2.Install (sinkNode2);

      sinkApp2.Start (Seconds (10.));
      sinkApp2.Stop (Seconds (TotalTime));


      // ---------- source --------- // 

      Ptr<Node> srcNode2 = adhocNodes.Get(i);
      Ptr<Ipv4> ipv4_2 = srcNode2->GetObject<Ipv4> ();                 // Get Ipv4 instance of the node
      Ipv4Address addr2 = ipv4_2->GetAddress (1, 0).GetLocal ();       // Get Ipv4InterfaceAddress of 1th interface.
      Address srcAddress2 = InetSocketAddress(addr2);

      Ptr<MyApp> app2 = CreateObject<MyApp> ();
      Ptr<Socket> ns3TcpSocket2 = Socket::CreateSocket (srcNode2, TcpSocketFactory::GetTypeId ());
      ns3TcpSocket2->Bind();
      
      app2->Setup (ns3TcpSocket2, sinkAddress2, srcAddress2, packetSize, nPackets, DataRate (dataRate));
      srcNode2->AddApplication(app2);
      Ptr<UniformRandomVariable> var2 = CreateObject<UniformRandomVariable> ();
      app2->SetStartTime (Seconds (var2->GetValue (10.0, 11.0)));
      app2->SetStopTime (Seconds (TotalTime));

     
      // ------------------- tracing congestion window ------------------ //   
      // cwnd is traced on every source node 

      // Ptr<OutputStreamWrapper> stream2 = asciiTraceHelper.CreateFileStream (dir + filePrefix + "_" + std::to_string(i) + ".cwnd");
      // ns3TcpSocket1->TraceConnectWithoutContext ("CongestionWindow", MakeBoundCallback (&CwndChange, stream2));
    }

    
  // --------------- generating output files -------------- //  
  AsciiTraceHelper ascii;
  wifiPhy.EnableAsciiAll(ascii.CreateFileStream(dir + filePrefix + "_phy.tr"));

  AnimationInterface anim(dir + filePrefix + "_anim.xml");

  FlowMonitorHelper flowmon;                             
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();        
  // wifiPhy.EnablePcap(dir + filePrefix, adhocDevices.Get (0));

  // Simulator::Schedule (Seconds (1.1), &CalculateThroughput);

  Simulator::Stop (Seconds (TotalTime));
  Simulator::Run ();



  // ------------------------ Network Performance Calculation ------------------------------- //

  uint32_t sentPackets = 0;         
  uint32_t receivedPackets = 0;     
  uint32_t lostPackets = 0;         

  int tcp_flows = 0;
  int count = 0;
  float avgThroughput = 0;

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier());
  std::map<FlowId, FlowMonitor::FlowStats> stats =  monitor->GetFlowStats();

  for(std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin(); iter != stats.end(); iter++)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter->first);

    NS_LOG_UNCOND("\nFlow Id: " << iter->first);
    if(t.protocol == 6)
    {
        NS_LOG_UNCOND("Protocol: TCP");
        tcp_flows++;
    }
      else
        NS_LOG_UNCOND("Protocol: UDP");
    NS_LOG_UNCOND("Src Addr: " << t.sourceAddress);
    NS_LOG_UNCOND("Dst Addr: " << t.destinationAddress);
    NS_LOG_UNCOND("Sent Packets: " << iter->second.txPackets);
    NS_LOG_UNCOND("Received Packets: " << iter->second.rxPackets);
    NS_LOG_UNCOND("Lost Packets: " << iter->second.lostPackets);
    
    NS_LOG_UNCOND("====== Metric Calculation ====== ");
    NS_LOG_UNCOND("\tPacket Delivery Ratio: " << iter->second.rxPackets*100/iter->second.txPackets << "%");
    NS_LOG_UNCOND("\tPacket Loss Ratio: " << iter->second.lostPackets * 100/iter->second.txPackets << "%");
    // NS_LOG_UNCOND("Lost Packets: " << iter->second.txPackets - iter->second.rxPackets);
    // NS_LOG_UNCOND("Packet Loss Ratio: " << (iter->second.txPackets - iter->second.rxPackets)*100/iter->second.txPackets << "%");
    NS_LOG_UNCOND("\tEnd to End Delay: " << iter->second.delaySum.GetSeconds() * 1000 / iter->second.rxPackets << " ms");
    NS_LOG_UNCOND("\tJitter: " << iter->second.jitterSum.GetSeconds() * 1000 / (iter->second.rxPackets - 1) << " ms");
    NS_LOG_UNCOND("\tThroughput: " << iter->second.rxBytes * 8.0 /(iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds())/1024 << " kbps");

    sentPackets += iter->second.txPackets;
    receivedPackets += iter->second.rxPackets;
    lostPackets += iter->second.lostPackets;
    // lostPackets += (iter->second.txPackets - iter->second.rxPackets);
    avgThroughput += iter->second.rxBytes * 8.0 /(iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds())/1024;
    count++;
  }

  avgThroughput = avgThroughput/count;
  NS_LOG_UNCOND("\n--------------- Simulation Stats ------------"<<std::endl);
  NS_LOG_UNCOND("Total Flow : " << count);
  NS_LOG_UNCOND("Total TCP Flow: " << tcp_flows);
  NS_LOG_UNCOND("Total sent packets: " << sentPackets);
  NS_LOG_UNCOND("Total Received Packets: " << receivedPackets);
  NS_LOG_UNCOND("Total Lost Packets: " << lostPackets);
  NS_LOG_UNCOND("Packet Loss Ratio: " << lostPackets*100/sentPackets << "%");
  NS_LOG_UNCOND("Packet Delivery Ratio: " << receivedPackets * 100 /sentPackets << "%");
  NS_LOG_UNCOND("Average Throughput: " << avgThroughput << " kbps");
  

  monitor->SerializeToXmlFile ((dir + filePrefix + ".xml").c_str(), false, false);

  Simulator::Destroy ();

  return 0;
}