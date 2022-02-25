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

NS_LOG_COMPONENT_DEFINE ("TaskB");


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

  // NS_LOG_UNCOND(srcAddr.GetIpv4 ()<< "--->"<< sinkAddr.GetIpv4 ());
  // NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << "\t" << "packet size: " << m_packetSize <<"\n");

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


static void
CwndChange (Ptr<OutputStreamWrapper> stream, uint32_t oldCwnd, uint32_t newCwnd)
{
  // NS_LOG_UNCOND (Simulator::Now ().GetSeconds () << "\t" << newCwnd);
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;
}


void
CalculateThroughput ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */
  double cur = (sink->GetTotalRx () - lastTotalRx) * (double) 8 / 1e5;      // Convert Application RX Packets to MBits. 
  // std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;
  *throughput_stream->GetStream() << Simulator::Now().GetSeconds() << "   " << cur << std::endl;
  lastTotalRx = sink->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput);
}

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
  int nodeToTrace = 0;
  bool vegasW = false;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("nWifi", "number of nodes", nWifi);
  cmd.AddValue ("nSinks", "number of sinks/sources", nSinks);
  cmd.AddValue ("TotalTime", "total simulation time", TotalTime);
  cmd.AddValue ("range", "transmission range of a node", range);
  cmd.AddValue ("nPackets", "total number of packets to be sent by a node", nPackets);
  cmd.AddValue ("pps", "number of packets sent per second", nPackets_per_second);
  cmd.AddValue ("distance", "distance between the nodes", distance);
  cmd.AddValue ("nodes_in_a_row", "how many nodes will remain in a row", nodes_in_a_row);
  cmd.AddValue ("errorRate", "error rate of wifi physical", errorRate);
  cmd.AddValue ("nodeToTrace", "node to trace congestion window", nodeToTrace);
  cmd.AddValue ("vegasW", "apply vegasW", vegasW);
  cmd.Parse (argc, argv);


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
  std::string dir = "TaskB_Output/";
  std::string dirToSave = "mkdir -p " + dir;
  if (system (dirToSave.c_str ()) == -1)
  {
    exit (1);
  }

  std::string filePrefix = "TaskB";



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

  for (int i = 0; i < nSinks; i++)
    {
      // ---------- sink ------------ //
      
      Ptr<Node> sinkNode = adhocNodes.Get (i);
      Address sinkAddress, anyAddress;
      sinkAddress = InetSocketAddress (adhocInterfaces.GetAddress (i), port);
      anyAddress = InetSocketAddress (Ipv4Address::GetAny (), port);

      PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", anyAddress);
      ApplicationContainer sinkApp = sinkHelper.Install (sinkNode);

      sinkApp.Start (Seconds (10.));
      sinkApp.Stop (Seconds (TotalTime));


      // ---------- source --------- // 

      Ptr<Node> srcNode = adhocNodes.Get(nWifi - i - 1);
      Ptr<Ipv4> ipv4 = srcNode->GetObject<Ipv4> ();                 // Get Ipv4 instance of the node
      Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal ();       // Get Ipv4InterfaceAddress of 1th interface.
      Address srcAddress = InetSocketAddress(addr);

      Ptr<MyApp> app = CreateObject<MyApp> ();
      Ptr<Socket> ns3TcpSocket = Socket::CreateSocket (srcNode, TcpSocketFactory::GetTypeId ());
      ns3TcpSocket->Bind();
      
      app->Setup (ns3TcpSocket, sinkAddress, srcAddress, packetSize, nPackets, DataRate (dataRate));
      srcNode->AddApplication(app);
      Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
      app->SetStartTime (Seconds (var->GetValue (10.0, 11.0)));
      app->SetStopTime (Seconds (TotalTime));


      // ------------------- tracing congestion window ------------------ //
      Ptr<OutputStreamWrapper> stream = asciiTraceHelper.CreateFileStream (dir + filePrefix + "_" + std::to_string(i) + "_" + tcpVariant + ".cwnd");
      ns3TcpSocket->TraceConnectWithoutContext ("CongestionWindow", MakeBoundCallback (&CwndChange, stream));
      
      // ------------------- tracing throughput ----------------- //
      if(i == nodeToTrace)
      {
        throughput_stream = asciiTraceHelper.CreateFileStream (dir + filePrefix + "_throughput_" + tcpVariant + ".txt");

        sink = StaticCast<PacketSink> (sinkApp.Get(0));
      }
    }

    
  AsciiTraceHelper ascii;
  wifiPhy.EnableAsciiAll(ascii.CreateFileStream(dir + filePrefix + "_phy.tr"));

  AnimationInterface anim(dir + filePrefix + "_anim.xml");

  FlowMonitorHelper flowmon;                             
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();        
  wifiPhy.EnablePcap(dir + filePrefix, adhocDevices.Get (nodeToTrace));

  Simulator::Schedule (Seconds (1.1), &CalculateThroughput);

  Simulator::Stop (Seconds (TotalTime));
  Simulator::Run ();



  // ------------------------ Network Performance Calculation ------------------------------- //

  uint32_t sentPackets = 0;         
  uint32_t receivedPackets = 0;     
  uint32_t lostPackets = 0;         

  int tcp_flows = 0;
  int j = 0;
  float avgThroughput = 0;
  Time jitter;
  Time delay;

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
    NS_LOG_UNCOND("Lost Packets: " << iter->second.txPackets - iter->second.rxPackets);\
    
    NS_LOG_UNCOND("======= Metric Calculation ========");
    NS_LOG_UNCOND("\tPacket Delivery Ratio: " << iter->second.rxPackets*100/iter->second.txPackets << "%");
    NS_LOG_UNCOND("\tPacket Loss Ratio: " << (iter->second.txPackets - iter->second.rxPackets)*100/iter->second.txPackets << "%");
    if(iter->second.rxBytes != 0)
    {
      NS_LOG_UNCOND("\tEnd to End Delay: " << iter->second.delaySum.GetSeconds() * 1000 / iter->second.rxPackets << " ms");
    }
    else
      NS_LOG_UNCOND("\tEnd to End Delay = NA");
    NS_LOG_UNCOND("\tJitter: " << iter->second.jitterSum.GetSeconds() * 1000 / (iter->second.rxPackets - 1) << " ms");
    NS_LOG_UNCOND("\tThroughput: " << iter->second.rxBytes * 8.0 /(iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds())/1024 << " kbps");

    sentPackets += iter->second.txPackets;
    receivedPackets += iter->second.rxPackets;
    lostPackets += (iter->second.txPackets - iter->second.rxPackets);
    avgThroughput += iter->second.rxBytes * 8.0 /(iter->second.timeLastRxPacket.GetSeconds() - iter->second.timeFirstTxPacket.GetSeconds())/1024;
    j++;
  }

  avgThroughput = avgThroughput/j;
  NS_LOG_UNCOND("\n--------------- Simulation Stats ------------"<<std::endl);
  NS_LOG_UNCOND("Total Flow : " << j);
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