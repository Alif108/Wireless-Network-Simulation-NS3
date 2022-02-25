#include <fstream>
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/propagation-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/csma-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv6-flow-classifier.h"
#include "ns3/flow-monitor-helper.h"
#include <ns3/lr-wpan-error-model.h>
using namespace ns3;


bool tracing = false;
uint16_t nSourceNodes=9;
uint32_t nWsnNodes=10;
uint16_t sinkPort=9;
double start_time = 0;
double duration = 50.0;
double nodeSpeed = 5;
double stop_time;
int packetSize = 1024;
int pps = 100;
std::string filePrefix;

int main (int argc, char** argv) 
{
  
  CommandLine cmd (__FILE__);
  cmd.AddValue ("tracing", "turn on log components", tracing);
  cmd.AddValue ("nWsnNodes", "number of nodes", nWsnNodes);
  cmd.AddValue ("nSourceNodes", "number of senders", nSourceNodes);
  cmd.AddValue ("nodeSpeed", "speed of nodes", nodeSpeed);
  cmd.AddValue ("pps", "packets per second", pps);
  cmd.Parse (argc, argv);

  if( tracing ) 
  {
    LogComponentEnable("PacketSink", LOG_LEVEL_INFO);
  }

  // -------------- create directory for output files ------------ //
  std::string dir = "TaskA_Low_Rate_Output/";
  std::string dirToSave = "mkdir -p " + dir;
  if (system (dirToSave.c_str ()) == -1)
  {
    std::cout<<"Could Not Create Output Directory"<<std::endl;
    exit (1);
  }

  filePrefix = "TaskA_Low";
 
  // nWsnNodes = nSourceNodes + 1;
  stop_time = start_time + duration;

  std::cout << "------------------------------------------------------\n"; 
  std::cout << "Source Count: " << nSourceNodes << "\n"; 
  std::cout << "------------------------------------------------------\n"; 

  NodeContainer wsnNodes;
  wsnNodes.Create (nWsnNodes);

  NodeContainer wiredNodes;
  wiredNodes.Create (1);
  wiredNodes.Add (wsnNodes.Get (0));

  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (5),
                                 "DeltaY", DoubleValue (5),
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));
  // mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";

  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (-100, 100, -100, 100)),
                             "Speed", StringValue (ssSpeed.str ()));
  mobility.Install (wsnNodes);

  LrWpanHelper lrWpanHelper;
  NetDeviceContainer lrwpanDevices = lrWpanHelper.Install (wsnNodes);

  lrWpanHelper.AssociateToPan (lrwpanDevices, 0);

  InternetStackHelper internetv6;
  internetv6.Install (wsnNodes);
  internetv6.Install (wiredNodes.Get (0));

  SixLowPanHelper sixLowPanHelper;
  NetDeviceContainer sixLowPanDevices = sixLowPanHelper.Install (lrwpanDevices);

  CsmaHelper csmaHelper;
  NetDeviceContainer csmaDevices = csmaHelper.Install (wiredNodes);

  Ipv6AddressHelper ipv6;
  ipv6.SetBase (Ipv6Address ("2001:cafe::"), Ipv6Prefix (64));
  Ipv6InterfaceContainer wiredDeviceInterfaces;
  wiredDeviceInterfaces = ipv6.Assign (csmaDevices);
  wiredDeviceInterfaces.SetForwarding (1, true);
  wiredDeviceInterfaces.SetDefaultRouteInAllNodes (1);

  ipv6.SetBase (Ipv6Address ("2001:f00d::"), Ipv6Prefix (64));
  Ipv6InterfaceContainer wsnDeviceInterfaces;
  wsnDeviceInterfaces = ipv6.Assign (sixLowPanDevices);
  wsnDeviceInterfaces.SetForwarding (0, true);
  wsnDeviceInterfaces.SetDefaultRouteInAllNodes (0);

  for (uint32_t i = 0; i < sixLowPanDevices.GetN (); i++) 
  {
    Ptr<NetDevice> dev = sixLowPanDevices.Get (i);
    dev->SetAttribute ("UseMeshUnder", BooleanValue (true));
    dev->SetAttribute ("MeshUnderRadius", UintegerValue (10));
  }

  for( uint32_t i=1; i<=nSourceNodes; i++ ) 
  {
    BulkSendHelper sourceApp ("ns3::TcpSocketFactory",
                              Inet6SocketAddress (wiredDeviceInterfaces.GetAddress (0, 1), 
                              sinkPort));
    sourceApp.SetAttribute ("SendSize", UintegerValue (packetSize * pps));
    sourceApp.SetAttribute ("MaxBytes", UintegerValue (1 << 21));                             // 2 MB
    ApplicationContainer sourceApps = sourceApp.Install (wsnNodes.Get (i));
    sourceApps.Start (Seconds (start_time));
    sourceApps.Stop (Seconds (stop_time));

    PacketSinkHelper sinkApp ("ns3::TcpSocketFactory",
    Inet6SocketAddress (Ipv6Address::GetAny (), sinkPort));
    sinkApp.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
    ApplicationContainer sinkApps = sinkApp.Install (wiredNodes.Get(0));
    sinkApps.Start (Seconds (0.0));
    sinkApps.Stop (Seconds (stop_time));

    sinkPort++;
  }

  if (tracing) 
  {
    AsciiTraceHelper ascii;
    lrWpanHelper.EnableAsciiAll (ascii.CreateFileStream (dir + filePrefix + ".tr"));
    lrWpanHelper.EnablePcapAll (dir + filePrefix, false);

    csmaHelper.EnableAsciiAll (ascii.CreateFileStream (dir + filePrefix + ".tr"));
    csmaHelper.EnablePcapAll (dir + filePrefix, false);
  }

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  Simulator::Stop (Seconds (stop_time));
  Simulator::Run ();

  // ----------------------------- Network Performance Calculation ------------------------------ //

  flowmon.SerializeToXmlFile(dir + filePrefix + ".xml", false, false);

  Ptr<Ipv6FlowClassifier> classifier = DynamicCast<Ipv6FlowClassifier> (flowmon.GetClassifier6() );
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

  int count = 0;
  double total_sent_packets = 0;
  double total_received_packets = 0;
  double total_lost_packets = 0;
  double total_throughput = 0;
  Time total_delay;


  for(auto iter = stats.begin(); iter != stats.end(); ++iter)
  {
    count++;

    Ipv6FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);

    std::cout<<std::endl;

    NS_LOG_UNCOND("====Flow ID: " << iter->first << "====");
    NS_LOG_UNCOND("src addr: " << t.sourceAddress << "-- dest addr: " << t.destinationAddress);
    NS_LOG_UNCOND("Sent Packets =" <<iter->second.txPackets);
    NS_LOG_UNCOND("Received Packets =" <<iter->second.rxPackets);
    NS_LOG_UNCOND("Lost Packets =" <<iter->second.lostPackets);
    NS_LOG_UNCOND("Dropped Packet = " << iter->second.txPackets - iter->second.rxPackets - iter->second.lostPackets);
    
    NS_LOG_UNCOND("Metrics Calculated:");
    
    NS_LOG_UNCOND("\tNetwork Throughput =" <<iter->second.rxBytes * 8.0/(iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds())/1024 << " kbps");
    if(iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds() != 0)
      total_throughput += iter->second.rxBytes * 8.0/(iter->second.timeLastRxPacket.GetSeconds()-iter->second.timeFirstTxPacket.GetSeconds())/1024;

    if(iter->second.rxBytes != 0)
    {
      NS_LOG_UNCOND("\tEnd to End Delay = " << iter->second.delaySum /iter->second.rxPackets);
      total_delay += iter->second.delaySum /iter->second.rxPackets;
    }
    else
      NS_LOG_UNCOND("\tEnd to End Delay = NA");
    
    NS_LOG_UNCOND("\tPacket delivery ratio = " <<iter->second.rxPackets*100.0/iter->second.txPackets << "%");
    NS_LOG_UNCOND("\tPacket loss ratio =" << (iter->second.txPackets-iter->second.rxPackets)*100.0/iter->second.txPackets << "%");

    total_sent_packets += iter->second.txPackets;
    total_received_packets += iter->second.rxPackets;
    total_lost_packets += iter->second.lostPackets;
  }

  std::cout<<std::endl;
  NS_LOG_UNCOND (" ================ Network Metrics ================= ");
  NS_LOG_UNCOND ("Average Throughput: " << total_throughput/count << " Kbps");
  NS_LOG_UNCOND ("Average End to End Delay: " << total_delay/count);
  NS_LOG_UNCOND ("Average Delivery Ratio: " << total_received_packets * 100 / total_sent_packets << " %");
  NS_LOG_UNCOND ("Average Drop Ratio: " << total_lost_packets * 100 / total_sent_packets << " %");

  Simulator::Destroy ();

  return 0;
}

