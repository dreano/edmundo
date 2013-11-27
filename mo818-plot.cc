#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/network-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/bridge-helper.h"
#include "ns3/stats-module.h"
#include "ns3/flow-monitor-module.h"
#include <vector>
#include <stdint.h>
#include <sstream>
#include <fstream>
#include <iostream>

using namespace ns3;

#define MODULE_NAME    "mo818"
#define DATA_PATH      "data/"
#define NUM_WIFIS      2
#define NUM_SAMPLES    10
#define SIM_START_TIME 0.5
#define SIM_DURATION   120.0
#define AP_RANGE       20.0
#define MIN_STAS       4
#define MAX_STAS       20
#define STA_INCR       4

#define TRAF_TYPE_CBR   0
#define TRAF_TYPE_BURST 1

#define MOB_LEVEL_LOW  0
#define MOB_LEVEL_MED  1
#define MOB_LEVEL_HIGH 2

class Experiment
{
public:
  Experiment ();
  Experiment (uint32_t mobLevel, uint32_t trafficType);
  void Run (uint32_t mobLevel, uint32_t trafficType, uint32_t enablePcap);

private:
  std::vector<NodeContainer> staNodes;
  std::vector<NetDeviceContainer> staDevices;
  std::vector<NetDeviceContainer> apDevices;
  std::vector<Ipv4InterfaceContainer> staInterfaces;
  std::vector<Ipv4InterfaceContainer> apInterfaces;
  std::string pcapPrefix;

  Gnuplot lossPlot;
  Gnuplot delayPlot;
  Gnuplot throughputPlot;
  Gnuplot2dDataset closestLossDataSet;
  Gnuplot2dDataset allLossDataSet;
  Gnuplot2dDataset fartherstLossDataSet;
  Gnuplot2dDataset closestDelayDataSet;
  Gnuplot2dDataset allDelayDataSet;
  Gnuplot2dDataset fartherstDelayDataSet;
  Gnuplot2dDataset closestThroughputDataSet;
  Gnuplot2dDataset allThroughputDataSet;
  Gnuplot2dDataset fartherstThroughputDataSet;

  Average< double > closestLossSamples;
  Average< double > allLossSamples;
  Average< double > fartherstLossSamples;
  Average< double > closestDelaySamples;
  Average< double > allDelaySamples;
  Average< double > fartherstDelaySamples;
  Average< double > closestThroughputSamples;
  Average< double > allThroughputSamples;
  Average< double > fartherstThroughputSamples;

  NetDeviceContainer connectP2pNodes (Ptr< Node > n1, Ptr< Node > n2);
  void createBss(YansWifiPhyHelper wifiPhy, 
                 Ptr< YansWifiChannel > wifiChannel,
                 Ptr< Node > apNode, Ptr< Node > routerNode, 
                 uint32_t i, uint32_t nStas,
                 Ipv4AddressHelper& ip, uint32_t mobLevel,
		 uint32_t enablePcap);
  void installApps(Ptr< Node > from, Ptr< Node > to, Ipv4Address destIp, uint32_t trafficType);
  void getStats(FlowMonitorHelper& helper, Ipv4Address closestAddress, Ipv4Address fartherstAddress, uint32_t nStas);
  void AddDataPoint(uint32_t totalStas, Gnuplot2dDataset& dataSet, Average< double >& samples);
  void RunOneSample (uint32_t sample, uint32_t nStas, uint32_t mobLevel, uint32_t trafficType, uint32_t enablePcap);
  void RunOnePass (uint32_t nStas, uint32_t mobLevel, uint32_t trafficType, uint32_t enablePcap);
};

Experiment::Experiment ()
{
}

Experiment::Experiment (uint32_t mobLevel, uint32_t trafficType)
{
  const char* mob;
  switch (mobLevel)
    {
    case MOB_LEVEL_HIGH:
      mob = "high";
      break; 
    case MOB_LEVEL_MED:
      mob = "medium";
      break; 
    case MOB_LEVEL_LOW:
    default:
      mob = "low";
      break; 
    }
  const char *traf = (trafficType == TRAF_TYPE_CBR) ? "CBR" : "burst";
  
  closestLossDataSet = Gnuplot2dDataset ("Closest");
  allLossDataSet = Gnuplot2dDataset ("Average");
  fartherstLossDataSet = Gnuplot2dDataset ("Fartherst");
  closestDelayDataSet = Gnuplot2dDataset ("Closest");
  allDelayDataSet = Gnuplot2dDataset ("Average");
  fartherstDelayDataSet = Gnuplot2dDataset ("Fartherst");
  closestThroughputDataSet = Gnuplot2dDataset ("Closest");
  allThroughputDataSet = Gnuplot2dDataset ("Average");
  fartherstThroughputDataSet = Gnuplot2dDataset ("Fartherst");

  closestLossDataSet.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  allLossDataSet.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  fartherstLossDataSet.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  closestLossDataSet.SetErrorBars (Gnuplot2dDataset::Y);
  allLossDataSet.SetErrorBars (Gnuplot2dDataset::Y);
  fartherstLossDataSet.SetErrorBars (Gnuplot2dDataset::Y);
  std::ostringstream lossFile;
  lossFile << "mo818-loss-mob-" << mobLevel << "-traf-" << trafficType << ".png";
  std::ostringstream lossTitle;
  lossTitle << "Loss: mobility " << mob << ", " << traf;
  lossPlot = Gnuplot (lossFile.str (), lossTitle.str ());
  lossPlot.AddDataset (closestLossDataSet);
  lossPlot.AddDataset (allLossDataSet);
  lossPlot.AddDataset (fartherstLossDataSet);
  lossPlot.SetLegend("# WiFi stations", "Loss (%)");

  closestDelayDataSet.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  allDelayDataSet.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  fartherstDelayDataSet.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  closestDelayDataSet.SetErrorBars (Gnuplot2dDataset::Y);
  allDelayDataSet.SetErrorBars (Gnuplot2dDataset::Y);
  fartherstDelayDataSet.SetErrorBars (Gnuplot2dDataset::Y);
  std::ostringstream delayFile;
  delayFile << "mo818-delay-mob-" << mobLevel << "-traf-" << trafficType << ".png";
  std::ostringstream delayTitle;
  delayTitle << "Delay: mobility " << mob << ", " << traf;
  delayPlot = Gnuplot (delayFile.str (), delayTitle.str ());
  delayPlot.AddDataset (closestDelayDataSet);
  delayPlot.AddDataset (allDelayDataSet);
  delayPlot.AddDataset (fartherstDelayDataSet);
  delayPlot.SetLegend("# WiFi stations", "Delay (ms)");

  closestThroughputDataSet.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  allThroughputDataSet.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  fartherstThroughputDataSet.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  closestThroughputDataSet.SetErrorBars (Gnuplot2dDataset::Y);
  allThroughputDataSet.SetErrorBars (Gnuplot2dDataset::Y);
  fartherstThroughputDataSet.SetErrorBars (Gnuplot2dDataset::Y);
  std::ostringstream throughputFile;
  throughputFile << "mo818-throughput-mob-" << mobLevel << "-traf-" << trafficType << ".png";
  std::ostringstream throughputTitle;
  throughputTitle << "Throughput: mobility " << mob << ", " << traf;
  throughputPlot = Gnuplot (throughputFile.str (), throughputTitle.str ());
  throughputPlot.AddDataset (closestThroughputDataSet);
  throughputPlot.AddDataset (allThroughputDataSet);
  throughputPlot.AddDataset (fartherstThroughputDataSet);
  throughputPlot.SetLegend("# WiFi stations", "Throughput (Kibps)");
}

NetDeviceContainer 
Experiment::connectP2pNodes (Ptr< Node > n1, Ptr< Node > n2)
{
  NodeContainer p2pNodes;
  p2pNodes.Add (n1);
  p2pNodes.Add (n2);

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer p2pDevices;
  p2pDevices = pointToPoint.Install (p2pNodes);

  //pointToPoint.EnablePcap (pcapPrefix, p2pDevices.Get (0));

  return p2pDevices;
}

void 
Experiment::createBss(YansWifiPhyHelper wifiPhy, 
                      Ptr< YansWifiChannel > wifiChannel,
                      Ptr< Node > apNode, Ptr< Node > routerNode, 
                      uint32_t i, uint32_t nStas,
                      Ipv4AddressHelper& ip, uint32_t mobLevel,
		      uint32_t enablePcap)
{
  NodeContainer closestSta;
  NodeContainer otherSta;
  NodeContainer sta;
  NetDeviceContainer staDev;
  NetDeviceContainer apDev;
  Ipv4InterfaceContainer staInterface;
  Ipv4InterfaceContainer apInterface;
  BridgeHelper bridge;
  MobilityHelper mobility;
  WifiHelper wifi = WifiHelper::Default ();
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifiPhy.SetChannel (wifiChannel);

  closestSta.Create (1);
  otherSta.Create (nStas-1);
  sta.Add (closestSta);
  sta.Add (otherSta);
  Ssid ssid = Ssid ("mo818-ssid");
  wifiMac.SetType ("ns3::ApWifiMac",
                   "Ssid", SsidValue (ssid));
  apDev = wifi.Install (wifiPhy, wifiMac, apNode);

  NetDeviceContainer bridgeDev;
  bridgeDev = bridge.Install (apNode, NetDeviceContainer (apDev, apNode->GetDevice (1)));

  // assign AP IP address to bridge, not wifi
  apInterface = ip.Assign (bridgeDev);

  if (enablePcap) wifiPhy.EnablePcap (pcapPrefix, apDev);

  // setup the STAs
  InternetStackHelper stack;
  stack.Install (sta);

  // setup mobility model for the closet station
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  if (i % 2 == 0)
    {
      positionAlloc->Add (Vector (AP_RANGE / 10.0, AP_RANGE / 4, 0.0));
    }
  else
    {
      positionAlloc->Add (Vector (1.4 * AP_RANGE, AP_RANGE / 4, 0.0));
    }
  mobility.SetPositionAllocator (positionAlloc);

  // calculate mobility model speed based on mobility level
  std::ostringstream speed;
  speed <<"ns3::ConstantRandomVariable[Constant=" << mobLevel << ".0]";
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Mode", StringValue ("Time"),
                             "Time", StringValue ("60s"),
                             "Speed", StringValue (speed.str ()),
                             "Bounds", RectangleValue (Rectangle (i * AP_RANGE / 2.0,
                                                                  i * AP_RANGE / 2.0 + AP_RANGE,
                                                                  0.0,
                                                                  AP_RANGE / 2.0)));
  mobility.Install (closestSta);

  // setup mobility model for the other stations
  if (i % 2 == 0)
    {
      mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                     "MinX", DoubleValue (2.0 * AP_RANGE / 10.0),
                                     "MinY", DoubleValue (0.0),
                                     "DeltaX", DoubleValue (AP_RANGE / 10.0),
                                     "DeltaY", DoubleValue (AP_RANGE / 10.0),
                                     "GridWidth", UintegerValue (6),
                                     "LayoutType", StringValue ("ColumnFirst"));
    }
  else
    {
      uint32_t nCols = ((nStas - 1) / 6) + (((nStas - 1) % 6 == 0) ? 0 : 1);
      mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                     "MinX", DoubleValue (AP_RANGE * (14.0 - nCols) / 10.0),
                                     "MinY", DoubleValue (0.0),
                                     "DeltaX", DoubleValue (AP_RANGE / 10.0),
                                     "DeltaY", DoubleValue (AP_RANGE / 10.0),
                                     "GridWidth", UintegerValue (6),
                                     "LayoutType", StringValue ("ColumnFirst"));
    }
  mobility.Install (otherSta);
  wifiMac.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid),
                   "ActiveProbing", BooleanValue (false));
  staDev = wifi.Install (wifiPhy, wifiMac, sta);
  staInterface = ip.Assign (staDev);

  // save everything in containers.
  staNodes.push_back (sta);
  apDevices.push_back (apDev);
  apInterfaces.push_back (apInterface);
  staDevices.push_back (staDev);
  staInterfaces.push_back (staInterface);
}

void Experiment::installApps(Ptr< Node > from, Ptr< Node > to, Ipv4Address destIp, uint32_t trafficType)
{
  uint16_t port = 12345;
  Address dest = InetSocketAddress (destIp, port);
 
  //UniformVariable uv;
  Time st =  Seconds (SIM_START_TIME); // + MilliSeconds (uv.GetValue (0.0, 500.0));
  //std::ostringstream rate;
 
  switch (trafficType)
    {
    case TRAF_TYPE_CBR:
    default:
      {
        PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory", 
				     InetSocketAddress (Ipv4Address::GetAny (), port));
        ApplicationContainer sinkApp = sinkHelper.Install (to);
        sinkApp.Start (Seconds (SIM_START_TIME));
        sinkApp.Stop (Seconds (SIM_DURATION));

        OnOffHelper onoff = OnOffHelper ("ns3::UdpSocketFactory", dest);
	// rate << 200000 + uv.GetInteger (0, 1000) << "bps";
        onoff.SetConstantRate (DataRate ("200Kbps"), 512);
        ApplicationContainer apps = onoff.Install (from);
        apps.Start (st);
        apps.Stop (Seconds (SIM_DURATION));
        break;
      }
    case TRAF_TYPE_BURST:
      {
        PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", 
				     InetSocketAddress (Ipv4Address::GetAny (), port));
        ApplicationContainer sinkApp = sinkHelper.Install (to);
        sinkApp.Start (Seconds (SIM_START_TIME));
        sinkApp.Stop (Seconds (SIM_DURATION));

        OnOffHelper clientHelper ("ns3::TcpSocketFactory", dest);
        clientHelper.SetAttribute ("OnTime", StringValue ("ns3::NormalRandomVariable[Mean=1.0|Variance=0.5]"));
        clientHelper.SetAttribute ("OffTime", StringValue ("ns3::UniformRandomVariable[Min=2.0|Max=5.0]"));
	//rate << 400000 + uv.GetInteger (0, 1000) << "bps";
        clientHelper.SetConstantRate (DataRate ("400Kbps"), 1500);

        ApplicationContainer clientApps;
        clientApps = clientHelper.Install (from);
        clientApps.Start (st);
        clientApps.Stop (Seconds (SIM_DURATION));
        break;
      }
    }
}

void Experiment::getStats(FlowMonitorHelper& helper, Ipv4Address closestAddress, Ipv4Address fartherstAddress, uint32_t nStas)
{
  uint64_t allRxBytes = 0, closestRxBytes = 0, fartherstRxBytes = 0;
  uint32_t allRxPackets = 0, closestRxPackets = 0, fartherstRxPackets = 0;
  uint32_t allTxPackets = 0, closestTxPackets = 0, fartherstTxPackets = 0;
  Time allDelaySum(0), closestDelaySum(0), fartherstDelaySum(0);

  helper.GetMonitor ()->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (helper.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = helper.GetMonitor ()->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      allRxBytes += i->second.rxBytes;
      allRxPackets += i->second.rxPackets;
      allTxPackets += i->second.txPackets;
      allDelaySum += i->second.delaySum;
      if (t.sourceAddress == closestAddress || t.destinationAddress == closestAddress)
	{
	  closestRxBytes += i->second.rxBytes;
	  closestRxPackets += i->second.rxPackets;
	  closestTxPackets += i->second.txPackets;
	  closestDelaySum += i->second.delaySum;
	}
      if (t.sourceAddress == fartherstAddress || t.destinationAddress == fartherstAddress)
	{
	  fartherstRxBytes += i->second.rxBytes;
	  fartherstRxPackets += i->second.rxPackets;
	  fartherstTxPackets += i->second.txPackets;
	  fartherstDelaySum += i->second.delaySum;
	}
    }

  double closestLoss = (closestTxPackets == 0) ? 0.0 :
    100 * (1.0 - ((double) closestRxPackets) / ((double) closestTxPackets));
  double closestDelay = (closestRxPackets == 0) ? 0.0 :
    ((double) closestDelaySum.GetMilliSeconds ()) / ((double) closestRxPackets);
  double closestThroughput = closestRxBytes * 8.0 / (SIM_DURATION * 1000.0);

  double allLoss = (allTxPackets == 0) ? 0.0 :
    100 * (1.0 - ((double) allRxPackets) / ((double) allTxPackets));
  double allDelay = (allRxPackets == 0) ? 0.0 :
    ((double) allDelaySum.GetMilliSeconds ()) / ((double) allRxPackets);
  double allThroughput = allRxBytes * 8.0 / (SIM_DURATION * 1000.0 * stats.size());

  double fartherstLoss = (fartherstTxPackets == 0) ? 0.0 :
    100 * (1.0 - ((double) fartherstRxPackets) / ((double) fartherstTxPackets));
  double fartherstDelay = (fartherstRxPackets == 0) ? 0.0 :
    ((double) fartherstDelaySum.GetMilliSeconds ()) / ((double) fartherstRxPackets);
  double fartherstThroughput = fartherstRxBytes * 8.0 / (SIM_DURATION * 1000.0);

  closestLossSamples.Update (closestLoss);
  allLossSamples.Update (allLoss);
  fartherstLossSamples.Update (fartherstLoss);

  closestDelaySamples.Update (closestDelay);
  allDelaySamples.Update (allDelay);
  fartherstDelaySamples.Update (fartherstDelay);

  closestThroughputSamples.Update (closestThroughput);
  allThroughputSamples.Update (allThroughput);
  fartherstThroughputSamples.Update (fartherstThroughput);
}

void
Experiment::RunOneSample (uint32_t sample, uint32_t nStas, uint32_t mobLevel, uint32_t trafficType, uint32_t enablePcap)
{
  std::ostringstream pref;
  pref << DATA_PATH << MODULE_NAME << "-" << nStas * NUM_WIFIS << "-mob-" << mobLevel << "-traf-" << trafficType << "-samp-" << sample;
  pcapPrefix = pref.str ();

  NodeContainer term;
  term.Create (1);
  NodeContainer router;
  router.Create (1);

  InternetStackHelper stack;
  stack.Install (term);
  stack.Install (router);

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");

  NetDeviceContainer p2pDevices;
  p2pDevices = connectP2pNodes (term.Get (0), router.Get (0));
  Ipv4InterfaceContainer p2pInterfaces;
  p2pInterfaces = address.Assign (p2pDevices);

  NodeContainer apNodes;
  apNodes.Create (NUM_WIFIS);
  stack.Install (apNodes);

  CsmaHelper csma;
  NodeContainer backboneNodes;
  backboneNodes.Add (router);
  backboneNodes.Add (apNodes);
  NetDeviceContainer backboneDevices;
  backboneDevices = csma.Install (backboneNodes);

  Ipv4AddressHelper ip;
  ip.SetBase ("10.2.1.0", "255.255.255.0");
  
  NetDeviceContainer routerCsmaDev;
  routerCsmaDev.Add (backboneDevices.Get (0));
  ip.Assign (routerCsmaDev);

  //csma.EnablePcap (pcapPrefix, backboneDevices.Get (0));

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannelFactory = YansWifiChannelHelper::Default ();
  Ptr< YansWifiChannel > wifiChannel = wifiChannelFactory.Create ();

  // setup mobility model for APs
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, AP_RANGE / 4, 0.0));
  positionAlloc->Add (Vector (1.5 * AP_RANGE, AP_RANGE / 4, 0.0));
  MobilityHelper mobility;
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (apNodes);

  for (uint32_t i = 0; i < NUM_WIFIS; i++)
  {
    createBss (wifiPhy, wifiChannel, apNodes.Get (i), router.Get (0),
               i, nStas, ip, mobLevel, enablePcap);
  }

  // Ptr< Node > closestSta   = staNodes[0].Get (0);
  // Ptr< Node > fartherstSta = staNodes[0].Get (nStas - 1);
  // installApps (term.Get (0), closestSta, staInterfaces[0].GetAddress (0), trafficType);
  // installApps (term.Get (0), fartherstSta, staInterfaces[0].GetAddress (nStas - 1), trafficType);
  // installApps (staNodes[1].Get (0), term.Get (0), p2pInterfaces.GetAddress (0), trafficType);
  //installApps (staNodes[1].Get (nStas - 1), term.Get (0), p2pInterfaces.GetAddress (0), trafficType);

  for (uint32_t i = 0; i < nStas; i++)
  {
    installApps (term.Get (0), staNodes[0].Get (i), staInterfaces[0].GetAddress (i), trafficType);
    installApps (term.Get (0), staNodes[1].Get (i), staInterfaces[1].GetAddress (i), trafficType);
  }

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  //wifiPhy.EnablePcap (pcapPrefix, apDevices[0].Get (0));

  FlowMonitorHelper fmHelper;
  fmHelper.InstallAll ();

  Simulator::Stop (Seconds (SIM_DURATION));

  Simulator::Run ();

  getStats (fmHelper, staInterfaces[0].GetAddress (0), staInterfaces[0].GetAddress (nStas - 1), nStas);

  staNodes.clear ();
  apDevices.clear ();
  apInterfaces.clear ();
  staDevices.clear ();
  staInterfaces.clear ();

  Simulator::Destroy ();
}

void
Experiment::AddDataPoint(uint32_t totalStas, Gnuplot2dDataset& dataSet, Average< double >& samples)
{
  dataSet.Add (totalStas, samples.Avg (), samples.Error95 ());
}

void
Experiment::RunOnePass (uint32_t nStas, uint32_t mobLevel, uint32_t trafficType, uint32_t enablePcap)
{
  closestLossSamples.Reset ();
  allLossSamples.Reset ();
  fartherstLossSamples.Reset ();

  closestDelaySamples.Reset ();
  allDelaySamples.Reset ();
  fartherstDelaySamples.Reset ();

  closestThroughputSamples.Reset ();
  allThroughputSamples.Reset ();
  fartherstThroughputSamples.Reset ();

  for (uint32_t i = 1; i <= NUM_SAMPLES; i++)
    {
      RunOneSample(i, nStas, mobLevel, trafficType, enablePcap);
    }

  std::cout << "\nNumber of wireless stations: " << nStas * NUM_WIFIS << std::endl;
  std::cout << "Closest\n";
  std::cout << "  Loss:       " << closestLossSamples.Avg () 
	    << "% (error=" << closestLossSamples.Error95 () << ")\n";
  std::cout << "  Delay:      " << closestDelaySamples.Avg ()
	    << " ms (error=" << closestDelaySamples.Error95 () << ")\n";
  std::cout << "  Throughput: " << closestThroughputSamples.Avg ()
	    << " Kibps (error=" << closestThroughputSamples.Error95 () << ")\n";
  std::cout << "Average\n";
  std::cout << "  Loss:       " << allLossSamples.Avg () 
	    << "% (error=" << allLossSamples.Error95 () << ")\n";
  std::cout << "  Delay:      " << allDelaySamples.Avg ()
	    << " ms (error=" << allDelaySamples.Error95 () << ")\n";
  std::cout << "  Throughput: " << allThroughputSamples.Avg ()
	    << " Kibps (error=" << allThroughputSamples.Error95 () << ")\n";
  std::cout << "Fartherst\n";
  std::cout << "  Loss:       " << fartherstLossSamples.Avg () 
	    << "% (error=" << fartherstLossSamples.Error95 () << ")\n";
  std::cout << "  Delay:      " << fartherstDelaySamples.Avg ()
	    << " ms (error=" << fartherstDelaySamples.Error95 () << ")\n";
  std::cout << "  Throughput: " << fartherstThroughputSamples.Avg ()
	    << " Kibps (error=" << fartherstThroughputSamples.Error95 () << ")\n";

  uint32_t totalStas = nStas * 2;
  AddDataPoint (totalStas, closestLossDataSet, closestLossSamples);
  AddDataPoint (totalStas, allLossDataSet, allLossSamples);
  AddDataPoint (totalStas, fartherstLossDataSet, fartherstLossSamples);

  AddDataPoint (totalStas, closestDelayDataSet, closestDelaySamples);
  AddDataPoint (totalStas, allDelayDataSet, allDelaySamples);
  AddDataPoint (totalStas, fartherstDelayDataSet, fartherstDelaySamples);

  AddDataPoint (totalStas, closestThroughputDataSet, closestThroughputSamples);
  AddDataPoint (totalStas, allThroughputDataSet, allThroughputSamples);
  AddDataPoint (totalStas, fartherstThroughputDataSet, fartherstThroughputSamples);
}

void
Experiment::Run (uint32_t mobLevel, uint32_t trafficType, uint32_t enablePcap)
{
  std::cout << "mobLevel=" << mobLevel
            << ", trafficType=" << trafficType 
	    << ", duration of sample=" << SIM_DURATION << "s"
            << ", AP range=" << AP_RANGE << "m"
            << ", #samples=" << NUM_SAMPLES
	    << ", enablePcap=" << enablePcap
	    << std::endl;
 
  for (uint32_t nStas = MIN_STAS; nStas <= MAX_STAS; nStas += STA_INCR)
    {
      RunOnePass(nStas, mobLevel, trafficType, enablePcap);
    }
  
  std::ostringstream lossFileName;
  lossFileName << DATA_PATH << "mo818-loss-mob-" << mobLevel << "-traf-" << trafficType << ".dat";
  std::ofstream lossFile (lossFileName.str ().c_str ());
  lossPlot.GenerateOutput (lossFile);

  std::ostringstream delayFileName;
  delayFileName << DATA_PATH << "mo818-delay-mob-" << mobLevel << "-traf-" << trafficType << ".dat";
  std::ofstream delayFile (delayFileName.str ().c_str ());
  delayPlot.GenerateOutput (delayFile);

  std::ostringstream throughputFileName;
  throughputFileName << DATA_PATH << "mo818-throughput-mob-" << mobLevel << "-traf-" << trafficType << ".dat";
  std::ofstream throughputFile (throughputFileName.str ().c_str ());
  throughputPlot.GenerateOutput (throughputFile);
}

int main (int argc, char *argv[])
{
  uint32_t mobLevel = 9999;
  uint32_t trafficType = 9999;
  uint32_t enablePcap = 1;

  CommandLine cmd;
  cmd.AddValue ("mobLevel", "Mobility level [LOW(0), MED(1), HIGH(2)], default=LOW(0)", mobLevel);
  cmd.AddValue ("trafficType", "Traffic type [CBR(0), BURST(1)], default=CBR", trafficType);
  cmd.AddValue ("enablePcap", "Enable .pcap generation [no(0), yes(1)], default=yes(1)", enablePcap);
  cmd.Parse (argc, argv);

  if (mobLevel < MOB_LEVEL_LOW || mobLevel > MOB_LEVEL_HIGH ||
      trafficType < TRAF_TYPE_CBR || trafficType > TRAF_TYPE_BURST)
    {
      for (mobLevel = MOB_LEVEL_LOW; mobLevel <= MOB_LEVEL_HIGH; mobLevel++)
	{
	  for (trafficType = TRAF_TYPE_CBR; trafficType <= TRAF_TYPE_BURST; trafficType++)
	    {
	      Experiment experiment(mobLevel, trafficType);
	      experiment.Run(mobLevel, trafficType, enablePcap);
	    }
	}
    }
  else
    {
      Experiment experiment(mobLevel, trafficType);
      experiment.Run(mobLevel, trafficType, enablePcap);
    }
}
