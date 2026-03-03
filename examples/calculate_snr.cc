// Try to run the snr calculation
#include <iostream>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/leo-module.h"
#include "ns3/network-module.h"
#include "ns3/aodv-module.h"
#include "ns3/udp-server.h"
#include "ns3/topology-reader.h"
#include "ns3/channel.h"

#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"

#include <fstream>
#include <cmath>
#include <tuple>

using namespace ns3;

// SNR = Signal Power / Noise Power

// use rxPower to compute data rate
// using Shannon-Hartley theorem: C = B * log2(1 + SNR)
// where C is channel capacity (data rate), B is bandwidth, SNR is signal-to-noise ratio
// SNR in linear scale: SNR = 10^(SNR_dB/10

struct Topology {
    NetDeviceContainer utNet;
    NodeContainer users;
    NodeContainer satellites;
    std::vector<bool> nolink_BS;
    std::ofstream graph_out;
};

Topology topo;

std::vector<std::tuple<double, uint64_t, uint64_t>> time_sent_recv;
uint64_t total_sent_bytes = 0;
uint64_t total_recv_bytes = 0;

map<int, double> delay;
map<uint32_t, Time> txTime; // record send time
map<uint32_t, Time> rxTime; // record receive time
double delay_sum = 0.0;
int cnt = 0;
static void EchoTxRx (std::string context, const Ptr< const Packet > packet, const TcpHeader &header, const Ptr< const TcpSocketBase > socket)
{
    // TODO: Calculate end-to-end delay
    // Hint1: Parse the packet (you may refer context.find())
    // Hint2: Store send/arrival time for the same sequence number
    std::cout << Simulator::Now () << ":" << context << ":" << packet->GetUid() << ":" << socket->GetNode () << ":" << header.GetSequenceNumber () << std::endl;
    // Hint3: Calculate end-to-end delay

    uint32_t uid = packet->GetUid();
    if (context.find("/Tx") != std::string::npos) {
        // Record send time
        txTime[uid] = Simulator::Now();
        total_sent_bytes += packet->GetSize();
        time_sent_recv.emplace_back(Simulator::Now().GetSeconds(), total_sent_bytes, total_recv_bytes);
    } else if (context.find("/Rx") != std::string::npos) {
        // Record receive time
        rxTime[uid] = Simulator::Now();
        total_recv_bytes += packet->GetSize();
        time_sent_recv.emplace_back(Simulator::Now().GetSeconds(), total_sent_bytes, total_recv_bytes);
        // Calculate delay if send time exists
        if (txTime.find(uid) != txTime.end()) { // find send time
            delay[uid] = (rxTime[uid] - txTime[uid]).GetSeconds();
            cout << "Packet " << uid << " end-to-end delay: " << delay[uid] << "s" << endl;
            delay_sum += delay[uid];
            cnt++;
        }
    }


}

double FindDataRate(Ptr<MockNetDevice> src, double rxPower){
  double noiseDB = -90; // default noise power in dB
  double snrDB = rxPower - noiseDB;
  double se = log2(1+pow(10, snrDB/10));
  return src->GetBandwidth() * se * 1e6;

}

// find the rx power of each user-satellite link
void FindUtRxPower(vector<pair<pair<int, int>, double>> &v){
  Ptr<const Channel> c = ((topo.utNet).Get(0))->GetChannel();
  if(c == nullptr) return;
  
  Ptr<const MockChannel> mc = DynamicCast<const MockChannel>(c);
  Ptr<PropagationLossModel> pLoss = mc->GetPropagationLoss ();
  if(pLoss == 0) return;
  
  for(int i=0;i<(int)topo.users.GetN();i++){
    Ptr<const Node> src_node = topo.users.Get(i);
    Ptr<MockNetDevice> src = DynamicCast<MockNetDevice>(src_node->GetDevice(0));
    Ptr<MobilityModel> srcMob = src_node->GetObject<MobilityModel> ();
    double txPower = src->GetTxPower ();
    double rxPower = txPower;
    
    for(int j=0;j<topo.satellites.GetN();j++){
      Ptr<const Node> dst_node = topo.satellites.Get(j);
      Ptr<MobilityModel> dstMob = dst_node->GetObject<MobilityModel> ();
      // double distance = srcMob->GetDistanceFrom (dstMob);
      // cout<<"distance between "<<src_node->GetId()<<" and "<<dst_node->GetId()<<" is "<<distance<<endl;
      
      // (transmitter power, receiver position, transmitter position)
      rxPower = pLoss->CalcRxPower (txPower, srcMob, dstMob);
      if (rxPower >= -900.0){
        // cout<<"link between "<<src_node->GetId()<<" "<<dst_node->GetId()<<" has rx power: "<<rxPower<<", ";
        double dataRate = FindDataRate(src, rxPower);
        //cout<<"data rate: "<< dataRate<<" MHz"<<endl;
        //cout<<src_node->GetId()<<"\tdst_node:"<<dst_node->GetId()<<"\tdataRate:"<<dataRate<<endl;
        v.emplace_back(make_pair(src_node->GetId(), dst_node->GetId()), dataRate);
        topo.nolink_BS[i] = false;
      }
    }
  }
}


void FindRxPower(std::string context, Ptr<const MobilityModel> position){
    vector<pair<pair<int, int>, double>> v;
    FindUtRxPower(v);
	//cout << "FindRxPower called" << endl; 
	for(auto &entry : v){
		topo.graph_out << entry.first.first << "\t" << entry.first.second << "\t" << entry.second << endl;
	}
}

void connect ()
{
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Tx", MakeCallback (&EchoTxRx));
    Config::Connect ("/NodeList/*/$ns3::TcpL4Protocol/SocketList/*/Rx", MakeCallback (&EchoTxRx));
}



void initial_position (const NodeContainer &satellites, int sz)
{
    for(int i = 0; i < min((int)satellites.GetN(), sz); i++){
        // Get satellite position
        Vector pos = satellites.Get(i)->GetObject<MobilityModel>()->GetPosition();
        // Convert position to latitude & longtitude
        double r = sqrt(pos.x*pos.x + pos.y*pos.y + pos.z*pos.z);
        double lat = asin(pos.z / r) * 180.0 / M_PI;
        double longit = atan2(pos.y, pos.x) * 180 / M_PI;
        cout << "Satellite " << i << " latitude = " << lat << ", longtitude = " << longit << endl;
    }
}

NS_LOG_COMPONENT_DEFINE ("CalculateSnrExample");

int main (int argc, char *argv[])
{

    CommandLine cmd;
    std::string orbitFile;
    std::string traceFile;
    LeoLatLong source (6.06692, 73.0213);
    LeoLatLong destination (7.06692, 74.0213);
    std::string islRate = "2Gbps";
    std::string constellation = "TelesatGateway";
    double bandwidth = 20.0; //MHz
    uint16_t port = 9;
    uint32_t latGws = 20;
    uint32_t lonGws = 20;
    double duration = 10;
    bool islEnabled = false;
    bool pcap = false;
    uint64_t ttlThresh = 0;
    std::string routingProto = "aodv";

    cmd.AddValue("bandwidth", "Bandwidth for the channel in MHz", bandwidth);
    cmd.AddValue("orbitFile", "CSV file with orbit parameters", orbitFile);
    cmd.AddValue("traceFile", "CSV file to store mobility trace in", traceFile);
    cmd.AddValue("precision", "ns3::LeoCircularOrbitMobilityModel::Precision");
    cmd.AddValue("duration", "Duration of the simulation in seconds", duration);
    cmd.AddValue("source", "Traffic source", source);
    cmd.AddValue("destination", "Traffic destination", destination);
    cmd.AddValue("islRate", "ns3::MockNetDevice::DataRate");
    cmd.AddValue("constellation", "LEO constellation link settings name", constellation);
    cmd.AddValue("routing", "Routing protocol", routingProto);
    cmd.AddValue("islEnabled", "Enable inter-satellite links", islEnabled);
    cmd.AddValue("latGws", "Latitudal rows of gateways", latGws);
    cmd.AddValue("lonGws", "Longitudinal rows of gateways", lonGws);
    cmd.AddValue("ttlThresh", "TTL threshold", ttlThresh);
    cmd.AddValue("destOnly", "ns3::aodv::RoutingProtocol::DestinationOnly");
    cmd.AddValue("routeTimeout", "ns3::aodv::RoutingProtocol::ActiveRouteTimeout");
    cmd.AddValue("pcap", "Enable packet capture", pcap);
    cmd.Parse (argc, argv);

    std::streambuf *coutbuf = std::cout.rdbuf();
    // redirect cout if traceFile
    std::ofstream out;
    out.open (traceFile);
    if (out.is_open ())
    {
        std::cout.rdbuf(out.rdbuf());
    }

    LeoOrbitNodeHelper orbit;
    NodeContainer satellites;
    if (!orbitFile.empty())
    {
        satellites = orbit.Install (orbitFile);
    }
    else
    {
        satellites = orbit.Install ({ LeoOrbit (1200, 20, 5, 5) });
    }

    LeoGndNodeHelper ground;
    NodeContainer users = ground.Install (source, destination);

    LeoChannelHelper utCh;
    utCh.SetConstellation (constellation);
    utCh.SetGndDeviceAttribute("DataRate", StringValue("8kbps"));
    utCh.SetGndDeviceAttribute("BandWidth", DoubleValue(bandwidth));
    utCh.SetSatDeviceAttribute("BandWidth", DoubleValue(bandwidth));
    utCh.SetPropagationLossModelAttribute("BandWidth",DoubleValue(bandwidth));
    utCh.SetPropagationLossModelAttribute("Frequency",DoubleValue(28.5));
    NetDeviceContainer utNet = utCh.Install (satellites, users);

    initial_position(satellites, 5);

    InternetStackHelper stack;
    AodvHelper aodv;
    aodv.Set ("EnableHello", BooleanValue (false));
    //aodv.Set ("HelloInterval", TimeValue (Seconds (10)));
    if (ttlThresh != 0)
    {
        aodv.Set ("TtlThreshold", UintegerValue (ttlThresh));
        aodv.Set ("NetDiameter", UintegerValue (2*ttlThresh));
    }
    stack.SetRoutingHelper (aodv);

    // Install internet stack on nodes
    stack.Install (satellites);
    stack.Install (users);

    Ipv4AddressHelper ipv4;

    ipv4.SetBase ("10.1.0.0", "255.255.0.0");
    ipv4.Assign (utNet);

    if (islEnabled)
    {
        std::cerr << "ISL enabled" << std::endl;
        IslHelper islCh;
        islCh.SetPropagationLossModelAttribute("BandWidth",DoubleValue(bandwidth));
        islCh.SetPropagationLossModelAttribute("Frequency",DoubleValue(28.5));

        NetDeviceContainer islNet = islCh.Install (satellites);
        ipv4.SetBase ("10.2.0.0", "255.255.0.0");
        ipv4.Assign (islNet);
    }

    Ipv4Address remote = users.Get (1)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
    BulkSendHelper sender ("ns3::TcpSocketFactory",
            InetSocketAddress (remote, port));
    // Set the amount of data to send in bytes.  Zero is unlimited.
    sender.SetAttribute ("MaxBytes", UintegerValue (1024));
    sender.SetAttribute ("SendSize", UintegerValue (512));
    ApplicationContainer sourceApps = sender.Install (users.Get (0));
    sourceApps.Start (Seconds (0.0));

    //
    // Create a PacketSinkApplication and install it on node 1
    //
    PacketSinkHelper sink ("ns3::TcpSocketFactory",
            InetSocketAddress (Ipv4Address::GetAny (), port));
    ApplicationContainer sinkApps = sink.Install (users.Get (1));
    sinkApps.Start (Seconds (0.0));

    // Fix segmentation fault
   
    Simulator::Schedule(Seconds(1e-7), &connect);

    //
    // Set up tracing if enabled
    //
    if (pcap)
    {
        AsciiTraceHelper ascii;
        utCh.EnableAsciiAll (ascii.CreateFileStream ("tcp-bulk-send.tr"));
        utCh.EnablePcapAll ("tcp-bulk-send", false);
    }

    std::cerr << "LOCAL =" << users.Get (0)->GetId () << std::endl;
    std::cerr << "REMOTE=" << users.Get (1)->GetId () << ",addr=" << Ipv4Address::ConvertFrom (remote) << std::endl;

    // initialize topo
    topo.utNet = utNet;
    topo.users = users;
    topo.satellites = satellites;
    topo.nolink_BS.resize(users.GetN(), true);
    topo.graph_out.open("topo_graph.txt");


    NS_LOG_INFO ("Run Simulation.");
    Simulator::Stop (Seconds (duration));
    Simulator::Run ();
    Simulator::Destroy ();
    NS_LOG_INFO ("Done.");

    Ptr<PacketSink> sink1 = DynamicCast<PacketSink> (sinkApps.Get (0));
    std::cout << users.Get (0)->GetId () << ":" << users.Get (1)->GetId () << ": " << sink1->GetTotalRx () << std::endl;
    double avg_delay = 0;
    int cnt = 0;
    for(auto &[seq, t]: delay){
        avg_delay += delay[seq];
        cnt++;
    }

    if (cnt > 0) {
        avg_delay /= cnt;
        std::cout << "Packet average end-to-end delay is " << avg_delay << "s" << endl;
    }

    std::ofstream graph_out("graphout.txt");
    for(auto &[t, sent, recv]: time_sent_recv){
        graph_out << t << " " << sent << " " << recv << std::endl;
    }
    graph_out.close();

	topo.graph_out.close();
    out.close ();
    std::cout.rdbuf(coutbuf);

    return 0;
}