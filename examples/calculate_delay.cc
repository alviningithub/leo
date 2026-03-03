/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Tim Schubert <ns-3-leo@timschubert.net>
 */

#include <iostream>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/leo-module.h"
#include "ns3/network-module.h"
#include "ns3/aodv-module.h"
#include "ns3/udp-server.h"

using namespace ns3;

map<uint32_t, double> delay;
map<uint32_t,Time> packet_send_time;

static void EchoTxRx (std::string context, const Ptr< const Packet > packet, const TcpHeader &header, const Ptr< const TcpSocketBase > socket)
{
    // TODO: Calculate end-to-end delay
    // Hint1: Parse the packet (you may refer context.find())
    std::size_t found =  context.find("Tx");
    // Hint2: Store send/arrival time for the same sequence number
    uint32_t seq_num = static_cast<uint32_t>(header.GetSequenceNumber().GetValue());
    uint32_t ack_num = static_cast<uint32_t>(header.GetAckNumber().GetValue());
    // std::cout << Simulator::Now () << ":" << context << ":seq=" << seq_num << ":ack=" << ack_num << ":" << socket->GetNode () << ":" << header.GetSequenceNumber () << std::endl;
    // Calculate end-to-end delay keyed by sequence number
    Time cur_time = Simulator::Now();
    if (found != std::string::npos)
    {
        packet_send_time[seq_num] = cur_time;
    }
    else
    {
        auto it = packet_send_time.find(seq_num);
        if (it != packet_send_time.end())
        {
            delay[seq_num] = (cur_time - it->second).ToDouble(Time::S);
            cout<<  static_cast<double>(Simulator::Now().GetMicroSeconds())/1000000.0 << " " << delay[seq_num] << endl;
        }
        else
        {
            // No matching Tx record found; ignore or log if needed
        }
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

NS_LOG_COMPONENT_DEFINE ("LeoBulkSendTracingExample");

int main (int argc, char *argv[])
{

    CommandLine cmd;
    std::string orbitFile;
    std::string traceFile;
    LeoLatLong source (6.06692, 73.0213);
    LeoLatLong destination (7.06692, 74.0213);
    std::string islRate = "2Gbps";
    std::string constellation = "TelesatGateway";
    uint16_t port = 9;
    uint32_t latGws = 20;
    uint32_t lonGws = 20;
    double duration = 100;
    bool islEnabled = false;
    bool pcap = false;
    uint64_t ttlThresh = 0;
    std::string routingProto = "aodv";
    double bandwidth = 2100.0; // MHz, accordingly to telesat const

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
    cmd.AddValue("bandwidth", "Bandwidth in MHz for path loss and data rate calculation", bandwidth);
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
        // defining orbits in code (height, inclination, satellites per plane, number of planes)
        satellites = orbit.Install ({ LeoOrbit (1200, 20, 5, 5) });
    }


    LeoGndNodeHelper ground;
    NodeContainer users = ground.Install (source, destination);


    LeoChannelHelper utCh;
    utCh.SetConstellation (constellation);
    utCh.SetGndDeviceAttribute("DataRate", StringValue("8kbps"));
    utCh.SetGndDeviceAttribute("BandWidth", DoubleValue(bandwidth));//Accordingly to telesat const
    utCh.SetSatDeviceAttribute("BandWidth", DoubleValue(bandwidth));
    utCh.SetPropagationLossModelAttribute("BandWidth",DoubleValue(bandwidth));
    utCh.SetPropagationLossModelAttribute("Frequency",DoubleValue(28.5)); 
    NetDeviceContainer utNet = utCh.Install (satellites, users);
    Ptr<MockNetDevice> single = DynamicCast<MockNetDevice>(utNet.Get(0)); 
    cout<<single->GetBandwidth();

    // initial_position(satellites, 25);

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
        islCh.SetDeviceAttribute("BandWidth", DoubleValue(bandwidth));//Accordingly to telesat const
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
    sender.SetAttribute ("MaxBytes", UintegerValue (0));
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
    // Simulator::Schedule(Seconds(1e-7),&FindRxPower);

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

    NS_LOG_INFO ("Run Simulation.");
    Simulator::Stop (Seconds (duration));
    Simulator::Run ();
    Simulator::Destroy ();
    NS_LOG_INFO ("Done.");

    Ptr<PacketSink> sink1 = DynamicCast<PacketSink> (sinkApps.Get (0));
    std::cout << users.Get (0)->GetId () << ":" << users.Get (1)->GetId () << ": " << sink1->GetTotalRx () << std::endl;

    // TODO: Output End-to-end Delay
    double avg_delay = 0;
    int cnt = 0;
    out.close ();
    std::cout.rdbuf(coutbuf);

    return 0;
}