#include "HighwayProject.h"
#include "HighwayProjectXml.h"
#include "HighwayXml.h"
#include "Highway.h"
#include "HighwayConnectionXml.h"
#include "WifiConfigurationXml.h"
#include "VehicleGeneratorXml.h"
#include "VehicleGenerator.h"
#include "TrafficLightGenerator.h"
#include "ns3/simulator.h"
#include <map>
#include <limits>
#include <set>
#include <list>
#include <iostream>
#include <fstream>

using namespace std;
using namespace ns3;

/**
 * This utility function calculates the routing maps used for the Highways
 * It uses Dijkstra's algorithm to calculate a map of each HighwayId to its previous
 * HighwayId.  A node is a Highway.  There is one edge per Highway connection
 */
map<int, int> HighwayProject::Djikstra(int source, map<int, list<int> > connectionList) {
    // Create a map of distances to particular Highways
    map<int, double> distanceMap;

    // Create the retern map
    map<int, int> previousMap;
    // The set of all visited nodes
    set<int> allNodes;

    // For each of the highways in available, initialize distance to max
    for (map<int, Ptr<Highway> >::iterator it = m_highways.begin(); it != m_highways.end(); it++) {
        distanceMap[it->first] = numeric_limits<double>::max();
        allNodes.insert(it->first);
    }

    // Set the distance map distance from the source to 0
    distanceMap[source] = 0.0;

    // Until we have visited all nodes
    while (!allNodes.empty()) {
        // Get the shortest next distance
        double distance = numeric_limits<double>::max();
        // The current node to start searching from
        int currentNode = -1;
        for (set<int>::iterator it = allNodes.begin(); it != allNodes.end(); it++) {
            // Iterate over remaining nodes to find the one with the shortest distance
            if (distanceMap[(*it)] < distance) {
                distance = distanceMap[(*it)];
                currentNode = *it;
            }
        }

        // If we have nowhere left to go
        if (distance == numeric_limits<double>::max()) {
            // break
            break;
        }

        // Remove the current node from the set of nodes
        allNodes.erase(currentNode);

        // Find the current set of edges for the current node
        map<int, list<int> >::iterator it = connectionList.find(currentNode);
        if (it != connectionList.end()) {
            // For each edge
            for (list<int>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++) {
                // Check the distance along the edge
                double currentDistance = m_highways[(*it2)]->GetHighwayLength();
                // Find the edge with the shortest distance
                if (distanceMap[(*it2)] == numeric_limits<double>::max() || (distanceMap[(*it2)] < distance + currentDistance)) {
                    //Update the distance
                    distanceMap[(*it2)] = distance + currentDistance;
                    //Update the previous map
                    previousMap[(*it2)] = (it->first);
                }
            }
        }
    }

    // Return the step map
    return previousMap;
}

/**
 * Create the HighwayProject for the supplied configuration data
 */
HighwayProject::HighwayProject(HighwayProjectXml projectXml) {
    //Get the step distance
    m_dt = projectXml.GetDt();
    //Initialize the collections
    m_vehGens = list<Ptr<VehicleGenerator> >();
    m_highways = map<int, Ptr<Highway> >();
    m_trafficGens = list<Ptr<TrafficLightGenerator> >();
    //Set the project xml
    m_projectXml = projectXml;

    //Initialize the edge list map
    map<int, list<int > > nodes;

    //Get all of the highwayxml
    list<HighwayXml> xmlHighways = projectXml.GetHighways();
    //For each of the highway configuration
    for (list<HighwayXml>::iterator it = xmlHighways.begin(); it != xmlHighways.end(); it++) {
        HighwayXml highwayXml = *it;
        //Create a new Highway
        Ptr<Highway> newHighway = CreateObject<Highway > ();
        //Configure the Highway
        newHighway->SetHighwayId(highwayXml.GetHighwayId());
        newHighway->SetHighwayLength(highwayXml.GetLength());
        newHighway->SetDirection(highwayXml.GetDirection());
        newHighway->SetXPos(highwayXml.GetStartX());
        newHighway->SetYPos(highwayXml.GetStartY());
        newHighway->SetLaneWidth(highwayXml.GetLaneWidth());
        newHighway->SetNumberOfLanes(highwayXml.GetNumberOfLanes());
        newHighway->SetLeftTurnSpeed(highwayXml.GetLeftTurnSpeed());
        newHighway->SetRightTurnSpeed(highwayXml.GetRightTurnSpeed());
        newHighway->InitHighway();
        //Put the highway int the map
        m_highways[highwayXml.GetHighwayId()] = newHighway;
    }

    //Have to iterate again to create connections
    for (list<HighwayXml>::iterator it = xmlHighways.begin(); it != xmlHighways.end(); it++) {
        HighwayXml highwayXml = *it;
        Ptr<Highway> highway = m_highways[highwayXml.GetHighwayId()];
        list<int> connected;

        list<HighwayConnectionXml> backward = highwayXml.GetBackHighways();
        for (list<HighwayConnectionXml>::iterator it = backward.begin(); it != backward.end(); it++) {
            HighwayConnectionXml conn = *it;
            Ptr<Highway> otherHighway = m_highways[conn.GetHighwayId()];
            highway->AddBackHighway(otherHighway, conn.GetLaneOffset(), conn.GetOffset());
        }

        list<HighwayConnectionXml> forward = highwayXml.GetFrontHighways();
        for (list<HighwayConnectionXml>::iterator it = forward.begin(); it != forward.end(); it++) {
            HighwayConnectionXml conn = *it;
            Ptr<Highway> otherHighway = m_highways[conn.GetHighwayId()];
            highway->AddFrontHighway(otherHighway, conn.GetLaneOffset(), conn.GetOffset());
            connected.push_back(conn.GetHighwayId());
        }

        list<HighwayConnectionXml> rightward = highwayXml.GetRightHighways();
        for (list<HighwayConnectionXml>::iterator it = rightward.begin(); it != rightward.end(); it++) {
            HighwayConnectionXml conn = *it;
            Ptr<Highway> otherHighway = m_highways[conn.GetHighwayId()];
            highway->AddRightHighway(otherHighway, conn.GetLaneOffset(), conn.GetOffset());
            connected.push_back(conn.GetHighwayId());
        }

        list<HighwayConnectionXml> leftward = highwayXml.GetLeftHighways();
        for (list<HighwayConnectionXml>::iterator it = leftward.begin(); it != leftward.end(); it++) {
            HighwayConnectionXml conn = *it;
            Ptr<Highway> otherHighway = m_highways[conn.GetHighwayId()];
            highway->AddLeftHighway(otherHighway, conn.GetLaneOffset(), conn.GetOffset());
            connected.push_back(conn.GetHighwayId());
        }

        nodes[highwayXml.GetHighwayId()] = connected;
    }

    for (list<HighwayXml>::iterator it = xmlHighways.begin(); it != xmlHighways.end(); it++) {
        HighwayXml highwayXml = *it;
        int sourceId = highwayXml.GetHighwayId();
        Ptr<Highway> currentHighway = m_highways[sourceId];
        map<int, int> prevMap = Djikstra(sourceId, nodes);
        map<int, Highway::TurningType> routingMap;

        for (list<HighwayXml>::iterator it2 = xmlHighways.begin(); it2 != xmlHighways.end(); it2++) {
            int destId = (*it2).GetHighwayId();
            map<int, int>::iterator entry = prevMap.find(destId);
            while (entry != prevMap.end()) {
                if (sourceId == entry->second) {
                    break;
                }
                entry = prevMap.find(entry->second);
            }

            int nextId = entry->first;

            routingMap[destId] = currentHighway->GetTurningTypeForHighwayId(nextId);
        }


        currentHighway->SetRoutingMap(routingMap);

    }

    map<int, WifiConfigurationXml> wifiConfigMap;
    list<WifiConfigurationXml> wifiList = projectXml.GetWifiConfigs();

    for (list<WifiConfigurationXml>::iterator it = wifiList.begin(); it != wifiList.end(); it++) {
        WifiConfigurationXml wifiConfig = *it;
        wifiConfigMap[wifiConfig.GetWifiConfigId()] = wifiConfig;
    }

    list<VehicleGeneratorXml> vehicleGeneratorList = projectXml.GetVehicleGenerators();

    WifiConfigurationXml defaultConfig;
    defaultConfig.LoadDefaults();

    for (list<VehicleGeneratorXml>::iterator it = vehicleGeneratorList.begin(); it != vehicleGeneratorList.end(); it++) {
        VehicleGeneratorXml genXml = *it;
        Ptr<Highway> highway = m_highways[genXml.GetHighwayId()];

        WifiConfigurationXml wifiConfig;
        if (wifiConfigMap.find(genXml.GetWifiConfigId()) != wifiConfigMap.end()) {
            wifiConfig = wifiConfigMap[genXml.GetWifiConfigId()];
        } else {
            wifiConfig = defaultConfig;
        }

        Ptr<VehicleGenerator> generator = CreateObject<VehicleGenerator > (highway);
        generator->SetFlow(genXml.GetFlow());
        generator->SetLowVelocity(genXml.GetLowVelocity());
        generator->SetHighVelocity(genXml.GetHighVelocity());
        generator->SetMinGap(genXml.GetMinGap());
        generator->SetPenetrationRate(genXml.GetPenetrationRate());
        generator->SetDestinationMap(genXml.GetDestinationMap());
        m_vehGens.push_back(generator);
        
        generator->SetWifiHelper(wifiConfig.GetWifiHelper());
        generator->SetNqosWifiMacHelper(wifiConfig.GetWifiMacHelper());
        generator->SetYansWifiPhyHelper(wifiConfig.GetWifiPhyHelper());

    }

    list<TrafficLightGeneratorXml> tlGenList = projectXml.GetTrafficLightGenerators();

    for (list<TrafficLightGeneratorXml>::iterator it = tlGenList.begin(); it != tlGenList.end(); it++) {
        TrafficLightGeneratorXml tlGenXml = *it;

        Ptr<TrafficLightGenerator> genPtr = CreateObject<TrafficLightGenerator>();
        list<TrafficPointXml> tpXmlList = tlGenXml.GetTrafficPoints();

        for (list<TrafficPointXml>::iterator it2 = tpXmlList.begin(); it2 != tpXmlList.end(); it2++) {
            TrafficPointXml tpXml = *it2;
            TrafficLightGenerator::Side side;
            if (tpXml.GetSide() == "LEFT") {
                side = TrafficLightGenerator::LEFT;
            } else if (tpXml.GetSide() == "RIGHT") {
                side = TrafficLightGenerator::RIGHT;
            } else if (tpXml.GetSide() == "TOP") {
                side = TrafficLightGenerator::TOP;
            } else if (tpXml.GetSide() == "BOTTOM") {
                side = TrafficLightGenerator::BOTTOM;
            }

            Ptr<Highway> referencedHighway = m_highways[tpXml.GetHighwayId()];

            genPtr->SetHighway(side, referencedHighway, tpXml.GetDistance(), tpXml.GetLeftTurnLanes());
        }

        m_trafficGens.push_back(genPtr);

    }

}

void HighwayProject::SetVehicleControlCallback(Callback<bool, Ptr<Highway>, Ptr<Vehicle>, double> controllCallback) {
    for(map<int, ns3::Ptr<ns3::Highway> >::iterator it = m_highways.begin(); it != m_highways.end(); it++) {
        it->second->SetControlVehicleCallback(controllCallback);
    }
}

void HighwayProject::VehicleReceive(ns3::Ptr<ns3::Vehicle> veh, ns3::Ptr<const ns3::Packet> p, ns3::Address add) {
    m_netTrace << Simulator::Now().GetNanoSeconds() << "," << veh->GetVehicleId() << ",Vehicle Receive,0" << endl;
}

void HighwayProject::EnableVehicleReceive() {
    for(list<Ptr<VehicleGenerator> >::iterator it = m_vehGens.begin(); it != m_vehGens.end(); it++) {
        (*it)->SetReceiveDataCallback(MakeCallback(&HighwayProject::VehicleReceive, this));
    }
}


void HighwayProject::DeviceTrace(ns3::Ptr<ns3::Vehicle> veh, string context, ns3::Ptr<const ns3::Packet> p) {
    m_netTrace << Simulator::Now().GetNanoSeconds() << "," << veh->GetVehicleId() << ",\"Device Trace:" << context << "\",1" << endl;
}

void HighwayProject::EnableDeviceTrace() {
    for(list<Ptr<VehicleGenerator> >::iterator it = m_vehGens.begin(); it != m_vehGens.end(); it++) {
        (*it)->SetDevTxTraceCallback(MakeCallback(&HighwayProject::DeviceTrace, this));
        (*it)->SetDevRxTraceCallback(MakeCallback(&HighwayProject::DeviceTrace, this));
    }
}

void HighwayProject::PhyRxOkTrace(ns3::Ptr<ns3::Vehicle> veh, string context, ns3::Ptr<const ns3::Packet> p, double snr, ns3::WifiMode mode, enum ns3::WifiPreamble preamble) {
    m_netTrace << Simulator::Now().GetNanoSeconds() << "," << veh->GetVehicleId() << ",\"PhyRxOk Trace:" << context << ":" << snr << "\",2" << endl;
}

void HighwayProject::EnablePhyRxOkTrace() {
    for(list<Ptr<VehicleGenerator> >::iterator it = m_vehGens.begin(); it != m_vehGens.end(); it++) {
        (*it)->SetPhyRxOkTraceCallback(MakeCallback(&HighwayProject::PhyRxOkTrace, this));
    }
}

void HighwayProject::PhyRxErrorTrace(Ptr<Vehicle> veh, std::string context, ns3::Ptr<const ns3::Packet> packet, double snr) {
    m_netTrace << Simulator::Now().GetNanoSeconds() << "," << veh->GetVehicleId() << ",\"PhyRxError Trace:" << context << ":" << snr << "\",3" << endl;
}

void HighwayProject::EnablePhyRxErrorTrace() {
    for(list<Ptr<VehicleGenerator> >::iterator it = m_vehGens.begin(); it != m_vehGens.end(); it++) {
        (*it)->SetPhyRxErrorTraceCallback(MakeCallback(&HighwayProject::PhyRxErrorTrace, this));
    }
}

void HighwayProject::PhyTxTrace(Ptr<Vehicle> veh, std::string context, ns3::Ptr<const ns3::Packet> packet, ns3::WifiMode mode, ns3::WifiPreamble preamble, uint8_t txPower) {
    m_netTrace << Simulator::Now().GetNanoSeconds() << "," << veh->GetVehicleId() << ",\"PhyTx Trace:" << context << ":" << txPower << "\",4" << endl;
}

void HighwayProject::EnablePhyTxTrace() {
    for(list<Ptr<VehicleGenerator> >::iterator it = m_vehGens.begin(); it != m_vehGens.end(); it++) {
        (*it)->SetPhyTxTraceCallback(MakeCallback(&HighwayProject::PhyTxTrace, this));
    }
}

void HighwayProject::PhyStateTrace(Ptr<Vehicle> veh, std::string context, ns3::Time start, ns3::Time duration, enum ns3::WifiPhy::State state) {
    m_netTrace << Simulator::Now().GetNanoSeconds() << "," << veh->GetVehicleId() << ",\"EnablePhyTx Trace:" << context << ":" << start.GetNanoSeconds() << ":" << duration.GetNanoSeconds() << "\",5" << endl;
}

void HighwayProject::EnablePhyStateTrace() {
    for(list<Ptr<VehicleGenerator> >::iterator it = m_vehGens.begin(); it != m_vehGens.end(); it++) {
        (*it)->SetPhyStateTraceCallback(MakeCallback(&HighwayProject::PhyStateTrace, this));
    }
}


void HighwayProject::SetVehTraceFile(string fileName) {
    m_vehTraceFileName = fileName;
}

void HighwayProject::SetNetTraceFile(string fileName) {
    m_netTraceFileName = fileName;
}

void HighwayProject::Start() {
    m_vehTrace.open(m_vehTraceFileName.c_str());
    m_netTrace.open(m_netTraceFileName.c_str());
    for(list<Ptr<VehicleGenerator> >::iterator it = m_vehGens.begin(); it != m_vehGens.end(); it++) {
        (*it)->init();
    }
    for(list<Ptr<TrafficLightGenerator> >::iterator it = m_trafficGens.begin(); it != m_trafficGens.end(); it++) {
        (*it)->Start();
    }
    Simulator::Schedule(Seconds(0.0), &Step, this);
    Simulator::Stop(Seconds(m_projectXml.GetTotalTimeInSeconds()));
}

std::map<int, ns3::Ptr<ns3::Highway> > HighwayProject::getHighways() {
    return m_highways;
}

void HighwayProject::Step(HighwayProject* project) {

    for(map<int, Ptr<Highway> >::iterator it = project->m_highways.begin(); it != project->m_highways.end(); it++) {
        Ptr<Highway> highway = it->second;
        Highway::Step(highway);
    }
    for(map<int, Ptr<Highway> >::iterator it = project->m_highways.begin(); it != project->m_highways.end(); it++) {
        Ptr<Highway> highway = it->second;
        highway->HandleTransfers();
    }

    for(map<int, Ptr<Highway> >::iterator it = project->m_highways.begin(); it != project->m_highways.end(); it++) {
        Ptr<Highway> highway = it->second;
        for(int i = 1; i <= highway->GetNumberOfLanes(); i++) {
            list<Ptr<Vehicle> >* vehList = highway->GetVehiclesInLane(i);
            if(vehList != NULL) {
                for(list<Ptr<Vehicle> >::iterator it2 = vehList->begin(); it2 != vehList->end(); it2++) {
                    Ptr<Vehicle> veh = (*it2);
                    project->m_vehTrace << Simulator::Now().GetNanoSeconds() << "," << veh->GetVehicleId() << "," << veh->GetVehicleType() << "," << veh->GetPosition().x << "," << veh->GetPosition().y << "," << veh->GetDirection() << ","
                            << veh->GetVelocity() << "," << veh->GetAcceleration() << endl;
                }
            }
        }
    }
    Simulator::Schedule(Seconds(project->m_dt), &HighwayProject::Step, project);
}