/* 
 * File:   HighwayProject.h
 * Author: bdupont
 *
 * Created on September 20, 2011, 11:17 AM
 */

#ifndef _HIGHWAYPROJECT_H
#define	_HIGHWAYPROJECT_H

#include "ns3/ptr.h"
#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/common-module.h"
#include "ns3/node-module.h"
#include "ns3/helper-module.h"
#include "ns3/mobility-module.h"
#include "ns3/contrib-module.h"
#include "ns3/wifi-module.h"
#include "HighwayProjectXml.h"
#include "Vehicle.h"
#include "Highway.h"
#include "VehicleGenerator.h"
#include "TrafficLightGenerator.h"
#include <string>
#include <map>
#include <iostream>
#include <fstream>

using namespace std;

class HighwayProject {
private:
    double m_dt;
    std::map<int, ns3::Ptr<ns3::Highway> > m_highways;
    std::list<ns3::Ptr<ns3::VehicleGenerator> > m_vehGens;
    std::list<ns3::Ptr<ns3::TrafficLightGenerator> > m_trafficGens;
    HighwayProjectXml m_projectXml;

    string m_vehTraceFileName;
    string m_netTraceFileName;

    std::ofstream m_vehTrace;
    std::ofstream m_netTrace;

    ns3::VehicleReceiveCallback vehReceiveCallback;
    ns3::DeviceTraceCallback deviceTraceCallback;
    ns3::PhyRxOkTraceCallback phyRxOkTraceCallback;
    ns3::PhyRxErrorTraceCallback phyRxErrorCallback;
    ns3::PhyTxTraceCallback phyTxTraceCallback;
    ns3::PhyStateTraceCallback phyStateTraceCallback;

    map<int, int> Djikstra(int source, map<int, list<int> > connectionList);

    

public:
    HighwayProject(HighwayProjectXml projectXml);

    void SetVehTraceFile(string fileName);
    void SetNetTraceFile(string fileName);

    void Start();

    std::map<int, ns3::Ptr<ns3::Highway> > getHighways();

    void VehicleReceive(ns3::Ptr<ns3::Vehicle> veh, ns3::Ptr<const ns3::Packet> p, ns3::Address add);
    void EnableVehicleReceive();

    void DeviceTrace(ns3::Ptr<ns3::Vehicle> veh, string context, ns3::Ptr<const ns3::Packet> p);
    void EnableDeviceTrace();

    void PhyRxOkTrace(ns3::Ptr<ns3::Vehicle> veh, string context, ns3::Ptr<const ns3::Packet> p, double snr, ns3::WifiMode mode, enum ns3::WifiPreamble preamble);
    void EnablePhyRxOkTrace();

    void PhyRxErrorTrace(ns3::Ptr<ns3::Vehicle> veh, std::string context, ns3::Ptr<const ns3::Packet> packet, double snr);
    void EnablePhyRxErrorTrace();

    void PhyTxTrace(ns3::Ptr<ns3::Vehicle> veh, std::string context, ns3::Ptr<const ns3::Packet> packet, ns3::WifiMode mode, ns3::WifiPreamble preamble, uint8_t txPower);
    void EnablePhyTxTrace();

    void PhyStateTrace(ns3::Ptr<ns3::Vehicle> veh, std::string context, ns3::Time start, ns3::Time duration, enum ns3::WifiPhy::State state);
    void EnablePhyStateTrace();

    static void Step(HighwayProject* project);

};

#endif	/* _HIGHWAYPROJECT_H */

