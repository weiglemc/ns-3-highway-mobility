/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005-2011 Old Dominion University [ARBABI]
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
 * Author: Bradley Dupont <bradley.dupont@cs.odu.edu>
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

/**
 * This class Controls Highways, VehicleGenerators, and TrafficLightGenerators
 * It is the main way to run the vehicle simulations
 */
class HighwayProject {
private:
    // The time step for highways
    double m_dt;
    // A map of highwayId to highway
    std::map<int, ns3::Ptr<ns3::Highway> > m_highways;
    // A list of vehicle generators
    std::list<ns3::Ptr<ns3::VehicleGenerator> > m_vehGens;
    // A list of traffic generators
    std::list<ns3::Ptr<ns3::TrafficLightGenerator> > m_trafficGens;
    // The XML document for configuring the HighwayProject
    HighwayProjectXml m_projectXml;

    // The name of the file used for the vehilce tracing
    string m_vehTraceFileName;
    // The name of the file used for the net tracing
    string m_netTraceFileName;

    // The output stream for vehicle tracing
    std::ofstream m_vehTrace;
    // The output stream for net tracing
    std::ofstream m_netTrace;

    // The callbacks for the various net traces
    ns3::VehicleReceiveCallback vehReceiveCallback;
    ns3::DeviceTraceCallback deviceTraceCallback;
    ns3::PhyRxOkTraceCallback phyRxOkTraceCallback;
    ns3::PhyRxErrorTraceCallback phyRxErrorCallback;
    ns3::PhyTxTraceCallback phyTxTraceCallback;
    ns3::PhyStateTraceCallback phyStateTraceCallback;

    // A private function for calculating the routing maps for Highways
    map<int, int> Djikstra(int source, map<int, list<int> > connectionList);

    

public:
    // The only constructor
    HighwayProject(HighwayProjectXml projectXml);

    // Sets the name of the trace file for the vehicle trace
    void SetVehTraceFile(string fileName);
    // Sets the name of the trace file for the net trace
    void SetNetTraceFile(string fileName);

    // Starts the highway simulation
    void Start();

    // Returns the Highway map
    std::map<int, ns3::Ptr<ns3::Highway> > getHighways();

    /**
     * Below are the functions used for the net tracebacks
     * Each callback has an enable function associated with it that
     * allows for individual callbacks to be enabled
     */

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

    void SetVehicleControlCallback(Callback<bool, Ptr<Highway>, Ptr<Vehicle>, double> controllCallback);

    // Calls the next step on the simulation
    static void Step(HighwayProject* project);

};

#endif	/* _HIGHWAYPROJECT_H */

