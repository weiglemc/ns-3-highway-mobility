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
 * Author: Hadi Arbabi <marbabi@cs.odu.edu>
 * `       Bradley Dupont <bradley.dupont@cs.odu.edu>
 */


#ifndef _VEHICLEGENERATOR_H
#define	_VEHICLEGENERATOR_H

#include "ns3/callback.h"
#include "ns3/ptr.h"
#include "ns3/object.h"
#include "ns3/random-variable.h"
#include "ns3/vector.h"
#include "Model.h"
#include "LaneChange.h"
#include "Highway.h"
#include "Vehicle.h"
#include <map>
#include <list>
#include <utility>

namespace ns3 {

    class VehicleGenerator : public ns3::Object {
    private:
        Ptr<Highway> m_highway; // the highway this generator is feeding vehicles into
        double m_flow; // traffic flow in veh/s at entrance (not randomized).  Can override standard by passing in a custom value for m_RVFlow
        double m_lowVelocity; // lower end of traffic velocity.  Can override standard by passing in a custom value for m_RVSpeed
        double m_highVelocity;  // higher end of traffic velocity.  Can override standard by passing in a custom value for m_RVSpeed
        double m_remainder; // remainder of vehicle before sending a new one ine.
        int m_currentLane; // interal lane number to track what lane we are inserting a vehicle on
        double m_penetrationRate; // The rate (0-100%) of vehicles with WIFI
        double m_dt; // The time between steps.  Supplied by the highway
        double m_minGap; // The minimum gap between vehicles
        int m_truckProability; // The probability (0-100%) of the next vehicle being a truck
        std::map<double, int> m_destinationMap;  // A map of destinations.  See .cc file for details
        RandomVariable m_RVFlow; // highway flow random variable in positive direction
        RandomVariable m_RVSpeed; // vehicles speed distribution
        WifiHelper m_wifiHelper; // a wifi helper apply to setup vehicles Wifi
        NqosWifiMacHelper m_wifiMacHelper; // a wifi mac helper apply to setup vehicles Wifi
        YansWifiPhyHelper m_wifiPhyHelper; // a wifi phy helper apply to setup vehicles Wifi

        /// For Catching an event when a packet is received by any vehicles in the Highway.
        VehicleReceiveCallback m_receiveData;
        /// For Catching DevTxTrace.
        DeviceTraceCallback m_devTxTrace;
        /// For Catching DevRxTrace.m_phyRxOkTrace
        DeviceTraceCallback m_devRxTrace;
        /// For Catching PhyRxOkTrace.
        PhyRxOkTraceCallback m_phyRxOkTrace;
        /// For Catching PhyRxErrorTrace.
        PhyRxErrorTraceCallback m_phyRxErrorTrace;
        /// For Catching PhyTxTrace.
        PhyTxTraceCallback m_phyTxTrace;
        /// For Catching PhyStateTrace.
        PhyStateTraceCallback m_phyStateTrace;

        // Called by static method to process step
        void insertVehicles();

    public:
        /// Override TypeId.
        static TypeId GetTypeId(void);

        // A vehicle generator must have a Highway to put vehicles into
        VehicleGenerator(Ptr<Highway> associatedHighway);
        // Generator's destructor
        ~VehicleGenerator();

        // Initializes VehicleGenerator
        void init();

        // Sets the flow (overrides current m_rvFlow)
        void SetFlow(double flow);
        // Gets the flow
        double GetFlow();

        // Sets the lower end of the uniform velocity distribution
        void SetLowVelocity(double lowVelocity);
        // Gets the lower end of the uniform velocity distribution
        double GetLowVelocity();

        // Sets the higher end of the uniform velocity distribution
        void SetHighVelocity(double highVelocity);
        // Gets the higher end of the uniform velocity distribution
        double GetHighVelocity();

        // Sets the penetration rate
        void SetPenetrationRate(double penetrationRate);
        // Gets the penetration rate
        double GetPenetrationRate();

        // Sets the minimum gap
        void SetMinGap(double minGap);
        // Gets the minimum gap
        double GetMinGap();

        // Sets the truck probability rate
        void SetTruckProbability(int truckProbability);
        // Gets the truck probability rate
        int GetTruckProbability();

        // Sets the random variable to use for vehicle flow.  Overrides m_flow
        void SetRVFlow(RandomVariable rvFlow);
        // Gets the current random variable to use for vehicle flow.
        RandomVariable GetRVFlow();

        // Sets the random variable to use for vehicle speed. Overrides low and high velocity
        // Note that lowVelocity is still used as the starting velocity
        void SetRVSpeed(RandomVariable rvSpeed);
        // Gets the current random variable to use for vehicle speed.
        RandomVariable GetRVSpeed();

        // Sets the WifiHelper to use for creating vehicles
        void SetWifiHelper(WifiHelper wifiHelper);
        // Gets the WifiHelper to use for creating vehicles
        WifiHelper GetWifiHelper();

        // Sets the Mac helper to use for creating vehicles
        void SetNqosWifiMacHelper(NqosWifiMacHelper wifiMacHelper);
        // Gets the Mac helper to use for creating vehicles
        NqosWifiMacHelper GetNqosWifiMacHelper();

        // Sets the Phy helper to use for creating vehicles
        void SetYansWifiPhyHelper(YansWifiPhyHelper wifiPhyHelper);
        // Gets the Phy helper to use for creating vehicles
        YansWifiPhyHelper GetYansWifiPhyHelper();

        // This takes a list of weighted destinations and converts them to the destination
        // map.  If the sum of all of the doubles in the list is 1, then the weights function
        // as percentages.  Otherwise, the weights are averaged out.
        void SetDestinationMap(std::list<std::pair<double, int> > destinationMap);
        // Gets the cumulative percentage map for destinations.  Each entry in the map
        // is a cumulative percentage required to get to that point.  In other words
        // if a destination weighting is passed into SetDestinationMap of:
        // (0.3, 1), (0.3, 2), (0.4, 3)
        // This means that there should be a 30% chance of a vehicle going to
        // 1, a 30% chance of a vehicle going to 2, and a 40% chance of a vehicle
        // going to three.  This is translated to the following entries:
        // (0.3, 1), (0.6, 2), (1.0, 3)
        // A uniform variable from [0,1) is queried to get the value. A value
        // from [0,0.3) would go to 1, [0.3,0.6) would be 2 and [0.6,1) would
        // go to 3.
        std::map<double, int> GetDestinationMap();

        /// Returns the Highway's Receive Data callback.
        VehicleReceiveCallback GetReceiveDataCallback();
        /// Sets the Highway's Receive Data callback.
        void SetReceiveDataCallback(VehicleReceiveCallback receiveData);
        /// Returns the Highway's DevTxTrace callback.
        DeviceTraceCallback GetDevTxTraceCallback();
        /// Sets the Highway's DevTxTrace callback.
        void SetDevTxTraceCallback(DeviceTraceCallback devTxTrace);
        /// Returns the Highway's DevRxTrace callback.
        DeviceTraceCallback GetDevRxTraceCallback();
        /// Sets the Highway's DevRxTrace callback.
        void SetDevRxTraceCallback(DeviceTraceCallback devRxTrace);
        /// Returns the Highway's PhyRxOkTrace callback.
        PhyRxOkTraceCallback GetPhyRxOkTraceCallback();
        /// Sets the Highway's PhyRxOkTrace callback.
        void SetPhyRxOkTraceCallback(PhyRxOkTraceCallback phyRxOkTrace);
        /// Returns the Highway's PhyRxErrorTrace callback.
        PhyRxErrorTraceCallback GetPhyRxErrorTraceCallback();
        /// Sets the Highway's PhyRxErrorTrace callback.
        void SetPhyRxErrorTraceCallback(PhyRxErrorTraceCallback phyRxErrorTrace);
        /// Returns the Highway's PhyTxTrace callbacl.
        PhyTxTraceCallback GetPhyTxTraceCallback();
        /// Sets the Highway's PhyTxTrace callback.
        void SetPhyTxTraceCallback(PhyTxTraceCallback phyTxTrace);
        /// Returns the Highway's PhyStateTrace callback.
        PhyStateTraceCallback GetPhyStateTraceCallback();
        /// Sets the Highway's PhyStateTrace callback.
        void SetPhyStateTraceCallback(PhyStateTraceCallback phyStateTrace);
        /// Returns the Highway Control Vehicle callback.
        Callback<bool, Ptr<Highway>, Ptr<Vehicle>, double> GetControlVehicleCallback();

        /**
         * Runs one mobility Step for the given vehicle generator.
         * This function is called each interval dt
         */
        static void Step(Ptr<VehicleGenerator> vehicleGenerator);

    };

}

#endif	/* _VEHICLEGENERATOR_H */

