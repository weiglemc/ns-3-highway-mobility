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
 *         Bradley Dupont <bradley.dupont@cs.odu.edu>
 */

#include <iostream>
#include <map>
#include <list>
#include "VehicleGenerator.h"
#include "Vehicle.h"
#include "Model.h"
#include "LaneChange.h"
#include "Geometry.h"
#include "ns3/simulator.h"
#include "IdGenerator.h"

namespace ns3 {

    // Override the ns3 type
    TypeId VehicleGenerator::GetTypeId(void) {
        static TypeId tid = TypeId("ns3::VehicleGenerator")
                .SetParent<Object > ()
                ;
        return tid;
    }

    // This is a utility function to create the standard
    // IDM model for a sedan
    Ptr<Model> CreateSedanModel() {
        Ptr<Model> model = CreateObject<Model > ();
        model->SetDesiredVelocity(11.176);
        model->SetDeltaV(4.0);
        model->SetAcceleration(0.5);
        model->SetDeceleration(3.0);
        model->SetMinimumGap(2.0);
        model->SetTimeHeadway(0.1);
        model->SetSqrtAccelerationDeceleration(sqrt(model->GetAcceleration() * model->GetDeceleration()));
        return model;
    }

    // This is a utility function to create the standard
    // lane changing model for a sedan
    Ptr<LaneChange> CreateSedanLaneChangeModel() {
        Ptr<LaneChange> laneChange = CreateObject<LaneChange > ();
        laneChange->SetPolitenessFactor(0.2);
        laneChange->SetDbThreshold(0.3);
        laneChange->SetGapMin(2.0);
        laneChange->SetMaxSafeBreakingDeceleration(12.0);
        laneChange->SetBiasRight(0.2);
        return laneChange;
    }

    // This is a utility function to create the standard
    // IDM model for a truck
    Ptr<Model> CreateTruckModel() {
        Ptr<Model> model = CreateObject<Model > ();
        model->SetDesiredVelocity(13.0);
        model->SetDeltaV(4.0);
        model->SetAcceleration(0.2);
        model->SetDeceleration(4.0);
        model->SetMinimumGap(2.0);
        model->SetTimeHeadway(0.5);
        model->SetSqrtAccelerationDeceleration(sqrt(model->GetAcceleration() * model->GetDeceleration()));
        return model;
    }

    // This is a utility function to create the standard
    // lane changing model for a truck
    Ptr<LaneChange> CreateTruckLaneChangeModel() {
        Ptr<LaneChange> laneChange = CreateObject<LaneChange > ();
        laneChange->SetPolitenessFactor(0.2);
        laneChange->SetDbThreshold(0.2);
        laneChange->SetGapMin(2.0);
        laneChange->SetMaxSafeBreakingDeceleration(12.0);
        laneChange->SetBiasRight(0.3);
        return laneChange;
    }

    // The only construction
    VehicleGenerator::VehicleGenerator(Ptr<Highway> associatedHighway) {
        // Get the m_dt from the highway we are associated to
        m_highway = associatedHighway;
        m_dt = m_highway->GetDeltaT();


        // Default to 1 veh/s
        m_flow = 1;
        m_lowVelocity = 11.176; // 25 MPH
        m_highVelocity = 11.176; // constant
        m_currentLane = 1; // start in the far left lane
        m_minGap = 33; //3 second rule for following at 11 m/s
        m_remainder = 0; // The value accumulated to next point
        m_penetrationRate = 100; // Default penetration rate is 100%

        // Create the default speed and flow random variables
        // These can be overriden later
        m_RVFlow = UniformVariable(m_flow*m_dt, m_flow * m_dt);
        m_RVSpeed = UniformVariable(11.176, 11.176);

        // Initialize the destination map to being empty
        m_destinationMap = std::map<double, int>();

        // Setup a standard set of wifi attributes
        m_wifiHelper = WifiHelper::Default();
        m_wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211a);
        m_wifiMacHelper = NqosWifiMacHelper::Default();
        m_wifiPhyHelper = YansWifiPhyHelper::Default();
        YansWifiChannelHelper m_wifiChannelHelper = YansWifiChannelHelper::Default();
        m_wifiMacHelper.SetType("ns3::AdhocWifiMac");
        m_wifiHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue("OfdmRate6MbpsBW10MHz"));
        //m_wifiChannelHelper.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
        Ptr<YansWifiChannel> m_wifiChannel = m_wifiChannelHelper.Create();
        m_wifiPhyHelper.SetChannel(m_wifiChannel);
        m_wifiPhyHelper.Set("TxPowerStart", DoubleValue(21.5)); // 250-300 meter transmission range
        m_wifiPhyHelper.Set("TxPowerEnd", DoubleValue(21.5)); // 250-300 meter transmission range
        m_wifiPhyHelper.Set("TxPowerLevels", UintegerValue(1));
        m_wifiPhyHelper.Set("TxGain", DoubleValue(2));
        m_wifiPhyHelper.Set("RxGain", DoubleValue(2));
        m_wifiPhyHelper.Set("EnergyDetectionThreshold", DoubleValue(-101.0));
    }

    // We don't do any dynamic allocations
    VehicleGenerator::~VehicleGenerator() {
    }

    // The init function starts the generator calling once very m_dt seconds.
    void VehicleGenerator::init() {
        Simulator::Schedule(Seconds(m_dt), &VehicleGenerator::Step, Ptr<VehicleGenerator > (this));
    }

    // This function does most of the functionality.
    void VehicleGenerator::insertVehicles() {
        // This variable is used for vehicle type determination
        UniformVariable uRnd(0, 100);
        // This variable is used for wifi equipping descision
        UniformVariable uRnd2(0, 100);
        // This variable is used for destination determination
        UniformVariable uRnd3;
        // Added to m_remainder to determion if it is time to put a vehicle in
        double rate1 = m_RVFlow.GetValue();
        // The gap to the previous vehicle in the lane
        double gap;
        // The velocity the vehicle starts at

        // Use uRnd to determine if we are a truck
        bool isSedan = uRnd.GetValue() <= m_truckProability;
        // Get the default lengths
        int length = isSedan ? 4 : 8;
        double vel = m_lowVelocity;

        // Get the first vehicle in the current lane
        Ptr<Vehicle> last = m_highway->GetFirstVehicle(m_currentLane);
        // If the last vehicle is not there
        if (last == NULL) {
            // We have no need for a gap
            gap = m_minGap + 1;
        } else {
            // Get the distance
            gap = m_highway->GetDistanceToFirstVehicle(m_currentLane) - length;
        }

        // Add to the remaining time
        m_remainder += rate1;
        // If the remainder is greater than 1 and we don't have a gap
        if (m_remainder >= 1 && gap > m_minGap) {
            // Subtract a total sum from the remainder (leave leftover)
            m_remainder -= 1;
            // Create the vehicle object
            Ptr<Vehicle> temp = CreateObject<Vehicle > ();
            // Use uRnd2 to determine if it is equipped with WIF
            if (uRnd2.GetValue() <= m_penetrationRate) {
                // Setup the wifi
                temp->IsEquipped = true;
                temp->SetupWifi(m_wifiHelper, m_wifiPhyHelper, m_wifiMacHelper);
            } else {
                // Do nothing with wifi
                temp->IsEquipped = false;
            }
            // Get the vehicle ID from the IDGenerator (needed to avoid collisions
            // with other vehicle generators and TrafficLightGenerators)
            temp->SetVehicleId(IdGenerator::nextVehicleId());
            // Set the lane for the vehicle fo rthe lane we want
            temp->SetLane(m_currentLane);
            // Set the velocity to the starting velocity
            temp->SetVelocity(vel);
            // Set the initial acceleration to 0
            temp->SetAcceleration(0.0);
            if (temp->IsEquipped == true) {
                // If we are equipped, set the callbacks to the currently available
                // ones
                temp->SetReceiveCallback(m_receiveData);
                temp->SetDevTxTraceCallback(m_devTxTrace);
                temp->SetDevRxTraceCallback(m_devRxTrace);
                temp->SetPhyRxOkTraceCallback(m_phyRxOkTrace);
                temp->SetPhyRxErrorTraceCallback(m_phyRxErrorTrace);
                temp->SetPhyTxTraceCallback(m_phyTxTrace);
                temp->SetPhyStateTraceCallback(m_phyStateTrace);
            }

            // Create the appropriate model and assign it to the
            // vehicle
            if (isSedan) {
                Ptr<Model> tempModel = CreateSedanModel();
                tempModel->SetDesiredVelocity(m_RVSpeed.GetValue());
                temp->SetModel(tempModel);
                temp->SetLaneChange(CreateSedanLaneChangeModel());
                temp->SetLength(4);
                temp->SetWidth(2);
            } else {
                Ptr<Model> tempModel = CreateTruckModel();
                tempModel->SetDesiredVelocity(m_RVSpeed.GetValue());
                temp->SetModel(tempModel);
                temp->SetLaneChange(CreateTruckLaneChangeModel());
                temp->SetLength(8);
                temp->SetWidth(2);
            }

            // Determine what destination we are going to
            int destination = -1;
            // Get the destinatio probablity value range [0,1)
            double destVal = uRnd3.GetValue();
            for (std::map<double, int>::iterator it = m_destinationMap.begin(); it != m_destinationMap.end(); it++) {
                // Set the destination
                destination = it->second;
                // If we have gotten to where we need to be in
                // the map
                if (destVal < it->first) {
                    // Stop the search
                    break;
                }
            }
            // Set the destination.  A destination of -1 will not harm the
            // simulation.  Highway handles unkown destinations by defaulting
            // to STRAIGHT
            temp->AddDestination(destination);

            // Add the vehicle to the beginning of the highway
            m_highway->AddVehicleToBeginning(temp);
            // We need to put vehicles in other lanes
            m_currentLane++;
            // Reset the lane to 0 once we are done
            if (m_currentLane > m_highway->GetNumberOfLanes()) {
                m_currentLane = 1;
            }
        }

        // Schecule the next run
        Simulator::Schedule(Seconds(m_dt), &VehicleGenerator::Step, Ptr<VehicleGenerator > (this));
    }


    // Sets the flow (overrides current m_rvFlow)
    void VehicleGenerator::SetFlow(double flow) {
        m_flow = flow;
        m_RVFlow = UniformVariable(m_flow*m_dt, m_flow * m_dt);
    }

    // Gets the flow
    double VehicleGenerator::GetFlow() {
        return m_flow;
    }

    // Sets the lower end of the uniform velocity distribution
    void VehicleGenerator::SetLowVelocity(double lowVelocity) {
        m_lowVelocity = lowVelocity;
        m_RVSpeed = UniformVariable(m_lowVelocity, m_highVelocity);
    }

    // Gets the lower end of the uniform velocity distribution
    double VehicleGenerator::GetLowVelocity() {
        return m_lowVelocity;
    }

    // Sets the higher end of the uniform velocity distribution
    void VehicleGenerator::SetHighVelocity(double highVelocity) {
        m_highVelocity = highVelocity;
        m_RVSpeed = UniformVariable(m_lowVelocity, m_highVelocity);
    }

    // Gets the higher end of the uniform velocity distribution
    double VehicleGenerator::GetHighVelocity() {
        return m_highVelocity;
    }

    // Sets the penetration rate
    void VehicleGenerator::SetPenetrationRate(double penetrationRate) {
        m_penetrationRate = penetrationRate;
    }

    // Gets the penetration rate
    double VehicleGenerator::GetPenetrationRate() {
        return m_penetrationRate;
    }

    // Sets the minimum gap
    void VehicleGenerator::SetMinGap(double minGap) {
        m_minGap = minGap;
    }

    // Gets the minimum gap
    double VehicleGenerator::GetMinGap() {
        return m_minGap;
    }

    // Sets the truck probability rate
    void VehicleGenerator::SetTruckProbability(int truckProbability) {
        m_truckProability = truckProbability;
    }

    // Gets the truck probability rate
    int VehicleGenerator::GetTruckProbability() {
        return m_truckProability;
    }

    // Sets the random variable to use for vehicle flow.  Overrides m_flow
    void VehicleGenerator::SetRVFlow(RandomVariable rvFlow) {
        m_RVFlow = rvFlow;
    }

    // Gets the current random variable to use for vehicle flow.
    RandomVariable VehicleGenerator::GetRVFlow() {
        return m_RVFlow;
    }

    // Sets the random variable to use for vehicle speed. Overrides low and high velocity
    // Note that lowVelocity is still used as the starting velocity
    void VehicleGenerator::SetRVSpeed(RandomVariable rvSpeed) {
        m_RVSpeed = rvSpeed;
    }

    // Gets the current random variable to use for vehicle speed.
    RandomVariable VehicleGenerator::GetRVSpeed() {
        return m_RVSpeed;
    }

    // Sets the WifiHelper to use for creating vehicles
    void VehicleGenerator::SetWifiHelper(WifiHelper wifiHelper) {
        m_wifiHelper = wifiHelper;
    }

    // Gets the WifiHelper to use for creating vehicles
    WifiHelper VehicleGenerator::GetWifiHelper() {
        return m_wifiHelper;
    }

    // Sets the Mac helper to use for creating vehicles
    void VehicleGenerator::SetNqosWifiMacHelper(NqosWifiMacHelper wifiMacHelper) {
        m_wifiMacHelper = wifiMacHelper;
    }

    // Gets the Mac helper to use for creating vehicles
    NqosWifiMacHelper VehicleGenerator::GetNqosWifiMacHelper() {
        return m_wifiMacHelper;
    }

    // Sets the Phy helper to use for creating vehicles
    void VehicleGenerator::SetYansWifiPhyHelper(YansWifiPhyHelper wifiPhyHelper) {
        m_wifiPhyHelper = wifiPhyHelper;
    }

    // Gets the Phy helper to use for creating vehicles
    YansWifiPhyHelper VehicleGenerator::GetYansWifiPhyHelper() {
        return m_wifiPhyHelper;
    }

    // This takes a list of weighted destinations and converts them to the destination
    // map.  If the sum of all of the doubles in the list is 1, then the weights function
    // as percentages.  Otherwise, the weights are normalized.
    void VehicleGenerator::SetDestinationMap(std::list<std::pair<double, int> > destinationMap) {
        // Sum up all of the weights so that we can normalize the function to [0,1]
        double sum = 0.0;
        for (std::list<std::pair<double, int> >::iterator it = destinationMap.begin(); it != destinationMap.end(); it++) {
            sum += it->first;
        }

        // Clear the previous map
        m_destinationMap.clear();

        // Sum up the values so that the map is a cumulative value function
        // In C++ map is a sorted tree map
        double sum2 = 0.0;
        for (std::list<std::pair<double, int> >::iterator it = destinationMap.begin(); it != destinationMap.end(); it++) {
            sum2 += (((*it).first) / sum);
            m_destinationMap[sum2] = it->second;
        }
    }

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
    std::map<double, int> VehicleGenerator::GetDestinationMap() {
        return m_destinationMap;
    }

    VehicleReceiveCallback VehicleGenerator::GetReceiveDataCallback() {
        return m_receiveData;
    }

    void VehicleGenerator::SetReceiveDataCallback(VehicleReceiveCallback receiveData) {
        m_receiveData = receiveData;
    }

    DeviceTraceCallback VehicleGenerator::GetDevTxTraceCallback() {
        return m_devTxTrace;
    }

    void VehicleGenerator::SetDevTxTraceCallback(DeviceTraceCallback devTxTrace) {
        m_devTxTrace = devTxTrace;
    }

    DeviceTraceCallback VehicleGenerator::GetDevRxTraceCallback() {
        return m_devRxTrace;
    }

    void VehicleGenerator::SetDevRxTraceCallback(DeviceTraceCallback devRxTrace) {
        m_devRxTrace = devRxTrace;
    }

    PhyRxOkTraceCallback VehicleGenerator::GetPhyRxOkTraceCallback() {
        return m_phyRxOkTrace;
    }

    void VehicleGenerator::SetPhyRxOkTraceCallback(PhyRxOkTraceCallback phyRxOkTrace) {        m_phyRxOkTrace = phyRxOkTrace;
    }

    PhyRxErrorTraceCallback VehicleGenerator::GetPhyRxErrorTraceCallback() {
        return m_phyRxErrorTrace;
    }

    void VehicleGenerator::SetPhyRxErrorTraceCallback(PhyRxErrorTraceCallback phyRxErrorTrace) {
        m_phyRxErrorTrace = phyRxErrorTrace;
    }

    PhyTxTraceCallback VehicleGenerator::GetPhyTxTraceCallback() {
        return m_phyTxTrace;
    }

    void VehicleGenerator::SetPhyTxTraceCallback(PhyTxTraceCallback phyTxTrace) {
        m_phyTxTrace = phyTxTrace;
    }

    PhyStateTraceCallback VehicleGenerator::GetPhyStateTraceCallback() {
        return m_phyStateTrace;
    }

    void VehicleGenerator::SetPhyStateTraceCallback(PhyStateTraceCallback phyStateTrace) {
        m_phyStateTrace = phyStateTrace;
    }

    /**
     * Runs one mobility Step for the given vehicle generator.
     * This function is called each interval dt
     */
    void VehicleGenerator::Step(Ptr<VehicleGenerator> vehicleGenerator) {
        vehicleGenerator->insertVehicles();
    }
}