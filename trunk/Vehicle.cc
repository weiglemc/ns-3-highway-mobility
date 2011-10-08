/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005-2009 Old Dominion University [ARBABI]
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
 *         Bradley Dupont <bdupont@cs.odu.edu>
 */

#include "Vehicle.h"
#include "Geometry.h"
#include <cmath>
#include <iostream>

namespace ns3 {

    TypeId Vehicle::GetTypeId(void) {
        static TypeId tid = TypeId("ns3::Vehicle")
                .SetParent<Object > ()
                .AddConstructor<Vehicle > ()
                ;
        return tid;
    }

    /*
     * The only constructor
     */
    Vehicle::Vehicle() {
        m_node = CreateObject<Node > ();
        MobilityHelper mobility;
        mobility.Install(m_node);
        m_vehicleId = 1;
        m_lane = 0;
        m_direction = 0;
        m_velocity = 0.0;
        m_acceleration = 0.0;
        m_model = 0;
        m_laneChange = 0;
        m_length = 0;
        m_width = 0;
        m_typeId = 0;
        IsEquipped = true;
    }

    /*
     * Empty destructor
     */
    Vehicle::~Vehicle() {
    }

    /*
     * Sets up Wifi for this vehicle using the supplied helpers
     */
    void Vehicle::SetupWifi(const WifiHelper &wifi, const YansWifiPhyHelper &phy, const NqosWifiMacHelper &mac) {
        // Don't do anything if we are equipped
        if (IsEquipped == false) return;

        //Create the node container
        NetDeviceContainer d;
        NodeContainer n(m_node);
        //Use the wifi helper to install the channel on the node
        d = wifi.Install(phy, mac, n);

        // Use this for concatenating strings
        std::ostringstream oss;

        // These attach callbacks to the various types of results from Wifi
        oss << "/NodeList/" << m_node->GetId() << "/DeviceList/0/Mac/MacTx";
        Config::Connect(oss.str(), MakeCallback(&Vehicle::DevTxTrace, this));
        oss.str("");
        oss << "/NodeList/" << m_node->GetId() << "/DeviceList/0/Mac/MacRx";
        Config::Connect(oss.str(), MakeCallback(&Vehicle::DevRxTrace, this));
        oss.str("");
        oss << "/NodeList/" << m_node->GetId() << "/DeviceList/0/Phy/State/RxOk";
        Config::Connect(oss.str(), MakeCallback(&Vehicle::PhyRxOkTrace, this));
        oss.str("");
        oss << "/NodeList/" << m_node->GetId() << "/DeviceList/0/Phy/State/RxError";
        Config::Connect(oss.str(), MakeCallback(&Vehicle::PhyRxErrorTrace, this));
        oss.str("");
        oss << "/NodeList/" << m_node->GetId() << "/DeviceList/0/Phy/State/Tx";
        Config::Connect(oss.str(), MakeCallback(&Vehicle::PhyTxTrace, this));
        oss.str("");
        oss << "/NodeList/" << m_node->GetId() << "/DeviceList/0/Phy/State/State";
        Config::Connect(oss.str(), MakeCallback(&Vehicle::PhyStateTrace, this));

        // This is the final level of the callback (generally the most important)
        m_device = d.Get(0);
        m_device->SetReceiveCallback(MakeCallback(&Vehicle::ReceivePacket, this));
    }

    /**
     * Returns the vehicle type
     * 0 = vehicle, 1 = obstacle, 2 = traffic light
     */
    int Vehicle::GetVehicleType() {
        return m_typeId;
    }

    /**
     * Sets the vehicle type
     */
    void Vehicle::SetVehicleType(int typeId) {
        m_typeId = typeId;
    }

    /**
     * Sets the direction as radians off of the positive X axis
     */
    void Vehicle::SetDirection(double value) {
        m_direction = value;
    }

    /**
     * Returns the directions
     */
    double Vehicle::GetDirection() {
        return m_direction;
    }

    /**
     * Returns the unique vehicle ID for this vehicle
     */
    int Vehicle::GetVehicleId() {
        return m_vehicleId;
    }

    /**
     * Sets the vehicle ID (should be Unique, use IDGenerator)
     */
    void Vehicle::SetVehicleId(int value) {
        m_vehicleId = value;
    }

    /**
     * Gets the IDM for this Vehicle
     */
    Ptr<Model> Vehicle::GetModel() {
        return m_model;
    }

    /**
     * Sets the IDM for this Vehicle
     */
    void Vehicle::SetModel(Ptr<Model> value) {
        m_model = value;
    }

    /**
     * Gets the Lane Change model for this Vehicle
     */
    Ptr<LaneChange> Vehicle::GetLaneChange() {
        return m_laneChange;
    }

    /**
     * Sets the Lane Change model for this Vehicle
     */
    void Vehicle::SetLaneChange(Ptr<LaneChange> value) {
        m_laneChange = value;
    }

    /**
     * Returns a ns3::Vector representing the location of the center of the
     * back bumper of this vehicle
     */
    Vector Vehicle::GetPosition() {
        return m_node->GetObject<MobilityModel > ()->GetPosition();
    }

    /**
     * Sets the position of this vehicle
     */
    void Vehicle::SetPosition(Vector value) {
        m_node->GetObject<MobilityModel > ()->SetPosition(value);
    }

    /**
     * Returns the length of this Vehicle (in meters)
     */
    double Vehicle::GetLength() {
        return m_length;
    }

    /**
     * Sets the length of this Vehicle (in meters)
     */
    void Vehicle::SetLength(double value) {
        if (value < 0)
            value = 0;
        m_length = value;
    }

    /**
     * Pushes the supplied highway ID onto the top of the
     * destination stack
     */
    void Vehicle::AddDestination(int destinationId) {
        destinations.push_front(destinationId);
    }

    /**
     * Returns the destination on the top of the dstination stack
     * Returns -100000 if it is empty
     */
    int Vehicle::GetDestination() {
        if (destinations.empty()) {
            return -100000;
        } else {
            return destinations.front();
        }
    }

    /**
     * Removes the destination on the top of the destination stack
     */
    void Vehicle::ArriveAtDestination() {
        destinations.pop_front();

    }

    /**
     * Returns the width of vehicle (in meters)
     */
    double Vehicle::GetWidth() {
        return m_width;
    }

    /**
     * Sets the width of the vehicle
     */
    void Vehicle::SetWidth(double value) {
        if (value < 0)
            value = 0;
        m_width = value;
    }

    /**
     * returns the current velocity of the vehicle
     */
    double Vehicle::GetVelocity() {
        return m_velocity;
    }

    /**
     * Sets the current velocity of the vehicle
     */
    void Vehicle::SetVelocity(double value) {
        m_velocity = value;
    }

    /**
     * returns the current acceleration of the vehicle
     */
    double Vehicle::GetAcceleration() {
        return m_acceleration;
    }

    /**
     * Sets the current acceleration of the vehicle
     */
    void Vehicle::SetAcceleration(double acc) {
        m_acceleration = acc;
    }

    /**
     * Returns the lane ID this vehicle is currently associated with
     */
    int Vehicle::GetLane() {
        return m_lane;
    }

    /**
     * Sets the lane ID this vehicle is currently associated with
     */
    void Vehicle::SetLane(int value) {
        m_lane = value;
    }

    /**
     * Sets the acceleration based on the vehicle in front of this
     * Vehicle and the distance from this vehicle to the next vehicle
     */
    void Vehicle::Accelerate(Ptr<Vehicle> vwd, double distance) {
        m_acceleration = Acceleration(vwd, distance);
    }

    /**
     * Calculates and returns the acceleration based on the vehicle in front
     * of this Vehicle and the distance from this vehicle to the vehicle in front
     */
    double Vehicle::Acceleration(Ptr<Vehicle> vwd, double distance) {
        //This is the highest decelleration this Vehicle can do
        double safeValue = m_laneChange->GetMaxSafeBreakingDeceleration();
        //Get the acceleration from the associated model
        double accelValue = m_model->CalculateAcceleration(Ptr<Vehicle > (this), vwd, distance);
        //If it is a stoplight and the decelleration is more than the safe
        //acceleration, return NAN as a signal that vehicle will not change acceleration
        if (vwd != 0 && vwd->GetVehicleType() == 2 &&
                fabs(accelValue) > fabs(safeValue)) {
            accelValue = NAN;
        }
        return accelValue;
    }

    /**
     * Adjust the position of the vehicle based on direction, velocity,
     * and direciton
     */
    void Vehicle::TranslatePosition(double dt) {
        // Get the current position
        Vector v = this->GetPosition();
        // The array holding the result
        double result[2];
        // Initialize the array with the starting point
        result[0] = v.x;
        result[1] = v.y;
        // Use Geometry.h to move the point
        translatePoint(result, m_direction, dt * m_velocity);
        // Set the position based on the result
        v.x = result[0];
        v.y = result[1];
        this->SetPosition(v);
    }

    /**
     * Adjust the velocity of the Vehicle based on the current acceleration
     */
    void Vehicle::TranslateVelocity(double dt) {
        m_velocity += (m_acceleration * dt);
        if (m_velocity <= 0.0) {
            m_velocity = 0.0;
        }
    }

    /**
     * \param frontOld the Vehicle in the front at the same lane.
     * \param distanceFOld the distance to frontOld
     * \param frontNew the Vehicle in the front at the target lane.
     * \param distanceFNew the distance to frontNew
     * \param backNew the Vehicle in back at the target lane.
     * \param distanceBNew the distance to backNew
     * \param toLeft the inclination for the target lane. True: target lane is left, False: target lane is right.
     * \returns true if the change of the lane is possible for the Vehicle.
     */
    bool Vehicle::CheckLaneChange(Ptr<Vehicle> frontOld, double distanceFOld, Ptr<Vehicle> frontNew, double distanceFNew, Ptr<Vehicle> backNew, double distanceBNew, bool toLeft) {
        return m_laneChange->CheckLaneChange(Ptr<Vehicle > (this), frontOld, distanceFOld, frontNew, distanceFNew, backNew, distanceBNew, toLeft);
    }

    /**
     * Returns true if v1 is in front of v2
     * Only works if v1 and v2 are travelling in the same direction
     * along the same line
     */
    bool Vehicle::Compare(Ptr<Vehicle> v1, Ptr<Vehicle> v2) {

        return !anglesEqual(v1->GetDirection(),
                angleToPoint(v1->GetPosition().x, v1->GetPosition().y,
                v2->GetPosition().x, v2->GetPosition().y));
    }

    void Vehicle::DevTxTrace(std::string context, Ptr<const Packet> p) {
        if (!m_devTxTrace.IsNull())
            m_devTxTrace(Ptr<Vehicle > (this), context, p);
    }

    void Vehicle::DevRxTrace(std::string context, Ptr<const Packet> p) {
        if (!m_devRxTrace.IsNull())
            m_devRxTrace(Ptr<Vehicle > (this), context, p);
    }

    void Vehicle::PhyRxOkTrace(std::string context, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble) {
        if (!m_phyRxOkTrace.IsNull())
            m_phyRxOkTrace(Ptr<Vehicle > (this), context, packet, snr, mode, preamble);
    }

    void Vehicle::PhyRxErrorTrace(std::string context, Ptr<const Packet> packet, double snr) {
        if (!m_phyRxErrorTrace.IsNull())
            m_phyRxErrorTrace(Ptr<Vehicle > (this), context, packet, snr);
    }

    void Vehicle::PhyTxTrace(std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower) {
        if (!m_phyTxTrace.IsNull())
            m_phyTxTrace(Ptr<Vehicle > (this), context, packet, mode, preamble, txPower);
    }

    void Vehicle::PhyStateTrace(std::string context, Time start, Time duration, enum WifiPhy::State state) {
        if (!m_phyStateTrace.IsNull())
            m_phyStateTrace(Ptr<Vehicle > (this), context, start, duration, state);
    }

    bool Vehicle::ReceivePacket(Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol, const Address& address) {
        if (!m_receive.IsNull())
            m_receive(Ptr<Vehicle > (this), packet, address);
        return true;
    }

    Address Vehicle::GetAddress() {
        return m_device->GetAddress();
    }

    Address Vehicle::GetBroadcastAddress() {
        return m_device->GetBroadcast();
    }

    bool Vehicle::SendTo(Address address, Ptr<Packet> packet) {
        return m_device->Send(packet, address, 1);
    }

    VehicleReceiveCallback Vehicle::GetReceiveCallback() {
        return m_receive;
    }

    void Vehicle::SetReceiveCallback(VehicleReceiveCallback receive) {
        m_receive = receive;
    }

    DeviceTraceCallback Vehicle::GetDevTxTraceCallback() {
        return m_devTxTrace;
    }

    void Vehicle::SetDevTxTraceCallback(DeviceTraceCallback devTxTrace) {
        m_devTxTrace = devTxTrace;
    }

    DeviceTraceCallback Vehicle::GetDevRxTraceCallback() {
        return m_devRxTrace;
    }

    void Vehicle::SetDevRxTraceCallback(DeviceTraceCallback devRxTrace) {
        m_devRxTrace = devRxTrace;
    }

    PhyRxOkTraceCallback Vehicle::GetPhyRxOkTraceCallback() {
        return m_phyRxOkTrace;
    }

    void Vehicle::SetPhyRxOkTraceCallback(PhyRxOkTraceCallback phyRxOkTrace) {
        m_phyRxOkTrace = phyRxOkTrace;
    }

    PhyRxErrorTraceCallback Vehicle::GetPhyRxErrorTraceCallback() {
        return m_phyRxErrorTrace;
    }

    void Vehicle::SetPhyRxErrorTraceCallback(PhyRxErrorTraceCallback phyRxErrorTrace) {
        m_phyRxErrorTrace = phyRxErrorTrace;
    }

    PhyTxTraceCallback Vehicle::GetPhyTxTraceCallback() {
        return m_phyTxTrace;
    }

    void Vehicle::SetPhyTxTraceCallback(PhyTxTraceCallback phyTxTrace) {
        m_phyTxTrace = phyTxTrace;
    }

    PhyStateTraceCallback Vehicle::GetPhyStateTraceCallback() {
        return m_phyStateTrace;
    }

    void Vehicle::SetPhyStateTraceCallback(PhyStateTraceCallback phyStateTrace) {
        m_phyStateTrace = phyStateTrace;
    }
}
