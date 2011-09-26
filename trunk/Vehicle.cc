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

    Vehicle::~Vehicle() {
    }

    void Vehicle::SetupWifi(const WifiHelper &wifi, const YansWifiPhyHelper &phy, const NqosWifiMacHelper &mac) {
        if (IsEquipped == false) return;
        NetDeviceContainer d;
        NodeContainer n(m_node);
        d = wifi.Install(phy, mac, n);

        std::ostringstream oss;

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

        m_device = d.Get(0);
        m_device->SetReceiveCallback(MakeCallback(&Vehicle::ReceivePacket, this));
    }

    int Vehicle::GetVehicleType() {
        return m_typeId;
    }

    void Vehicle::SetVehicleType(int typeId) {
        m_typeId = typeId;
    }

    void Vehicle::SetDirection(double value) {
        m_direction = value;
    }

    double Vehicle::GetDirection() {
        return m_direction;
    }

    int Vehicle::GetVehicleId() {
        return m_vehicleId;
    }

    void Vehicle::SetVehicleId(int value) {
        m_vehicleId = value;
    }

    Ptr<Model> Vehicle::GetModel() {
        return m_model;
    }

    void Vehicle::SetModel(Ptr<Model> value) {
        m_model = value;
    }

    Ptr<LaneChange> Vehicle::GetLaneChange() {
        return m_laneChange;
    }

    void Vehicle::SetLaneChange(Ptr<LaneChange> value) {
        m_laneChange = value;
    }

    Vector Vehicle::GetPosition() {
        return m_node->GetObject<MobilityModel > ()->GetPosition();
    }

    void Vehicle::SetPosition(Vector value) {
        m_node->GetObject<MobilityModel > ()->SetPosition(value);
    }

    double Vehicle::GetLength() {
        return m_length;
    }

    void Vehicle::SetLength(double value) {
        if (value < 0)
            value = 0;
        m_length = value;
    }

    void Vehicle::AddDestination(int destinationId) {
        destinations.push_front(destinationId);
    }

    int Vehicle::GetDestination() {
        if (destinations.empty()) {
            return -100000;
        } else {
            return destinations.front();
        }
    }

    void Vehicle::ArriveAtDestination() {
        destinations.pop_front();

    }

    double Vehicle::GetWidth() {
        return m_width;
    }

    void Vehicle::SetWidth(double value) {
        if (value < 0)
            value = 0;
        m_width = value;
    }

    double Vehicle::GetVelocity() {
        return m_velocity;
    }

    void Vehicle::SetVelocity(double value) {
        m_velocity = value;
    }

    double Vehicle::GetAcceleration() {
        return m_acceleration;
    }

    void Vehicle::SetAcceleration(double acc) {
        m_acceleration = acc;
    }

    int Vehicle::GetLane() {
        return m_lane;
    }

    void Vehicle::SetLane(int value) {
        m_lane = value;
    }

    void Vehicle::Accelerate(Ptr<Vehicle> vwd, double distance) {
        m_acceleration = Acceleration(vwd, distance);
    }

    double Vehicle::Acceleration(Ptr<Vehicle> vwd, double distance) {
        double safeValue = m_laneChange->GetMaxSafeBreakingDeceleration();
        double accelValue = m_model->CalculateAcceleration(Ptr<Vehicle > (this), vwd, distance);
        //If it is a stoplight and the decelleration is more than the safe
        //acceleration, return NAN as a signal that vehicle will not change acceleration

        if (vwd != 0 && vwd->GetVehicleType() == 2 &&
                fabs(accelValue) > fabs(safeValue)) {
            accelValue = NAN;
        }
        return accelValue;
    }

    void Vehicle::TranslatePosition(double dt) {
        Vector v = this->GetPosition();
        double result[2];
        result[0] = v.x;
        result[1] = v.y;
        translatePoint(result, m_direction, dt * m_velocity);
        v.x = result[0];
        v.y = result[1];
        this->SetPosition(v);
    }

    void Vehicle::TranslateVelocity(double dt) {
        m_velocity += (m_acceleration * dt);
        if (m_velocity <= 0.0) {
            m_velocity = 0.0;
        }
    }

    bool Vehicle::CheckLaneChange(Ptr<Vehicle> frontOld, double distanceFOld, Ptr<Vehicle> frontNew, double distanceFNew, Ptr<Vehicle> backNew, double distanceBNew, bool toLeft) {
        return m_laneChange->CheckLaneChange(Ptr<Vehicle > (this), frontOld, distanceFOld, frontNew, distanceFNew, backNew, distanceBNew, toLeft);
    }

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
