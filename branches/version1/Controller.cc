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

#include <iostream>
#include <sstream>
#include "Controller.h"

using namespace std;
using namespace ns3;

namespace ns3
{
  Controller::Controller()
  { 
	  T=-1.0; 
	  Plot=false;
  }
  Controller::Controller(Ptr<Highway> highway)
  {
    this->highway=highway;
  }

  void Controller::SetHighway(Ptr<Highway> highway)
  {
    this->highway=highway;
  }
  
  Ptr<Highway> Controller::GetHighway()
  {
    return this->highway;
  }
  
  bool Controller::InitVehicle(Ptr<Highway> highway, int& VID)
  {
      // Block the road with warning, 500 meters away, at the right most lane [lane=0, dir=1]
      Ptr<Obstacle> obstacle=CreateObject<Obstacle>();
      obstacle->SetupWifi(highway->GetWifiHelper(), highway->GetYansWifiPhyHelper(), highway->GetNqosWifiMacHelper());
      obstacle->SetVehicleId(VID++);
      obstacle->SetDirection(1);
      obstacle->SetLane(0);
      obstacle->SetPosition(Vector(500, highway->GetYForLane(0,1), 0));
      obstacle->SetLength(8);
      obstacle->SetWidth(2);
      obstacle->SetReceiveCallback(highway->GetReceiveDataCallback());
      highway->AddVehicle(obstacle);
      Simulator::Schedule(Seconds(0.0), &Controller::BroadcastWarning, this, obstacle);
      		
      // Create a police car moving toward the blocked spot, remember its VehicleId will be 2 
      // We also show how we can assign this car new wifiphy instead of using default highway wifi,
      // for example a police with a stronger transmission.
      // Remember all the vehicles must use the same shared wifi channel which already hold by highway.
      // Highway TxPower default value is 21.0 for (250m-300m) range, 30.0 makes the police has higher range.
      // Also let police car has a higher max speed.
      YansWifiPhyHelper policePhyHelper = YansWifiPhyHelper::Default();
      policePhyHelper.SetChannel(highway->GetWifiChannel());
      policePhyHelper.Set("TxPowerStart",DoubleValue(30.0));
      policePhyHelper.Set("TxPowerEnd",DoubleValue(30.0));			

      Ptr<Vehicle> police=CreateObject<Vehicle>();
      police->SetupWifi(highway->GetWifiHelper(), policePhyHelper, highway->GetNqosWifiMacHelper());
      police->SetVehicleId(VID++);
      police->SetDirection(1);
      police->SetLane(1);
      police->SetPosition(Vector(0.0, highway->GetYForLane(1,1), 0));
      police->SetVelocity(0.0);
      police->SetAcceleration(0.0);
      Ptr<Model> policeModel=highway->CreateSedanModel();
      policeModel->SetDesiredVelocity(36.0);  // max speed 36(m/s)
      police->SetModel(policeModel);          // or common sedan model: highway->GetSedanModel()
      police->SetLaneChange(highway->GetSedanLaneChange());
      police->SetLength(4);
      police->SetWidth(2);
      police->SetReceiveCallback(highway->GetReceiveDataCallback());
      highway->AddVehicle(police);
	   
      highway->SetAutoInject(true);

      // Return true: a signal to highway that the lane lists (queues) in where obstacles and vehicles are being added
      // must be sorted based on their positions.
      // Return false: to ignore sorting (do not return false when vehicles are manually added to the highway).
      return true;
  }
	
  bool Controller::ControlVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle, double dt)
  {
    // we aim to create outputs which are readable by gnuplot for visulization purpose
    // this can be happen at beginning of each simulation step here. 
    if(Plot==true)
    {
      bool newStep=false;
      double now=Simulator::Now().GetHighPrecision().GetDouble();
      if(now > T)
      {
        T = now;
        newStep=true;
      }
			
      if(newStep==true)
      {
        if(T!=0.0)
        {
          cout << "e" << endl;
          //cout << "pause " << dt << endl;
        }
        float xrange = highway->GetHighwayLength();
        float yrange = highway->GetLaneWidth()*highway->GetNumberOfLanes();
        if(highway->GetTwoDirectional()) yrange=2*yrange + highway->GetMedianGap();
        cout << "set xrange [0:"<< xrange <<"]" << endl;
        cout << "set yrange [0:"<< yrange <<"]" << endl;
        cout << "plot '-' w points" << endl;
        newStep=false;
      }
      if(newStep==false)
      {
        cout << vehicle->GetPosition().x << " " << vehicle->GetPosition().y << endl;
      }
    }

    // to decelerate and stop the police car reaching the obstacle 
    if(vehicle->GetVehicleId()==2 && vehicle->GetPosition().x >=400)
    {
      vehicle->SetAcceleration(-2.0);
      // return true: a signal to highway that we aim to manually control the vechile 
      return true;
    }

    // return false: a signal to highway that lets the vehicle automatically be handled (using IDM/MOBIL rules)
    return false;
  }
	
  void Controller::BroadcastWarning(Ptr<Vehicle> veh)
  {
    stringstream msg;
    msg << veh->GetVehicleId() 
        << " " << veh->GetPosition().x 
        << " has blocked the road at x=" << veh->GetPosition().x 
        << " direction=" << veh->GetDirection()
        << " lane=" << veh->GetLane();
		
    Ptr<Packet> packet = Create<Packet>((uint8_t*) msg.str().c_str(), msg.str().length());

    veh->SendTo(veh->GetBroadcastAddress(), packet);
		
    Simulator::Schedule(Seconds(5.0),&Controller::BroadcastWarning, this, veh);
  }

  void Controller::ReceiveData(Ptr<Vehicle> veh, Ptr<const Packet> packet, Address address)
  {
    string data=string((char*)packet->PeekData());
    stringstream ss (stringstream::in | stringstream::out);
			
    double obs_id, obs_x; 
    ss << data;
    ss >> obs_id;
    ss >> obs_x;

    int vid=veh->GetVehicleId();
    double now=Simulator::Now().GetSeconds();

    if(!Plot)
      cout << "at t=" << now << " vehicle " << vid << " received message=[" << data << "]" << endl;

    //the police reaction to the received message
    if(vid==2)
    {
      double diff=obs_x - veh->GetPosition().x;
      stringstream msg;
      if(diff>2)
        msg << "police is only " << obs_x - veh->GetPosition().x << "(m) away from you vehicle " << obs_id;  
      else
        msg << "police is at the location x~" << obs_x << " to observe the case with id=" << obs_id;

      Ptr<Packet> packet = Create<Packet>((uint8_t*) msg.str().c_str(), msg.str().length());
      veh->SendTo(address, packet);
    }
  }
}
