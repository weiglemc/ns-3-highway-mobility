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
#include "Highway.h"
#include "ns3/simulator.h"
#include <math.h>

namespace ns3
{
  TypeId Highway::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::Highway")
    .SetParent<Object> ()
    .AddConstructor<Highway> ()
    ;
    return tid;
  }

  void Highway::InitHighway()
  {
    if(m_laneChangeSedan == 0)
      {
        m_laneChangeSedan = CreateSedanLaneChangeModel();
      }
    if(m_laneChangeTruck == 0)
      {
        m_laneChangeTruck = CreateTruckLaneChangeModel();
      }
    if(m_sedan == 0)
      {
        m_sedan = CreateSedanModel();
      }
    if(m_truck == 0)
      {
        m_truck = CreateTruckModel();
      }

    m_vehicleId = 1;
    bool init = false;
    
	if(!m_initVehicle.IsNull())
	  {
        init = m_initVehicle(Ptr<Highway>(this), m_vehicleId);
	  }

    if(init==true)
      {
        for(int i=0;i<m_numberOfLanes;i++)
          {
            m_vehicles[i].sort(ns3::Vehicle::Compare);
            if(m_twoDirectional==true) 
              m_vehiclesOpp[i].sort(ns3::Vehicle::Compare);
          }
      }

    if(m_autoInject==true) 
	  InjectVehicles(m_injectionSafetyGap, (int)m_sedanTruckPerc);
  }

  Ptr<Model> Highway::CreateSedanModel()
  {
    Ptr<Model> model = CreateObject<Model>();
    model->SetDesiredVelocity(25.0);
    model->SetDeltaV(4.0);
    model->SetAcceleration(0.5);  
    model->SetDeceleration(3.0);  
    model->SetMinimumGap(2.0);
    model->SetTimeHeadway(0.1);
    model->SetSqrtAccelerationDeceleration(sqrt(model->GetAcceleration() * model->GetDeceleration()));
    return model;
  }

  Ptr<LaneChange> Highway::CreateSedanLaneChangeModel()
  {
    Ptr<LaneChange> laneChange = CreateObject<LaneChange>();
    laneChange->SetPolitenessFactor(0.2);
    laneChange->SetDbThreshold(0.3);
    laneChange->SetGapMin(2.0);
    laneChange->SetMaxSafeBreakingDeceleration(12.0);
    laneChange->SetBiasRight(0.2);
    return laneChange;
  }

  Ptr<Model> Highway::CreateTruckModel()
  {
    Ptr<Model> model = CreateObject<Model>();
    model->SetDesiredVelocity(22.0);
    model->SetDeltaV(4.0);
    model->SetAcceleration(0.2);
    model->SetDeceleration(4.0);
    model->SetMinimumGap(2.0);
    model->SetTimeHeadway(0.5);
    model->SetSqrtAccelerationDeceleration(sqrt(model->GetAcceleration() * model->GetDeceleration()));
    return model;
  }

  Ptr<LaneChange> Highway::CreateTruckLaneChangeModel()
  {
    Ptr<LaneChange> laneChange = CreateObject<LaneChange>();
    laneChange->SetPolitenessFactor(0.2);
    laneChange->SetDbThreshold(0.2);
    laneChange->SetGapMin(2.0);
    laneChange->SetMaxSafeBreakingDeceleration(12.0);
    laneChange->SetBiasRight(0.3);
    return laneChange;
  }

  Ptr<Vehicle> Highway::GetVehicle(std::list<Ptr<Vehicle> > v, int index)
  {
	std::list<Ptr<Vehicle> >::iterator i=v.begin();
    advance(i,index);
    return *i;
  }

  double Highway::GetYForLane(int lane, int dir)
  {
    if(dir == 1)
	  {
		return m_laneWidth * (lane + 0.5);
	  }
    return ((2 * m_numberOfLanes * m_laneWidth) + m_medianGap - GetYForLane(lane, 1));
  }

  double Highway::GetMedianGap()
  {
    return m_medianGap;
  }

  void Highway::SetMedianGap(double value)
  {
    m_medianGap=value;
  }

  double Highway::GetInjectionGap()
  {
    return m_injectionSafetyGap;
  }

  void Highway::SetInjectionGap(double value)
  {
    if(value > 0 )
      m_injectionSafetyGap= value;
  }

  double Highway::GetInjectionMixValue()
  {
    return m_sedanTruckPerc;
  }

  void Highway::SetInjectionMixValue(double value)
  {
    if(value>=0 && value<=100) 
	  m_sedanTruckPerc=value;
  }

  bool Highway::GetChangeLane()
  {
    return m_changeLaneSet;
  }

  void Highway::SetChangeLane(bool value)
  {
    m_changeLaneSet=value;
  }

  void Highway::InjectVehicles(double minGap, int p)
  {
    UniformVariable uRnd(0,100);
	UniformVariable uRnd2(0,100);
    double rate1 = m_RVFlowDirPos.GetValue();
    double rate2 = m_RVFlowDirNeg.GetValue();
	int last;
	double gap;
	double vel = 0.0;

	last = m_vehicles[m_currentLaneDirPos].size()-1;
	if (last < 0)
	  {
        gap = minGap + 1;
	  }
	else
	  {
        gap = GetVehicle(m_vehicles[m_currentLaneDirPos],last)->GetPosition().x;
        vel = m_velocityDirPos;//GetVehicle(m_vehicles[m_currentLaneDirPos],last)->GetVelocity();
      }

    m_remainderDirPos += rate1;
		if(m_remainderDirPos >= 1 && gap > minGap)
	      {
			m_remainderDirPos -= 1;
            if (uRnd.GetValue() <= p)        
              {
                Ptr<Vehicle> temp=CreateObject<Vehicle>();
				if(uRnd2.GetValue() <= m_penetrationRate) 
				{
					temp->IsEquipped = true;
					temp->SetupWifi(m_wifiHelper, m_wifiPhyHelper, m_wifiMacHelper);
				}
				else
				{
					temp->IsEquipped=false;
				}
                temp->SetVehicleId(m_vehicleId++);
                temp->SetDirection(1);
                temp->SetPosition(Vector(-4,GetYForLane(m_currentLaneDirPos,1),0));
                temp->SetLane(m_currentLaneDirPos);
                temp->SetVelocity(vel);
                temp->SetAcceleration(0.0);
				Ptr<Model> tempModel=CreateSedanModel();
				tempModel->SetDesiredVelocity(m_RVSpeed.GetValue());
                temp->SetModel(tempModel);
                temp->SetLaneChange(m_laneChangeSedan);
                temp->SetLength(4);
                temp->SetWidth(2);
				if(temp->IsEquipped ==true)
				{
                temp->SetReceiveCallback(m_receiveData);
                temp->SetDevTxTraceCallback(m_devTxTrace);
                temp->SetDevRxTraceCallback(m_devRxTrace);
                temp->SetPhyRxOkTraceCallback(m_phyRxOkTrace);
                temp->SetPhyRxErrorTraceCallback(m_phyRxErrorTrace);
                temp->SetPhyTxTraceCallback(m_phyTxTrace);
                temp->SetPhyStateTraceCallback(m_phyStateTrace);
				}
                m_vehicles[m_currentLaneDirPos].push_back(temp);
              }
            else
              {
                Ptr<Vehicle> temp=CreateObject<Vehicle>();
                if(uRnd2.GetValue() <= m_penetrationRate)
				{
					temp->IsEquipped = true;
					temp->SetupWifi(m_wifiHelper, m_wifiPhyHelper, m_wifiMacHelper);
				}
			    else
				{
					temp->IsEquipped=false;
				}
                temp->SetVehicleId(m_vehicleId++);
                temp->SetDirection(1);
                temp->SetPosition(Vector(-8,GetYForLane(m_currentLaneDirPos,1),0));
                temp->SetLane(m_currentLaneDirPos);
                temp->SetVelocity(vel);
                temp->SetAcceleration(0.0);
				Ptr<Model> tempModel=CreateTruckModel();
				tempModel->SetDesiredVelocity(m_RVSpeed.GetValue());
                temp->SetModel(tempModel);
                temp->SetLaneChange(m_laneChangeTruck);
                temp->SetLength(8);
                temp->SetWidth(2);
				if(temp->IsEquipped ==true)
				{
                temp->SetReceiveCallback(m_receiveData);
                temp->SetDevTxTraceCallback(m_devTxTrace);
                temp->SetDevRxTraceCallback(m_devRxTrace);
                temp->SetPhyRxOkTraceCallback(m_phyRxOkTrace);
                temp->SetPhyRxErrorTraceCallback(m_phyRxErrorTrace);
                temp->SetPhyTxTraceCallback(m_phyTxTrace);
                temp->SetPhyStateTraceCallback(m_phyStateTrace);
				}
                m_vehicles[m_currentLaneDirPos].push_back(temp);
		      }
	        m_currentLaneDirPos++;
	        if(m_currentLaneDirPos >= m_numberOfLanes) m_currentLaneDirPos = 0;
		  }

    if(m_twoDirectional==true)
	  {		
    	last = m_vehiclesOpp[m_currentLaneDirNeg].size()-1;
	    if (last < 0)
	      {
            gap = minGap + 1;
	      }
	    else
	      {
            gap = m_highwayLength - GetVehicle(m_vehiclesOpp[m_currentLaneDirNeg],last)->GetPosition().x;
            vel = m_velocityDirNeg;//GetVehicle(m_vehiclesOpp[m_currentLaneDirNeg],last)->GetVelocity();
          }

          m_remainderDirNeg += rate2;
		  if(m_remainderDirNeg >= 1 && gap > minGap)
	      {
			m_remainderDirNeg -= 1;
            if (uRnd.GetValue() <= p)        
              {
                Ptr<Vehicle> temp=CreateObject<Vehicle>();
                if(uRnd2.GetValue() <= m_penetrationRate)
				{
					temp->IsEquipped = true;
					temp->SetupWifi(m_wifiHelper, m_wifiPhyHelper, m_wifiMacHelper);
				}
			    else
				{
					temp->IsEquipped=false;
				}
                temp->SetVehicleId(m_vehicleId++);
                temp->SetDirection(-1);
                temp->SetPosition(Vector(m_highwayLength+4,GetYForLane(m_currentLaneDirNeg,-1),0));
                temp->SetLane(m_currentLaneDirNeg);
                temp->SetVelocity(vel);
                temp->SetAcceleration(0.0);
				Ptr<Model> tempModel=CreateSedanModel();
				tempModel->SetDesiredVelocity(m_RVSpeed.GetValue());
                temp->SetModel(tempModel);
                temp->SetLaneChange(m_laneChangeSedan);
                temp->SetLength(4);
                temp->SetWidth(2);
				if(temp->IsEquipped == true)
				{
                temp->SetReceiveCallback(m_receiveData);
                temp->SetDevTxTraceCallback(m_devTxTrace);
                temp->SetDevRxTraceCallback(m_devRxTrace);
                temp->SetPhyRxOkTraceCallback(m_phyRxOkTrace);
                temp->SetPhyRxErrorTraceCallback(m_phyRxErrorTrace);
                temp->SetPhyTxTraceCallback(m_phyTxTrace);
                temp->SetPhyStateTraceCallback(m_phyStateTrace);
				}
                m_vehiclesOpp[m_currentLaneDirNeg].push_back(temp);
              }
            else
              {
                Ptr<Vehicle> temp=CreateObject<Vehicle>();
                if(uRnd2.GetValue() <= m_penetrationRate)
				{
					temp->IsEquipped = true;
					temp->SetupWifi(m_wifiHelper, m_wifiPhyHelper, m_wifiMacHelper);
				}
				else
				{
					temp->IsEquipped=false;
				}
                temp->SetVehicleId(m_vehicleId++);
                temp->SetDirection(-1);
                temp->SetPosition(Vector(m_highwayLength+8,GetYForLane(m_currentLaneDirNeg,-1),0));
                temp->SetLane(m_currentLaneDirNeg);
                temp->SetVelocity(vel);
                temp->SetAcceleration(0.0);
				Ptr<Model> tempModel=CreateTruckModel();
				tempModel->SetDesiredVelocity(m_RVSpeed.GetValue());
                temp->SetModel(tempModel);
                temp->SetLaneChange(m_laneChangeTruck);
                temp->SetLength(8);
                temp->SetWidth(2);
				if(temp->IsEquipped == true)
				{
                temp->SetReceiveCallback(m_receiveData);
                temp->SetDevTxTraceCallback(m_devTxTrace);
                temp->SetDevRxTraceCallback(m_devRxTrace);
                temp->SetPhyRxOkTraceCallback(m_phyRxOkTrace);
                temp->SetPhyRxErrorTraceCallback(m_phyRxErrorTrace);
                temp->SetPhyTxTraceCallback(m_phyTxTrace);
                temp->SetPhyStateTraceCallback(m_phyStateTrace);
				}
                m_vehiclesOpp[m_currentLaneDirNeg].push_back(temp);
		      }
	        m_currentLaneDirNeg++;
	        if(m_currentLaneDirNeg >= m_numberOfLanes) m_currentLaneDirNeg = 0;
		  }
	    
      }
  }
  void Highway::TranslateVehicles()
  {
    if(m_stopped==true) return;
    static int loop=0;
    // NOTE: ORDER OF CALLING THIS FUNCTIONS IS VERY VERY IMPORTANT (EFFECT OF CURRENT SPEED, POSITION, DECICION)
    if(loop==10) loop=0;
    if(loop==0 && m_changeLaneSet==true) 
      {
        ChangeLane(m_vehicles);
        if(m_twoDirectional==true) 
          ChangeLane(m_vehiclesOpp);
      }

    TranslatePositionVelocity(m_vehicles, m_dt);
    if(m_twoDirectional==true) 
	  TranslatePositionVelocity(m_vehiclesOpp, m_dt); 
    
	Accelerate(m_vehicles, m_dt);
    
	if(m_twoDirectional==true) 
	  Accelerate(m_vehiclesOpp, m_dt);
    
    if(m_autoInject==true) 
	  InjectVehicles(m_injectionSafetyGap, (int)m_sedanTruckPerc);
 
    loop++;
	Simulator::Schedule(Seconds(m_dt), &Highway::Step, Ptr<Highway>(this));    
  }

  void Highway::Accelerate(std::list<Ptr<Vehicle> > vehicles[], double dt)
  {
    for (int i = 0; i < m_numberOfLanes; i++)
      {
        for (uint j = 0; j < vehicles[i].size(); j++)
          {
            Ptr<Vehicle> veh=GetVehicle(vehicles[i],j);
            bool controled=false;
            if(!m_controlVehicle.IsNull()) 
		      controled=m_controlVehicle(Ptr<Highway>(this), veh, dt);
            if(controled==false)
              {
                if (j == 0)
                  {
                    veh->Accelerate(0);
                    continue;
                  }
                veh->Accelerate(GetVehicle(vehicles[i],j - 1));
              }
           }
      }
  }

  void Highway::TranslatePositionVelocity(std::list<Ptr<Vehicle> > vehicles[], double dt)
  {
    std::list<Ptr<Vehicle> > reachedEnd;
    for (int i = 0; i < m_numberOfLanes; i++)
      {
        for (uint j = 0; j < vehicles[i].size(); j++)
          {
            Ptr<Vehicle> veh=GetVehicle(vehicles[i],j);
            veh->TranslatePosition(dt);
            veh->TranslateVelocity(dt);
            if(veh->GetPosition().x > m_highwayLength && veh->GetDirection()==1) 
			  reachedEnd.push_back(veh);
            else if(veh->GetPosition().x <0 && veh->GetDirection()==-1) 
			  reachedEnd.push_back(veh);
          }

        for(uint r=0; r<reachedEnd.size(); r++)
          {
            Ptr<Vehicle> rm=GetVehicle(reachedEnd, r);
            vehicles[i].remove(rm);
            if(rm->IsEquipped==true) rm->GetReceiveCallback().Nullify();
            // to put vehicle's node far away from the highway
            // we cannot dispose the vehicle here because its node may still be involved in send and receive process
            rm->SetPosition(Vector(10000, 10000, 10000)); 
            rm=0;
          }

        reachedEnd.clear();
      }
  }

  void Highway::ChangeLane(std::list<Ptr<Vehicle> > vehicles[])
  {
    if (m_numberOfLanes <= 1)
      {
        return;
      }

    for (int i = 0; i < m_numberOfLanes; i++)
      {
        if (i < 1)
          {
            DoChangeLaneIfPossible(vehicles,i, i + 1);
            continue;
          }
        if (i + 1 >= m_numberOfLanes)
          {
            DoChangeLaneIfPossible(vehicles,i, i - 1);
            continue;
          }
        DoChangeLaneIfPossible(vehicles, i, i + 1);
        DoChangeLaneIfPossible(vehicles, i, i - 1);
      }     
  }

  void Highway::DoChangeLaneIfPossible(std::list<Ptr<Vehicle> > vehicles[], int curLane, int desLane)
  {
	std::list<Ptr<Vehicle> > canChange;
    canChange.clear();
    
	for (uint j = 0; j < vehicles[curLane].size(); j++)
      {
        Ptr<Vehicle> fOld = 0;
        if (j > 0)
          {
            fOld = GetVehicle(vehicles[curLane],j - 1);
          }

        FindSideVehicles(vehicles, GetVehicle(vehicles[curLane],j), desLane);
        if (GetVehicle(vehicles[curLane],j)->CheckLaneChange(fOld, m_tempVehicles[0], m_tempVehicles[1], (curLane < desLane) ? true : false))
          {
            canChange.push_back(GetVehicle(vehicles[curLane],j));
          }               
      }

    for (uint j = 0; j < canChange.size(); j++)
      {
        Vector position=GetVehicle(canChange,j)->GetPosition();
        position.y=GetYForLane(desLane, GetVehicle(canChange,j)->GetDirection());
        GetVehicle(canChange,j)->SetLane(desLane);
        GetVehicle(canChange,j)->SetPosition(position);
        vehicles[curLane].remove(GetVehicle(canChange,j));
        vehicles[desLane].push_back(GetVehicle(canChange,j));
      }

	m_vehicles[desLane].sort(ns3::Vehicle::Compare);
  }

  void Highway::FindSideVehicles(std::list<Ptr<Vehicle> > vehicles[], Ptr<Vehicle> veh, int sideLane)
  {
    int front=-1, back=-1;
    m_tempVehicles[0] = 0;
	m_tempVehicles[1] = 0;
    for (uint i = 0; i < vehicles[sideLane].size(); i++)
      {
        if(veh->GetDirection() == 1)
          {	
            if (GetVehicle(vehicles[sideLane],i)->GetPosition().x <= veh->GetPosition().x)
              {
                back = i;
                front = back - 1;
                break;
              }
          }
        else
          {
            if (GetVehicle(vehicles[sideLane],i)->GetPosition().x >= veh->GetPosition().x)
            {
              back = i;
              front = back - 1;
              break;
            }
          }
      }

    if (back < 0)
      {
        front = vehicles[sideLane].size()-1;
      }
    if (back > -1)
      {
        m_tempVehicles[1] = GetVehicle(vehicles[sideLane], back);
      }
    if (front > -1)
      {
        m_tempVehicles[0] = GetVehicle(vehicles[sideLane], front);
      }
  }

  Highway::Highway()
  {
    m_dt=0.1;
    m_vehicleId=1;
    m_numberOfLanes=1;
    m_highwayLength=1000;
    m_laneWidth=5;
    m_laneChangeSedan=0;
    m_laneChangeTruck=0;
    m_sedan=0;
    m_truck=0;
    m_autoInject=true;
    m_twoDirectional=false;
    m_medianGap=5;
    m_injectionSafetyGap=50;
    m_sedanTruckPerc=80;
    m_changeLaneSet=false;
	m_flowDirPos=1;
	m_flowDirNeg=1;
	m_velocityDirPos=0;
	m_velocityDirNeg=0;
	m_currentLaneDirPos=0;
	m_currentLaneDirNeg=0;
	m_remainderDirPos=0;
	m_remainderDirNeg=0;
	m_penetrationRate=100;
	m_RVFlowDirPos = UniformVariable(m_flowDirPos*m_dt, m_flowDirPos*m_dt);
	m_RVFlowDirNeg = UniformVariable(m_flowDirNeg*m_dt, m_flowDirPos*m_dt);

    // Setup Wifi
    m_wifiHelper = WifiHelper::Default();	
    m_wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211a);
    m_wifiMacHelper = NqosWifiMacHelper::Default();
    m_wifiPhyHelper = YansWifiPhyHelper::Default();
    m_wifiChannelHelper = YansWifiChannelHelper::Default ();
    m_wifiMacHelper.SetType ("ns3::AdhocWifiMac");
    m_wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue ("wifia-6mbs"));
    //m_wifiChannelHelper.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
	m_wifiChannel = m_wifiChannelHelper.Create();
    m_wifiPhyHelper.SetChannel (m_wifiChannel);
    m_wifiPhyHelper.Set("TxPowerStart",DoubleValue(21.5));	// 250-300 meter transmission range 
    m_wifiPhyHelper.Set("TxPowerEnd",DoubleValue(21.5));      // 250-300 meter transmission range 			
    m_wifiPhyHelper.Set("TxPowerLevels",UintegerValue(1)); 
    m_wifiPhyHelper.Set("TxGain",DoubleValue(2)); 
    m_wifiPhyHelper.Set("RxGain",DoubleValue(2));  
    m_wifiPhyHelper.Set("EnergyDetectionThreshold", DoubleValue(-101.0));
  }

  Highway::~Highway()
  {
    m_laneChangeSedan=0;
    m_laneChangeTruck=0;
    m_sedan=0;
    m_truck=0;
    m_tempVehicles[0]=0; 
	m_tempVehicles[1]=0;

    for(int i=0;i<m_numberOfLanes;i++)
      {
        m_vehicles[i].erase(m_vehicles[i].begin(), m_vehicles[i].end());
      }
    if(m_twoDirectional==true)
      {
        for(int i=0;i<m_numberOfLanes;i++)
        {
          m_vehiclesOpp[i].erase(m_vehiclesOpp[i].begin(), m_vehiclesOpp[i].end());
        }
      }
  }
  void Highway::Step(Ptr<Highway> highway)
  {
    highway->TranslateVehicles();
  }
  void Highway::Start()
  {
    m_stopped=false;
    InitHighway();
    Simulator::Schedule(Seconds(0.0), &Step, Ptr<Highway>(this));
  }

  void Highway::Stop()
  {
    m_stopped=true;
  }
 
  double Highway::GetDeltaT()
  {
    return m_dt;
  }

  void Highway::SetDeltaT(double value)
  {
    if(value<=0) 
	  value=0.1;

    m_dt=value;
  }

  bool Highway::GetTwoDirectional()
  {
    return m_twoDirectional;
  }

  void Highway::SetTwoDirectional(bool value)
  {
    m_twoDirectional=value;
  }

  int Highway::GetNumberOfLanes()
  {
    return m_numberOfLanes;
  }

  void Highway::SetNumberOfLanes(int value)
  {
    if(value<1) 
	  value=1;
    else if(value>5) 
	  value=5;

    m_numberOfLanes=value;
  }

  double Highway::GetHighwayLength()
  {
    return m_highwayLength;
  }

  void Highway::SetHighwayLength(double value)
  {
    if(value<0) 
	  value=10000;

    m_highwayLength=value;
  }

  double Highway::GetLaneWidth()
  {
   return m_laneWidth;
  }

  void Highway::SetLaneWidth(double value)
  {
    if(value<0) 
	  value=5;

    m_laneWidth=value;
  }

  void Highway::SetPenetrationRate(double value)
  {
    if(value>100) m_penetrationRate=100;
	else if(value<0) m_penetrationRate=0;
	else m_penetrationRate=value;
  }

  double Highway::GetPenetrationRate()
  {
	  return m_penetrationRate;
  }

  void Highway::PrintVehicles()
  {
    std::cout << "Lane 2----------------" << Simulator::Now()<< "--------" << std::endl;
    for(uint i=0; i<m_vehicles[1].size();i++)
      {
        Ptr<Vehicle> v=GetVehicle(m_vehicles[1],i);
		std::cout<< v->GetVehicleId() << ":" << v->GetPosition().x 
                 << ":" << v->GetPosition().y << ":" << v->GetVelocity() << std::endl;
      }
    std::cout << "----------------------" << std::endl;
  }

  void Highway::SetFlowPositiveDirection(double value)
  {
    if(value>=0) m_flowDirPos = value;
  }
	 
  void Highway::SetFlowNegativeDirection(double value)
  {
    if(value>=0) m_flowDirNeg = value;
  }

  void Highway::SetFlowRVPositiveDirection(RandomVariable rv)
  {
	  m_RVFlowDirPos = rv;
  }

  void Highway::SetFlowRVNegativeDirection(RandomVariable rv)
  {
	  m_RVFlowDirNeg = rv;
  }

  void Highway::SetSpeedRV(RandomVariable rv)
  {
	  m_RVSpeed = rv;
  }
	  
  void Highway::SetVelocityPositiveDirection(double value)
  {
    if(value>=0) m_velocityDirPos = value;
  }
	  
  void Highway::SetVelocityNegativeDirection(double value)
  {
    if(value>=0) m_velocityDirNeg = value;
  }

  Ptr<Model> Highway::GetSedanModel()
  {
    return m_sedan;
  }

  void Highway::SetSedanModel(Ptr<Model> value)
  {
    m_sedan=value;
  }

  Ptr<Model> Highway::GetTruckModel()
  {
    return m_truck;
  }

  void Highway::SetTruckModel(Ptr<Model> value)
  {
    m_truck=value;
  }

  Ptr<LaneChange> Highway::GetSedanLaneChange()
  {
    return m_laneChangeSedan;
  }

  void Highway::SetSedanLaneChange(Ptr<LaneChange> value)
  {
    m_laneChangeSedan=value;
  }

  Ptr<LaneChange> Highway::GetTruckLaneChange()
  {
    return m_laneChangeTruck;
  }

  void Highway::SetTruckLaneChange(Ptr<LaneChange> value)
  {
    m_laneChangeTruck=value;
  }

  bool Highway::GetAutoInject()
  {
    return m_autoInject;
  }

  void Highway::SetAutoInject(bool value)
  {
    m_autoInject=value; 
  }

  int Highway::GetLastVehicleId()
  {
    return m_vehicleId;
  }

  WifiHelper Highway::GetWifiHelper()
  {
    return m_wifiHelper;
  }

  NqosWifiMacHelper Highway::GetNqosWifiMacHelper()
  {
    return m_wifiMacHelper;
  }

  YansWifiPhyHelper Highway::GetYansWifiPhyHelper()
  {
	return m_wifiPhyHelper;
  }

  void Highway::SetYansWifiPhyHelper(YansWifiPhyHelper yWifiPhyHelper)
  {
	m_wifiPhyHelper = yWifiPhyHelper;
  }
  	  
  Ptr<YansWifiChannel> Highway::GetWifiChannel()
  {
    return m_wifiChannel;
  }

  void Highway::AddVehicle(Ptr<Vehicle> vehicle)
  {
    int lane=vehicle->GetLane();
    int dir=vehicle->GetDirection();
    if(lane < m_numberOfLanes && lane >= 0)
      {
        if(dir==1) 
		  m_vehicles[lane].push_back(vehicle);
        else if(dir==-1) 
		  m_vehiclesOpp[lane].push_back(vehicle);
      }
  }

  Ptr<Vehicle> Highway::FindVehicle(int vid)
  {
    Ptr<Vehicle> v=0;

    for(int i=0;i<m_numberOfLanes;i++)
      {
        for(uint j=0;j<m_vehicles[i].size();j++)
          {	
            v=GetVehicle(m_vehicles[i],j);
            if(v->GetVehicleId()==vid) 
			  return v;
          }
      }
    if(m_twoDirectional==true)
      {
        for(int i=0;i<m_numberOfLanes;i++)
          {
            for(uint j=0;j<m_vehiclesOpp[i].size();j++)
              {	
                v=GetVehicle(m_vehiclesOpp[i],j);
                if(v->GetVehicleId()==vid) 
				  return v;
              }
          }
      }
    return v;
  }

  std::list<Ptr<Vehicle> > Highway::FindVehiclesInRange(Ptr<Vehicle> vehicle, double range)
  {
    std::list<Ptr<Vehicle> > neighbors;
    if(range<=0) 
	  return neighbors;

    Ptr<Vehicle> v=0;
    double diff=0;
    Vector pos, p;
    p=vehicle->GetPosition();
    for(int i=0;i<m_numberOfLanes;i++)
      {
        for(uint j=0;j<m_vehicles[i].size();j++)
          {	
            v=GetVehicle(m_vehicles[i],j);
            pos=v->GetPosition();
            if(v->GetVehicleId()==vehicle->GetVehicleId()) 
			  continue;
            
			diff= sqrt(pow(pos.x-p.x,2) + pow(pos.y-p.y,2));
            if(diff < range) neighbors.push_back(v);
          }
      }
    if(m_twoDirectional==true)
      {
        for(int i=0;i<m_numberOfLanes;i++)
        {
          for(uint j=0;j<m_vehiclesOpp[i].size();j++)
            {	
              v=GetVehicle(m_vehiclesOpp[i],j);
              pos=v->GetPosition();
              if(v->GetVehicleId()==vehicle->GetVehicleId()) 
			    continue;

              diff= sqrt(pow(pos.x-p.x,2) + pow(pos.y-p.y,2));
              if(diff < range) neighbors.push_back(v);
            }
        }
      }

  return neighbors;
  }

  std::list<Ptr<Vehicle> > Highway::FindVehiclesInSegment(double x1, double x2, int lane, int dir)
  {
    std::list<Ptr<Vehicle> > segment;
    Ptr<Vehicle> v=0;
    Vector pos;
    if(dir==1 && lane< 5 && lane>=0)
      {
        for(uint j=0;j<m_vehicles[lane].size();j++)
          {	
            v=GetVehicle(m_vehicles[lane],j);
            pos=v->GetPosition();
            if(pos.x >= x1 && pos.x < x2) segment.push_back(v);
          }
      }
    else if(dir==-1 && m_twoDirectional==true && lane < 5 && lane >= 0)
      {
        for(uint j=0;j<m_vehiclesOpp[lane].size();j++)
          { 	
            v=GetVehicle(m_vehiclesOpp[lane],j);
            pos=v->GetPosition();
            if(pos.x >= x1 && pos.x < x2) 
			  segment.push_back(v);
          }
      }

    return segment;		
  }
  Callback<void, Ptr<Vehicle>, Ptr<const Packet>, Address> Highway::GetReceiveDataCallback()
  {
    return m_receiveData;
  }

  void Highway::SetReceiveDataCallback(Callback<void, Ptr<Vehicle>, Ptr<const Packet>, Address> receiveData)
  {
    m_receiveData = receiveData;
  }

  DeviceTraceCallback Highway::GetDevTxTraceCallback()
  {
    return m_devTxTrace;
  }

  void Highway::SetDevTxTraceCallback(DeviceTraceCallback devTxTrace)
  {
    m_devTxTrace = devTxTrace;
  }

  DeviceTraceCallback Highway::GetDevRxTraceCallback()
  {
    return m_devRxTrace;
  }
  
  void Highway::SetDevRxTraceCallback(DeviceTraceCallback devRxTrace)
  {
    m_devRxTrace = devRxTrace;
  }

  PhyRxOkTraceCallback Highway::GetPhyRxOkTraceCallback()
  {
    return m_phyRxOkTrace;
  }

  void Highway::SetPhyRxOkTraceCallback(PhyRxOkTraceCallback phyRxOkTrace)
  {
    m_phyRxOkTrace = phyRxOkTrace;
  }

  PhyRxErrorTraceCallback Highway::GetPhyRxErrorTraceCallback()
  {
    return m_phyRxErrorTrace; 
  }

  void Highway::SetPhyRxErrorTraceCallback(PhyRxErrorTraceCallback phyRxErrorTrace)
  {
    m_phyRxErrorTrace = phyRxErrorTrace;
  }

  PhyTxTraceCallback Highway::GetPhyTxTraceCallback()
  {
    return m_phyTxTrace;
  }

  void Highway::SetPhyTxTraceCallback(PhyTxTraceCallback phyTxTrace)
  {
    m_phyTxTrace = phyTxTrace;
  }

  PhyStateTraceCallback Highway::GetPhyStateTraceCallback()
  {
    return m_phyStateTrace;
  }

  void Highway::SetPhyStateTraceCallback(PhyStateTraceCallback phyStateTrace)
  {
    m_phyStateTrace = phyStateTrace;
  }

  Callback<bool, Ptr<Highway> ,Ptr<Vehicle> , double> Highway::GetControlVehicleCallback()
  {
    return m_controlVehicle;
  }

  void Highway::SetControlVehicleCallback(Callback<bool, Ptr<Highway> ,Ptr<Vehicle> , double> controlVehicle)
  {
    m_controlVehicle = controlVehicle;
  }

  Callback<bool, Ptr<Highway>, int&> Highway::GetInitVehicleCallback()
  {
    return m_initVehicle;
  }

  void Highway::SetInitVehicleCallback(Callback<bool, Ptr<Highway>, int&> initVehicle)
  {
    m_initVehicle = initVehicle;
  }
}
