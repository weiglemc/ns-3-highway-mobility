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

/*
	This the starting point of the simulation and experiments.
	The main function will parse the input and parameter settings.
	Creates a highway and set the highway parameters. then bind the events (callbacks)
	to the created controller and designed handlers. Sets the highway start and end time,
	and eventually runs the simulation which is basically running a highway with a controller.
	You can add your functions to controller to create various scenarios. 
*/

#include <fstream>
#include <iostream>
#include <iomanip>
#include "ns3/core-module.h"
#include "ns3/common-module.h"
#include "ns3/node-module.h"
#include "ns3/helper-module.h"
#include "ns3/mobility-module.h"
#include "ns3/contrib-module.h"
#include "ns3/wifi-module.h"
#include "ns3/random-variable.h"
#include "math.h"
#include "Highway.h"
#include "Controller.h"

NS_LOG_COMPONENT_DEFINE ("HADI");

using namespace ns3;
using namespace std;

static void Start(Ptr<Highway> highway)
{
  highway->Start();
}

static void Stop(Ptr<Highway> highway)
{
  highway->Stop();
}

int main (int argc, char *argv[])
{ 
  ns3::PacketMetadata::Enable();
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
	
  float simTime=1.0;
  bool plot=false;
  bool twoDirectional=false;
  bool laneChange=true;
  double flow1=0.5, flow2=0.5;
  double vel1=29, vel2=29;
  double pRate=100;
  double mix=80;
  double gap=5;
  double speedLimit=29;
  double speedStd=1;
  int numberOfLanes=3;
  int runNumber=1;
  string directory="./";
  string fp="";
  int distribution=0;
  double std1=0.0, std2=0.0;
  double maxFlow=5.0;
  double transmissionPower=21.5;

  // Process command-line args
  CommandLine cmd;
  cmd.AddValue ("time", "simulation time", simTime);
  cmd.AddValue ("plot", "generate output fot gnuplot", plot);  
  cmd.AddValue ("dir", "one or two directional", twoDirectional);
  cmd.AddValue ("dis", " 0 = Uniform, 1 = Expoential, 2 = Normal, 3 = Log Normal, default = 0", distribution);
  cmd.AddValue ("flow1", "traffic flow mean at entrance", flow1);
  cmd.AddValue ("std1", "traffic flow std at entrance", std1);
  cmd.AddValue ("vel1", "traffic velocity mean at entrance", vel1);
  cmd.AddValue ("flow2", "traffic flow mean at entrance (other direction)", flow2);
  cmd.AddValue ("std2", "traffic flow std at entrance", std2);
  cmd.AddValue ("vel2", "traffic velocity mean at entrance (other direction)", vel2);
  cmd.AddValue ("maxflow", "traffic maximum flow/lane at entrance (both directions)", maxFlow);
  cmd.AddValue ("prate","penetration rate of equipped vehicles", pRate);
  cmd.AddValue ("mix", "car to truck injection mix percentage", mix);
  cmd.AddValue ("gap", "injection gap at entrance", gap);
  cmd.AddValue ("lane", "number of lanes", numberOfLanes);
  cmd.AddValue ("spl", "speed limit", speedLimit);
  cmd.AddValue ("spstd", "speed std", speedStd);
  cmd.AddValue ("lc", "lane change", laneChange); 
  cmd.AddValue ("rn", "run number", runNumber);
  cmd.AddValue ("fp", "prefix for filenames", fp);
  cmd.AddValue ("pw", "transmission power", transmissionPower);

  cmd.Parse(argc, argv);

  // Validate flow rate and speed 
  if(flow1<0) flow1=0;
  else if(flow1>maxFlow*numberOfLanes) flow1=maxFlow*numberOfLanes;
  if(flow2<0) flow2=0;
  else if(flow2>maxFlow*numberOfLanes) flow2=maxFlow*numberOfLanes;
  if(vel1<5) vel1=5;
  else if(vel1>33) vel1=33;
  if(vel2<5) vel2=5;
  else if(vel2>33) vel2=33;
  
  if(gap<2) gap=2;

  directory+=fp;
  fp=directory;

  double deltaT=0.1;
  RandomVariable RV1, RV2, RVSpeed;

  // Setup Speed (Normal(mean, variance, bound))
  int bound=5;
  RVSpeed = NormalVariable(speedLimit, speedStd*speedStd, bound);

  // Setup Flow Rate Distribution
  if(distribution == 1)
  {
	  RV1 = ExponentialVariable(flow1*deltaT, maxFlow*deltaT);
	  RV2 = ExponentialVariable(flow2*deltaT, maxFlow*deltaT);
  }
  else if(distribution == 2)
  {
	  RV1 = NormalVariable(flow1*deltaT, (std1*deltaT)*(std1*deltaT));
	  RV2 = NormalVariable(flow2*deltaT, (std2*deltaT)*(std2*deltaT));
  }
  else if(distribution == 3)
  {
	  double mu1 = log (flow1 * deltaT) - 0.5 * log (1+ std1/(flow1*flow1*deltaT));
	  double sig1= sqrt(log (1+ std1/(flow1*flow1*deltaT)));
	  RV1 = LogNormalVariable(mu1, sig1);
	  	  
	  double mu2 = log (flow2 * deltaT) - 0.5 * log (1+ std2/(flow2*flow2*deltaT));
	  double sig2= sqrt(log (1+ std2/(flow2*flow2*deltaT)));
	  RV2 = LogNormalVariable(mu2, sig2);
  }
  else
  {  
	  // including distribution == 0
	  RV1 = UniformVariable(flow1*deltaT, (flow1+std1)*deltaT);
	  RV2 = UniformVariable(flow2*deltaT, (flow2+std2)*deltaT);
  }

  Ptr<Highway> highway=CreateObject<Highway>();
  Ptr<Controller> controller=CreateObject<Controller>();

  // Bind an experiment (controller) to highway
  controller->SetHighway(highway);
  controller->Plot=plot;

  // Setup parameters for highway
  highway->SetHighwayLength(10000);
  highway->SetLaneWidth(5);
  highway->SetNumberOfLanes(numberOfLanes);
  highway->SetChangeLane(laneChange);
  highway->SetTwoDirectional(twoDirectional);
  highway->SetMedianGap(5);
  highway->SetInjectionGap(gap);
  highway->SetInjectionMixValue(mix);
  highway->SetAutoInject(true);
  highway->SetSpeedRV(RVSpeed);
  highway->SetFlowPositiveDirection(flow1);
  highway->SetVelocityPositiveDirection(vel1);
  highway->SetFlowNegativeDirection(flow2);
  highway->SetVelocityNegativeDirection(vel2);
  highway->SetFlowRVPositiveDirection(RV1);
  highway->SetFlowRVNegativeDirection(RV2);
  highway->SetPenetrationRate(pRate);
  highway->SetDeltaT(deltaT);
  
  // Update the transmission range of wifi shared in the Highway.
  YansWifiPhyHelper tempHelper = highway->GetYansWifiPhyHelper();
  tempHelper.Set("TxPowerStart",DoubleValue(transmissionPower)); // 250-300 meter transmission range 
  tempHelper.Set("TxPowerEnd",DoubleValue(transmissionPower));   // 250-300 meter transmission range 
  highway->SetYansWifiPhyHelper(tempHelper);
  
  // Bind the Highway/Vehicle events to the event handlers. Controller's will catch them.  
  highway->SetControlVehicleCallback(MakeCallback(&Controller::ControlVehicle,controller));
  highway->SetInitVehicleCallback(MakeCallback(&Controller::InitVehicle,controller));
  highway->SetReceiveDataCallback(MakeCallback(&Controller::ReceiveData,controller));
  
  // Setup seed and run-number (to affect random variable outcome of different runs)
  if(runNumber < 1) runNumber=1;
  SeedManager::SetSeed(1);
  SeedManager::SetRun(runNumber);

  // Output summary of parameters' value
  ofstream simPar;
  string name=fp;
  fp+="-Parameters";
  simPar.open(fp.c_str());
  simPar << "time = " << simTime << " run number = " << runNumber <<endl 
  << "number of lanes = " << numberOfLanes << " two directional = " << twoDirectional << " lane change = " << laneChange<< endl
  << "flow distribution = " << distribution << endl
  << "flow1 = " << flow1 << " vel1 = " << vel1 << endl
  << "flow2 = " << flow1 << " vel2 = " << vel1 << endl
  << "transmission power = " << transmissionPower << endl
  << "injection gap = " << gap << " injection mix = " << mix << " penetration rate = " << pRate << endl
  << "speed limit = " << speedLimit << " speed std = " << speedStd << endl
  << "file prefix = " << name << endl;
  simPar.close();
  
  // Schedule and run highway
  Simulator::Schedule(Seconds(0.0), &Start, highway);
  Simulator::Schedule(Seconds(simTime), &Stop, highway);
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();

  return 0;
}
