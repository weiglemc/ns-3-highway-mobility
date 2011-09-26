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

#ifndef CLASS_CONTROLLER_
#define CLASS_CONTROLLER_

#include "Highway.h"
#include "Obstacle.h"

using namespace ns3;
using namespace std;

namespace ns3
{

  /**
  * \brief Controller is the main class to manage the events (callbacks), traces, rules, and etc.
  *
  * Controller can be assumed as an application which is tied with the highway and vehicles.
  * we implement the VANETs simulations here, design and form the basic of each experiments. 
  */
  class Controller : public Object
  {
    private:
      double T;
      Ptr<Highway> highway;
    public:
      /// Constructor.
      Controller();
      /// Constructor.
      Controller(Ptr<Highway> highway);
      /// to broadcast a warning by vehicle veh. 
      void BroadcastWarning(Ptr<Vehicle> veh);
      /// event handler for ReceiveData callbacks.
      void ReceiveData (Ptr<Vehicle> veh, Ptr<const Packet> packet, Address address);
      /// event handler for ControlVehicle callbacks.
      bool ControlVehicle(Ptr<Highway> highway, Ptr<Vehicle> vehicle, double dt);
      /// event handler for InitVehicle callbacks.
      bool InitVehicle(Ptr<Highway> highway,  int& VID);
      /// sets the highway bound to this controller.
      void SetHighway(Ptr<Highway> highway);
      /// returns the highway bound to this controller.
      Ptr<Highway> GetHighway();
      /// a flag for plotting vehicles.
      bool Plot;
  };
}
#endif
