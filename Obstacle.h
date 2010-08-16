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

#ifndef CLASS_OBSTACLE_
#define CLASS_OBSTACLE_

#include "ns3/ptr.h"
#include "Vehicle.h"

namespace ns3
{
  /**
  * \brief Obstacle is a static Vehicle with no mobility (its Velocity = Acceleration = Model = LaneChange = 0).  
  *
  * An Obstacle can be used as an barrier, a road obstacle, a VANET RSU, or a station along/side the roadway [Highway].
  * An Obstacle has all the capabilities of a Vehicle except it can not be mobile.
  */
  class Obstacle: public ns3::Vehicle
  {
    public:

      /// Override TypeId.
      static TypeId GetTypeId (void);
      /// Constructor to initialized velocity, acceleration, model, lanechange to zero.
      Obstacle();
      /// Never accelerates. Set its acceleration always to 0.
      virtual void Accelerate(Ptr<Vehicle> vwd);
      /// Never changelanes. Always returns false. 
      virtual bool CheckLaneChange(Ptr<Vehicle> frontOld, Ptr<Vehicle> frontNew, Ptr<Vehicle> backNew, bool toLeft);
      /// Never moves. This function must do nothing.
      virtual void TranslatePosition(double dt);
      /// Never speeds. Set its velocity always to 0.
      virtual void TranslateVelocity(double dt);
      /// Never accelerates and returns 0. Since the returned computed acceleration must be always 0 for a static object.
      virtual double Acceleration(Ptr<Vehicle> vwd);
  };
};
#endif
