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

#include "Obstacle.h"

namespace ns3
{
  TypeId Obstacle::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::Obstacle")
      .SetParent<Vehicle> ()
      .AddConstructor<Obstacle> ()
      ;
    return tid;
  }

  Obstacle::Obstacle()
  {
    SetAcceleration(0.0);
    SetVelocity(0.0);
    SetModel(0);
    SetLaneChange(0);
  }
	
  void Obstacle::Accelerate(Ptr<Vehicle> vwd)
  {
    SetAcceleration(0.0);
  }

  bool Obstacle::CheckLaneChange(Ptr<Vehicle> frontOld, Ptr<Vehicle> frontNew, Ptr<Vehicle> backNew, bool toLeft)
  {
    return false;
  }

  void Obstacle::TranslatePosition(double dt)
  {
    ;
  }

  void Obstacle::TranslateVelocity(double dt)
  {
    SetVelocity(0.0);
  }

  double Obstacle::Acceleration(Ptr<Vehicle> vwd)
  {
    return 0.0;
  }
}
