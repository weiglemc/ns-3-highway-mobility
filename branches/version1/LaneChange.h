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

#ifndef CLASS_LANECHANGE_
#define CLASS_LANECHANGE_

#include "ns3/ptr.h"
#include "ns3/object.h"
#include "Vehicle.h"

namespace ns3
{
  class Vehicle;
  /**
  * \brief IDM/MOBIL LaneChange Model.
  *
  * A Vehicle mobiliy and its acceleration in a particular lane of the roadway can be determined by IDM Car Following Model.
  * Sometimes a Vehicle can have opportunity to switch its current lane to a desird adjacent lane.
  * LaneChange Model can check the possibility of changing lane for a particular Vehicle based on several factors such as:
  * Driver Politeness Factor, Minimum Safe Distance and Breaking Deceleration to the Vehicle(s).
  * in front and back (the same lane) and in the desired (target) lane.
  * see: http://www.vwi.tu-dresden.de/~treiber/MicroApplet/IDM.html
  * see: http://www.vwi.tu-dresden.de/~treiber/MicroApplet/MOBIL.html
  */
  class LaneChange : public ns3::Object
  {
    private: 
  
      double m_politenessFactor;                    // politeness factor.
      double m_dbThreshold;                         // changing threshold.
      double m_gapMin;                              // minimum safe (net) distance.
      double m_maxSafeBreakingDeceleration;         // maximum safe braking deceleration.
      double m_biasRight;                           // bias to right/left.
        
    public: 

      /// Override TypeId
      static TypeId GetTypeId (void);
      /// Constructor.
      LaneChange();
      /**
      * \returns the Politness Factor. 
      */
      double GetPolitenessFactor();
      /**
      * \param value a Politeness Factor.
      */
      void SetPolitenessFactor(double value);
      /**
      * \returns the Db Threshold for changing lanes.
      */
      double GetDbThreshold();
      /**
      * \param value the Db Threshold for changing lanes.
      */
      void SetDbThreshold(double value);
      /**
      * \returns the Gap Minimum which is equivalent to Minimum Safe Distance.
      */
      double GetGapMin();
      /**
      * \param value a Gap Minimum which is equivalent to Minimum Safe Distance.
      */
      void SetGapMin(double value);
      /**
      * \returns the Maximum Safe Breaking Deceleration.
      */
      double GetMaxSafeBreakingDeceleration();
      /**
      * \param value the Maximum Safe Breaking Distance.
      */
      void SetMaxSafeBreakingDeceleration(double value);
      /**
      * \returns the Bias to Right.	
      */
      double GetBiasRight();
      /**
      * \param value the Bias to Right.
      */
      void SetBiasRight(double value);
      /**
      * \param me the current considered Vehicle.
      * \param fOld the front Vehicle in the current lane.
      * \param fNew the front Vehicle in the target lane (left or right) based on value of toLeft.
      * \param bNew the back  Vehicle in the target lane (left or right) based on value of toLeft.
      * \param toLeft true if the adjacent target lane is on the left, false if the adjacent target lane is on the right.
      * \returns true if changing lane for the Vehicle (me) to the target lane is possible, false otherwise. 
	  *
      * see: http://www.vwi.tu-dresden.de/~treiber/MicroApplet/MOBIL.html
      */
      bool CheckLaneChange(Ptr<Vehicle> me, Ptr<Vehicle> fOld, Ptr<Vehicle> fNew, Ptr<Vehicle> bNew, bool toLeft);    
  };
};
#endif
