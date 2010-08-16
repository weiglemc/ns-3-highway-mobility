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

#ifndef CLASS_MODEL_
#define CLASS_MODEL_

#include "ns3/ptr.h"
#include "ns3/object.h"
#include "Vehicle.h"

namespace ns3
{
  class Vehicle;

  /**
  * \brief Intelligent Driver Model (IDM) Car Following Model.
  *
  * In IDM Model, the amount of Acceleration for the desired Vehicle is calculated based on the Vehicle in front of it.
  * This Acceleration will be function of several factors such as: Desired Velocity, DeltaV, Gap from the front Vehicle, and ... 
  * see: http://www.vwi.tu-dresden.de/~treiber/MicroApplet/IDM.html .
  * see: http://www.vwi.tu-dresden.de/~treiber/MicroApplet/MOBIL.html .
  */
  class Model: public ns3::Object
  {
    private:  

      /// desired velocity.
      double m_desiredVelocity;	
	  /// deltaV (an acceleration exponent). 
      double m_deltaV;	
      /// acceleration.
      double m_acceleration;		
      /// deceleration.
      double m_deceleration;		
      /// minimum gap.
      double m_minimumGap;		
      /// time headway.
      double m_timeHeadway;
      /// square root of (acceleration) multiply (deceleration).
      double m_sqrtAccDec;

    public:

      /// Override TypeId.
      static TypeId GetTypeId (void);
      /**
      * \param bwd the Vehicle in back. (current considered Vehicle)
      * \param vwd the Vechile in front.
      * \returns the calculated acceleration for the considered Vehicle bwd based on the front Vehicle vwd and IDM rules.
	  *
	  * see: http://www.vwi.tu-dresden.de/~treiber/MicroApplet/IDM.html . 
      */
      double CalculateAcceleration(Ptr<Vehicle> bwd, Ptr<Vehicle> vwd);
      /// Set Desired Velocity.
      void SetDesiredVelocity(double desiredVelocity);
      /// Get Desired Velocity.
      double GetDesiredVelocity();
	  /// Set DeltaV (an acceleration exponent). 
      void SetDeltaV(double deltaV);
	  /// Get DeltaV (an acceleration exponent). 
      double GetDeltaV();
      /// Set Acceleration.
      void SetAcceleration(double acceleration);
      /// Get Acceleration.
      double GetAcceleration();
      /// Set Deceleration.
      void SetDeceleration(double deceleration);
      /// Get Deceleration.
      double GetDeceleration();
      /// Set Minimum Gap.
      void SetMinimumGap(double minimumGap);
      /// Get Minimum Gap.
      double GetMinimumGap();
      /// Get Time Headway.
      void SetTimeHeadway(double timeHeadway);
      /// Get Time Headway.
      double GetTimeHeadway();
      /// Set Square Root of (Acceleration) multiply (Deceleration).
      void SetSqrtAccelerationDeceleration(double sqrtAccDec);
      /// Get Square Root of (Acceleration) multiply (Deceleration).
      double GetSqrtAccelerationDeceleration();
  };
};
#endif
