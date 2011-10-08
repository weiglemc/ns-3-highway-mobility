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

#include "Model.h"
#include <math.h>

namespace ns3 {

    TypeId Model::GetTypeId(void) {
        static TypeId tid = TypeId("ns3::Model")
                .SetParent<Object > ()
                ;
        return tid;
    }

    /**
     * \param bwd the Vehicle in back. (current considered Vehicle)
     * \param vwd the Vechile in front.
     * \returns the calculated acceleration for the considered Vehicle bwd based on the front Vehicle vwd and IDM rules.
     *
     * see: http://www.vwi.tu-dresden.de/~treiber/MicroApplet/IDM.html .
     */
    double Model::CalculateAcceleration(Ptr<Vehicle> bwd, Ptr<Vehicle> vwd, double distance) {
        //If the forward vehicle is actually there
        if (vwd != 0/*null*/) {
            //The difference in velocity between the current vehicle and the forward vehicle
            double delta_v = bwd->GetVelocity() - vwd->GetVelocity();
            //Teh distance is pre-calculated
            double s = distance; //pos: in the back of vehicles
            //The velocity of the vehicle in the back
            double vel = bwd->GetVelocity();
            //s_star_raw is a distance calculated based on velcoity and available acceleration/decelleration
            double s_star_raw = m_minimumGap + vel * m_timeHeadway + (vel * delta_v) / (2 * m_sqrtAccDec);
            //s_star has to be at least the minimum gap required
            double s_star = (s_star_raw > m_minimumGap) ? s_star_raw : m_minimumGap;
            double acc = m_acceleration * (1 - pow(vel / m_desiredVelocity, m_deltaV) - (s_star * s_star) / (s * s));
            return acc;
        } else {
            //If there is no forward vehicle
            //A delta v is calculated to keep the acceleration to a reasonable level
            //If we did not have a delta v, it would make the vehicle accelerate at its
            //maximum if there were no forward vehicles
            double delta_v = bwd->GetVelocity() - 25.0;
            //Set s to 500 meters
            double s = 500;
            //Get the velocity of the back vehicle
            double vel = bwd->GetVelocity();
            //s_star_raw is calculated based on the assumed velocities above
            double s_star_raw = m_minimumGap + vel * m_timeHeadway + (vel * delta_v) / (2 * m_sqrtAccDec);
            //s_star has to be at least the minimum gap
            double s_star = (s_star_raw > m_minimumGap) ? s_star_raw : m_minimumGap;
            //Calculate the acceleration
            double acc = m_acceleration * (1 - pow(vel / m_desiredVelocity, m_deltaV) - (s_star * s_star) / (s * s));
            return acc;
        }
    }

    void Model::SetDesiredVelocity(double desiredVelocity) {
        m_desiredVelocity = desiredVelocity;
    }

    double Model::GetDesiredVelocity() {
        return m_desiredVelocity;
    }

    void Model::SetDeltaV(double deltaV) {
        m_deltaV = deltaV;
    }

    double Model::GetDeltaV() {
        return m_deltaV;
    }

    void Model::SetAcceleration(double acceleration) {
        m_acceleration = acceleration;
    }

    double Model::GetAcceleration() {
        return m_acceleration;
    }

    void Model::SetDeceleration(double deceleration) {
        m_deceleration = deceleration;
    }

    double Model::GetDeceleration() {
        return m_deceleration;
    }

    void Model::SetMinimumGap(double minimumGap) {
        m_minimumGap = minimumGap;
    }

    double Model::GetMinimumGap() {
        return m_minimumGap;
    }

    void Model::SetTimeHeadway(double timeHeadway) {
        m_timeHeadway = timeHeadway;
    }

    double Model::GetTimeHeadway() {
        return m_timeHeadway;
    }

    void Model::SetSqrtAccelerationDeceleration(double sqrtAccDec) {
        m_sqrtAccDec = sqrtAccDec;
    }

    double Model::GetSqrtAccelerationDeceleration() {
        return m_sqrtAccDec;
    }
}
