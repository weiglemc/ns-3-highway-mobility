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

#include "LaneChange.h"
#include "Geometry.h"

namespace ns3 {

    TypeId LaneChange::GetTypeId(void) {
        static TypeId tid = TypeId("ns3::LaneChange")
                .SetParent<Object > ()
                .AddConstructor<LaneChange > ()
                ;
        return tid;
    }

    LaneChange::LaneChange() {
        ;
    }

    double LaneChange::GetPolitenessFactor() {
        return m_politenessFactor;
    }

    void LaneChange::SetPolitenessFactor(double value) {
        m_politenessFactor = value;
    }

    double LaneChange::GetDbThreshold() {
        return m_dbThreshold;
    }

    void LaneChange::SetDbThreshold(double value) {
        m_dbThreshold = value;
    }

    double LaneChange::GetGapMin() {
        return m_gapMin;
    }

    void LaneChange::SetGapMin(double value) {
        m_gapMin = value;
    }

    double LaneChange::GetMaxSafeBreakingDeceleration() {
        return m_maxSafeBreakingDeceleration;
    }

    void LaneChange::SetMaxSafeBreakingDeceleration(double value) {
        m_maxSafeBreakingDeceleration = value;
    }

    double LaneChange::GetBiasRight() {
        return m_biasRight;
    }

    void LaneChange::SetBiasRight(double value) {
        m_biasRight = value;
    }

    /**
     * \param me the current considered Vehicle.
     * \param fOld the front Vehicle in the current lane.
     * \param distanceFOld the distance from the current vehicle to fOld
     * \param fNew the front Vehicle in the target lane (left or right) based on value of toLeft.
     * \param distanceFNew the distance from where the current vehicle would be in the target lane to fNew
     * \param bNew the back  Vehicle in the target lane (left or right) based on value of toLeft.
     * \param distanceBNew the distance where where the current vehicle would be in the taget lane to bNew
     * \param toLeft true if the adjacent target lane is on the left, false if the adjacent target lane is on the right.
     * \returns true if changing lane for the Vehicle (me) to the target lane is possible, false otherwise.
     *
     * see: http://www.vwi.tu-dresden.de/~treiber/MicroApplet/MOBIL.html
     */
    bool LaneChange::CheckLaneChange(Ptr<Vehicle> me, Ptr<Vehicle> fOld, double distanceFOld, Ptr<Vehicle> fNew, double distanceFNew, Ptr<Vehicle> bNew, double distanceBNew, bool toLeft) {
        //Holders for various position
        Vector fNewPosition, bNewPosition;
        Vector oldMePosition = me->GetPosition();
        Vector mePosition = me->GetPosition();
        //Get the acceleation based on the current front vehicle
        double oldAcceleration = me->Acceleration(fOld, distanceFOld);
        //The instersection point of where the vehicle will be in the new lane
        double intersection[3];
        //The acceleration the vehicle in the back will experience if this vehicle changes lane
        double bNew_acc;
        //The length oft he vehicle in the back
        double bNewLength;
        //The disadvantage the back vehicle will experience
        double others_disadv = 0;
        //This is for checking that the intersection point can be calculated
        bool havePosition = false;
        //The result of whether we can change lanes
        bool result = false;



        //If the forward vehicle is non-existant, set the position of the
        //front vehicle to very very far ahead
        if (fNew == 0/*null*/) {
            fNewPosition.x = 100000000000.0;
            fNewPosition.y = 100000000000.0;
        } else {
            //Get the position
            fNewPosition = fNew->GetPosition();
            //Get the intersection point so that we can get the point in the
            //next lane where this vehicle will be if it moves
            findIntersection(me->GetDirection(), me->GetPosition().x, me->GetPosition().y,
                    fNew->GetDirection(), fNew->GetPosition().x, fNew->GetPosition().y,
                    intersection);
            //IF there is no error getting the intersection
            if (intersection[0] != -1) {
                havePosition = true;
                //Set the position of the vehicle to the point in the
                //adjacent lane
                mePosition.x = intersection[1];
                mePosition.y = intersection[2];
                me->SetPosition(mePosition);
            }
        }

        //If the back vehicle does not exists or doesn't have a lane model
        if (bNew == 0/*null*/ || bNew->GetLaneChange() == NULL) {
            //Set the position very far away so that lane change will happen
            bNewPosition.x = -100000000000.0;
            bNewPosition.y = -100000000000.0;
            bNewLength = me->GetLength();
            //Set the acceleration so that lane change will happen
            bNew_acc = -m_maxSafeBreakingDeceleration + 1;
        } else {
            //Get the position of the vehicle
            bNewPosition = bNew->GetPosition();
            bNewLength = bNew->GetLength();
            //If we don't have the position
            if (!havePosition) {
                //Get it based on the back vehicle
                findIntersection(me->GetDirection(), me->GetPosition().x, me->GetPosition().y,
                        bNew->GetDirection(), bNew->GetPosition().x, bNew->GetPosition().y,
                        intersection);
                if (intersection[0] != -1) {
                    mePosition.x = intersection[1];
                    mePosition.y = intersection[2];
                    me->SetPosition(mePosition);
                }
            }
            //Get the acceleration the back vehicle will experience if this vehicle
            //changes lane
            bNew_acc = bNew->Acceleration(me, distanceBNew);
        }

        //If we have vehicles to check against
        if (fNew != 0/*null*/ && bNew != 0 /*null*/ && bNew->GetLaneChange() != NULL) {
            //Determine the disadvantaged caused to the back vehicle
            others_disadv = bNew->Acceleration(fNew, distanceFNew) - bNew_acc;
        }

        //Make sure there is enough distance for this vehicle to fit in
        double gapFront = CalculateDistance(fNewPosition, mePosition) - me->GetLength();
        double gapBack = CalculateDistance(bNewPosition, mePosition) - bNewLength;
        //If there is enough of a gap and the vehicle in the back will be able to break safely
        if ((gapFront > m_gapMin) && (gapBack > m_gapMin) && (bNew_acc > -m_maxSafeBreakingDeceleration)) {
            //Check the advantage to this vehicle by figuring out if we can accelerate
            //faster in the other lane and adding in the bias to the right
            double my_adv = me->Acceleration(fNew, distanceFNew) - oldAcceleration + ((toLeft) ? -1 : 1) * m_biasRight;
            //Clean up the others disadvantage
            if (others_disadv < 0) {
                others_disadv = 0;
            }
            //Adjust the advantage by a politeness factor
            if (my_adv - m_politenessFactor * others_disadv > m_dbThreshold) {
                result = true;
            } else {
                result = false;
            }
        } else {
            result = false;
        }
        //Set the position back to the original so that no adjustments are
        //needed
        me->SetPosition(oldMePosition);
        return result;
    }
}
