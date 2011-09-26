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

    bool LaneChange::CheckLaneChange(Ptr<Vehicle> me, Ptr<Vehicle> fOld, double distanceFOld, Ptr<Vehicle> fNew, double distanceFNew, Ptr<Vehicle> bNew, double distanceBNew, bool toLeft) {

        Vector fNewPosition, bNewPosition;
        Vector oldMePosition = me->GetPosition();
        Vector mePosition = me->GetPosition();
        double oldAcceleration = me->Acceleration(fOld, distanceFOld);
        double intersection[3];
        double bNew_acc;
        double bNewLength;
        double others_disadv = 0;
        bool havePosition = false;
        bool result = false;



        if (fNew == 0/*null*/) {
            fNewPosition.x = 100000000000.0;
            fNewPosition.y = 100000000000.0;
        } else {
            fNewPosition = fNew->GetPosition();
            findIntersection(me->GetDirection(), me->GetPosition().x, me->GetPosition().y,
                    fNew->GetDirection(), fNew->GetPosition().x, fNew->GetPosition().y,
                    intersection);
            if (intersection[0] != -1) {
                havePosition = true;
                mePosition.x = intersection[1];
                mePosition.y = intersection[2];
                me->SetPosition(mePosition);
            }
        }

        if (bNew == 0/*null*/) {
            bNewPosition.x = -100000000000.0;
            bNewPosition.y = -100000000000.0;
            bNewLength = me->GetLength();
            bNew_acc = -m_maxSafeBreakingDeceleration + 1;
        } else {
            bNewPosition = bNew->GetPosition();
            bNewLength = bNew->GetLength();
            if (!havePosition) {
                findIntersection(me->GetDirection(), me->GetPosition().x, me->GetPosition().y,
                        bNew->GetDirection(), bNew->GetPosition().x, bNew->GetPosition().y,
                        intersection);
                if (intersection[0] != -1) {
                    mePosition.x = intersection[1];
                    mePosition.y = intersection[2];
                    me->SetPosition(mePosition);
                }
            }
            bNew_acc = bNew->Acceleration(me, distanceBNew);
        }

        if (fNew != 0/*null*/ && bNew != 0 /*null*/) {
            others_disadv = bNew->Acceleration(fNew, distanceFNew) - bNew_acc;
        }

        double gapFront = CalculateDistance(fNewPosition, mePosition) - me->GetLength();
        double gapBack = CalculateDistance(bNewPosition, mePosition) - bNewLength;
        if ((gapFront > m_gapMin) && (gapBack > m_gapMin) && (bNew_acc > -m_maxSafeBreakingDeceleration)) {
            double my_adv = me->Acceleration(fNew, distanceFNew) - oldAcceleration + ((toLeft) ? -1 : 1) * m_biasRight;
            if (others_disadv < 0) {
                others_disadv = 0;
            }
            if (my_adv - m_politenessFactor * others_disadv > m_dbThreshold) {
                result = true;
            } else {
                result = false;
            }
        } else {
            result = false;
        }
        me->SetPosition(oldMePosition);
        return result;
    }
}
