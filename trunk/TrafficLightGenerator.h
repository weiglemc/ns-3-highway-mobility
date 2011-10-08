/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005-2011 Old Dominion University [ARBABI]
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
 *         Bradley Dupont <bradley.dupont@cs.odu.edu>
 */

#ifndef _TRAFFICLIGHTGENERATOR_H
#define	_TRAFFICLIGHTGENERATOR_H

#include "ns3/ptr.h"
#include "ns3/object.h"
#include "Highway.h"
#include <list>
#include <map>

namespace ns3 {

    /**
     * The TrafficLightGenerator manages traffic lights on a four way intersection
     * It does this by creating obstacles and putting them into the lane to
     * simulate a red light and then removing it when the light turns green
     */
    class TrafficLightGenerator : public ns3::Object {
    public:

        //Describes what side of the Intersection
        //we are talking about
        enum Side {
            LEFT,
            RIGHT,
            TOP,
            BOTTOM
        };

        // The current state the traffic signal is in
        enum LightState {
            LEFTTURN_LR,
            WAIT_LR,
            STRAIGHT_LR,
            WAITALL_LR,
            LEFTTURN_TB,
            WAIT_TB,
            STRAIGHT_TB,
            WAITALL_TB
        };

        // A utility struct to keep data together
        // This associates a highway with the necessary info
        struct TrafficPoint {
            // The associated highway
            Ptr<Highway> highway;
            // The distance into the highway to place the lights
            double distance;
            // The list of lanes that are left turn lanes
            std::list<int> leftTurnLanes;
            // A map for lane number to Obstacle for the left lanes
            std::map<int, Ptr<Vehicle> > leftTurnLights;
            // A map for lane number to Obstacle for the straight lanes
            std::map<int, Ptr<Vehicle> > straightLights;
        };

    private:

        // This map keeps all of the TrafficPoints for the generator
        std::map<Side, TrafficPoint* > m_highwayMap;

        // This keeps track of the current state of the signal
        LightState m_currentState;
        // This boolean says the light has a Highway for left/right
        bool m_hasLR;
        // This boolean says the light has a Highwya for top/bottom
        bool m_hasTB;
        // The time the Highways are green light for straight
        double m_timeStraight;
        // The time the Highways are green light for left
        double m_timeLeft;
        // The time between switching states when all lights are red
        double m_timeBuffer;

        // A utility method for changing the lights in the highway
        void ChangeLights(Side side, bool leftTurnLane, bool toRed);

    public:

        // The only constructor
        TrafficLightGenerator();

        /* Adds another traffic point with the supplied information
         * param theSide: The side this Highway is
         * param highway: The highway to have the lights added
         * param distanceInHighway: The distance into the highway to place the lights
         * param leftTurnLanes: The lanes which are left turn lanes
         * */
        void SetHighway(Side theSide, Ptr<Highway> highway, double distanceInHighway, std::list<int> leftTurnLanes);

        // Sets the time the straight path is open
        void SetTimeStraight(double timeInSeconds);
        // Gets the time the straight path is open
        double GetTimeStraight();

        // Sets the time the left path is open
        void SetTimeLeft(double timeInSeconds);
        // Gets the time the left path is open
        double GetTimeLeft();

        // Sets the time between states when all lights are red
        void SetTimeBuffer(double timeInSeconds);
        // Gets the time between states when all lights are red
        double GetTimeBuffer();

        // Starts the TrafficLightGenerator
        void Start();

        // Steps the traffic light to the next state
        static void Step(TrafficLightGenerator* ptr);
    };

}

#endif	/* _TRAFFICLIGHTGENERATOR_H */

