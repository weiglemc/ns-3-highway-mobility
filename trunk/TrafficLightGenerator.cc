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

#include "TrafficLightGenerator.h"
#include "IdGenerator.h"
#include "Vehicle.h"
#include "Obstacle.h"
#include "ns3/simulator.h"
#include <map>
#include <iostream>

namespace ns3 {

    /*
     * Constructor initializes the various collections needed
     */
    TrafficLightGenerator::TrafficLightGenerator() {
        // Create the map
        m_highwayMap = std::map<Side, TrafficPoint*>();
        // Make sure that each side has an entry
        m_highwayMap[LEFT] = NULL;
        m_highwayMap[RIGHT] = NULL;
        m_highwayMap[TOP] = NULL;
        m_highwayMap[BOTTOM] = NULL;
        // Default timers
        m_timeLeft = 20.0;
        m_timeStraight = 30.0;
        m_timeBuffer = 5.0;
    }

    /*
     * Sets the time the straight path is open
     */
    void TrafficLightGenerator::SetTimeStraight(double timeInSeconds) {
        m_timeStraight = timeInSeconds;
    }

    /*
     * Gets the time the straight path is open
     */
    double TrafficLightGenerator::GetTimeStraight() {
        return m_timeStraight;
    }

    /*
     * Sets the time the left path is open
     */
    void TrafficLightGenerator::SetTimeLeft(double timeInSeconds) {
        m_timeLeft = timeInSeconds;
    }

    /*
     * Gets the time the left path is open
     */
    double TrafficLightGenerator::GetTimeLeft() {
        return m_timeLeft;
    }

    /*
     * Sets the time between states when all lights are red
     */
    void TrafficLightGenerator::SetTimeBuffer(double timeInSeconds) {
        m_timeBuffer = timeInSeconds;
    }

    /*
     * Gets the time between states when all lights are red
     */
    double TrafficLightGenerator::GetTimeBuffer() {
        return m_timeBuffer;
    }

    /* Adds another traffic point with the supplied information
     * param theSide: The side this Highway is
     * param highway: The highway to have the lights added
     * param distanceInHighway: The distance into the highway to place the lights
     * param leftTurnLanes: The lanes which are left turn lanes
     * */
    void TrafficLightGenerator::SetHighway(Side theSide, Ptr<Highway> highway, double distanceInHighway, std::list<int> leftTurnLanes) {
        // Create the traffic pointer
        TrafficPoint* newPoint = new TrafficPoint;
        // Set the highway
        newPoint->highway = highway;
        // Set the distance into the highway
        newPoint->distance = distanceInHighway;
        // Sets the left turn lanes
        newPoint->leftTurnLanes = leftTurnLanes;

        // Initialize the maps for holding the pre-created obstacles
        newPoint->leftTurnLights = std::map<int, Ptr<Vehicle> >();
        newPoint->straightLights = std::map<int, Ptr<Vehicle> >();

        // Create all of the Obstacles for the various lanes
        for (int i = 1; i <= highway->GetNumberOfLanes(); i++) {
            // Check to see if the lane is a left turn lane or not
            bool isLeft = false;
            for (std::list<int>::iterator it = leftTurnLanes.begin(); it != leftTurnLanes.end(); it++) {
                if (i == (*it)) {
                    isLeft = true;
                    break;
                }
            }

            // Create the obstacle
            Ptr<Vehicle> obstacle = CreateObject<Obstacle > ();
            // Set the id from the IdGenerator
            obstacle->SetVehicleId(IdGenerator::nextVehicleId());
            // Set the current lane
            obstacle->SetLane(i);
            // Ste the vehicle type to 2 (traffic light)
            obstacle->SetVehicleType(2);
            // Put the obstalce into the correct map
            if (isLeft) {
                newPoint->leftTurnLights[i] = obstacle;
            } else {
                newPoint->straightLights[i] = obstacle;
            }
        }

        // Set the traffic point to the correct side
        m_highwayMap[theSide] = newPoint;
    }

    /**
     * Start the TrafficLightGenerator
     * Initializes all of the lights and starts the scheduling
     */
    void TrafficLightGenerator::Start() {

        // Add the lights to all lanes for the initial state
        for (std::map<Side, TrafficPoint*>::iterator it = m_highwayMap.begin(); it != m_highwayMap.end(); it++) {
            // For this side, set the left lights to red
            ChangeLights(it->first, true, true);
            // For this side, set the straight lights to red
            ChangeLights(it->first, false, true);

            // If the side is a left or a right, set the left/right flag to true
            if (it->first == LEFT || it->first == RIGHT) {
                m_hasLR = true;
            }
            // If the side is a top or a bottom, set the top/bottom flag to true
            if (it->first == TOP || it->first == BOTTOM) {
                m_hasTB = true;
            }
        }

        // If we have left/right, the start state is left turn first on the left/right direction
        if (m_hasLR) {
            m_currentState = LEFTTURN_LR;
        } else if (m_hasTB) {
            // Else we start with a left turn on the top/bottom direction
            m_currentState = LEFTTURN_TB;
        }

        // If we have any highways configured, start the scheduler
        if (m_hasLR || m_hasTB) {
            Simulator::Schedule(Seconds(0.0), &Step, this);
        }

    }

    /**
     * This is a utility function for changing the lights on a particular side
     * param: side - the side to change the lights on
     * param: leftTurnLane - true if we are modifying the left turn lanes, else straight
     * param: toRed - true if we are adding the obstacles, else we are removing the obstacles
     */
    void TrafficLightGenerator::ChangeLights(Side side, bool leftTurnLane, bool toRed) {
        // Check to make sure we have data on this side
        if (m_highwayMap.find(side) == m_highwayMap.end()) {
            return;
        }
        // Get the side
        TrafficPoint* tp = m_highwayMap[side];

        // If we are adding the obstalces
        if (toRed) {
            // If we are handling left turn lanes
            if (leftTurnLane) {
                // For each left turn lane configured
                for (std::list<int>::iterator it = tp->leftTurnLanes.begin(); it != tp->leftTurnLanes.end(); it++) {
                    // Add light to the highway at the specified distance
                    Ptr<Vehicle> veh = tp->leftTurnLights[(*it)];
                    tp->highway->AddVehicle(veh, tp->distance);
                }
            // If we are handling right turn lanes
            } else {
                // For each lane
                for (int i = 1; i <= tp->highway->GetNumberOfLanes(); i++) {
                    // Check for this being a left turn lane
                    bool isLeft = false;
                    for (std::list<int>::iterator it = tp->leftTurnLanes.begin(); it != tp->leftTurnLanes.end(); it++) {
                        if (i == (*it)) {
                            isLeft = true;
                            break;
                        }
                    }
                    // If it is a left turn lane, skip it
                    if (isLeft) {
                        continue;
                    }
                    // Add the light to the highway at the specified distance
                    Ptr<Vehicle> veh = tp->straightLights[i];
                    tp->highway->AddVehicle(veh, tp->distance);
                }
            }
        // If we are removing the obstalces
        } else {
            // If we are handling left turn lanes
            if (leftTurnLane) {
                // For each obstacle we created, remove it
                for (std::map<int, Ptr<Vehicle> >::iterator it = tp->leftTurnLights.begin(); it != tp->leftTurnLights.end(); it++) {
                    tp->highway->RemoveVehicle(it->second);
                }
            // If we are handling straight lanes
            } else {
                // For each obstacle we created, remove it
                for (std::map<int, Ptr<Vehicle> >::iterator it = tp->straightLights.begin(); it != tp->straightLights.end(); it++) {
                    tp->highway->RemoveVehicle(it->second);
                }
            }
        }

    }

    /**
     * Step from the current state to the next state
     */
    void TrafficLightGenerator::Step(TrafficLightGenerator* ptr) {
        // Create the time to sleep variable
        double timeToSleep = 0.0;
        // Switch on the current state
        switch (ptr->m_currentState) {
            // If the current state is left turn in the left/right direction
            case LEFTTURN_LR:
            {
                // We remove the lights from the left turn lanes
                // on the left and right sides
                ptr->ChangeLights(LEFT, true, false);
                ptr->ChangeLights(RIGHT, true, false);
                // Set the next state to be waiting
                ptr->m_currentState = WAIT_LR;
                // Set the time to wait as the time for left turn lights
                timeToSleep = ptr->m_timeLeft;
                break;
            }
            // If the current state is wait in the left/right direction
            case WAIT_LR:
            {
                // We add the lights for the left turn lanes
                // on the left and right sides
                ptr->ChangeLights(LEFT, true, true);
                ptr->ChangeLights(RIGHT, true, true);
                // The next state is going straight
                ptr->m_currentState = STRAIGHT_LR;
                // Wait for the buffer time
                timeToSleep = ptr->m_timeBuffer;
                break;
            }
            // If the current state is go straight
            case STRAIGHT_LR:
            {
                // We remove the lights for the straight lanes
                // on the left and right sides
                ptr->ChangeLights(LEFT, false, false);
                ptr->ChangeLights(RIGHT, false, false);
                // The next state is everything waits
                ptr->m_currentState = WAITALL_LR;
                // Wait for the straight lights time
                timeToSleep = ptr->m_timeStraight;
                break;
            }
            // If the current state is the time to transition from left/right
            // to top bottom
            case WAITALL_LR:
            {
                // We add the lights for the straigh lanes
                // on the left and right
                ptr->ChangeLights(LEFT, false, true);
                ptr->ChangeLights(RIGHT, false, true);
                // If we have a top bottom
                if (ptr->m_hasTB) {
                    // The next state is the left turn for top/bottom
                    ptr->m_currentState = LEFTTURN_TB;
                } else {
                    // Otherwise start the loop over again
                    ptr->m_currentState = LEFTTURN_LR;
                }
                // The time we sleep is the wait time
                timeToSleep = ptr->m_timeBuffer;
                break;
            }
            // Same as LEFTTURN_LR except for top/bottom
            case LEFTTURN_TB:
            {
                ptr->ChangeLights(TOP, true, false);
                ptr->ChangeLights(BOTTOM, true, false);
                ptr->m_currentState = WAIT_TB;
                timeToSleep = ptr->m_timeLeft;
                break;
            }
            // Same as WAIT_LR except for top/bottom
            case WAIT_TB:
            {
                ptr->ChangeLights(TOP, true, true);
                ptr->ChangeLights(BOTTOM, true, true);
                ptr->m_currentState = STRAIGHT_TB;
                timeToSleep = ptr->m_timeBuffer;
                break;
            }
            // Same as STRAIGHT_LR except for top/bottom
            case STRAIGHT_TB:
            {
                ptr->ChangeLights(TOP, false, false);
                ptr->ChangeLights(BOTTOM, false, false);
                ptr->m_currentState = WAITALL_TB;
                timeToSleep = ptr->m_timeStraight;
                break;
            }
            // Same as WAITALL_LR except for top/bottom
            case WAITALL_TB:
            {
                ptr->ChangeLights(TOP, false, true);
                ptr->ChangeLights(BOTTOM, false, true);
                if (ptr->m_hasLR) {
                    ptr->m_currentState = LEFTTURN_LR;
                } else {
                    ptr->m_currentState = LEFTTURN_TB;
                }
                timeToSleep = ptr->m_timeBuffer;
                break;
            }
        }

        //Schedule the next step for the traffic light generator
        Simulator::Schedule(Seconds(timeToSleep), &Step, ptr);
    }

}

