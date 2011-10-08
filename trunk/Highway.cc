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

#include <iostream>
#include "Highway.h"
#include "ns3/simulator.h"
#include "Geometry.h"
#include "Obstacle.h"
#include "IdGenerator.h"
#include <math.h>

namespace ns3 {

    TypeId Highway::GetTypeId(void) {
        static TypeId tid = TypeId("ns3::Highway")
                .SetParent<Object > ()
                .AddConstructor<Highway > ()
                ;
        return tid;
    }

    //*******************************************************************
    //
    //Constructors/Destructors
    //
    //*******************************************************************

    Highway::Highway() {
        //Default direction, dplacement, length, and width
        m_direction = 0.0;
        m_xPos = 0.0;
        m_yPos = 0.0;
        m_dt = 0.1;
        m_highwayLength = 1000;
        m_laneWidth = 5;

        //Default is 10 MPH
        m_leftTurnSpeed = 4.4704;
        m_rightTurnSpeed = 4.4704;

        //Default number of lanes is 1
        m_numberOfLanes = 1;

        //These arrays are initialized to a size of 1 with NULL entries
        m_frontHighways = new Ptr<Highway> [1];
        m_frontHighways[0] = NULL;
        m_frontOffsets = new int [1];
        m_backHighways = new Ptr<Highway>[1];
        m_backHighways[0] = NULL;
        m_backOffsets = new int [1];
        m_rightHighways = new Ptr<Highway> [1];
        m_rightHighways[0] = NULL;
        m_rightOffsets = new int [1];
        m_leftHighways = new Ptr<Highway> [1];
        m_leftHighways[0] = NULL;
        m_leftOffsets = new int [1];

        m_changeLaneSet = true;
        m_loop = 0;

        //Initialize the routing map
        m_routingMap = std::map<int, TurningType > ();

        //Create the transfer list
        transferList = new list<Ptr<Vehicle> >();
    }

    //Cleans up memeory

    Highway::~Highway() {
        m_tempVehicles[0] = 0;
        m_tempVehicles[1] = 0;
        transferList->clear();
        delete transferList;
        delete [] m_frontHighways;
        delete [] m_frontOffsets;
        delete [] m_backHighways;
        delete [] m_backOffsets;
        delete [] m_rightHighways;
        delete [] m_rightOffsets;
        delete [] m_leftHighways;
        delete [] m_leftOffsets;
        ClearLanes();
    }

    void Highway::SetHighwayId(int highwayId) {
        m_highwayId = highwayId;
    }

    int Highway::GetHighwayId() {
        return m_highwayId;
    }

    void Highway::SetRoutingMap(std::map<int, TurningType> routingMap) {
        m_routingMap = routingMap;
    }

    //Initializing the Highway

    void Highway::InitHighway() {
        InitLanes();
    }

    //This function deletes all vehicles from all lanes and clears the lane
    //map

    void Highway::ClearLanes() {
        LaneMap::iterator it;
        for (it = m_laneMap.begin(); it != m_laneMap.end(); it++) {
            it->second->vehicles->clear();
            delete it->second->vehicles;
            delete it->second;
        }
        m_laneMap.clear();

    }

    void Highway::InitLanes() {
        //The purpose of this function is to initialize the lanes based on
        //the highway location and the number of lanes.  The lanes are centered
        //on the Highway start and are arranged along the Y axis unless the highway
        //is straight up and down (where they are arranged along the X axis)
        //Note that the first lanes is the "leftmost"

        //Normalizing the angle to [0,2pi)
        double normalizedAngle = normalizeAngle(m_direction);

        double laneXShift = 0.0;
        double laneYShift = 0.0;
        double sineValue = 0.0;
        double startXVal = 0.0;
        double startYVal = 0.0;
        double result[2];


        //If the angle is 90 degrees
        if (anglesEqual(normalizedAngle, M_PI / 2.0)) {
            //The lanes shift in the X their entire length
            laneXShift = m_laneWidth;
            //If the angle is 270 degrees
        } else if (anglesEqual(normalizedAngle, 3.0 * M_PI / 2.0)) {
            //The lanes shift in the X their entire length (but in the
            //opposite direction)
            laneXShift = -m_laneWidth;
        } else {
            //Any other angles, all lane positions are moved only in the Y
            //location.  The following calculations determine, based on the
            //lane width, how much the start point of the lane has to shift
            //between lanes
            if (normalizedAngle >= 0 && normalizedAngle < M_PI / 2.0) {
                sineValue = -sin((M_PI / 2.0) - normalizedAngle);
            } else if (normalizedAngle > M_PI / 2.0 && normalizedAngle < M_PI) {
                sineValue = sin(normalizedAngle - (M_PI / 2.0));
            } else if (anglesEqual(normalizedAngle, M_PI)) {
                sineValue = 1.0;
            } else if (normalizedAngle > M_PI && normalizedAngle < 3.0 * M_PI / 2.0) {
                sineValue = sin(3.0 * M_PI / 2.0 - normalizedAngle);
            } else {
                sineValue = -sin(normalizedAngle - 3.0 * M_PI / 2.0);
            }
            laneYShift = m_laneWidth / sineValue;
        }

        //This is the distance for half of the lanes
        double distance = (m_laneWidth * (m_numberOfLanes - 1)) / 2.0;

        //If the angle is 90 degrees
        if (anglesEqual(normalizedAngle, M_PI / 2.0)) {
            //The shift is entire in the X direction
            startXVal = m_xPos - distance;
            startYVal = m_yPos;
            //If the angle is 270 degrees
        } else if (anglesEqual(normalizedAngle, 3.0 * M_PI / 2.0)) {
            //The shift is in X, but in the other direction
            startXVal = m_xPos + distance;
            startYVal = m_yPos;
        } else {
            //Shift is in Y, modified by the geometry of the angle
            startYVal = m_yPos - distance / sineValue;
            startXVal = m_xPos;
        }

        //For each lane
        for (int i = 1; i <= m_numberOfLanes; i++) {
            //Create the lane, assigning the current start X and Y,
            //calculate the end X and Y, and prepare the startXVal and
            //startYVal for the next lane
            Lane* lane = new Lane;
            lane->id = i;
            lane->direction = m_direction;
            lane->vehicles = new VehicleList();
            lane->startX = startXVal;
            lane->startY = startYVal;
            result[0] = startXVal;
            result[1] = startYVal;
            translatePoint(result, m_direction, m_highwayLength);
            lane->endX = result[0];
            lane->endY = result[1];
            startXVal += laneXShift;
            startYVal += laneYShift;
            //Add to the map based on the ID
            m_laneMap[i] = lane;
        }

    }

    bool Highway::GetChangeLane() {
        return m_changeLaneSet;
    }

    void Highway::SetChangeLane(bool value) {
        m_changeLaneSet = value;
    }

    double Highway::GetDeltaT() {
        return m_dt;
    }

    void Highway::SetDeltaT(double value) {
        if (value <= 0)
            value = 0.1;

        m_dt = value;
    }

    //Add a Highway for going straight

    void Highway::AddFrontHighway(Ptr<Highway> frontHighway, int laneOffset, int frontOffset) {
        //Check that the lane offset is viable
        if (laneOffset < 0 || laneOffset > (m_numberOfLanes - 1)) {
            return;
        }

        //For each location from the laneOffset for as many lanes as
        //possible
        int start = laneOffset;
        int end = min(laneOffset + frontHighway->GetNumberOfLanes(), m_numberOfLanes);
        for (int i = start; i < end; i++) {
            //Assign the highway and the offset
            m_frontHighways[i] = frontHighway;
            m_frontOffsets[i] = frontOffset;
        }

        //For each lane
        for (int i = 1; i <= m_numberOfLanes; i++) {
            Lane* aLane = m_laneMap[i];
            //Clear any current obstacles
            aLane->vehicles->clear();
            //If there is no connecting highway
            if (m_frontHighways[i - 1] == NULL && m_rightHighways[i - 1] == NULL && m_leftHighways[i - 1] == NULL) {
                //Create an obstacle to force vehicles to get out of the lane
                Ptr<Vehicle> obstacle = CreateObject<Obstacle > ();
                obstacle->SetVehicleId(IdGenerator::nextVehicleId());
                obstacle->SetLane(i);
                obstacle->SetVehicleType(1);
                //Add it at the end of the highway
                AddVehicle(obstacle, m_highwayLength);
            }
        }
    }

    //Add a highway behind (for lane change calculations mostly)

    void Highway::AddBackHighway(Ptr<Highway> backHighway, int laneOffset, int backOffset) {
        if (laneOffset < 0 || laneOffset > (m_numberOfLanes - 1)) {
            return;
        }
        int start = laneOffset;
        int end = min(laneOffset + backHighway->GetNumberOfLanes(), m_numberOfLanes);
        for (int i = start; i < end; i++) {
            m_backHighways[i] = backHighway;
            m_backOffsets[i] = backOffset;
        }

        //Vehicles don't travel backwards, so we don't have to place
        //obstalces
    }

    //Add a Right Highway
    //These Right Highways are used instead of a Front highway if the vehicle wants to
    //turn right

    void Highway::AddRightHighway(Ptr<Highway> rightHighway, int laneOffset, int rightOffset) {
        if (laneOffset < 0 || laneOffset > (m_numberOfLanes - 1)) {
            return;
        }
        int start = laneOffset;
        int end = min(laneOffset + rightHighway->GetNumberOfLanes(), m_numberOfLanes);
        for (int i = start; i < end; i++) {
            m_rightHighways[i] = rightHighway;
            m_rightOffsets[i] = rightOffset;
        }

        for (int i = 1; i <= m_numberOfLanes; i++) {
            Lane* aLane = m_laneMap[i];
            aLane->vehicles->clear();
            if (m_frontHighways[i - 1] == NULL && m_rightHighways[i - 1] == NULL && m_leftHighways[i - 1] == NULL) {
                Ptr<Vehicle> obstacle = CreateObject<Obstacle > ();
                obstacle->SetVehicleId(IdGenerator::nextVehicleId());
                obstacle->SetLane(i);
                obstacle->SetVehicleType(1);
                AddVehicle(obstacle, m_highwayLength);
            }
        }
    }

    //Add a Left Highway
    //These Left Highways are used instead of a Front Highway if the vehicle wants to
    //turn left

    void Highway::AddLeftHighway(Ptr<Highway> leftHighway, int laneOffset, int leftOffset) {
        if (laneOffset < 0 || laneOffset > (m_numberOfLanes - 1)) {
            return;
        }
        int start = laneOffset;
        int end = min(laneOffset + leftHighway->GetNumberOfLanes(), m_numberOfLanes);
        for (int i = start; i < end; i++) {
            m_leftHighways[i] = leftHighway;
            m_leftOffsets[i] = leftOffset;
        }

        for (int i = 1; i <= m_numberOfLanes; i++) {
            Lane* aLane = m_laneMap[i];
            aLane->vehicles->clear();
            if (m_frontHighways[i - 1] == NULL && m_rightHighways[i - 1] == NULL && m_leftHighways[i - 1] == NULL) {
                Ptr<Vehicle> obstacle = CreateObject<Obstacle > ();
                obstacle->SetVehicleId(IdGenerator::nextVehicleId());
                obstacle->SetLane(i);
                obstacle->SetVehicleType(1);
                AddVehicle(obstacle, m_highwayLength);
            }
        }
    }

    Highway::TurningType Highway::GetTurningTypeForHighwayId(int destHighwayId) {
        //The default value is STRAIGHT, even if the highway is not connected
        TurningType retValue = STRAIGHT;

        //Search all lanes for the highway
        for (int i = 0; i < m_numberOfLanes; i++) {
            if (m_leftHighways[i] != NULL && m_leftHighways[i]->GetHighwayId() == destHighwayId) {
                retValue = LEFT;
                break;
            }
            if (m_rightHighways[i] != NULL && m_rightHighways[i]->GetHighwayId() == destHighwayId) {
                retValue = RIGHT;
                break;
            }
        }

        return retValue;
    }

    int Highway::GetNumberOfLanes() {
        return m_numberOfLanes;
    }

    void Highway::SetNumberOfLanes(int value) {
        if (value < 1)
            value = 1;

        m_numberOfLanes = value;

        //Anytime the number of lanes changes, we need to reset the
        //highway connection arrays
        m_frontHighways = new Ptr<Highway>[m_numberOfLanes];
        m_frontOffsets = new int[m_numberOfLanes];
        m_backHighways = new Ptr<Highway>[m_numberOfLanes];
        m_backOffsets = new int[m_numberOfLanes];
        m_rightHighways = new Ptr<Highway>[m_numberOfLanes];
        m_rightOffsets = new int[m_numberOfLanes];
        m_leftHighways = new Ptr<Highway>[m_numberOfLanes];
        m_leftOffsets = new int[m_numberOfLanes];
        for (int i = 0; i < m_numberOfLanes; i++) {
            m_frontHighways[i] = NULL;
            m_backHighways[i] = NULL;
            m_rightHighways[i] = NULL;
            m_leftHighways[i] = NULL;
        }
    }

    double Highway::GetHighwayLength() {
        return m_highwayLength;
    }

    void Highway::SetHighwayLength(double value) {
        if (value < 0)
            value = 10000;

        m_highwayLength = value;
    }

    double Highway::GetLaneWidth() {
        return m_laneWidth;
    }

    void Highway::SetLaneWidth(double value) {
        if (value < 0)
            value = 5;

        m_laneWidth = value;
    }

    double Highway::GetXPos() {
        return m_xPos;
    }

    void Highway::SetXPos(double xPos) {
        m_xPos = xPos;
    }

    double Highway::GetYPos() {
        return m_yPos;
    }

    void Highway::SetYPos(double yPos) {
        m_yPos = yPos;
    }

    double Highway::GetRightTurnSpeed() {
        return m_rightTurnSpeed;
    }

    void Highway::SetRightTurnSpeed(double rightTurnSpeed) {
        m_rightTurnSpeed = rightTurnSpeed;
    }

    double Highway::GetLeftTurnSpeed() {
        return m_leftTurnSpeed;
    }

    void Highway::SetLeftTurnSpeed(double leftTurnSpeed) {
        m_leftTurnSpeed = leftTurnSpeed;
    }

    void Highway::SetDirection(double direction) {
        m_direction = direction;
    }

    double Highway::GetDirection() {
        return m_direction;
    }

    //This returns the turning type based on the routing map supplied to
    //this Highway instance (STRAIGHT for default)

    Highway::TurningType Highway::GetTurningType(int highwayId) {
        std::map<int, TurningType>::iterator it;
        it = m_routingMap.find(highwayId);
        if (it == m_routingMap.end()) {
            return STRAIGHT;
        } else {
            return it->second;
        }
    }

    //Searches all lanes for the vehicle and removes it

    void Highway::RemoveVehicle(int vid) {
        int minIndex = 1;
        int maxIndex = m_numberOfLanes;
        for (int i = minIndex; i <= maxIndex; i++) {
            if (i == 0) {
                continue;
            }
            VehicleList* vList = m_laneMap[i]->vehicles;
            for (VehicleList::iterator itr = vList->begin(); itr != vList->end(); itr++) {
                if ((*itr)->GetVehicleId() == vid) {
                    vList->erase(itr);
                    break;
                }
            }

        }
    }

    //Same as RemoveVehicle(int)

    void Highway::RemoveVehicle(Ptr<Vehicle> vehicle) {
        RemoveVehicle(vehicle->GetVehicleId());
    }

    //A utility function for iterating through the linked list used

    Ptr<Vehicle> Highway::GetVehicleFromList(VehicleList* v, int index) {
        VehicleList::iterator i = v->begin();
        advance(i, index);
        return *i;
    }

    //Returns the vehicle in the lane specified at the index specified

    Ptr<Vehicle> Highway::GetVehicle(int laneNumber, int index) {
        //Check and make sure that the laneNumber is valid
        if (laneNumber < 0 || laneNumber > m_numberOfLanes) {
            return NULL;
        }

        //Get the vehicle list
        VehicleList* v = m_laneMap[laneNumber]->vehicles;
        //If the index is less than 0
        if (index < 0) {
            //We are looking for a vehicle that is past the end of the current
            //Highway
            //If the front highway is not NULL
            if (m_frontHighways[laneNumber - 1] != NULL) {
                //return that Highway's first Vehicle
                return m_frontHighways[laneNumber - 1]->GetFirstVehicle(laneNumber - m_frontOffsets[laneNumber - 1]);
            } else {
                return NULL;
            }
            //If the index is greater than any we current have
        } else if (index >= (int) v->size()) {
            //Then we need to look for a vehicle that is before the start of
            //the current Highway
            //If the back Highway is not null
            if (m_backHighways[laneNumber - 1] != NULL) {
                //return that Highway's last vehicle
                return m_backHighways[laneNumber - 1]->GetLastVehicle(laneNumber - m_backOffsets[laneNumber - 1]);
            } else {
                return NULL;
            }
            //If it is a vehicle available in this highway
        } else {
            //Return the vehicle
            VehicleList::iterator i = v->begin();
            advance(i, index);
            return *i;
        }
    }

    //Returns the distance to a vehicle, either in this Highway or a Highway
    //connected to this one.  If no Vehicle is available, return NAN

    double Highway::GetDistance(int laneNumber, int index, bool fromStart) {
        //Get the current lane
        Lane* vehiclesLane = m_laneMap[laneNumber];

        //If the index is less than 0
        if (index < 0) {
            //We are looking for a vehicle that is after the end of this Highway
            if (m_frontHighways[laneNumber - 1] != NULL) {
                //Get the result from the connected highway
                double result = m_frontHighways[laneNumber - 1]->GetDistanceToFirstVehicle(laneNumber - m_frontOffsets[laneNumber - 1]);
                //If there is a result
                if (!isnan(result)) {
                    //Based on where we are measuring from, add m_highwayLength plus
                    //the result
                    return fromStart ? result + m_highwayLength : result;
                } else {
                    return NAN;
                }
            } else {
                return NAN;
            }
            //If the index is greater than the current number of vehicles
        } else if (index >= (int) vehiclesLane->vehicles->size()) {
            //We are looking for a vehicle that is before the start of this Highway
            if (m_backHighways[laneNumber - 1] != NULL) {
                //Get the result from the connected highway
                double result = m_backHighways[laneNumber - 1]->GetDistanceToLastVehicle(laneNumber - m_backOffsets[laneNumber - 1]);
                //If there is a result
                if (!isnan(result)) {
                    //Based on where we are measuring from, add m_highwayLength plus
                    //the result
                    return fromStart ? result : result + m_highwayLength;
                } else {
                    return NAN;
                }
            } else {
                return NAN;
            }
            //Otherwise, it is a vehicle in this lane
        } else {
            //Get the distance
            Ptr<Vehicle> vehicle = GetVehicle(laneNumber, index);
            if (vehicle == NULL) {
                return NAN;
            } else {
                if (fromStart) {
                    return directDistance(vehicle->GetPosition().x, vehicle->GetPosition().y, vehiclesLane->startX, vehiclesLane->startY);
                } else {
                    return directDistance(vehicle->GetPosition().x, vehicle->GetPosition().y, vehiclesLane->endX, vehiclesLane->endY);
                }
            }
        }
    }

    //When asking for the First vehicle, what we are looking for is the
    //vehicle that is closest in the lane to startX, startY

    Ptr<Vehicle> Highway::GetFirstVehicle(int lane) {
        //Check that the Lane is correct
        if (lane < 1 || lane > m_numberOfLanes) {
            return NULL;
        }
        //Get the lane
        VehicleList* vehicles = m_laneMap[lane]->vehicles;
        //If the list is empty
        if (vehicles->empty()) {
            //If there is a connected highway
            if (m_frontHighways[lane - 1] != NULL) {
                //Get the first vehicle from that Highway
                return m_frontHighways[lane - 1]->GetFirstVehicle(lane - m_frontOffsets[lane - 1]);
            } else {
                return NULL;
            }
        }
        //We need the vehicle with the highest index
        return vehicles->back();
    }

    //We are asking for the distance to the first vehicle from the start
    //of the lane

    double Highway::GetDistanceToFirstVehicle(int lane) {
        //Check that the lane is correct
        if (lane < 1 || lane > m_numberOfLanes) {
            return NAN;
        }
        VehicleList* vehicles = m_laneMap[lane]->vehicles;
        //If the list is empty
        if (vehicles->empty()) {
            //Make sure that we geth the distance to the vehicle in the
            //connecting lane by passing in a negative index (always < 0)
            return GetDistance(lane, -1, true);
        } else {
            //Otherwise, get the distance to the vehicle with the largest index
            return GetDistance(lane, vehicles->size() - 1, true);
        }
    }

    //We are asking for the vehicle that is the closest to the end of the lane

    Ptr<Vehicle> Highway::GetLastVehicle(int lane) {
        if (lane < 1 || lane > m_numberOfLanes) {
            return NULL;
        }
        VehicleList* vehicles = m_laneMap[lane]->vehicles;
        //If the lane is current empty
        if (vehicles->empty()) {
            //If we have a back highway
            if (m_backHighways[lane - 1] != NULL) {
                //Get the last vehilce from that Highway
                return m_backHighways[lane - 1]->GetLastVehicle(lane - m_backOffsets[lane - 1]);
            } else {
                return NULL;
            }
        }
        return vehicles->front();
    }

    //We are asking for the distance from the end of the lane to the
    //vehicle closest to the end of the lane

    double Highway::GetDistanceToLastVehicle(int lane) {
        if (lane < 1 || lane > m_numberOfLanes) {
            return NAN;
        }
        VehicleList* vehicles = m_laneMap[lane]->vehicles;
        //If the current lane is empty
        if (vehicles->empty()) {
            //Ensure that we get the distance for the vehicle in the
            //connecting lane by passing in an index greater than
            return GetDistance(lane, 1, false);
        } else {
            //Otherwise, get the distance to the vehilce with the smallest
            //index
            return GetDistance(lane, 0, false);
        }
    }

    //Returns the start location of the requested lane

    Vector Highway::GetLaneStart(int lane) {
        Vector v(-10000.0, -10000.0, 0.0);
        if (lane >= 1 && lane <= m_numberOfLanes) {
            v.x = m_laneMap[lane]->startX;
            v.y = m_laneMap[lane]->startY;
        }
        return v;
    }

    //This function is the point of entry for controlling vehicles in the Highway

    void Highway::TranslateVehicles() {

        //Only every tenth loop do we try to change lanes
        // NOTE: ORDER OF CALLING THIS FUNCTIONS IS VERY VERY IMPORTANT (EFFECT OF CURRENT SPEED, POSITION, DECICION)
        if (m_loop == 10) m_loop = 0;
        if (m_loop == 0 && m_changeLaneSet == true) {
            if (m_changeLaneSet) {
                ChangeLane();
            }
        }

        //First, we translate the position and velocities of all vehicles
        TranslatePositionVelocity(m_dt);

        //Then we modify the accelerations of the vehicles for the next loop
        Accelerate(m_dt);

        m_loop++;

    }

    //This function iterators over all possible lane combinations and
    //checks lane changing from any combination

    void Highway::ChangeLane() {
        if (m_numberOfLanes <= 1) {
            return;
        }

        for (int i = 1; i <= m_numberOfLanes; i++) {
            if (i <= 1) {
                DoChangeLaneIfPossible(i, i + 1);
                continue;
            }
            if (i + 1 >= (m_numberOfLanes + 1)) {
                DoChangeLaneIfPossible(i, i - 1);
                continue;
            }
            DoChangeLaneIfPossible(i, i + 1);
            DoChangeLaneIfPossible(i, i - 1);
        }

    }

    void Highway::DoChangeLaneIfPossible(int curLane, int desLane) {
        //result is used to store point transforms
        double result[2];

        //This list indicates that a vehicle is ready to be moved to the
        //destination lane
        std::list<Ptr<Vehicle> >* canChange = new std::list<Ptr<Vehicle> >();

        //The current lane
        VehicleList* currentLane = m_laneMap[curLane]->vehicles;

        //The desintation lane
        Lane* destLane = m_laneMap[desLane];

        //These booleans indicate whether the source and destination lanes
        //have left connections, right connections, and/or straight connections
        bool curLeft = (m_leftHighways[curLane - 1] != NULL);
        bool curRight = (m_rightHighways[curLane - 1] != NULL);
        bool desLeft = (m_leftHighways[desLane - 1] != NULL);
        bool desRight = (m_rightHighways[desLane - 1] != NULL);
        bool desStraight = (m_frontHighways[desLane - 1] != NULL);

        //For each vehicle in the lane
        for (uint j = 0; j < currentLane->size(); j++) {
            Ptr<Vehicle> current = GetVehicle(curLane, j);
            //Some objects (traffic lights, obstacles) do not have lane changing
            //ability and should not be processed
            if (current->GetLaneChange() == NULL) {
                continue;
            }

            //We will have to modify the right bias of the vehicle, and so keep
            //a copy here
            double oldBias = current->GetLaneChange()->GetBiasRight();

            //We determine the direction this vehicle wants to turn based on
            //its current destination
            TurningType turningType = GetTurningType(current->GetDestination());
            switch (turningType) {
                    //If we are turning left
                case LEFT:
                {
                    //If the current lane is a left turning lane and the
                    //destination is not, we do not want to make the switch
                    if (curLeft && !desLeft) {
                        continue;
                    }
                    //If the current lane is not a left turning lane, we want
                    //to force the vehicle as far left as we can, so we set
                    //the right bias to -100
                    if (!curLeft) {
                        current->GetLaneChange()->SetBiasRight(-100);
                    }
                    break;
                }
                    //If we are turning right
                case RIGHT:
                {
                    //If the current lane is a right turning lane and the
                    //destination is not, we do not want to make the switch
                    if (curRight && !desRight) {
                        continue;
                    }
                    //If the current lane is not a right turning lane, we want
                    //to force the vehicle as far right as we can, so we set
                    //the right bias to 100
                    if (!curRight) {
                        current->GetLaneChange()->SetBiasRight(100);
                    }
                    break;
                }
                    //If we are going straight
                case STRAIGHT:
                {
                    //We do not want to get into a turn-only lane
                    //if we want to go straight
                    //However, if there is no connecting highway at all,
                    //we will still be able to change lanes
                    if ((desLeft || desRight) && !desStraight) {
                        continue;
                    }
                    break;
                }
            }

            //Get the distance to the current vehicle from the start of the
            //lane
            double currentDistance = GetDistance(curLane, j, true);

            //Get the vehicle in front of the current vehicle
            //note that GetVehicle handles searching connecting Highways
            Ptr<Vehicle> fOld = GetVehicle(curLane, j - 1);
            //Get the distance between the current vehicle and the vehicle in front
            //Subtracting the current vehicles length is because location is tracked from
            //the vehicle's back bumper
            double distance = GetDistance(curLane, j - 1, true) - currentDistance - current->GetLength();

            //This function finds the side vehicles in the destination lane
            //It places the results into m_tempVehicles and m_tempDistances
            FindSideVehicles(current, currentDistance, desLane);

            //Pass control to the Vehicle's LaneChange model with all the necessary
            //information
            if (current->CheckLaneChange(fOld, distance, m_tempVehicles[0], m_tempDistances[0], m_tempVehicles[1], m_tempDistances[1], (desLane < curLane) ? true : false)) {
                //If the lane change is desired, push the vehicle into the "canChange" list
                canChange->push_back(GetVehicle(curLane, j));
            }

            //Reset the vehicle's bias
            current->GetLaneChange()->SetBiasRight(oldBias);

        }

        //For each vehicle in the canChange list
        for (uint j = 0; j < canChange->size(); j++) {
            //Get the vehicle
            Ptr<Vehicle> curVehicle = GetVehicleFromList(canChange, j);

            //Translate the vehicle from its current position
            Vector position = curVehicle->GetPosition();
            result[0] = position.x;
            result[1] = position.y;
            double angleChange = (curLane < desLane) ? -M_PI / 2.0 : M_PI / 2.0;
            //to the point in the adjacent lane
            translatePoint(result, GetVehicleFromList(canChange, j)->GetDirection() + angleChange, m_laneWidth);
            position.x = result[0];
            position.y = result[1];
            //Adjust the vehicles parameters
            curVehicle->SetLane(desLane);
            curVehicle->SetPosition(position);
            currentLane->remove(curVehicle);
            //Insert the vehicle in the lane
            InsertIntoLane(curVehicle, destLane);
        }

        //Clean up
        canChange->clear();
        delete canChange;
    }

    //This function does the main body of the work of translating vehicles along
    //the highway

    void Highway::TranslatePositionVelocity(double dt) {
        //This array is for storing the point translations
        double tempPos[2];
        int maxIndex = m_numberOfLanes;
        //for each lane
        for (int i = 1; i <= maxIndex; i++) {
            //Get the lane
            Lane *theLane = m_laneMap[i];
            VehicleList* vList = theLane->vehicles;
            //For every vehicle in the lane
            for (uint j = 0; j < vList->size(); j++) {
                //Preset the variables
                bool translate = true;
                Ptr<Vehicle> veh = GetVehicle(i, j);

                bool controled = false;
                if (!m_controlVehicle.IsNull()) {
                    controled = m_controlVehicle(Ptr<Highway > (this), veh, dt);
                }
                if(controled) {
                    continue;
                }


                tempPos[0] = theLane->startX;
                tempPos[1] = theLane->startY;
                double vehDistance = GetDistance(i, j, true);
                if (veh->GetLaneChange() == NULL) {
                    //No need to accelerate or change lanes on something with no
                    //lane change
                    continue;
                }

                //Get the vehicle in front of the current vehicle
                //note that GetVehicle handles searching connecting Highways
                Ptr<Vehicle> fwdVeh = GetVehicle(i, j - 1);
                //Get the distance between the current vehicle and the vehicle in front
                //Subtracting the current vehicles length is because location is tracked from
                //the vehicle's back bumper
                double distance = GetDistance(i, j - 1, true) - vehDistance - veh->GetLength();

                //We can translate If
                //There is no forward vehicle
                if (fwdVeh == NULL) {
                    translate = true;
                    //Or if the acceleration value is NAN
                } else if (isnan(veh->Acceleration(fwdVeh, distance))) {
                    translate = true;
                    //Or if the gap is greater than the minimum gap
                } else if (distance >= veh->GetLaneChange()->GetGapMin()) {
                    translate = true;
                    //otherwise
                } else {
                    //We do not translate
                    translate = false;
                }

                if (translate) {
                    //Use a total distance metric rather than a step metric to
                    //improve accuracy over long paths.
                    double totalDistance = vehDistance + dt * veh->GetVelocity();
                    translatePoint(tempPos, m_direction, totalDistance);

                    if (fwdVeh != NULL) {
                        //Move vehicle up in list if we pass, say, a traffic light without stopping
                        if (!anglesEqual(veh->GetDirection(), angleToPoint(tempPos[0], tempPos[1], fwdVeh->GetPosition().x, fwdVeh->GetPosition().y))) {
                            VehicleList::iterator itr = vList->begin();
                            advance(itr, j);
                            itr = vList->erase(itr);
                            if (itr == vList->begin()) {
                                vList->push_front(veh);
                            } else {
                                itr--;
                                vList->insert(itr, veh);
                            }
                        }
                    }

                    //Adjust the vehicles position
                    veh->SetPosition(Vector3D(tempPos[0], tempPos[1], 0.0));
                    //If we reach the end of the highway, we push the vehicle to the transfer list for
                    //further processing
                    if (!anglesEqual(angleToPoint(tempPos[0], tempPos[1], theLane->endX, theLane->endY), theLane->direction)) {
                        transferList->push_back(veh);
                    }
                }

                veh->TranslateVelocity(dt);

            }

            //Anything in the transfer list is ready to be removed
            for (uint r = 0; r < transferList->size(); r++) {
                Ptr<Vehicle> rm = GetVehicleFromList(transferList, r);
                for (VehicleList::iterator itr = vList->begin(); itr != vList->end(); itr++) {
                    if ((*itr) == rm) {
                        vList->erase(itr);
                        break;
                    }
                }
            }

        }
    }

    //This function changes the acceleration value of the vehicle
    //for the next loop

    void Highway::Accelerate(double dt) {
        int maxIndex = m_numberOfLanes;
        //For each lane
        for (int i = 1; i <= maxIndex; i++) {
            Lane *theLane = m_laneMap[i];
            VehicleList* vList = theLane->vehicles;
            //For each vehicle in the lane
            for (uint j = 0; j < vList->size(); j++) {
                Ptr<Vehicle> veh = GetVehicle(i, j);
                if (veh->GetLaneChange() == NULL) {
                    //No need to accelerate or change lanes on something with no
                    //lane change
                    continue;
                }
                //Get the distance from the front of the lane
                double vehDistance = GetDistance(i, j, true);
                //Check the controling status
                bool controled = false;
                if (!m_controlVehicle.IsNull()) {
                    controled = m_controlVehicle(Ptr<Highway > (this), veh, dt);
                }
                if (controled == false) {
                    //We will be modifying the speed if the vehicle is supposed to be
                    //turning, so we keep a copy here
                    double currentDesiredSpeed = veh->GetModel()->GetDesiredVelocity();
                    //Initialize the new speed
                    double newSpeed = currentDesiredSpeed;
                    //Get the turning desire for the vehicle based on its destination
                    TurningType type = GetTurningType(veh->GetDestination());
                    switch (type) {
                        case LEFT:
                        {
                            //If we are turning left, the new speed is a linear decrease from the
                            //desired speed to the left turn speed by the time the vehicle
                            //gets to the end of the highway
                            if(currentDesiredSpeed > m_leftTurnSpeed) {
                                newSpeed = currentDesiredSpeed - ((currentDesiredSpeed - m_leftTurnSpeed) / m_highwayLength) * vehDistance;
                            }
                            break;
                        }
                        case RIGHT:
                        {
                            //If we are turning right, the new speed is a linear decrease from the
                            //desired speed to the right turn speed by the time the vehicle
                            //gets to the end of the Highway
                            if(currentDesiredSpeed > m_rightTurnSpeed) {
                                newSpeed = currentDesiredSpeed - ((currentDesiredSpeed - m_rightTurnSpeed) / m_highwayLength) * vehDistance;
                            }
                            break;
                        }
                        case STRAIGHT:
                        {
                            //Going straight does not change the desired speed
                            newSpeed = currentDesiredSpeed;
                            break;
                        }
                    }
                    //Adjust the model based on the turning type
                    veh->GetModel()->SetDesiredVelocity(newSpeed);

                    //Get the vehicle in front of the current vehicle
                    //note that GetVehicle handles searching connecting Highways
                    Ptr<Vehicle> otherVeh = GetVehicle(i, j - 1);
                    //Get the distance between the current vehicle and the vehicle in front
                    //Subtracting the current vehicles length is because location is tracked from
                    //the vehicle's back bumper
                    double distance = GetDistance(i, j - 1, true) - vehDistance - veh->GetLength();
                    //Get the acceleration from the Vehicle (does not change the vehicles acceleration
                    double accelValue = veh->Acceleration(otherVeh, distance);
                    //If the next entry is a stop light and we aren't stopping for it
                    //(we're too close), base acceleration on the vehicle past the light
                    if (isnan(accelValue)) {
                        otherVeh = GetVehicle(i, j - 2);
                        distance = GetDistance(i, j - 2, true) - vehDistance - veh->GetLength();
                        accelValue = veh->Acceleration(otherVeh, distance);
                    }
                    //Addjust the acceleration
                    veh->SetAcceleration(accelValue);
                    //Revert the desired speed
                    veh->GetModel()->SetDesiredVelocity(currentDesiredSpeed);
                }
            }
        }
    }

    void Highway::HandleTransfers() {
        //For each vehicle in the transfer list
        for (list<Ptr<Vehicle> >::iterator it = transferList->begin(); it != transferList->end(); it++) {
            Ptr<Vehicle> rm = (*it);
            //If there is no highway to move the vehicle to
            if (!HandleTransfer(rm)) {
                //Disable the necessary callbacks
                if (rm->IsEquipped == true) rm->GetReceiveCallback().Nullify();
                // to put vehicle's node far away from the highway
                // we cannot dispose the vehicle here because its node may still be involved in send and receive process
                rm->SetPosition(Vector(10000000, 10000000, 10000000));
                rm = 0;
            }
        }
        //Clear the list
        transferList->clear();
    }

    //This function handles the transfer of a vehicle to a connecting
    //Highway (if there is one)
    bool Highway::HandleTransfer(Ptr<Vehicle> veh) {
        //Get the turning desire for the vehicle
        TurningType turningType = GetTurningType(veh->GetDestination());
        //Get the vehicles current lane
        Lane *currentLane = m_laneMap[veh->GetLane()];
        //Get the destination highway from the front (the default)
        Ptr<Highway> destHighway = m_frontHighways[veh->GetLane() - 1];
        //Get the offset
        int destOffset = m_frontOffsets[veh->GetLane() - 1];
        switch (turningType) {
            case LEFT:
            {
                //If we are turning left, check for a left highway
                if (m_leftHighways[veh->GetLane() - 1] != NULL) {
                    destHighway = m_leftHighways[veh->GetLane() - 1];
                    destOffset = m_leftOffsets[veh->GetLane() - 1];
                }
                break;
            }
            case RIGHT:
            {
                //If we are turning right, check for a right highway
                if (m_rightHighways[veh->GetLane() - 1] != NULL) {
                    destHighway = m_rightHighways[veh->GetLane() - 1];
                    destOffset = m_rightOffsets[veh->GetLane() - 1];
                }
                break;
            }
            default:
            {
                //Even if we are going straight, if the current lane does
                //not have a forward highway, try to use either a left or
                //right, if it is available
                if (destHighway == NULL) {
                    if (m_rightHighways[veh->GetLane() - 1] != NULL) {
                        destHighway = m_rightHighways[veh->GetLane() - 1];
                        destOffset = m_rightOffsets[veh->GetLane() - 1];
                    } else if (m_leftHighways[veh->GetLane() - 1] != NULL) {
                        destHighway = m_leftHighways[veh->GetLane() - 1];
                        destOffset = m_leftOffsets[veh->GetLane() - 1];
                    }
                }
                break;
            }
        }

        //If there is no destination highway, return false
        if (destHighway == NULL) {
            return false;
        } else {
            //First determine how far past the current lane the vehicle has travelled
            double distancePastEnd = directDistance(veh->GetPosition().x, veh->GetPosition().y, currentLane->endX, currentLane->endY);
            //Adjust the vehicles lane
            veh->SetLane(veh->GetLane() - destOffset);
            //Insert it into the connecting highway the necessary distance
            //Note that the vehicles position may be altered based on the direction
            //the destination is point (if it is differnt, the Vehicle will need
            //to be adjusted
            destHighway->AddVehicle(veh, distancePastEnd);
            return true;
        }

    }

    //This is the primary function for inserting vehicles into the lane
    void Highway::InsertIntoLane(Ptr<Vehicle> veh, Lane *destLane) {
        if(veh->GetDestination() == m_highwayId) {
            veh->ArriveAtDestination();
        }
        VehicleList::iterator it;
        veh->SetDirection(destLane->direction);
        //First we check to insure that the vehicle is actually in range of the
        //Highway
        if (!anglesEqual(angleToPoint(veh->GetPosition().x, veh->GetPosition().y, destLane->endX, destLane->endY), destLane->direction)) {
            //If it is not, try to transfer it to a connecting Highway
            if (!HandleTransfer(veh)) {
                //If we could not transfer it, remove it from simulation
                if (veh->IsEquipped == true) veh->GetReceiveCallback().Nullify();
                // to put vehicle's node far away from the highway
                // we cannot dispose the vehicle here because its node may still be involved in send and receive process
                veh->SetPosition(Vector(10000000, 10000000, 10000000));
                veh = 0;

            }
        } else {
            //We are in this highway, so find where this Vehicle goes by compare this vehicle
            //to others in the lane
            for (it = destLane->vehicles->begin(); it != destLane->vehicles->end(); it++) {
                //Break when we get to a Vehicle that this vehicle should be
                //in front of
                if (Vehicle::Compare(veh, (*it))) {
                    break;
                }
            }
            //If we are at the end, this vehicle is at the end of the lane
            if (it == destLane->vehicles->end()) {
                destLane->vehicles->push_back(veh);
            //Otherwise, we insert it where needed
            } else {
                destLane->vehicles->insert(it, veh);
            }
        }
    }

    //This function finds vehicles in front and behind a certain vehicle in a certain lane
    void Highway::FindSideVehicles(Ptr<Vehicle> veh, double currentDistance, int sideLane) {
        //Clear tempVehicles
        m_tempVehicles[0] = 0;
        m_tempVehicles[1] = 0;

        //First, we have to find the point in the side lane where this vehicle
        //would be if it change lanes
        double intersectionPoint[3];
        Lane *lanePtr = m_laneMap[sideLane];
        findIntersection(veh->GetDirection(), veh->GetPosition().x, veh->GetPosition().y, lanePtr->direction, lanePtr->startX, lanePtr->startY, intersectionPoint);

        //Get the destination lane
        VehicleList* sLane = lanePtr->vehicles;

        //Initialize the indexes of the vehicles to look for
        int backIndex = -1;
        int forwardIndex = -1;

        //For each vehicle in this lane
        for (uint i = 0; i < sLane->size(); i++) {
            Ptr<Vehicle> curVeh = GetVehicle(sideLane, i);
            //Ignore stop lights when determining lane changing
            if(curVeh->GetVehicleType() == 2) {
                continue;
            }
            //If this vehicle is in front of the point
            if (!anglesEqual(curVeh->GetDirection(), angleToPoint(curVeh->GetPosition().x, curVeh->GetPosition().y, intersectionPoint[1], intersectionPoint[2]))) {
                //set it as the forward index
                forwardIndex = i;
            } else {
                //Else, set it as the back index
                backIndex = i;
                //Set the forward indexn as the previous index
                forwardIndex = i - 1;
                break;
            }
        }

        //If we actually found a vehicle in this lane
        if (backIndex > -1) {
            //Set the vehicle and distance
            m_tempVehicles[1] = GetVehicle(sideLane, backIndex);
            m_tempDistances[1] = currentDistance - GetDistance(sideLane, backIndex, true);
        } else {
            //Search the connecting Highways
            m_tempVehicles[1] = GetVehicle(sideLane, sLane->size() + 1);
            m_tempDistances[1] = currentDistance + GetDistance(sideLane, sLane->size() + 1, true);
        }

        //If we actually found a vehicle in this lane
        if (forwardIndex > -1) {
            //Set the vehicle and distance
            m_tempVehicles[0] = GetVehicle(sideLane, forwardIndex);
            m_tempDistances[0] = GetDistance(sideLane, forwardIndex, true) - currentDistance;
        } else {
            //Search the connecting Highways
            m_tempVehicles[0] = GetVehicle(sideLane, -1);
            m_tempDistances[0] = GetDistance(sideLane, -1, false) + (m_highwayLength - currentDistance);
        }
    }

    //This function is deprecated.  Handling the stepping of the Highway
    //is the HighwayProject class
    void Highway::Start() {
        m_stopped = false;
        //InitHighway();
        Simulator::Schedule(Seconds(0.0), &Step, Ptr<Highway > (this));
    }

    //This function is deprecated.  Handling the stepping of the Highway
    //is the HighwayProject class
    void Highway::Stop() {
        m_stopped = true;
    }

    //Gets the location of a point in a lane at a certain distance from
    //either the start or end of the lane
    Vector Highway::GetPosition(bool start, double distance, int lane) {
        double result[2];
        double direction;
        Lane *theLane = m_laneMap[lane];
        if (start) {
            result[0] = theLane->startX;
            result[1] = theLane->startY;
            direction = theLane->direction;
        } else {
            result[0] = theLane->endX;
            result[1] = theLane->endY;
            direction = M_PI + theLane->direction;
        }
        translatePoint(result, direction, distance);
        return Vector3D(result[0], result[1], 0.0);
    }

    //Adds the vehicle with no changes to position
    void Highway::AddVehicle(Ptr<Vehicle> vehicle) {
        int lane = vehicle->GetLane();
        int minIndex = 1;
        int maxIndex = m_numberOfLanes;
        //If the lane is not viable, do not insert
        if (lane <= maxIndex && lane >= minIndex) {
            InsertIntoLane(vehicle, m_laneMap[lane]);
        }
    }

    //This adds teh vehicle and forces it to "snap" to the start
    //of the desired lane
    void Highway::AddVehicleToBeginning(Ptr<Vehicle> vehicle) {
        int lane = vehicle->GetLane();
        int minIndex = 1;
        int maxIndex = m_numberOfLanes;
        if (lane <= maxIndex && lane >= minIndex) {
            vehicle->SetPosition(Vector3D(m_laneMap[lane]->startX, m_laneMap[lane]->startY, 0.0));
            InsertIntoLane(vehicle, m_laneMap[lane]);
        }
    }

    //This add the vehicle and forces it to "snap" to a point in the
    //lane a certain distance from the start of the lane
    void Highway::AddVehicle(Ptr<Vehicle> vehicle, double distance) {
        int lane = vehicle->GetLane();
        int minIndex = 1;
        int maxIndex = m_numberOfLanes;
        if (lane <= maxIndex && lane >= minIndex) {
            vehicle->SetPosition(GetPosition(true, distance, lane));
            InsertIntoLane(vehicle, m_laneMap[lane]);
        }
    }

    //Searches the entire Highway for the vehicle ID provided
    Ptr<Vehicle> Highway::FindVehicle(int vid) {
        Ptr<Vehicle> v = 0;
        int minIndex = 1;
        int maxIndex = m_numberOfLanes;
        for (int i = minIndex; i <= maxIndex; i++) {
            if (i == 0) {
                continue;
            }
            VehicleList* vList = m_laneMap[i]->vehicles;
            for (uint j = 0; j < vList->size(); j++) {
                v = GetVehicleFromList(vList, j);
                if (v->GetVehicleId() == vid) {
                    return v;
                }
            }
        }
        return v;
    }

    //This function returns a list of Vehicles that are within a certain range of the
    //querying vehicle
    std::list<Ptr<Vehicle> > Highway::FindVehiclesInRange(Ptr<Vehicle> vehicle, double range) {
        std::list<Ptr<Vehicle> > neighbors;
        if (range <= 0) {
            return neighbors;
        }

        Ptr<Vehicle> v = 0;
        double diff = 0;
        Vector pos, p;
        p = vehicle->GetPosition();
        int minIndex = 1;
        int maxIndex = m_numberOfLanes;
        for (int i = minIndex; i < maxIndex; i++) {
            if (i == 0) {
                continue;
            }
            VehicleList* vList = m_laneMap[i]->vehicles;
            for (uint j = 0; j < vList->size(); j++) {
                v = GetVehicleFromList(vList, j);
                pos = v->GetPosition();
                if (v->GetVehicleId() == vehicle->GetVehicleId()) {
                    continue;
                }
                diff = sqrt(pow(pos.x - p.x, 2) + pow(pos.y - p.y, 2));
                if (diff < range) {
                    neighbors.push_back(v);
                }
            }
        }

        return neighbors;
    }

    std::list<Ptr<Vehicle> > Highway::FindVehiclesInSegment(double distanceInHighway, double segmentSize) {
        std::list<Ptr<Vehicle> > segment;


        //Ptr<Vehicle> v=0;
        //Vector pos;
        //if(dir==1 && lane< 5 && lane>=0)
        //  {
        //    for(uint j=0;j<m_vehicles[lane].size();j++)
        //      {
        //        v=GetVehicle(m_vehicles[lane],j);
        //        pos=v->GetPosition();
        //        if(pos.x >= x1 && pos.x < x2) segment.push_back(v);
        //      }
        //  }
        //else if(dir==-1 && m_twoDirectional==true && lane < 5 && lane >= 0)
        //  {
        //    for(uint j=0;j<m_vehiclesOpp[lane].size();j++)
        //      {
        //        v=GetVehicle(m_vehiclesOpp[lane],j);
        //        pos=v->GetPosition();
        //        if(pos.x >= x1 && pos.x < x2)
        //	  segment.push_back(v);
        //      }
        //  }
        //NOT CURRENTLY IMPLEMENTED
        return segment;
    }

    //This function simply returns the list of Vehicles for the supplied lane number
    std::list<Ptr<Vehicle> >* Highway::GetVehiclesInLane(int laneNumber) {
        if (laneNumber > 0 && laneNumber <= m_numberOfLanes) {
            return m_laneMap[laneNumber]->vehicles;
        } else {
            return NULL;
        }

    }

    Callback<bool, Ptr<Highway>, Ptr<Vehicle>, double> Highway::GetControlVehicleCallback() {
        return m_controlVehicle;
    }

    void Highway::SetControlVehicleCallback(Callback<bool, Ptr<Highway>, Ptr<Vehicle>, double> controlVehicle) {
        m_controlVehicle = controlVehicle;
    }

    void Highway::Step(Ptr<Highway> highway) {
        highway->TranslateVehicles();
    }

}
