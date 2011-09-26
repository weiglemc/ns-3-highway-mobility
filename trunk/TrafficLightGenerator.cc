#include "TrafficLightGenerator.h"
#include "IdGenerator.h"
#include "Vehicle.h"
#include "Obstacle.h"
#include "ns3/simulator.h"
#include <map>

namespace ns3 {

    TrafficLightGenerator::TrafficLightGenerator() {
        m_highwayMap = std::map<Side, TrafficPoint*>();
        m_highwayMap[LEFT] = NULL;
        m_highwayMap[RIGHT] = NULL;
        m_highwayMap[TOP] = NULL;
        m_highwayMap[BOTTOM] = NULL;
        m_timeLeft = 20.0;
        m_timeStraight = 30.0;
        m_timeBuffer = 5.0;
    }

    void TrafficLightGenerator::SetTimeStraight(double timeInSeconds) {
        m_timeStraight = timeInSeconds;
    }

    double TrafficLightGenerator::GetTimeStraight() {
        return m_timeStraight;
    }

    void TrafficLightGenerator::SetTimeLeft(double timeInSeconds) {
        m_timeLeft = timeInSeconds;
    }

    double TrafficLightGenerator::GetTimeLeft() {
        return m_timeLeft;
    }

    void TrafficLightGenerator::SetTimeBuffer(double timeInSeconds) {
        m_timeBuffer = timeInSeconds;
    }

    double TrafficLightGenerator::GetTimeBuffer() {
        return m_timeBuffer;
    }

    void TrafficLightGenerator::SetHighway(Side theSide, Ptr<Highway> highway, double distanceInHighway, std::list<int> leftTurnLanes) {
        TrafficPoint* newPoint = new TrafficPoint;
        newPoint->highway = highway;
        newPoint->distance = distanceInHighway;
        newPoint->leftTurnLanes = leftTurnLanes;

        newPoint->leftTurnLights = std::map<int, Ptr<Vehicle> >();
        newPoint->straightLights = std::map<int, Ptr<Vehicle> >();

        for (int i = 1; i <= highway->GetNumberOfLanes(); i++) {
            bool isLeft = false;
            for (std::list<int>::iterator it = leftTurnLanes.begin(); it != leftTurnLanes.end(); it++) {
                if (i == (*it)) {
                    isLeft = true;
                    break;
                }
            }
            Ptr<Vehicle> obstacle = CreateObject<Obstacle > ();
            obstacle->SetVehicleId(IdGenerator::nextVehicleId());
            obstacle->SetLane(i);
            obstacle->SetVehicleType(2);

            if (isLeft) {
                newPoint->leftTurnLights[i] = obstacle;
            } else {
                newPoint->straightLights[i] = obstacle;
            }
        }

        m_highwayMap[theSide] = newPoint;
    }

    void TrafficLightGenerator::Start() {
        
        for (std::map<Side, TrafficPoint*>::iterator it = m_highwayMap.begin(); it != m_highwayMap.end(); it++) {
            ChangeLights(it->first, true, true);
            ChangeLights(it->first, false, true);
            if(it->first == LEFT || it->first == RIGHT) {
                m_hasLR = true;
            }
            if(it->first == TOP || it->first == BOTTOM) {
                m_hasTB = true;
            }
        }

        if(m_hasLR) {
            m_currentState = LEFTTURN_LR;
        } else if (m_hasTB) {
            m_currentState = LEFTTURN_TB;
        }

        if(m_hasLR || m_hasTB) {
            Simulator::Schedule(Seconds(0.0), &Step, this);
        }

    }

    void TrafficLightGenerator::ChangeLights(Side side, bool leftTurnLane, bool toRed) {
        if(m_highwayMap.find(side) == m_highwayMap.end()) {
            return;
        }
        TrafficPoint* tp = m_highwayMap[side];

        if (toRed) {
            if (leftTurnLane) {
                for (std::list<int>::iterator it = tp->leftTurnLanes.begin(); it != tp->leftTurnLanes.end(); it++) {
                    Ptr<Vehicle> veh = tp->leftTurnLights[(*it)];
                    tp->highway->AddVehicle(veh, tp->distance);
                }
            } else {
                for (int i = 1; i <= tp->highway->GetNumberOfLanes(); i++) {
                    bool isLeft = false;
                    for (std::list<int>::iterator it = tp->leftTurnLanes.begin(); it != tp->leftTurnLanes.end(); it++) {
                        if (i == (*it)) {
                            isLeft = true;
                            break;
                        }
                    }
                    if(isLeft) {
                        continue;
                    }
                    Ptr<Vehicle> veh = tp->straightLights[i];
                    tp->highway->AddVehicle(veh, tp->distance);
                }
            }
        } else {
            if (leftTurnLane) {
                for(std::map<int, Ptr<Vehicle> >::iterator it = tp->leftTurnLights.begin(); it != tp->leftTurnLights.end(); it++) {
                    tp->highway->RemoveVehicle(it->second);
                }
            } else {
                for(std::map<int, Ptr<Vehicle> >::iterator it = tp->straightLights.begin(); it != tp->straightLights.end(); it++) {
                    tp->highway->RemoveVehicle(it->second);
                }
            }
        }

    }

    void TrafficLightGenerator::Step(TrafficLightGenerator* ptr) {
        double timeToSleep = 0.0;
        switch(ptr->m_currentState) {
            case LEFTTURN_LR: {
                ptr->ChangeLights(LEFT, true, false);
                ptr->ChangeLights(RIGHT, true, false);
                ptr->m_currentState = WAIT_LR;
                timeToSleep = ptr->m_timeLeft;
                break;
            }
            case WAIT_LR: {
                ptr->ChangeLights(LEFT, true, true);
                ptr->ChangeLights(RIGHT, true, true);
                ptr->m_currentState = STRAIGHT_LR;
                timeToSleep = ptr->m_timeBuffer;
                break;
            }
            case STRAIGHT_LR: {
                ptr->ChangeLights(LEFT, false, false);
                ptr->ChangeLights(RIGHT, false, false);
                ptr->m_currentState = WAITALL_LR;
                timeToSleep = ptr->m_timeStraight;
                break;
            }
            case WAITALL_LR: {
                ptr->ChangeLights(LEFT, false, true);
                ptr->ChangeLights(RIGHT, false, true);
                if(ptr->m_hasTB) {
                    ptr->m_currentState = LEFTTURN_TB;
                } else {
                    ptr->m_currentState = LEFTTURN_LR;
                }
                timeToSleep = ptr->m_timeBuffer;
                break;
            }
            case LEFTTURN_TB: {
                ptr->ChangeLights(TOP, true, false);
                ptr->ChangeLights(BOTTOM, true, false);
                ptr->m_currentState = WAIT_TB;
                timeToSleep = ptr->m_timeLeft;
                break;
            }
            case WAIT_TB: {
                ptr->ChangeLights(TOP, true, true);
                ptr->ChangeLights(BOTTOM, true, true);
                ptr->m_currentState = STRAIGHT_TB;
                timeToSleep = ptr->m_timeBuffer;
                break;
            }
            case STRAIGHT_TB: {
                ptr->ChangeLights(TOP, false, false);
                ptr->ChangeLights(BOTTOM, false, false);
                ptr->m_currentState = WAITALL_TB;
                timeToSleep = ptr->m_timeStraight;
                break;
            }
            case WAITALL_TB: {
                ptr->ChangeLights(TOP, false, true);
                ptr->ChangeLights(BOTTOM, false, true);
                if(ptr->m_hasLR) {
                    ptr->m_currentState = LEFTTURN_LR;
                } else {
                    ptr->m_currentState = LEFTTURN_TB;
                }
                timeToSleep = ptr->m_timeBuffer;
                break;
            }
        }
        Simulator::Schedule(Seconds(timeToSleep), &Step, ptr);
    }

}

