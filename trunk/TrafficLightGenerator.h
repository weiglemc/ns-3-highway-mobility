/* 
 * File:   TrafficLightGenerator.h
 * Author: bdupont
 *
 * Created on September 22, 2011, 11:03 AM
 */

#ifndef _TRAFFICLIGHTGENERATOR_H
#define	_TRAFFICLIGHTGENERATOR_H

#include "ns3/ptr.h"
#include "ns3/object.h"
#include "Highway.h"
#include <list>
#include <map>

namespace ns3 {

    class TrafficLightGenerator : public ns3::Object {
    public:

        enum Side {
            LEFT,
            RIGHT,
            TOP,
            BOTTOM
        };

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

        struct TrafficPoint {
            Ptr<Highway> highway;
            double distance;
            std::list<int> leftTurnLanes;
            std::map<int, Ptr<Vehicle> > leftTurnLights;
            std::map<int, Ptr<Vehicle> > straightLights;
        };

    private:

        std::map<Side, TrafficPoint* > m_highwayMap;

        LightState m_currentState;
        bool m_hasLR;
        bool m_hasTB;
        double m_timeStraight;
        double m_timeLeft;
        double m_timeBuffer;

        void ChangeLights(Side side, bool leftTurnLane, bool toRed);

    public:

        TrafficLightGenerator();

        void SetHighway(Side theSide, Ptr<Highway> highway, double distanceInHighway, std::list<int> leftTurnLanes);

        void SetTimeStraight(double timeInSeconds);
        double GetTimeStraight();

        void SetTimeLeft(double timeInSeconds);
        double GetTimeLeft();

        void SetTimeBuffer(double timeInSeconds);
        double GetTimeBuffer();

        void Start();

        static void Step(TrafficLightGenerator* ptr);
    };

}

#endif	/* _TRAFFICLIGHTGENERATOR_H */

