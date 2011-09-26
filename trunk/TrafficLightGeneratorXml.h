/* 
 * File:   TrafficLightGeneratorXml.h
 * Author: bdupont
 *
 * Created on September 25, 2011, 7:10 PM
 */

#ifndef _TRAFFICLIGHTGENERATORXML_H
#define	_TRAFFICLIGHTGENERATORXML_H

#include "tinyxml.h"
#include "TrafficPointXml.h"
#include <list>

class TrafficLightGeneratorXml {
private:
    list<TrafficPointXml> m_trafficPoints;
    double m_timeLeft;
    double m_timeStraight;
    double m_timeBuffer;
public:
    double GetTimeBuffer() const {
        return m_timeBuffer;
    }

    void SetTimeBuffer(double timeBuffer) {
        this->m_timeBuffer = timeBuffer;
    }

    double GetTimeLeft() const {
        return m_timeLeft;
    }

    void SetTimeLeft(double timeLeft) {
        this->m_timeLeft = timeLeft;
    }

    double GetTimeStraight() const {
        return m_timeStraight;
    }

    void SetTimeStraight(double timeStraight) {
        this->m_timeStraight = timeStraight;
    }

    list<TrafficPointXml> GetTrafficPoints() {
        return m_trafficPoints;
    }

    void SetTrafficPoints(list<TrafficPointXml> trafficPoints) {
        m_trafficPoints = trafficPoints;
    }

    void LoadFromXml(TiXmlHandle root) {
        m_timeLeft = 20.0;
        m_timeStraight = 40.0;
        m_timeBuffer = 5.0;
        m_trafficPoints = list<TrafficPointXml>();

        TiXmlElement* rootPtr = root.Element();
        if(rootPtr) {
            rootPtr->QueryDoubleAttribute("timeStraight", &m_timeStraight);
            rootPtr->QueryDoubleAttribute("timeLeft", &m_timeLeft);
            rootPtr->QueryDoubleAttribute("timeBuffer", &m_timeBuffer);

            TiXmlHandle trafficPointHandle = root.FirstChildElement("trafficPoint");
            for(TiXmlElement* tpEle = trafficPointHandle.Element(); tpEle; tpEle = tpEle->NextSiblingElement()) {
                TrafficPointXml tp;
                TiXmlHandle tpHandle(tpEle);
                tp.LoadFromXml(tpHandle);
                m_trafficPoints.push_back(tp);
            }
        }
    }

};


#endif	/* _TRAFFICLIGHTGENERATORXML_H */

