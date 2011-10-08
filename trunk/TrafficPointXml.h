/* 
 * File:   TrafficPointXml.h
 * Author: bdupont
 *
 * Created on September 25, 2011, 7:11 PM
 */

#ifndef _TRAFFICPOINTXML_H
#define	_TRAFFICPOINTXML_H

#include "tinyxml.h"
#include <stdlib.h>
#include <string>
#include <iostream>

using namespace std;

class TrafficPointXml {
private:
    string m_side;
    int m_highwayId;
    double m_distance;
    list<int> leftTurnLanes;
public:

    double GetDistance() const {
        return m_distance;
    }

    void SetDistance(double distance) {
        this->m_distance = distance;
    }

    int GetHighwayId() const {
        return m_highwayId;
    }

    void SetHighwayId(int highwayId) {
        this->m_highwayId = highwayId;
    }

    string GetSide() const {
        return m_side;
    }

    void SetSide(string side) {
        this->m_side = side;
    }

    list<int> GetLeftTurnLanes() const {
        return leftTurnLanes;
    }

    void SetLeftTurnLanes(list<int> leftTurnLanes) {
        this->leftTurnLanes = leftTurnLanes;
    }

    void LoadFromXml(TiXmlHandle root) {

        TiXmlElement* rootPtr = root.Element();
        if (rootPtr) {
            rootPtr->QueryDoubleAttribute("distance", &m_distance);
            rootPtr->QueryIntAttribute("highwayId", &m_highwayId);
            m_side = rootPtr->Attribute("side");

            TiXmlHandle leftTurnHandle = root.FirstChildElement("leftTurnLane");
            for (TiXmlElement* leftTurnElement = leftTurnHandle.Element(); leftTurnElement; leftTurnElement = leftTurnElement->NextSiblingElement()) {
                leftTurnLanes.push_back(atoi(leftTurnElement->GetText()));
            }

        }
    }

};


#endif	/* _TRAFFICPOINTXML_H */

