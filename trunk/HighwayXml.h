/* 
 * File:   HighwayXml.h
 * Author: bdupont
 *
 * Created on September 19, 2011, 8:33 PM
 */

#ifndef _HIGHWAYXML_H
#define	_HIGHWAYXML_H

#include <iostream>
#include <list>
#include "tinyxml.h"
#include "HighwayConnectionXml.h"

using namespace std;

class HighwayXml {
private:
    int m_highwayId;
    int m_numberOfLanes;
    double m_direction;
    double m_length;
    double m_startX;
    double m_startY;
    double m_leftTurnSpeed;
    double m_rightTurnSpeed;
    double m_laneWidth;
    list<HighwayConnectionXml> m_frontHighways;
    list<HighwayConnectionXml> m_backHighways;
    list<HighwayConnectionXml> m_rightHighways;
    list<HighwayConnectionXml> m_leftHighways;

public:

    list<HighwayConnectionXml> GetBackHighways() {
        return m_backHighways;
    }

    void SetBackHighways(list<HighwayConnectionXml> backHighways) {
        m_backHighways = backHighways;
    }

    double GetDirection() {
        return m_direction;
    }

    void SetDirection(double direction) {
        m_direction = direction;
    }

    list<HighwayConnectionXml> GetFrontHighways() {
        return m_frontHighways;
    }

    void SetFrontHighways(list<HighwayConnectionXml> frontHighways) {
        m_frontHighways = frontHighways;
    }

    int GetHighwayId() {
        return m_highwayId;
    }

    void SetHighwayId(int highwayId) {
        m_highwayId = highwayId;
    }

    double GetLaneWidth() {
        return m_laneWidth;
    }

    void SetLaneWidth(double laneWidth) {
        m_laneWidth = laneWidth;
    }

    list<HighwayConnectionXml> GetLeftHighways() {
        return m_leftHighways;
    }

    void SetLeftHighways(list<HighwayConnectionXml> leftHighways) {
        m_leftHighways = leftHighways;
    }

    double GetLeftTurnSpeed() {
        return m_leftTurnSpeed;
    }

    void SetLeftTurnSpeed(double leftTurnSpeed) {
        m_leftTurnSpeed = leftTurnSpeed;
    }

    double GetLength() {
        return m_length;
    }

    void SetLength(double length) {
        m_length = length;
    }

    int GetNumberOfLanes() {
        return m_numberOfLanes;
    }

    void SetNumberOfLanes(int numberOfLanes) {
        m_numberOfLanes = numberOfLanes;
    }

    list<HighwayConnectionXml> GetRightHighways() {
        return m_rightHighways;
    }

    void SetRightHighways(list<HighwayConnectionXml> rightHighways) {
        m_rightHighways = rightHighways;
    }

    double GetRightTurnSpeed() {
        return m_rightTurnSpeed;
    }

    void SetRightTurnSpeed(double rightTurnSpeed) {
        m_rightTurnSpeed = rightTurnSpeed;
    }

    double GetStartX() {
        return m_startX;
    }

    void SetStartX(double startX) {
        m_startX = startX;
    }

    double GetStartY() {
        return m_startY;
    }

    void SetStartY(double startY) {
        m_startY = startY;
    }

    void LoadFromXml(TiXmlHandle root) {
        m_highwayId = -100000;
        m_numberOfLanes = 1;
        m_direction = 0.0;
        m_length = 1000;
        m_startX = 0.0;
        m_startY = 0.0;
        m_leftTurnSpeed = 2.2352;
        m_rightTurnSpeed = 2.2352;
        m_laneWidth = 5.0;
        m_frontHighways = list<HighwayConnectionXml>();
        m_backHighways = list<HighwayConnectionXml>();
        m_rightHighways = list<HighwayConnectionXml>();
        m_leftHighways = list<HighwayConnectionXml>();

        TiXmlElement* highwayRootPtr = root.Element();
        highwayRootPtr->QueryIntAttribute("highwayId", &m_highwayId);
        highwayRootPtr->QueryIntAttribute("numberOfLanes", &m_numberOfLanes);
        highwayRootPtr->QueryDoubleAttribute("direction", &m_direction);
        highwayRootPtr->QueryDoubleAttribute("length", &m_length);
        highwayRootPtr->QueryDoubleAttribute("startX", &m_startX);
        highwayRootPtr->QueryDoubleAttribute("startY", &m_startY);
        highwayRootPtr->QueryDoubleAttribute("leftTurnSpeed", &m_leftTurnSpeed);
        highwayRootPtr->QueryDoubleAttribute("rightTurnSpeed", &m_rightTurnSpeed);
        highwayRootPtr->QueryDoubleAttribute("laneWidth", &m_laneWidth);

        TiXmlHandle frontHighwaysRoot = root.FirstChildElement("frontHighways");
        TiXmlElement* frontHighwaysRootPtr = frontHighwaysRoot.Element();
        if(frontHighwaysRootPtr) {
            TiXmlHandle connectionChild = TiXmlHandle(frontHighwaysRootPtr->FirstChildElement());
            TiXmlElement* connectionChildPtr = connectionChild.Element();
            for(;connectionChildPtr;connectionChildPtr = connectionChildPtr->NextSiblingElement()) {
                TiXmlHandle connectionHandle = TiXmlHandle(connectionChildPtr);
                HighwayConnectionXml connection;
                connection.LoadFromXml(connectionHandle);
                m_frontHighways.push_back(connection);
            }
        }

        TiXmlHandle backHighwaysRoot = root.FirstChildElement("backHighways");
        TiXmlElement* backHighwaysRootPtr = backHighwaysRoot.Element();
        if(backHighwaysRootPtr) {
            TiXmlHandle connectionChild = TiXmlHandle(backHighwaysRootPtr->FirstChildElement());
            TiXmlElement* connectionChildPtr = connectionChild.Element();
            for(;connectionChildPtr;connectionChildPtr = connectionChildPtr->NextSiblingElement()) {
                TiXmlHandle connectionHandle = TiXmlHandle(connectionChildPtr);
                HighwayConnectionXml connection;
                connection.LoadFromXml(connectionHandle);
                m_backHighways.push_back(connection);
            }
        }

        TiXmlHandle rightHighwaysRoot = root.FirstChildElement("rightHighways");
        TiXmlElement* rightHighwaysRootPtr = rightHighwaysRoot.Element();
        if(rightHighwaysRootPtr) {
            TiXmlHandle connectionChild = TiXmlHandle(rightHighwaysRootPtr->FirstChildElement());
            TiXmlElement* connectionChildPtr = connectionChild.Element();
            for(;connectionChildPtr;connectionChildPtr = connectionChildPtr->NextSiblingElement()) {
                TiXmlHandle connectionHandle = TiXmlHandle(connectionChildPtr);
                HighwayConnectionXml connection;
                connection.LoadFromXml(connectionHandle);
                m_rightHighways.push_back(connection);
            }
        }

        TiXmlHandle leftHighwaysRoot = root.FirstChildElement("leftHighways");
        TiXmlElement* leftHighwaysRootPtr = leftHighwaysRoot.Element();
        if(leftHighwaysRootPtr) {
            TiXmlHandle connectionChild = TiXmlHandle(leftHighwaysRootPtr->FirstChildElement());
            TiXmlElement* connectionChildPtr = connectionChild.Element();
            for(;connectionChildPtr;connectionChildPtr = connectionChildPtr->NextSiblingElement()) {
                TiXmlHandle connectionHandle = TiXmlHandle(connectionChildPtr);
                HighwayConnectionXml connection;
                connection.LoadFromXml(connectionHandle);
                m_leftHighways.push_back(connection);
            }
        }

    }

};


#endif	/* _HIGHWAYXML_H */

