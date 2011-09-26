/* 
 * File:   VehicleGeneratorXml.h
 * Author: bdupont
 *
 * Created on September 19, 2011, 8:39 PM
 */

#ifndef _VEHICLEGENERATORXML_H
#define	_VEHICLEGENERATORXML_H

#include "tinyxml.h"

using namespace std;

class VehicleGeneratorXml {
private:
    int m_highwayId;
    int m_wifiConfigId;
    double m_flow;
    double m_velocity;
    double m_minGap;
    double m_penetrationRate;
    map<double, int> m_destinationMap;
public:

    double GetFlow() {
        return m_flow;
    }

    void SetFlow(double flow) {
        m_flow = flow;
    }

    int GetHighwayId() {
        return m_highwayId;
    }

    void SetHighwayId(int highwayId) {
        m_highwayId = highwayId;
    }

    double GetMinGap() {
        return m_minGap;
    }

    void SetMinGap(double minGap) {
        m_minGap = minGap;
    }

    double GetPenetrationRate() {
        return m_penetrationRate;
    }

    void SetPenetrationRate(double penetrationRate) {
        m_penetrationRate = penetrationRate;
    }

    double GetVelocity() {
        return m_velocity;
    }

    void SetVelocity(double velocity) {
        m_velocity = velocity;
    }

    int GetWifiConfigId() {
        return m_wifiConfigId;
    }

    void SetWifiConfigId(int wifiConfigId) {
        m_wifiConfigId = wifiConfigId;
    }

    map<double, int> GetDestinationMap() {
        return m_destinationMap;
    }

    void SetDestinationMap(map<double, int> destinationMap) {
        m_destinationMap = destinationMap;
    }

    void LoadFromXml(TiXmlHandle root) {
        m_highwayId = -100000;
        m_wifiConfigId = -10000;
        m_flow = 1.0;
        m_velocity = 11.176;
        m_minGap = 33.0;
        m_penetrationRate = 100.0;
        m_destinationMap = map<double, int>();

        TiXmlElement* rootPtr = root.Element();
        if (rootPtr) {
            rootPtr->QueryIntAttribute("highwayId", &m_highwayId);
            rootPtr->QueryIntAttribute("wifiConfigId", &m_wifiConfigId);
            rootPtr->QueryDoubleAttribute("flow", &m_flow);
            rootPtr->QueryDoubleAttribute("velocity", &m_velocity);
            rootPtr->QueryDoubleAttribute("minGap", &m_minGap);
            rootPtr->QueryDoubleAttribute("penetrationRate", &m_penetrationRate);

            TiXmlHandle destHandle = root.FirstChildElement("destination");

            double weight = 0.0;
            int dest = -1;
            for(TiXmlElement* destPtr = destHandle.Element(); destPtr; destPtr = destPtr->NextSiblingElement()) {
                destPtr->QueryDoubleAttribute("weight", &weight);
                destPtr->QueryIntAttribute("destinationId", &dest);
                m_destinationMap[weight] = dest;
            }

        }
    }

};

#endif	/* _VEHICLEGENERATORXML_H */

