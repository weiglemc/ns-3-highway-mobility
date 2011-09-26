/* 
 * File:   HighwayProject.h
 * Author: bdupont
 *
 * Created on September 19, 2011, 8:31 PM
 */

#ifndef _HIGHWAYPROJECTXML_H
#define	_HIGHWAYPROJECTXML_H

#include <list>
#include <map>
#include <stdlib.h>
#include "tinyxml.h"
#include "HighwayXml.h"
#include "VehicleGeneratorXml.h"
#include "WifiConfigurationXml.h"
#include "TrafficLightGeneratorXml.h"

using namespace std;

class HighwayProjectXml {
private:
    int m_numberOfRuns;
    int m_totalTimeInSeconds;
    double m_dt;
    list<HighwayXml> m_highways;
    list<WifiConfigurationXml> m_wifiConfigs;
    list<VehicleGeneratorXml> m_vehicleGenerators;
    list<TrafficLightGeneratorXml> m_trafficLightGenerators;

public:

    list<HighwayXml> GetHighways() {
        return m_highways;
    }

    void SetHighways(list<HighwayXml> highways) {
        m_highways = highways;
    }

    int GetNumberOfRuns() {
        return m_numberOfRuns;
    }

    void SetNumberOfRuns(int numberOfRuns) {
        m_numberOfRuns = numberOfRuns;
    }

    int GetTotalTimeInSeconds() {
        return m_totalTimeInSeconds;
    }

    double GetDt() {
        return m_dt;
    }

    void SetDt(double dt) {
        m_dt = dt;
    }

    void SetTotalTimeInSeconds(int totalTimeInSeconds) {
        m_totalTimeInSeconds = totalTimeInSeconds;
    }

    list<VehicleGeneratorXml> GetVehicleGenerators() {
        return m_vehicleGenerators;
    }

    void SetVehicleGenerators(list<VehicleGeneratorXml> vehicleGenerators) {
        m_vehicleGenerators = vehicleGenerators;
    }

    list<WifiConfigurationXml> GetWifiConfigs() {
        return m_wifiConfigs;
    }

    void SetWifiConfigs(list<WifiConfigurationXml> wifiConfigs) {
        m_wifiConfigs = wifiConfigs;
    }

    list<TrafficLightGeneratorXml> GetTrafficLightGenerators() const {
        return m_trafficLightGenerators;
    }

    void SetTrafficLightGenerators(list<TrafficLightGeneratorXml> trafficLightGenerators) {
        this->m_trafficLightGenerators = trafficLightGenerators;
    }

    void LoadFromXml(TiXmlHandle root) {
        m_numberOfRuns = 1;
        m_totalTimeInSeconds = 30;
        m_dt = 0.1;
        m_highways = list<HighwayXml > ();
        m_wifiConfigs = list<WifiConfigurationXml > ();
        m_vehicleGenerators = list<VehicleGeneratorXml > ();
        m_trafficLightGenerators = list<TrafficLightGeneratorXml > ();

        TiXmlElement* rootPtr = root.Element();

        if (rootPtr != NULL) {
            rootPtr->QueryIntAttribute("numberOfRuns", &m_numberOfRuns);
            rootPtr->QueryIntAttribute("totalTimeInSeconds", &m_totalTimeInSeconds);
            rootPtr->QueryDoubleAttribute("dt", &m_dt);
            TiXmlHandle highwaysRoot = root.FirstChildElement("highways");
            TiXmlElement* highwaysRootPtr = highwaysRoot.Element();
            if (highwaysRootPtr) {
                TiXmlElement* highwayPtr = highwaysRootPtr->FirstChildElement("highway");
                for (; highwayPtr; highwayPtr = highwayPtr->NextSiblingElement()) {
                    HighwayXml newHighway;
                    TiXmlHandle highwayHandle = TiXmlHandle(highwayPtr);
                    newHighway.LoadFromXml(highwayHandle);
                    m_highways.push_back(newHighway);
                }
            }

            TiXmlHandle wifiConfigRoot = root.FirstChildElement("wifiConfigurations");
            TiXmlElement* wifiConfigRootPtr = wifiConfigRoot.Element();
            if (wifiConfigRootPtr) {
                TiXmlElement* wifiConfig = wifiConfigRootPtr->FirstChildElement("wifiConfiguration");
                for (; wifiConfig; wifiConfig = wifiConfig->NextSiblingElement()) {
                    WifiConfigurationXml newWifi;
                    TiXmlHandle wifiHandle = TiXmlHandle(wifiConfig);
                    newWifi.LoadFromXml(wifiHandle);
                    m_wifiConfigs.push_back(newWifi);
                }
            }

            TiXmlHandle generatorRoot = root.FirstChildElement("vehicleGenerators");
            TiXmlElement* generatorRootPtr = generatorRoot.Element();
            if (generatorRootPtr) {
                TiXmlElement* generator = generatorRoot.FirstChildElement("vehicleGenerator").Element();
                for (; generator; generator = generator->NextSiblingElement()) {
                    VehicleGeneratorXml newGenerator;
                    TiXmlHandle generatorHandle = TiXmlHandle(generator);
                    newGenerator.LoadFromXml(generatorHandle);
                    m_vehicleGenerators.push_back(newGenerator);
                }
            }

            TiXmlHandle trafficRoot = root.FirstChildElement("trafficLightGenerators");
            TiXmlElement* trafficRootPtr = trafficRoot.Element();
            if (trafficRootPtr) {
                TiXmlElement* traffic = trafficRoot.FirstChildElement("trafficGenerator").Element();
                for(; traffic; traffic = traffic->NextSiblingElement()) {
                    TiXmlHandle trafficHandle(traffic);
                    TrafficLightGeneratorXml tlGenerator;
                    tlGenerator.LoadFromXml(trafficHandle);
                    m_trafficLightGenerators.push_back(tlGenerator);
                }
            }
        }
    }

};

#endif	/* _HIGHWAYPROJECTXML_H */

