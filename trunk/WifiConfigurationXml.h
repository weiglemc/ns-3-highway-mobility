/* 
 * File:   WifiConfigurationXml.h
 * Author: bdupont
 *
 * Created on September 20, 2011, 10:26 AM
 */

#ifndef _WIFICONFIGURATIONXML_H
#define	_WIFICONFIGURATIONXML_H

#include "tinyxml.h"
#include "ns3/simulator.h"
#include <string>

using namespace std;
using namespace ns3;

class WifiConfigurationXml {
private:
    int m_wifiConfigId;
    string m_wifiStandard;
    string m_dataMode;
    double m_txPowerStart;
    double m_txPowerEnd;
    int m_txPowerLevels;
    double m_txGain;
    double m_rxGain;
    double m_energyDetectionThreshold;

    WifiHelper wifiHelper;
    NqosWifiMacHelper wifiMacHelper;
    YansWifiPhyHelper wifiPhyHelper;

    void CreateFromAttribs() {
        wifiHelper = WifiHelper::Default();

        if ("WIFI_PHY_STANDARD_80211a" == m_wifiStandard) {
            wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211a);
        } else if ("WIFI_PHY_STANDARD_80211b" == m_wifiStandard) {
            wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211b);
        } else if ("WIFI_PHY_STANDARD_80211_10Mhz" == m_wifiStandard) {
            wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211_10Mhz);
        } else if ("WIFI_PHY_STANDARD_80211_10Mhz" == m_wifiStandard) {
            wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211_10Mhz);
        } else if ("WIFI_PHY_STANDARD_80211_5Mhz" == m_wifiStandard) {
            wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211_5Mhz);
        } else if ("WIFI_PHY_STANDARD_holland" == m_wifiStandard) {
            wifiHelper.SetStandard(WIFI_PHY_STANDARD_holland);
        } else if ("WIFI_PHY_STANDARD_80211p_CCH" == m_wifiStandard) {
            wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211p_CCH);
        } else if ("WIFI_PHY_STANDARD_80211p_SCH" == m_wifiStandard) {
            wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211p_SCH);
        } else {
            wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211a);
        }

        wifiMacHelper = NqosWifiMacHelper::Default();
        wifiPhyHelper = YansWifiPhyHelper::Default();
        YansWifiChannelHelper wifiChannelHelper = YansWifiChannelHelper::Default();
        wifiMacHelper.SetType("ns3::AdhocWifiMac");
        wifiHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(m_dataMode));
        //m_wifiChannelHelper.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
        Ptr<YansWifiChannel> wifiChannel = wifiChannelHelper.Create();
        wifiPhyHelper.SetChannel(wifiChannel);
        wifiPhyHelper.Set("TxPowerStart", DoubleValue(m_txPowerStart)); // 250-300 meter transmission range
        wifiPhyHelper.Set("TxPowerEnd", DoubleValue(m_txPowerEnd)); // 250-300 meter transmission range
        wifiPhyHelper.Set("TxPowerLevels", UintegerValue(m_txPowerLevels));
        wifiPhyHelper.Set("TxGain", DoubleValue(m_txGain));
        wifiPhyHelper.Set("RxGain", DoubleValue(m_rxGain));
        wifiPhyHelper.Set("EnergyDetectionThreshold", DoubleValue(m_energyDetectionThreshold));

    }


public:

    WifiHelper GetWifiHelper() {
        return wifiHelper;
    }

    NqosWifiMacHelper GetWifiMacHelper() {
        return wifiMacHelper;
    }

    YansWifiPhyHelper GetWifiPhyHelper() {
        return wifiPhyHelper;
    }

    string GetDataMode() {
        return m_dataMode;
    }

    void SetDataMode(string dataMode) {
        m_dataMode = dataMode;
    }

    double GetEnergyDetectionThreshold() {
        return m_energyDetectionThreshold;
    }

    void SetEnergyDetectionThreshold(double energyDetectionThreshold) {
        m_energyDetectionThreshold = energyDetectionThreshold;
    }

    double GetRxGain() {
        return m_rxGain;
    }

    void SetRxGain(double rxGain) {
        m_rxGain = rxGain;
    }

    double GetTxGain() {
        return m_txGain;
    }

    void SetTxGain(double txGain) {
        m_txGain = txGain;
    }

    double GetTxPowerEnd() {
        return m_txPowerEnd;
    }

    void SetTxPowerEnd(double txPowerEnd) {
        m_txPowerEnd = txPowerEnd;
    }

    int GetTxPowerLevels() {
        return m_txPowerLevels;
    }

    void SetTxPowerLevels(int txPowerLevels) {
        m_txPowerLevels = txPowerLevels;
    }

    double GetTxPowerStart() {
        return m_txPowerStart;
    }

    void SetTxPowerStart(double txPowerStart) {
        m_txPowerStart = txPowerStart;
    }

    int GetWifiConfigId() {
        return m_wifiConfigId;
    }

    void SetWifiConfigId(int wifiConfigId) {
        m_wifiConfigId = wifiConfigId;
    }

    string GetWifiStandard() {
        return m_wifiStandard;
    }

    void SetWifiStandard(string wifiStandard) {
        m_wifiStandard = wifiStandard;
    }

    void LoadDefaults() {
        m_wifiStandard = "WIFI_PHY_STANDARD_80211a";
        m_dataMode = "OfdmRate6MbpsBW10MHz";
        m_txPowerStart = 21.5;
        m_txPowerEnd = 21.5;
        m_txPowerLevels = 1;
        m_txGain = 2.0;
        m_rxGain = 2.0;
        m_energyDetectionThreshold = -101.0;

        CreateFromAttribs();
    }

    void LoadFromXml(TiXmlHandle wifiConfigRoot) {
        m_wifiStandard = "WIFI_PHY_STANDARD_80211a";
        m_dataMode = "OfdmRate6MbpsBW10MHz";
        m_txPowerStart = 21.5;
        m_txPowerEnd = 21.5;
        m_txPowerLevels = 1;
        m_txGain = 2.0;
        m_rxGain = 2.0;
        m_energyDetectionThreshold = -101.0;

        TiXmlElement* wifiConfigRootPtr = wifiConfigRoot.Element();
        if(wifiConfigRootPtr) {
            m_wifiStandard = string(wifiConfigRootPtr->Attribute("wifiStandard"));
            m_dataMode = string(wifiConfigRootPtr->Attribute("dataMode"));
            wifiConfigRootPtr->QueryDoubleAttribute("txPowerStart", &m_txPowerStart);
            wifiConfigRootPtr->QueryDoubleAttribute("txPowerEnd", &m_txPowerEnd);
            wifiConfigRootPtr->QueryIntAttribute("txPowerLevels", &m_txPowerLevels);
            wifiConfigRootPtr->QueryDoubleAttribute("txGain", &m_txGain);
            wifiConfigRootPtr->QueryDoubleAttribute("rxGain", &m_rxGain);
            wifiConfigRootPtr->QueryDoubleAttribute("energyDetectionThreshold", &m_energyDetectionThreshold);
        }

        CreateFromAttribs();
    }

};


#endif	/* _WIFICONFIGURATIONXML_H */

