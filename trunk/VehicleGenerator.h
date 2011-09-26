/* 
 * File:   VehicleGenerator.h
 * Author: bdupont
 *
 * Created on August 9, 2011, 8:06 PM
 */

#ifndef _VEHICLEGENERATOR_H
#define	_VEHICLEGENERATOR_H

#include "ns3/callback.h"
#include "ns3/ptr.h"
#include "ns3/object.h"
#include "ns3/random-variable.h"
#include "ns3/vector.h"
#include "Model.h"
#include "LaneChange.h"
#include "Highway.h"
#include "Vehicle.h"
#include <map>

namespace ns3 {

    class VehicleGenerator : public ns3::Object {
    private:
        Ptr<Highway> m_highway; // the highway this generator is feeding vehicles into
        double m_flow; // traffic flow veh/s in positive direction at entrance.
        double m_velocity; // traffic velocity in positive direction at entrance.
        double m_remainder; // remainder of vehicles.
        int m_currentLane;
        double m_penetrationRate;
        double m_dt;
        double m_minGap;
        int m_truckProability;
        std::map<double, int> m_destinationMap;
        RandomVariable m_RVFlow; // highway flow random variable in positive direction
        RandomVariable m_RVSpeed; // vehicles speed distribution
        WifiHelper m_wifiHelper; // a wifi helper apply to setup vehicles Wifi
        NqosWifiMacHelper m_wifiMacHelper; // a wifi mac helper apply to setup vehicles Wifi
        YansWifiPhyHelper m_wifiPhyHelper; // a wifi phy helper apply to setup vehicles Wifi

        /// For Catching an event when a packet is received by any vehicles in the Highway.
        VehicleReceiveCallback m_receiveData;
        /// For Catching DevTxTrace.
        DeviceTraceCallback m_devTxTrace;
        /// For Catching DevRxTrace.m_phyRxOkTrace
        DeviceTraceCallback m_devRxTrace;
        /// For Catching PhyRxOkTrace.
        PhyRxOkTraceCallback m_phyRxOkTrace;
        /// For Catching PhyRxErrorTrace.
        PhyRxErrorTraceCallback m_phyRxErrorTrace;
        /// For Catching PhyTxTrace.
        PhyTxTraceCallback m_phyTxTrace;
        /// For Catching PhyStateTrace.
        PhyStateTraceCallback m_phyStateTrace;

        void insertVehicles();

    public:
        /// Override TypeId.
        static TypeId GetTypeId(void);

        VehicleGenerator(Ptr<Highway> associatedHighway);
        ~VehicleGenerator();

        void init();

        void setFlow(double flow);
        double getFlow();

        void setVelocity(double velocity);
        double getVelocity();

        void setMinGap(double minGap);
        double getMinGap();

        void setTruckProbability(int truckProbability);
        int getTruckProbability();

        void setRVFlow(RandomVariable rvFlow);
        RandomVariable getRVFlow();

        void setRVSpeed(RandomVariable rvSpeed);
        RandomVariable getRVSpeed();

        void setWifiHelper(WifiHelper wifiHelper);
        WifiHelper getWifiHelper();

        void setNqosWifiMacHelper(NqosWifiMacHelper wifiMacHelper);
        NqosWifiMacHelper getNqosWifiMacHelper();

        void setYansWifiPhyHelper(YansWifiPhyHelper wifiPhyHelper);
        YansWifiPhyHelper getYansWifiPhyHelper();

        void setDestinationMap(std::map<double, int> destinationMap);
        std::map<double, int> getDestinationMap();

        /// Returns the Highway's Receive Data callback.
        VehicleReceiveCallback GetReceiveDataCallback();
        /// Sets the Highway's Receive Data callback.
        void SetReceiveDataCallback(VehicleReceiveCallback receiveData);
        /// Returns the Highway's DevTxTrace callback.
        DeviceTraceCallback GetDevTxTraceCallback();
        /// Sets the Highway's DevTxTrace callback.
        void SetDevTxTraceCallback(DeviceTraceCallback devTxTrace);
        /// Returns the Highway's DevRxTrace callback.
        DeviceTraceCallback GetDevRxTraceCallback();
        /// Sets the Highway's DevRxTrace callback.
        void SetDevRxTraceCallback(DeviceTraceCallback devRxTrace);
        /// Returns the Highway's PhyRxOkTrace callback.
        PhyRxOkTraceCallback GetPhyRxOkTraceCallback();
        /// Sets the Highway's PhyRxOkTrace callback.
        void SetPhyRxOkTraceCallback(PhyRxOkTraceCallback phyRxOkTrace);
        /// Returns the Highway's PhyRxErrorTrace callback.
        PhyRxErrorTraceCallback GetPhyRxErrorTraceCallback();
        /// Sets the Highway's PhyRxErrorTrace callback.
        void SetPhyRxErrorTraceCallback(PhyRxErrorTraceCallback phyRxErrorTrace);
        /// Returns the Highway's PhyTxTrace callbacl.
        PhyTxTraceCallback GetPhyTxTraceCallback();
        /// Sets the Highway's PhyTxTrace callback.
        void SetPhyTxTraceCallback(PhyTxTraceCallback phyTxTrace);
        /// Returns the Highway's PhyStateTrace callback.
        PhyStateTraceCallback GetPhyStateTraceCallback();
        /// Sets the Highway's PhyStateTrace callback.
        void SetPhyStateTraceCallback(PhyStateTraceCallback phyStateTrace);
        /// Returns the Highway Control Vehicle callback.
        Callback<bool, Ptr<Highway>, Ptr<Vehicle>, double> GetControlVehicleCallback();

        /**
         * Runs one mobility Step for the given highway.
         * This function is called each interval dt to simulated the mobility through TranslateVehicles().
         */
        static void Step(Ptr<VehicleGenerator> vehicleGenerator);

    };

}

#endif	/* _VEHICLEGENERATOR_H */

