#include <iostream>
#include "VehicleGenerator.h"
#include "Vehicle.h"
#include "Model.h"
#include "LaneChange.h"
#include "Geometry.h"
#include "ns3/simulator.h"
#include "IdGenerator.h"

namespace ns3 {

    TypeId VehicleGenerator::GetTypeId(void) {
        static TypeId tid = TypeId("ns3::VehicleGenerator")
                .SetParent<Object > ()
                ;
        return tid;
    }

    Ptr<Model> CreateSedanModel() {
        Ptr<Model> model = CreateObject<Model > ();
        model->SetDesiredVelocity(11.176);
        model->SetDeltaV(4.0);
        model->SetAcceleration(0.5);
        model->SetDeceleration(3.0);
        model->SetMinimumGap(2.0);
        model->SetTimeHeadway(0.1);
        model->SetSqrtAccelerationDeceleration(sqrt(model->GetAcceleration() * model->GetDeceleration()));
        return model;
    }

    Ptr<LaneChange> CreateSedanLaneChangeModel() {
        Ptr<LaneChange> laneChange = CreateObject<LaneChange > ();
        laneChange->SetPolitenessFactor(0.2);
        laneChange->SetDbThreshold(0.3);
        laneChange->SetGapMin(2.0);
        laneChange->SetMaxSafeBreakingDeceleration(12.0);
        laneChange->SetBiasRight(0.2);
        return laneChange;
    }

    Ptr<Model> CreateTruckModel() {
        Ptr<Model> model = CreateObject<Model > ();
        model->SetDesiredVelocity(13.0);
        model->SetDeltaV(4.0);
        model->SetAcceleration(0.2);
        model->SetDeceleration(4.0);
        model->SetMinimumGap(2.0);
        model->SetTimeHeadway(0.5);
        model->SetSqrtAccelerationDeceleration(sqrt(model->GetAcceleration() * model->GetDeceleration()));
        return model;
    }

    Ptr<LaneChange> CreateTruckLaneChangeModel() {
        Ptr<LaneChange> laneChange = CreateObject<LaneChange > ();
        laneChange->SetPolitenessFactor(0.2);
        laneChange->SetDbThreshold(0.2);
        laneChange->SetGapMin(2.0);
        laneChange->SetMaxSafeBreakingDeceleration(12.0);
        laneChange->SetBiasRight(0.3);
        return laneChange;
    }

    VehicleGenerator::VehicleGenerator(Ptr<Highway> associatedHighway) {
        m_highway = associatedHighway;
        m_dt = m_highway->GetDeltaT();


        m_flow = 1;
        m_velocity = 11.176; // 25 MPH
        m_currentLane = 1;
        m_minGap = 33; //3 second rule for following at 11 m/s
        m_remainder = 0;
        m_penetrationRate = 100;

        m_RVFlow = UniformVariable(m_flow*m_dt, m_flow * m_dt);
        m_RVSpeed = UniformVariable(11.176, 11.176);

        m_destinationMap = std::map<double, int>();

        m_wifiHelper = WifiHelper::Default();
        m_wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211a);
        m_wifiMacHelper = NqosWifiMacHelper::Default();
        m_wifiPhyHelper = YansWifiPhyHelper::Default();
        YansWifiChannelHelper m_wifiChannelHelper = YansWifiChannelHelper::Default();
        m_wifiMacHelper.SetType("ns3::AdhocWifiMac");
        m_wifiHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue("OfdmRate6MbpsBW10MHz"));
        //m_wifiChannelHelper.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
        Ptr<YansWifiChannel> m_wifiChannel = m_wifiChannelHelper.Create();
        m_wifiPhyHelper.SetChannel(m_wifiChannel);
        m_wifiPhyHelper.Set("TxPowerStart", DoubleValue(21.5)); // 250-300 meter transmission range
        m_wifiPhyHelper.Set("TxPowerEnd", DoubleValue(21.5)); // 250-300 meter transmission range
        m_wifiPhyHelper.Set("TxPowerLevels", UintegerValue(1));
        m_wifiPhyHelper.Set("TxGain", DoubleValue(2));
        m_wifiPhyHelper.Set("RxGain", DoubleValue(2));
        m_wifiPhyHelper.Set("EnergyDetectionThreshold", DoubleValue(-101.0));
    }

    VehicleGenerator::~VehicleGenerator() {
    }

    void VehicleGenerator::init() {
        Simulator::Schedule(Seconds(m_dt), &VehicleGenerator::Step, Ptr<VehicleGenerator > (this));
    }

    void VehicleGenerator::insertVehicles() {
        map<double, int> modifiedDestMap;
        UniformVariable uRnd(0, 100);
        UniformVariable uRnd2(0, 100);
        UniformVariable uRnd3;
        double rate1 = m_RVFlow.GetValue();
        double gap;
        double vel = 0.0;


        bool isSedan = uRnd.GetValue() <= m_truckProability;
        int length = isSedan ? 4 : 8;

        Ptr<Vehicle> last = m_highway->GetFirstVehicle(m_currentLane);
        //last = m_vehicles[m_currentLaneDirPos].size()-1;
        if (last == NULL) {
            gap = m_minGap + 1;
            vel = m_velocity;
        } else {
            gap = m_highway->GetDistanceToFirstVehicle(m_currentLane) - length;
            vel = m_velocity; //GetVehicle(m_vehicles[m_currentLaneDirPos],last)->GetVelocity();
        }

        m_remainder += rate1;
        if (m_remainder >= 1 && gap > m_minGap) {
            m_remainder -= 1;
            Ptr<Vehicle> temp = CreateObject<Vehicle > ();
            if (uRnd2.GetValue() <= m_penetrationRate) {
                temp->IsEquipped = true;
                temp->SetupWifi(m_wifiHelper, m_wifiPhyHelper, m_wifiMacHelper);
            } else {
                temp->IsEquipped = false;
            }
            temp->SetVehicleId(IdGenerator::nextVehicleId());
            temp->SetLane(m_currentLane);
            temp->SetVelocity(vel);
            temp->SetAcceleration(0.0);
            if (temp->IsEquipped == true) {
                temp->SetReceiveCallback(m_receiveData);
                temp->SetDevTxTraceCallback(m_devTxTrace);
                temp->SetDevRxTraceCallback(m_devRxTrace);
                temp->SetPhyRxOkTraceCallback(m_phyRxOkTrace);
                temp->SetPhyRxErrorTraceCallback(m_phyRxErrorTrace);
                temp->SetPhyTxTraceCallback(m_phyTxTrace);
                temp->SetPhyStateTraceCallback(m_phyStateTrace);
            }
            if (isSedan) {
                Ptr<Model> tempModel = CreateSedanModel();
                tempModel->SetDesiredVelocity(m_RVSpeed.GetValue());
                temp->SetModel(tempModel);
                temp->SetLaneChange(CreateSedanLaneChangeModel());
                temp->SetLength(4);
                temp->SetWidth(2);
            } else {
                Ptr<Model> tempModel = CreateTruckModel();
                tempModel->SetDesiredVelocity(m_RVSpeed.GetValue());
                temp->SetModel(tempModel);
                temp->SetLaneChange(CreateTruckLaneChangeModel());
                temp->SetLength(8);
                temp->SetWidth(2);
            }

            int destination = -1;
            double destVal = uRnd3.GetValue();
            for(std::map<double, int>::iterator it = m_destinationMap.begin(); it != m_destinationMap.end(); it++) {
                if(destVal > it->first) {
                    destination = it->second;
                    break;
                }
            }
            temp->AddDestination(destination);

            m_highway->AddVehicleToBeginning(temp);
            m_currentLane++;
            if (m_currentLane > m_highway->GetNumberOfLanes()) {
                m_currentLane = 1;
            }
        }


        Simulator::Schedule(Seconds(m_dt), &VehicleGenerator::Step, Ptr<VehicleGenerator > (this));
    }

    void VehicleGenerator::setFlow(double flow) {
        m_flow = flow;
    }

    double VehicleGenerator::getFlow() {
        return m_flow;
    }

    void VehicleGenerator::setVelocity(double velocity) {
        m_velocity = velocity;
    }

    double VehicleGenerator::getVelocity() {
        return m_velocity;
    }

    void VehicleGenerator::setMinGap(double minGap) {
        m_minGap = minGap;
    }

    double VehicleGenerator::getMinGap() {
        return m_minGap;
    }

    void VehicleGenerator::setTruckProbability(int truckProbability) {
        m_truckProability = truckProbability;
    }

    int VehicleGenerator::getTruckProbability() {
        return m_truckProability;
    }

    void VehicleGenerator::setRVFlow(RandomVariable rvFlow) {
        m_RVFlow = rvFlow;
    }

    RandomVariable VehicleGenerator::getRVFlow() {
        return m_RVFlow;
    }

    void VehicleGenerator::setRVSpeed(RandomVariable rvSpeed) {
        m_RVSpeed = rvSpeed;
    }

    RandomVariable VehicleGenerator::getRVSpeed() {
        return m_RVSpeed;
    }

    void VehicleGenerator::setWifiHelper(WifiHelper wifiHelper) {
        m_wifiHelper = wifiHelper;
    }

    WifiHelper VehicleGenerator::getWifiHelper() {
        return m_wifiHelper;
    }

    void VehicleGenerator::setNqosWifiMacHelper(NqosWifiMacHelper wifiMacHelper) {
        m_wifiMacHelper = wifiMacHelper;
    }

    NqosWifiMacHelper VehicleGenerator::getNqosWifiMacHelper() {
        return m_wifiMacHelper;
    }

    void VehicleGenerator::setYansWifiPhyHelper(YansWifiPhyHelper wifiPhyHelper) {
        m_wifiPhyHelper = wifiPhyHelper;
    }

    YansWifiPhyHelper VehicleGenerator::getYansWifiPhyHelper() {
        return m_wifiPhyHelper;
    }

    void VehicleGenerator::setDestinationMap(std::map<double, int> destinationMap) {
        double sum = 0.0;
        for(std::map<double, int>::iterator it = destinationMap.begin(); it != destinationMap.end(); it++) {
            sum += it->first;
        }

        m_destinationMap.clear();

        for(std::map<double, int>::iterator it = destinationMap.begin(); it != destinationMap.end(); it++) {
            m_destinationMap[((it->first)/sum)] = it->second;
        }
    }

    std::map<double, int> VehicleGenerator::getDestinationMap() {
        return m_destinationMap;
    }

    VehicleReceiveCallback VehicleGenerator::GetReceiveDataCallback() {
        return m_receiveData;
    }

    void VehicleGenerator::SetReceiveDataCallback(VehicleReceiveCallback receiveData) {
        m_receiveData = receiveData;
    }

    DeviceTraceCallback VehicleGenerator::GetDevTxTraceCallback() {
        return m_devTxTrace;
    }

    void VehicleGenerator::SetDevTxTraceCallback(DeviceTraceCallback devTxTrace) {
        m_devTxTrace = devTxTrace;
    }

    DeviceTraceCallback VehicleGenerator::GetDevRxTraceCallback() {
        return m_devRxTrace;
    }

    void VehicleGenerator::SetDevRxTraceCallback(DeviceTraceCallback devRxTrace) {
        m_devRxTrace = devRxTrace;
    }

    PhyRxOkTraceCallback VehicleGenerator::GetPhyRxOkTraceCallback() {
        return m_phyRxOkTrace;
    }

    void VehicleGenerator::SetPhyRxOkTraceCallback(PhyRxOkTraceCallback phyRxOkTrace) {
        m_phyRxOkTrace = phyRxOkTrace;
    }

    PhyRxErrorTraceCallback VehicleGenerator::GetPhyRxErrorTraceCallback() {
        return m_phyRxErrorTrace;
    }

    void VehicleGenerator::SetPhyRxErrorTraceCallback(PhyRxErrorTraceCallback phyRxErrorTrace) {
        m_phyRxErrorTrace = phyRxErrorTrace;
    }

    PhyTxTraceCallback VehicleGenerator::GetPhyTxTraceCallback() {
        return m_phyTxTrace;
    }

    void VehicleGenerator::SetPhyTxTraceCallback(PhyTxTraceCallback phyTxTrace) {
        m_phyTxTrace = phyTxTrace;
    }

    PhyStateTraceCallback VehicleGenerator::GetPhyStateTraceCallback() {
        return m_phyStateTrace;
    }

    void VehicleGenerator::SetPhyStateTraceCallback(PhyStateTraceCallback phyStateTrace) {
        m_phyStateTrace = phyStateTrace;
    }

    void VehicleGenerator::Step(Ptr<VehicleGenerator> vehicleGenerator) {
        vehicleGenerator->insertVehicles();
    }
}