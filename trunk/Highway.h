/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005-2009 Old Dominion University [ARBABI]
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Hadi Arbabi <marbabi@cs.odu.edu>
 *         Bradley Dupont <bradley.dupont@cs.odu.edu>
 */

#ifndef CLASS_HIGHWAY_
#define CLASS_HIGHWAY_

#include "ns3/callback.h"
#include "ns3/ptr.h"
#include "ns3/object.h"
#include "ns3/random-variable.h"
#include "ns3/vector.h"
#include "Vehicle.h"
#include "Model.h"
#include "LaneChange.h"
#include <list>
#include <map>

namespace ns3 {

    /**
     * \brief Highway is a place holder of the Vehicle (s) which manages each step of the Vehicle mobility.
     *
     * At each step (interval dt), Highway  browse vehicles of each lane in order of their poistions. Highway moves
     * each Vehicle or does the change of lane based on the required information given by each vehicle and its
     * adjacent vehicles following IDM Model and LaneChange rules. It is possible to
     * add vehicles to the highway at anytime after initialization is complete (including during simulation).
     * Also, Highway raises a ControlVehicle event at each passed interval dt to give possibility to access/control Vehicles of the Highway.
     *
     */
    class Highway : public ns3::Object {
    public:

        enum TurningType {
            STRAIGHT,
            LEFT,
            RIGHT
        };

    private:
        //A short-hand for a list of Ptr<Vehicle>
        typedef std::list<Ptr<Vehicle> > VehicleList;

        //A Lane contains information on its own direction, its start
        //and end points, and the vehicles currently in the lane.
        //Vehicles with a lower index in the list are closer to the
        //end point.
        struct Lane {
            VehicleList* vehicles;
            int id;
            double startX;
            double endX;
            double startY;
            double endY;
            double direction;
        };

        //A short-hand for a map of Lane*
        typedef std::map<int, Lane* > LaneMap;

        //The LaneMap contains a map of all the available lanes.  Each lane
        //has an id in the range [1,m_numberOfLanes] inclusive on both ends.
        //1 is the "leftmost" lane, m_numberOfLanes is the "rightmost" lane.
        LaneMap m_laneMap;

        //This list keeps track, at each step, of Vehicles that have reached
        //the end of the Highway and are ready for transferring.
        std::list<Ptr<Vehicle> >* transferList;

        int m_highwayId;  //The ID of this higwya
        int m_numberOfLanes; // number of lanes for each direction.
        double m_highwayLength; // the length of the highway.
        double m_direction; // the direction the highway is pointing (radians off +X axis)
        double m_xPos; //the x position of the start of the highway
        double m_yPos; //the y position of the start of the highway
        double m_laneWidth; // the width of each lane in the roadway.
        double m_sedanTruckPerc; // the percantage of sedans against trucks when autoinjection is used.
        double m_dt; // the mobility step interval. (duraion between each step)
        double m_leftTurnSpeed; // the speed to set a vehicle to before it turns left
        double m_rightTurnSpeed; // the speed to set a vehicle to before it turns right
        bool m_stopped; // true, if the highway manager is stopped.
        bool m_changeLaneSet; // true, if we desire the vehicles be able to change their lanes with IDM/MOBIL conditions.
        int m_loop; // counts the current loop

        Ptr<Vehicle> m_tempVehicles[2]; // temp vehicles.
        double m_tempDistances[2]; // distances to temp vehicles

        //These arrays store connecting highways in the range
        //[0,m_numberOfLanes).  Each index is referenced by the
        //ID (laneId-1).  If the highway at that point is NULL, there
        //is no connecting Highway in that lane.
        Ptr<Highway>* m_frontHighways;
        int* m_frontOffsets;
        Ptr<Highway>* m_backHighways;
        int* m_backOffsets;
        Ptr<Highway>* m_leftHighways;
        int* m_leftOffsets;
        Ptr<Highway>* m_rightHighways;
        int* m_rightOffsets;
        std::map<int, TurningType> m_routingMap;


        /// Clears all lane data
        void ClearLanes();
        /// Initializes lanes
        void InitLanes();
        /// Translates the Vehicles to the new position.
        void TranslateVehicles();
        /// Calculates the position and velocity of each vehicle for the passed step and the next step.
        void TranslatePositionVelocity(double dt);
        /// Calculates the acceleration of the vehicles in passed step and for the next step.
        void Accelerate(double dt);
        /// Gets the turning desire for a particular vehicle
        TurningType GetTurningType(int highwayId);
        /// Changes the vehicle lanes if possible.
        void ChangeLane();
        /// Changes the vehicle lanes from current lanes to the destination lane.
        void DoChangeLaneIfPossible(int curLane, int desLane);
        /// Find the Vehicles on the Side of the current vehicle veh.
        void FindSideVehicles(Ptr<Vehicle> veh, double currentDistance, int sideLane);
        /// Returns an iterator for the insertion point of the vehicle
        void InsertIntoLane(Ptr<Vehicle> veh, Lane *destLane);
        /// Returns the distance in a lane, to a vehicle at index either from
        /// the front or the back
        double GetDistance(int laneNumber, int index, bool toFront);
        /// Either transfer veh to a connecting Highway or disables its WIFI and removes it
        bool HandleTransfer(Ptr<Vehicle> veh);
        /// Gets a vehicle from a list at a certain index
        Ptr<Vehicle> GetVehicleFromList(VehicleList* vList, int index);

        /**
         * An event called for each step of mobility for each Vehicle inside the Highway.
         * It gives the Highway, the Vehicle, and value of dt.
         * If we return true, it means the Cehicle at this step is being controlled manually by the user.
         * If we return false, the Vehicle mobility (acceleration) is ruled by the car following conditions.
         * this callback must point to the function which handles such event.
         */
        Callback<bool, Ptr<Highway>, Ptr<Vehicle>, double> m_controlVehicle;

    public:

        /// Override TypeId.
        static TypeId GetTypeId(void);
        /**
         * Setting the default values:
         * dt=1.0 , VehicleId=1, Number of Lanes=1, Highway Length=1000(m), Width of Lanes=5(m)
         */
        Highway();
        /// Destructor to clean up the lists.
        ~Highway();

        /// Sets the HighwayId
        void SetHighwayId(int highwayId);
        /// Returns the HighwayId
        int GetHighwayId();

        /// This routing map is used to determine in what direction a
        /// Vehicle will need to turn in order to get to the highwayID
        /// supplied
        void SetRoutingMap(std::map<int, TurningType> routingMap);

        /**
         * Starts the highway.
         */
        void Start();
        /**
         * Stops the highway. Therefore no vehicles mobility after.
         */
        void Stop();
        /// Initializes the Highway
        void InitHighway();

        /// Handle the transfers of all Vehicles in m_transferList
        void HandleTransfers();

        /// Sets the direction for the Highway in radians off the +X axis
        void SetDirection(double direction);
        /// Gets the direction for the Highway in radians off the +X axis
        double GetDirection();
        /**
         * \returns the Number of Lanes in the Highway.
         */
        int GetNumberOfLanes();
        /**
         * \param value the Number of Lanes the highway can have for each direction [min=0, max=5].
         */
        void SetNumberOfLanes(int value);
        /**
         * \returns the Length of the Highway.
         */
        double GetHighwayLength();
        /**
         * \param value the Length of the Highway.
         */
        void SetHighwayLength(double value);
        /**
         * \returns the Width of the Lanes.
         */
        double GetLaneWidth();
        /**
         * \param value the Width of the Lanes.
         */
        void SetLaneWidth(double value);

        /// Gets the X point of the start of the Highway
        double GetXPos();
        /// Sets the X point of the start of the Highway
        void SetXPos(double xPos);

        /// Gets the Y point of the start of the Highway
        double GetYPos();
        /// Sets the Y point of the start of the Highway
        void SetYPos(double yPos);

        // These speeds are used to slow the vehicle down to the supplied
        // speed by the time the vehicle is ready to make the turn.  This
        // is done with a linear decrease over the course of the length
        // of the Highway
        double GetRightTurnSpeed();
        void SetRightTurnSpeed(double rightTurnSpeed);
        double GetLeftTurnSpeed();
        void SetLeftTurnSpeed(double leftTurnSpeed);

        /// Get the x-y position in a Highway based on distance from start/end
        Vector GetPosition(bool start, double distance, int lane);
        Vector GetLaneStart(int lane);
        /**
         * \returns true if changing lanes in Highway is on, false if off.
         */
        bool GetChangeLane();
        /**
         * \param value true to turn the changing lanes on, false for off.
         */
        void SetChangeLane(bool value);
        /**
         * \returns the value of interval dt, the duration of each mobility step. A interval between each steps.
         */
        double GetDeltaT(void);
        /**
         * \param value the interval dt, the duration of each mobility step. A interval between each steps.
         */
        void SetDeltaT(double value);

        // Sets either the front, back, left, and right connecting highways.
        // The laneOffset is the first lane in this highway that connects to the
        // first lane in the second high (starts from the left).
        // The second offset is used to determine the lane number in the connecting
        // highway by subtracting the offset from the current lane number.
        void AddFrontHighway(Ptr<Highway> frontHigway, int laneOffset, int frontOffset);
        void AddBackHighway(Ptr<Highway> backHighway, int laneOffset, int backOffset);
        void AddRightHighway(Ptr<Highway> rightHighway, int laneOffset, int rightOffset);
        void AddLeftHighway(Ptr<Highway> leftHighway, int laneOffset, int leftOffset);

        //Searches all connected highways for the specified highwayId.  Returns
        //Straight, Left, or Right depending on where it is connected
        TurningType GetTurningTypeForHighwayId(int destHighwayId);

        /**
         * it will add the vehicle in to the Highway based on the vehicle lane and direction to the appropriate Highway list.
         */
        void AddVehicle(Ptr<Vehicle> vehicle);
        /**
         * This adds the vehicle and sets position as well as direction to compensate for slight variations between highways/intersections
         */
        void AddVehicleToBeginning(Ptr<Vehicle> vehicle);
        /**
         * This adds a vehicle in the lane specified at the distance specified
         */
        void AddVehicle(Ptr<Vehicle> vehicle, double distance);
        /**
         * Remove a vehicle from the highway
         * */
        void RemoveVehicle(int vehicleId);
        /**
         * Remove a vehicle from the highway
         * */
        void RemoveVehicle(Ptr<Vehicle> vehicle);
        /**
         * \returns the retrieved Vehicle at the specific Index from the list of highway vehicles.
         */
        Ptr<Vehicle> GetVehicle(int laneNumber, int index);
        /**
         * \returns the Vehicle that is closest to the beginning of the lane
         */
        Ptr<Vehicle> GetFirstVehicle(int lane);
        /**
         * \returns the distance to the Vehicle that is closest to the beginning of the lane
         */
        double GetDistanceToFirstVehicle(int lane);
        /**
         * \returns the Vehicle that is closest to the end of the lane
         */
        Ptr<Vehicle> GetLastVehicle(int lane);
        /**
         * \returns the distance to the Vehicle that is closest to the end of the lane
         */
        double GetDistanceToLastVehicle(int lane);
        /**
         * \returns the Vehicle from the Highway given its VehicleId (vid).
         */
        Ptr<Vehicle> FindVehicle(int vid);
        /**
         * \returns the list of vehicles within the specific Range from a desired Vehicle in the Highway.
         */
        std::list<Ptr<Vehicle> > FindVehiclesInRange(Ptr<Vehicle> vehicle, double range);
        /**
         * \returns the list of vehicles for each Lane and Direction in a specific segment of the Highway from x1 to x2.
         */
        std::list<Ptr<Vehicle> > FindVehiclesInSegment(double distanceInHighway, double segmentSize);

        std::list<Ptr<Vehicle> >* GetVehiclesInLane(int laneNumber);

        /// Returns the Highway Control Vehicle callback.
        Callback<bool, Ptr<Highway>, Ptr<Vehicle>, double> GetControlVehicleCallback();
        /// Sets the Highway Control Vehicle callback.
        void SetControlVehicleCallback(Callback<bool, Ptr<Highway>, Ptr<Vehicle>, double> controlVehicle);
        /**
         * Runs one mobility Step for the given highway.
         * This function is called each interval dt to simulated the mobility through TranslateVehicles().
         */
        static void Step(Ptr<Highway> highway);
    };
};
#endif
