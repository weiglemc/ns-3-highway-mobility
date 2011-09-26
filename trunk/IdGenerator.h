/* 
 * File:   IdGenerator.h
 * Author: bdupont
 *
 * Created on August 10, 2011, 7:03 PM
 */

#ifndef _IDGENERATOR_H
#define	_IDGENERATOR_H

class IdGenerator {
private:
    static int vehicleId;
    static int highwayId;

public:
    static int nextHighwayId() {
        return highwayId++;
    }

    static int nextVehicleId() {
        return vehicleId++;
    }


};

#endif	/* _IDGENERATOR_H */

