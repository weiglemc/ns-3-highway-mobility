/* 
 * File:   Intersection.h
 * Author: bdupont
 *
 * Created on August 18, 2011, 7:44 PM
 */

#ifndef _INTERSECTION_H
#define	_INTERSECTION_H

#include "ns3/ptr.h"
#include "ns3/object.h"
#include "Highway.h"


namespace ns3 {
    
    class Intersection : public ns3::Object {
    public:
        static TypeId GetTypeId(void);
    };
    
}

#endif	/* _INTERSECTION_H */

