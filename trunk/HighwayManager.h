/* 
 * File:   HighwayManager.h
 * Author: bdupont
 *
 * Created on August 21, 2011, 7:44 AM
 */

#ifndef _HIGHWAYMANAGER_H
#define	_HIGHWAYMANAGER_H

#include "Highway.h"
#include <map>

namespace ns3 {

    class HighwayManager : public ns3::Object {
    private:
        std::map<int, Ptr<Highway> > highwayMap;

    public:
        void addHighway(Ptr<Highway> highway);

        void generateRoutes();

    };

}

#endif	/* _HIGHWAYMANAGER_H */

