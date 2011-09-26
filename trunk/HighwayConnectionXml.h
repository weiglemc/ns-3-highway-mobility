/* 
 * File:   HighwayConnectionXml.h
 * Author: bdupont
 *
 * Created on September 19, 2011, 8:35 PM
 */

#ifndef _HIGHWAYCONNECTIONXML_H
#define	_HIGHWAYCONNECTIONXML_H

#include "tinyxml.h"

using namespace std;

class HighwayConnectionXml {
private:
    int m_highwayId;
    int m_laneOffset;
    int m_offset;
    
public:
    
    int GetHighwayId() {
        return m_highwayId;
    }

    void SetHighwayId(int highwayId) {
        m_highwayId = highwayId;
    }

    int GetLaneOffset() {
        return m_laneOffset;
    }

    void SetLaneOffset(int laneOffset) {
        m_laneOffset = laneOffset;
    }

    int GetOffset() {
        return m_offset;
    }

    void SetOffset(int offset) {
        m_offset = offset;
    }

    void LoadFromXml(TiXmlHandle root) {

        m_highwayId = -10000;
        m_laneOffset = 0;
        m_offset = 0;

        TiXmlElement* rootPtr = root.Element();
        if(rootPtr) {
            rootPtr->QueryIntAttribute("highwayId", &m_highwayId);
            rootPtr->QueryIntAttribute("laneOffset", &m_laneOffset);
            rootPtr->QueryIntAttribute("offset", &m_offset);
        }

    }
    
};

#endif	/* _HIGHWAYCONNECTIONXML_H */

