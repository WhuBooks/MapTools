//
// Created by 98420 on 2018/8/30.
//

#include "Road.h"
#include <cassert>

namespace maplib
{
    namespace model
    {
        void Road::initialize()
        {
            assert(!m_lanes.empty());
            
            std::sort(m_lanes.begin(),m_lanes.end());
            double ave_length=0.0;
            for(const Lane &lane : m_lanes)
                ave_length+=lane.m_info.length;
            m_info.ave_length=ave_length/(double)m_lanes.size();
            
        }
    
        Lane &Road::operator[](int lane_id)
        {
            assert(!m_lanes.empty());
            for(Lane &lane : m_lanes)
                if(lane.m_info.id==lane_id)
                    return lane;
            std::cerr<<"Target LaneId is Out of Range"<<std::endl;
            return m_lanes[0];
        }
    }
}

