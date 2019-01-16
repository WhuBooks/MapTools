//
// Created by 98420 on 2018/8/30.
//

#include "Link.h"
#include <cassert>

namespace maplib{
    namespace model{
    
        void Link::initialize()
        {
            m_info.type=calDirection();
        }
    
        LinkType Link::calDirection()
        {
            return LinkType::TurnStraight;
        }
    
        Lane &Link::operator[](int lane_id)
        {
            assert(!m_lanes.empty());
            for(Lane &lane : m_lanes)
                if(lane.m_info.id==lane_id)
                    return lane;
            std::cerr<<"Target LaneId is Out of Range"<<std::endl;
            return m_lanes[0];
        }
    
        std::vector<Link> Link::extractBySRoadId(const std::vector<Link> &links, int roadid)
        {
            std::vector<Link> result;
            for(const Link &link : links)
                if(link.m_info.s_road_id==roadid)
                    result.push_back(link);
            return result;
        }
    
        std::vector<Link> Link::extractByERoadId(const std::vector<Link> &links, int roadid)
        {
            std::vector<Link> result;
            for(const Link &link : links)
                if(link.m_info.e_road_id==roadid)
                    result.push_back(link);
            return result;
        }
    }
}