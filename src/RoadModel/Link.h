//
// Created by 98420 on 2018/8/30.
//

#ifndef MAPTOOLS_LINK_H
#define MAPTOOLS_LINK_H

#include <iostream>
#include <vector>
#include <algorithm>

#include "Lane.h"

namespace maplib
{
    namespace model
    {
        enum LinkType
        {
            TurnLeft = 0,
            TurnStraight = 1,
            TurnRight = 2,
            TurnBack = 3
        };
        
        struct LinkInfo
        {
            int id;
            int s_road_id;
            int e_road_id;
            double ave_length;
            LinkType type;
        };

        class Link
        {
        public:
            Link() = default;
            
            ~Link() = default;
            
            void initialize();
    
            LinkType calDirection();
            
            Lane &operator[](int lane_id);
            
            static std::vector<Link> extractBySRoadId(const std::vector<Link> &links,int roadid);
            
            static std::vector<Link> extractByERoadId(const std::vector<Link> &links,int roadid);
            
            LinkInfo m_info;
            LaneVec m_lanes;
        private:

        
        
        };
        typedef std::vector<Link> LinkVec;
        
    }
}




#endif //MAPTOOLS_LINK_H
