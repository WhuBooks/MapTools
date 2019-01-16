//
// Created by 98420 on 2018/8/30.
//

#ifndef MAPTOOLS_ROAD_H
#define MAPTOOLS_ROAD_H

#include <iostream>
#include <vector>
#include <algorithm>

#include "GnssPoint2D.h"
#include "Lane.h"

namespace maplib
{
    namespace model
    {
        enum Type
        {
            Road=1,
            Link=2
        };
        enum Diretion
        {
            TurnLeft = 0,
            TurnStraight = 1,
            TurnRight = 2,
            TurnBack = 3
        };
        struct RoadInfo
        {
            int id;
            std::string name;
            double ave_length;
        };
        
        class Road
        {
        public:
            Road() = default;
            
            ~Road() = default;
            
            void initialize();
            
            Lane &operator[](int lane_id);
    
            RoadInfo m_info;
            LaneVec m_lanes;
        private:
        
        
        };
    }
}





#endif //MAPTOOLS_ROAD_H
