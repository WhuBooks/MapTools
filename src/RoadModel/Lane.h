//
// Created by 98420 on 2018/8/30.
//

#ifndef MAPTOOLS_LANE_H
#define MAPTOOLS_LANE_H

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>

#include "GnssPoint2D.h"
#include "Block.h"

namespace maplib
{
    namespace model
    {
        struct LaneInfo
        {
            int id;
            double width;
            double length;
            double density;
            LaneInfo():id(-1),width(3.0),length(-1.0),density(-1.0) {}
        };
        
        class GnssPoint2D;
        class Block;
        
        class Lane
        {
        public:
            Lane():root(nullptr),m_info(LaneInfo()){};
            
            ~Lane() = default;
            
            void initialize();
            
            bool operator<(const Lane &other) const
            {
                return this->leftNum(other) > other.leftNum(*this);
            }
            
            bool operator==(const Lane &other) const
            {
                return this->leftNum(other) == other.leftNum(*this);
            }
            
            bool operator>(const Lane &other) const
            {
                return this->leftNum(other) < other.leftNum(*this);
            }
            
            double calLength()const;
            double calDensity()const;
            
            LaneInfo m_info;
            std::vector<GnssPoint2D> m_data;
            std::shared_ptr<Block> root;
        private:
            /// return the numbers of the point on other's left side
            int leftNum(const Lane &other) const;
            
        };
        typedef std::vector<Lane> LaneVec;
    }
}

#endif //MAPTOOLS_LANE_H
