//
// Created by 98420 on 2018/8/31.
//

#ifndef MAPTOOLS_BLOCK_H
#define MAPTOOLS_BLOCK_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <map>

#include "GnssPoint2D.h"
#include "Lane.h"

namespace maplib{
    namespace model
    {
        enum Type
        {
            Road=1,
            Link=2
        };
        enum Direction
        {
            TurnLeft = 0,
            TurnStraight = 1,
            TurnRight = 2,
            TurnBack = 3
        };
        struct BlockInfo
        {
            int id;
            Type type;
            Direction direction;
            std::unordered_set<int> s_blocks_ids;
            std::unordered_set<int> e_blocks_ids;
            std::vector<int> s_lane_ids;
            std::vector<int> e_lane_ids;
            double ave_length;
            
            bool hasBlockSID(int id)const {
                return s_blocks_ids.find(id)!=s_blocks_ids.end();
            }
            
            bool hasBlockEID(int id)const{
                return e_blocks_ids.find(id)!=e_blocks_ids.end();
            }
        };
        
        class GnssPoint2D;
        class Lane;
        
        class Block
        {
        public:
            Block()= default;
            ~Block()= default;
            
            void initialize();
    
            Lane &operator[](int lane_id);
    
            static std::vector<Block> extractBySBlockId(const std::vector<Block> &blocks,int sid);
    
            static std::vector<Block> extractByEBlockId(const std::vector<Block> &blocks,int eid);
            
            static Block extractBySEBlockId(const std::vector<Block> &blocks,int sid,int eid);
            
            static std::vector<GnssPoint2D> extractAllPts(const std::map<int,Block> &map);
            
            BlockInfo m_info;
            std::vector<Lane> m_lane_vec;
        private:
        
        };
        typedef std::map<int,Block> Map;
    }
}



#endif //MAPTOOLS_BLOCK_H
