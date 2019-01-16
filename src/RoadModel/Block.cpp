//
// Created by 98420 on 2018/8/31.
//

#include "Block.h"

#include <cassert>
#include <cmath>

namespace maplib{
    namespace model
    {
        void Block::initialize()
        {
        
        }
    
        Lane &Block::operator[](int lane_id)
        {
            assert(!m_lane_vec.empty());
            for(Lane &lane : m_lane_vec)
                if(lane.m_info.id==lane_id)
                    return lane;
            std::cerr<<"Target LaneId is Out of Range"<<std::endl;
            return m_lane_vec[0];
        }
    
        std::vector<Block> Block::extractBySBlockId(const std::vector<Block> &blocks, int sid)
        {
            std::vector<Block> result;
            for(const Block &block : blocks)
                if(block.m_info.hasBlockSID(sid))
                    result.push_back(block);
            return result;
        }
    
        std::vector<Block> Block::extractByEBlockId(const std::vector<Block> &blocks, int eid)
        {
            std::vector<Block> result;
            for(const Block &block : blocks)
                if(block.m_info.hasBlockEID(eid))
                    result.push_back(block);
            return result;
        }
    
        Block Block::extractBySEBlockId(const std::vector<Block> &blocks, int sid, int eid)
        {
            for(const Block &block : blocks)
                if(block.m_info.hasBlockSID(sid)&&block.m_info.hasBlockEID(eid))
                    return block;
            return Block();
        }
    
        std::vector<GnssPoint2D> Block::extractAllPts(const std::map<int, Block> &map)
        {
            std::vector<GnssPoint2D> result;
            for (const std::pair<int, Block> &pair:map)
            {
                for(const Lane &lane : pair.second.m_lane_vec)
                    result.insert(result.end(),lane.m_data.begin(),lane.m_data.end());
            }
            return result;
        }
    }
}