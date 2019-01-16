//
// Created by books on 2018/5/15.
//

#ifndef MAPTOOLS_LANEMESSAGE_H
#define MAPTOOLS_LANEMESSAGE_H

#include "Common.h"
#include <LcmType/ckCarLight.hpp>

/// Change Lane Light Function and Parameters
const unsigned char LOW_BEAM = 1 << 0;
const unsigned char HIGH_BEAM = 1 << 1;
const unsigned char POS_LIGHT = 1 << 2;
const unsigned char LEFT_TURN_LIGHT = 1 << 3;
const unsigned char RIGHT_TURN_LIGHT = 1 << 4;
const unsigned char HAZARD_WARNING_LIGHT = LEFT_TURN_LIGHT | RIGHT_TURN_LIGHT;
const unsigned char BRAKE_LIGHT = 1 << 5;

bool lightOn(unsigned char light, unsigned char curLight) {
    return (curLight&light) ==light;
}

enum ChangeLane
{
    CHANGE_NONE=0,
    CHANGE_LEFT=1,
    CHANGE_RIGHT=2
};
ChangeLane g_change_lane=CHANGE_NONE;
int g_change_lane_id=-1;
std::mutex g_change_lane_mutex;

void GetLaneMessage(const MapBase::RoadVec &vroad)
{
    //find target by roadid or connid
    bool findroad = false;
    MapBase::Road road;
    for (const MapBase::Road &ro : vroad)
    {
        if (ro.roadid == g_match.roadid)
        {
            findroad = true;
            road = ro;
            break;
        }
    }

    if (!findroad)
    {
        SendEmptyMap();
        return;
    }

    RoadMsg ckroad;

    //find path in each lane
    int laneIndex = 0;
    int curIndex = -1;
    for (const MapBase::Lane &la : road.vlane)
    {
        curIndex=(la.laneid==g_match.laneid)?laneIndex:curIndex;
        laneIndex++;

        LaneMsg laneMsg=ExtractLaneMsg(la.vec,g_match);
        ckroad.push_back(laneMsg);
    }


    int targetIndex=curIndex;
    {
        /// set g_change_laneid if g_change_lane_id=-1
        if(g_change_lane_id==-1)
        {
            std::unique_lock<std::mutex> lock_change_lane(g_change_lane_mutex);
            ///only change once before light signal return to none
            if (g_change_lane == CHANGE_LEFT)
            {
                targetIndex = (curIndex > 0) ? curIndex - 1 : curIndex;
                g_change_lane_id=targetIndex;
            }
            else if (g_change_lane == CHANGE_RIGHT)
            {
                targetIndex = (curIndex < (int)road.vlane.size() - 1) ? curIndex + 1 : curIndex;
                g_change_lane_id=targetIndex;
            }
            else
                targetIndex = curIndex;

            lock_change_lane.unlock();
        }
        else
            targetIndex=g_change_lane_id;
    }

    ckLcmType::Map_t mapMsg;
    mapMsg.curpos=Pos(g_match.x,g_match.y,g_match.yaw);
    mapMsg.curindex=curIndex;
    mapMsg.targetindex=targetIndex;
    mapMsg.inconn=OutConn;
    mapMsg.lanenum=(int)ckroad.size();
    mapMsg.pointnum=backward+forward;
    mapMsg.road.assign(ckroad.begin(),ckroad.end());
    map_handler->SendLcm(&mapMsg);

    ckLcmType::Draw_t draw_msg=LocalDrawMsg(mapMsg);
    draw_map_handler->SendLcm(&draw_msg);
}



#endif //MAPTOOLS_LANEMESSAGE_H
