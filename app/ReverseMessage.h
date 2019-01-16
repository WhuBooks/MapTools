//
// Created by books on 2018/5/24.
//

#ifndef MAPTOOLS_REVERSELANE_H
#define MAPTOOLS_REVERSELANE_H

#include "Common.h"

void GetReverseMessage(const MapBase::RoadVec &vroad)
{
    if(g_match.reverseid==-1)
        return;

    DrawHandlerPtr draw_handler(new LcmHandler<ckLcmType::Draw_t>());
    draw_handler->SetNet("udpm://238.255.76.67:7667?ttl=64");
    draw_handler->SetChannel("CKREVERSEDRAW");
    draw_handler->InitialSend();

    ReverseHandlerPtr reverse_handler(new LcmHandler<ckLcmType::ReverseLane_t>());
    reverse_handler->SetNet("udpm://238.255.76.67:7667?ttl=64");
    reverse_handler->SetChannel("CKREVERSE");
    reverse_handler->InitialSend();

    LaneMsg laneMsg(0);
    for(const MapBase::Road &road : vroad)
    {
        if(road.roadid==g_match.reverseid)
        {
            if(!road.vlane.empty())
            {
                laneMsg=ExtractLaneMsg(road.vlane.front().vec,g_match.ReverseDir());
            }
        }
    }

    ckLcmType::ReverseLane_t reverseMsg;
    reverseMsg.pointnum=(int)laneMsg.size();
    reverseMsg.lane.assign(laneMsg.begin(),laneMsg.end());
    reverse_handler->SendLcm(&reverseMsg);

    ckLcmType::Draw_t draw_msg=LocalDrawMsg(laneMsg);
    draw_handler->SendLcm(&draw_msg);
}


#endif //MAPTOOLS_REVERSELANE_H
