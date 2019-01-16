//
// Created by books on 2018/5/9.
//

#ifndef MAPTOOLS_ROADMESSGE_H
#define MAPTOOLS_ROADMESSGE_H

#include "Common.h"


void GetRoadMessage(const MapBase::RoadVec &vroad,const MapBase::ConnVec &vconn)
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

    bool findconn=false;
    MapBase::Connline conn;
    for(const MapBase::Connline &co : vconn)
    {
        if(co.connid==g_match.connid)
        {
            findconn=true;
            conn=co;
            break;
        }
    }

    if (!findroad)
    {
        SendEmptyMap();
        return;
    }

    RoadMsg ckroad;
    ConnStatus status=OutConn;
    int curIndex = -1;
    int targetIndex=-1;

    if(g_match.laneid!=-1)
    {
        int laneindex=0;
        /// find current index and target index
        for(const MapBase::Lane &la : road.vlane)
        {
            curIndex = (la.laneid == g_match.laneid) ? laneindex : curIndex;

            /// if find connline, find target index
            if (findconn)
                targetIndex=(la.laneid==conn.slaneid)?laneindex:targetIndex;
            
            laneindex++;
        }
        
        if(targetIndex==-1)
            targetIndex=curIndex;
        
        /// check OutConn or EnterConn
        int target_nearest_index=NearestIndex(road.vlane[targetIndex].vec,g_match);
        if(target_nearest_index+forward<road.vlane[targetIndex].vec.size())
        {
            for(const MapBase::Lane &la : road.vlane)
            {
                LaneMsg laneMsg=ExtractLaneMsg(la.vec,g_match);
                ckroad.push_back(laneMsg);
            }
            status=ConnStatus::OutConn;
        }
        else
        {
            MapBase::GnssVec vec=road.vlane[targetIndex].vec;
            if(findconn)
                vec.insert(vec.end(),conn.data.begin(),conn.data.end());
            LaneMsg laneMsg=ExtractLaneMsg(vec,g_match);
            ckroad.push_back(laneMsg);
            status=ConnStatus::EnterConn;
            targetIndex=0;
            curIndex=0;
        }
    }
    else if(findconn)
    {
        /// check InConn or LeaveConn
        int conn_nearest_index=NearestIndex(conn.data,g_match);
        if(conn_nearest_index+forward<conn.data.size())
        {
            LaneMsg laneMsg=ExtractLaneMsg(conn.data,g_match);
            ckroad.push_back(laneMsg);
            status=ConnStatus::InConn;
            targetIndex=0;
            curIndex=0;
        }
        else
        {
            MapBase::Lane nearestLane;
            for(const MapBase::Lane &la : road.vlane)
            {
                if(la.laneid==conn.elaneid)
                    nearestLane=la;
            }

            MapBase::GnssVec vec=conn.data;
            vec.insert(vec.end(),nearestLane.vec.begin(),nearestLane.vec.end());

            LaneMsg laneMsg=ExtractLaneMsg(vec,g_match);
            ckroad.push_back(laneMsg);
            status=ConnStatus::LeaveConn;
            targetIndex=0;
            curIndex=0;
        }
    }

    ckLcmType::Map_t mapMsg;
    mapMsg.curpos=Pos(g_match.x,g_match.y,g_match.yaw);
    mapMsg.lanenum=(int)ckroad.size();
    mapMsg.pointnum=backward+forward;
    mapMsg.curindex=curIndex;
    mapMsg.targetindex=targetIndex;
    mapMsg.inconn=status;
    mapMsg.road.assign(ckroad.begin(),ckroad.end());
    map_handler->SendLcm(&mapMsg);

    ckLcmType::Draw_t drawMsg=LocalDrawMsg(mapMsg);
    draw_map_handler->SendLcm(&drawMsg);
}



#endif //MAPTOOLS_ROADMESSGE_H
