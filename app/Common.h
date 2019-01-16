//
// Created by books on 2018/5/15.
//

#ifndef MAPTOOLS_GLOBAL_H
#define MAPTOOLS_GLOBAL_H

#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <random>
#include <algorithm>
#include <chrono>
#include <functional>

#include <Geometry.h>
#include <Grid.h>
#include <RoadModel.h>
#include <Graph.h>
#include <FileIO.h>
#include <Cluster.h>
#include <DouglasPeucker.h>
#include <Log.h>

#include <LcmType/LcmHandler.h>
#include <LcmType/Draw_t.hpp>
#include <LcmType/Location.hpp>
#include <LcmType/GPSData.hpp>
#include <LcmType/MapPt.hpp>
#include <LcmType/Block.hpp>
#include <LcmType/Map.hpp>

double move_left_val=0.0;

MapBase::PointXYA cur_pos;
MapBase::RoadPoint g_match;
MapBase::Mission g_mission;

bool g_need_send_map=true;
int g_update_map=5;
int g_update_times=0;
bool g_come_target=true;

static const int forward=50;
static const int backward=10;

enum ConnStatus
{
    OutConn=0,
    EnterConn=1,
    LeaveConn=2,
    InConn=3
};
ConnStatus g_conn_status=OutConn;

typedef nox_lcm::GPSData GpsMag;
typedef nox_lcm::Location LocMsg;
typedef nox_lcm::MapPt PtMsg;
typedef nox_lcm::Block BlockMsg;
typedef std::vector<BlockMsg> BlocksMsg;
typedef nox_lcm::Map MapMsg;
typedef ckLcmType::Draw_t DrawMsg;

typedef LcmHandler<nox_lcm::GPSData>::Ptr GpsHandlerPtr;
typedef LcmHandler<nox_lcm::Location>::Ptr LocHandlerPtr;
typedef LcmHandler<nox_lcm::Map>::Ptr MapHandlerPtr;
typedef LcmHandler<ckLcmType::Draw_t>::Ptr DrawHandlerPtr;

MapHandlerPtr map_handler(new LcmHandler<nox_lcm::Map>);
DrawHandlerPtr draw_map_handler(new LcmHandler<ckLcmType::Draw_t>);

int NearestIndex(const MapBase::GnssVec &vec,const MapBase::PointXYA &pos)
{
    int index=0;
    double min_dis=10000000000;
    for(std::size_t i=0;i<vec.size();i++)
    {
        double tmp_dis=pos.Distance(vec[i]);
        if(tmp_dis<min_dis)
        {
            min_dis=tmp_dis;
            index=i;
        }
    }
    return index;
}

PtMsg ExtractPos(MapBase::GnssData gd)
{
    PtMsg msg;
    
    MapBase::PointXYA local(-move_left_val,0.0,gd.yaw);
    MapBase::PointXYA world=local.ToWorld(gd);
    gd.x=world.x;
    gd.y=world.y;
    
    /// convert Gauss to Cartesian
    msg.x=gd.y;
    msg.y=gd.x;
    msg.yaw=MapBase::Degree(360.0-gd.yaw);
    msg.velocity=std::sqrt(gd.up*gd.up+gd.north*gd.north+gd.east*gd.east);
    return msg;
}

void ResortBlock(MapBase::GnssVec &vec)
{
    if(vec.size()<=1)
        return;
    
    MapBase::PointXYA origin(0.0,0.0,vec.front().yaw);
    std::sort(vec.begin(),vec.end(),[&](const MapBase::GnssData &gd1,const MapBase::GnssData &gd2){
        MapBase::PointXYA local1=gd1.ToLocal(origin);
        MapBase::PointXYA local2=gd2.ToLocal(origin);
        return local1.x<local2.x;
    });
}

BlockMsg ExtractBlock(MapBase::GnssVec gv,int target_index)
{
    BlockMsg block_msg;
    /// ResortBlock(gv);
    for(const MapBase::GnssData &gd : gv)
    {
        block_msg.cells.push_back(ExtractPos(gd));
    }
    block_msg.block_size=block_msg.cells.size();
    block_msg.target_index=target_index;
    return block_msg;
}

DrawMsg LocalDrawMsg(const BlocksMsg &blocks,const MapBase::PointXYA &origin)
{
    std::vector<double> vx,vy;
    for(const BlockMsg &block : blocks)
    {
        for(const PtMsg &pt_msg : block.cells)
        {
            MapBase::PointXYA pt(pt_msg.y,pt_msg.x,pt_msg.yaw);
            MapBase::PointXYA pt_local=pt.ToLocal(origin);
            vx.push_back(pt_local.x);
            vy.push_back(pt_local.y);
        }
    }
    
    DrawMsg draw_msg;
    draw_msg.timestamp=0;
    draw_msg.ptnum=vx.size();
    draw_msg.x.swap(vx);
    draw_msg.y.swap(vy);
    return draw_msg;
}

void InitialLcmHandler()
{
    draw_map_handler->SetNet("udpm://238.255.76.67:7667?ttl=1");
    draw_map_handler->SetChannel("HTMAPDRAW");
    draw_map_handler->InitialSend();
    
    //map_handler->SetNet("udpm://238.255.76.67:7667?ttl=64");
    map_handler->SetChannel("HTMAP");
    map_handler->InitialSend();
    
}

void SendEmptyMap()
{
    MapMsg map_msg;
    map_msg.block_num=0;
    map_handler->SendLcm(&map_msg);

    ckLcmType::Draw_t empty_draw_msg;
    empty_draw_msg.ptnum=0;
    draw_map_handler->SendLcm(&empty_draw_msg);
}


#endif //MAPTOOLS_GLOBAL_H
