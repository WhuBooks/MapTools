//
// Created by books on 18-7-24.
//

#ifndef MAPTOOLS_BLOCKMESSAGE_H
#define MAPTOOLS_BLOCKMESSAGE_H

#include "Common.h"
#include <gps/convert_coordinates.hpp>
#include <ctime>
#include <sys/time.h>
#include <chrono>

std::chrono::steady_clock::time_point tp_last=std::chrono::steady_clock::now();
const double map_msg_interval=1.0;

static const double LatZero=31.5921;
static const double LonZero=120.7752;

class Convert
{
public :
    
    Convert(double lat,double lon)
    {
        scale = convert_coordinates::lat_to_scale(lat);
        convert_coordinates::latlon_to_mercator(lat, lon, scale, x0, y0);
    }
    
    void LL2XY(double lat,double lon,double &x,double &y)
    {
        double tx, ty;
        convert_coordinates::latlon_to_mercator<double>(lat, lon, scale, tx, ty);
        
//        x = tx - x0;
//        y = ty - y0;
        
        /// convert Cartesian to Gauss
        y=tx-x0;
        x=ty-y0;
    }
    
    void XY2LL(double x,double y,double &lat,double &lon)
    {
        convert_coordinates::mercator_to_latlon(y + x0, x + y0, scale, lat, lon);
    }

private:
    
    double x0;
    double y0;
    
    double scale;
    
};

bool CheckBlock(const MapBase::GnssVec &vec)
{
    if(vec.size()<=1)
        return true;
    MapBase::GnssData left=vec.front();
    for(int i=1;i<vec.size();i++)
    {
        MapBase::PointXYA local=vec[i].ToLocal(left);
        
        if(std::abs(local.y)>1.0)
            return false;
    }
    
    return true;
    
}



BlocksMsg FromCurrRoad(const MapBase::Road &road,int target_index=0)
{
//    std::vector<MapBase::GnssVec> tmp_lanes;
//
//    int min_lane_size=1000000000;
//    for(const MapBase::Lane &lane : road.vlane)
//    {
//        int index=NearestIndex(lane.vec,g_match);
//
//        int s_index=(index<backward)?0:index-backward;
//        //int e_index=(index+forward>lane.vec.size())?(int)lane.vec.size():(index+forward);
//        int e_index=lane.vec.size();
//
//        MapBase::GnssVec tmp_lane;
//        tmp_lane.assign(lane.vec.begin()+s_index,lane.vec.begin()+e_index);
//        tmp_lanes.push_back(tmp_lane);
//
//        min_lane_size=(min_lane_size<tmp_lane.size())?min_lane_size:tmp_lane.size();
//    }
    
    int target_lane_index=NearestIndex(road.vlane[target_index].vec,g_match);
    int s_index=(target_lane_index<backward)?0:target_lane_index-backward;
    //int e_index=(index+forward>lane.vec.size())?(int)lane.vec.size():(index+forward);
    int e_index=road.vlane[target_index].vec.size();
    
    BlocksMsg blocks;
    for(int i=s_index;i<e_index;i++)
    {
        MapBase::GnssData tmp_target=road.vlane[target_index].vec[i];
        MapBase::GnssVec tmp_vec;
        for(const MapBase::Lane &lane : road.vlane)
        {
            int tmp_index=NearestIndex(lane.vec,tmp_target);
            tmp_vec.push_back(lane.vec[tmp_index]);
        }
        if(CheckBlock(tmp_vec))
            blocks.push_back(ExtractBlock(tmp_vec,target_index));
    }
    return blocks;
//    for(int i=0;i<min_lane_size;i++)
//    {
//        MapBase::GnssVec tmp_vec;
//        for(const MapBase::GnssVec &vec : tmp_lanes)
//            tmp_vec.push_back(vec[i]);
//        blocks.push_back(ExtractBlock(tmp_vec,target_index));
//
//    }
//    return blocks;
}

BlocksMsg FromNextRoad(const MapBase::Road &road,int target_index=0)
{
    BlocksMsg blocks;
    
    for(int i=0;i<road.vlane[target_index].vec.size();i++)
    {
        MapBase::GnssData tmp_target=road.vlane[target_index].vec[i];
        MapBase::GnssVec tmp_vec;
        for(const MapBase::Lane &lane : road.vlane)
        {
            int tmp_index=NearestIndex(lane.vec,tmp_target);
            tmp_vec.push_back(lane.vec[tmp_index]);
        }
        
        if(CheckBlock(tmp_vec))
            blocks.push_back(ExtractBlock(tmp_vec,target_index));
    }
    return blocks;
    
    
//    for(int i=0;;i++)
//    {
//        MapBase::GnssVec tmp_vec;
//        bool flag=true;
//        for(const MapBase::Lane &lane : road.vlane)
//        {
//            if(lane.vec.size()<=i)
//            {
//                flag=false;
//                break;
//            }
//            tmp_vec.push_back(lane.vec[i]);
//        }
//        if(!flag)
//            break;
//        blocks.push_back(ExtractBlock(tmp_vec,target_index));
//    }
//
//    return blocks;
}

BlocksMsg FromCurrConn(const MapBase::Connline &conn)
{
    BlocksMsg blocks;

    int index=NearestIndex(conn.data,g_match);
    int s_index=index<backward?0:index-backward ;
//    int e_index=(index+forward>conn.data.size())?conn.data.size():index+forward;
    int e_index=conn.data.size();
    for(int i=s_index;i<e_index;i++)
        blocks.push_back(ExtractBlock(MapBase::GnssVec{conn.data[i]},0));
    return blocks;
}

BlocksMsg FromNextConn(const MapBase::Connline &conn)
{
    BlocksMsg blocks;

    for(const MapBase::GnssData &tmp : conn.data)
        blocks.push_back(ExtractBlock(MapBase::GnssVec{tmp},0));
    return blocks;
}

void SendBlockMsg(const MapBase::RoadVec &vroad,const MapBase::ConnVec &vconn)
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
        std::cerr<<"Can't Find Proper Road In BlockMessage!"<<std::endl;
        return;
    }
    
    BlocksMsg blocks;
    if(g_match.laneid!=-1)
    {
        
        bool need_next_conn=true,need_next_road=true;
        
        /// current road to Block message
        BlocksMsg curr_road_block=FromCurrRoad(road,road.IndexFromID(conn.slaneid));
        blocks.insert(blocks.end(),curr_road_block.begin(),curr_road_block.end());
        
        /// Conn to Block
        
        if(need_next_conn && findconn)
        {
//            MapBase::Connline conn_tmp;
            BlocksMsg next_conn_block = FromNextConn(conn);
            blocks.insert(blocks.end(),next_conn_block.begin(),next_conn_block.end());
        }
        
        /// next road to Block Message
        if(need_next_road)
        {
            MapBase::Road next_road;
            bool find_next_road=false;
            for(const MapBase::Road &ro : vroad)
            {
                if(ro.roadid==conn.eroadid)
                {
                    find_next_road=true;
                    next_road=ro;
                    break;
                }
            }
            if(find_next_road)
            {
                BlocksMsg next_road_block=FromNextRoad(next_road,next_road.IndexFromID(conn.elaneid));
                blocks.insert(blocks.end(),next_road_block.begin(),next_road_block.end());
            }
        }
    }
    else if(findconn)
    {
        bool need_next_road=true;
        
        /// current conn to Block message
        BlocksMsg curr_conn_block=FromCurrConn(conn);
        blocks.insert(blocks.end(),curr_conn_block.begin(),curr_conn_block.end());
        if(need_next_road && findroad)
        {
            BlocksMsg next_road_block=FromNextRoad(road,road.IndexFromID(conn.elaneid));
            blocks.insert(blocks.end(),next_road_block.begin(),next_road_block.end());
        }
    }
    
    MapMsg map_msg;
	timeval t;
	gettimeofday(&t,NULL);
	map_msg.time=t.tv_sec*1000+t.tv_usec/1000;
    map_msg.block_num=(int)blocks.size();
    map_msg.blocks.assign(blocks.begin(),blocks.end());
    map_msg.status=0;
    
    std::chrono::steady_clock::time_point tpn=std::chrono::steady_clock::now();
    long diff=std::chrono::duration_cast<std::chrono::seconds>(tpn-tp_last).count();
    
    map_msg.status=(g_update_map>0)?map_msg.status|MapMsg::update:map_msg.status;
    map_msg.status=(g_come_target)?map_msg.status|MapMsg::slow:map_msg.status;
    map_msg.status=map_msg.status|(g_update_times<<24);
    if(diff>map_msg_interval || g_need_send_map)
    {
        g_update_map--;
        tp_last=tpn;
        std::cout<<"Send Map"<<std::endl;
        if(map_msg.status%2==1)
            std::cout<<"Update!!!!!!!"<<std::endl;
        map_handler->SendLcm(&map_msg);
    }
    
    /// block to draw message
    DrawMsg draw_msg=LocalDrawMsg(blocks,cur_pos);
    draw_map_handler->SendLcm(&draw_msg);

}


#endif //MAPTOOLS_BLOCKMESSAGE_H
