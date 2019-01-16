//
// Created by books on 2018/5/10.
//

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

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

#include <gps/convert_coordinates.hpp>


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
        y = tx - x0;
        x = ty - y0;
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

typedef nox_lcm::GPSData GpsMsg;
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

MapBase::PointXYA InitialPoint(const MapBase::RoadVec &vroad)
{
    MapBase::GnssVec vec;
    for(const MapBase::Road &ro : vroad)
    {
        for(const MapBase::Lane &la : ro.vlane)
        {
            vec.insert(vec.end(),la.vec.begin(),la.vec.end());
        }
    }

    std::shuffle(vec.begin(),vec.end(),std::default_random_engine(std::time(nullptr)));
    return MapBase::PointXYA(vec.front().x,vec.front().y,vec.front().yaw);
}

int main()
{
    std::string model_dir="../dataset/Model";
    MapBase::RoadVec vroad;
    if (!MapBase::ReadRoad(model_dir, vroad))
        return -1;
    std::cout << "Road file has been read ~ " << vroad.size() << std::endl;

    MapBase::ConnVec vconn;
    MapBase::ReadConline(model_dir,vconn);

    MapBase::RoadVec tmp_roads;
    for(const MapBase::Road &road : vroad)
    {
        bool is_s=false,is_e=false;
        for(const MapBase::Connline &conn : vconn)
        {
            if(conn.sroadid==road.roadid)
                is_s=true;
            if(conn.eroadid==road.roadid)
                is_e=true;
        }

        if(is_e && is_s)
        {
            tmp_roads.push_back(road);
        }

    }

    MapBase::PointXYA cur_pos=InitialPoint(tmp_roads);
    double x=cur_pos.x;
    double y=cur_pos.y;
    double yaw=cur_pos.yaw;
    std::cout<<"Initial Pos ~ "<<std::setprecision(12)<<x<<"\t"<<y<<"\t"<<yaw<<std::endl<<std::endl;

    MapHandlerPtr maphandler(new LcmHandler<nox_lcm::Map>);
    maphandler->SetNet("udpm://238.255.76.67:7667?ttl=64");
    maphandler->SetChannel("HTMAP");
    maphandler->InitialListen();
    
//    LocHandlerPtr lochandler(new LcmHandler<nox_lcm::Location>);
//    lochandler->SetNet("udpm://238.255.76.67:7667?ttl=64");
//    lochandler->SetChannel("HTLOCATION");
//    lochandler->InitialSend();

    GpsHandlerPtr gpshandler(new LcmHandler<GpsMsg>);
    gpshandler->SetChannel("GPSData");
    gpshandler->InitialSend();
    
    std::mutex loc_mutex;
    bool flag=true;
    std::thread loc_thread([&]() {
        Convert convert(LatZero,LonZero);
        while(flag)
        {
            std::unique_lock<std::mutex> lock(loc_mutex);

            
            GpsMsg gpsMsg;
            double lat,lon;
            convert.XY2LL(x,y,lat,lon);
            gpsMsg.latitude=lat;
            gpsMsg.longitude=lon;
            gpsMsg.heading=yaw;
            
            gpshandler->SendLcm(&gpsMsg);
            
            lock.unlock();

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    std::default_random_engine e(std::time(nullptr));
    std::normal_distribution<double> normal(0.0,0.05);
    while(true)
    {

        nox_lcm::Map mapMsg;
        maphandler->GetData(mapMsg);

        if(mapMsg.blocks.empty())
            continue;
        
        int index=-1;
        double min_dis=1000000000;
        for(int i=0;i<mapMsg.block_num;i++)
        {
            double block_min_dis=1000000000;
            for(const PtMsg &pt_msg : mapMsg.blocks[i].cells)
            {
                MapBase::PointXYA pt(pt_msg.y,pt_msg.x,pt_msg.yaw);
                double dis=pt.Distance(cur_pos);
                block_min_dis=block_min_dis<dis?block_min_dis:dis;
            }
            if(min_dis>block_min_dis)
            {
                min_dis=block_min_dis;
                index=i;
            }
        }

        int step=3;
        index=std::min(index+step,mapMsg.block_num-1);
        
        PtMsg next_pos=mapMsg.blocks[index].cells[mapMsg.blocks[index].target_index];

        std::unique_lock<std::mutex> lock(loc_mutex);
        x=next_pos.y+normal(e);
        y=next_pos.x+normal(e);
        yaw=MapBase::Degree(360.0-next_pos.yaw);
        cur_pos=MapBase::PointXYA(x,y,yaw);
        lock.unlock();

        std::cout<<"Get Map Message."<<std::endl;
//        std::cout<<"Current Pos ~ "<<std::setprecision(12)<<pos.x<<"\t"<<pos.y<<"\t"<<pos.yaw<<std::endl;
//        std::cout<<"Current Index ~ "<<mapMsg.curindex<<std::endl;
//        std::cout<<"Target Index ~ "<<mapMsg.targetindex<<std::endl;
//        std::cout<<"Conn Status ~ "<<(int)mapMsg.inconn<<std::endl;
        std::cout<<"Next Pos ~ "<<std::setprecision(12)<<x<<"\t"<<y<<"\t"<<yaw<<std::endl<<std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }



}