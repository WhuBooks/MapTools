//
// Created by books on 18-7-30.
//

#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <thread>

#include <Geometry.h>
#include <RoadModel.h>
#include <Clothoid.h>
#include <FileIO.h>
#include <DouglasPeucker.h>
#include <Cluster.h>
#include <Log.h>

#include <LcmType/LcmHandler.h>
#include <LcmType/Draw_t.hpp>

#include <gps/convert_coordinates.hpp>
#include <ctime>
#include <sys/time.h>

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


int main(int argc,char **argv)
{
    std::string modeldir = (argc == 1) ? "../dataset/Model_2017_2018" : std::string(argv[1]);
    
    MapBase::RoadVec full_road_vec;
    if (!MapBase::ReadRoad(modeldir, full_road_vec))
        return -1;
    for(MapBase::Road &road : full_road_vec)
        road.CalReverseId(full_road_vec);
    std::cout << "Road file has been read ~ " << full_road_vec.size() << std::endl;
    
    MapBase::ConnVec full_conn_vec;
    MapBase::ReadConline(modeldir, full_conn_vec);
    std::cout << "Connect line has been read ~ " << full_conn_vec.size() << std::endl;
    
    int min_lane_id=-1;
    for(const MapBase::Road &road : full_road_vec)
    {
        for(const MapBase::Lane &lane : road.vlane)
            if(min_lane_id<=lane.laneid)
                min_lane_id=lane.laneid+1;
    }
    
    int s_conn_id=168;
    int e_conn_id=1109;
    int road_id=10;
    int lane_id=335;
    
    MapBase::GnssData s_back,e_front;
    for(const MapBase::Connline &conn : full_conn_vec)
    {
//        if(s_conn_id==conn.connid)
//        {
//            s_back=conn.data.back();
//        }
        if(e_conn_id==conn.connid)
        {
            e_front=conn.data.front();
        }
    }
    
    for(const MapBase::Road &road : full_road_vec)
    {
        if(road.roadid==road_id)
        {
            for (const MapBase::Lane &lane : road.vlane)
            {
                if (lane.laneid == lane_id)
                {
                    s_back=lane.vec.front();
                }
            }
        }
    }
    
    
    MapBase::Clothoid clothoid;
    MapBase::PtXYAVec tmp_lane=clothoid.Build(s_back,e_front,1000,0.05);
    
    Convert convert(LatZero,LonZero);
    MapBase::GnssVec tmp_vec;
    for(const MapBase::PointXYA &pt : tmp_lane)
    {
        MapBase::GnssData gd;
        gd.id=MapBase::GetNewGnssID();
        gd.timestamp=s_back.timestamp;
        gd.x=pt.x;
        gd.y=pt.y;
        gd.hgt=s_back.hgt;
        gd.yaw=pt.yaw;
        gd.roll=s_back.roll;
        gd.pitch=s_back.pitch;
        gd.east=s_back.east;
        gd.north=s_back.north;
        gd.up=s_back.up;
        double lat=0.0,lon=0.0;
        convert.XY2LL(pt.x,pt.y,lat,lon);
        gd.lat=lat;
        gd.lon=lon;
        tmp_vec.push_back(gd);
    }
    
    if(!tmp_lane.empty())
    {
        std::string filename="Road_"+std::to_string(road_id)+"Lane_"+std::to_string(min_lane_id)+".txt";
        MapBase::DouglasPeucker douglasPeucker(0.01,1.0);
        MapBase::GnssVec sparse=douglasPeucker.Process(tmp_vec);
        MapBase::WriteGnss(filename,sparse);
        std::cout<<"Succeed"<<std::endl;
    }
}