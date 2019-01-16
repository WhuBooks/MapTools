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

void Local(const MapBase::Lane &s_lane,const MapBase::Lane &e_lane,const MapBase::PtXYAVec &vec,std::vector<double> &x,std::vector<double> &y)
{
    x.clear();
    y.clear();
    
    MapBase::PointXYA origin=s_lane.vec.back();
    for(int i=s_lane.vec.size()-10;i<s_lane.vec.size();i++)
    {
        MapBase::PointXYA local=s_lane.vec[i].ToLocal(origin);
        x.push_back(local.x);
        y.push_back(local.y);
    }
    for(const MapBase::PointXYA &pt : vec)
    {
        MapBase::PointXYA local=pt.ToLocal(origin);
        x.push_back(local.x);
        y.push_back(local.y);
    }
    for(int i=0;i<10;i++)
    {
        MapBase::PointXYA local=e_lane.vec[i].ToLocal(origin);
        x.push_back(local.x);
        y.push_back(local.y);
    }
}

int main(int argc,char **argv)
{
    std::string modeldir = (argc == 1) ? "../dataset/Model" : std::string(argv[1]);
    
    MapBase::RoadVec full_road_vec;
    if (!MapBase::ReadRoad(modeldir, full_road_vec))
        return -1;
    for(MapBase::Road &road : full_road_vec)
        road.CalReverseId(full_road_vec);
    std::cout << "Road file has been read ~ " << full_road_vec.size() << std::endl;
    
    MapBase::ConnVec full_conn_vec;
    MapBase::ReadConline(modeldir, full_conn_vec);
    std::cout << "Connect line has been read ~ " << full_conn_vec.size() << std::endl;
    
    int min_conn_id=-1;
    for(const MapBase::Connline &conn : full_conn_vec)
        if(min_conn_id<=conn.connid)
            min_conn_id=conn.connid+1;
    
    typedef LcmHandler<ckLcmType::Draw_t>::Ptr DrawHandlerPtr;
    DrawHandlerPtr draw_map_handler(new LcmHandler<ckLcmType::Draw_t>);
    
    draw_map_handler->SetNet("udpm://238.255.76.67:7667?ttl=1");
    draw_map_handler->SetChannel("HTMAPDRAW");
    draw_map_handler->InitialSend();
    
    std::vector<std::vector<int>> s_road_e_road;
    std::ifstream ifs("../dataset/NewConn.txt");
    if(!ifs.is_open())
        return -1;
    
    while(ifs.good()&&!ifs.eof())
    {
        int s_road_id=-1,s_lane_id=-1,e_road_id=-1,e_lane_id=-1;
        ifs>>s_road_id>>s_lane_id>>e_road_id>>e_lane_id;
        s_road_e_road.push_back(std::vector<int>({s_road_id,s_lane_id,e_road_id,e_lane_id}));
    }
    s_road_e_road.erase(s_road_e_road.end());
    std::cout<<"New Conn Size ~ "<<s_road_e_road.size()<<std::endl;
    
    MapBase::Clothoid clothoid;
    std::string dir_name=MapBase::GetNameFromTime();
    MapBase::DirBuild(dir_name);
    Convert convert(LatZero,LonZero);
    for(const auto &tmp : s_road_e_road)
    {
        
        MapBase::GnssData s_back,e_front;
        MapBase::Lane s_lane,e_lane;
        bool find_back=false,find_front= false;
        for(const MapBase::Road &road : full_road_vec)
        {
            if(road.roadid==tmp[0])
            {
                for(const MapBase::Lane &lane : road.vlane)
                {
                    if(lane.laneid==tmp[1])
                    {
                        find_back=true;
                        s_back = lane.vec.back();
                        s_lane=lane;
                    }
                }
            }
    
            if(road.roadid==tmp[2])
            {
                for(const MapBase::Lane &lane : road.vlane)
                {
                    if(lane.laneid==tmp[3])
                    {
                        find_front=true;
                        e_front = lane.vec.front();
                        e_lane=lane;
                    }
                }
            }
            
            if(find_front&&find_back)
                break;
        }
        
        if(!find_back || !find_front)
        {
            std::cerr<<"Can't Find Back Or Front."<<std::endl;
            continue;
        }
        
//        double middle_x=(s_back.x+e_front.x)/2.0;
//        double middle_y=(s_back.y+e_front.y)/2.0;
//        double angleDiff=std::max(s_back.yaw,e_front.yaw)-std::min(s_back.yaw,e_front.yaw);
//        if(angleDiff>180.0)
//            angleDiff=360.0-angleDiff;
//        angleDiff=angleDiff/2.0+std::min(s_back.yaw,e_front.yaw);
//
//        MapBase::PointXYA middle(middle_x,middle_y,angleDiff);
//        MapBase::PtXYAVec tmp1=clothoid.Build(s_back,middle,300,0.05);
//        MapBase::PtXYAVec tmp2=clothoid.Build(middle,e_front,300,0.05);
//        MapBase::PtXYAVec tmp_1_2;
//        tmp_1_2.insert(tmp_1_2.end(),tmp1.begin(),tmp1.end());
//        tmp_1_2.insert(tmp_1_2.end(),tmp2.begin(),tmp2.end());
        
        MapBase::PtXYAVec tmp_conn_data=clothoid.Build(s_back,e_front,300,0.05);
//        tmp_conn_data.swap(tmp_1_2);
    
        std::cout<<"SRoad="<<tmp[0]<<"\t";
        std::cout<<"SLane="<<tmp[1]<<"\t";
        std::cout<<"ERoad="<<tmp[2]<<"\t";
        std::cout<<"ELane="<<tmp[3]<<"\t";
        
        if(tmp_conn_data.empty())
        {
            std::cout<<"Failed"<<std::endl;
            continue;
        }
    
        std::cout<<"Succeed"<<std::endl;
        
        MapBase::GnssVec tmp_conn;
        for(const MapBase::PointXYA &pt : tmp_conn_data)
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
            tmp_conn.push_back(gd);
        }
        

        std::string tmp_conn_file=dir_name+"/connid_"+std::to_string(min_conn_id++)+"sroad_"+std::to_string(tmp[0])+"eroad_"+std::to_string(tmp[2])+".txt";
        MapBase::WriteGnss(tmp_conn_file,tmp_conn);
        
        
        
        std::vector<double> vx,vy;
        Local(s_lane,e_lane,tmp_conn_data,vx,vy);
        ckLcmType::Draw_t msg;
        msg.ptnum=vx.size();
        msg.x.swap(vx);
        msg.y.swap(vy);
        
//        for(int i=0;i<10;i++)
//        {
//            draw_map_handler->SendLcm(&msg);
//            //std::this_thread::sleep_for(std::chrono::milliseconds(50));
//        }
        
        //getchar();
    }
    
    return 1;
}