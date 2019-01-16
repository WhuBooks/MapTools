//
// Created by books on 18-7-29.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>

#include <FileIO.h>
#include <Geometry.h>
#include <Graph.h>
#include <RoadModel.h>
#include <Cluster.h>
#include <DouglasPeucker.h>


double CalAngleDiff(double angle1,double angle2)
{
    double diff=std::max(angle1,angle2)-std::min(angle1,angle2);
    if(diff>180)
        diff=360-diff;
    return diff;
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
    
    /// modify road and conn
    bool has_lane_connection=MapBase::ReadConlineLaneID(modeldir,full_conn_vec);
    for (MapBase::Connline &conn : full_conn_vec)
    {
        if(has_lane_connection)
            conn.ConnectLane(full_road_vec);
        else
            conn.InitLaneID(full_road_vec);
    }
    
    int num=0;
    double diff_thres=20;
    std::vector<double> larger_vec;
    std::map<int,double> larger_map;
    std::vector<int> all_zero_lane_ids;
    std::vector<int> all_zero_conn_ids;
    
    for(const MapBase::Road &road : full_road_vec)
    {
        for(const MapBase::Lane &lane : road.vlane)
        {
            bool all_zero=true;
            MapBase::GnssVec vec=lane.vec;
            
            for(int i=0;i<vec.size()-1;i++)
            {
                MapBase::GnssData s=vec[i];
                MapBase::GnssData e=vec[i+1];
                
                double dx=e.x-s.x;
                double dy=e.y-s.y;
                
                double theta=MapBase::Grad2Degree(std::atan2(dy,dx));
                double yaw=s.yaw;
                
                double diff=CalAngleDiff(yaw,theta);
                if(diff>diff_thres)
                {
                    larger_map[s.id]=diff;
                    larger_vec.push_back(diff);
                }
                
                all_zero=all_zero&&yaw==0.0;
            }
            if(all_zero)
                all_zero_lane_ids.push_back(lane.laneid);
        }
    }
    
    for(const MapBase::Connline &conn : full_conn_vec)
    {
        bool all_zero=true;
        MapBase::GnssVec vec=conn.data;
        for(int i=0;i<vec.size()-1;i++)
        {
            MapBase::GnssData s=vec[i];
            MapBase::GnssData e=vec[i+1];
        
            double dx=e.x-s.x;
            double dy=e.y-s.y;
        
            double theta=MapBase::Grad2Degree(std::atan2(dy,dx));
            double yaw=s.yaw;
    
            double diff=CalAngleDiff(yaw,theta);
            if(diff>diff_thres)
            {
                larger_map[s.id]=diff;
                larger_vec.push_back(diff);
            }
            
            all_zero=all_zero&&yaw==0.0;
        }
        if(all_zero)
            all_zero_conn_ids.push_back(conn.connid);
    }

    std::cout<<larger_vec.size()<<std::endl;
    std::cout<<all_zero_lane_ids.size()<<std::endl;
    std::cout<<all_zero_conn_ids.size()<<std::endl;
    
    return 1;
    
}