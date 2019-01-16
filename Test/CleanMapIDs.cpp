//
// Created by books on 18-7-30.
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
    
    double angle_diff_thres=30.0;
    
    int ptId=0;
    MapBase::DouglasPeucker dp(0.01,1.0);
    
    MapBase::RoadVec tmp_road_vec;
    for(const MapBase::Road &road : full_road_vec)
    {
        MapBase::Road tmp_road;
        tmp_road.roadid=road.roadid;
        MapBase::LaneVec lane_vec;
        for(const MapBase::Lane &lane : road.vlane)
        {
            MapBase::Lane tmp_lane;
            tmp_lane.laneid=lane.laneid;
            tmp_lane.vec=dp.Process(lane.vec);
            MapBase::GnssVec vec=dp.Process(lane.vec);
            MapBase::GnssVec tmp_vec;
            int num=0;
            for(int i=0;i<vec.size()-1;i++)
            {
                MapBase::GnssData s=vec[i];
                MapBase::GnssData e=vec[i+1];
    
                double dx=e.x-s.x;
                double dy=e.y-s.y;
    
                double theta=MapBase::Grad2Degree(std::atan2(dy,dx));
                double yaw=s.yaw;
    
                double diff=CalAngleDiff(yaw,theta);
                if(diff>angle_diff_thres)
                {
                    num++;
                    s.yaw = theta;
                }
                
                s.id=ptId++;
                tmp_vec.push_back(s);
            }
            std::cout<<"LaneId ~ "<<lane.laneid<<"\tWrong Pts Num ~ "<<num<<std::endl;
            
            tmp_lane.vec.assign(tmp_vec.begin(),tmp_vec.end());
            lane_vec.push_back(tmp_lane);
        }
        tmp_road_vec.push_back(road);
    }
    
    MapBase::ConnVec tmp_conn_vec;
    for(const MapBase::Connline &conn : full_conn_vec)
    {
        MapBase::Connline tmp_conn;
        tmp_conn.connid=conn.connid;
        tmp_conn.sroadid=conn.sroadid;
        tmp_conn.eroadid=conn.eroadid;
        tmp_conn.slaneid=conn.slaneid;
        tmp_conn.elaneid=conn.elaneid;
        
        MapBase::GnssVec vec=dp.Process(conn.data);
        MapBase::GnssVec tmp_vec;
        int num=0;
        for(int i=0;i<vec.size()-1;i++)
        {
            MapBase::GnssData s=vec[i];
            MapBase::GnssData e=vec[i+1];
        
            double dx=e.x-s.x;
            double dy=e.y-s.y;
        
            double theta=MapBase::Grad2Degree(std::atan2(dy,dx));
            double yaw=s.yaw;
        
            double diff=CalAngleDiff(yaw,theta);
            if(diff>angle_diff_thres)
            {
                num++;
                s.yaw = theta;
            }
        
            s.id=ptId++;
            tmp_vec.push_back(s);
        }
        std::cout<<"ConnId ~ "<<conn.connid<<"\tWrong Pts Num ~ "<<num<<std::endl;
        tmp_conn.data.assign(tmp_vec.begin(),tmp_vec.end());
        tmp_conn_vec.push_back(tmp_conn);
    }
    
    std::map<int,int> road_ids_map;
    std::map<int,int> lane_ids_map;
    int roadId=0;
    int laneId=0;
    for(MapBase::Road &road : tmp_road_vec)
    {
        int old_road_id = road.roadid;
        road.roadid = roadId++;
        road_ids_map[old_road_id] = road.roadid;
    
        for (MapBase::Lane &lane : road.vlane)
        {
            int old_lane_id=lane.laneid;
            lane.laneid = laneId++;
            lane_ids_map[old_lane_id]=lane.laneid;
        }
    }
    int connId=0;
    for(MapBase::Connline &conn : tmp_conn_vec)
    {
        conn.connid=connId++;
        int sroadid=conn.sroadid;
        int eroadid=conn.eroadid;
        int slaneid=conn.slaneid;
        int elaneid=conn.elaneid;
        conn.sroadid=road_ids_map[sroadid];
        conn.eroadid=road_ids_map[eroadid];
        conn.slaneid=lane_ids_map[slaneid];
        conn.elaneid=lane_ids_map[elaneid];
    }
    
    std::string newdir="../dataset/"+MapBase::GetNameFromTime();
    MapBase::DirBuild(newdir);
    
    std::string newlanedir=newdir+"/Lane/";
    std::string newconndir=newdir+"/Conn/";
    MapBase::DirBuild(newlanedir);
    MapBase::DirBuild(newconndir);
    
    std::string road_index_file=newdir+"/roadindex.txt";
    std::string conn_index_file=newdir+"/connindex.txt";
    std::string lane_index_file=newdir+"/laneindex.txt";
    
    std::ofstream ofs1(road_index_file,std::ios::out);
    std::ofstream ofs2(conn_index_file,std::ios::out);
    std::ofstream ofs3(lane_index_file,std::ios::out);
    for(const MapBase::Road &road : tmp_road_vec)
    {
        for(const MapBase::Lane &lane : road.vlane)
        {
            std::string tmp_file=newlanedir+std::to_string(lane.laneid)+".txt";
            MapBase::WriteGnss(tmp_file,lane.vec);
            ofs1<<road.roadid<<"\t"<<lane.laneid<<"\n";
        }
    }
    
    for(const MapBase::Connline &conn : tmp_conn_vec)
    {
        std::string tmp_file=newconndir+std::to_string(conn.connid)+".txt";
        MapBase::WriteGnss(tmp_file,conn.data);
        ofs2<<conn.connid<<"\t"<<conn.sroadid<<"\t"<<conn.eroadid<<"\n";
        ofs3<<conn.connid<<"\t"<<conn.slaneid<<"\t"<<conn.elaneid<<"\n";
    }
    
    ofs1.close();
    ofs2.close();
    ofs3.close();
    
    
}