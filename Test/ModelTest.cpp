//
// Created by books on 2018/5/10.
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

double CalDensity(const MapBase::GnssVec &vec)
{
    std::vector<double> dis_vec;
    for(int i=0;i<vec.size()-1;i++)
    {
        MapBase::GnssData gd1=vec[i];
        MapBase::GnssData gd2=vec[i+1];
        double dis=gd1.Distance(gd2);
        dis_vec.push_back(dis);
    }
    
    double dis_ave=std::accumulate(dis_vec.begin(),dis_vec.end(),0.0)/dis_vec.size();
    return dis_ave;
    
}


int main(int argc,char **argv)
{
    std::string model_dir = (argc > 1) ? std::string(argv[1]) : "../dataset/Model";

    MapBase::RoadVec road_vec;
    MapBase::ReadRoad(model_dir, road_vec);
    std::cout<<"Road Size ~ "<<road_vec.size()<<std::endl;

    MapBase::ConnVec conn_vec;
    MapBase::ReadConline(model_dir, conn_vec);
    std::cout<<"Conn Size ~ "<<conn_vec.size()<<std::endl;
    
    std::cout << "Start Check Connect Relation." << std::endl;
    bool flag = true;
    for (const MapBase::Connline &conn : conn_vec)
    {
        int s_id = conn.sroadid;
        int e_id = conn.eroadid;
        bool find_s_id = false;
        bool find_e_id = false;
        for (const MapBase::Road &road : road_vec)
        {
            if (road.roadid == s_id)
                find_s_id = true;
            if (road.roadid == e_id)
                find_e_id = true;
        }
        flag = flag && find_s_id && find_e_id;
        if (!find_s_id || !find_e_id)
            std::cout << "Connect Relation Error ~ " << conn.connid << "\t" << s_id << "\t" << e_id << std::endl;
    }
    if (flag)
        std::cout << "All Connect Relation Checked." << std::endl;

    MapBase::Graph graph(road_vec,conn_vec);
    graph.Initialize();

    /// estimate density of road and connline
    for(const MapBase::Road &road : road_vec)
    {
        for(const MapBase::Lane &lane : road.vlane)
        {
            double dis_ave=CalDensity(lane.vec);
            std::cout<<"Lane ~ "<<lane.laneid<<"\t Density ~ "<<std::setprecision(8)<<dis_ave<<std::endl;
        }
    }
    
    for(const MapBase::Connline &conn : conn_vec)
    {
        double dis_ave=CalDensity(conn.data);
        std::cout<<"Conn ~ "<<conn.connid<<"\t Density ~ "<<std::setprecision(8)<<dis_ave<<std::endl;
    }


    /// calculate the link error between
    for(const MapBase::Connline &conn : conn_vec)
    {
        MapBase::Road road_s,road_e;
        int s_id=conn.sroadid;
        int e_id=conn.eroadid;

        for(const MapBase::Road &road : road_vec)
        {
            if(road.roadid==s_id)
                road_s=road;
            else if(road.roadid==e_id)
                road_e=road;
        }

        if(!road_s.vlane.empty()&&!road_e.vlane.empty())
        {
            double min_s_local_x = 100000000000;
            int s_lane_id = -1;
            for (const MapBase::Lane &lane : road_s.vlane)
            {
                MapBase::GnssData lane_back = lane.vec.back();
                MapBase::GnssData conn_front = conn.data.front();
                MapBase::PointXYA conn_front_local = conn_front.ToLocal(lane_back);
                double local_x = std::abs(conn_front_local.x);
                if (local_x < min_s_local_x)
                {
                    min_s_local_x = local_x;
                    s_lane_id = lane.laneid;
                }
            }

            double min_e_local_x = 100000000000;
            int e_lane_id = -1;
            for (const MapBase::Lane &lane : road_e.vlane)
            {
                MapBase::GnssData lane_front = lane.vec.front();
                MapBase::GnssData conn_back = conn.data.back();
                MapBase::PointXYA conn_back_local = conn_back.ToLocal(lane_front);
                double local_x = std::abs(conn_back_local.x);
                if (local_x < min_e_local_x)
                {
                    min_e_local_x = local_x;
                    e_lane_id = lane.laneid;
                }
            }

//            std::cout << "Conn ID ~ " << conn.connid << "\t";
//            std::cout << "Start Lane ~ " << s_lane_id << "\tDistance ~ " << min_s_local_x << "\t";
//            std::cout << "End Lane ~ " << e_lane_id << "\tDistance ~ " << min_e_local_x << std::endl;
//            std::cout<<std::endl;
        }
    }



    return 1;
}