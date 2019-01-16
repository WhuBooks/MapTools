//
// Created by books on 18-7-31.
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
    for (MapBase::Road &road : full_road_vec)
        road.CalReverseId(full_road_vec);
    std::cout << "Road file has been read ~ " << full_road_vec.size() << std::endl;
    
    MapBase::ConnVec full_conn_vec;
    MapBase::ReadConline(modeldir, full_conn_vec);
    std::cout << "Connect line has been read ~ " << full_conn_vec.size() << std::endl;
    
    /// modify road and conn
    for (MapBase::Connline &conn : full_conn_vec)
        conn.InitLaneID(full_road_vec);
 
    std::string filename="conn_lane_ids.txt";
    std::ofstream ofs(filename);
    for(const MapBase::Connline &conn : full_conn_vec)
        ofs<<conn.connid<<"\t"<<conn.slaneid<<"\t"<<conn.elaneid<<std::endl;
    ofs.close();
    return 1;
}