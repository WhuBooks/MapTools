//
// Created by books on 18-7-27.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <random>

#include <FileIO.h>
#include <Geometry.h>
#include <Graph.h>
#include <RoadModel.h>
#include <Cluster.h>
#include <DouglasPeucker.h>


int main(int argc,char **argv)
{
    std::string model_dir = (argc > 1) ? std::string(argv[1]) : "../dataset/Model";
    
    MapBase::RoadVec road_vec;
    MapBase::ReadRoad(model_dir, road_vec);
    std::cout<<"Road Size ~ "<<road_vec.size()<<std::endl;
    
    std::string filename=MapBase::GetNameFromTime()+".txt";
    std::ofstream ofs(filename);
    
    if(!ofs.is_open())
        return -1;
    
    std::default_random_engine e(std::time(nullptr));
    std::uniform_real_distribution<> uniform(0.25,0.4);
    
    for(const MapBase::Road &road : road_vec)
    {
        MapBase::GnssVec gnssVec=road.vlane.front().vec;
        
        int size=std::floor(uniform(e)*gnssVec.size());
        MapBase::GnssData gd=gnssVec[size];
        
        ofs<<road.roadid<<"\t"<<gd.id<<"\t"<<std::setprecision(10)<<gd.lat<<"\t"<<gd.lon<<"\t"<<gd.yaw<<std::endl;
    }
    ofs.close();
    
}