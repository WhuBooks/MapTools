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

int main(int argc,char **argv)
{
    std::string input_dir=argc==1?"../dataset/Model/Lane":std::string(argv[1]);
    std::string output_dir=argc<3?"../dataset/NewModel":std::string(argv[2]);
    
    output_dir+="/"+MapBase::GetNameFromTime();
    if(!MapBase::DirBuild(output_dir))
        return -1;
    std::cout<<"Output Directory ~ "<<output_dir<<std::endl;
    
    
    std::vector<std::string> file_vec=MapBase::GetFiles(input_dir);
    for(const std::string &filename : file_vec)
    {
        MapBase::GnssVec gnssVec;
        MapBase::ReadGnss(filename,gnssVec);
        for(MapBase::GnssData &gd : gnssVec)
        {
            MapBase::PointXYA pt_local(-1.0,0.0,gd.yaw);
            MapBase::PointXYA pt_world=pt_local.ToWorld(gd);
            gd.x=pt_world.x;
            gd.y=pt_world.y;
        }
        
        std::string output_filename=output_dir+"/"+MapBase::SplitNameWithExt(filename);
        MapBase::WriteGnss(output_filename,gnssVec);
        
    }
    
}