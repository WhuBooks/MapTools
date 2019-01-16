//
// Created by books on 18-8-8.
//

#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <FileIO.h>
#include <LcmType/LcmHandler.h>
#include <LcmType/GPSData.hpp>

int main()
{
    std::string filename="Record/"+MapBase::GetNameFromTime()+".txt";
    std::ofstream ofs(filename,std::ios::out);
    if(!ofs.is_open())
    {
        std::cerr<<"Can't Open File ~ "<<filename<<std::endl;
        return -1;
    }
    
    LcmHandler<nox_lcm::GPSData>::Ptr handler(new LcmHandler<nox_lcm::GPSData>);
    handler->SetChannel("GPSData");
    handler->InitialListen();
    
    ofs<<std::setiosflags(std::ios::fixed);
    while(true)
    {
        nox_lcm::GPSData gps_msg;
        handler->GetData(gps_msg);
        
        ofs<<std::setprecision(2)<<gps_msg.time<<"\t";
        ofs<<std::setprecision(10)<<gps_msg.longitude<<"\t";
        ofs<<std::setprecision(10)<<gps_msg.latitude<<"\t";

        ofs<<std::setprecision(10)<<gps_msg.roll<<"\t";
        ofs<<std::setprecision(10)<<gps_msg.pitch<<"\t";
        ofs<<std::setprecision(10)<<gps_msg.heading<<"\t";
        
        ofs<<std::setprecision(10)<<gps_msg.v.x<<"\t";
        ofs<<std::setprecision(10)<<gps_msg.v.x<<"\t";
        ofs<<std::setprecision(10)<<gps_msg.v.x<<"\t";
    
        ofs<<std::setprecision(10)<<gps_msg.a.x<<"\t";
        ofs<<std::setprecision(10)<<gps_msg.a.x<<"\t";
        ofs<<std::setprecision(10)<<gps_msg.a.x<<"\t";
    
        ofs<<std::setprecision(10)<<gps_msg.w.x<<"\t";
        ofs<<std::setprecision(10)<<gps_msg.w.x<<"\t";
        ofs<<std::setprecision(10)<<gps_msg.w.x<<"\t";
    
        ofs<<std::setprecision(2)<<gps_msg.state<<"\n";
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
}