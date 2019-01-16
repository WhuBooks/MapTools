//
// Created by books on 18-7-24.
//

#include <iostream>
#include <fstream>
#include <vector>

#include <map_header.h>

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
        x=ty-y0;
        y=tx-x0;
    }
    
    void XY2LL(double x,double y,double &lat,double &lon)
    {
        convert_coordinates::mercator_to_latlon(x + x0, y + y0, scale, lat, lon);
    }
    
private:
    
    double x0;
    double y0;
    
    double scale;
    
};


int main(int argc,char **argv)
{
    std::string input_dir=argc==1?"../dataset/Model/Lane":std::string(argv[1]);
    std::string output_dir=argc<3?"../dataset/NewModel":std::string(argv[2]);
    
    output_dir+="/"+MapBase::GetNameFromTime();
    if(!MapBase::DirBuild(output_dir))
        return -1;
    std::cout<<"Output Directory ~ "<<output_dir<<std::endl;
    
    Convert convert(LatZero,LonZero);
    
    std::vector<std::string> file_vec=MapBase::GetFiles(input_dir);
    for(const std::string &filename : file_vec)
    {
        MapBase::GnssVec gnssVec;
        MapBase::ReadGnss(filename,gnssVec);
        for(MapBase::GnssData &gd : gnssVec)
        {
            convert.LL2XY(gd.lat,gd.lon,gd.x,gd.y);
        }
        
        std::string output_filename=output_dir+"/"+MapBase::SplitNameWithExt(filename);
        MapBase::WriteGnss(output_filename,gnssVec);
        
    }
    
    return 1;
}