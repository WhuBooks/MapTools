//
// Created by books on 8/29/18.
//

#include "Coordinate.h"
#include "convert_coordinates.hpp"
#include "AngleUtil.h"

namespace util
{
    void Coordinate::wgs84ToGauss(double lat, double lon, double &x, double &y)
    {
        double CosB = std::cos(util::degree2Grad(lat));
        double SinB = std::sin(util::degree2Grad(lat));
        double CosBSquare = CosB * CosB;
        double SinBSquare = SinB * SinB;
        
        double l = ((lon - gauss_L0) * 3600) / gauss_RHO;
        
        double N = 6399698.902 - (21562.267 - (108.973 - 0.612 * CosBSquare) * CosBSquare) * CosBSquare;
        
        double a0 = 32140.404 - (135.3302 - (0.7092 - 0.0040 * CosBSquare) * CosBSquare) * CosBSquare;
        
        double a4 = (0.25 + 0.00252 * CosBSquare) * CosBSquare - 0.04166;
        
        double a6 = (0.166 * CosBSquare - 0.084) * CosBSquare;
        
        double a3 = (0.3333333 + 0.001123 * CosBSquare) * CosBSquare - 0.1666667;
        
        double a5 = 0.0083 - (0.1667 - (0.1968 + 0.0040 * CosBSquare) * CosBSquare) * CosBSquare;
        
        x = 6367558.4969 * (lat * 3600) / gauss_RHO - (a0 - (0.5 + (a4 + a6 * l * l) * l * l)
                                                            * l * l * N) * SinB * CosB;
        y = (1 + (a3 + a5 * l * l) * l * l) * l * N * CosB + 500000;
        
    }
    
    void Coordinate::gaussToWgs84(double x, double y, double &lat, double &lon)
    {
        //        const double RHO = 206265;
        double myBeta = x / 6367558.4969 * gauss_RHO;
        
        double myCosBeta = std::cos(myBeta / 648000 * M_PI);
        double myCosBetaSquare = myCosBeta * myCosBeta;
        double mySinBeta = std::sin(myBeta / 648000 * M_PI);
        
        double myBf = myBeta + (50221746 + (293622 + (2350 + 22 * myCosBetaSquare) * myCosBetaSquare) * myCosBetaSquare) * 1e-10 * mySinBeta * myCosBeta * gauss_RHO;
        
        double myCosBf = std::cos(myBf / 648000.0 * M_PI);
        double myCosBfSquare = myCosBf * myCosBf;
        double mySinBf = std::sin(myBf / 648000.0 * M_PI);
        
        double myNf = 6399698.902 - (21562.267 - (108.973 - 0.612 * myCosBfSquare) * myCosBfSquare) * myCosBfSquare;
        
        double myZ = y / (myNf * myCosBf);
        
        double myb2 = (0.5 + 0.003369 * myCosBfSquare) * mySinBf * myCosBf;
        
        double myb3 = 0.333333 - (0.166667 - 0.001123 * myCosBfSquare) * myCosBfSquare;
        
        double myb4 = 0.25 + (0.16161 + 0.00562 * myCosBfSquare) * myCosBfSquare;
        
        double myb5 = 0.2 - (0.1667 - 0.0088 * myCosBfSquare) * myCosBfSquare;
        
        double myl = (1.0 - (myb3 - myb5 * myZ * myZ) * myZ * myZ) * myZ * gauss_RHO;
        
        lon = gauss_L0 * 3600.0 + myl;
        lat = myBf - (1.0 - (myb4 - 0.12 * myZ * myZ) * myZ * myZ) * myZ * myZ * myb2 * gauss_RHO;
        
    }
    
    void Coordinate::wgs84ToMercator(double lon, double lat, double &x, double &y)
    {
        double tx, ty;
        convert_coordinates::latlon_to_mercator<double>(lat, lon, mercator_scale, tx, ty);
        x = ty - mercator_y0;
        y = tx - mercator_x0;
    }
    
    void Coordinate::mercatorToWgs84(double x, double y, double &lat, double &lon)
    {
        convert_coordinates::mercator_to_latlon(y + mercator_x0, x + mercator_y0, mercator_scale, lat, lon);
    }
    
    Coordinate::Coordinate()
    {
        if (gauss_Distinguish == 6)
        {
            gauss_order = int(LonZero / 6.0) + 1.0;
            gauss_L0 = 6.0 * gauss_order - 3.0;
        }
        else
        {
            gauss_order = int(LonZero / 3.0 + 0.5);
            gauss_L0 = 3.0 * gauss_order;
        }
        
        mercator_scale = convert_coordinates::lat_to_scale(LatZero);
        convert_coordinates::latlon_to_mercator(LatZero, LonZero, mercator_scale, mercator_x0, mercator_y0);
    }
    
}