//
// Created by books on 18-8-9.
//

#include "Coordinate.h"

namespace MapBase{
    
    void Coordinate::Wgs84ToGauss(double lon, double lat, int distinguish, double &x, double &y)
    {
    
        const double RHO = 206265;
    
        int order = 0;
        if (distinguish == 6)
            order = int(lon / 6) + 1;
        else
            order = int(lon / 3 + 0.5);
    
        int L0 = 0;
        if (distinguish == 6)
            L0 = 6 * order - 3;
        else
            L0 = 3 * order;
    
        double CosB = std::cos(Degree2Grad(lat));
        double SinB = std::sin(Degree2Grad(lat));
        double CosBSquare = CosB * CosB;
        double SinBSquare = SinB * SinB;
    
        double l = ((lon - L0) * 3600) / RHO;
    
        double N = 6399698.902 - (21562.267 - (108.973 - 0.612 * CosBSquare) * CosBSquare) * CosBSquare;
    
        double a0 = 32140.404 - (135.3302 - (0.7092 - 0.0040 * CosBSquare) * CosBSquare) * CosBSquare;
    
        double a4 = (0.25 + 0.00252 * CosBSquare) * CosBSquare - 0.04166;
    
        double a6 = (0.166 * CosBSquare - 0.084) * CosBSquare;
    
        double a3 = (0.3333333 + 0.001123 * CosBSquare) * CosBSquare - 0.1666667;
    
        double a5 = 0.0083 - (0.1667 - (0.1968 + 0.0040 * CosBSquare) * CosBSquare) * CosBSquare;
    
        x = 6367558.4969 * (lat * 3600) / RHO - (a0 - (0.5 + (a4 + a6 * l * l) * l * l)
                                                      * l * l * N) * SinB * CosB;
        y = (1 + (a3 + a5 * l * l) * l * l) * l * N * CosB+500000;
    
    }
    
    void Coordinate::Wgs84ToMercator(double lon, double lat, double &x, double &y)
    {
        double tx, ty;
        convert_coordinates::latlon_to_mercator<double>(lat, lon, mercator_scale, tx, ty);
        x=ty-mercator_y0;
        y=tx-mercator_x0;
    }
    
    void Coordinate::MercatorToWgs84(double x, double y, double &lat, double &lon)
    {
        convert_coordinates::mercator_to_latlon(y + mercator_x0, x + mercator_y0, mercator_scale, lat, lon);
    }
}