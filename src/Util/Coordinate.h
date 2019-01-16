//
// Created by books on 8/29/18.
//

#ifndef MAPTOOLS_COORDINATE_H
#define MAPTOOLS_COORDINATE_H

#include <iostream>
#include <cmath>

namespace util
{
    class Coordinate
    {
    public:
        static Coordinate &getInstance()
        {
            static Coordinate local;
            return local;
        }
        
        void wgs84ToGauss(double lat, double lon, double &x, double &y);
        
        void gaussToWgs84(double x,double y,double &lat,double &lon);
        
        void wgs84ToMercator(double lon, double lat, double &x, double &y);
        
        void mercatorToWgs84(double x, double y, double &lat, double &lon);
    
    private:
        Coordinate();
        
        Coordinate(const Coordinate &) = default;
        
        Coordinate &operator=(const Coordinate &) {};
    
        const double LatZero = 31.5921;
        const double LonZero = 120.7752;
        
        const double gauss_RHO = 206265;
        const int gauss_Distinguish=6;
        
        double gauss_L0;
        double gauss_order;
        
        double mercator_x0;
        double mercator_y0;
        double mercator_scale;
    };
}



#endif //MAPTOOLS_COORDINATE_H
