//
// Created by books on 18-8-9.
//

#ifndef MAPTOOLS_COORDINATE_H
#define MAPTOOLS_COORDINATE_H

#include <iostream>
#include <fstream>
#include <vector>

#include "Geometry.h"

#include <gps/convert_coordinates.hpp>

namespace MapBase{
    class Coordinate
    {
    public:
        static Coordinate & GetInstance()
        {
            static Coordinate instance;
            return instance;
        }
    
        void Wgs84ToGauss(double lon, double lat,int distinguish, double &x, double &y);
        
        void Wgs84ToMercator(double lon,double lat,double &x,double &y);
        
        void MercatorToWgs84(double x,double y,double &lat,double &lon);
        
    private:
        Coordinate()
        {
            static const double LatZero=31.5921;
            static const double LonZero=120.7752;
            mercator_scale = convert_coordinates::lat_to_scale(LatZero);
            convert_coordinates::latlon_to_mercator(LatZero, LonZero, mercator_scale, mercator_x0, mercator_y0);
        };
        Coordinate(const Coordinate &other);
        Coordinate& operator=(const Coordinate &);
   
        double mercator_x0;
        double mercator_y0;
        double mercator_scale;
        
    };
}



#endif //MAPTOOLS_COORDINATE_H
