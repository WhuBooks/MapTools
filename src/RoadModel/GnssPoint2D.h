//
// Created by 98420 on 2018/8/30.
//

#ifndef MAPTOOLS_GNSSPOINT2D_H
#define MAPTOOLS_GNSSPOINT2D_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <memory>

#include <Geometry/Point2D.h>
#include "Lane.h"

namespace maplib
{
    namespace model
    {
        struct GnssInfo
        {
            int id;
            int timestamp;
            double lon;
            double lat;
            double hgt;
            double roll;
            double pitch;
            double velocity;
            double accelerate;
    
            GnssInfo():id(-1),timestamp(0),lon(0),lat(0),hgt(0),roll(0),pitch(0),velocity(0),accelerate(0) {}
        };
        
        class Lane;
        
        class GnssPoint2D : public geometry::Point2D
        {
        public:
            GnssPoint2D(double x,double y,double yaw,GnssInfo info=GnssInfo()):Point2D(x,y,yaw),m_info(info),root(nullptr) {}
            
            std::vector<GnssPoint2D> buffer(const std::vector<GnssPoint2D> &target,double radius)
            {
                std::vector<GnssPoint2D> result;
                for (const GnssPoint2D &pt : target)
                    if (this->distance(pt) < radius)
                        result.push_back(pt);
                return result;
            }
            GnssInfo m_info;
            
            std::shared_ptr<Lane> root;
            //Lane * root;
        };
        
        typedef std::vector<GnssPoint2D> GnssVec;
        
        static double calLength(const GnssVec &vec)
        {
            double result=0.0;
            for(int i=1;i<vec.size();i++)
                result+=vec[i-1].distance(vec[i]);
            return result;
        }
        
    }
}


#endif //MAPTOOLS_GNSSPOINT2D_H
