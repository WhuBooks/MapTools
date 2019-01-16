//
// Created by 98420 on 2018/8/30.
//

#ifndef MAPTOOLS_POINT2D_H
#define MAPTOOLS_POINT2D_H

#include <iostream>
#include "Geometry.h"

namespace maplib
{
    namespace geometry
    {
        class Point2D : public GeometryBase
        {
        public:
            Point2D(double x, double y, double yaw);
            
            ~Point2D() = default;
            
            void pan(double _dx, double _dy);
    
            Point2D shift(double right, double up);
            
            double distance(const Point2D &pt) const;
            
            bool sameDirection(const Point2D &pt) const;
            
            double yawDiff(const Point2D &pt) const;
            
            bool isBehind(const Point2D &pt) const;
            
            /// convert this point from world coordinate system to refer's local coordinate system
            /// local coordinate system : y point to front, x point to right, not change azimuth
            Point2D toLocal(const Point2D &) const;
            
            /// convert this point from refer's local coordinate system to world coordinate system
            /// world coordinate system : x point to north, y point to east, azimuth is the clockwise angle from x axis
            Point2D toWorld(const Point2D &) const;
    
            Point2D reverseDir() const;
            
            double verticalDistance(const Point2D &pt_s,const Point2D &pt_e)const;
            
            bool operator==(const Point2D &pt) const
            {
                return m_x == pt.m_x && m_y == pt.m_y && m_yaw == pt.m_yaw;
            }
            
            bool operator>(const Point2D &pt) const
            {
                return m_x > pt.m_x || (m_x == pt.m_x && m_y > pt.m_y) || (m_x == pt.m_x && m_y == pt.m_y && m_yaw > pt.m_yaw);
            }
            
            bool operator<(const Point2D &pt) const
            {
                return m_x < pt.m_x || (m_x == pt.m_x && m_y < pt.m_y) || (m_x == pt.m_x && m_y == pt.m_y && m_yaw < pt.m_yaw);
            }
            
            double m_x;
            double m_y;
            double m_yaw;
            
            double m_yaw_sin;
            double m_yaw_cos;
        };
        typedef std::vector<Point2D> Pt2DVec;
    }
}


#endif //MAPTOOLS_POINT2D_H
