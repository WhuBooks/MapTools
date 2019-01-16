//
// Created by 98420 on 2018/8/30.
//

#include "Point2D.h"
#include <Util/AngleUtil.h>

namespace maplib
{
    namespace geometry
    {
        Point2D::Point2D(double x, double y, double yaw) : GeometryBase()
        {
            m_x = x;
            m_y = y;
            m_yaw = yaw;
            
            m_yaw_sin = std::sin(util::degree2Grad(yaw));
            m_yaw_cos = std::cos(util::degree2Grad(yaw));
        }
        
        void Point2D::pan(double _dx, double _dy)
        {
            m_x += _dx;
            m_y += _dy;
        }
    
        Point2D Point2D::shift(double right, double up)
        {
            double x = m_x - right * m_yaw_sin + up * m_yaw_cos;
            double y = m_y + right * m_yaw_cos + up * m_yaw_sin;
            return Point2D(x, y, m_yaw);
        }
        
        double Point2D::distance(const Point2D &pt) const
        {
            double dx = std::abs(pt.m_x - m_x);
            double dy = std::abs(pt.m_y - m_y);
            return std::sqrt(dx * dx + dy * dy);
        }
        
        bool Point2D::sameDirection(const Point2D &pt) const
        {
            return (m_yaw_sin * pt.m_yaw_sin + m_yaw_cos * pt.m_yaw_cos > 0.0);
        }
    
        double Point2D::yawDiff(const Point2D &pt) const
        {
            return util::degreeDiff(m_yaw,pt.m_yaw);
        }
        
        bool Point2D::isBehind(const Point2D &pt) const
        {
            double dx = pt.m_x - m_x;
            double dy = pt.m_y - m_y;
            double azi = std::atan2(dy, dx);
            
            double azi_sin = std::sin(azi);
            double azi_cos = std::cos(azi);
            
            return (azi_sin * m_yaw_sin + azi_cos * m_yaw_cos) > 0.0;
        }
    
        Point2D Point2D::toLocal(const Point2D &refer) const
        {
            //get vector (local origin -> this) and local coordinate of this
            double deltax = m_x - refer.m_x;
            double deltay = m_y - refer.m_y;
            double xlocal = deltax * (-refer.m_yaw_sin) + deltay * refer.m_yaw_cos;
            double ylocal = deltax * refer.m_yaw_cos + deltay * refer.m_yaw_sin;
            double yawnew = m_yaw;
            return Point2D(xlocal, ylocal, yawnew);
        }
    
        Point2D Point2D::toWorld(const Point2D &refer) const
        {
            //calculate the local coordinate of world origin
            double deltax = -refer.m_x;
            double deltay = -refer.m_y;
            double xorigin = deltax * (-refer.m_yaw_sin) + deltay * refer.m_yaw_cos;
            double yorigin = deltax * refer.m_yaw_cos + deltay * refer.m_yaw_sin;
            
            //get vector (world origin -> this) and world coordinate of this
            deltax = m_x - xorigin;
            deltay = m_y - yorigin;
            double xworld = deltax * (-refer.m_yaw_sin) + deltay * refer.m_yaw_cos;
            double yworld = deltax * refer.m_yaw_cos + deltay * refer.m_yaw_sin;
            double yawworld = m_yaw;
            return Point2D(xworld, yworld, yawworld);
        }
    
        Point2D Point2D::reverseDir() const
        {
            return Point2D(m_x, m_y, util::degree(m_yaw + 180.0));
        }
        
        double Point2D::verticalDistance(const Point2D &pt_s, const Point2D &pt_e) const
        {
            double A=pt_e.m_y-pt_s.m_y;
            double B=pt_s.m_x-pt_e.m_x;
            double C=-(pt_s.m_y*B+pt_s.m_x*A);
            double dis=std::abs((A*m_x+B*m_y+C)/std::sqrt(A*A+B*B));
            return dis;
        }
    }
}