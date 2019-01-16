//
// Created by 98420 on 2018/8/30.
//

#include "Lane.h"

namespace maplib
{
    namespace model
    {
        void Lane::initialize()
        {
        
        }
        
        int Lane::leftNum(const Lane &other) const
        {
            int num=0;
            for(const GnssPoint2D &pt_cur : m_data)
            {
                bool on_left=false;
                double min_local_y=1000000;
                for(const GnssPoint2D &pt_other : m_data)
                {
                    geometry::Point2D pt_local=pt_cur.toLocal(pt_other);
                    if(std::abs(pt_local.m_y)<min_local_y)
                    {
                        min_local_y=std::abs(pt_local.m_y);
                        on_left=pt_local.m_x<0.0;
                    }
                }
                num=(on_left)?num+1:num;
            }
            return num;
        }
    
        double Lane::calLength() const
        {
            double result=0.0;
            for(int i=1;i<m_data.size();i++)
                result+=m_data[i-1].distance(m_data[i]);
            return result;
        }
    
        double Lane::calDensity() const
        {
            return calLength()/(double)m_data.size();
        }
    

    }
}