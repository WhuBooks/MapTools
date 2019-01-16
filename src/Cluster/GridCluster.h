//
// Created by 98420 on 2018/8/31.
//

#ifndef MAPTOOLS_GRIDCLUSTER_H
#define MAPTOOLS_GRIDCLUSTER_H

#include <iostream>
#include <vector>
#include <algorithm>

namespace maplib
{
    namespace cluster
    {
        
        template<typename T>
        class GridCluster{
    
            struct Patch
            {
                T center;
                double width;
                double height;
                std::vector<T> data;
                Patch()= default;
                Patch(const T &_cen,double _width,double _height):center(_cen),width(_width),height(_height){}
                
                bool contain(const T &pt)const{
                    return std::abs(pt.m_x-center.m_x)<=width/2.0&&std::abs(pt.m_y-center.m_y)<=height/2.0;
                }
                
                double distance(const T &pt)const{
                    return pt.distance(center);
                }
            };
            
        public:
            GridCluster()= default;
            ~GridCluster()= default;
            GridCluster(const std::vector<T> &pts);
            void loadAllPts(const std::vector<T> &pts);
            
            bool spatialSearch(const T &target,T &match,bool useYaw=true);
        private:
            std::vector<Patch> m_patches;
            std::vector<T> m_pts;
        };
    }
}

#include "GridCluster.hpp"

#endif //MAPTOOLS_GRIDCLUSTER_H
