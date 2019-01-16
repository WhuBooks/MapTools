//
// Created by 98420 on 2018/8/31.
//

#ifndef MAPTOOLS_GRIDCLUSTER_HPP
#define MAPTOOLS_GRIDCLUSTER_HPP

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "GridCluster.h"

namespace maplib
{
    namespace cluster
    {
        const int PatchNum=10;
        const double BufferSize=5.0;
        
        template<typename T>
        GridCluster<T>::GridCluster(const std::vector<T> &pts)
        {
            loadAllPts(pts);
        }
    
        template<typename T>
        void GridCluster<T>::loadAllPts(const std::vector<T> &pts)
        {
            m_pts.assign(pts.begin(),pts.end());
            m_patches.clear();
            
            double xmin=std::numeric_limits<double>::max();
            double ymin=std::numeric_limits<double>::max();
            double xmax=-std::numeric_limits<double>::max();
            double ymax=-std::numeric_limits<double>::max();
            
            for(const T &pt : pts)
            {
                double pt_x = pt.m_x;
                double pt_y = pt.m_y;
    
                xmin = xmin > pt_x ? pt_x : xmin;
                xmax = xmax < pt_x ? pt_x : xmax;
                ymin = ymin > pt_y ? pt_y : ymin;
                ymax = ymax < pt_y ? pt_y : ymax;
            }
            
            double patch_width=(xmax-xmin)/PatchNum;
            double patch_height=(ymax-ymin)/PatchNum;
            for(int i=0;i<=PatchNum;i++)
            {
                for(int j=0;j<=PatchNum;j++)
                {
                    double cen_x=xmin+(j+0.5)*patch_width;
                    double cen_y=ymin+(j+0.5)*patch_height;
                    Patch tmp_patch(T(cen_x,cen_y,0.0),patch_width+2.0*BufferSize,patch_height+2.0*BufferSize);
                    m_patches.push_back(tmp_patch);
                }
            }
            
            
            for(const T &pt : pts)
            {
                for(Patch &patch : m_patches)
                {
                    if(patch.contain(pt))
                        patch.data.push_back(pt);
                }
            }
        }
    
        template<typename T>
        bool GridCluster<T>::spatialSearch(const T &target, T &match, bool useYaw)
        {
            /// point to patch
            std::vector<T> patch_data;
            double min_patch_dis=std::numeric_limits<double>::max();
            for(const Patch &patch : m_patches)
            {
                if(patch.contain(target))
                {
                    double tmp_patch_dis=patch.distance(target);
                    if(tmp_patch_dis<min_patch_dis)
                    {
                        min_patch_dis=tmp_patch_dis;
                        patch_data.assign(patch.data.begin(),patch.data.end());
                    }
                }
            }
            if(patch_data.empty())
                return false;
            
            /// point to buffer
            std::vector<T> target_buff=target.buffer(patch_data,BufferSize);
            if(target_buff.empty())
                return false;
            
            double min_dis=std::numeric_limits<double>::max();
            bool find=false;
            for(const T &pt : target_buff)
            {
                if(useYaw && pt.yawDiff(target)>45)
                    continue;
                
                find=true;
                double tmp_dis=pt.distance(target);
                if(tmp_dis<min_dis)
                {
                    min_dis = tmp_dis;
                    match = pt;
                }
            }
            return find;
        }
    }
}

#endif //MAPTOOLS_GRIDCLUSTER_HPP
