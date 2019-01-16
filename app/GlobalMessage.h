//
// Created by books on 2018/5/24.
//

#ifndef MAPTOOLS_GLOBALMESSAGE_H
#define MAPTOOLS_GLOBALMESSAGE_H

#include "Common.h"

MapBase::GnssVec g_global_map;
std::vector<int> g_road_nodes_id;
void InitGlobalMap(const MapBase::RoadVec &vroad,const std::vector<int> &roadids)
{
    g_global_map.clear();
    g_road_nodes_id.clear();

    MapBase::DouglasPeucker dp(10.0,20);
    for (const int &roadid : roadids)
    {
        for (const MapBase::Road &road : vroad)
        {
            if (road.roadid == roadid && !road.vlane.empty())
            {
                MapBase::Lane lane=road.vlane.front();
                MapBase::GnssVec tmp_global_map=dp.Process(lane.vec);
                g_global_map.insert(g_global_map.end(),tmp_global_map.begin(),tmp_global_map.end());
                g_road_nodes_id.push_back(tmp_global_map.front().id);
                g_road_nodes_id.push_back(tmp_global_map.back().id);
            }
        }
    }

}

double AngleDiff(double angle1,double angle2)
{
    double diff = std::abs(angle1 - angle2);
    if (diff > M_PI)
        diff = 2 * M_PI - diff;
    return diff;
}

bool NodeContain(int id)
{
    for (const int &node_id : g_road_nodes_id)
    {
        if (node_id == id)
            return true;
    }
    return false;
}

void GetGlobalMessage()
{
    int dis_index=-1;
    int angle_index=-1;
    double min_dis_dis = std::numeric_limits<double>::max();
    double min_dis_angle=std::numeric_limits<double>::max();
    for (std::size_t i = 0; i <g_global_map.size(); i++)
    {
        MapBase::GnssData gd = g_global_map[i];
        double tmp_dis = cur_pos.Distance(gd);

        double tmp_yaw = MapBase::Degree2Grad(gd.yaw);
        double pos_yaw = MapBase::Degree2Grad(cur_pos.yaw);
        double pos_tmp_yaw = std::atan2(gd.y - cur_pos.y, gd.x - cur_pos.x);
        if (pos_tmp_yaw < 0)
            pos_tmp_yaw += 2 * M_PI;

        double anglediff1 = AngleDiff(tmp_yaw, pos_tmp_yaw) * 180.0 / M_PI;
        double anglediff2 = AngleDiff(pos_tmp_yaw, pos_yaw) * 180.0 / M_PI;

        if (anglediff1 < 45 && anglediff2 < 45)
        {
            if (tmp_dis < min_dis_dis)
            {
				min_dis_dis = tmp_dis;
                angle_index = i;
            }
        }

        if (tmp_dis > 100)
        {
            if (tmp_dis < min_dis_angle)
            {
                min_dis_angle = tmp_dis;
                dis_index = i;
            }
        }
    }

    int index = (angle_index!=-1)?angle_index:((dis_index!=-1)?dis_index:-1);
    if(index==-1)
    {
        SendEmptyGlobal();
        return;
    }

    std::vector<ckLcmType::ckPointXYAA_t> lane_msg;
    std::vector<double> vx,vy;
    for(int i=index;i<g_global_map.size();i++)
    {

        MapBase::GnssData gd = g_global_map[i];
        ckLcmType::ckPointXYAA_t pt;
        pt.id = gd.id;
        pt.x = gd.x;
        pt.y = gd.y;
        pt.yaw = gd.yaw;
        pt.attr = (NodeContain(gd.id)) ? 1 : 0;
        lane_msg.push_back(pt);

        MapBase::PointXYA local_pt = gd.ToLocal(cur_pos);
        vx.push_back(local_pt.x);
        vy.push_back(local_pt.y);
    }

    ckLcmType::GlobalMap_t globalMsg;
    globalMsg.pointnum=(int)lane_msg.size();
    globalMsg.path.assign(lane_msg.begin(),lane_msg.end());
    global_handler->SendLcm(&globalMsg);

    ckLcmType::Draw_t draw_msg;
    draw_msg.ptnum=(int)vx.size();
    draw_msg.x=vx;
    draw_msg.y=vy;
    draw_global_handler->SendLcm(&draw_msg);

}

#endif //MAPTOOLS_GLOBALMESSAGE_H
