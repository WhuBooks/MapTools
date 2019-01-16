//
// Created by books on 2018/6/15.
//

#ifndef MAPTOOLS_VELOCITYMESSAGE_H
#define MAPTOOLS_VELOCITYMESSAGE_H

#include "Common.h"

const double g_recommend_velocity=30.0;

std::mutex g_esr_mutex;
struct EsrPt:public MapBase::PointXYA
{
    double velo_x;
    double velo_y;
    EsrPt(const MapBase::PointXYA &refer,const ckLcmType::Target_t &msg)
    {
        MapBase::PointXYA local(msg.CoorX,msg.CoorY,refer.yaw);
        MapBase::PointXYA world=local.ToWorld(refer);

        x=world.x;
        y=world.y;
        yaw=world.yaw;
        velo_x=msg.RangeRateX;
        velo_y=msg.RangeRateY;
    }
};
std::vector<EsrPt> g_esr_vec;


void GetVelocity()
{
    double tmp_velocity=g_recommend_velocity;

    MapBase::PointXYA tmp_pos(cur_pos.x,cur_pos.y,cur_pos.yaw);
    std::vector<EsrPt> tmp_esr_vec;
    {
        std::unique_lock<std::mutex> lock(g_esr_mutex);
        tmp_esr_vec.assign(g_esr_vec.begin(),g_esr_vec.end());
        lock.unlock();
    }

    double esr_factor=1.0;
    for(const EsrPt &esr_pt : tmp_esr_vec)
    {
        MapBase::PointXYA local=esr_pt.ToLocal(tmp_pos);
        if(std::abs(local.x)<2.0)
        {
            if(local.y<60)
            {
                esr_factor=0.5;
            }
        }
    }
    tmp_velocity=tmp_velocity*esr_factor;

    ckLcmType::ckVelocity_t velocity_msg;
    velocity_msg.velocity=tmp_velocity;
    velocity_handler->SendLcm(&velocity_msg);

#ifdef USEDEBUG
    std::cout<<"Current Velocity ~ "<<tmp_velocity<<std::endl;
#endif

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}


#endif //MAPTOOLS_VELOCITYMESSAGE_H
