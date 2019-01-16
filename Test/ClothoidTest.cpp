//
// Created by books on 2018/5/17.
//
#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <thread>
#include <mutex>

#include <Geometry.h>
#include <RoadModel.h>
#include <Clothoid.h>
#include <FileIO.h>
#include <DouglasPeucker.h>

#include <LcmType/LcmHandler.h>
#include <LcmType/Draw_t.hpp>

typedef LcmHandler<ckLcmType::Draw_t>::Ptr DrawHandlerPtr;

bool Smooth(MapBase::GnssVec &conn,const MapBase::PointXYA &s_back,const MapBase::PointXYA &e_front)
{
    MapBase::Clothoid clothoid;

    MapBase::GnssVec smooth;
    for(int i=0;i<conn.size();i++)
    {
        MapBase::GnssData gd_i=conn[i];
        MapBase::PointXYA gd_i_local=gd_i.ToLocal(s_back);
        if(std::abs(gd_i_local.y)<3 *std::abs(gd_i_local.x) || gd_i_local.y<0)
            continue;

        MapBase::PtXYAVec tmp_smmoth_s=clothoid.Build(s_back,gd_i,50,0.05);
        if(!tmp_smmoth_s.empty())
        {
            for(const MapBase::PointXYA &pt : tmp_smmoth_s)
            {
                MapBase::GnssData gd_pt;
                gd_pt.x=pt.x;
                gd_pt.y=pt.y;
                gd_pt.yaw=pt.yaw;
                gd_pt.CalYawSinAndCos();
                smooth.push_back(gd_pt);
            }
            smooth.insert(smooth.end(),conn.begin()+i+1,conn.end());
            break;
        }
    }

    if(smooth.empty())
        return false;

    for(int i=smooth.size()-1;i>=0;i--)
    {
        MapBase::GnssData gd_i=smooth[i];
        MapBase::PointXYA gd_i_local=gd_i.ToLocal(e_front);
        if(std::abs(gd_i_local.y)<3*std::abs(gd_i_local.x) || gd_i_local.y>0)
            continue;

        MapBase::PtXYAVec tmp_smooth_e=clothoid.Build(gd_i,e_front,50,0.05);
        if(!tmp_smooth_e.empty())
        {
            MapBase::GnssVec result;
            result.assign(smooth.begin(),smooth.begin()+i-1);
            for(const MapBase::PointXYA &pt : tmp_smooth_e)
            {
                MapBase::GnssData gd_pt;
                gd_pt.x=pt.x;
                gd_pt.y=pt.y;
                gd_pt.yaw=pt.yaw;
                gd_pt.CalYawSinAndCos();
                result.push_back(gd_pt);
            }
            conn.swap(result);
            return true;
        }
    }
    return false;
}

ckLcmType::Draw_t Transform(const MapBase::GnssVec &vec,const MapBase::GnssData &pt)
{
    int num=0;
    std::vector<double> x,y;
    for(const MapBase::GnssData &gd : vec)
    {
        MapBase::PointXYA pt_local=gd.ToLocal(pt);
        x.push_back(pt_local.x-5);
        y.push_back(pt_local.y-5);
        num++;
    }
    ckLcmType::Draw_t msg;
    msg.ptnum=num;
    msg.x.swap(x);
    msg.y.swap(y);
    return msg;
}

ckLcmType::Draw_t Transform2(const MapBase::GnssVec &vec,const MapBase::GnssData &pt)
{
    int num=0;
    std::vector<double> x,y;
    for(const MapBase::GnssData &gd : vec)
    {
        MapBase::PointXYA pt_local=gd.ToLocal(pt);
        x.push_back(pt_local.x+5);
        y.push_back(pt_local.y+5);
        num++;
    }
    ckLcmType::Draw_t msg;
    msg.ptnum=num;
    msg.x.swap(x);
    msg.y.swap(y);
    return msg;
}

int main(int argc,char **argv)
{
    std::string model_dir = (argc > 1) ? std::string(argv[1]) : "../dataset/Model";

    MapBase::RoadVec road_vec;
    MapBase::ReadRoad(model_dir, road_vec);
    std::cout<<"Road Size ~ "<<road_vec.size()<<std::endl;

    MapBase::ConnVec conn_vec;
    MapBase::ReadConline(model_dir, conn_vec);
    std::cout<<"Conn Size ~ "<<conn_vec.size()<<std::endl;

    ckLcmType::Draw_t origin_s_msg,origin_c_msg,origin_e_msg;
    origin_s_msg.ptnum=0;
    origin_c_msg.ptnum=0;
    origin_e_msg.ptnum=0;
    std::mutex origin_mutex;
    bool origin_msg_flag=true;
    std::thread send_origin([&](){
        DrawHandlerPtr draw_origin_s_handler(new LcmHandler<ckLcmType::Draw_t>());
        draw_origin_s_handler->SetNet("udpm://238.255.76.67:7667?ttl=128");
        draw_origin_s_handler->SetChannel("CKORIGINSDRAW");
        draw_origin_s_handler->InitialSend();

        DrawHandlerPtr draw_origin_c_handler(new LcmHandler<ckLcmType::Draw_t>());
        draw_origin_c_handler->SetNet("udpm://238.255.76.67:7667?ttl=128");
        draw_origin_c_handler->SetChannel("CKORIGINCDRAW");
        draw_origin_c_handler->InitialSend();

        DrawHandlerPtr draw_origin_e_handler(new LcmHandler<ckLcmType::Draw_t>());
        draw_origin_e_handler->SetNet("udpm://238.255.76.67:7667?ttl=128");
        draw_origin_e_handler->SetChannel("CKORIGINEDRAW");
        draw_origin_e_handler->InitialSend();

        std::cout<<"Start Send Origin Message."<<std::endl;
        while(origin_msg_flag)
        {
            std::unique_lock<std::mutex> lock(origin_mutex);
            draw_origin_s_handler->SendLcm(&origin_s_msg);
            draw_origin_c_handler->SendLcm(&origin_c_msg);
            draw_origin_e_handler->SendLcm(&origin_e_msg);
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    ckLcmType::Draw_t smooth_s_msg,smooth_c_msg,smooth_e_msg;
    smooth_c_msg.ptnum=0;
    smooth_e_msg.ptnum=0;
    smooth_s_msg.ptnum=0;
    std::mutex smooth_mutex;
    bool smooth_msg_flag=true;
    std::thread send_smooth([&]{
        DrawHandlerPtr smooth_s_handler(new LcmHandler<ckLcmType::Draw_t>());
        smooth_s_handler->SetNet("udpm://238.255.76.67:7667?ttl=64");
        smooth_s_handler->SetChannel("CKSMOOTHSDRAW");
        smooth_s_handler->InitialSend();

        DrawHandlerPtr smooth_c_handler(new LcmHandler<ckLcmType::Draw_t>());
        smooth_c_handler->SetNet("udpm://238.255.76.67:7667?ttl=64");
        smooth_c_handler->SetChannel("CKSMOOTHCDRAW");
        smooth_c_handler->InitialSend();

        DrawHandlerPtr smooth_e_handler(new LcmHandler<ckLcmType::Draw_t>());
        smooth_e_handler->SetNet("udpm://238.255.76.67:7667?ttl=64");
        smooth_e_handler->SetChannel("CKSMOOTHEDRAW");
        smooth_e_handler->InitialSend();

        std::cout<<"Start Send Smooth Message."<<std::endl;
        while(smooth_msg_flag)
        {
            std::unique_lock<std::mutex> lock(smooth_mutex);
            smooth_s_handler->SendLcm(&smooth_s_msg);
            smooth_c_handler->SendLcm(&smooth_c_msg);
            smooth_e_handler->SendLcm(&smooth_e_msg);
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(1));

    /// calculate the link error between
    for(MapBase::Connline &conn : conn_vec)
    {
        MapBase::Road road_s,road_e;
        int s_id=conn.sroadid;
        int e_id=conn.eroadid;

        for(const MapBase::Road &road : road_vec)
        {
            if(road.roadid==s_id)
                road_s=road;
            else if(road.roadid==e_id)
                road_e=road;
        }

        if(!road_s.vlane.empty()&&!road_e.vlane.empty())
        {
            MapBase::GnssVec gnss_vec_s;
            double min_s_local_x = 100000000000;
            int s_lane_id = -1;
            for (const MapBase::Lane &lane : road_s.vlane)
            {
                MapBase::GnssData lane_back = lane.vec.back();
                MapBase::GnssData conn_front = conn.data.front();
                MapBase::PointXYA conn_front_local = conn_front.ToLocal(lane_back);
                double local_x = std::abs(conn_front_local.x);
                if (local_x < min_s_local_x)
                {
                    min_s_local_x = local_x;
                    s_lane_id = lane.laneid;
                    gnss_vec_s.assign(lane.vec.end()-20,lane.vec.end());
                }
            }

            MapBase::GnssVec gnss_vec_e;
            double min_e_local_x = 100000000000;
            int e_lane_id = -1;
            for (const MapBase::Lane &lane : road_e.vlane)
            {
                MapBase::GnssData lane_front = lane.vec.front();
                MapBase::GnssData conn_back = conn.data.back();
                MapBase::PointXYA conn_back_local = conn_back.ToLocal(lane_front);
                double local_x = std::abs(conn_back_local.x);
                if (local_x < min_e_local_x)
                {
                    min_e_local_x = local_x;
                    e_lane_id = lane.laneid;
                    gnss_vec_e.assign(lane.vec.begin(),lane.vec.begin()+20);
                }
            }

            MapBase::GnssData pt=conn.data.front();
            {
                std::unique_lock<std::mutex> lock(origin_mutex);

                origin_s_msg=Transform(gnss_vec_s,pt);
                origin_c_msg=Transform(conn.data,pt);
                origin_e_msg=Transform(gnss_vec_e,pt);

                lock.unlock();
            }

//            MapBase::GnssVec conn_data=conn.data;
           // bool smooth=Smooth(conn_data,gnss_vec_s.back(),gnss_vec_e.front());
            {
                conn.Smooth(gnss_vec_s.back(),gnss_vec_e.front());
                MapBase::DouglasPeucker dp;
                MapBase::GnssVec tmp=dp.Process(conn.data);
                //if (smooth)
                {
                    std::unique_lock<std::mutex> lock(smooth_mutex);
                    smooth_s_msg = Transform2(gnss_vec_s, pt);
                    smooth_c_msg = Transform2(tmp, pt);
                    smooth_e_msg = Transform2(gnss_vec_e, pt);
                    lock.unlock();
                }
//                else
//                {
//                    std::cerr << "Can't Smooth." << std::endl;
//                    std::unique_lock<std::mutex> lock(smooth_mutex);
//                    smooth_s_msg.ptnum=0;
//                    smooth_s_msg.x.clear();
//                    smooth_s_msg.y.clear();
//                    smooth_c_msg.ptnum=0;
//                    smooth_c_msg.x.clear();
//                    smooth_c_msg.y.clear();
//                    smooth_e_msg.ptnum=0;
//                    smooth_e_msg.x.clear();
//                    smooth_e_msg.y.clear();
//                    lock.unlock();
//                }
            }


            std::cout << "Conn ID ~ " << conn.connid << "\t";
            std::cout << "Start Lane ~ " << s_lane_id << "\tDistance ~ " << min_s_local_x << "\t";
            std::cout << "End Lane ~ " << e_lane_id << "\tDistance ~ " << min_e_local_x << std::endl;

            getchar();

        }
        else
            std::cout<<"Conn ID ~ "<<conn.connid<<"\t"<<"Relation Error."<<std::endl;
        std::cout<<std::endl;
    }

    origin_msg_flag=false;
    smooth_msg_flag=false;
    send_origin.join();
    send_smooth.join();

    return 1;
}

