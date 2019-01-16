//
// Created by books on 2018/3/23.
//

#include "Common.h"
#include "LcmType/ckChangeLane_t.hpp"
#include "LaneMessage.h"
#include "ReverseMessage.h"
#include "GlobalMessage.h"
#include "VelocityMessage.h"

#include <DouglasPeucker.h>
#include <Cluster.h>

typedef LcmHandler<ckLcmType::ckCarLight>::Ptr CarLightHandlerPtr;

int main(int argc,char **argv)
{
    InitialLcmHandler();

    std::string model_dir = (argc > 1) ? std::string(argv[1]) : "Model";
    std::vector<std::string> vfiles = MapBase::GetFiles(model_dir);
    MapBase::LaneVec vlane;
    MapBase::DouglasPeucker douglas_peucker;
    int laneid = 0;
    for (const std::string &file : vfiles)
    {
        MapBase::GnssVec la;
        bool flag = MapBase::ReadGnssWithoutID(file, la);

        if (flag)
        {
            MapBase::GnssVec tmp = douglas_peucker.Process(la);
            MapBase::Lane lane;
            lane.laneid = laneid++;
            lane.vec.swap(tmp);
            vlane.push_back(lane);
        }
    }

    MapBase::Road ro;
    ro.roadid = 0;
    ro.vlane.swap(vlane);
    ro.reverseId = 1;
    ro.Initilize();

    MapBase::RoadVec staticRoadVec;
    staticRoadVec.push_back(ro);

    std::string reverse_dir = "Reverse";
    vlane.clear();
    vfiles.clear();
    vfiles = MapBase::GetFiles(reverse_dir);
    for (const std::string &file : vfiles)
    {
        MapBase::GnssVec la;
        bool flag = MapBase::ReadGnssWithoutID(file, la);

        if (flag)
        {
            MapBase::GnssVec tmp = douglas_peucker.Process(la);
            MapBase::Lane lane;
            lane.laneid = laneid++;
            lane.vec.swap(tmp);
            vlane.push_back(lane);
        }
    }

    if(!vlane.empty())
    {
        MapBase::Road reverse_ro;
        reverse_ro.roadid = 1;
        reverse_ro.vlane.swap(vlane);
        reverse_ro.reverseId = 0;
        reverse_ro.Initilize();
        staticRoadVec.push_back(reverse_ro);
    }

    std::vector<int> roadids;
    for(const MapBase::Road &road : staticRoadVec)
        roadids.push_back(ro.roadid);
    InitGlobalMap(staticRoadVec, roadids);

    MapBase::Cluster staticCluster;
    MapBase::ConnVec staticConnVec;
    staticCluster.Load(staticRoadVec, staticConnVec);
    staticCluster.Create();
    bool staticclusterflag = staticCluster.Check();
    if (!staticclusterflag)
    {
        std::cerr << "Static Spatial Cluster is incorrect!" << std::endl;
        return -1;
    }

    /// change lane signal
    std::cout << "Wait for car light signal..." << std::endl;
    bool update_change_lane = true;
//    std::thread tupdate_change_lane = std::thread([&]() {
//        CarLightHandlerPtr car_light_handler(new LcmHandler<ckLcmType::ckCarLight>());
//        car_light_handler->SetNet("udpm://238.255.76.67:7667?ttl=1");
//        car_light_handler->SetChannel("CKCARLIGHT");
//        car_light_handler->InitialListen();
//        while (update_change_lane)
//        {
//            ckLcmType::ckCarLight change_lane_msg;
//            car_light_handler->GetData(change_lane_msg);
//
//            std::unique_lock<std::mutex> lock(g_change_lane_mutex);
//            unsigned char curLight = (unsigned char) change_lane_msg.light;
//            g_change_lane = CHANGE_NONE;
//            if (lightOn(LEFT_TURN_LIGHT, curLight))
//                g_change_lane = CHANGE_LEFT;
//            if (lightOn(RIGHT_TURN_LIGHT, curLight))
//                g_change_lane = CHANGE_RIGHT;
//            if (lightOn(HAZARD_WARNING_LIGHT, curLight))
//                g_change_lane = CHANGE_NONE;
//            /// if turn light been close, then reset the g_change_lane_id
//            g_change_lane_id = (g_change_lane == CHANGE_NONE) ? -1 : g_change_lane_id;
//            lock.unlock();
//        }
//    });
		std::thread tupdate_change_lane=std::thread([&](){
			typedef LcmHandler<ckLcmType::ckChangeLane_t>::Ptr ChangeLanePtr;
			ChangeLanePtr change_lane_handler(new LcmHandler<ckLcmType::ckChangeLane_t>());
			change_lane_handler->SetNet("udpm://238.255.76.67:7667?ttl=1");
			change_lane_handler->SetChannel("CKCHANGELANE");
			change_lane_handler->InitialListen();

			while(update_change_lane)
			{
				ckLcmType::ckChangeLane_t change_lane_msg;
				change_lane_handler->GetData(change_lane_msg);
				std::unique_lock<std::mutex> lock(g_change_lane_mutex);

				if (change_lane_msg.change_signal == 1)
					g_change_lane = CHANGE_LEFT;
				else if (change_lane_msg.change_signal == 2)
					g_change_lane = CHANGE_RIGHT;
				else
					g_change_lane = CHANGE_NONE;

				g_change_lane_id = (g_change_lane == CHANGE_NONE) ? -1 : g_change_lane_id;

				lock.unlock();
			}
		});



    std::cout << "Wait for location message..." << std::endl;
    LocHandlerPtr lochandler(new LcmHandler<ckLcmType::Location_t>);
    lochandler->SetNet("udpm://238.255.76.67:7667?ttl=64");
    lochandler->SetChannel("CKLOCATION");
    lochandler->InitialListen();
    ckLcmType::Location_t locmes;
    lochandler->GetData(locmes);
    std::cout << "Receive location message!" << std::endl;
    cur_pos = MapBase::PointXYA(locmes.gau_pos[0], locmes.gau_pos[1], locmes.orientation[2]);

    /// esr signal
    std::cout<<"Wait for esr message..."<<std::endl;
    bool update_esr=true;
    std::thread tupdate_esr=std::thread([&](){
        LcmHandler<ckLcmType::ESR_t>::Ptr esr_handler(new LcmHandler<ckLcmType::ESR_t>());
        esr_handler->SetChannel("ESR");
        esr_handler->InitialListen();

        while(update_esr)
        {
            ckLcmType::ESR_t esr_msg;
            esr_handler->GetData(esr_msg);

            std::unique_lock<std::mutex> lock(g_esr_mutex);
            MapBase::PointXYA tmp_cur_pos(cur_pos.x,cur_pos.y,cur_pos.yaw);
            for(int i=0;i<(int)esr_msg.TargetCount;i++)
            {
                ckLcmType::Target_t target=esr_msg.Targets[i];
                g_esr_vec.push_back(EsrPt(tmp_cur_pos,target));
            }
            lock.unlock();
        }
    });

	std::thread th_velocity=std::thread([&]{
		std::cout<<"Start Velocity Control."<<std::endl;
		while(true)
		{
			GetVelocity();
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	});

    MapBase::Log log(2);
    log.Start();
    while (true)
    {
        std::string word;
        std::chrono::steady_clock::time_point stp = std::chrono::steady_clock::now();

        lochandler->GetData(locmes);
        cur_pos = MapBase::PointXYA(locmes.gau_pos[0], locmes.gau_pos[1], locmes.orientation[2]);

        /************Spatial Search**********/
        bool flag = staticCluster.SpatialSearch(cur_pos, g_match);

        log.Ends("Spatial search result : ");
        log.Ends("road id = " + std::to_string(g_match.roadid));
        log.Ends("lane id = " + std::to_string(g_match.laneid));
        log.Ends("conn id = " + std::to_string(g_match.connid));
		log.Endl("target lane index = " + std::to_string(g_change_lane_id));

        if (flag)
        {
            std::thread sendroadthread = std::thread(GetLaneMessage, staticRoadVec);
            //std::thread sendreversethread = std::thread(GetReverseMessage, staticRoadVec);

            sendroadthread.join();
           // sendreversethread.join();

            SendEmptyGlobal();
            SendEmptyReverse();
        }
        else
        {
            log.Endl(word += "Can't find match map point in current clusters!");
            SendEmptyMap();
            SendEmptyReverse();
            std::thread sendglobalthread = std::thread(GetGlobalMessage);
            sendglobalthread.join();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

//        LogStatus(word);
        log.Update();

        std::chrono::steady_clock::time_point etp = std::chrono::steady_clock::now();
        std::chrono::milliseconds span = std::chrono::duration_cast<std::chrono::milliseconds>(etp - stp);
        if (span.count() < 20)
            std::this_thread::sleep_for(std::chrono::milliseconds(20 - span.count()));
    }
    log.Stop();

}
