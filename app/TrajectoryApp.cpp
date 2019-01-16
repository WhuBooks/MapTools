//
// Created by books on 18-8-14.
//

#include "BlockMessage.h"

#include <LcmType/map_to_client.hpp>
#include <LcmType/client_to_map.hpp>

typedef nox_lcm::map_to_client M2CMsg;
typedef nox_lcm::client_to_map C2MMsg;
typedef LcmHandler<M2CMsg>::Ptr M2CHandlerPtr;
typedef LcmHandler<C2MMsg>::Ptr C2MHandlerPtr;

enum MapStatus
{
    MapPlanSucceed = 0, /// map plan succeed, please wait car's arrival
    MapPlanFailed = 1, /// map plan failed, please pick another target
    MapDriveInvoke = 2, /// unmanned drive stop by man's invoke
    MapWaited = 3     /// map stop and wait for next command
};

enum ClientCMD
{
    CMD_Unmanned = 0, /// umanned drive, will or already send a target
    CMD_Stop = 1,  /// stop at the current position, clear current target and wait for new command
    CMD_Manned = 2 /// manned drive
};

bool SameDir(const MapBase::PointXYA &pt1,const MapBase::PointXYA &pt2)
{
    double angle_diff=std::abs(pt1.yaw-pt2.yaw);
    if(angle_diff>180.0)
        angle_diff=MapBase::Degree(360.0-angle_diff);
    return angle_diff<30.0;
}


int main(int argc,char **argv)
{
    InitialLcmHandler();
    
    std::string modeldir = (argc == 1) ? "../dataset/Model" : std::string(argv[1]);
    
    MapBase::RoadVec full_road_vec;
    if (!MapBase::ReadRoad(modeldir, full_road_vec))
        return -1;
    for (MapBase::Road &road : full_road_vec)
        road.CalReverseId(full_road_vec);
    std::cout << "Road file has been read ~ " << full_road_vec.size() << std::endl;
    
    MapBase::ConnVec full_conn_vec;
    MapBase::ReadConline(modeldir, full_conn_vec);
    std::cout << "Connect line has been read ~ " << full_conn_vec.size() << std::endl;
    
    /// modify road and conn
    bool has_lane_connection=MapBase::ReadConlineLaneID(modeldir,full_conn_vec);
    for (MapBase::Connline &conn : full_conn_vec)
    {
        if(has_lane_connection)
            conn.ConnectLane(full_road_vec);
        else
            conn.InitLaneID(full_road_vec);
    }
    
    MapBase::SceneVec vscene;
    std::string scenefile = modeldir + "/Scene.txt";
    MapBase::ReadScene(scenefile, vscene);
    std::cout << "Scene file has been read ~ " << vscene.size() << std::endl;
    
    std::string traj_dir=(argc<=2)?"../dataset/Trajectory":std::string(argv[2]);
    std::vector<std::string> traj_files=MapBase::GetFiles(traj_dir);
    std::vector<MapBase::GnssVec> trajectories;
    for(const std::string tmp_str : traj_files)
    {
        MapBase::GnssVec vec;
        MapBase::ReadGnssWithoutID(tmp_str,vec);
        MapBase::DouglasPeucker dp(0.01,1.0);
        MapBase::GnssVec traj=dp.Process(vec);
        trajectories.push_back(traj);
    }
    std::cout<<"Read Trajectories ~ "<<trajectories.size()<<std::endl;
    
    MapBase::Cluster full_cluster;
    full_cluster.Load(full_road_vec, full_conn_vec);
    full_cluster.Create();
    bool clusterflag = full_cluster.Check();
    if (!clusterflag)
    {
        std::cerr << "Static Spatial Cluster is incorrect!" << std::endl;
        return -1;
    }
    
    //build path
    MapBase::Graph graph = MapBase::Graph(full_road_vec, full_conn_vec);
    graph.Initialize();
    std::cout << "Graph build done!" << std::endl;
    
    /// location msg
    Convert convert(LatZero, LonZero);
    std::cout << "Wait for location message..." << std::endl;
    GpsHandlerPtr gpsHandler(new LcmHandler<GpsMag>);
    gpsHandler->SetChannel("GPSData");
    gpsHandler->InitialListen();
    
    GpsMag gps_msg;
    std::mutex cur_pos_mutex;
    gpsHandler->GetData(gps_msg);
    {
        double x = 0.0, y = 0.0;
        convert.LL2XY(gps_msg.latitude, gps_msg.longitude, x, y);
        cur_pos = MapBase::PointXYA(x, y, gps_msg.heading);
    }
    std::cout << "Receive location message!" << std::endl;
    
    /// first match in full cluster, get partial cluster
    MapBase::Cluster partial_cluster;
    MapBase::RoadVec partial_road_vec;
    MapBase::ConnVec partial_conn_vec;
    
    /// target point
    MapBase::PointXYA target_point;
    std::function<bool(int,int)> FindTargetPoint=[&](int roadid,int ptid)->bool{
        for(const MapBase::Road &road : full_road_vec)
        {
            if(road.roadid==roadid)
                for(const MapBase::Lane &lane : road.vlane)
                    for(const MapBase::GnssData &gd : lane.vec)
                        if(gd.id==ptid)
                        {
                            target_point=MapBase::PointXYA(gd.x,gd.y,gd.yaw);
                            return true;
                        }
        }
        return false;
    };
    
    MapBase::GnssVec target_trajectory;
    std::function<bool(const MapBase::PointXYA &)> FindTraj=[&](const MapBase::PointXYA &pt)->bool{
        std::vector<MapBase::GnssVec> traj_candidate;
        for(const MapBase::GnssVec &traj : trajectories)
        {
            bool find_head=false;
            bool find_tail=false;
            MapBase::GnssVec tmp_traj;
            for(int i=0;i<traj.size();i++)
            {
                if(!find_head)
                {
                    if(traj[i].Distance(pt)<10 && SameDir(traj[i],pt))
                    {
                        find_head=true;
                    }
                }
                if(find_head && !find_tail)
                {
                    if(traj[i].Distance(target_point)<10 && SameDir(traj[i],target_point))
                    {
                        find_tail = true;
                        break;
                    }
                }
                
                if(find_head && !find_tail)
                {
                    tmp_traj.push_back(traj[i]);
                }
            }
            if(find_head && find_tail && tmp_traj.size()>30)
                traj_candidate.push_back(tmp_traj);
        }
        
        if(traj_candidate.empty())
            return false;
        
        std::sort(traj_candidate.begin(),traj_candidate.end(),[](const MapBase::GnssVec &traj1,const MapBase::GnssVec &traj2){
           return traj1.size()<traj2.size();
        });
        
        target_trajectory=traj_candidate.front();
        
        MapBase::Lane lane;
        lane.laneid=10000;
        lane.vec=target_trajectory;
        MapBase::Road tmp_road;
        tmp_road.roadid=100000;
        tmp_road.vlane.push_back(lane);
        
        partial_road_vec.clear();
        partial_conn_vec.clear();
        partial_road_vec.push_back(tmp_road);
        partial_cluster.Load(partial_road_vec,partial_conn_vec);
        
        
        return true;
    };
    
    long receive_c2m_times=0;
    MapStatus status=MapStatus::MapWaited;
    std::mutex status_mutex;
    std::thread status_thread=std::thread([&](){
        
        M2CHandlerPtr m2CHandler(new LcmHandler<M2CMsg>);
        m2CHandler->SetChannel("MapStatus");
        m2CHandler->InitialSend();
        
        while(true)
        {
            M2CMsg msg;
            {
                std::unique_lock<std::mutex> lock(status_mutex);
                msg.time_stamp=receive_c2m_times;
                msg.map_status = status;
                lock.unlock();
            }
            m2CHandler->SendLcm(&msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        
        
    });
    
    bool keep_driving=false;
    std::mutex g_keep_driving_mutex;
    std::thread app_thread=std::thread([&](){
        
        C2MHandlerPtr c2MHandler(new LcmHandler<C2MMsg>);
        c2MHandler->SetChannel("ClientCMD");
        c2MHandler->InitialListen();
        
        while(true)
        {
            C2MMsg c2MMsg;
            c2MHandler->GetData(c2MMsg);
            receive_c2m_times++;
            std::cout<<"Receive Times ~ "<<receive_c2m_times<<std::endl;
            
            MapBase::PointXYA tmp_pos;
            {
                std::unique_lock<std::mutex> lock(cur_pos_mutex);
                tmp_pos=MapBase::PointXYA(cur_pos.x,cur_pos.y,cur_pos.yaw);
                lock.unlock();
            }
            ClientCMD c2m_cmd = (ClientCMD) c2MMsg.cmd;
            int c2m_roadid = c2MMsg.roadId;
            int c2m_ptid = c2MMsg.pointId;
            
            /// unmanned drive cmd
            if (c2m_cmd == CMD_Unmanned)
            {
                /// hasn't select a target
                if (c2m_roadid == -1)
                {
                    /// send waited map status
                    {
                        std::unique_lock<std::mutex> lock(status_mutex);
                        std::cout << "Waiting for A New Target." << std::endl;
                        status=MapWaited;
                        lock.unlock();
                    }
                    
                    {
                        std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                        keep_driving=false;
                        lock.unlock();
                    }
                    
                    continue;
                }
                
                /// select a new target
                if (partial_road_vec.empty() || partial_conn_vec.empty())
                {
                    
                    MapBase::RoadPoint full_match;
                    bool full_match_flag = full_cluster.SpatialSearch(cur_pos, full_match);
                    std::cout<<"Global Match In ~ "<<full_match.roadid<<std::endl;
                    
                    /// replan
                    bool find_flag=FindTargetPoint(c2m_roadid,c2m_ptid);
//                    bool plan_flag = ReplanFunc(full_match.roadid, c2m_roadid);
                    bool traj_flag=FindTraj(tmp_pos);
                    
                    if(full_match_flag&&find_flag && traj_flag)
                    {
    
                        /// send plan succeed map status
                        {
                            std::unique_lock<std::mutex> lock(status_mutex);
                            std::cout << "RePlan and Find Succeed!" << std::endl;
                            status = MapPlanSucceed;
                            lock.unlock();
                        }
    
                        {
                            std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                            keep_driving = true;
                            lock.unlock();
                        }
    
                        continue;
                    }
                    else
                    {
                        
                        /// send plan failed map status
                        {
                            std::unique_lock<std::mutex> lock(status_mutex);
                            std::cout << "Replan Failed or Car is in the Fork or Target Point is Not In Map!" << std::endl;
                            status=MapPlanFailed;
                            lock.unlock();
                        }
                        
                        {
                            std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                            keep_driving=false;
                            lock.unlock();
                        }
                        
                        partial_cluster.Clear();
                        partial_road_vec.clear();
                        partial_conn_vec.clear();
                        
                        continue;
                    }
                    
                }
                else
                {
                    /// send plan succeed map status
                    {
                        std::unique_lock<std::mutex> lock(status_mutex);
                        std::cout<<"In Unmanned Driving!"<<std::endl;
                        status=MapPlanSucceed;
                        lock.unlock();
                    }
                    
                    {
                        std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                        keep_driving=true;
                        lock.unlock();
                    }
                    continue;
                }
            }
            else if (c2m_cmd == CMD_Stop)
            {
                
                /// send stop to plan module
                {
                    std::unique_lock<std::mutex> lock(status_mutex);
                    std::cout << "Mannuly Stop!" << std::endl;
                    if(partial_road_vec.empty() || partial_conn_vec.empty())
                        status=MapWaited;
                    else
                        status=MapPlanSucceed;
                    lock.unlock();
                }
                
                {
                    std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                    keep_driving=false;
                    lock.unlock();
                }
                
                continue;
            }
            else if (c2m_cmd == CMD_Manned)
            {
                /// send invoke to plan module, and reset partial set
                {
                    std::unique_lock<std::mutex> lock(status_mutex);
                    std::cout << "Mannuly Stop and Clear!" << std::endl;
                    status=MapWaited;
                    lock.unlock();
                }
                
                {
                    std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                    keep_driving=false;
                    lock.unlock();
                }
                
                
                partial_road_vec.clear();
                partial_conn_vec.clear();
                partial_cluster.Clear();
                
                continue;
            }
        }
        
    });
    
    MapBase::Log log(2);
    log.Start();
    
    int match_interval=50;
    int match_failed_time=0;
    int last_road_id=-1;
    int last_conn_id=-1;
    while (true)
    {
        std::chrono::steady_clock::time_point stp = std::chrono::steady_clock::now();
        
        gpsHandler->GetData(gps_msg);
        {
            double x = 0.0, y = 0.0;
            convert.LL2XY(gps_msg.latitude, gps_msg.longitude, x, y);
            {
                std::unique_lock<std::mutex> lock(cur_pos_mutex);
                cur_pos = MapBase::PointXYA(x, y, gps_msg.heading);
                lock.unlock();
            }
        }
        
        /// if can't match more than 10 times, then consume mannuly invoke
        if(match_failed_time>10)
        {
            /// arrive at target
            {
                std::unique_lock<std::mutex> lock(status_mutex);
                std::cout << "Mannuly Invoke!!! Need a New Target!" << std::endl;
                status=MapDriveInvoke;
                lock.unlock();
            }
            
            {
                std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                keep_driving=false;
                lock.unlock();
            }
            
            partial_road_vec.clear();
            partial_conn_vec.clear();
            partial_cluster.Clear();
            match_failed_time=5;
        }
        
        log.Ends("Current Position : ");
        log.Ends(std::to_string(cur_pos.x));
        log.Ends(std::to_string(cur_pos.y));
        log.Endl(std::to_string(cur_pos.yaw));
        
        /// if map status is stop or waited
        if(!keep_driving)
        {
            SendEmptyMap();
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }
        
        /// arrive at target
        if(g_match.Distance(target_point)<20.0 && g_match.SameDirection(target_point) && !partial_road_vec.empty())
        {
            {
                std::unique_lock<std::mutex> lock(status_mutex);
                std::cout << "Arrive at Target!!" << std::endl;
                status=MapWaited;
                lock.unlock();
            }
            
            {
                std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                keep_driving=false;
                lock.unlock();
            }
            
            receive_c2m_times++;
            
            partial_road_vec.clear();
            partial_conn_vec.clear();
            partial_cluster.Clear();
            
            continue;
        }
        
        
        /************Spatial Search**********/
        bool match_flag = partial_cluster.SpatialSearch(cur_pos, g_match);
        
        log.Ends("Spatial search result :");
        log.Ends("road id = " + std::to_string(g_match.roadid));
        log.Ends("lane id = " + std::to_string(g_match.laneid));
        log.Endl("conn id = " + std::to_string(g_match.connid));
        
        
        if (match_flag)
        {
            /// adjust match result
            for (const MapBase::Connline &conn : partial_conn_vec)
            {
                if (g_match.roadid != -1)
                {
                    /// if match in road, set conn as road's next conn
                    if (g_match.roadid == conn.sroadid)
                    {
                        g_match.connid = conn.connid;
                        break;
                    }
                }
                else
                {
                    /// if match in conn, set road as conn's next road
                    if (g_match.connid == conn.connid)
                    {
                        g_match.roadid = conn.eroadid;
                        break;
                    }
                }
            }
            
            log.Ends("Processed spatial search result :");
            log.Ends("road id = " + std::to_string(g_match.roadid));
            log.Ends("lane id = " + std::to_string(g_match.laneid));
            log.Endl("conn id = " + std::to_string(g_match.connid));
            
            g_need_send_map=(last_road_id!=g_match.roadid || last_conn_id!=g_match.connid);
            if(g_need_send_map)
                g_update_times++;
            g_update_map=(last_road_id!=g_match.roadid||last_conn_id!=g_match.connid)?5:g_update_map;
            last_road_id=g_match.roadid;
            last_conn_id=g_match.connid;
            g_come_target=(g_match.roadid==g_mission.roadid);
            
            std::thread sendroadthread = std::thread(SendBlockMsg, partial_road_vec, partial_conn_vec);
            sendroadthread.join();
        }
        else
        {
            log.Endl("Can't Match In Map.");
            SendEmptyMap();
            match_failed_time++;
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
        log.Update();
        
        std::chrono::steady_clock::time_point etp = std::chrono::steady_clock::now();
        std::chrono::milliseconds span = std::chrono::duration_cast<std::chrono::milliseconds>(etp - stp);
        if (span.count() < match_interval)
            std::this_thread::sleep_for(std::chrono::milliseconds(match_interval - span.count()));
    }
    log.Stop();
    
}
