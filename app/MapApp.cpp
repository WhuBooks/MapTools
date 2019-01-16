//
// Created by books on 18-7-28.
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
    MapWaited = 3,     /// map stop and wait for next command
    MapDrivingPause=4  /// pause by man's command
};

enum ClientCMD
{
    CMD_Start = 0, /// umanned drive, will or already send a target
    CMD_Stop = 1,  /// stop at the current position, clear current target and wait for new command
    CMD_Pause = 2, /// manned drive
    CMD_Continue=3
};



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
    std::mutex g_partial_mutex;
    
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
    
    /// replan function
    std::function<bool(MapBase::RoadPoint, int)> ReplanFunc = [&](MapBase::RoadPoint full_match, int target) -> bool {
        
        std::unique_lock<std::mutex> lock(g_partial_mutex);
    
        MapBase::ConnVec tmp_partial_conn_vec;
        if(full_match.roadid==-1)
        {
            for(const MapBase::Connline &conn : full_conn_vec)
            {
                if(conn.connid==full_match.connid)
                {
                    full_match.roadid=conn.eroadid;
                    tmp_partial_conn_vec.push_back(conn);
                    break;
                }
            }
        }
        
        /// query graph and update cluster filter
        partial_road_vec = graph.QueryRoad(full_match.roadid, target);
        partial_conn_vec = graph.QueryConn(full_match.roadid, target);
        
        tmp_partial_conn_vec.insert(tmp_partial_conn_vec.end(),partial_conn_vec.begin(),partial_conn_vec.end());
        partial_conn_vec.swap(tmp_partial_conn_vec);

        
        if (partial_road_vec.empty() || partial_conn_vec.empty())
            return false;
        
        /// trim target road to target point
        for(MapBase::Road &road : partial_road_vec)
        {
            if(road.roadid==target)
            {
                for(MapBase::Lane &lane : road.vlane)
                {
                    int tmp_index=NearestIndex(lane.vec,target_point);
                    MapBase::GnssVec tmp_lane_vec;
                    tmp_lane_vec.assign(lane.vec.begin(),lane.vec.begin()+tmp_index);
                    lane.vec.swap(tmp_lane_vec);
                }
                break;
            }
        }
        
        g_mission.roadid=target;
        partial_cluster.Load(partial_road_vec, partial_conn_vec);
        partial_cluster.Create();
        std::cout << "Replan Done ~ " <<target<< std::endl;
        return true;
        
    };
    
    std::vector<double> g_map_lat_vec,g_map_lon_vec;
    std::function<void()> ExtractFunc=[&](){
        g_map_lat_vec.clear();
        g_map_lon_vec.clear();
        if(!partial_road_vec.empty() && !partial_conn_vec.empty())
        {
            MapBase::GnssVec tmp_path;
            for(int i=0;i<partial_road_vec.size()-1;i++)
            {
                MapBase::Road tmp_road=partial_road_vec[i];
                MapBase::Connline tmp_conn=partial_conn_vec[i];
                
                tmp_path.insert(tmp_path.end(),tmp_road.vlane.front().vec.begin(),tmp_road.vlane.front().vec.end());
                tmp_path.insert(tmp_path.end(),tmp_conn.data.begin(),tmp_conn.data.end());
            }
            tmp_path.insert(tmp_path.end(),partial_road_vec.back().vlane.front().vec.begin(),partial_road_vec.back().vlane.front().vec.end());
            
            for(const MapBase::GnssData &gd : tmp_path)
            {
                g_map_lat_vec.push_back(gd.lat);
                g_map_lon_vec.push_back(gd.lon);
            }
        }
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
            std::unique_lock<std::mutex> lock(status_mutex);
            msg.time_stamp = receive_c2m_times;
            msg.map_status = status;
            if (status == MapPlanSucceed || status == MapDrivingPause)
            {
                //std::cout << "Path Size ~ " << msg.pt_num << std::endl;
        
                msg.pt_lat.assign(g_map_lat_vec.begin(), g_map_lat_vec.end());
                msg.pt_lon.assign(g_map_lon_vec.begin(), g_map_lon_vec.end());

            }
            else
                msg.pt_num=0;
            
            lock.unlock();
            if (msg.pt_lon.size() != msg.pt_lat.size())
            {
                msg.pt_num = 0;
                msg.pt_lon.clear();
                msg.pt_lat.clear();
            }
            else
                msg.pt_num = msg.pt_lat.size();
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
            std::cout << "Receive Times ~ " << receive_c2m_times << std::endl;
    
            MapBase::PointXYA tmp_pos;
            {
                std::unique_lock<std::mutex> lock(cur_pos_mutex);
                tmp_pos = MapBase::PointXYA(cur_pos.x, cur_pos.y, cur_pos.yaw);
                lock.unlock();
            }
            ClientCMD c2m_cmd = (ClientCMD) c2MMsg.cmd;
            int c2m_roadid = c2MMsg.roadId;
            int c2m_ptid = c2MMsg.pointId;
    
            switch (c2m_cmd)
            {
                case CMD_Start:
                    /// hasn't select a target
                    if (c2m_roadid == -1)
                    {
                        /// send waited map status
                        {
                            std::unique_lock<std::mutex> lock(status_mutex);
                            std::cout << "Waiting for A New Target." << std::endl;
                            status = MapWaited;
                            lock.unlock();
                        }
            
                        {
                            std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                            keep_driving = false;
                            lock.unlock();
                        }
            
                        break;
                    }
        
                    /// select a new target
                    if (partial_road_vec.empty() || partial_conn_vec.empty())
                    {
            
                        MapBase::RoadPoint full_match;
                        bool full_match_flag = full_cluster.SpatialSearch(cur_pos, full_match);
                        std::cout << "Global Match In ~ " << full_match.roadid << std::endl;
            
                        /// replan
                        bool find_flag = FindTargetPoint(c2m_roadid, c2m_ptid);
                        bool plan_flag = ReplanFunc(full_match, c2m_roadid);
                        
                        
                        if (plan_flag && full_match_flag && find_flag)
                        {
                            ExtractFunc();
                            
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
                
                            break;
                        }
                        else
                        {
                
                            /// send plan failed map status
                            {
                                std::unique_lock<std::mutex> lock(status_mutex);
                                std::cout << "Replan Failed or Car is in the Fork or Target Point is Not In Map!" << std::endl;
                                status = MapPlanFailed;
                                lock.unlock();
                            }
                
                            {
                                std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                                keep_driving = false;
                                lock.unlock();
                            }
                            {
                                std::unique_lock<std::mutex> lock(g_partial_mutex);
    
                                partial_cluster.Clear();
                                partial_road_vec.clear();
                                partial_conn_vec.clear();
                                
                                lock.unlock();
                            }

                
                            break;
                        }
            
                    }
                    else
                    {
                        /// send plan succeed map status
                        {
                            std::unique_lock<std::mutex> lock(status_mutex);
                            std::cout << "In Unmanned Driving!" << std::endl;
                            status = MapPlanSucceed;
                            lock.unlock();
                        }
            
                        {
                            std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                            keep_driving = true;
                            lock.unlock();
                        }
                        break;
                    }
                    break;
                case CMD_Stop:
                    /// send invoke to plan module, and reset partial set
                    {
                        std::unique_lock<std::mutex> lock(status_mutex);
                        std::cout << "Mannuly Stop and Clear!" << std::endl;
                        status = MapWaited;
                        lock.unlock();
                    }
        
                    {
                        std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                        keep_driving = false;
                        lock.unlock();
                    }
        
                    {
                        std::unique_lock<std::mutex> lock(g_partial_mutex);
            
                        partial_cluster.Clear();
                        partial_road_vec.clear();
                        partial_conn_vec.clear();
            
                        lock.unlock();
                    }

                    break;
    
                case CMD_Pause:
                    /// send stop to plan module
                    if(!partial_road_vec.empty()&&!partial_conn_vec.empty())
                    {
                        {
                            std::unique_lock<std::mutex> lock(status_mutex);
                            std::cout << "Mannuly Pause!" << std::endl;
                            status=MapDrivingPause;
                            lock.unlock();
                        }
    
                        {
                            std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                            keep_driving=false;
                            lock.unlock();
                        }
                    }
                    break;
    
                case CMD_Continue:
                    /// send continue to plan module when there is already a target
                    if(!partial_conn_vec.empty() && ! partial_road_vec.empty())
                    {
                        {
                            std::unique_lock<std::mutex> lock(status_mutex);
                            std::cout<<"Mannuly Continue!"<<std::endl;
                            status=MapPlanSucceed;
                            lock.unlock();
                        }
                        
                        {
                            std::unique_lock<std::mutex> lock(g_keep_driving_mutex);
                            keep_driving=true;
                            lock.unlock();
                        }
                    }
        
                    break;
    
    
                default:
                    break;
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
            log.Update();
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
    
    
        {
            std::unique_lock<std::mutex> lock(g_partial_mutex);
            
            if(partial_road_vec.empty()||partial_conn_vec.empty())
                continue;
            
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
            
                g_need_send_map = (last_road_id != g_match.roadid || last_conn_id != g_match.connid);
                if (g_need_send_map)
                    g_update_times++;
                g_update_map = (last_road_id != g_match.roadid || last_conn_id != g_match.connid) ? 5 : g_update_map;
                last_road_id = g_match.roadid;
                last_conn_id = g_match.connid;
                g_come_target = (g_match.roadid == g_mission.roadid);
            
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
            lock.unlock();
        }
        
        

        
        std::chrono::steady_clock::time_point etp = std::chrono::steady_clock::now();
        std::chrono::milliseconds span = std::chrono::duration_cast<std::chrono::milliseconds>(etp - stp);
        if (span.count() < match_interval)
            std::this_thread::sleep_for(std::chrono::milliseconds(match_interval - span.count()));
    }
    log.Stop();
    
}
