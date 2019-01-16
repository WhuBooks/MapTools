//
// Created by books on 2018/5/9.
//


#include "BlockMessage.h"

#define UseMissionFile 0


int main(int argc,char **argv)
{
    InitialLcmHandler();

    std::string modeldir = (argc == 1) ? "../dataset/Model" : std::string(argv[1]);

    MapBase::RoadVec full_road_vec;
    if (!MapBase::ReadRoad(modeldir, full_road_vec))
        return -1;
    for(MapBase::Road &road : full_road_vec)
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

#if UseMissionFile
    MapBase::MissionQueue qmission;
    std::string missionfile = modeldir + "/Mission.txt";
    MapBase::ReadMission(missionfile, qmission);
    g_mission = qmission.front();
    qmission.pop();
#endif

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
//    graph.PrintInfo();
    std::cout << "Graph build done!" << std::endl;

    /// location msg
    Convert convert(LatZero,LonZero);
    std::cout << "Wait for location message..." << std::endl;
    GpsHandlerPtr gpsHandler(new LcmHandler<GpsMag>);
    gpsHandler->SetChannel("GPSData");
    gpsHandler->InitialListen();
    
    GpsMag gps_msg;
    gpsHandler->GetData(gps_msg);
    {
        double x=0.0,y=0.0;
        convert.LL2XY(gps_msg.latitude,gps_msg.longitude,x,y);
        cur_pos=MapBase::PointXYA(x,y,gps_msg.heading);
    }
    std::cout << "Receive location message!" << std::endl;
    
    std::cout<<"Current Position : "<<cur_pos.x<<" "<<cur_pos.y<<" "<<cur_pos.yaw<<std::endl;
    
    /// first match in full cluster, get partial cluster
    MapBase::Cluster partial_cluster;
    MapBase::RoadVec partial_road_vec;
    MapBase::ConnVec partial_conn_vec;
    
    std::function<bool(void)> ReplanFunc=[&]()->bool{
#if UseMissionFile
        /// if arrive current mission, update next mission
            if(g_match.roadid==g_mission.roadid)
            {
                if(!qmission.empty())
                {
                    g_mission = qmission.front();
                    qmission.pop();
                }
            }
#else
        int g_mission_id = graph.InitRandomMission(g_match.roadid);
        g_mission.roadid = g_mission_id;
#endif
        if(g_mission.roadid==-1)
            return false;
    
        MapBase::ConnVec tmp_partial_conn_vec;
        if(g_match.roadid==-1)
        {
            for(const MapBase::Connline &conn : full_conn_vec)
            {
                if(conn.connid==g_match.connid)
                {
                    g_match.roadid=conn.eroadid;
                    tmp_partial_conn_vec.push_back(conn);
                    break;
                }
            }
        }
        
        /// query graph and update cluster filter
        partial_road_vec=graph.QueryRoad(g_match.roadid,g_mission.roadid);
        partial_conn_vec=graph.QueryConn(g_match.roadid,g_mission.roadid);
    
        tmp_partial_conn_vec.insert(tmp_partial_conn_vec.end(),partial_conn_vec.begin(),partial_conn_vec.end());
        partial_conn_vec.swap(tmp_partial_conn_vec);
        
        partial_cluster.Load(partial_road_vec,partial_conn_vec);
        partial_cluster.Create();
        std::cout<<"Replan Done ~ "<<g_mission.roadid<<std::endl;
        return true;
        
    };
    
    bool full_match_flag=full_cluster.SpatialSearch(cur_pos,g_match);
    if(full_match_flag)
    {
        /// if start at conn, then set road as the conn's sroad
        if(g_match.roadid==-1)
        {
            for(const MapBase::Connline &conn : full_conn_vec)
            {
                if(conn.connid==g_match.connid)
                {
                    g_match.roadid=conn.sroadid;
                    break;
                }
            }
        }
        std::cout<<"First Match In ~ "<<g_match.roadid<<std::endl;
        
        if(g_match.roadid==-1)
        {
            std::cerr<<"Can't Find First Road ID!"<<std::endl;
            return -1;
        }
        
        bool replan_flag=ReplanFunc();
        if(!replan_flag)
        {
            std::cerr<<"Plan Failed In First Turn!"<<std::endl;
            return -1;
        }
    
//        /// global road plan
//        {
//#if UseMissionFile
//            /// if arrive current mission, update next mission
//            if(g_match.roadid==g_mission.roadid)
//            {
//                if(!qmission.empty())
//                {
//                    g_mission = qmission.front();
//                    qmission.pop();
//                }
//            }
//#else
//            int g_mission_id = graph.InitRandomMission(g_match.roadid);
//            g_mission.roadid = g_mission_id;
//#endif
//        }
//
//        if(g_mission.roadid==-1)
//        {
//            std::cerr<<"Can't Find First Mission."<<std::endl;
//            return -1;
//        }
//
//        /// query graph and update cluster filter
//        partial_road_vec=graph.QueryRoad(g_match.roadid,g_mission.roadid);
//        partial_conn_vec=graph.QueryConn(g_match.roadid,g_mission.roadid);
//        partial_cluster.Load(partial_road_vec,partial_conn_vec);
//        partial_cluster.Create();
//        std::cout<<"Replan Done ~ "<<g_mission.roadid<<std::endl;
    }
    else
    {
        std::cerr<<"Can't Match In First Turn!"<<std::endl;
        return -1;
    }

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
            double x=0.0,y=0.0;
            convert.LL2XY(gps_msg.latitude,gps_msg.longitude,x,y);
            cur_pos=MapBase::PointXYA(x,y,gps_msg.heading);
        }
        
        log.Ends("Current Position : ");
        log.Ends(std::to_string(cur_pos.x));
        log.Ends(std::to_string(cur_pos.y));
        log.Endl(std::to_string(cur_pos.yaw));
        
        /************Spatial Search**********/
        bool match_flag = partial_cluster.SpatialSearch(cur_pos, g_match);

        log.Ends("Spatial search result :");
        log.Ends("road id = " + std::to_string(g_match.roadid));
        log.Ends("lane id = " + std::to_string(g_match.laneid));
        log.Endl("conn id = " + std::to_string(g_match.connid));
        log.Endl("Current Mission ID = "+std::to_string(g_mission.roadid));
        
        if(g_match.roadid==g_mission.roadid || match_failed_time>10)
        {
            
            full_cluster.SpatialSearch(cur_pos,g_match);
//            if(g_match.roadid==-1)
//            {
//                for(const MapBase::Connline &conn : full_conn_vec)
//                {
//                    if(conn.connid==g_match.connid)
//                    {
//                        g_match.roadid=conn.sroadid;
//                        break;
//                    }
//                }
//            }
//            if(g_match.roadid==-1)
//            {
//                std::cerr << "Match Error In Full Map" << std::endl;
//                return -1;
//            }
            
            std::cout<<"Replan At Road : "<<g_match.roadid<<std::endl;
            bool replan_flag=ReplanFunc();
            if(!replan_flag)
            {
                std::cerr<<"Can't Get Next Mission."<<std::endl;
                return -1;
            }
            match_failed_time=5;
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            continue;
        }

//        /// global road plan
//        if(g_match.roadid==g_mission.roadid)
//        {
//#if UseMissionFile
//            if(!qmission.empty())
//            {
//                g_mission = qmission.front();
//                qmission.pop();
//            }
//#else
//            int g_mission_id = graph.InitRandomMission(g_match.roadid);
//            g_mission.roadid = g_mission_id;
//#endif
//            if (g_mission.roadid == -1)
//            {
//                std::cerr << "Can't Get Next Mission." << std::endl;
//                return -1;
//            }
//
//            /// query graph and update cluster filter
//            partial_road_vec = graph.QueryRoad(g_match.roadid, g_mission.roadid);
//            partial_conn_vec = graph.QueryConn(g_match.roadid, g_mission.roadid);
//            partial_cluster.Load(partial_road_vec, partial_conn_vec);
//            partial_cluster.Create();
//            std::cout << "Replan Done ~ " << g_mission.roadid << std::endl;
//            continue;
//        }
        
        
        if (match_flag)
        {
            match_failed_time=0;
            
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
            
            std::thread sendroadthread = std::thread(SendBlockMsg, partial_road_vec,partial_conn_vec);
            sendroadthread.join();
        }
        else
        {
            log.Endl("Can't Match In Map.");
            SendEmptyMap();
            match_failed_time++;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        log.Update();

        std::chrono::steady_clock::time_point etp = std::chrono::steady_clock::now();
        std::chrono::milliseconds span = std::chrono::duration_cast<std::chrono::milliseconds>(etp - stp);
        if (span.count() < match_interval)
            std::this_thread::sleep_for(std::chrono::milliseconds(match_interval - span.count()));
    }
    log.Stop();

}