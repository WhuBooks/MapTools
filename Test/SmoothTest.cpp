//
// Created by books on 2018/6/7.
//

#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <thread>

#include <Geometry.h>
#include <RoadModel.h>
#include <Clothoid.h>
#include <FileIO.h>
#include <DouglasPeucker.h>
#include <Cluster.h>
#include <Log.h>

#include <LcmType/Map_t.hpp>
#include <LcmType/LcmHandler.h>
#include <LcmType/Draw_t.hpp>
#include <LcmType/Location_t.hpp>

typedef LcmHandler<ckLcmType::Map_t>::Ptr MapHandlerPtr;
typedef LcmHandler<ckLcmType::Draw_t>::Ptr DrawHandlerPtr;
typedef LcmHandler<ckLcmType::Location_t>::Ptr LocHandlerPtr;

MapBase::PointXYA cur_pos;
MapBase::RoadPoint g_match;

static const int forward=50;
static const int backward=10;

enum ConnStatus
{
    OutConn=0,
    EnterConn=1,
    LeaveConn=2,
    InConn=3
};

typedef ckLcmType::ckPointXYA_t PtMsg;
typedef std::vector<ckLcmType::ckPointXYA_t> LaneMsg;
typedef std::vector<std::vector<ckLcmType::ckPointXYA_t>> RoadMsg;
typedef ckLcmType::Map_t MapMsg;
typedef ckLcmType::Draw_t DrawMsg;

PtMsg Pos(double x,double y,double yaw)
{
    PtMsg pt;
    pt.x=x;
    pt.y=y;
    pt.yaw=yaw;
    return pt;
}

int NearestIndex(const MapBase::GnssVec &vec,const MapBase::PointXYA &pos)
{
    int index=0;
    double min_dis=10000000000;
    for(std::size_t i=0;i<vec.size();i++)
    {
        double tmp_dis=pos.Distance(vec[i]);
        if(tmp_dis<min_dis)
        {
            min_dis=tmp_dis;
            index=i;
        }
    }
    return index;
}

LaneMsg ExtractLaneMsg(const MapBase::GnssVec &vec,const MapBase::PointXYA &pos)
{
    LaneMsg msg;
    int index = NearestIndex(vec,pos);

    for (int i = -backward; i < forward; i++)
    {
        int cur_index = index + i;
        if (cur_index < 0 || cur_index >= vec.size())
            msg.push_back(Pos(-1000, -1000, -1000));
        else
            msg.push_back(Pos(vec[cur_index].x, vec[cur_index].y, vec[cur_index].yaw));
    }
    return msg;
}

DrawMsg LocalDrawMsg(const MapMsg &map_msg)
{
    DrawMsg draw_msg;
    std::vector<double> vx,vy;
    for(const LaneMsg &lane_msg : map_msg.road)
    {
        for (const PtMsg &pt_msg : lane_msg)
        {
            if (pt_msg.x == -1000 || pt_msg.y == -1000 || pt_msg.yaw == -1000)
                continue;

            MapBase::PointXYA pt(pt_msg.x, pt_msg.y, pt_msg.yaw);
            MapBase::PointXYA local = pt.ToLocal(g_match);

            vx.push_back(local.x);
            vy.push_back(local.y);
        }
    }
    draw_msg.ptnum=(int)vx.size();
    draw_msg.x.swap(vx);
    draw_msg.y.swap(vy);
    return draw_msg;
}

void GetRoadMessage(const MapBase::RoadVec &vroad,const MapBase::ConnVec &vconn)
{
    DrawHandlerPtr drawhandler(new LcmHandler<ckLcmType::Draw_t>);
    drawhandler->SetNet("udpm://238.255.76.67:7667?ttl=64");
    drawhandler->SetChannel("CKMAPDRAW");
    drawhandler->InitialSend();

    MapHandlerPtr maphandler(new LcmHandler<ckLcmType::Map_t>);
    maphandler->SetNet("udpm://238.255.76.67:7667?ttl=64");
    maphandler->SetChannel("CKMAP");
    maphandler->InitialSend();

    //find target by roadid or connid
    bool findroad = false;
    MapBase::Road road;
    for (const MapBase::Road &ro : vroad)
    {
        if (ro.roadid == g_match.roadid)
        {
            findroad = true;
            road = ro;
            break;
        }
    }

    bool findconn=false;
    MapBase::Connline conn;
    for(const MapBase::Connline &co : vconn)
    {
        if(co.connid==g_match.connid)
        {
            findconn=true;
            conn=co;
            break;
        }
    }

    if (!findroad)
    {
        ckLcmType::Draw_t drawmessage;
        drawmessage.ptnum = 0;
        drawhandler->SendLcm(&drawmessage);
        return;
    }

    RoadMsg ckroad;
    ConnStatus status=OutConn;
    int curIndex = -1;
    int targetIndex=-1;

    if(g_match.laneid!=-1)
    {
        int laneindex=0;
        /// find current index and target index
        for(const MapBase::Lane &la : road.vlane)
        {
            curIndex = (la.laneid == g_match.laneid) ? laneindex : curIndex;

            /// if find connline, find target index
            if (findconn)
                targetIndex=(la.laneid==conn.slaneid)?laneindex:targetIndex;

            laneindex++;
        }

        if(targetIndex==-1)
            targetIndex=curIndex;

        /// check OutConn or EnterConn
        int target_nearest_index=NearestIndex(road.vlane[targetIndex].vec,g_match);
        if(target_nearest_index+forward<road.vlane[targetIndex].vec.size())
        {
            for(const MapBase::Lane &la : road.vlane)
            {
                LaneMsg laneMsg=ExtractLaneMsg(la.vec,g_match);
                ckroad.push_back(laneMsg);
            }
            status=ConnStatus::OutConn;
        }
        else
        {
            MapBase::GnssVec vec=road.vlane[targetIndex].vec;
            if(findconn)
                vec.insert(vec.end(),conn.data.begin(),conn.data.end());
            LaneMsg laneMsg=ExtractLaneMsg(vec,g_match);
            ckroad.push_back(laneMsg);
            status=ConnStatus::EnterConn;
            targetIndex=0;
            curIndex=0;
        }
    }
    else if(findconn)
    {
        /// check InConn or LeaveConn
        int conn_nearest_index=NearestIndex(conn.data,g_match);
        if(conn_nearest_index+forward<conn.data.size())
        {
            LaneMsg laneMsg=ExtractLaneMsg(conn.data,g_match);
            ckroad.push_back(laneMsg);
            status=ConnStatus::InConn;
            targetIndex=0;
            curIndex=0;
        }
        else
        {
            MapBase::Lane nearestLane;
            for(const MapBase::Lane &la : road.vlane)
            {
                if(la.laneid==conn.elaneid)
                    nearestLane=la;
            }

            MapBase::GnssVec vec=conn.data;
            vec.insert(vec.end(),nearestLane.vec.begin(),nearestLane.vec.end());

            LaneMsg laneMsg=ExtractLaneMsg(vec,g_match);
            ckroad.push_back(laneMsg);
            status=ConnStatus::LeaveConn;
            targetIndex=0;
            curIndex=0;
        }
    }

    ckLcmType::Map_t mapMsg;
    mapMsg.curpos=Pos(g_match.x,g_match.y,g_match.yaw);
    mapMsg.lanenum=(int)ckroad.size();
    mapMsg.pointnum=backward+forward;
    mapMsg.curindex=curIndex;
    mapMsg.targetindex=targetIndex;
    mapMsg.inconn=status;
    mapMsg.road.assign(ckroad.begin(),ckroad.end());
    maphandler->SendLcm(&mapMsg);

    ckLcmType::Draw_t drawMsg=LocalDrawMsg(mapMsg);
    drawhandler->SendLcm(&drawMsg);
}

void SendEmpty()
{
    ckLcmType::Map_t mapMsg;
    mapMsg.lanenum=0;
    mapMsg.pointnum=0;

    ckLcmType::Draw_t drawMsg;
    drawMsg.ptnum=0;

    DrawHandlerPtr drawhandler(new LcmHandler<ckLcmType::Draw_t>);
    drawhandler->SetNet("udpm://238.255.76.67:7667?ttl=64");
    drawhandler->SetChannel("CKMAPDRAW");
    drawhandler->InitialSend();

    MapHandlerPtr maphandler(new LcmHandler<ckLcmType::Map_t>);
    maphandler->SetNet("udpm://238.255.76.67:7667?ttl=64");
    maphandler->SetChannel("CKMAP");
    maphandler->InitialSend();

    drawhandler->SendLcm(&drawMsg);
    maphandler->SendLcm(&mapMsg);
}


int main()
{
    std::string modeldir = "TestModel";

    std::string file1=modeldir+"/0.txt";
    std::string file2=modeldir+"/1.txt";
    std::string file3=modeldir+"/2.txt";

    MapBase::GnssVec vec1,vec2,vec3;
    MapBase::DouglasPeucker dp;
    MapBase::ReadGnssWithoutID(file1,vec1);
    MapBase::ReadGnssWithoutID(file2,vec2);
    MapBase::ReadGnssWithoutID(file3,vec3);

    MapBase::RoadVec vroad;

    MapBase::Road road1;
    road1.roadid=0;
    MapBase::Lane lane1;
    lane1.laneid=0;
    lane1.vec=dp.Process(vec1);
    road1.vlane.push_back(lane1);
    road1.Initilize();

    MapBase::Road road2;
    road2.roadid=1;
    MapBase::Lane lane2;
    lane2.laneid=1;
    lane2.vec=dp.Process(vec2);
    road2.vlane.push_back(lane2);
    road2.Initilize();

    vroad.push_back(road1);
    vroad.push_back(road2);

    MapBase::Connline conn1;
    conn1.connid=0;
    conn1.data=dp.Process(vec3);
    conn1.sroadid=0;
    conn1.eroadid=1;
//    conn1.InitLaneID(vroad);

    MapBase::ConnVec vconn;
    vconn.push_back(conn1);

    /// modify road and conn
    for(MapBase::Connline &conn : vconn)
        conn.InitLaneID(vroad);

    MapBase::Cluster cluster;
    cluster.Load(vroad, vconn);
    cluster.Create();
    bool clusterflag = cluster.Check();
    if (!clusterflag)
    {
        std::cerr << "Static Spatial Cluster is incorrect!" << std::endl;
        return -1;
    }

    /// location msg
    std::cout << "Wait for location message..." << std::endl;
    LocHandlerPtr lochandler(new LcmHandler<ckLcmType::Location_t>);
    lochandler->SetNet("udpm://238.255.76.67:7667?ttl=64");
    lochandler->SetChannel("CKLOCATION");
    lochandler->InitialListen();
    ckLcmType::Location_t locmes;
    lochandler->GetData(locmes);
    std::cout << "Receive location message!" << std::endl;
    cur_pos = MapBase::PointXYA(locmes.gau_pos[0], locmes.gau_pos[1], locmes.orientation[2]);

    MapBase::Log log(2);
    log.Start();

    while(true)
    {
        std::chrono::steady_clock::time_point stp = std::chrono::steady_clock::now();

        lochandler->GetData(locmes);
        cur_pos = MapBase::PointXYA(locmes.gau_pos[0], locmes.gau_pos[1],locmes.orientation[2]);

        log.Ends("Current Position : ");
        log.Ends(std::to_string(cur_pos.x));
        log.Ends(std::to_string(cur_pos.y));
        log.Endl(std::to_string(cur_pos.yaw));

        /************Spatial Search**********/
        bool match_flag = cluster.SpatialSearch(cur_pos, g_match);

        log.Ends("Spatial search result :");
        log.Ends("road id = " + std::to_string(g_match.roadid));
        log.Ends("lane id = " + std::to_string(g_match.laneid));
        log.Endl("conn id = " + std::to_string(g_match.connid));

        if (match_flag)
        {
            /// adjust match result
            for (const MapBase::Connline &conn : vconn)
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

            GetRoadMessage(vroad,vconn);

        }
        else
        {
            log.Endl("Can't Match In Map.");
            SendEmpty();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        log.Update();

        std::chrono::steady_clock::time_point etp = std::chrono::steady_clock::now();
        std::chrono::milliseconds span = std::chrono::duration_cast<std::chrono::milliseconds>(etp - stp);
        if (span.count() < 20)
            std::this_thread::sleep_for(std::chrono::milliseconds(20 - span.count()));
    }

}