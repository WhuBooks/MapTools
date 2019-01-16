//
// Created by books on 2017/8/2.
//

#ifndef _MAPBASE_GRAPH_H_
#define _MAPBASE_GRAPH_H_

#include "Geometry.h"
#include "RoadModel.h"

#include <iostream>
#include <map>
#include <vector>
#include <deque>
#include <random>
#include <list>
#include <tuple>
#include <ctime>

namespace MapBase
{
    class Graph
    {
        //used in Flyod, represented the link relation of two fork
        struct Link
        {
            int s_id;
            int e_id;
            //m_id=-1 represented s_id directly connect to e_id
            //m_id=-2 represented s_id can't arrive at e_id
            int m_id;
            double dis;
            Link()
            {
                s_id = -1;
                e_id = -1;
                m_id = -1;
            };
        };

        typedef std::vector<Link> LinkVec;

    public:
        Graph();

        Graph(const RoadVec &vroad, const ConnVec &vconn);

        ~Graph();

        void Initialize();

        RoadVec QueryRoad(const int &matchroadid, const int &missionroadid);
        ConnVec QueryConn(const int &matchroadid, const int &missionroadid);

        std::vector<int> QueryRoadIDs(const int &matchid,const int &targetid);
        std::vector<int> QueryConnIDs(const int &matchid,const int &targetid);

        void PrintInfo();
        void PrintErrorInfo();

        //get and set
        const PathVec &getMvpath() const;

        void setMvroad(const RoadVec &vroad);

        void setMvpath(const PathVec &vpath);

        void setMvconn(const ConnVec &vconn);

        int InitRandomMission(int missionid);

    private:
        bool CheckNoAvailble(const std::vector<int> &idlst) const;

        void Floyed();

        void ExtractPath();

        void CheckPath();

        double CalLength(int sroadid,int eroadid,int connid);
    private:
        RoadVec mvroad;
        LinkVec mvlink;
        PathVec mvpath;
        ConnVec mvconn;
    };
}



#endif //MAPTOOLS_GRAPH_H
