//
// Created by books on 2017/8/2.
//

#include "Graph.h"

namespace MapBase
{
    Graph::Graph()
    {

    }

    Graph::Graph(const RoadVec &vroad, const ConnVec &vconn)
    {
        mvroad = vroad;
        mvconn = vconn;
    }

    Graph::~Graph()
    {

    }

    void Graph::Initialize()
    {
        //initialize the link table
        for (const Road rdi:mvroad)
        {
            for (const Road &rdj : mvroad)
            {
                Link lk;
                lk.s_id = rdi.roadid;
                lk.e_id = rdj.roadid;
                lk.m_id = -2;
                lk.dis = std::numeric_limits<double>::max() / 3;
                if (lk.s_id != lk.e_id)
                    mvlink.push_back(lk);
            }
        }
        std::cout << "[ Graph ] ~ Initialize link table!" << std::endl;

        //use floyed algorithm to find best middle fork
        Floyed();
        std::cout << "[ Graph ] ~ Floyed algorithm!" << std::endl;

        ExtractPath();
        std::cout << "[ Graph ] ~ Extract path from link!" << std::endl;

        CheckPath();
        std::cout << "[ Graph ] ~ Check Path!" << std::endl;

    }
    
    RoadVec Graph::QueryRoad(const int &matchroadid, const int &missionroadid)
    {
        for (const Path &pa : mvpath)
        {
            if (pa.sroadid == matchroadid && pa.eroadid == missionroadid)
            {
                RoadVec result;
                for (const int &roid : pa.vroadid)
                    for (const Road &rd : mvroad)
                        if (rd.roadid == roid)
                            result.push_back(rd);
                return result;
            }
        }
        return RoadVec();
    }

    ConnVec Graph::QueryConn(const int &matchroadid, const int &missionroadid)
    {
        for (const Path &pa : mvpath)
        {
            if (pa.sroadid == matchroadid && pa.eroadid == missionroadid)
            {
                ConnVec result;
                for (std::size_t i = 0; i < pa.vroadid.size() - 1; i++)
                {
                    int s_id = pa.vroadid[i];
                    int e_id = pa.vroadid[i + 1];
                    for (const Connline &conn : mvconn)
                    {
                        if (conn.sroadid == s_id && conn.eroadid == e_id)
                            result.push_back(conn);
                    }
                }
                return result;
            }
        }
        return ConnVec();
    }

    std::vector<int> Graph::QueryRoadIDs(const int &matchid, const int &targetid)
    {
        for (const Path &pa : mvpath)
        {
            if (pa.sroadid == matchid && pa.eroadid == targetid)
            {
                std::vector<int> ids;
                for (const int &roid : pa.vroadid)
                    for (const Road &rd : mvroad)
                        if (rd.roadid == roid)
                            ids.push_back(rd.roadid);
                return ids;
            }
        }

        return std::vector<int>();
    }

    std::vector<int> Graph::QueryConnIDs(const int &matchid, const int &targetid)
    {
        for (const Path &pa : mvpath)
        {
            if (pa.sroadid == matchid && pa.eroadid == targetid)
            {
                std::vector<int> ids;
                for (std::size_t i = 0; i < pa.vroadid.size() - 1; i++)
                {
                    int s_id = pa.vroadid[i];
                    int e_id = pa.vroadid[i + 1];
                    for (const Connline &conn : mvconn)
                    {
                        if (conn.sroadid == s_id && conn.eroadid == e_id)
                            ids.push_back(conn.connid);
                    }
                }
                return ids;
            }
        }
        return std::vector<int>();
    }

    void Graph::Floyed()
    {
        //find all the directly connected roads and their smallest distance
        for (const Connline &conn : mvconn)
        {
            for (Link &lk : mvlink)
            {
                if (lk.s_id == conn.sroadid && lk.e_id == conn.eroadid)
                {
                    lk.m_id = -1;
                    double length = CalLength(conn.sroadid, conn.eroadid, conn.connid);
                    if (lk.dis > length)
                        lk.dis = length;
                }

            }
        }

        //three layers of iteration
        for (std::size_t k = 0; k < mvroad.size(); k++)
        {
            Road node_k = mvroad[k];
            for (std::size_t i = 0; i < mvroad.size(); i++)
            {
                if (i == k)
                    continue;
                Road node_i = mvroad[i];
                for (std::size_t j = 0; j < mvroad.size(); j++)
                {
                    if (i == j)
                        continue;
                    Road node_j = mvroad[j];

                    std::size_t ij_index = 0, ik_index = 0, kj_index = 0;
                    for (std::size_t t = 0; t < mvlink.size(); t++)
                    {
                        if (mvlink[t].s_id == node_i.roadid && mvlink[t].e_id == node_j.roadid)
                            ij_index = t;
                        if (mvlink[t].s_id == node_i.roadid && mvlink[t].e_id == node_k.roadid)
                            ik_index = t;
                        if (mvlink[t].s_id == node_k.roadid && mvlink[t].e_id == node_j.roadid)
                            kj_index = t;
                    }
                    if (mvlink[ij_index].dis > mvlink[ik_index].dis + mvlink[kj_index].dis)
                    {
                        mvlink[ij_index].m_id = node_k.roadid;
                        mvlink[ij_index].dis = mvlink[ik_index].dis + mvlink[kj_index].dis;
                    }
                }
            }
        }
    }

    void Graph::ExtractPath()
    {
        //trim the link table, delete the unarrivable link
        std::vector<Link>::iterator iter;
        for (iter = mvlink.begin(); iter != mvlink.end();)
        {
            if (iter->m_id == -2)
                iter = mvlink.erase(iter);
            else
                iter++;
        }

        //build path
        for (const Link &link : mvlink)
        {
            Path pa;
            pa.sroadid = link.s_id;
            pa.eroadid = link.e_id;

            std::vector<int> idlst;
            idlst.push_back(link.s_id);
            //idlst.push_back(link.m_id);
            idlst.push_back(link.e_id);
            bool noavailble = CheckNoAvailble(idlst);

            while (!noavailble)
            {
                std::vector<int>::iterator iditer;
                bool insertflag = false;
                for (iditer = idlst.begin(); iditer != idlst.end() - 1; iditer++)
                {
                    int sid = *iditer;
                    int eid = *(iditer + 1);
                    for (const Link &lk : mvlink)
                    {
                        if (lk.s_id == sid && lk.e_id == eid)
                        {
                            if (lk.m_id != -1)
                            {
                                idlst.insert(iditer + 1, lk.m_id);
                                insertflag = true;
                            }
                            break;
                        }
                    }
                    if (insertflag)
                        break;
                }
                noavailble = CheckNoAvailble(idlst);
            }

            pa.vroadid = idlst;
            mvpath.push_back(pa);
        }
    }

    bool Graph::CheckNoAvailble(const std::vector<int> &idvec) const
    {
        bool flag = true;
        for (std::size_t i = 0; i < idvec.size() - 1; i++)
        {
            int sid = idvec[i];
            int eid = idvec[i + 1];
            for (const Link &lk : mvlink)
            {
                if (lk.s_id == sid && lk.e_id == eid)
                    flag = flag && (lk.m_id == -1);
            }
        }

        return flag;
    }

    double Graph::CalLength(int sroadid, int eroadid, int connid)
    {
        double length = 0.0;
        for (const Road &rd : mvroad)
            if (rd.roadid == sroadid || rd.roadid == eroadid)
                length += rd.avelength;
        for (const Connline &cn : mvconn)
            if (cn.connid == connid)
                length += cn.Length();
        return length;
    }

    void Graph::CheckPath()
    {
        std::vector<Path> tmp_path_vec;
        for (const Path &pa : mvpath)
        {
            bool findall = true;
            for (std::size_t i = 0; i < pa.vroadid.size() - 1; i++)
            {
                bool findconn = false;
                for (const Connline &conn : mvconn)
                {
                    if (conn.sroadid == pa.vroadid[i] && conn.eroadid == pa.vroadid[i + 1])
                    {
                        findconn = true;
                        break;
                    }
                }
                findall = findall && findconn;
            }
            if(findall)
                tmp_path_vec.push_back(pa);
        }
        mvpath.swap(tmp_path_vec);
    }


    const PathVec &Graph::getMvpath() const
    {
        return mvpath;
    }

    void Graph::setMvpath(const PathVec &vpath)
    {
        this->mvpath = vpath;
    }

    void Graph::setMvroad(const RoadVec &vroad)
    {
        this->mvroad = vroad;
    }

    void Graph::setMvconn(const ConnVec &vconn)
    {
        this->mvconn = vconn;
    }

    void Graph::PrintInfo()
    {
        for (const Path &pa : mvpath)
        {
            std::cout << "[ Graph ] ~ start road id = " << pa.sroadid << "\t";
            std::cout << "end road id = " << pa.eroadid << std::endl;
            bool findall = true;
            for (std::size_t i = 0; i < pa.vroadid.size() - 1; i++)
            {
                bool findconn = false;
                for (const Connline &conn : mvconn)
                {
                    if (conn.sroadid == pa.vroadid[i] && conn.eroadid == pa.vroadid[i + 1])
                    {
                        std::cout << pa.vroadid[i] << " -> (" << conn.connid << ") -> ";
                        findconn = true;
                        break;
                    }
                }
                findall = findall && findconn;
            }
            std::cout << pa.vroadid.back() << std::endl;
            if(!findall)
                std::cout<<"Can't Extract Full Path."<<std::endl;
        }
    }

    int Graph::InitRandomMission(int roadid)
    {
        PathVec tmpPathVec;
        PathVec oncePathVec;
        for (const Path &path : mvpath)
        {
            if (path.sroadid == roadid)
            {
                /// make sure this path's end node can arrive other node
                bool find=false;
                for(const Path &other : mvpath)
                {
                    if(other.sroadid==path.eroadid)
                        find=true;
                }

                if(find)
                    tmpPathVec.push_back(path);
                else
                    oncePathVec.push_back(path);
            }
        }

        std::shuffle(tmpPathVec.begin(), tmpPathVec.end(), std::default_random_engine(std::time(nullptr)));
        std::shuffle(oncePathVec.begin(),oncePathVec.end(),std::default_random_engine(std::time(nullptr)));

        if (!tmpPathVec.empty())
            return tmpPathVec.front().eroadid;
        
        if(!oncePathVec.empty())
            return oncePathVec.front().eroadid;
        
        return -1;

    }

    void Graph::PrintErrorInfo()
    {
        for(int i=0;i<mvroad.size();i++)
        {
            for(int j=0;j<mvroad.size();j++)
            {
                if(i==j)
                    continue;

                int s_id=mvroad[i].roadid;
                int e_id=mvroad[j].roadid;
                bool find=false;
                for(const Path &path : mvpath)
                {
                    if(path.sroadid==s_id && path.eroadid==e_id)
                        find=true;
                }

                if(!find)
                    std::cout<<"Can't Find Path ~ "<<s_id<<"\t"<<e_id<<std::endl;
            }
        }
    }

}