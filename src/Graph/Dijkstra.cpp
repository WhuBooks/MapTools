//
// Created by books on 18-9-7.
//

#include "Dijkstra.h"

namespace maplib{
    namespace graph{
    
        Dijkstra::Dijkstra(const NodeVec &nodes, const EdgeVec &edges) : Graph(nodes, edges)
        {
            for(const Node &s_node:nodes)
            {
                PathVec tmp_paths=calOnePath(s_node.id);
                m_paths.insert(m_paths.end(),tmp_paths.begin(),tmp_paths.end());
            }
        }
    
        PathVec Dijkstra::calOnePath(int s_id)
        {
            /// initialize U and V with edges
            std::map<int,double> U;
            U[s_id]=0.0;
            std::map<int,double> V;
            std::map<int,int> V_pres;
            for(const Node &node : m_nodes)
            {
                if (node.id != s_id)
                {
                    V_pres[node.id] = s_id;
                    V[node.id] = -1;
                }
            }
            for(const Edge &edge : m_edges)
            {
                if(edge.s_id==s_id)
                {
                    V[edge.e_id] = edge.dis;
                }
            }
            
            PathVec tmp_paths;
            while(U.size()<m_nodes.size())
            {
                /// find nearest node
                double min_dis=10000000000;
                int min_id=-1;
                for(const std::pair<int,double> &pair : V)
                {
                    if(pair.second<min_dis && pair.second!=-1)
                    {
                        min_dis=pair.second;
                        min_id=pair.first;
                    }
                }
                
                if(min_id==-1)
                    break;
                
                /// extract a new path
                Path tmp_path;
                tmp_path.s_id=s_id;
                tmp_path.e_id=min_id;
                if(V_pres[min_id]==s_id)
                    tmp_path.vec={s_id,min_id};
                else
                {
                    for(const Path &path : tmp_paths)
                    {
                        if(path.e_id==V_pres[min_id])
                        {
                            tmp_path.vec=path.vec;
                            tmp_path.vec.push_back(min_id);
                        }
                    }
                }
                tmp_paths.push_back(tmp_path);
                
                /// add min_id to U
                U[min_id]=min_dis;
                V.erase(min_id);
                V_pres.erase(min_id);
                
                /// update V and V_pres
                for(const Edge &edge : m_edges)
                {
                    if(U.find(edge.s_id)!=U.end()&&V.find(edge.e_id)!=V.end())
                    {
                        if(U[edge.s_id]+edge.dis<V[edge.e_id])
                        {
                            V[edge.e_id]=U[edge.s_id]+edge.dis;
                            V_pres[edge.e_id]=edge.s_id;
                        }
                    }
                }
            }
            return tmp_paths;
            
        }
    }
}