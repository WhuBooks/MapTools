//
// Created by 98420 on 2018/8/30.
//

#include "Floyd.h"
#include <cassert>
#include <map>
#include <list>

namespace maplib
{
    namespace graph{
    
        Floyd::Floyd(const NodeVec &nodes, const EdgeVec &edges) : Graph(nodes, edges)
        {
            calAllPaths();
        }
        
    
        const int Unknown=-1;
        const int Unarrived=-2;
        const int SameNode=-3;
        const int Straight=-4;
        
        void ExtractPath(const std::vector<std::vector<int>> &table,std::list<int>::iterator s_iter,std::list<int>::iterator e_iter,std::list<int> &list)
        {
            int val=table[*s_iter][*e_iter];
            assert(val!=Unarrived && val!=SameNode && val!=Unknown);
            if(val==Straight)
                return;
            
            std::list<int>::iterator m_iter=list.insert(e_iter,val);
            ExtractPath(table,s_iter,m_iter,list);
            ExtractPath(table,m_iter,e_iter,list);
        }
        
        
        void Floyd::calAllPaths()
        {
            /// initialize link table and distance table
            std::sort(m_nodes.begin(),m_nodes.end(),[](const Node &n1,const Node &n2){
               return n1.id<n2.id;
            });
            const int node_num=m_nodes.size();
            const double max_distance=std::numeric_limits<double>::max()/3.0;
            std::vector<std::vector<int>> link_table(node_num,std::vector<int>(node_num,Unknown));
            std::vector<std::vector<double>> distance_table(node_num,std::vector<double>(node_num,max_distance));
            
            /// update table using edges
            for(int i=0;i<node_num;i++)
            {
                std::vector<Edge> tmp_edges;
                std::map<int,Edge> tmp_edges_map;
                for(const Edge &edge : m_edges)
                {
                    /// find edges start with node[i]
                    if(edge.s_id==m_nodes[i].id)
                        tmp_edges_map[edge.e_id]=edge;
                }
                
                for(int j=0;j<node_num;j++)
                {
                    /// same node
                    if(i==j)
                    {
                        link_table[i][j]=SameNode;
                        distance_table[i][j]=0.0;
                        continue;
                    }
                    
                    /// can't arrive at node j
                    if(tmp_edges_map.empty())
                    {
                        link_table[i][j]=Unarrived;
                        continue;
                    }
                    
                    /// straight link to node j
                    if(tmp_edges_map.find(m_nodes[j].id)!=tmp_edges_map.end())
                    {
                        link_table[i][j]=Straight;
                        distance_table[i][j]=tmp_edges_map[m_nodes[j].id].dis;
                    }
                }
            }
            
            /// three iterations to find suitable mid of each link
            for(int k=0;k<node_num;k++)
            {
                for(int i=0;i<node_num;i++)
                {
                    for(int j=0;j<node_num;j++)
                    {
                        /// if i and j is same or straight link or unarrived
                        if(link_table[i][j]==SameNode || link_table[i][j]==Straight || link_table[i][j]==Unarrived)
                            continue;
                        
                        /// if i and k is same or unarrived
                        if(link_table[i][k]==SameNode || link_table[i][k]==Unarrived)
                            continue;
                        
                        /// if k and j is same or unarrived
                        if(link_table[k][j]==SameNode || link_table[k][j]==Unarrived)
                            continue;
                        
                        /// compare distance_i_j with the plus of distance_i_k and distance_k_j
                        if(distance_table[i][j]>distance_table[i][k]+distance_table[k][j])
                        {
                            link_table[i][j]=k;
                            distance_table[i][j]=distance_table[i][k]+distance_table[k][j];
                        }
                    }
                }
            }
            
            /// extract paths
            m_paths.clear();
            for(int i=0;i<node_num;i++)
            {
                for(int j=0;j<node_num;j++)
                {
                    if(link_table[i][j]==Unarrived || link_table[i][j]==Unknown || link_table[i][j]==SameNode)
                        continue;
                    
                    int s_id=m_nodes[i].id;
                    int e_id=m_nodes[j].id;
                    std::vector<int> tmp_vec;
                    if(link_table[i][j]==Straight)
                        tmp_vec=std::vector<int>(s_id,e_id);
                    else
                    {
                        std::list<int> list;
                        std::list<int>::iterator s_iter=list.insert(list.end(),s_id);
                        std::list<int>::iterator e_iter=list.insert(list.end(),e_id);
                        ExtractPath(link_table,s_iter,e_iter,list);
                        for(std::list<int>::iterator iter=list.begin();iter!=list.end();iter++)
                            tmp_vec.push_back(m_nodes[*iter].id);
                    }
                    Path path;
                    path.s_id=m_nodes[i].id;
                    path.e_id=m_nodes[j].id;
                    path.vec=tmp_vec;
                    m_paths.push_back(path);
                }
            }

        }
    
//        bool Floyd::findPath(int s_id, int e_id) const
//        {
//            for(const Path &path : m_paths)
//                if(path.s_id==s_id && path.e_id==e_id)
//                    return true;
//            return false;
//        }
//
//        Path Floyd::getPath(int s_id, int e_id) const
//        {
//            for(const Path &path : m_paths)
//                if(path.s_id==s_id && path.e_id==e_id)
//                    return path;
//            std::cerr<<"You Must Input Suitable SID and EID."<<std::endl;
//            return m_paths.front();
//        }
    

    
    }
}
