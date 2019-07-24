//
// Created by books on 18-9-7.
//

#include "AStar.h"
#include <map>
#include <tuple>

namespace maplib
{
    namespace graph{
    
        AStar::AStar(const NodeVec &nodes, const EdgeVec &edges) : Graph(nodes, edges)
        {
            for(const Node &node_s : nodes)
            {
                for(const Node &node_e : nodes)
                {
                    if(node_s.id==node_e.id)
                        continue;
                    Path tmp_path=calOnePath(node_s.id,node_e.id);
                    if(!tmp_path.vec.empty())
                        m_paths.push_back(tmp_path);
                }
            }
        }

        AStar::AStar(const NodeVec &nodes, const EdgeVec &edges, const CB &cb): Graph(nodes, edges)
        {
            for(const Node &node_s : nodes)
            {
                for(const Node &node_e : nodes)
                {
                    if(node_s.id==node_e.id)
                        continue;
                    Path tmp_path=calOnePath(node_s.id,node_e.id,cb);
                    if(!tmp_path.vec.empty())
                        m_paths.push_back(tmp_path);
                }
            }
        }
    
        Path AStar::calOnePath(int s_id, int e_id)
        {
            /// [nodeId,[prevNodeId,G]]
            std::map<int,std::tuple<int,double>> open_list;
            std::map<int,std::tuple<int,double>> closed_list;
            std::map<int,std::tuple<int,double>>::iterator iter_tmp,iter_min_cost;
            
            open_list[s_id]=std::make_tuple(-1,0);

            while(open_list.find(e_id)==open_list.end())
            {
                if(open_list.empty())
                    return Path();

                /// find the minimal cost node in open list
                double min_g=100000000;
                int min_open_id=-1;
                for(const std::pair<int,std::tuple<int,double>> &pair : open_list)
                {
                    if(std::get<1>(pair.second)<min_g)
                    {
                        min_g=std::get<1>(pair.second);
                        min_open_id=pair.first;
                    }
                }

                /// move current node to close list
                closed_list[min_open_id]=open_list[min_open_id];
                open_list.erase(min_open_id);

                /// find the nodes which current node can arrive
                for(const Edge &edge : m_edges)
                {
                    if(edge.s_id==min_open_id)
                    {
                        /// if the node has been inside closed list, ignore
                        if(closed_list.find(edge.e_id)!=closed_list.end())
                            continue;

                        /// if the node has been inside open list but it will not be closer with current node as middle
                        double tmp_dis=std::get<1>(closed_list[min_open_id])+edge.dis;
                        if(open_list.find(edge.e_id)!=open_list.end() && tmp_dis>=std::get<1>(open_list[edge.e_id]))
                            continue;

                        open_list[edge.e_id]=std::make_tuple(min_open_id,std::get<1>(closed_list[min_open_id])+edge.dis);
                    
                    }
                } 
            }
            
            std::vector<int> vec={e_id};
            int prevNodeId=std::get<0>(open_list[e_id]);
            while(prevNodeId!=-1)
            {
                vec.push_back(prevNodeId);
                prevNodeId=std::get<0>(closed_list[prevNodeId]);
            }

            Path path;
            path.s_id=s_id;
            path.e_id=e_id;
            path.vec.assign(vec.rbegin(),vec.rend());

            return path;
        }
    }
}