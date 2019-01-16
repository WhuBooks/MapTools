//
// Created by books on 18-9-7.
//

#include "AStar.h"

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
    
        Path AStar::calOnePath(int s_id, int e_id)
        {
            
            
            
            
            return Path();
        }
    }
}