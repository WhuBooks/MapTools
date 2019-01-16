//
// Created by 98420 on 2018/8/30.
//

#ifndef MAPTOOLS_FLOYD_H
#define MAPTOOLS_FLOYD_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <random>

#include "Graph.h"

namespace maplib
{
    namespace graph
    {
        class Floyd : public Graph
        {
        public:
            Floyd(const NodeVec &nodes, const EdgeVec &edges);
            ~Floyd() = default;
    
//            bool findPath(int s_id,int e_id)const;
//            Path getPath(int s_id,int e_id) const;
            
        private:
            void calAllPaths();
            
//            PathVec m_paths;
        };
        
    }
}




#endif //MAPTOOLS_FLOYD_H
