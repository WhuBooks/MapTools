//
// Created by books on 18-9-7.
//

#ifndef MAPTOOLS_DIJKSTRA_H
#define MAPTOOLS_DIJKSTRA_H

#include "Graph.h"
#include <iostream>
#include <map>

namespace maplib{
    namespace graph{
        class Dijkstra:public Graph
        {
        public:
            Dijkstra(const NodeVec &nodes, const EdgeVec &edges);
            ~Dijkstra()= default;
    
//            bool findPath(int s_id,int e_id)const;
//            Path getPath(int s_id,int e_id) const;

        private:
            PathVec calOnePath(int s_id);
            
            
        };
    }
}




#endif //MAPTOOLS_DIJKSTRA_H
