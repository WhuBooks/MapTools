//
// Created by books on 18-9-7.
//

#ifndef MAPTOOLS_ASTAR_H
#define MAPTOOLS_ASTAR_H

#include <iostream>
#include "Graph.h"

namespace maplib
{
    namespace graph{
        class AStar:public Graph
        {
        public:
            AStar(const NodeVec &nodes, const EdgeVec &edges);
            ~AStar()= default;

        private:
            Path calOnePath(int s_id,int e_id);
        };
    }
}




#endif //MAPTOOLS_ASTAR_H
