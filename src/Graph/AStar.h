//
// Created by books on 18-9-7.
//

#ifndef MAPTOOLS_ASTAR_H
#define MAPTOOLS_ASTAR_H

#include <iostream>
#include "Graph.h"
#include <functional>

namespace maplib
{
namespace graph
{
class AStar : public Graph
{
    typedef std::function<double(int aNodeId, int bNodeId)> CB;

public:
    AStar(const NodeVec &nodes, const EdgeVec &edges);
    AStar(const NodeVec &nodes, const EdgeVec &edges, const CB &cb);
    ~AStar() = default;

private:
    Path calOnePath(int s_id, int e_id);
    Path calOnePath(int s_id, int e_id, const CB &cb);
};
} // namespace graph
} // namespace maplib

#endif //MAPTOOLS_ASTAR_H
