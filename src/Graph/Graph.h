//
// Created by 98420 on 2018/8/30.
//

#ifndef MAPTOOLS_GRAPH_H
#define MAPTOOLS_GRAPH_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <random>

namespace maplib
{
    namespace graph
    {
        struct Node
        {
            int id;
        };
        typedef std::vector<Node> NodeVec;
        struct Edge
        {
            int s_id;
            int e_id;
            double dis;
        };
        typedef std::vector<Edge> EdgeVec;
        
        struct Path
        {
            int s_id;
            int e_id;
            std::vector<int> vec;
        };
        typedef std::vector<Path> PathVec;
        
        class Graph
        {
        public:
            Graph(const NodeVec &nodes,const EdgeVec &edges):m_nodes(nodes),m_edges(edges) {};
            virtual ~Graph()= default;
            
            virtual bool findPath(int s_id,int e_id)const{
                for(const Path &path : m_paths)
                    if(path.s_id==s_id && path.e_id==e_id)
                        return true;
                return false;
            };
            virtual Path getPath(int s_id,int e_id)const{
                for(const Path &path : m_paths)
                    if(path.s_id==s_id && path.e_id==e_id)
                        return path;
                std::cerr<<"You Must Input Suitable SID and EID."<<std::endl;
                return m_paths.front();
            };
            
        protected:
            NodeVec m_nodes;
            EdgeVec m_edges;
            PathVec m_paths;
        };
        
    }
}

#endif //MAPTOOLS_GRAPH_H
