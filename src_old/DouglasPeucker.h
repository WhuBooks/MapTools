//
// Created by books on 2018/3/23.
//

#ifndef MAPTOOLS_DOUGLASPEUCKER_H
#define MAPTOOLS_DOUGLASPEUCKER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

#include <Geometry.h>
#include <RoadModel.h>

namespace MapBase
{

    class DouglasPeucker
    {
    public:
        DouglasPeucker();
        DouglasPeucker(double sparse,double dense);

        ~DouglasPeucker();

        GnssVec Process(const GnssVec &input,bool check=false);
        
    private:

        void CheckMapBase(const GnssVec &data,std::vector<int> &idlst);

        void Doglas_Puke(int start, int end, std::vector<int> &idlst,const GnssVec &data);

        void Dense(std::vector<int> &idlst, const GnssVec &data);

        static const double GeoDisThreshold;
        static const double GeoAngleThreshold;
        double m_SparseThreshold;
        double m_DenseThreshold;

    };
}

#endif //MAPTOOLS_DOUGLASPEUCKER_H
