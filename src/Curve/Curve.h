//
// Created by 98420 on 2018/8/31.
//

#ifndef MAPTOOLS_SPARSE_H
#define MAPTOOLS_SPARSE_H

#include <iostream>
#include <vector>
#include <algorithm>

namespace maplib
{
    namespace curve
    {
        template <typename T>
        std::vector<T> douglasPeucker(const std::vector<T> &input,double sparse_threshold=0.05,double dense_threshold=1.0);
        
        template <typename T>
        std::vector<T> clothoid(const T &s,const T &e,int npts=50,double tol=0.05);
    }
}

#include "DouglasPeucker.hpp"
#include "Clothoid.hpp"

#endif //MAPTOOLS_SPARSE_H
