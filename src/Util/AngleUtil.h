//
// Created by books on 8/29/18.
//

#ifndef MAPTOOLS_ANGLEUTIL_H
#define MAPTOOLS_ANGLEUTIL_H

#include <iostream>
#include <cmath>

namespace util
{
    double degree(double val);
    
    double grad(double val);
    
    double degree2Grad(double val);
    
    double grad2Degree(double val);
    
    double degreeDiff(double degree1, double degree2);
}


#endif //MAPTOOLS_ANGLEUTIL_H
