//
// Created by 98420 on 2018/8/31.
//
#include "AngleUtil.h"

namespace util
{
    double degree(double val)
    {
        while (val < 0.0)
            val += 360.0;
        while (val >= 360.0)
            val -= 360.0;
        return val;
    }
    
    double grad(double val)
    {
        while (val < 0.0)
            val += 2.0 * M_PI;
        while (val >= 2.0 * M_PI)
            val -= 2.0 * M_PI;
        return val;
    }
    
    double degree2Grad(double val)
    {
        return degree(val) * M_PI / 180.0;
    }
    
    double grad2Degree(double val)
    {
        return grad(val) * 180.0 / M_PI;
    }
    
    double degreeDiff(double degree1, double degree2)
    {
        double diff = std::abs(degree(degree1) - degree(degree2));
        if (diff > 180.0)
            diff = 360.0 - diff;
        return degree(diff);
    }
}