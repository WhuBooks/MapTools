//
// Created by 98420 on 2018/8/30.
//

#include <iostream>
#include <vector>
#include "AClass.h"

#include <Geometry/Point2D.h>
#include <Curve/Curve.h>

void LibTest()
{
    maplib::geometry::Point2D pt_s(1.0,10.0,100);
    maplib::geometry::Point2D pt_e(10.0,10.0,100);
    maplib::geometry::Pt2DVec vec=maplib::curve::clothoid<maplib::geometry::Point2D>(pt_s,pt_e);
    
}

class A
{
public:
    int x;
};

class B
{
public:
    int x;
};

template <typename T>
void Cout(std::vector<T> vector)
{
    for(int i=0;i<vector.size();i++)
        std::cout<<vector[i].x<<std::endl;
}

int main()
{
    LibTest();
    
    std::vector<A> vec1;
    std::vector<B> vec2;
    Process(vec1);
    
    for(int i=0;i<10;i++)
    {
        A a;
        B b;
        a.x=i;
        b.x=i;
        vec1.push_back(a);
        vec2.push_back(b);
    }
    Cout(vec1);
    Cout(vec2);
}