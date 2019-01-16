//
// Created by books on 2017/11/21.
//

#ifndef MAPTOOLS_CLOTHOID_H
#define MAPTOOLS_CLOTHOID_H

#include <iostream>
#include <cstdio>
#include <cmath>
#include <Geometry.h>
#include <RoadModel.h>
#include <functional>

#define eps1 2.2204e-016
#define inf 10000000
#define MAXD 100000000
#define MINDIS 4

namespace MapBase
{
	class Clothoid
	{

	public:
        Clothoid()= default;
		~Clothoid()= default;

		PtXYAVec Build(const PointXYA &pt_s,const PointXYA &pt_e,int npts,double tol);

		//void Smooth(const RoadVec &road_vec,ConnVec &conn_vec);

	private:
		void intXY(std::vector<double> &X, std::vector<double> &Y, int nk, double a, double b, double c);
		void evalXYaLarge(std::vector<double> &X, std::vector<double> &Y, int nk, double a, double b);
		void evalXYaSmall(std::vector<double> &X, std::vector<double> &Y, int nk, double a, double b, double p);

	};
}


#endif //MAPTOOLS_CLOTHOID_H
