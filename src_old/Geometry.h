#ifndef _MAPBASE_GEOMETRY_H_
#define _MAPBASE_GEOMETRY_H_

#include <cmath>
#include <numeric>
#include <vector>
#include <algorithm>

namespace MapBase
{

#ifdef WIN32
#define M_PI 3.141592657
#endif

	static double Degree(double val)
	{
		while (val < 0.0 || val >= 360.0)
		{
			if (val < 0.0)
				val += 360.0;
			else if (val >= 360.0)
				val -= 360.0;
		}
		return val;
	}

	static double Grad(double val)
	{
		while (val < 0.0 || val >= 2 * M_PI)
		{
			if (val < 0.0)
				val += 2 * M_PI;
			else if (val >= 2 * M_PI)
				val -= 2 * M_PI;
		}
		return val;
	}

	static double Degree2Grad(double val)
	{
		double degree = Degree(val);
		double grad = degree * M_PI / 180;
		return Grad(grad);
	}

	static double Grad2Degree(double val)
	{
		double grad = Grad(val);
		double degree = grad * 180 / M_PI;
		return Degree(degree);
	}

	static void Wgs84ToGauss(double lon, double lat,int distinguish, double &x, double &y)
	{

		const double RHO = 206265;

		int order = 0;
		if (distinguish == 6)
			order = int(lon / 6) + 1;
		else
			order = int(lon / 3 + 0.5);

		int L0 = 0;
		if (distinguish == 6)
			L0 = 6 * order - 3;
		else
			L0 = 3 * order;

		double CosB = std::cos(Degree2Grad(lat));
		double SinB = std::sin(Degree2Grad(lat));
		double CosBSquare = CosB * CosB;
		double SinBSquare = SinB * SinB;

		double l = ((lon - L0) * 3600) / RHO;

		double N = 6399698.902 - (21562.267 - (108.973 - 0.612 * CosBSquare) * CosBSquare) * CosBSquare;

		double a0 = 32140.404 - (135.3302 - (0.7092 - 0.0040 * CosBSquare) * CosBSquare) * CosBSquare;

		double a4 = (0.25 + 0.00252 * CosBSquare) * CosBSquare - 0.04166;

		double a6 = (0.166 * CosBSquare - 0.084) * CosBSquare;

		double a3 = (0.3333333 + 0.001123 * CosBSquare) * CosBSquare - 0.1666667;

		double a5 = 0.0083 - (0.1667 - (0.1968 + 0.0040 * CosBSquare) * CosBSquare) * CosBSquare;

		x = 6367558.4969 * (lat * 3600) / RHO - (a0 - (0.5 + (a4 + a6 * l * l) * l * l)
													  * l * l * N) * SinB * CosB;
		y = (1 + (a3 + a5 * l * l) * l * l) * l * N * CosB+500000;

	}
	
	class Geometry
	{
	public:
		Geometry();

		Geometry(int _id);

		virtual ~Geometry();

		virtual void Translation(double _dx, double _dy) {};

		virtual void Resize(double xFac, double yFac) {};
	protected:
		int GID;
	};

	class PointXYA : public Geometry
	{
	public:
		PointXYA();

		PointXYA(double _x, double _y, double _yaw = 0);

		~PointXYA();

		double CaculateYaw(PointXYA next);

		void CalYawSinAndCos();

		void Pan(double _dx, double _dy);

		PointXYA Shift(double right, double up);

		double Distance(const PointXYA &pt) const;

		bool SameDirection(const PointXYA &pt) const;

		bool IsBehind(const PointXYA &pt) const;

		//local->world or world->local both base on a world coordinate origin
		PointXYA ToLocal(const PointXYA &) const;

		PointXYA ToWorld(const PointXYA &) const;

		PointXYA ReverseDir() const;

		double x, y, yaw;
		double yaw_sin, yaw_cos;

		bool operator==(const PointXYA &pt) const
		{
			return x == pt.x && y == pt.y && yaw == pt.yaw;
		};

		bool operator>(const PointXYA &pt) const
		{
			return x > pt.x || (x == pt.x && y > pt.y) || (x == pt.x && y == pt.y && yaw > pt.yaw);
		};

		bool operator<(const PointXYA &pt) const
		{
			return x < pt.x || (x == pt.x && y < pt.y) || (x == pt.x && y == pt.y && yaw < pt.yaw);
		};

	private:

	};

	typedef std::vector<PointXYA> PtXYAVec;
	typedef std::vector<PointXYA>::iterator PtXYAVecIter;

	class Envelope : public Geometry
	{
	public:
		Envelope();

		Envelope(PointXYA center, double width, double height);

		Envelope(double _xmin, double _ymin, double _xmax, double _ymax);

		bool Contain(const PointXYA &p) const;

		bool Overlap(const Envelope &e) const;

		void Translation(double _dx, double _dy);

		void Resize(double _xFac, double yFac);

		double Distance(const PointXYA &pt) const;

		double xmin, xmax, ymin, ymax;
		double xcenter, ycenter;
		PointXYA ptcen;
	};

	class Polyline : public Geometry
	{
	public:
		enum DisType
		{
			AveDis = 0,
			MinDis = 1,
			MidDis = 2,
			MaxDis = 3
		};

		Polyline();

		Polyline(const std::vector<PointXYA> &pVec);

		Envelope GetEnvelope();

		void Translation(double _dx, double _dy) override;

		double GetLength(int start, int end);

//		double CalDistance(const PointXYA &pt,DisType type,bool usedirection=true) const;

		std::vector<PointXYA> ptVec;

	};


}


#endif
