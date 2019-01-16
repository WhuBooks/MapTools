#include "Geometry.h"
//#define _SCL_SECURE_NO_WARNINGS
namespace MapBase{

	//static int g_current_id = 0;
	Geometry::Geometry()
	{
		//GID=g_current_id++;
	}
	Geometry::Geometry(int _id) :GID(_id)
	{
	}

	Geometry::~Geometry()
	{
	}

	PointXYA::PointXYA() :Geometry()
	{
        yaw_sin=-1000;
        yaw_cos=-1000;
	}

	PointXYA::PointXYA(double _x, double _y, double _yaw)
		: Geometry(), x(_x), y(_y), yaw(_yaw)
	{
		yaw_sin = std::sin(Degree2Grad(_yaw));
		yaw_cos = std::cos(Degree2Grad(_yaw));
//		yaw_sin=std::sin(yaw);
//		yaw_cos=std::cos(yaw);
	}

	PointXYA::~PointXYA()
	{
	}

	double PointXYA::CaculateYaw(PointXYA next)
	{
		double dx = next.x - x;
		double dy = next.y - y;
		double azi = std::atan2(dy, dx);
		return azi;
	}

	void PointXYA::CalYawSinAndCos()
	{
		yaw_sin = std::sin(Degree2Grad(yaw));
		yaw_cos = std::cos(Degree2Grad(yaw));
//		yaw_sin=std::sin(yaw);
//		yaw_cos=std::cos(yaw);
	}

	void PointXYA::Pan(double _dx, double _dy)
	{
		double xnew = x + _dx;
		double ynew = y + _dy;
	}

	PointXYA PointXYA::Shift(double right, double up)
	{
		double xnew=x-right*yaw_sin+up*yaw_cos;
		double ynew=y+right*yaw_cos+up*yaw_sin;
		return PointXYA(xnew,ynew,yaw);
	}

	/// current point is behind of pt
    bool PointXYA::IsBehind(const PointXYA &pt) const
    {
        double dx = pt.x - x;
        double dy = pt.y - y;
        double azi = std::atan2(dy, dx);

        double azi_sin=std::sin(azi);
        double azi_cos=std::cos(azi);

        return (azi_sin*yaw_sin+azi_cos*yaw_cos)>0;
    }

	/// convert this point from world soordinate system to refer's local coordinate system
	/// local coordinate system : y point to front, x point to right, not change azimuth
    PointXYA PointXYA::ToLocal(const PointXYA &refer) const
    {
        if(refer.yaw_sin==-1000||this->yaw_sin==-1000)
        {
            return PointXYA();
        }
		//get vector (local origin -> this) and local coordinate of this
		double deltax=x-refer.x;
		double deltay=y-refer.y;
		double xlocal=deltax*(-refer.yaw_sin)+deltay*refer.yaw_cos;
		double ylocal=deltax*refer.yaw_cos+deltay*refer.yaw_sin;
		double yawnew=yaw;
		return PointXYA(xlocal,ylocal,yawnew);
    }

	/// convert this point from refer's local coordinate system to world coordinate system
	/// world coordinate system : x point to north, y point to east, azimuth is the clockwise angle from x axis
	PointXYA PointXYA::ToWorld(const PointXYA &refer) const
	{
		if(refer.yaw_sin==-1000||this->yaw_sin==-1000)
		{
			return PointXYA();
		}
		//calculate the local coordinate of world origin
		double deltax=-refer.x;
		double deltay=-refer.y;
		double xorigin=deltax*(-refer.yaw_sin)+deltay*refer.yaw_cos;
		double yorigin=deltax*refer.yaw_cos+deltay*refer.yaw_sin;

		//get vector (world origin -> this) and world coordinate of this
		deltax=x-xorigin;
		deltay=y-yorigin;
		double xworld=deltax*(-refer.yaw_sin)+deltay*refer.yaw_cos;
		double yworld=deltax*refer.yaw_cos+deltay*refer.yaw_sin;
		double yawworld=yaw;
		return PointXYA(xworld,yworld,yawworld);
	}

	double PointXYA::Distance(const PointXYA &pt) const
	{
		double dx = std::fabs(pt.x - x);
		double dy = std::fabs(pt.y - y);
		return std::sqrt(dx*dx + dy*dy);
	}

	bool PointXYA::SameDirection(const PointXYA &pt) const
	{
		return (yaw_sin*pt.yaw_sin + yaw_cos*pt.yaw_cos > 0);
	}

    PointXYA PointXYA::ReverseDir() const
    {
    	double reverse_yaw=this->yaw+180.0;
    	if(reverse_yaw>360.0)
			reverse_yaw=reverse_yaw-360;
		return PointXYA(x,y,reverse_yaw);
    }

    Envelope::Envelope() : Geometry()
	{

	}

	Envelope::Envelope(PointXYA center, double width, double height) : Geometry()
	{
		xcenter = center.x;
		ycenter = center.y;
		xmin = xcenter - width / 2;
		xmax = xcenter + width / 2;
		ymin = ycenter - height / 2;
		ymax = ycenter + height / 2;
        ptcen=PointXYA(xcenter,ycenter);
	}

	Envelope::Envelope(double _xmin, double _ymin, double _xmax, double _ymax)
		:Geometry(), xmin(_xmin), ymin(_ymin), xmax(_xmax), ymax(_ymax)
	{
		xcenter = _xmin + (_xmax - _xmin) / 2;
		ycenter = _ymin + (_ymax - _ymin) / 2;
        ptcen=PointXYA(xcenter,ycenter);
	}

	bool Envelope::Contain(const PointXYA &p) const
	{
		return p.x < xmax && p.x > xmin && p.y > ymin && p.y < ymax;
	}

	bool Envelope::Overlap(const Envelope &e) const
	{
		return false;
	}

	void Envelope::Translation(double _dx, double _dy)
	{
		xmin += _dx;
		xmax += _dx;
		xcenter += _dx;
		ymin += _dy;
		ymax += _dy;
		ycenter += _dy;
	}

	void Envelope::Resize(double _xFac, double _yFac)
	{
		double halfwidth = (xmax - xmin) / 2 * _xFac;
		double halfheight = (ymax - ymin) / 2 * _yFac;
		xmin = xcenter - halfwidth;
		xmax = xcenter + halfwidth;
		ymin = xcenter - halfheight;
		ymax = ycenter + halfheight;
	}

    double Envelope::Distance(const PointXYA &pt) const
    {
        return ptcen.Distance(pt);
    }

	Polyline::Polyline() :Geometry()
	{
	}

	Polyline::Polyline(const std::vector<PointXYA> &pVec) : Geometry()
	{
		ptVec.reserve(pVec.size() + 5);
		//std::copy(pVec.begin(), pVec.end(), ptVec);
		//std::memcpy(ptVec.data(), pVec.data(), pVec.size());
        ptVec.assign(pVec.begin(),pVec.end());
	}

	Envelope Polyline::GetEnvelope()
	{
		double xmin = std::numeric_limits<double>::max();
		double xmax = std::numeric_limits<double>::min();
		double ymin = xmin;
		double ymax = xmax;
		for(const PointXYA &pt : ptVec)
		{
			xmin = (pt.x < xmin) ? pt.x : xmin;
			xmax = (pt.x > xmax) ? pt.x : xmax;
			ymin = (pt.y < ymin) ? pt.y : ymin;
			ymax = (pt.y > ymax) ? pt.y : ymax;
		}
		return Envelope(xmin, ymin, xmax, ymax);
	}

	void Polyline::Translation(double _dx, double _dy)
	{
		for(PointXYA &pt : ptVec)
		{
			pt.x += _dx;
			pt.y += _dy;
		}
	}

	double Polyline::GetLength(int start, int end)
	{
		int num = (int) (ptVec.size() - 1);
		if (end <= start || start < 0 || start >= ptVec.size() - 2||end>=ptVec.size())
		{
			return 0;
		}
		double length = 0;
		for (int i = start; i < end; i++)
		{
			PointXYA pt1 = ptVec[i];
			PointXYA pt2 = ptVec[i + 1];
			double dis = pt1.Distance(pt2);
			length += dis;
		}
		return length;
	}

}