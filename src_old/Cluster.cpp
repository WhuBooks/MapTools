//
// Created by books on 2017/11/2.
//

#include "Cluster.h"

namespace MapBase
{

	Cluster::Cluster()
	{
		pointbuffer=10;
		num=10;
	}

	Cluster::~Cluster()
	{

	}

	void Cluster::Load(const RoadVec &rvec, const ConnVec &cvec)
	{
		map.clear();

		roadVec.clear();
		roadVec.assign(rvec.begin(),rvec.end());

		connVec.clear();
		connVec.assign(cvec.begin(),cvec.end());

		filterRoadIDs.clear();
		filterConnIDs.clear();

		/// build map
		for (const MapBase::Road &ro : rvec)
		{
			filterRoadIDs.push_back(ro.roadid);
			for (const MapBase::Lane &la : ro.vlane)
			{
				for (const MapBase::GnssData &pt : la.vec)
				{
					MapBase::RoadPoint p;
					p.x = pt.x;
					p.y = pt.y;
					p.yaw = pt.yaw;
					p.CalYawSinAndCos();
					p.roadid = ro.roadid;
					p.laneid = la.laneid;
					p.connid = -1;
					p.slaneid=-1;
					p.elaneid=-1;
					p.reverseid=ro.reverseId;
					map.push_back(p);
				}
			}
		}
		for (const MapBase::Connline &conn : cvec)
		{
			filterConnIDs.push_back(conn.connid);
			for (const MapBase::GnssData &pt : conn.data)
			{
				MapBase::RoadPoint p;
				p.x = pt.x;
				p.y = pt.y;
				p.yaw = pt.yaw;
				p.CalYawSinAndCos();
				p.roadid = -1;
				p.laneid = -1;
				p.connid = conn.connid;
				p.slaneid=conn.slaneid;
				p.elaneid=conn.elaneid;
				p.reverseid=-1;
				map.push_back(p);
			}
		}
	}

	void Cluster::Create()
	{
		vpatch.clear();
		//find border
		double xmin = std::numeric_limits<double>::max();
		double ymin = std::numeric_limits<double>::max();
		double xmax = -std::numeric_limits<double>::max();
		double ymax = -std::numeric_limits<double>::max();
		for (const MapBase::RoadPoint &lp : map)
		{
			double x = lp.x;
			double y = lp.y;
			xmin = (x < xmin) ? x : xmin;
			xmax = (x > xmax) ? x : xmax;
			ymin = (y < ymin) ? y : ymin;
			ymax = (y > ymax) ? y : ymax;
		}

		//calculate step
		double x_step = (xmax - xmin) / num;
		double y_step = (ymax - ymin) / num;

		int id = 0;
		//create patch
		for (int i = 0; i < num; i++)
		{
			for (int j = 0; j < num; j++)
			{
				double x1 = xmin + i * x_step - pointbuffer;
				double x2 = xmin + (i + 1) * x_step + pointbuffer;
				double y1 = ymin + j * y_step - pointbuffer;
				double y2 = ymin + (j + 1) * y_step + pointbuffer;
				MapBase::Envelope env(x1, y1, x2, y2);
				MapBase::RoadPointVec vec;
				for (const MapBase::RoadPoint &p :map)
				{
					if (env.Contain(p))
						vec.push_back(p);
				}
				if (!vec.empty())
				{
					Patch pa;
					pa.patchid = id++;
					pa.data.swap(vec);
					pa.env = env;
					vpatch.push_back(pa);
				}
			}
		}
	}

	bool Cluster::Check() const
	{
		std::size_t num = 0;
		for (const Patch &pa : vpatch)
			num += pa.data.size();
		return num > map.size();
	}

	bool Cluster::Filter(const RoadPoint &rd)
	{
		for(const int roadid : filterRoadIDs)
		{
			if(rd.roadid==roadid)
				return true;
		}

		for(const int connid : filterConnIDs)
		{
			if(rd.connid==connid)
				return true;
		}

		return false;
	}

	bool Cluster::SpatialSearch(const PointXYA &pos, RoadPoint &match)
	{
		/************** spatial cluster **************/
		Patch cur_pacth;
		double min_chessboard=std::numeric_limits<double>::max();
		for(const Patch &pa : vpatch)
		{
			//double tmp=std::fabs(cur.x-pa.env.xcenter)+std::fabs(cur.y-pa.env.ycenter);
			double tmp=pa.env.Distance(pos);
			if(tmp<min_chessboard)
			{
				min_chessboard=tmp;
				cur_pacth=pa;
			}
		}

		/*************** point buffer *************/
		MapBase::Envelope curEnv(pos, 2*pointbuffer, 2*pointbuffer);
		std::vector<MapBase::RoadPoint> tmp_map;
		for(const MapBase::RoadPoint &p : cur_pacth.data)
		{
			if (curEnv.Contain(p))
			{
				if(!Filter(p))
					continue;
				tmp_map.push_back(p);
			}
		}

		if(tmp_map.empty())
		{
			for(const MapBase::RoadPoint &p : map)
			{
				if(curEnv.Contain(p))
				{
					tmp_map.push_back(p);
				}
			}
		}
		if(tmp_map.empty())
			return false;

		/************* point-point *************/
		double min_dis = std::numeric_limits<double>::max();
		bool flag=false;

		for(const MapBase::RoadPoint &p : tmp_map)
		{
			double tmp_dis = pos.Distance(p);
			bool samedirec = pos.SameDirection(p);
			if (samedirec && tmp_dis < min_dis)
			{
				min_dis = tmp_dis;
				match=p;
				flag=true;
			}
		}

		return flag;
	}

	void Cluster::SearchWithoutYaw(const PointXYA &pos, RoadPoint &match)
	{
		double xtmp=-1,ytmp=-1,yawtmp=-1,roadid=-1,laneid=-1,connid=-1;

		double minDis=std::numeric_limits<double>::max();
		for(const RoadPoint &rp : map)
		{
			double tmpDis=rp.Distance(pos);
			if(tmpDis<minDis)
			{
				minDis=tmpDis;
				match=rp;
			}
		}
		if(match.roadid==-1)
		{
			for(const Connline &co : connVec)
			{
				if(co.connid==match.connid)
				{
					match.roadid=co.sroadid;
					break;
				}
			}
		}
	}

	void Cluster::SetFilter(std::vector<int> roadids, std::vector<int> connids)
	{
		filterRoadIDs.swap(roadids);
		filterConnIDs.swap(connids);
	}

	void Cluster::ResetFilter()
	{
		filterRoadIDs.clear();
		filterConnIDs.clear();

		for(const Road &rd : roadVec)
			filterRoadIDs.push_back(rd.roadid);
		for(const Connline &co : connVec)
			filterConnIDs.push_back(co.connid);
	}
    
    void Cluster::Clear()
    {
     	roadVec.clear();
     	connVec.clear();
     	map.clear();
     	vpatch.clear();
    }
}