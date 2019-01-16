//
// Created by books on 2017/10/26.
//

#include "RoadModel.h"
#include "FileIO.h"
#include <DouglasPeucker.h>

namespace MapBase
{
	GnssVec GnssData::Buffer(const std::vector<GnssData> &data, double _width, double _height) const
	{
		GnssVec result;
		PointXYA center(x, y, yaw);
		Envelope env(center, _width, _height);

		for (const GnssData &gd : data)
		{
			if (env.Contain(gd))
			{
				result.push_back(gd);
			}
		}
		return result;
	}

	void Scene::InitRegion()
	{
		double half_length=3.0;
		double half_width=3.0;
		double step_length=0.2;
		double step_width=0.2;
		if(type==Type::CrossLine)
		{
			half_width = this->width / 2.0;
			half_length = this->length / 2.0;
			step_width = this->width / 10.0;
			step_length = this->length / 10.0;
		}

		for (double i = -half_width; i <= half_width; i = i + step_width)
		{
			for (double j = -half_length; j <= half_length; j = j + step_length)
			{
				PointXYA tmp = this->Shift(i, j);
				region.push_back(tmp);
			}
		}

	}

	std::vector<PointXYA> Scene::LocalRegion(const PointXYA &refer) const
	{
		std::vector<PointXYA> vlocal;
		vlocal.reserve(region.size());
		for (const PointXYA &tmp : region)
		{
			PointXYA tmplocal = tmp.ToLocal(refer);
			vlocal.push_back(tmplocal);
		}
		return vlocal;
	}


	void Connline::Smooth(const GnssData &s_back, const GnssData &e_front)
	{
		MapBase::Clothoid clothoid;

		MapBase::GnssVec smooth_s;
		for(int i=0;i<data.size();i++)
		{
			MapBase::GnssData gd_i=data[i];
			MapBase::PointXYA gd_i_local=gd_i.ToLocal(s_back);
			double tmppp_y=gd_i_local.y;
			double tmppp_x=gd_i_local.x;
			if(std::abs(gd_i_local.y)<std::abs(gd_i_local.x) || gd_i_local.y<0)
				continue;

			MapBase::PtXYAVec tmp_smmoth_s=clothoid.Build(s_back,gd_i,50,0.05);
			if(!tmp_smmoth_s.empty())
			{
				for(const MapBase::PointXYA &pt : tmp_smmoth_s)
				{
					MapBase::GnssData gd_pt;
					gd_pt.id=GetNewGnssID();
					gd_pt.x=pt.x;
					gd_pt.y=pt.y;
					gd_pt.yaw=pt.yaw;
					gd_pt.up=s_back.up;
					gd_pt.north=s_back.north;
					gd_pt.east=s_back.east;
					gd_pt.CalYawSinAndCos();
					smooth_s.push_back(gd_pt);
				}
				smooth_s.insert(smooth_s.end(),data.begin()+i+1,data.end());
				break;
			}
		}

		if(smooth_s.empty())
			return;
//		std::cout<<"Smooth Front OK"<<std::endl;
		data.clear();
		data.assign(smooth_s.begin(),smooth_s.end());

		for(int i=smooth_s.size()-1;i>=0;i--)
		{
			MapBase::GnssData gd_i=smooth_s[i];
			MapBase::PointXYA gd_i_local=gd_i.ToLocal(e_front);
			if(std::abs(gd_i_local.y)<3*std::abs(gd_i_local.x) || gd_i_local.y>0)
				continue;

			MapBase::PtXYAVec tmp_smooth_e=clothoid.Build(gd_i,e_front,50,0.05);
			if(!tmp_smooth_e.empty())
			{
				MapBase::GnssVec result;
				result.assign(smooth_s.begin(),smooth_s.begin()+i-1);
				for(const MapBase::PointXYA &pt : tmp_smooth_e)
				{
					MapBase::GnssData gd_pt;
					gd_pt.id=GetNewGnssID();
					gd_pt.x=pt.x;
					gd_pt.y=pt.y;
					gd_pt.yaw=pt.yaw;
					gd_pt.up=e_front.up;
					gd_pt.north=e_front.north;
					gd_pt.east=e_front.east;
					gd_pt.CalYawSinAndCos();
					result.push_back(gd_pt);
				}
				data.swap(result);
				return;
			}
		}
	}

	void Connline::CalDirection()
	{
		GnssData first = data.front();
		PointXYA lastlocal = data.back().ToLocal(first);
		double lastx = lastlocal.x;
		double lasty = lastlocal.y;
		double lastangle = std::atan2(lasty, lastx);

		if (lasty < 0)
			direction = TurnBack;
		else if (lastangle < M_PI * 0.4 && lastangle > 0)
			direction = TurnRight;
		else if (lastangle > 0.6 * M_PI && lastangle < M_PI)
			direction = TurnLeft;
		else
			direction = TurnStraight;
	}

	void Connline::InitLaneID(const RoadVec &road_vec)
	{
		slaneid = -1;
		elaneid = -1;
		GnssData s_back;
		GnssData e_front;
		for (const Road &road : road_vec)
		{
			if (road.roadid == sroadid)
			{
				double min_local_dis = 100000000000;
				for (const Lane &lane : road.vlane)
				{
					PointXYA local = this->data.front().ToLocal(lane.vec.back());
					if (std::abs(local.x) < min_local_dis)
					{
						min_local_dis = local.x;
						slaneid = lane.laneid;
						s_back = lane.vec.back();
					}
				}
			}
			
			if (road.roadid == eroadid)
			{
				double min_local_dis = 100000000000;
				for (const Lane &lane : road.vlane)
				{
					PointXYA local = this->data.back().ToLocal(lane.vec.front());
					if (std::abs(local.x) < min_local_dis)
					{
						min_local_dis = local.x;
						elaneid = lane.laneid;
						e_front = lane.vec.front();
					}
				}
			}
		}
		if (true)
		{
			Smooth(s_back, e_front);
			DouglasPeucker *dp = new DouglasPeucker();
			GnssVec vec = dp->Process(data);
			data.swap(vec);
		}
	}
	
	void Connline::ConnectLane(const RoadVec &road_vec)
	{
		GnssData s_back;
		GnssData e_front;
		for (const Road &road : road_vec)
		{
			if (road.roadid == sroadid)
			{
				for (const Lane &lane : road.vlane)
				{
					if (lane.laneid==slaneid)
						s_back = lane.vec.back();
				}
			}
			
			if (road.roadid == eroadid)
			{
				for (const Lane &lane : road.vlane)
				{
					if (lane.laneid==elaneid)
						e_front = lane.vec.front();
				}
			}
		}
		if (false)
		{
			Smooth(s_back, e_front);
			DouglasPeucker *dp = new DouglasPeucker();
			GnssVec vec = dp->Process(data);
			data.swap(vec);
		}
	}
	
	double Lane::LeftNum(const MapBase::Lane &other) const
	{
		int left_num = 0;
		for (const MapBase::GnssData &gd_other : other.vec)
		{
			double min_dis = 10000000000000;
			MapBase::GnssData gd_min_dis;
			for (const MapBase::GnssData &gd : vec)
			{
				double dis = gd.Distance(gd_other);
				if (dis < min_dis)
				{
					min_dis = dis;
					gd_min_dis = gd;
				}
			}

			MapBase::PointXYA pt_local = gd_min_dis.ToLocal(gd_other);
			if (pt_local.x < 0)
				left_num++;
		}

		return (double) left_num / vec.size();
	}

	void Road::Initilize()
	{
		reverseId=-1;
		avelength = 0.0;
		for (const Lane &la : vlane)
		{
			GnssData front = la.vec.front();
			GnssData back = la.vec.back();
			GnssData mid=la.vec[la.vec.size()/2];
			vs.push_back(front);
			ve.push_back(back);
			vmid.push_back(mid);
			double tmpdis = 0.0;
			for (std::size_t i = 0; i < la.vec.size() - 1; i++)
			{
				GnssData s = la.vec[i];
				GnssData e = la.vec[i + 1];
				tmpdis += s.Distance(e);
			}
			avelength += tmpdis;
		}
		avelength /= vlane.size();

		/// recalculate the vlane
		std::sort(vlane.begin(),vlane.end(),[](const MapBase::Lane &la1,const MapBase::Lane la2){
			return la1.LeftNum(la2)>la2.LeftNum(la1);
		});

	}

	void Road::CalReverseId(const std::vector<Road> &vroad)
	{
		double minDis=std::numeric_limits<double>::max();
		reverseId=-1;
		for(const Road &ro : vroad)
		{
			if(ro.roadid==roadid)
				continue;

			for(const GnssData &mid1: vmid)
			{
				for(const GnssData &mid2: ro.vmid)
				{
					double tempDis=mid1.Distance(mid2);
					if(tempDis<minDis)
					{
						minDis=tempDis;
						reverseId=ro.roadid;
					}
				}
			}
		}
	}


}