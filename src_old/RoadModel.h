//
// Created by books on 2017/10/26.
//

#ifndef MAPTOOLS_ROADMODEL_H
#define MAPTOOLS_ROADMODEL_H

#include <Geometry.h>
#include <Clothoid.h>
//#include <DouglasPeucker.h>

#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <random>
#include <list>
#include <tuple>

namespace MapBase
{
//	class DouglasPeucker;

	class GnssData : public PointXYA
	{
	public:
		int id;
		double timestamp;
		double lon;
		double lat;
		double hgt;
		double roll;
		double pitch;
		double east;
		double up;
		double north;

		std::vector<GnssData> Buffer(const std::vector<GnssData> &data, double _width, double _height) const;
	};
	typedef std::vector<GnssData> GnssVec;
	typedef std::vector<GnssData>::iterator GnssVecIter;
	typedef std::vector<GnssData>::const_iterator GnssVecConstIter;

	/// scene struct
	class Scene : public PointXYA
	{
	public :
		enum Type
		{
			StopLine = 1,
			CrossLine = 2,
			LightSign = 3,
			TrafficSign = 4,
			StopArea = 5,
			SinglePath=6,
			Cone=7,
			UTurn=8
		};
		int id;
		int roadid;
		Type type;
		double width;
		double length;

		//world coordinate
		std::vector<PointXYA> region;

		void InitRegion();

		std::vector<PointXYA> LocalRegion(const PointXYA &refer) const;
	};
	typedef std::vector<Scene> SceneVec;
	typedef std::vector<Scene>::iterator SceneVecIter;

	/// struct of start and end id of different lane
	struct SEID
	{
		int s_id;
		int e_id;

		SEID()
		{
			s_id = e_id = -1;
		};

		bool operator<(const SEID &other) const
		{
			return s_id < other.s_id;
		};

		bool operator>(const SEID &other) const
		{
			return s_id > other.s_id;
		};

		bool operator==(const SEID &other) const
		{
			return s_id == other.s_id;
		};

		inline bool Contain(int id) const
		{
			return id >= s_id && id <= e_id;
		};
	};
	typedef std::vector<SEID> SEIDVec;

	/// package PointXYA, including roadid and laneid
	struct RoadPoint : public MapBase::PointXYA
	{
		int roadid;
		int laneid;
		int connid;
		int slaneid;
		int elaneid;

		int reverseid;
	};
	typedef std::vector<RoadPoint> RoadPointVec;

	struct Lane
	{
		int laneid;
		MapBase::GnssVec vec;

		bool operator<(const Lane &other) const
		{
			return laneid < other.laneid;
		}

		bool operator>(const Lane &other) const
		{
			return laneid > other.laneid;
		}

		bool operator==(const Lane &other) const
		{
			return laneid == other.laneid;
		}

		/// other lane's left point refer to this lane
		double LeftNum(const MapBase::Lane &other) const;

	};
	typedef std::vector<Lane> LaneVec;

	/// Road struct
	struct Road
	{
		int roadid;
		LaneVec vlane;
		double avelength;

		int reverseId;

		MapBase::GnssVec vs;
		MapBase::GnssVec ve;
		MapBase::GnssVec vmid;

		void Initilize();

		/// find the reverse road
		void CalReverseId(const std::vector<Road> &vroad);
		
		int IndexFromID(int laneid)const
		{
			for (int i = 0; i < vlane.size(); i++)
			{
				if (vlane[i].laneid == laneid)
					return i;
			}
			return 0;
		}

	};
	typedef std::vector<Road> RoadVec;

	struct Connline
	{
		enum Direction
		{
			TurnLeft = 0,
			TurnRight = 1,
			TurnStraight = 2,
			TurnBack = 3
		};

		int connid;
		int sroadid;
		int eroadid;
		int slaneid;
		int elaneid;
		GnssVec data;
		Direction direction;

		double Length() const
		{
			double length = 0.0;
			for (std::size_t i = 0; i < data.size() - 1; i++)
				length += data[i].Distance(data[i + 1]);
			return length;
		}

		void CalDirection();

		void InitLaneID(const RoadVec &road_vec);
		
		void ConnectLane(const RoadVec &road_vec);

		void Smooth(const GnssData &s_back,const GnssData &e_front);

		bool operator<(const Connline &other) const
		{
			return sroadid < other.sroadid ? true : sroadid == other.sroadid && eroadid < other.eroadid;
		}

		bool operator>(const Connline &other) const
		{
			return sroadid > other.sroadid ? true : sroadid == other.sroadid && eroadid > other.eroadid;
		}

		bool operator==(const Connline &other) const
		{
			return sroadid == other.sroadid && eroadid == other.eroadid;
		}
	};
	typedef std::vector<Connline> ConnVec;

	struct Path
	{
		int sroadid;
		int eroadid;
		std::vector<int> vroadid;
	};
	typedef std::vector<Path> PathVec;

	class Mission : public PointXYA
	{
	public:
		enum Type
		{
			UnUse=0,
			Used=1
		};
		int mid;
		int roadid;
		Type type;
	};
	typedef std::vector<Mission> MissionVec;
	typedef std::queue<Mission> MissionQueue;
}


#endif //MAPTOOLS_ROADMODEL_H
