//
// Created by books on 2017/11/2.
//

#ifndef MAPTOOLS_CLUSTER_H
#define MAPTOOLS_CLUSTER_H

#include <RoadModel.h>

namespace MapBase
{

	struct Patch
	{
		int patchid;
		MapBase::RoadPointVec data;
		MapBase::Envelope env;
	};

	class Cluster
	{
	public:
		Cluster();

		~Cluster();

		void Load(const RoadVec &rvec, const ConnVec &cvec);

		void Create();

		bool Check() const;

		bool Filter(const RoadPoint &rd);

		bool SpatialSearch(const PointXYA &pos, RoadPoint &match);

		void ResetFilter();

		void SetFilter(std::vector<int> roadids, std::vector<int> connids);

		void SearchWithoutYaw(const PointXYA &pos, RoadPoint &match);
		
		void Clear();
		
	private:
		int num;

		std::vector<int> filterRoadIDs;
		std::vector<int> filterConnIDs;

		RoadVec roadVec;
		ConnVec connVec;
		RoadPointVec map;
		std::vector<Patch> vpatch;
		double pointbuffer;
	};
}


#endif //MAPTOOLS_CLUSTER_H
