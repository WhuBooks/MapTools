#ifndef _MAPBASE_FILEIO_H_
#define _MAPBASE_FILEIO_H_

#include <string>
#include <string.h>
#include <vector>
#include "Geometry.h"
#include "RoadModel.h"
#include "Graph.h"
#include <fstream>
#include <ctime>
#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <queue>

#ifdef WIN32
#include <direct.h>
#include <io.h>
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#endif

namespace MapBase
{


	int GetNewGnssID();

	bool ReadGnssWithoutID(std::string filename, GnssVec &vec);

	bool ReadGnss(std::string filename, GnssVec &vec);

	bool WriteGnss(std::string filename, const GnssVec &vec);

	bool ReadSEID(const std::string &filename, SEIDVec &vec);

	bool ReadSEID2(const std::string &filename, SEIDVec &vec);

	bool ReadScene(std::string filename, SceneVec &vec);

	bool WriteScene(std::string filename, const SceneVec &vec);

	bool ReadRoad(std::string dir, RoadVec &vec);

	bool WriteRoad(std::string dir, const RoadVec &vec);

	bool ReadConline(std::string dir, ConnVec &vec);
	
	bool ReadConlineLaneID(std::string dir,ConnVec &vec);

	bool WriteConline(std::string dir, const ConnVec &vec);

	bool ReadPath(std::string filename, PathVec &vec);

	bool WritePath(std::string filename, const PathVec &vec);

	bool ReadMission(std::string filename, MissionVec &vec);

	bool ReadMission(std::string filename, MissionQueue &queue);

	bool WriteMission(std::string filename, const MissionVec &vec);

	std::string GetNameFromTime();

	bool DirExist(std::string path);

	bool DirBuild(std::string path);

	std::vector<std::string> GetFiles(const std::string &path);

	std::string SplitDirectory(const std::string &path);

	std::string SplitNameWithExt(const std::string &path);

	std::string SplitNameWithoutExt(const std::string &path);

	std::string SplitExtention(const std::string &path);
}

#endif


