#include "FileIO.h"
#include <iomanip>
#include <cassert>

namespace MapBase
{
    static int curid = 0;
    static int g_max_gnss_id = 0;

    int GetNewGnssID()
    {
        return g_max_gnss_id++;
    }

    bool ReadGnssWithoutID(std::string filename, GnssVec &vec)
    {
        std::ifstream infile;
        infile.open(filename, std::ios::in);
        if (!infile.is_open())
        {
            std::cout << "[ FileIO - ReadGnssWithoutID Error ] ~ " << filename << std::endl;
            return false;
        }

        while (infile.good() && !infile.eof())
        {
            GnssData data;
            double roll=0,pitch=0,yaw=0;
            data.id = curid++;
            infile >> data.timestamp >> data.lon >> data.lat >> data.x >> data.y;
            infile >> data.hgt >> data.roll >> data.pitch >> data.yaw >> data.east >> data.up >> data.north;
            data.CalYawSinAndCos();
            vec.push_back(data);
            g_max_gnss_id = g_max_gnss_id > data.id ? g_max_gnss_id : data.id;
        }
        if (!vec.empty())
            vec.erase(vec.end() - 1);
        infile.close();
        return true;
    }

    bool ReadGnss(std::string filename, GnssVec &vec)
    {
        std::ifstream infile;
        infile.open(filename);
        if (!infile.is_open())
        {
            std::cout << "[ FileIO - ReadGnss Error ] ~ " << filename << std::endl;
            return false;
        }

        while (infile.good() && !infile.eof())
        {
            GnssData data;
            double roll = 0, pitch = 0, yaw = 0;
            infile >> data.id >> data.timestamp;
            infile >> data.lon >> data.lat >> data.x >> data.y >> data.hgt;
            infile >> data.roll >> data.pitch >> data.yaw;
            infile >> data.east >> data.up >> data.north;
            data.CalYawSinAndCos();
            vec.push_back(data);
            g_max_gnss_id = g_max_gnss_id > data.id ? g_max_gnss_id : data.id;
        }

        if (!vec.empty())
            vec.erase(vec.end() - 1);
        infile.close();
        return true;
    }

    bool WriteGnss(std::string filename,const GnssVec &vec)
    {
        std::ofstream outfile;
        outfile.open(filename, std::ios::out | std::ios::trunc);
        if (!outfile.is_open())
        {
            std::cout << "[ FileIO - WriteGnss Error ] ~ " << filename << std::endl;
            return false;
        }

        outfile << std::setiosflags(std::ios::fixed);
        for (const GnssData &data : vec)
        {
            outfile << std::setprecision(0) << data.id << "\t";
            outfile << std::setprecision(3) << data.timestamp << "\t";
            outfile << std::setprecision(9) << data.lon << "\t";
            outfile << std::setprecision(9) << data.lat << "\t";
            outfile << std::setprecision(5) << data.x << "\t";
            outfile << std::setprecision(5) << data.y << "\t";
            outfile << std::setprecision(8) << data.hgt << "\t";
            outfile << std::setprecision(6) << data.roll << "\t";
            outfile << std::setprecision(6) << data.pitch << "\t";
            outfile << std::setprecision(6) << data.yaw << "\t";
            outfile << std::setprecision(6) << data.east << "\t";
            outfile << std::setprecision(6) << data.up << "\t";
            outfile << std::setprecision(6) << data.north << std::endl;
        }
        outfile.flush();
        outfile.close();
        return true;
    }

    bool ReadSEID(const std::string &filename, SEIDVec &vec)
    {
        std::ifstream infile;
        infile.open(filename, std::ios::in);
        if (!infile.is_open())
        {
            std::cout << "[ FileIO - ReadSEID Error ] ~ " << filename << std::endl;
            return false;
        }

        while (infile.good() && !infile.eof())
        {
            int s, e;
            infile >> s >> e;
            SEID id;
            id.s_id = std::min(s,e);
            id.e_id = std::max(s,e);
            vec.push_back(id);
        }
        infile.close();
        if (!vec.empty())
            vec.erase(vec.end() - 1);

        std::sort(vec.begin(), vec.end());
        for (size_t i = 0; i < vec.size() - 1; i++)
        {
            SEID &id1 = vec[i];
            SEID &id2 = vec[i + 1];
            if (id2.s_id == id1.e_id)
            {
                id2.s_id = id2.s_id + 1;
                id1.e_id = id1.e_id - 1;
            }
        }

        return true;
    }

	bool ReadSEID2(const std::string &filename,SEIDVec &vec)
	{
		std::ifstream infile;
		infile.open(filename, std::ios::in);
		if (!infile.is_open())
		{
			std::cout << "[ FileIO - ReadSEID Error ] ~ " << filename << std::endl;
			return false;
		}

		std::vector<int> origin;
		while (infile.good() && !infile.eof())
		{
			int s;
			infile>>s;
			origin.push_back(s);
		}
		infile.close();
		if (!origin.empty())
			origin.erase(origin.end() - 1);

		std::sort(origin.begin(),origin.end());
		assert(origin.size()%2==0);

		for(int i=0;i<origin.size();i=i+2)
		{
			int s=origin[i];
			int e=origin[i+1];

			SEID id;
			id.s_id = s;
			id.e_id = e;
			std::cout<<"s = "<<s<<"\te = "<<e<<"\tdis = "<<e-s<<std::endl;
			vec.push_back(id);
		}

		std::sort(vec.begin(), vec.end());
		for (size_t i = 0; i < vec.size() - 1; i++)
		{
			SEID &id1 = vec[i];
			SEID &id2 = vec[i + 1];
			if (id2.s_id == id1.e_id)
			{
				id2.s_id = id2.s_id + 1;
				id1.e_id = id1.e_id - 1;
			}
		}
        return true;
	}

    bool ReadScene(std::string filename, SceneVec &vec)
    {
        std::ifstream infile;
        infile.open(filename, std::ios::in);
        if (!infile.is_open())
        {
            std::cout << "[ FileIO - ReadScene Error ] ~ " << filename << std::endl;
            return false;
        }

        while (infile.good() && !infile.eof())
        {
            Scene feat;
            int type = 0;
            infile >> feat.id >> feat.roadid >> feat.x >> feat.y >> feat.yaw;
            infile >> type >> feat.length >> feat.width;
            feat.type = (Scene::Type) type;
            feat.CalYawSinAndCos();
            vec.push_back(feat);
        }
        if (!vec.empty())
            vec.erase(vec.end() - 1);
        infile.close();
        return true;
    }

    bool WriteScene(std::string filename,const SceneVec &vec)
    {
        std::ofstream outfile;
        outfile.open(filename,std::ios::out|std::ios::trunc);
        if(!outfile.is_open())
        {
            std::cout<<"[ FileIO - WriteScene Error ] ~ "<<filename<<std::endl;
            return false;
        }

        outfile << std::setiosflags(std::ios::fixed);
        for(const Scene sn : vec)
        {
            outfile << sn.id << "\t";
            outfile << sn.roadid << "\t";
            outfile << std::setprecision(5) << sn.x << "\t";
            outfile << std::setprecision(5) << sn.y << "\t";
            outfile << std::setprecision(5) << sn.yaw << "\t";
            outfile << (int) sn.type << "\t";
            outfile << std::setprecision(2) << sn.length << "\t";
            outfile << std::setprecision(2) << sn.width << std::endl;
        }

        return true;
    }

    bool ReadRoad(std::string dir, RoadVec &vec)
    {
        //read index file
        std::string indexfile = dir + "/roadindex.txt";
        std::string roaddir = dir + "/Lane";
        std::ifstream infile;
        infile.open(indexfile, std::ios::in);
        if (!infile.is_open())
        {
            std::cerr << "[ FileIO - ReadRoad Error ] ~ " << indexfile << std::endl;
            return false;
        }
        std::map<int, std::vector<int>> vlaneid;
        while (infile.good() && !infile.eof())
        {
            int roadid=-1;
            int laneid=-1;
            infile >> roadid >> laneid;
            std::string filename = roaddir + "/" + std::to_string(laneid) + ".txt";
            vlaneid[roadid].push_back(laneid);
        }

        //read road file
        std::map<int, std::vector<int>>::iterator lanefileiter;
        for (lanefileiter = vlaneid.begin(); lanefileiter != vlaneid.end(); lanefileiter++)
        {
            Road ro;
            ro.roadid = lanefileiter->first;
            if(ro.roadid==-1)
                continue;
            LaneVec lane_vec;
            for (const int &laneid : lanefileiter->second)
            {
                GnssVec gnss_vec;
                std::string lanefile = roaddir + "/" + std::to_string(laneid) + ".txt";
                bool flag = ReadGnss(lanefile, gnss_vec);
                if (flag)
                {
                    Lane lane;
                    lane.laneid = laneid;
                    lane.vec.swap(gnss_vec);
                    lane_vec.push_back(lane);
                }
            }
            /// sort by left num
//            std::sort(lane_vec.begin(),lane_vec.end(),[](const Lane &la1,const Lane &la2){
//               return la1.LeftNum(la2)>la2.LeftNum(la1);
//            });
//            ro.vlane.swap(lane_vec);
            ro.vlane.swap(lane_vec);
            ro.Initilize();
            vec.push_back(ro);
        }

        // calculate reverse road relation
        for(Road &ro : vec)
            ro.CalReverseId(vec);

        return true;
    }

    bool WriteRoad(std::string dir,const RoadVec &vec)
    {
        std::string indexfile = dir + "/roadindex.txt";
        std::ofstream outfile;
        outfile.open(indexfile, std::ios::out);
        if (!outfile.is_open())
        {
            std::cerr << "[ FileIO - WriteRoad Error ] ~ " << indexfile << std::endl;
            return false;
        }
        for (const Road &ro : vec)
        {
            for (const Lane &la : ro.vlane)
            {
                outfile << ro.roadid << "\t";
                outfile << la.laneid << std::endl;
            }
        }
        outfile.flush();
        outfile.close();
        return true;
    }

    bool ReadConline(std::string dir, ConnVec &vec)
    {
        //read index file
        std::string indexfile = dir + "/connindex.txt";
        std::string conndir = dir + "/Conn";
        std::ifstream infile;
        infile.open(indexfile, std::ios::in);
        if (!infile.is_open())
        {
            std::cerr << "[ FileIO - ReadConline Error ] ~ " << indexfile << std::endl;
            return false;
        }
        std::map<int,int> sroad_map,eroad_map;
        std::map<int,int>::iterator sroad_iter,eroad_iter;
        while(infile.good() && !infile.eof())
        {
            int connid=-1, sroadid=-1, eroadid=-1;
            infile >> connid >> sroadid >> eroadid;
            sroad_map[connid]=sroadid;
            eroad_map[connid]=eroadid;
        }
        
        for(sroad_iter=sroad_map.begin(),eroad_iter=eroad_map.begin();sroad_iter!=sroad_map.end() && eroad_iter!=eroad_map.end();sroad_iter++,eroad_iter++)
        {
            int connid=sroad_iter->first;
            int sroadid=sroad_iter->second;
            int eroadid=eroad_iter->second;
            if(connid==-1)
                continue;
            GnssVec data;
            std::string confile = conndir + "/" + std::to_string(connid) + ".txt";
            bool readflag = ReadGnss(confile, data);
            if (readflag)
            {
                Connline cl;
                cl.connid = connid;
                cl.sroadid = sroadid;
                cl.eroadid = eroadid;
                cl.data = data;
                cl.CalDirection();
                vec.push_back(cl);
            }
        }
//        while (infile.good() && !infile.eof())
//        {
//            int connid, sroadid, eroadid;
//            infile >> connid >> sroadid >> eroadid;
//            GnssVec data;
//            std::string confile = conndir + "/" + std::to_string(connid) + ".txt";
//            bool readflag = ReadGnss(confile, data);
//            if (readflag)
//            {
//                Connline cl;
//                cl.connid = connid;
//                cl.sroadid = sroadid;
//                cl.eroadid = eroadid;
//                cl.data = data;
//                cl.CalDirection();
//                vec.push_back(cl);
//            }
//        }

//        if (!vec.empty())
//            vec.erase(vec.end() - 1);

        infile.close();
        return true;
    }
    
    bool ReadConlineLaneID(std::string dir,ConnVec &vec)
    {
        std::string index_file=dir+"/laneindex.txt";
        std::ifstream ifs(index_file,std::ios::in);
        if(!ifs.is_open())
        {
            std::cerr<<"[FileIO - ReadConlineLaneID Error ] ~ "<<index_file<<std::endl;
            return false;
        }
        
        std::map<int,int> conn_slane_map,conn_elane_map;
        while(ifs.good()&&!ifs.eof())
        {
            int connid=-1,slaneid=-1,elaneid=-1;
            ifs>>connid>>slaneid>>elaneid;
            conn_slane_map[connid]=slaneid;
            conn_elane_map[connid]=elaneid;
        }
        
        for(MapBase::Connline &conn : vec)
        {
            conn.slaneid=conn_slane_map[conn.connid];
            conn.elaneid=conn_elane_map[conn.connid];
        }
        return true;
    }
    
    bool WriteConline(std::string dir,const ConnVec &vec)
    {
        std::string indexfile = dir + "/connindex.txt";
        std::ofstream outfile;
        outfile.open(indexfile, std::ios::out);
        if (!outfile.is_open())
        {
            std::cout << "[ FileIO - WriteConline Error ] ~ " << indexfile << std::endl;
            return false;
        }

        for (const Connline &cl : vec)
        {
            outfile << cl.connid << "\t";
            outfile << cl.sroadid << "\t";
            outfile << cl.eroadid << std::endl;
        }
        outfile.flush();
        outfile.close();
        return true;
    }

    bool ReadPath(std::string filename, PathVec &vec)
    {
        return false;
    }

    bool WritePath(std::string filename,const PathVec &vec)
    {
        std::ofstream outfile;
        outfile.open(filename, std::ios::out | std::ios::trunc);
        if (!outfile.is_open())
        {
            std::cout << "[ FileIO - WritePath Error ] ~ " << filename << std::endl;
            return false;
        }

        for (const Path &pa : vec)
        {
            for (const int roadid : pa.vroadid)
                outfile << roadid << "\t";
            outfile << std::endl;
        }
        outfile.flush();
        outfile.close();
        return true;
    }

	bool ReadMission(std::string filename,MissionVec &vec)
    {
        std::ifstream infile;
        infile.open(filename);
        if (!infile.is_open())
        {
            std::cout << "[ FileIO - ReadMission Error ] ~ " << filename << std::endl;
            return false;
        }

        while (infile.good() && !infile.eof())
        {
            Mission mission;
            infile >> mission.mid >> mission.roadid >> mission.x >> mission.y >> mission.yaw;
            mission.CalYawSinAndCos();
            vec.push_back(mission);
        }
        if (!vec.empty())
            vec.erase(vec.end() - 1);
        infile.close();
        return true;
    }
    bool ReadMission(std::string filename,MissionQueue &queue)
    {
        MissionVec vec;
        if(!ReadMission(filename,vec))
            return false;
        for(const Mission &mission : vec)
            queue.push(mission);
        return true;
    }

	bool WriteMission(std::string filename,const MissionVec &vec)
	{
		std::ofstream outfile;
		outfile.open(filename, std::ios::out | std::ios::trunc);
		if (!outfile.is_open())
		{
			std::cout << "[ FileIO - WriteMission Error ] ~ " << filename << std::endl;
			return false;
		}

		for (const Mission &data : vec)
		{
			outfile << std::setiosflags(std::ios::fixed);
			outfile << std::setprecision(0) << data.mid << "\t";
			outfile<<std::setprecision(0)<<data.roadid<<"\t";
			outfile << std::setprecision(5) << data.x << "\t";
			outfile << std::setprecision(5) << data.y << "\t";
			outfile<<std::setprecision(5)<<data.yaw<<std::endl;
		}
		outfile.flush();
		outfile.close();
		return true;
	}

    std::string GetNameFromTime()
    {
        time_t now = std::time(nullptr);
        std::tm *t = std::localtime(&now);
        std::string str;
        str += std::to_string(t->tm_year);
        str += "_";
        str += std::to_string(t->tm_mon);
        str += "_";
        str += std::to_string(t->tm_mday);
        str += "_";
        str += std::to_string(t->tm_hour);
        str += "_";
        str += std::to_string(t->tm_min);
        str += "_";
        str += std::to_string(t->tm_sec);
        return std::string(str);
    }

    //return directory exist or not
    bool DirExist(std::string path)
    {
#ifdef WIN32
        int accessflag=_access(path.c_str(),0);
        if(accessflag!=0)
            std::cout<<"[ FileIO - DirExist ] ~ "<<path<<std::endl;
        return accessflag==0;
#else
        int accessflag = access(path.c_str(), F_OK);
//        if (accessflag != 0)
//            std::cout << "[ FileIO - DirExist Error ] ~ " << path << std::endl;
        return accessflag == 0;
#endif
    }

    //create new directory
    bool DirBuild(std::string path)
    {
#ifdef WIN32
        int buildflag=_mkdir(path.c_str());
        if(buildflag!=0)
            std::cout<<"[ FileIO - DirBuild ] ~ "<<path<<std::endl;
        return buildflag==0;
#else

        int buildflag = mkdir(path.c_str(), S_IRWXU);
        if(buildflag!=0)
            std::cout<<"[ FileIO - DirBuild Error ] ~ "<<path<<std::endl;
        return buildflag == 0;
#endif
    }

    std::vector<std::string> GetFiles(const std::string &path)
    {
        std::vector<std::string> files;//存放文件名

#ifdef WIN32
        _finddata_t file;
        long lf;
        std::string p;
        //输入文件夹路径
        if ((lf=_findfirst(p.assign(path).append("\\*").c_str(), &file)) == -1)
        {
            std::cout<<path<<" not found!!!"<<std::endl;
        }
        else
        {
            while(_findnext(lf, &file) == 0)
            {
                if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
                    continue;
                files.push_back(path+"/"+file.name);
            }
        }
        _findclose(lf);

//        long hFile=0;
//        struct _finddata_t fileinfo;
//        std::string p;
//        if((hFile=_findfirst(p.assign(path).append("\\*").c_str(),&fileinfo))!=-1)
//        {
//            _findnext(hFile,&fileinfo);
//            do{
//                files.push_back(p.assign((fileinfo.name)));
//            }while(_findnext(hFile,&fileinfo)==0);
//            _findclose(hFile);
//        }

#else

        DIR *dir;
        struct dirent *ptr;
        char base[1000];

        if ((dir = opendir(path.c_str())) == nullptr)
        {
            perror("Open dir error...");
            exit(1);
        }

        while ((ptr = readdir(dir)) != nullptr)
        {
            //current dir OR parrent dir
            if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0)
                continue;
            //file
            if (ptr->d_type == 8)
                files.push_back(path+"/"+std::string(ptr->d_name));
            //link file
            else if (ptr->d_type == 10)
                continue;
            //dir
            else if (ptr->d_type == 4)
            {
                continue;
                //files.push_back(std::string(ptr->d_name));
            }
        }
        closedir(dir);
#endif

        //排序，按从小到大排序
        std::sort(files.begin(), files.end());
        return files;
    }

    std::string SplitDirectory(const std::string &path)
    {
        std::size_t pos = path.find_last_of("/\\");
        if(pos>=path.size())
            return std::string();
        return path.substr(0, pos);
    }

    std::string SplitNameWithExt(const std::string &path)
    {
        std::size_t pos = path.find_last_of("/\\");
        if (pos >= path.size())
            return path;
        return path.substr(pos + 1);
    }

    std::string SplitNameWithoutExt(const std::string &path)
    {
        std::string file=SplitNameWithExt(path);
        std::size_t pos2 = file.find_last_of('.');
        if(pos2>path.size())
            return file;
        return file.substr(0,pos2);
    }

    std::string SplitExtention(const std::string &path)
    {
        std::size_t pos = path.find_last_of('.');
        if(pos>=path.size())
            return std::string();
        return path.substr(pos + 1);
    }

}