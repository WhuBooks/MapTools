//
// Created by books on 18-8-9.
//

#include <iostream>
#include <Coordinate.h>
#include <RoadModel.h>
#include <DouglasPeucker.h>
#include <map_header.h>
#include <cassert>

//int main()
//{
//    std::string dir="HT_Add_Sparse/";
//    MapBase::GnssVec gnssVec;
//    for(int i=1;i<=7;i++)
//    {
//        std::string file=dir+std::to_string(i)+".txt";
//
//        MapBase::GnssVec tmp_vec;
//        MapBase::ReadGnss(file,tmp_vec);
//
//        gnssVec.insert(gnssVec.end(),tmp_vec.begin(),tmp_vec.end());
//
//    }
//
//    std::sort(gnssVec.begin(),gnssVec.end(),[](const MapBase::GnssData &gd1,const MapBase::GnssData &gd2){
//       return gd1.id<gd2.id;
//    });
//
//    std::function<MapBase::GnssVec(int,int)> extract_func=[&](int s_id,int e_id)->MapBase::GnssVec{
//        int s_index=-1,e_index=-1;
//        for(int i=0;i<gnssVec.size();i++)
//        {
//            if(gnssVec[i].id==s_id)
//                s_index=i;
//
//            if(gnssVec[i].id==e_id)
//                e_index=i;
//        }
//
//        assert(e_index>s_index);
//
//        MapBase::GnssVec result(gnssVec.begin()+s_index,gnssVec.begin()+e_index);
//        return result;
//    };
//
//    std::string lane_id_file=dir+"lane_id.txt";
//    std::ifstream ifs_lane_id(lane_id_file,std::ios::in);
//    std::vector<std::tuple<int,int>> lane_ids;
//    while(ifs_lane_id.good()&&!ifs_lane_id.eof())
//    {
//        int s_id,e_id;
//        ifs_lane_id>>s_id>>e_id;
//        lane_ids.push_back(std::make_tuple(s_id,e_id));
//    }
//    lane_ids.erase(lane_ids.end()-1);
//
//    int lane_id=44;
//    for(int i=0;i<lane_ids.size();i++)
//    {
//        MapBase::GnssVec tmp_lane=extract_func(std::get<0>(lane_ids[i]),std::get<1>(lane_ids[i]));
//        std::cout<<"Lane ~ "<<lane_id<<"\t Size ~ "<<tmp_lane.size()<<std::endl;
//        MapBase::WriteGnss(dir+"Lane/"+std::to_string(lane_id++)+".txt",tmp_lane);
//    }
//
//    std::string conn_id_file=dir+"conn_id.txt";
//    std::ifstream ifs_conn_id(conn_id_file,std::ios::in);
//    std::vector<std::tuple<int,int>> conn_ids;
//    while(ifs_conn_id.good()&&!ifs_conn_id.eof())
//    {
//        int s_id,e_id;
//        ifs_conn_id>>s_id>>e_id;
//        conn_ids.push_back(std::make_tuple(s_id,e_id));
//    }
//    conn_ids.erase(conn_ids.end()-1);
//
//    int conn_id=38;
//    for(int i=0;i<conn_ids.size();i++)
//    {
//        MapBase::GnssVec tmp_lane=extract_func(std::get<0>(conn_ids[i]),std::get<1>(conn_ids[i]));
//        std::cout<<"Conn ~ "<<conn_id<<"\t Size ~ "<<tmp_lane.size()<<std::endl;
//        MapBase::WriteGnss(dir+"Conn/"+std::to_string(conn_id++)+".txt",tmp_lane);
//    }
//
//}


//int main()
//{
//    std::string lane_dir="Lane";
//    std::vector<std::string> lane_file_vec=MapBase::GetFiles(lane_dir);
//    std::vector<MapBase::GnssVec> lane_vec;
//
//    for(const std::string filename : lane_file_vec)
//    {
//        MapBase::GnssVec lane;
//        MapBase::ReadGnss(filename,lane);
//        lane_vec.push_back(lane);
//    }
//
//    std::string filename="Sparse.txt";
//    MapBase::GnssVec gnssVec;
//    MapBase::ReadGnss(filename,gnssVec);
//
//    std::sort(lane_vec.begin(),lane_vec.end(),[](const MapBase::GnssVec &v1,const MapBase::GnssVec &v2){
//       return v1.front().id<v2.front().id;
//    });
//
//    std::string dir=MapBase::GetNameFromTime();
//    MapBase::DirBuild(dir);
//
//    int conn_id=0;
//    std::vector<MapBase::GnssVec> conn_vec;
//    for(int i=0;i<lane_vec.size()-1;i++)
//    {
//        int s_id=lane_vec[i].back().id+1;
//        int e_id=lane_vec[i+1].front().id-1;
//
//        MapBase::GnssVec tmp_conn;
//        for(int j=0;j<gnssVec.size();j++)
//        {
//            if(gnssVec[j].id>s_id && gnssVec[j].id<e_id)
//                tmp_conn.push_back(gnssVec[j]);
//        }
//        MapBase::WriteGnss(dir+"/"+std::to_string(conn_id++)+".txt",tmp_conn);
//        std::cout<<"S_ID ~ "<<s_id<<"\tE_ID ~ "<<e_id<<"\tNum ~ "<<tmp_conn.size()<<std::endl;
//    }
//
//}


//struct Mark
//{
//
//    double x;
//    double y;
//};
//
//bool isStart(const Mark &mark,const MapBase::GnssData &gd)
//{
//    double dis=std::sqrt((mark.x-gd.x)*(mark.x-gd.x)+(mark.y-gd.y)*(mark.y-gd.y));
//
//    double dx=mark.x-gd.x;
//    double dy=mark.y-gd.y;
//
//    dx/=dis;
//    dy/=dis;
//
//    double prod=dx*std::sin(MapBase::Degree2Grad(gd.yaw))+dy*std::cos(MapBase::Degree2Grad(gd.yaw));
////    return dis<1.0;
//    return prod>0 && dis<1.0;
//}
//
//bool isEnd(const Mark &mark,const MapBase::GnssData &gd)
//{
//    double dis=std::sqrt((mark.x-gd.x)*(mark.x-gd.x)+(mark.y-gd.y)*(mark.y-gd.y));
//
//    double dx=gd.x-mark.x;
//    double dy=gd.y-mark.y;
//
//    dx/=dis;
//    dy/=dis;
//
//    double prod=dx*std::sin(MapBase::Degree2Grad(gd.yaw))+dy*std::cos(MapBase::Degree2Grad(gd.yaw));
//
//    return prod>0 && dis<1.0;
//}
//
//int main()
//{
//    std::string filename="Sparse.txt";
//    MapBase::GnssVec gnssVec;
//    MapBase::ReadGnss(filename,gnssVec);
//
//    std::vector<Mark> mark_vec;
//    std::string mark_file="Mark.txt";
//    std::ifstream ifs(mark_file);
//    std::map<int,Mark> mark_map;
//    while(ifs.good()&&!ifs.eof())
//    {
//        int id;
//        Mark mark;
//        ifs>>id>>mark.x>>mark.y;
//        mark_map[id]=mark;
//        mark_vec.push_back(mark);
//    }
//
//    std::vector<std::tuple<int,int>> ids_vec;
//    std::string id_file="ID.txt";
//    std::ifstream ifs_id(id_file);
//    while(ifs_id.good()&&!ifs_id.eof())
//    {
//        int s_id,e_id;
//        ifs_id>>s_id>>e_id;
//        ids_vec.push_back(std::make_tuple(s_id,e_id));
//    }
//    ids_vec.erase(ids_vec.end()-1);
//
//    std::string dir=MapBase::GetNameFromTime();
//    MapBase::DirBuild(dir);
//
//    int laneid=0;
//    for(int i=0;i<ids_vec.size();i++)
//    {
//        int s_id=std::get<0>(ids_vec[i]);
//        int e_id=std::get<1>(ids_vec[i]);
//
//        Mark s_mark=mark_map[s_id];
//        Mark e_mark=mark_map[e_id];
//
//        std::vector<MapBase::GnssVec> lane_vec;
//
//        int s_index=-1,e_index=-1;
//        for(int j=0;j<gnssVec.size();j++)
//        {
//            if(s_index==-1)
//            {
//                if(isStart(s_mark,gnssVec[j]))
//                {
//                    s_index=j;
//                }
//            }
//
//            if(s_index!=-1&&e_index==-1)
//            {
//                if(isEnd(e_mark,gnssVec[j]))
//                {
//                    e_index=j;
//                    MapBase::GnssVec tmp_lane;
//                    tmp_lane.assign(gnssVec.begin()+s_index,gnssVec.begin()+e_index);
//                    lane_vec.push_back(tmp_lane);
//                    s_index=-1;
//                    e_index=-1;
//                }
//            }
//        }
//        std::cout<<"S_ID ~ "<<s_id<<"\tE_ID ~ "<<e_id<<"\tNum ~ "<<lane_vec.size()<<std::endl;
//
//        for(const MapBase::GnssVec &vec : lane_vec)
//        {
//            std::string lane_file=dir+"/"+std::to_string(laneid++)+".txt";
//            MapBase::WriteGnss(lane_file,vec);
//        }
//
//    }
//
//
//}




int main()
{

    std::string dir="Record";
    std::vector<std::string> file_vec=MapBase::GetFiles(dir);
    
    file_vec.clear();
    file_vec.push_back("Final_Add.txt");
    
    int id=43363;
    std::string sparse_dir=MapBase::GetNameFromTime();
    MapBase::DirBuild(sparse_dir);

    for(const std::string &file : file_vec)
    {
        std::ifstream ifs(file,std::ios::in);

        MapBase::GnssVec gnssVec;
        while(!ifs.eof()&&ifs.good())
        {
            int time,state;
            double lat,lon,roll,pitch,yaw,vx,vy,vz,ax,ay,az,wx,wy,wz;

            ifs>>time>>lon>>lat>>roll>>pitch>>yaw>>vx>>vy>>vz>>ax>>ay>>az>>wx>>wy>>wz>>state;

            double x,y;
            MapBase::Coordinate::GetInstance().Wgs84ToMercator(lon,lat,x,y);

            MapBase::GnssData gnssData;
            gnssData.id=id++;
            gnssData.timestamp=time;
            gnssData.lon=lon;
            gnssData.lat=lat;
            gnssData.x=x;
            gnssData.y=y;
            gnssData.hgt=0.0;
            gnssData.roll=MapBase::Degree(roll);
            gnssData.pitch=MapBase::Degree(pitch);
            gnssData.yaw=MapBase::Degree(yaw);
            gnssData.north=0;
            gnssData.east=0;
            gnssData.up=0;
            gnssVec.push_back(gnssData);
        }

        MapBase::DouglasPeucker douglasPeucker(0.01,1.0);
        MapBase::GnssVec process=douglasPeucker.Process(gnssVec);

        std::string out_file=sparse_dir+"/"+MapBase::SplitNameWithExt(file);
        MapBase::WriteGnss(out_file,process);
    }

    return 1;
}