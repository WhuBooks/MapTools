//
// Created by books on 2018/3/23.
//

#include "DouglasPeucker.h"

namespace MapBase{

    const double DouglasPeucker::GeoDisThreshold=1.0;
    const double DouglasPeucker::GeoAngleThreshold=M_PI/2;

    DouglasPeucker::DouglasPeucker()
    {
        m_SparseThreshold=0.1;
        m_DenseThreshold=1.0;
    }

    DouglasPeucker::DouglasPeucker(double sparse, double dense)
    {
        m_SparseThreshold = sparse;
        m_DenseThreshold = dense;
    }

    DouglasPeucker::~DouglasPeucker()
    {

    }


    bool Contain(const std::vector<int> &idlst,int id)
    {
        for(const int &val :idlst)
            if(val==id)
                return true;
        return false;
    }

    //caculate the distance from p3 to line (p1~p2)
    double CalDist(const GnssData &p1, const GnssData &p2,const GnssData &p3)
    {
        double A, B, C, m_dist;
        A = p2.y - p1.y;
        B = p1.x - p2.x;
        C = 0.0 - (p1.y*B + p1.x*A);
        m_dist = std::fabs((A*p3.x + B*p3.y + C) / std::sqrt(A*A + B*B));
        return m_dist;
    }

    GnssVec DouglasPeucker::Process(const GnssVec &input,bool check)
    {
//        std::cout << "[ Douglas Peucker ]Origin data : " << input.size() << std::endl;

        GnssVec lane;
        lane.assign(input.begin(),input.end());
        GnssVec::const_iterator iter;
        if(check)
        {
            std::vector<int> removelst;
            CheckMapBase(lane, removelst);

            GnssVec tmp;
            for (iter = lane.begin(); iter != lane.end();iter++)
            {
                if (Contain(removelst, iter->id))
                    continue;
                else
                    tmp.push_back(*iter);
            }
            lane.swap(tmp);
//            std::cout << "[ Douglas Peucker ]After MapBase check : " << input.size() << std::endl;
        }

        std::vector<int> idLst;
        Doglas_Puke(0, lane.size() - 1, idLst, lane);
        idLst.push_back(lane.front().id);
        idLst.push_back(lane.back().id);
//        std::cout << "[ Douglas Peucker ]Sparse result : " << idLst.size() << std::endl;

        std::sort(idLst.begin(), idLst.end());
        MapBase::GnssVec sparse_data;
        for (iter = lane.begin(); iter != lane.end(); iter++)
        {
            int id = iter->id;
            if (Contain(idLst, id))
                sparse_data.push_back(*iter);
        }

        Dense(idLst, lane);
//        std::cout << "[ Douglas Peucker ]Dense result : " << idLst.size() << std::endl;
        MapBase::GnssVec dense_data;
        for (iter = lane.begin(); iter != lane.end(); iter++)
        {
            int id = iter->id;
            if (Contain(idLst, id))
                dense_data.push_back(*iter);
        }
        return dense_data;

    }
    
    //check the MapBase to avoid break and jump
    void DouglasPeucker::CheckMapBase(const GnssVec &data,std::vector<int> &idlst)
    {
        GnssVecConstIter iter1,iter2,iter3;
        iter1=data.begin();
        iter2=data.begin()+1;
        iter3=data.begin()+2;
        while(iter1!=data.end()&&iter2!=data.end()&&iter3!=data.end())
        {
            double dis=CalDist(*iter1,*iter3,*iter2);

            double vec1_x=iter1->x-iter2->x;
            double vec1_y=iter1->y-iter2->y;
            double vec1_module=std::sqrt(vec1_x*vec1_x+vec1_y*vec1_y);

            double vec2_x=iter3->x-iter2->x;
            double vec2_y=iter3->y-iter2->y;
            double vec2_module=std::sqrt(vec2_x*vec2_x+vec2_y*vec2_y);

            double vec_pro=vec1_x*vec2_x+vec1_y*vec2_y;
            double cos=vec_pro/(2*vec1_module*vec2_module);

            double angle=std::acos(cos);

            if(dis>GeoDisThreshold || angle<GeoAngleThreshold)
            {
                idlst.push_back(iter2->id);
                iter2++;
                iter3++;
            }
            else
            {
                iter1++;
                iter2++;
                iter3++;
            }
        }
    }

    //Doglas puke algorithm
    void DouglasPeucker::Doglas_Puke(int start, int end, std::vector<int> &idlst,const GnssVec &data)
    {
        if (start + 1 == end)
        {
            return;
        }
        double maxDis = std::numeric_limits<double>::min();
        int pos = -1;
        for (int i = start+1; i < end; i++)
        {
            double dis = CalDist(data[start], data[end], data[i]);
            if (dis > maxDis)
            {
                maxDis = dis;
                pos = i;
            }
        }
        if (maxDis < m_SparseThreshold||pos==-1)
        {
            return;
        }
        else
        {
            idlst.push_back(data[pos].id);
            Doglas_Puke(start, pos, idlst, data);
            Doglas_Puke(pos, end, idlst, data);
        }
    }

    //keep dense
    void DouglasPeucker::Dense(std::vector<int> &idlst, const GnssVec &data)
    {
        double disSum = 0;
        for (int i = 0; i < data.size()-1; i++)
        {
            MapBase::GnssData p1 = data[i];
            MapBase::GnssData p2 = data[i + 1];
            if(Contain(idlst,p2.id))
            {
                disSum=0;
                continue;
            }

            disSum += p1.Distance(p2);
            if (disSum > m_DenseThreshold)
            {
                idlst.push_back(p2.id);
                disSum = 0;
            }
        }
    }


}