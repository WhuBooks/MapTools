//
// Created by 98420 on 2018/8/31.
//

#ifndef MAPTOOLS_SPARSE_HPP
#define MAPTOOLS_SPARSE_HPP

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

namespace maplib
{
    namespace curve
    {
        template <typename T>
        void extractStructure(const std::vector<T> &input,int s_index,int e_index,std::vector<int> &index_vec,double sparse_th)
        {
            if(s_index+1==e_index)
                return;
            double max_dis=-1;
            int pos=s_index+1;
            for(int i=s_index+1;i<e_index;i++)
            {
                double tmp_dis=input[i].verticalDistance(input[s_index],input[e_index]);
                if(tmp_dis>max_dis)
                {
                    max_dis=tmp_dis;
                    pos=i;
                }
            }
            if(max_dis<sparse_th)
                return;
            
            index_vec.push_back(pos);
            extractStructure(input,s_index,pos,index_vec,sparse_th);
            extractStructure(input,pos,e_index,index_vec,sparse_th);
        }
        
        
        template <typename T>
        std::vector<T> douglasPeucker(const std::vector<T> &input,double sparse_threshold,double dense_threshold)
        {
            if(input.size()<=2)
                return input;
            
            /// extract geometry structure
            std::vector<int> sparse_index_vec={0,input.size()-1};
            extractStructure(input,0,input.size()-1,sparse_index_vec,sparse_threshold);
            std::sort(sparse_index_vec.begin(),sparse_index_vec.end());
            
            /// keep dense structure
            std::vector<int> dense_index_vec(sparse_index_vec);
            for(int i=1;i<sparse_index_vec.size();i++)
            {
                int s_index=sparse_index_vec[i-1];
                int e_index=sparse_index_vec[i];
                
                double sum_dis=input[s_index].distance(input[e_index]);
                if(sum_dis>dense_threshold)
                {
                    int num=std::ceil(sum_dis/dense_threshold);
                    int step=std::ceil((e_index-s_index)/num);
                    for(int j=s_index+step;j<e_index;j+=step)
                        dense_index_vec.push_back(j);
                }
            }
            std::sort(dense_index_vec.begin(),dense_index_vec.end());
            
            std::vector<T> result;
            for(const int &index : dense_index_vec)
                result.push_back(input[index]);
            return result;
        }
    

    
    }
}

#endif //MAPTOOLS_SPARSE_HPP
