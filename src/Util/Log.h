//
// Created by 98420 on 2018/8/31.
//

#ifndef MAPTOOLS_LOG_H
#define MAPTOOLS_LOG_H

#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <chrono>
#include <ctime>

namespace util
{
    class Log
    {
    public:
        Log(int _interval=1);
    
        ~Log() = default;
    
        void ends(const std::string &str)
        {
            m_cur_word=m_cur_word+str+" ";
        };
    
        void endl(const std::string &str)
        {
            m_cur_word=m_cur_word+str+"\n";
        };
    
        void update();
    
        void start();
    
        void stop();

    private:
        bool m_update_flag;
        std::mutex m_update_mutex;
        std::thread m_update_thread;
        int m_interval;
        std::string m_cur_word;
        std::string m_word;
        std::string m_time_stamp;
    };
}



#endif //MAPTOOLS_LOG_H
